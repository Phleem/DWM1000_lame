#include "mbed.h"
#include <string>
#include <inttypes.h>
extern "C" {
#include "libdw1000.h"

}

DigitalOut heartbeat(LED2);
DigitalOut led(LED1);
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut cs(SPI_CS);
DigitalIn sIRQ(PA_0);
DigitalOut sReset(PA_1);
Serial pc(PA_9, PA_10, 9600);

    //the packed attribute makes sure the types only use their respective size in memory (8 bit for uint8_t), otherwise they would always use 32 bit
    //IT IS A GCC SPECIFIC DIRECTIVE
typedef struct __attribute__((packed, aligned(1))) DataFrame {
        uint8_t source;
        uint8_t destination;
        uint8_t type;
		uint8_t data[15];
}DFrame;

enum FrameType{
        PING=1,
        ANCHOR_RESPONSE=2,
        BEACON_RESPONSE=3,
        TRANSFER_FRAME=4,
        DISTANCES_FRAME=5
};



static void spiWrite(dwDevice_t* dev, const void* header, size_t headerLength,
                                      const void* data, size_t dataLength) {

	cs = 0;
	uint8_t* headerP = (uint8_t*) header;
	uint8_t* dataP = (uint8_t*) data;

	for(size_t i = 0; i<headerLength; ++i) {
		spi.write(headerP[i]);
	}
	for(size_t i = 0; i<dataLength; ++i) {
		spi.write(dataP[i]);
	}

	cs = 1;
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength) {
	cs = 0;
	uint8_t* headerP = (uint8_t*) header;
	uint8_t* dataP = (uint8_t*) data;

	for(size_t i = 0; i<headerLength; ++i) {
		spi.write(headerP[i]);
	}
	for(size_t i = 0; i<dataLength; ++i) {
		dataP[i] = spi.write(0);
	}

	cs = 1;
}


static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
	if (speed == dwSpiSpeedLow)
		spi.frequency(4*1000*1000);

	if (speed == dwSpiSpeedHigh)
		spi.frequency(20*1000*1000);
}

static void reset(dwDevice_t* dev)
{
	sReset = 0;
	wait(0.5);
	sReset = 1;
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
	wait(delay * 0.001f);
}

static dwOps_t ops = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
  .reset = reset
};

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
	unsigned int idx;
	unsigned int shift;

	idx = bit / 8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	uint8_t* targetByte = &data[idx];
	shift = bit % 8;
	if(val) {
		*targetByte |= (1<<shift);
	} else {
	  *targetByte &= ~(1<<shift);
	}
}

void dwStartDelayedTransmit(dwDevice_t* dev) {
	dwWriteTransmitFrameControlRegister(dev);
	setBit(dev->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dev->frameCheck);
	setBit(dev->sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
	setBit(dev->sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, true);
	dwSpiWrite(dev, SYS_CTRL, NO_SUB, dev->sysctrl, LEN_SYS_CTRL);
	if(dev->permanentReceive) {
		memset(dev->sysctrl, 0, LEN_SYS_CTRL);
		dev->deviceMode = RX_MODE;
		dwStartReceive(dev);
	} else if (dev->wait4resp) {
    dev->deviceMode = RX_MODE;
  } else {
		dev->deviceMode = IDLE_MODE;
	}
}

dwTime_t dwSetDelayBuffer(dwDevice_t* dev, const dwTime_t* delay, dwTime_t* futureTime) {
	uint8_t delayBytes[5];
	//dwTime_t futureTime = {.full = 0};
	dwGetSystemTimestamp(dev, futureTime);
	futureTime->full += delay->full;
  memcpy(delayBytes, futureTime->raw, sizeof(futureTime->raw));
	delayBytes[0] = 0;
	delayBytes[1] &= 0xFE;
	dwSpiWrite(dev, DX_TIME, NO_SUB, delayBytes, LEN_DX_TIME);
	// adjust expected time with configured antenna delay
  memcpy(futureTime->raw, delayBytes, sizeof(futureTime->raw));
	futureTime->full += dev->antennaDelay.full;
	return *futureTime;
}

static const double tsfreq = 499.2e6 * 128; // Timestamp counter frequency
static const double C = 299792458.0; // Speed of light

bool isBeacon = false;
volatile bool hasSendRanging = false;
volatile bool sendRanging = false;
dwDevice_t dwm_device;
dwDevice_t* dwm = &dwm_device;
dwTime_t tStartRound1;
dwTime_t tEndRound1;
dwTime_t tStartReply1;
dwTime_t tEndReply1;
dwTime_t tStartRound2;
dwTime_t tEndRound2;
dwTime_t tStartReply2;
dwTime_t tEndReply2;
dwTime_t tDelay;
uint64_t tRound1;
uint64_t tReply1;
uint64_t tRound2;
uint64_t tReply2;
long double tPropTick;

DFrame frame;

Ticker ranger;


void enableClocks(){
	uint8_t enable = 0b010100;
	dwSpiWrite(dwm, 0x36, 0, (void*) enable, sizeof(enable));
}

void calculateDeltaTime(dwTime_t* startTime, dwTime_t* endTime, uint64_t* result){

	uint64_t start = (startTime->full);
	uint64_t end = (endTime->full);

	if(end > start){
		*result = (end - start);
	}
	else{
		*result = (end + (1099511628000-start));
	}
}

void calculatePropagationFormula(const uint64_t& tRound1, const uint64_t& tReply1, const uint64_t& tRound2, const uint64_t& tReply2, long double& tPropTick){

	tPropTick = (long double)((tRound1 * tRound2) - (tReply1 * tReply2)) / (tRound1 + tReply1 + tRound2 + tReply2);
}

void setSendRangingFlag(){
	sendRanging = !sendRanging;
	hasSendRanging = false;
}

void sendBeaconResponse(dwDevice_t* dev) {
	frame.type = BEACON_RESPONSE;
	memcpy(frame.data, tStartRound1.raw, 5);
	memcpy((frame.data+5), tEndRound1.raw, 5);
	dwSetDelayBuffer(dev, &tDelay, &tEndReply2);
	memcpy((frame.data+10), tEndReply2.raw, 5);
	dwSetData(dev, (uint8_t*)&frame, sizeof(frame));
	dwNewTransmit(dev);
	dwStartTransmit(dev);
	//dwStartDelayedTransmit(dev);	
}

void sendRangingPoll(dwDevice_t* dev) {
	dwNewTransmit(dev);
	frame.type = PING;
	dwSetData(dev, (uint8_t*) &frame, sizeof(frame));
	dwStartTransmit(dev);
}

void sendAnchorResponse(dwDevice_t* dev){
	dwNewTransmit(dev);
	frame.type = ANCHOR_RESPONSE;
	dwSetData(dev, (uint8_t*) &frame, sizeof(frame));
	dwStartTransmit(dev);
}

void answerPing(dwDevice_t *dev){
	dwGetReceiveTimestamp(dev, &tStartReply1);
	sendAnchorResponse(dev);
}

void answerAnchorResponse(dwDevice_t *dev){
	ranger.detach();
	dwGetReceiveTimestamp(dev, &tEndRound1);
	sendBeaconResponse(dev);
	ranger.attach(&setSendRangingFlag, 5);
}

void calculatePropagation(dwDevice_t *dev){
	dwGetReceiveTimestamp(dev, &tEndRound2);
	dwGetData(dev, (uint8_t*)&frame, sizeof(frame));

	memcpy(tStartRound1.raw, frame.data, 5);
	memcpy(tEndRound1.raw, (frame.data+5), 5);
	memcpy(tEndReply2.raw, (frame.data+10), 5);

	calculateDeltaTime(&tStartRound1, &tEndRound1, &tRound1);
	calculateDeltaTime(&tStartReply1, &tEndReply1, &tReply1);
	calculateDeltaTime(&tEndRound1, &tEndReply2, &tReply2);
	calculateDeltaTime(&tEndReply1, &tEndRound2, &tRound2);
	
	//calculatePropagationFormulaInNS(tRound1, tReply1, tRound2, tReply2, tPropTick);
	calculatePropagationFormula(tRound1, tReply1, tRound2, tReply2, tPropTick);

	long double tPropTime = tPropTick / tsfreq; //in seconds
	long double distance = tPropTime * C; //~0.3 m per nanosecond

	pc.printf("%Lf is Propagation in ns\n",tPropTime);
	pc.printf("%Lf is Distance\n", distance);
	pc.printf("%"PRIu64"\n",tRound1);
	pc.printf("%"PRIu64"\n",tReply1);
	pc.printf("%"PRIu64"\n",tReply2);
	pc.printf("%"PRIu64"\n",tRound2);
	/*
	pc.printf("%"PRIu64"\n",tStartRound1);
	pc.printf("%"PRIu64"\n",tEndRound1);
	pc.printf("%"PRIu64"\n",tStartReply1);
    pc.printf("%"PRIu64"\n",tEndReply1);
	pc.printf("%"PRIu64"\n",tEndRound2);
	pc.printf("%"PRIu64"\n",tEndReply2);
	*/
	char seperator = 'X';
	pc.printf("%c\n",seperator);
	
}

void txcallback(dwDevice_t *dev){
	if(isBeacon == true){
		switch(frame.type){
			case PING: dwGetTransmitTimestamp(dev, &tStartRound1); break;
		}
	}
	else{
		switch(frame.type){
			case ANCHOR_RESPONSE: dwGetTransmitTimestamp(dev, &tEndReply1); break;
		}
	}
}

void rxcallback(dwDevice_t *dev){
  if(isBeacon == true){
	  dwGetData(dev, (uint8_t*)&frame, sizeof(frame));
	  switch(frame.type){
		  case ANCHOR_RESPONSE: answerAnchorResponse(dev); break;
	  }
  }
  else{
	  dwGetData(dev, (uint8_t*)&frame, sizeof(frame));
	  switch(frame.type){
		  case PING: answerPing(dev); break;
		  case BEACON_RESPONSE: calculatePropagation(dev); break;
	  }
	
  }

}

void rangingBeacon(){
	while(true){
		if(sIRQ){
			dwHandleInterrupt(dwm);
		}
		if(sendRanging && hasSendRanging == false){
			sendRangingPoll(dwm);
			hasSendRanging = true;
		}	
	} 
  
   
}

void rangingAnchor(){
	dwNewReceive(dwm);
	dwStartReceive(dwm);
	while(true){
		if(sIRQ){//poll IRQ-Pin
		dwHandleInterrupt(dwm);
		}
		
	}
		
}


// main() runs in its own thread in the OS
int main() {

	reset(dwm);

	heartbeat = 1;
	sReset = 1;
	cs = 1;
	dwInit(dwm, &ops);       // Init libdw
	uint8_t result = dwConfigure(dwm); // Configure the dw1000 chip
	if (result == 0) {
		dwEnableAllLeds(dwm);
	}

	dwTime_t delay = {.full = 0};
	dwSetAntenaDelay(dwm, delay);

	dwAttachSentHandler(dwm, txcallback);
	dwAttachReceivedHandler(dwm, rxcallback);

	dwNewConfiguration(dwm);
	dwSetDefaults(dwm);
	dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(dwm, CHANNEL_2);
	dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

	dwCommitConfiguration(dwm);

	dwReceivePermanently(dwm, true);

	tStartRound1.full = 0;
	tEndRound1.full = 0;
	tStartReply1.full = 0;
	tEndReply1.full = 0;
	tStartRound2.full = 0;
	tEndRound2.full = 0;
	tStartReply2.full = 0;
	tEndReply2.full = 0;

	tDelay.full = 0;
	tDelay.full = 74756096;//63897600*5; //5msec 
	frame.type = 0;

  if(isBeacon == true){
	dwInterruptOnReceived(dwm, true);
	dwInterruptOnSent(dwm, true);
	ranger.attach(&setSendRangingFlag, 5); //Interrupt on receiving a good Frame is triggered
    rangingBeacon();
  }
  else{
	dwInterruptOnReceived(dwm, true); //Interrupt on receiving a good Frame is triggered
	dwInterruptOnSent(dwm, true); //Interrupt on sending a Frame
    rangingAnchor();
  }
}
 //TODO: cast to double after making cast to double for tRound1 etc -> wonky Behavior