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

const char* txPacket = "foobar";
uint8_t data[10];
bool isBeacon = true;
volatile bool hasSendRanging = false;
volatile bool sendRanging = false;
dwDevice_t dwm_device;
dwDevice_t* dwm = &dwm_device;
dwTime_t tStartRound;
dwTime_t tEndRound;
dwTime_t tStartReply;
dwTime_t tDelay;
dwTime_t tEndReply;
long long int tRound;
long long int tReply;
double tProp;

Ticker ranger;

enum FrameType{
        PING=1,
        ANCHOR_RESPONSE,
        BEACON_RESPONSE,
        TRANSFER_FRAME,
        DISTANCES_FRAME
};
 
    //the packed attribute makes sure the types only use their respective size in memory (8 bit for uint8_t), otherwise they would always use 32 bit
    //IT IS A GCC SPECIFIC DIRECTIVE
struct __attribute__((packed, aligned(1))) Frame {
        uint8_t source;
        uint8_t destination;
        uint8_t type;
		uint8_t data[10];
};

void enableClocks(){
	uint8_t enable = 0b010100;
	dwSpiWrite(dwm, 0x36, 0, (void*) enable, sizeof(enable));
}

void setSendRangingFlag(){
	sendRanging = !sendRanging;
	hasSendRanging = false;
}

void calculateSSTimeOfFlight(long long int* timeRound, long long int* timeReply, double* tProp){
    *tProp = (float) (0.5f * (*timeRound - *timeReply));
}
void calculateDeltaTime(uint64_t* timeStart, uint64_t* timeEnd, long long int* timeDelta){
    *timeDelta = llabs((long long int)(*timeEnd) - (long long int)(*timeStart));
}

void calculatePropagation(dwDevice_t *dev){
	ranger.detach();
	dwGetReceiveTimestamp(dev, &tEndRound);
	dwGetData(dev, data, sizeof(data));

	memcpy(tStartReply.raw, data, 5);
	memcpy(tEndReply.raw, (data+5), 5);

	calculateDeltaTime(&(tStartReply.full), &(tEndReply.full), &tReply);
	calculateDeltaTime(&(tStartRound.full), &(tEndRound.full), &tRound);

	calculateSSTimeOfFlight(&tRound, &tReply, &tProp);

	tProp = tProp / 63.8976; //to ns
	tRound = tRound / 64;
	tReply = tReply / 64;
	double distance = tProp * 0.299792458; //~0.3 m per nanosecond

	pc.printf("%d is Propagation in ns\n",tProp);
	pc.printf("%d is Distance\n", distance);
	pc.printf("%lld\n", tRound);
	pc.printf("%lld\n", tReply);
    //pc.printf("%"PRIu64"\n",tStartRound.full);
	//pc.printf("%"PRIu64"\n",tEndRound.full);
	//pc.printf("%"PRIu64"\n",tStartReply.full);
	//pc.printf("%"PRIu64"\n",tEndReply.full);
	char seperator = 'X';
	pc.printf("%c\n",seperator);
	ranger.attach(&setSendRangingFlag, 5);
}

void send_dummy(dwDevice_t* dev) {
	dwNewTransmit(dev);
	//dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)txPacket, strlen(txPacket));
	dwStartTransmit(dev);
}

void sendReply(dwDevice_t* dev) {

	
	dwGetReceiveTimestamp(dev, &tStartReply);
	memcpy(data, tStartReply.raw, 5);
	dwSetDelayBuffer(dev, &tDelay, &tEndReply);
	memcpy((data+5), tEndReply.raw, 5);
	dwSetData(dev, data, sizeof(data));
	dwNewTransmit(dev);
	//dwStartDelayedTransmit(dev);

	//dwTime_t timeStart = {.full = 0};
	//dwGetSystemTimestamp(dev, &timeStart);
	
	

	//dwTime_t timeEnd = {.full = 0};
	//dwGetSystemTimestamp(dev, &timeEnd);
	dwStartTransmit(dev);
	
		
}

void send_ranging(dwDevice_t* dev) {
	dwNewTransmit(dev);
	dwStartTransmit(dev);
}

void txcallback(dwDevice_t *dev){
	if(isBeacon == false){
		dwGetTransmitTimestamp(dev, &tStartRound);
	}
}

void rxcallback(dwDevice_t *dev){
  if(isBeacon == true){
    sendReply(dev);
  }
  else{
	calculatePropagation(dev);
  }

}

void rangingBeacon(){
 	dwNewReceive(dwm);
  	dwStartReceive(dwm);
	while(true){
		if(sIRQ){
			dwHandleInterrupt(dwm);
		}
	} 
  
   
}

void rangingAnchor(){
	while(true){
		if(sIRQ){//poll IRQ-Pin
		dwHandleInterrupt(dwm);
		}
		if(sendRanging && hasSendRanging == false){
			send_ranging(dwm);
			hasSendRanging = true;
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

	tStartRound.full = 0;
	tEndRound.full = 0;
	tStartReply.full = 0;
	tEndReply.full = 0;
	tDelay.full = 0;
	tDelay.full = 74756096;//63897600*5; //5msec 
	

  if(isBeacon == true){
	dwInterruptOnReceived(dwm, true); //Interrupt on receiving a good Frame is triggered
    rangingBeacon();
  }
  else{
	dwInterruptOnReceived(dwm, true); //Interrupt on receiving a good Frame is triggered
	dwInterruptOnSent(dwm, true); //Interrupt on sending a Frame
	ranger.attach(&setSendRangingFlag, 5);
    rangingAnchor();
  }
}

//sending times successfully
//TODO: send with AntennaDelay
//TODO: calculate Distances