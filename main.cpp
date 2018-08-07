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
DigitalInOut sReset(PA_1);
Serial pc(PA_9, PA_10, 9600);
    //the packed attribute makes sure the types only use their respective size in memory (8 bit for uint8_t), otherwise they would always use 32 bit
    //IT IS A GCC SPECIFIC DIRECTIVE
typedef struct __attribute__((packed, aligned(1))) DataFrame {
		uint16_t frameControl  = 0b1000100001000000;
		uint8_t sequenceNumber = 0b00000000;
		uint8_t destPan[2];
		uint16_t destAddress;
        uint8_t srcAddress[2];
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


//TODO create correct Mac-header to be send at the beginning of each frame
//TODO FrameControlField 0b0000001000010001
//TODO SequenceNumber 0b00000000
//TODO DestPan&Address dwm->network&Address[0]+dwm->network&Address[1]+destAddress
//TODO SourceAddress dwm->network&Address[2]+dwm->network&Address[3]

//TODO frame initialize 


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
	/*
	sReset = 0;
	wait(0.5);
	sReset = 1;
	*/
	sReset.output();
  	sReset = 0;
	wait(0.1);
	sReset.input();
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
static const double speedOfLight = 299792458.0; // Speed of light in m/s
static const uint8_t noDataFramesize = 10;


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

//set before compiling
DFrame frame;
volatile uint16_t DestinationAddress = 0x01; //variable DestinationAddress to route from Anchor 
const int srcAddress = 0x01; //0 for Anchor, increment for Beacons
const bool isBeacon = true;

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

void sendTransferFrame(dwDevice_t* dev, uint16_t destAddress) {
	frame.type = TRANSFER_FRAME;
	frame.destAddress = destAddress;
	memcpy(frame.data, tStartReply1.raw, 5);
	memcpy((frame.data+5), tEndReply1.raw, 5);
	memcpy((frame.data+10), tEndRound2.raw, 5);
	dwSetData(dev, (uint8_t*)&frame, sizeof(frame));
	dwNewTransmit(dev);
	dwStartTransmit(dev);	
}

void sendRangingPoll(dwDevice_t* dev, uint16_t destAddress) {
	dwNewTransmit(dev);
	frame.type = PING;
	frame.destAddress = destAddress;
	dwSetData(dev, (uint8_t*) &frame, noDataFramesize);
	dwStartTransmit(dev);
}

void sendBeaconResponse(dwDevice_t* dev, uint16_t destAddress){
	dwNewTransmit(dev);
	frame.type = BEACON_RESPONSE;
	frame.destAddress = destAddress;
	dwSetData(dev, (uint8_t*) &frame, noDataFramesize);
	dwStartTransmit(dev);
}

void sendAnchorResponse(dwDevice_t* dev, uint16_t destAddress){
	dwNewTransmit(dev);
	frame.type = ANCHOR_RESPONSE;
	frame.destAddress = destAddress;
	dwSetData(dev, (uint8_t*) &frame, noDataFramesize);
	dwStartTransmit(dev);
}

void calculatePropagation(dwDevice_t *dev){

	ranger.detach();

	dwGetData(dev, (uint8_t*)&frame, sizeof(frame));

	memcpy(tStartReply1.raw, frame.data, 5);
	memcpy(tEndReply1.raw, (frame.data+5), 5);
	memcpy(tEndRound2.raw, (frame.data+10), 5);

	calculateDeltaTime(&tStartRound1, &tStartReply2, &tRound1);
	calculateDeltaTime(&tStartReply1, &tEndReply1, &tReply1);
	calculateDeltaTime(&tStartReply2, &tEndReply2, &tReply2);
	calculateDeltaTime(&tEndReply1, &tEndRound2, &tRound2);
	
	calculatePropagationFormula(tRound1, tReply1, tRound2, tReply2, tPropTick);

	long double tPropTime = (tPropTick / tsfreq); //in seconds

	long double distance = (tPropTime * speedOfLight);// - 154.03; //~0.3 m per nanosecond; offset 154.03 calculated at ~1 cm distance
 
	pc.printf("%0.12Lf,%Lf;\r\n",tPropTime, distance);
	//pc.printf(" is Distance in m\r\n", distance);
	/*
	pc.printf("%"PRIu64"\r\n",tRound1);
	pc.printf("%"PRIu64"\r\n",tReply1);
	pc.printf("%"PRIu64"\r\n",tReply2);
	pc.printf("%"PRIu64"\r\n",tRound2);
	
	pc.printf("%"PRIu64"\n",tStartRound1);
	pc.printf("%"PRIu64"\n",tEndRound1);
	pc.printf("%"PRIu64"\n",tStartReply1);
    pc.printf("%"PRIu64"\n",tEndReply1);
	pc.printf("%"PRIu64"\n",tEndRound2);
	pc.printf("%"PRIu64"\n",tEndReply2);
	*/
	tStartRound1.full = 0;
	tEndRound1.full = 0;
	tStartReply1.full = 0;
	tEndReply1.full = 0;
	tStartRound2.full = 0;
	tEndRound2.full = 0;
	tStartReply2.full = 0;
	tEndReply2.full = 0;

	ranger.attach(&setSendRangingFlag, 2);
}

void txcallback(dwDevice_t *dev){
	if(isBeacon == false){
		switch(frame.type){
			case PING: 
				dwGetTransmitTimestamp(dev, &tStartRound1); break;
			case ANCHOR_RESPONSE:
				dwGetReceiveTimestamp(dev, &tStartReply2); 
				dwGetTransmitTimestamp(dev, &tEndReply2); break;
		}
	}
	else{
		switch(frame.type){
			case BEACON_RESPONSE:
				dwGetReceiveTimestamp(dev, &tStartReply1); 
				dwGetTransmitTimestamp(dev, &tEndReply1); break;
		}
	}
}

void rxcallback(dwDevice_t *dev){
  if(isBeacon == false){
	  dwGetData(dev, (uint8_t*)&frame, noDataFramesize);
	  switch(frame.type){
		  case BEACON_RESPONSE: sendAnchorResponse(dev, DestinationAddress); break;
		  case TRANSFER_FRAME: calculatePropagation(dev); break;
	  }
  }
  else{
	  dwGetData(dev, (uint8_t*)&frame, noDataFramesize);
	  switch(frame.type){
		  case PING: 
		  	  sendBeaconResponse(dev, DestinationAddress); break;
		  case ANCHOR_RESPONSE: 
		  	  dwGetReceiveTimestamp(dev, &tEndRound2);
			  sendTransferFrame(dev, DestinationAddress); break;
	  }
	
  }

}

void receiveFailedHandler(dwDevice_t *dev){ //For debugging because no memory access in dwHandleInterrupt

}

void rangingAnchor(){
	//TODO globale Addressspeicherung nutzen für Schleife um Destination zu wechseln
	while(true){
		if(sIRQ){
			dwHandleInterrupt(dwm);
		}
		if(sendRanging && hasSendRanging == false){
			sendRangingPoll(dwm, DestinationAddress);
			hasSendRanging = true;
		}	
	} 
  
   
}

void rangingBeacon(){
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
	sReset.input();
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
	dwAttachReceiveFailedHandler(dwm, receiveFailedHandler); //For Debugging
	

	
	//start configuration
	dwNewConfiguration(dwm);
	dwSetDefaults(dwm);
	dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(dwm, CHANNEL_2);
	dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

	memset(dwm->networkAndAddress, srcAddress, 1);// set own SourceAddress
	memset((dwm->networkAndAddress + 1), 0, 1); //fill Address with 0s
	memset((dwm->networkAndAddress + 2), 0x88, LEN_PANADR-2); //set panID

	dwInterruptOnReceiveFailed(dwm, true); //for debugging

    //dwSetFrameFilterAllowData(dwm, true);
    dwSetFrameFilterAllowBeacon(dwm, true);
	dwSetFrameFilter(dwm, true); //for global frame filtering
	
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
		
	//initialize Frame
	if(isBeacon){
		DestinationAddress = 0x00;
	}

	memcpy(frame.srcAddress, dwm->networkAndAddress, 2);
	memcpy(frame.destPan, (dwm->networkAndAddress + 2), 2);

	frame.type = 0;


  if(isBeacon == true){
	dwInterruptOnReceived(dwm, true);
	dwInterruptOnSent(dwm, true); //Interrupt on receiving a good Frame is triggered
    rangingBeacon();
  }
  else{
	dwInterruptOnReceived(dwm, true); //Interrupt on receiving a good Frame is triggered
	dwInterruptOnSent(dwm, true); //Interrupt on sending a Frame
	ranger.attach(&setSendRangingFlag, 2); //set PollingIntervall to 5 seconds
    rangingAnchor();
  }
}
//update driver for dw1000

/*
Anchor is sending Poll and in Sysstatus Register everything seems to send correctly
Beacon is not getting an interrupt, not even a FrameReceptionFailed etc -> the Sysmask register seems to be set correctly 
WHY?? -> srcAddress was set as the upper 8 bits of the 16 bit storage, instead it has to be set as the lower 8 bit.
now an interrupt is thrown at beacon-side

read sysstatus in dwHandleInterrupt breakpoint -> no memory access
possible solution: attach ReceiveFailedHandler and set Breakpoint there 



Try different dwm as beacon
Check FrameFiltering again
*/