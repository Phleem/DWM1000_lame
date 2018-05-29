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

const char* txPacket = "foobar";
uint8_t data[255];
bool isBeacon = false;
bool sendRanging = false;
dwDevice_t dwm_device;
dwDevice_t* dwm = &dwm_device;
dwTime_t tStartRound;
dwTime_t tEndRound;
dwTime_t tStartReply;
dwTime_t tDelay;
dwTime_t tEndReply;
long long int tRound;
long long int tReply;
float tProp;

void enableClocks(){
	uint8_t enable = 0b010100;
	dwSpiWrite(dwm, 0x36, 0, (void*) enable, sizeof(enable));
}

void calculateSSTimeOfFlight(long long int* timeRound, long long int* timeReply, float* tProp){
    *tProp = (float) (0.5f * (*timeRound - *timeReply));
}
void calculateDeltaTime(uint64_t* timeStart, uint64_t* timeEnd, long long int* timeDelta){
    *timeDelta = llabs((long long int)(*timeEnd) - (long long int)(*timeStart));
}

void send_dummy(dwDevice_t* dev) {
	dwNewTransmit(dev);
	//dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)txPacket, strlen(txPacket));
	dwStartTransmit(dev);
}

void sendReply(dwDevice_t* dev) {

	dwGetReceiveTimestamp(dev, &tStartReply);
	data[0] = tStartReply.full;
	dwNewTransmit(dev);
	tEndReply = dwSetDelay(dev, &tDelay);
	data[40] = tEndReply.full;
	dwSetData(dev, data, sizeof(data));
	dwStartTransmit(dev);
}

void send_ranging(dwDevice_t* dev) {
	dwNewTransmit(dev);
	dwStartTransmit(dev);
}

void setSendRangingFlag(){
	sendRanging = !sendRanging;
}

void txcallback(dwDevice_t *dev){
	if(isBeacon == false){
		dwGetTransmitTimestamp(dev, &tStartRound);
		dwNewReceive(dwm);
   		dwStartReceive(dwm);
	}
}

void rxcallback(dwDevice_t *dev){
  if(isBeacon == true){
    sendReply(dev);
  }
  else{
	dwGetReceiveTimestamp(dev, &tEndRound);
	dwGetData(dev, tStartReply.raw, 5);
	dwGetData(dev, tEndReply.raw, 5);

	calculateDeltaTime(&(tStartReply.full), &(tEndReply.full), &tReply);
	calculateDeltaTime(&(tStartRound.full), &(tEndRound.full), &tRound);

	calculateSSTimeOfFlight(&tRound, &tReply, &tProp);

	tProp /= 63897.6f; //to ns
	float distance = tProp * 0.3f;

	pc.printf("%f\n",tProp);
	pc.printf("%f\n", distance);
    //pc.printf("%"PRIu64"\n",tStart.full);
	//pc.printf("%"PRIu64"\n",tEnd.full);
	char seperator = 'X';
	pc.printf("%c\n",seperator);

  }

}

void rangingBeacon(){
  dwNewReceive(dwm);
  //dwSetDefaults(dwm);
  dwStartReceive(dwm);
	while(true){
		dwHandleInterrupt(dwm);
	} 
  
   
}

void rangingAnchor(){

	if(sIRQ == 1){//poll IRQ-Pin
		dwHandleInterrupt(dwm);
	}
	if(sendRanging == true){
		send_ranging(dwm);
	}		
}

//TODO: implement SS-2Way-Ranging
//Anchor: send something, take timestamp
//Beacon: get TransmitTimestamp timeRound1
//Beacon: get ReceiveTimestamp timeReply1
//Beacon: isReceiveTimestampvailable
//Beacon: send to Sender
//Anchor: get TransmitTimestamp timeReply2
//Anchor: get ReceiveTimestamp timeRound2
//Anchor: send to Receiver timeReply2 und timeRound2
//Beacon: calculate timeOfFlight and Range


// main() runs in its own thread in the OS
int main() {

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

	dwInterruptOnReceived(dwm,true); //Interrupt on receiving a good Frame is triggered
	dwInterruptOnSent(dwm, true); //Interrupt on sending a Frame

	dwCommitConfiguration(dwm);
	//enableClocks();
	tStartRound.full = 0;
	tEndRound.full = 0;
	tStartReply.full = 0;
	tEndReply.full = 0;
	tDelay.full = 0;
	tDelay.full = 63897600000; //1sec
	//dwIdle(dwm);

  if(isBeacon == true){
	dwReceivePermanently(dwm, true);
    rangingBeacon();
  }
  else{
	Ticker ranger;
	ranger.attach(&setSendRangingFlag, 2.5);
    while(true){
      rangingAnchor();
    }
  }
}

//TODO implement calculation of Flightime; test