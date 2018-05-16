#include "mbed.h"
#include <string>
#include <inttypes.h>
extern "C" {
#include "libdw1000.h"

using namespace std;
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
bool isBeacon = false;
dwDevice_t dwm_device;
dwDevice_t* dwm = &dwm_device;
dwTime_t tStart;
dwTime_t tEnd;
uint32_t tProp;

string uint64ToString(uint64_t input) {
  string result = "";
  uint8_t base = 10;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

void enableClocks(){
	uint8_t enable = 0b010100;
	dwSpiWrite(dwm, 0x36, 0, (void*) enable, sizeof(enable));
}

void send_dummy(dwDevice_t* dev) {
	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)txPacket, strlen(txPacket));
	dwStartTransmit(dev);
}

void send_ranging(dwDevice_t* dev) {
	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)txPacket, strlen(txPacket));
  dwGetSystemTimestamp(dev, &tStart);
	dwStartTransmit(dev);
  dwNewReceive(dev);
  dwStartReceive(dev);
}

void txcallback(dwDevice_t *dev){}

void rxcallback(dwDevice_t *dev){
  if(isBeacon == true){
		
    send_dummy(dev);
    dwNewReceive(dev);
    dwStartReceive(dev);
  }
  else{
    dwGetSystemTimestamp(dev, &tEnd);
  }

}





void calculateSSTimeOfFlight(uint32_t* timeRound, uint32_t* timeReply, uint32_t* timeOfFlight){
    *timeOfFlight = (uint32_t) (0.5f * (*timeRound - *timeReply));
}
void calculateDeltaTime(uint32_t* timeStart, uint32_t* timeEnd, uint32_t* timeDelta){
    *timeDelta = (*timeEnd) - (*timeStart);
}

void rangingBeacon(){
  dwNewReceive(dwm);
  dwSetDefaults(dwm);
  dwStartReceive(dwm);
  dwInterruptOnReceived(dwm, true);

    while(true){
    dwHandleInterrupt(dwm);
    }
}

void rangingAnchor(){
    dwInterruptOnReceived(dwm, true);
		dwTime_t systemTime1;
		systemTime1.full = 0;
		dwTime_t systemTime2;
		systemTime2.full = 0;
		dwGetSystemTimestamp(dwm, &systemTime1);
		wait(2);
		dwGetSystemTimestamp(dwm, &systemTime2);
		pc.printf("%"PRIu32"\n",systemTime1.low32);
		pc.printf("%"PRIu32"\n",systemTime2.low32);
    
		send_ranging(dwm);
    calculateDeltaTime(&(tStart.low32), &(tEnd.low32), &tProp);
		
    //pc.printf("%"PRIu32"\n",tProp);
    //pc.printf("%"PRIu64"\n",tProp);
		//pc.printf("%s", uint64ToString(*tProp));
		//pc.printf("%ll", tProp);
		//pc.printf("%")
}
//Sender: send something
//Receiver: get TransmitTimestamp timeRound1
//Receiver: get ReceiveTimestamp timeReply1
//Receiver: isReceiveTimestampvailable
//Receiver: send to Sender
//Sender: get TransmitTimestamp timeReply2
//Sender: get ReceiveTimestamp timeRound2
//Sender: send to Receiver timeReply2 und timeRound2
//Receiver: calculate timeOfFlight and Range


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

	dwCommitConfiguration(dwm);
	enableClocks();
	tStart.full = 0;
	tEnd.full = 0;
	dwIdle(dwm);

  if(isBeacon == true){
    rangingBeacon();
  }
  else{
    while(true){
      rangingAnchor();
      wait(2);
    }
  }
}
