// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
#include "mbed.h"
#include "PC.h"                                     // Serial Port via USB for debugging with Terminal
#include "DW1000.h"                                 // our DW1000 device driver
#include "MM2WayRanging.h"                          // our self developed ranging application

#define myprintf    pc.printf                       // to make the code adaptable to other outputs that support printf

Serial              pc(PA_9, PA_10,9600); // tx, rx;           // USB UART Terminal
DW1000          dw(PA_7, PA_6, PA_5, PA_4, PA_0);   // Device driver instanceSPI pins: (MOSI, MISO, SCLK, CS, IRQ)
MM2WayRanging   node(dw);                          // Instance of the two way ranging algorithm

void rangeAndDisplayAll(){
    node.requestRangingAll();                       // Request ranging to all anchors
    for (int i = 1; i <= 5; i++) {                  // Output Results
        myprintf("%f, ", node.distances[i]);
        //myprintf("T:%f", node.roundtriptimes[i]);
        //myprintf("\r\n");
    }
    myprintf("\r\n");
}

void calibrationRanging(int destination){

    const int numberOfRangings = 100;
    float rangings[numberOfRangings];
    int index = 0;
    float mean = 0;
    float start, stop;

    Timer localTimer;
    localTimer.start();

    start = localTimer.read();

    while (1) {

        node.requestRanging(destination);
        if(node.overflow){
            myprintf("Overflow! Measured: %f\r\n", node.distances[destination]); // This is the output to see if a timer overflow was corrected by the two way ranging class
        }

        if (node.distances[destination] == -1) {
            myprintf("Measurement timed out\r\n");
            wait(0.001);
            continue;
        }

        if (node.distances[destination] < 100) {
            rangings[index] = node.distances[destination];
            //myprintf("%f\r\n", node.distances[destination]);
            index++;

            if (index == numberOfRangings) {
            stop = localTimer.read();

                for (int i = 0; i < numberOfRangings - 1; i++)
                    rangings[numberOfRangings - 1] += rangings[i];

                mean = rangings[numberOfRangings - 1] / numberOfRangings;
                myprintf("\r\n\r\nMean %i: %f\r\n", destination, mean);
                myprintf("Elapsed Time for %i: %f\r\n", numberOfRangings, stop - start);

                mean = 0;
                index = 0;

                //wait(2);

                start = localTimer.read();
            }
        }

        else myprintf("%f\r\n", node.distances[destination]);

    }

}

struct __attribute__((packed, aligned(1))) DistancesFrame {
        uint8_t source;
        uint8_t destination;
        uint8_t type;
        float dist[4];
    };



void altCallbackRX();
// -----------------------------------------------------------------------------------------------

int main() {
    pc.printf("\r\nDecaWave 1.0   up and running!\r\n");            // Splashscreen
    dw.setEUI(0xFAEDCD01FAEDCD01);                                  // basic methods called to check if we have a working SPI connection
    pc.printf("DEVICE_ID register: 0x%X\r\n", dw.getDeviceID());
    pc.printf("EUI register: %016llX\r\n", dw.getEUI());
    pc.printf("Voltage: %fV\r\n", dw.getVoltage());

    bool isSender = 1;
    char* messageToSend = "Alarm";
    uint32_t framelength = 0b11;
    uint32_t preamblelength = 0b1000000000000000000;
    uint32_t TXConf = framelength | preamblelength;


    if(isSender == 0){
      char* message[8];

      while(1){
        //dw.writeRegister32(DW1000_SYS_CTRL, 0, 256); //enable receiving
        //dw.readRegister(DW1000_RX_BUFFER, 0, message, dw.getFramelength());
        //pc.printf("%d\n", message);
        startRX();
        receiveString(message);
        pc.printf("I received %s", message);
      }


    }
    else{
      while(1){
        //dw.writeRegister32(DW1000_TX_FCTRL, 0x00, TXConf); //set Framelength (+PreambleLength??)
        //dw.writeRegister(DW1000_TX_BUFFER, 0, &messageToSend, 1 ); //write sendRegister
        //dw.writeRegister32(DW1000_SYS_CTRL, 0, 0b10); //set Startbit to start TX sending
        sendString(messageToSend);
        wait(2);
      }
    }



}


void altCallbackRX() { // this callback was used in our verification test case for the observer node which only receives and outputs the resulting data

   DistancesFrame distFrame;
   float distances[4];
   dw.readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)&distFrame, dw.getFramelength());

    if (distFrame.destination == 5) {
        for (int i = 0; i<4; i++){
            myprintf("%f, ", distFrame.dist[i]);
        }

        myprintf("\r\n");
    }
    dw.startRX();
}
