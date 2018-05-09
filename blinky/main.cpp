// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
#include "mbed.h"
#include "PC.h"                                     // Serial Port via USB for debugging with Terminal
#include "DW1000.h"                                 // our DW1000 device driver
#include "MM2WayRanging.h"                          // our self developed ranging application

#define myprintf    pc.printf                       // to make the code adaptable to other outputs that support printf

Serial              pc(PA_9, PA_10,9600); // tx, rx;           // USB UART Terminal
DW1000          dw(PA_7, PA_6, PA_5, PA_4, PA_0);   // Device driver instanceSPI pins: (MOSI, MISO, SCLK, CS, IRQ)
MM2WayRanging   node(dw);                          // Instance of the two way ranging algorithm
// -----------------------------------------------------------------------------------------------

uint64_t* createMACHeader(uint8_t destination, uint8_t source, uint8_t sequence_number){
    uint16_t frameControl = 0b1000100000000001;
    uint16_t panID = 13;
    uint64_t* macHeader = (uint64_t*)((frameControl << 72) | (sequence_number << 64) | (panID << 48) | (destination << 32) | (panID << 16) | source);
    return macHeader;

}
void setPanIDandShortAddress(uint16_t panID, uint16_t short_address){
  dw.writeRegister16(DW1000_PANADR, 0, short_address);
  dw.writeRegister16(DW1000_PANADR, 16, panID);
}

int main() {
    pc.printf("\r\nDecaWave 1.0   up and running!\r\n");            // Splashscreen
    dw.setEUI(0xFAEDCD01FAEDCD01);                                  // basic methods called to check if we have a working SPI connection
    pc.printf("DEVICE_ID register: 0x%X\r\n", dw.getDeviceID());
    pc.printf("EUI register: %016llX\r\n", dw.getEUI());
    pc.printf("Voltage: %fV\r\n", dw.getVoltage());
    pc.printf("TEST\n");
    dw.loadLDE();

    bool isSender = 1;

    uint8_t framelength = 0b1111111;
    uint32_t preamblelength = 0b1000000000000000000;
    uint32_t TXConf = 0b1000000000001111111 ;//framelength | preamblelength;

    dw.writeRegister(DW1000_SYS_CFG, 0, (uint8_t*)0b100, 3); // enable reception of dataFrames
    dw.writeRegister(DW1000_TX_FCTRL, 0, (uint8_t*)&TXConf, 19); //set Preamble to 64 and Framelength to 128

    if(isSender == 0){   //is anchor
      setPanIDandShortAddress(13, 0);//node.address = 0;
      uint64_t message;
      while(1){
        //dw.writeRegister32(DW1000_SYS_CTRL, 0, 256); //enable receiving
        //dw.readRegister(DW1000_RX_BUFFER, 0, message, dw.getFramelength());
        //pc.printf("%d\n", message);
        pc.printf("Recieving: \n");
        dw.startRX();
        dw.readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, dw.getFramelength());
        pc.printf("I received %d \n", message);
        wait(1);
      }
    }
    else{
      pc.printf("isSender\n");                   // is beacon
      uint8_t* message;
      setPanIDandShortAddress(13, 1);//node.address = 1;
      while(1){
        //dw.writeRegister32(DW1000_TX_FCTRL, 0x00, TXConf); //set Framelength (+PreambleLength??)
        //dw.writeRegister(DW1000_TX_BUFFER, 0, &messageToSend, 1 ); //write sendRegister
        //dw.writeRegister32(DW1000_SYS_CTRL, 0, 0b10); //set Startbit to start TX sending
        uint64_t* macHeader = createMACHeader(0,1,0);
        //message = (uint8_t*)((*macHeader << 8) | (uint16_t)'A');
        pc.printf("after macHeader");
        dw.sendFrame(message, (uint16_t)sizeof(message));

        uint64_t status = dw.getStatus();
        pc.printf("Status: ");
        pc.printf("%d\n", status);
        wait(2);
      }
    }

//TODO see systemevent register 0x0F for debugging purposes

}
