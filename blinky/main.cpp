#include "mbed.h"
#include "DW1000Utils.h"
#include "DW1000.h"

Serial pc(PA_9, PA_10); // tx, rx
SPI spi(PA_7, PA_6 ,PA_5) //MOSI, MISO, SerialClock
_
DigitalOut cs(PA_4); //ChipSelect

DigitalOut Pin0(PB_0) ;
DigitalOut Pin1(PB_1) ;


// main() runs in its own thread in the OS
int main() {
	cs = 1; //deselect chip	
	
	
    while (true) {
        Pin1 = 1;
        Pin0 = 1;
    
        //led1 = !led1;
        wait(1);
        
        
        Pin1 = 0;
        Pin0 = 0;
      
        wait(1);
        pc.printf("Hello World!\n");

    }
}

