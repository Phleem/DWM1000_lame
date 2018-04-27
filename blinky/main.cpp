#include "mbed.h"
#include "DW1000Utils.h"
#include "DW1000.h"

Serial pc(PA_9, PA_10); // tx, rx
SPI spi(PA_7, PA_6 ,PA_5); //MOSI, MISO, SerialClock

DigitalOut chipSelect(PA_4); //ChipSelect
InterruptIn irq(PA_0);

DigitalOut Pin0(PB_0) ;
DigitalOut Pin1(PB_1) ;


// main() runs in its own thread in the OS
int main() {
	chipSelect = 1; //deselect chip

	DW1000 dw1000(spi, &irq, PA_4, NC);

	dw1000.writeRegister32( DW1000_GPIO_CTRL , 0x00, 0x00); // Set GPIO as GPIO (no special function)
	dw1000.writeRegister32( DW1000_GPIO_CTRL , 0x08, 0b110000); //set DirectionRegisterMask, and set Direction to Output
	dw1000.writeRegister32( DW1000_GPIO_CTRL , 0x0C, 0b110011);// set GPIO_DOutMask to enable GPIO Output; Set GPIO 1 & GPIO 0 to HIGH and enable in Mask


uint32_t deviceID = 0;
deviceID = dw1000.getDeviceID();




    while (true) {
        Pin1 = 1;
        Pin0 = 1;

        //led1 = !led1;
        wait(1);


        Pin1 = 0;
        Pin0 = 0;

        wait(1);
        pc.printf("Hello World!\n");
				pc.printf("%x", deviceID);

    }
}
