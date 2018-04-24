#include "mbed.h"
Serial pc(PA_9, PA_10); // tx, rx

//DigitalOut led1(LED1);

DigitalOut Pin0(PB_0) ;
DigitalOut Pin1(PB_1) ;
DigitalOut Pin2(PB_2) ;
DigitalOut Pin3(PB_3) ;
DigitalOut Pin4(PB_4) ;
DigitalOut Pin5(PB_5) ;
DigitalOut Pin6(PB_6) ;


// main() runs in its own thread in the OS
int main() {
    while (true) {
        Pin1 = 1;
        Pin0 = 1;
        Pin2 = 1;
        Pin3 = 1;
        Pin4 = 1;
        Pin5 = 1;
        Pin6 = 1;
        //led1 = !led1;
        wait(0.01);
        
        
        Pin1 = 0;
        Pin0 = 0;
        Pin2 = 0;
        Pin3 = 0;
        Pin4 = 0;
        Pin5 = 0;
        Pin6 = 0;
        wait(0.01);
        pc.printf("Hello World!\n");

    }
}


 

 
//int main() {
//    
//    while(1) {
 //       pc.putc(pc.getc() + 1);
 //   }
//}
