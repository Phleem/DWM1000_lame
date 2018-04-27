// Adapted from Matthias Grob & Manuel Stalder - ETH Zürich - 2015

#include "DW1000.h"

// Change this depending on whether damaged or heatlhy DWM1000 modules are used.
const bool DWM1000_DAMAGED = true;
//const bool DWM1000_DAMAGED = false;

//#include "PC.h"
//static PC pc(USBTX, USBRX, 115200);           // USB UART Terminal

DW1000::DW1000(SPI& spi, InterruptIn* irq, PinName CS, PinName RESET)
: irq(irq), spi(spi), cs(CS), reset(RESET) {
    if (irq != NULL) {
        irq->rise(this, &DW1000::ISR);
    }

    setCallbacks(NULL, NULL);

    select();
    deselect();                         // Chip must be deselected first
    resetAll();                         // we do a soft reset of the DW1000 everytime the driver starts

    // Configuration TODO: make method for that
    // User Manual "2.5.5 Default Configurations that should be modified" p. 22
    //Those values are for the standard mode (6.8Mbps, 5, 16Mhz, 32 Symbols) and are INCOMPLETE!
//    writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);
//    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);
//    writeRegister32(DW1000_DRX_CONF, 0x08, 0x311A002D);
//    writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);
//    writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);
//    writeRegister32(DW1000_TX_POWER, 0, 0x0E082848);
//    writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);
//    writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);
//    writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xA6);


    //Those values are for the 110kbps mode (5, 16MHz, 1024 Symbols) and are quite complete
    writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);             //AGC_TUNE1 for 16MHz PRF
    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);         //AGC_TUNE2 (Universal)
    writeRegister16(DW1000_AGC_CTRL, 0x12, 0x0055);             //AGC_TUNE3 (Universal)

    writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);             //DRX_TUNE0b for 110kbps
    writeRegister16(DW1000_DRX_CONF, 0x04, 0x0087);             //DRX_TUNE1a for 16MHz PRF
    writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);             //DRX_TUNE1b for 110kbps & > 1024 symbols
    writeRegister32(DW1000_DRX_CONF, 0x08, 0x351A009A);         //PAC size for 1024 symbols preamble & 16MHz PRF
    //writeRegister32(DW1000_DRX_CONF, 0x08, 0x371A011D);               //PAC size for 2048 symbols preamble

    writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);              //LDE_CFG1
    writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);           //LDE_CFG2 for 16MHz PRF

    writeRegister32(DW1000_TX_POWER, 0, 0x28282828);            //Power for channel 5

    writeRegister8(DW1000_RF_CONF, 0x0B, 0xD8);                 //RF_RXCTRLH for channel 5
    writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);          //RF_TXCTRL for channel 5

    writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);                 //TC_PGDELAY for channel 5

    writeRegister32 (DW1000_FS_CTRL, 0x07, 0x0800041D);         //FS_PLLCFG for channel 5
    writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xA6);                //FS_PLLTUNE for channel 5

    loadLDE();                          // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned off or there's receiving malfunction see User Manual LDELOAD on p22 & p158
    
    // 110kbps CAUTION: a lot of other registers have to be set for an optimized operation on 110kbps
    writeRegister16(DW1000_TX_FCTRL, 1, 0x0800 | 0x0100 | 0x0080); // use 1024 symbols preamble (0x0800) (previously 2048 - 0x2800), 16MHz pulse repetition frequency (0x0100), 110kbps bit rate (0x0080) see p.69 of DW1000 User Manual
    writeRegister8(DW1000_SYS_CFG, 2, 0x44);    // enable special receiving option for 110kbps (disable smartTxPower)!! (0x44) see p.64 of DW1000 User Manual [DO NOT enable 1024 byte frames (0x03) becuase it generates disturbance of ranging don't know why...]

    writeRegister16(DW1000_TX_ANTD, 0, 16384); // set TX and RX Antenna delay to neutral because we calibrate afterwards
    writeRegister16(DW1000_LDE_CTRL, 0x1804, 16384); // = 2^14 a quarter of the range of the 16-Bit register which corresponds to zero calibration in a round trip (TX1+RX2+TX2+RX1)

    writeRegister8(DW1000_SYS_CFG, 3, 0x20);    // enable auto reenabling receiver after error

    if (irq != NULL) {
        irq->enable_irq();
    }
}

void DW1000::setCallbacks(void (*callbackRX)(void), void (*callbackTX)(void)) {
    bool RX = false;
    bool TX = false;
    if (callbackRX) {
        this->callbackRX.attach(callbackRX);
        RX = true;
    }
    if (callbackTX) {
        this->callbackTX.attach(callbackTX);
        TX = true;
    }
    setInterrupt(RX, TX);
}

uint32_t DW1000::getDeviceID() {
    uint32_t result;
    readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
    return result;
}

uint64_t DW1000::getEUI() {
    uint64_t result;
    readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
    return result;
}

void DW1000::setEUI(uint64_t EUI) {
    writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

float DW1000::getVoltage() {
    uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
    writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
    writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
    readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
    float Voltage = buffer[5] * 0.0057 + 2.3;
    //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration
    return Voltage;
}

uint64_t DW1000::getStatus() {
    return readRegister40(DW1000_SYS_STATUS, 0);
}

bool DW1000::hasReceivedFrame() {
    uint64_t status = getStatus();
    return status & 0x4000;
}

void DW1000::clearReceivedFlag() {
    writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
}

bool DW1000::hasTransmissionStarted() {
    uint64_t status = getStatus();
    return status & 0x10;
}

bool DW1000::hasSentPreamble() {
    uint64_t status = getStatus();
    return status & 0x20;
}

bool DW1000::hasSentPHYHeader() {
    uint64_t status = getStatus();
    return status & 0x40;
}

bool DW1000::hasSentFrame() {
    uint64_t status = getStatus();
    return status & 0x80;
}

void DW1000::clearSentFlag() {
    writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
}

uint64_t DW1000::getSYSTimestamp() {
    return readRegister40(DW1000_SYS_TIME, 0);
}

uint64_t DW1000::getRXTimestamp() {
    return readRegister40(DW1000_RX_TIME, 0);
}

uint64_t DW1000::getTXTimestamp() {
    return readRegister40(DW1000_TX_TIME, 0);
}

float DW1000::getSYSTimestampUS() {
    return getSYSTimestamp() * TIMEUNITS_TO_US;
}

float DW1000::getRXTimestampUS() {
    return getRXTimestamp() * TIMEUNITS_TO_US;
}

float DW1000::getTXTimestampUS() {
    return getTXTimestamp() * TIMEUNITS_TO_US;
}

uint16_t DW1000::getStdNoise() {
    return readRegister16(DW1000_RX_FQUAL, 0x00);
}

uint16_t DW1000::getPACC() {
    uint32_t v = readRegister32(DW1000_RX_FINFO, 0x00);
    v >>= 20;
    return static_cast<uint16_t>(v);
}

uint16_t DW1000::getFPINDEX() {
    return readRegister16(DW1000_RX_TIME, 0x05);
}

uint16_t DW1000::getFPAMPL1() {
    return readRegister16(DW1000_RX_TIME, 0x07);
}

uint16_t DW1000::getFPAMPL2() {
    return readRegister16(DW1000_RX_FQUAL, 0x02);
}

uint16_t DW1000::getFPAMPL3() {
    return readRegister16(DW1000_RX_FQUAL, 0x04);
}

uint16_t DW1000::getCIRPWR() {
    return readRegister16(DW1000_RX_FQUAL, 0x06);
}

uint8_t DW1000::getPRF()
{
    uint32_t prf_mask = static_cast<uint32_t>(0x1 << 19 | 0x1 << 18);
    uint32_t prf = readRegister32(DW1000_CHAN_CTRL, 0x00);
    prf >>= 18;
    return static_cast<uint8_t>(prf & prf_mask);
}

void DW1000::sendString(char* message) {
    sendFrame((uint8_t*)message, strlen(message)+1);
}

void DW1000::receiveString(char* message) {
    readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, getFramelength());  // get data from buffer
}

void DW1000::sendFrame(uint8_t* message, uint16_t length) {
    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames

    Timer timer;
    timer.start();
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
    
    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);
    
    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
}

void DW1000::sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp) {
    clearSentFlag();                                                // This is necessary, otherwise we pick up the transmission time of the previous send

    if (TxTimestamp > CONST_2POWER40) {
        TxTimestamp -= CONST_2POWER40;
    }

    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);

    writeRegister40(DW1000_DX_TIME, 0, TxTimestamp);                //write the timestamp on which to send the message

    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02 | 0x04);                // trigger sending process by setting the TXSTRT and TXDLYS bit
}

void DW1000::startRX() {
    writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
    wait_us(16);                                                    // According to page 81 in the user manual (RXENAB bit)
}

void DW1000::stopTRX() {
    writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode by setting the TRXOFF bit
}

// PRIVATE Methods ------------------------------------------------------------------------------------
void DW1000::loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
    writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
    writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
    wait_us(150);
    writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void DW1000::resetRX() {    
    writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void DW1000::hardwareReset(PinName reset_pin) {
    DigitalInOut reset(reset_pin);
    hardwareReset(reset);
}

void DW1000::hardwareReset(DigitalInOut& reset) {
    if (reset.is_connected()) {
        // DWM1000 RESET logic.
        if (DWM1000_DAMAGED) {
            // The following code works for damaged DWM1000 modules.
            // IMPORTANT: This will damage healthy DWM1000 modules!
            reset.output();
            reset = 1;
            wait_ms(100);
            reset = 0;
            wait_ms(100);
            reset = 1;
            wait_ms(100);
        } else {
            // The following code works for healthy DWM1000 modules
            reset.output();
            reset = 0;
            wait_ms(100);
            reset.input();
        }
    }
}

void DW1000::softwareReset() {
    stopTRX();
    clearReceivedFlag();
    clearSentFlag();
}

void DW1000::resetAll() {
    hardwareReset(reset);

    writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
    writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
    wait_us(10);                            // wait for PLL to lock
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}


void DW1000::setInterrupt(bool RX, bool TX) {
    writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

void DW1000::ISR() {
    uint64_t status = getStatus();
    if (status & 0x4000) {                                          // a frame was received
        callbackRX.call();
        writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
    }
    if (status & 0x80) {                                            // sending complete
        callbackTX.call();
        writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
    }
}

uint16_t DW1000::getFramelength() {
    uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
    framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
    return framelength;
}

// SPI Interface ------------------------------------------------------------------------------------
uint8_t DW1000::readRegister8(uint8_t reg, uint16_t subaddress) {
    uint8_t result;
    readRegister(reg, subaddress, &result, 1);
    return result;
}

uint16_t DW1000::readRegister16(uint8_t reg, uint16_t subaddress) {
    uint16_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 2);
    return result;
}

uint32_t DW1000::readRegister32(uint8_t reg, uint16_t subaddress) {
    uint32_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 4);
    return result;
}

uint64_t DW1000::readRegister40(uint8_t reg, uint16_t subaddress) {
    uint64_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 5);
    result &= 0xFFFFFFFFFF;                                 // only 40-Bit
    return result;
}

void DW1000::writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
    writeRegister(reg, subaddress, &buffer, 1);
}

void DW1000::writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}

void DW1000::writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}

void DW1000::writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer) {
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}

void DW1000::readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, false);
    for(int i=0; i<length; i++)                             // get data
        buffer[i] = spi.write(0x00);
    deselect();
}

void DW1000::writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, true);
    for(int i=0; i<length; i++)                             // put data
        spi.write(buffer[i]);
    deselect();
}

void DW1000::setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
    reg |=  (write * DW1000_WRITE_FLAG);                                        // set read/write flag
    select();
    if (subaddress > 0) {                                                       // there's a subadress, we need to set flag and send second header byte
        spi.write(reg | DW1000_SUBADDRESS_FLAG);
        if (subaddress > 0x7F) {                                                // sub address too long, we need to set flag and send third header byte
            spi.write((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG); // and 
            spi.write((uint8_t)(subaddress >> 7));
        } else {
            spi.write((uint8_t)subaddress);
        }
    } else {
        spi.write(reg);                                                         // say which register address we want to access
    }
}

void DW1000::select() {     // always called to start an SPI transmission
    if (irq != NULL) {
        irq->disable_irq();
    }
    cs = 0;                 // set Cable Select pin low to start transmission
}

void DW1000::deselect() {   // always called to end an SPI transmission
    cs = 1;                 // set Cable Select pin high to stop transmission
    if (irq != NULL) {
        irq->enable_irq();
    }
}
