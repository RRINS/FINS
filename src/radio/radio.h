#ifndef RADIO_H
#define RADIO_H

#include "mbed.h"

class Radio {
private:
    // defines MOSI, MISO, SCK, and CS
    SPI spi; //Radio(ARDUINO_UNO_SPI_MOSI, ARDUINO_UNO_SPI_MISO, ARDUINO_UNO_SPI_SCK, ARDUINO_UNO_SPI_CS);
    
    // toggle the transmit/receive of the chip
    // todo: this should be changed to a digitalOut
    uint8_t chipEnablePin;

    // interrupt for when new data is received
    // todo: change to an InterruptIn with a callback function for how to handle it.
    //       Incoming data should be processed as a command
    uint8_t interruptPin;

    void defaultSetup();

public:
    // default constructor
    Radio();




};

#endif //RADIO_H
