#ifndef RADIO_INTERFACE_H
#define RADIO_INTERFACE_H

#include "mbed.h"

class Radio {
private:
    // requires MOSI, MISO, SCK, and CS
    SPI spi;
    
    // toggle the transmit/receive of the chip
    DigitalOut radioMode;

    // active low
    DigitalOut chipSelect;

    // interrupt for when new data is received
    // Incoming data should be processed as a command
    InterruptIn interruptPin;

    void defaultSetup();

public:
    // default constructor
    Radio();




};

#endif //RADIO_INTERFACE_H