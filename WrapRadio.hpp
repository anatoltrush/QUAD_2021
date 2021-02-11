#ifndef WRAPRADIO_HPP
#define WRAPRADIO_HPP

#include <Arduino.h>

#include <RF24.h>

#define PIN_NRF_CE      9
#define PIN_NRF_CS      10
#define PIN_NRF_MOSI    11
#define PIN_NRF_MISO    12
#define PIN_NRF_SCK     13

class WrapRadio
{
  public:
    RF24* radio = NULL;

    byte pipeNo, gotByte;    

    WrapRadio();
    ~WrapRadio();

    void init();
};
#endif // WRAPRADIO_HPP
