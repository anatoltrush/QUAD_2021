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

    //byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" };
};
#endif // WRAPRADIO_HPP
