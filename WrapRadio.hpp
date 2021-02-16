#ifndef WRAPRADIO_HPP
#define WRAPRADIO_HPP

#include <Arduino.h>
#include <RF24.h>

#include "Defines.hpp"

class WrapRadio
{
  public:
    RF24* radio = NULL;

    uint8_t pipeNum = 0;
    uint8_t gotByte = 0;    

    WrapRadio();
    ~WrapRadio();

    void init();
};
#endif // WRAPRADIO_HPP
