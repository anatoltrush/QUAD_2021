#ifndef WRAPRADIO_H
#define WRAPRADIO_H

#include <RF24.h>

#include "Defines.h"

class WrapRadio
{
  public:
    RF24* radio = NULL;

    uint8_t pipeNum = 0;
    uint8_t ack_msg[SIZE_OF_ACK] = {0}; // ACK bytes: 0 - quad volt, 1 - is max reached

    WrapRadio();
    ~WrapRadio();

    void init();
    
    private:    
};
#endif // WRAPRADIO_H
