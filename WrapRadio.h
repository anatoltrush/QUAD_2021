#ifndef WRAPRADIO_H
#define WRAPRADIO_H

#include <RF24.h>

#include "Defines.h"

class WrapRadio
{
  public:
    RF24* radio = NULL;

    uint8_t pipeNum = 0;
    // ACK bytes: 0 - quad volt, 1 - is max reached
    uint8_t ack_msg[SIZE_OF_ACK] = {0};
    // left[1]throttle, left[0]yaw, right[4]pitch, right[3]roll
    uint8_t data_msg[SIZE_OF_DATA] = {0};

    WrapRadio();
    ~WrapRadio();

    void init();

    void getData(uint8_t volt, bool isReached, uint8_t numEng);
};
#endif // WRAPRADIO_H
