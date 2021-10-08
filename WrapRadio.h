#ifndef WRAPRADIO_H
#define WRAPRADIO_H

#include <RF24.h>

#include "Defines.h"

#define DEBUG_RAD

class WrapRadio
{
  public:
    WrapRadio();
    ~WrapRadio();

    RF24* radio = NULL;

    uint8_t pipeNum = 0;

    // ACK bytes: 0 - quad volt, 1 - is max reached
    uint8_t ack_msg[SIZE_OF_ACK] = {0};

    // left[1]throttle, left[0]yaw, right[4]pitch, right[3]roll
    uint8_t data_msg[SIZE_OF_DATA] = {0};

    bool isConnLost = true;

    void init();
    void getData(uint8_t volt, bool isReached, uint8_t numEng);

  private:
    uint32_t lastGetData = 0;
    uint32_t noGetData = 0;

    uint32_t prevNewRadio = 0;
};

// _________________________
//#define DEBUG_EXTRA
// 12200(12300)/7800(7700)
class Extra
{
  public:
    Extra();

    float voltOutput        = 0.0f;

    void flash(uint32_t ms);
    void getVoltQuad(uint32_t ms);
    void customCommand(uint8_t* msg_data, uint32_t ms);

  private:
    uint8_t PinFlash = 0;
    uint8_t PinVolt = 0;
    uint8_t PinAux1 = 0;
    uint8_t PinAux2 = 0;

    bool flashState = false;
    bool aux1State = false;

    int16_t voltPercent = 0;

    uint32_t prevFlashMs = 0;
    uint32_t prevVoltMs = 0;
    uint32_t prevCmndMs = 0;

    float diffMinMax = VOLT_MAX - VOLT_MIN;
};
#endif // WRAPRADIO_H
