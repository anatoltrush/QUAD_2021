#ifndef EXTRA_H
#define EXTRA_H

#include "Defines.h"

//#define DEBUG_EXTRA

class Extra
{
  public:
    Extra(uint8_t led_pin, uint8_t volt_pin);

    float voltOutput        = 0.0f;

    void flash(uint32_t ms);
    void getVoltQuad(uint32_t ms);
    void customCommand(uint8_t* msg_data, uint32_t ms);

  private:
    uint8_t flashPin = PIN_FLASH;
    uint8_t voltPin  = PIN_VOLT;

    bool ledState = false;

    int16_t voltPercent = 0;

    uint32_t prevFlashMs = 0;
    uint32_t prevVoltMs = 0;
    uint32_t prevCmndMs = 0;

    float diffMinMax = VOLT_MAX - VOLT_MIN;
};

#endif // EXTRA_H

// 12200(12300)/7800(7700)
