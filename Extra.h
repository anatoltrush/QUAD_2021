#ifndef EXTRA_H
#define EXTRA_H

#include "Defines.h"

//#define DEBUG_EXTRA

class Extra
{
  private:
    uint8_t flash_pin = PIN_FLASH;
    uint8_t volt_pin  = PIN_VOLT;

    uint8_t percent   = 0;

    uint32_t prev_ms_flash = 0;
    uint32_t prev_ms_volt = 0;

    bool led_state = false;

    float signal        = 0.0f;

  public:
    Extra(uint8_t led_pin, uint8_t volt_pin);

    float output        = 0.0f;

    void flash(uint32_t ms);
    void get_volt(uint32_t ms);
};

#endif // EXTRA_H

// 12.6 -> 12100/7900 -> 4.98V
// 10.2 -> 12100/7900 -> 4.03V

// 12500/7500?
