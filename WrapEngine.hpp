#ifndef WRAPENGINE_HPP
#define WRAPENGINE_HPP

#include <Arduino.h>
#include <Servo.h>

#include "Defines.hpp"

class WrapEngine
{
  public:

    Servo motorFR;
    Servo motorFL;
    Servo motorRR;
    Servo motorRL;

    uint16_t POWER_IN_Diag_FRRL = MIN_POWER;
    uint16_t POWER_IN_Diag_FLRR = MIN_POWER;

    uint16_t POWER_FR = MIN_POWER;
    uint16_t POWER_FL = MIN_POWER;
    uint16_t POWER_RR = MIN_POWER;
    uint16_t POWER_RL = MIN_POWER;

    uint16_t POWER_FULL_DIFF = 0;

    uint32_t _prev_millis = 0; // последний момент смены состояния

    void init();
    void apply(uint32_t ms);
    void apply(uint32_t ms, float gap_perc);
};
#endif // WRAPENGINE_HPP
