#ifndef WRAPENGINE_HPP
#define WRAPENGINE_HPP

#include <Arduino.h>
#include <Servo.h>

#include "Defines.hpp"

#define DEBUG_ENG

class WrapEngine
{
  public:
    WrapEngine();

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

    uint32_t _prev_millis = 0; // последний момент смены состояния

    void apply(uint32_t ms);
};
#endif // WRAPENGINE_HPP
