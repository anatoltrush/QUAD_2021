#ifndef WRAPENG_HPP
#define WRAPENG_HPP

#include <Servo.h>
#include <GyverPID.h>

#include "Defines.hpp"

//#define DEBUG_ENG

class WrapEng
{
  public:
    GyverPID regulator_FR_RL;
    GyverPID regulator_FL_RR;
    GyverPID regulator_D1_D2;

    uint16_t POWER_IN_Diag_FRRL = MIN_POWER;
    uint16_t POWER_IN_Diag_FLRR = MIN_POWER;

    uint16_t POWER_FR = MIN_POWER;
    uint16_t POWER_FL = MIN_POWER;
    uint16_t POWER_RR = MIN_POWER;
    uint16_t POWER_RL = MIN_POWER;

    bool isMaxReached = false;

    void init();
    void apply(uint16_t pid_FR_RL, uint16_t pid_FL_RR, uint32_t ms);

  private:
    const uint16_t POWER_FULL_DIFF = MAX_POWER - MIN_POWER;

    uint32_t prev_millis = 0;

    Servo motorFR;
    Servo motorFL;
    Servo motorRR;
    Servo motorRL;

#ifdef DEBUG_ENGINE
    uint16_t counter = 0;
#endif
};

#endif // WRAPENG_HPP
