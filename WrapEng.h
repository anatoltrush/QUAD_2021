#ifndef WRAPENG_H
#define WRAPENG_H

#include <Servo.h>
#include <GyverPID.h>

#include "Defines.h"

//#define DEBUG_ENG

class WrapEng
{
  public:
    GyverPID regulator_FR_RL;
    GyverPID regulator_FL_RR;
    GyverPID regulator_D1_D2;

    uint16_t POWER_IN_MAIN = MIN_POWER;
    uint16_t POWER_IN_Diag_FRRL = POWER_IN_MAIN;
    uint16_t POWER_IN_Diag_FLRR = POWER_IN_MAIN;

    uint16_t POWER_FR = MIN_POWER;
    uint16_t POWER_FL = MIN_POWER;
    uint16_t POWER_RR = MIN_POWER;
    uint16_t POWER_RL = MIN_POWER;

    // IS MAX
    bool isMaxReached = false;
    uint8_t numWarnEngine = 0;

    void init();
    void analyzeCommand(uint8_t* msg_data, uint32_t ms);
    void apply(uint32_t ms);

  private:
    const uint16_t POWER_FULL_DIFF = MAX_POWER - MIN_POWER;

    uint32_t prevApplyMs = 0;
    uint32_t prevCmndMs = 0;

    Servo motorFR; // 1
    Servo motorFL; // 2
    Servo motorRR; // 3
    Servo motorRL; // 4

#ifdef DEBUG_ENGINE
    uint16_t counter = 0;
#endif

    uint16_t* powers[4];
    void checkMinMax();
    void checkWarning();
};

#endif // WRAPENG_H
