#ifndef WRAPENG_H
#define WRAPENG_H

#include <Servo.h>
#include <GyverPID.h>

#include "Defines.h"

//#define DEBUG_ENG

enum State {
  OK,
  CONN_LOST
};

class WrapEng
{
  public:
    uint16_t POWER_MAIN = MIN_POWER;
    uint16_t POWER_Diag_FRRL = POWER_MAIN;
    uint16_t POWER_Diag_FLRR = POWER_MAIN;

    uint16_t POWER_FR = MIN_POWER;
    uint16_t POWER_FL = MIN_POWER;
    uint16_t POWER_RR = MIN_POWER;
    uint16_t POWER_RL = MIN_POWER;

    // IS MAX
    bool isMaxReached = false;
    uint8_t numWarnEngine = 0;

    void init();
    void setGyroData(float ax_x, float ax_y, float ax_z);
    void analyzeCommand(uint8_t* msg_data, bool isConnLost, uint32_t ms);
    void stabAndExec(uint32_t ms);

  private:
    State state = State::CONN_LOST;
    const uint16_t POWER_FULL_DIFF = MAX_POWER - MIN_POWER;

    uint32_t prevApplyMs = 0;
    uint32_t prevCmndMs = 0;

    GyverPID regulator_FR_RL;
    GyverPID regulator_FL_RR;
    GyverPID regulator_D1_D2;

    Servo motorFR; // 1
    Servo motorFL; // 2
    Servo motorRR; // 3
    Servo motorRL; // 4

#ifdef DEBUG_ENG
    uint16_t counter = 0;
#endif

    uint8_t counPowerDown = 0;

    uint16_t* powers[4];
    void checkMinMax();
    void checkWarning();

    void flyOk(uint8_t* msg_data);
    void flyConnLost();
};

#endif // WRAPENG_H
