#ifndef WRAPPID_HPP
#define WRAPPID_HPP

#include <Arduino.h>
#include <GyverPID.h>

#include "Defines.hpp"

#define DEBUG_PID

class WrapPID
{
  public:
    WrapPID();

    GyverPID regulator_FR_RL;
    GyverPID regulator_FL_RR;
    GyverPID regulator_D1_D2;

  private:
    uint16_t POWER_FULL_DIFF = 0;
};
#endif // WRAPPID_HPP
