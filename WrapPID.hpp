#ifndef WRAPPID_HPP
#define WRAPPID_HPP

#include <Arduino.h>
#include <GyverPID.h>

#include "Defines.hpp"

class WrapPID
{
  public:
    GyverPID regulator_FR_RL;
    GyverPID regulator_FL_RR;
    GyverPID regulator_D1_D2;

    void init();
};
#endif // WRAPPID_HPP
