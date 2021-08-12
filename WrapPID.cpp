#include "WrapPID.hpp"

void WrapPID::init()
{
  POWER_FULL_DIFF = MAX_POWER - MIN_POWER;

  regulator_FR_RL.setDirection(NORMAL);
  // regulator_FR_RL.setLimits(-(MAX_POWER - MIN_POWER), MAX_POWER - MIN_POWER);
  regulator_FR_RL.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF), POWER_FULL_DIFF * PID_LIM_COEFF);
  regulator_FR_RL.setDt(TIME_PID_MS);
  regulator_FR_RL.setpoint = 0;  // УСТАНОВКА УГЛА

  regulator_FL_RR.setDirection(NORMAL);
  // regulator_FL_RR.setLimits(-(MAX_POWER - MIN_POWER), MAX_POWER - MIN_POWER);
  regulator_FR_RL.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF), POWER_FULL_DIFF * PID_LIM_COEFF);
  regulator_FL_RR.setDt(TIME_PID_MS);
  regulator_FL_RR.setpoint = 0;
}
