#include "WrapEngine.hpp"

WrapEngine::WrapEngine()
{
  pinMode(FR_pin_out, OUTPUT);
  pinMode(FL_pin_out, OUTPUT);
  pinMode(RR_pin_out, OUTPUT);
  pinMode(RL_pin_out, OUTPUT);

  motorFR.attach(FR_pin_out);
  motorFL.attach(FL_pin_out);
  motorRR.attach(RR_pin_out);
  motorRL.attach(RL_pin_out);

  motorFR.writeMicroseconds(MAX_POWER);
  motorFL.writeMicroseconds(MAX_POWER);
  motorRR.writeMicroseconds(MAX_POWER);
  motorRL.writeMicroseconds(MAX_POWER);
  delay(2000);

  motorFR.writeMicroseconds(MIN_POWER);
  motorFL.writeMicroseconds(MIN_POWER);
  motorRR.writeMicroseconds(MIN_POWER);
  motorRL.writeMicroseconds(MIN_POWER);
  delay(2000);
}

void WrapEngine::apply()
{
  motorFR.writeMicroseconds(POWER_FR);
  motorFL.writeMicroseconds(POWER_FL);
  motorRR.writeMicroseconds(POWER_RR);
  motorRL.writeMicroseconds(POWER_RL);
}
