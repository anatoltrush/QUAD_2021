#include "WrapEngine.hpp"

void WrapEngine::init() {
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_FL, OUTPUT);
  pinMode(PIN_RR, OUTPUT);
  pinMode(PIN_RL, OUTPUT);

  motorFR.attach(PIN_FR);
  motorFL.attach(PIN_FL);
  motorRR.attach(PIN_RR);
  motorRL.attach(PIN_RL);

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

void WrapEngine::apply(uint32_t ms)
{
  if (millis() - _prev_millis > ms) {
    _prev_millis = millis(); // запоминаем момент времени

    // Diag FR <-o-> RL
    if (POWER_FR < MIN_POWER) POWER_FR = MIN_POWER;
    if (POWER_FR > MAX_POWER) POWER_FR = MAX_POWER;
    motorFR.writeMicroseconds(POWER_FR);

    if (POWER_RL < MIN_POWER) POWER_RL = MIN_POWER;
    if (POWER_RL > MAX_POWER) POWER_RL = MAX_POWER;
    motorRL.writeMicroseconds(POWER_RL);

    // Diag FL <-o-> RR
    if (POWER_FL < MIN_POWER) POWER_FL = MIN_POWER;
    if (POWER_FL > MAX_POWER) POWER_FL = MAX_POWER;
    motorFL.writeMicroseconds(POWER_FL);

    if (POWER_RR < MIN_POWER) POWER_RR = MIN_POWER;
    if (POWER_RR > MAX_POWER) POWER_RR = MAX_POWER;
    motorRR.writeMicroseconds(POWER_RR);
  }
}
