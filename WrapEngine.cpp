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

  regulator_FR_RL.setDirection(NORMAL);
  regulator_FR_RL.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF), POWER_FULL_DIFF * PID_LIM_COEFF);
  regulator_FR_RL.setDt(TIME_PID_MS);
  regulator_FR_RL.setpoint = 0;  // УСТАНОВКА УГЛА

  regulator_FL_RR.setDirection(NORMAL);
  regulator_FL_RR.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF), POWER_FULL_DIFF * PID_LIM_COEFF);
  regulator_FL_RR.setDt(TIME_PID_MS);
  regulator_FL_RR.setpoint = 0;
}

void WrapEngine::apply(uint16_t pid_FR_RL, uint16_t pid_FL_RR, uint32_t ms) {
  if (millis() - prev_millis >= ms) {
#ifdef DEBUG_ENGINE
    Serial.print(millis() - prev_millis);
    Serial.print("_");
    Serial.print(counter);
    Serial.print("_");
    Serial.println(__func__);
    counter++;
#endif
    prev_millis = millis(); // запоминаем момент времени
    //_________________________
    if (POWER_IN_Diag_FRRL >= MIN_DIAG_POWER) {
      POWER_FR = POWER_IN_Diag_FRRL + pid_FR_RL;
      POWER_RL = POWER_IN_Diag_FRRL - pid_FR_RL;
    }
    else {
      POWER_FR = POWER_IN_Diag_FRRL;
      POWER_RL = POWER_IN_Diag_FRRL;
    }

    if (POWER_IN_Diag_FLRR >= MIN_DIAG_POWER) {
      POWER_FL = POWER_IN_Diag_FLRR - pid_FL_RR;
      POWER_RR = POWER_IN_Diag_FLRR + pid_FL_RR;
    }
    else {
      POWER_FL = POWER_IN_Diag_FLRR;
      POWER_RR = POWER_IN_Diag_FLRR;
    }

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
