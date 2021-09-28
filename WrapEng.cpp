#include "WrapEng.h"

void WrapEng::init() {
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
  regulator_FR_RL.setpoint = OFFSET_FR_RL;
  regulator_FR_RL.Kp = PID_KP;
  regulator_FR_RL.Ki = PID_KI;
  regulator_FR_RL.Kd = PID_KD;

  regulator_FL_RR.setDirection(NORMAL);
  regulator_FL_RR.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF), POWER_FULL_DIFF * PID_LIM_COEFF);
  regulator_FL_RR.setDt(TIME_PID_MS);
  regulator_FL_RR.setpoint = OFFSET_FL_RR;
  regulator_FL_RR.Kp = PID_KP;
  regulator_FL_RR.Ki = PID_KI;
  regulator_FL_RR.Kd = PID_KD;

  powers[0] = &POWER_FR;
  powers[1] = &POWER_FL;
  powers[2] = &POWER_RR;
  powers[3] = &POWER_RL;
}

void WrapEng::analyze(uint8_t* msg_data){
  // implement
}

void WrapEng::apply(uint16_t pid_FR_RL, uint16_t pid_FL_RR, uint32_t ms) {
  if (millis() - prev_millis >= ms) {
#ifdef DEBUG_ENG
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

    checkMinMax();
    checkWarning();

    // Diag FR <-o-> RL
    motorFR.writeMicroseconds(POWER_FR);
    motorRL.writeMicroseconds(POWER_RL);
    // Diag FL <-o-> RR
    motorFL.writeMicroseconds(POWER_FL);
    motorRR.writeMicroseconds(POWER_RR);
  }
}

void WrapEng::checkMinMax() {
  for (size_t i = 0; i < 4; i++) {
    if (*powers[i] < MIN_POWER) *powers[i] = MIN_POWER;
    if (*powers[i] > MAX_POWER) *powers[i] = MAX_POWER;
  }
}

void WrapEng::checkWarning() {
  uint16_t maxPower = 0;
  for (size_t i = 0; i < 4; i++) {
    if (*powers[i] > maxPower) {
      maxPower = *powers[i];
      numWarnEngine = i + 1;
    }
  }
  //___
  if (maxPower > WARN_POWER) {
    isMaxReached = true;
  }
  else {
    isMaxReached = false;
    numWarnEngine = 0;
  }
}
