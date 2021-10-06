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

  regulator_D1_D2.setDirection(NORMAL);
  regulator_D1_D2.setDt(TIME_PID_MS);
  regulator_D1_D2.Kp = 0.0f;

  powers[0] = &POWER_FR;
  powers[1] = &POWER_FL;
  powers[2] = &POWER_RR;
  powers[3] = &POWER_RL;
}

void WrapEng::setGyroData(float ax_x, float ax_y, float ax_z) {
  regulator_FR_RL.input = ax_x;
  regulator_FL_RR.input = ax_y;
  regulator_D1_D2.input = ax_z;
}

void WrapEng::analyzeCommand(uint8_t* msg_data, bool isConnLost, uint32_t ms) {
  if (millis() - prevCmndMs >= ms) {
#ifdef DEBUG_ENG
    Serial.print(millis() - prevCmndMs);
    Serial.print("_");
    Serial.print(counter);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevCmndMs = millis(); // запоминаем момент времени
    //_________________________
    if (isConnLost) state = State::CONN_LOST;
    else state = State::OK;
    // --- --- --- --- ---
    if (state == State::OK)
      flyOk(msg_data);
    if (state == State::CONN_LOST)
      flyConnLost();
  }
}

void WrapEng::execute(uint32_t ms) {
  if (millis() - prevApplyMs >= ms) {
#ifdef DEBUG_ENG
    Serial.print(millis() - prevApplyMs);
    Serial.print("_");
    Serial.print(counter);
    Serial.print("_");
    Serial.println(__func__);
    counter++;
#endif
    prevApplyMs = millis(); // запоминаем момент времени
    //_________________________
    // D1 <-o-> D2
    uint16_t pid_D1_D2 = 0;

    // Diag FR <-o-> RL
    uint16_t pid_FR_RL = 0;
    if (POWER_IN_Diag_FRRL >= MIN_DIAG_POWER) {
      pid_FR_RL = (uint16_t)regulator_FR_RL.getResultTimer();
    }
    else {
      pid_FR_RL = 0.0f;
      regulator_FR_RL.integral = 0.0f;
    }
    POWER_FR = POWER_IN_Diag_FRRL + pid_FR_RL;
    POWER_RL = POWER_IN_Diag_FRRL - pid_FR_RL;

    // Diag FL <-o-> RR
    uint16_t pid_FL_RR = 0;
    if (POWER_IN_Diag_FLRR >= MIN_DIAG_POWER) {
      pid_FL_RR = (uint16_t)regulator_FL_RR.getResultTimer();
    }
    else {
      pid_FL_RR = 0.0f;
      regulator_FL_RR.integral = 0.0f;
    }
    POWER_FL = POWER_IN_Diag_FLRR - pid_FL_RR;
    POWER_RR = POWER_IN_Diag_FLRR + pid_FL_RR;

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
  isMaxReached = (maxPower > WARN_POWER) ? true : false;
}

void WrapEng::flyOk(uint8_t* msg_data) {
  // ---------- [1] THROTTLE ----------
  if (msg_data[BT_MSG_THR] == DATA_MAX) {
    if (!isMaxReached || POWER_IN_MAIN < MAX_POWER)
      POWER_IN_MAIN += THR_ADD_POWER;
  }
  if (msg_data[BT_MSG_THR] == DATA_MIN) {
    if (POWER_IN_MAIN > MIN_POWER)
      POWER_IN_MAIN -= THR_SUB_POWER;
  }
  POWER_IN_Diag_FRRL = POWER_IN_MAIN;
  POWER_IN_Diag_FLRR = POWER_IN_MAIN;

  // ---------- [0] YAW ----------
  float resOffsetD1D2 = OFFSET_D1_D2;
  if (msg_data[BT_MSG_YAW] == DATA_MAX) {
    // implement
  }
  if (msg_data[BT_MSG_YAW] == DATA_MIN) {
    // implement
  }

  // ---------- [3] ROLL + [4] PITCH ----------
  float resOffsetFRRL = OFFSET_FR_RL;
  float resOffsetFLRR = OFFSET_FL_RR;
  if (msg_data[BT_MSG_PTCH] == DATA_MAX && msg_data[BT_MSG_ROLL] == DATA_MAX) { // D1+ (FRRL)
    resOffsetFRRL += SET_ANGLE;
  }
  if (msg_data[BT_MSG_PTCH] == DATA_MIN && msg_data[BT_MSG_ROLL] == DATA_MIN) { // D1- (FRRL)
    resOffsetFRRL -= SET_ANGLE;
  }

  if (msg_data[BT_MSG_PTCH] == DATA_MAX && msg_data[BT_MSG_ROLL] == DATA_MIN) { // D2+ (FLRR)
    resOffsetFLRR += SET_ANGLE;
  }
  if (msg_data[BT_MSG_PTCH] == DATA_MIN && msg_data[BT_MSG_ROLL] == DATA_MAX) { // D2- (FLRR)
    resOffsetFLRR -= SET_ANGLE;
  }

  if (msg_data[BT_MSG_PTCH] == DATA_AVRG && msg_data[BT_MSG_ROLL] == DATA_MAX) { // ROLL+
    resOffsetFRRL += SET_ANGLE;
    resOffsetFLRR += SET_ANGLE;
  }
  if (msg_data[BT_MSG_PTCH] == DATA_AVRG && msg_data[BT_MSG_ROLL] == DATA_MIN) { // ROLL-
    resOffsetFRRL -= SET_ANGLE;
    resOffsetFLRR -= SET_ANGLE;
  }

  if (msg_data[BT_MSG_PTCH] == DATA_MAX && msg_data[BT_MSG_ROLL] == DATA_AVRG) { // PITCH+
    resOffsetFRRL += SET_ANGLE;
    resOffsetFLRR -= SET_ANGLE;
  }
  if (msg_data[BT_MSG_PTCH] == DATA_MIN && msg_data[BT_MSG_ROLL] == DATA_AVRG) { // PITCH-
    resOffsetFRRL -= SET_ANGLE;
    resOffsetFLRR += SET_ANGLE;
  }

  // ----- apply offsets -----
  regulator_FR_RL.setpoint = resOffsetFRRL;
  regulator_FL_RR.setpoint = resOffsetFLRR;

  // CUSTOM COMMANDS
  if (msg_data[BT_MSG_THR] == DATA_MIN && msg_data[BT_MSG_AUX2] == DATA_MAX) { // FULL DOWN
    POWER_IN_MAIN = MIN_POWER;
  }
}

void WrapEng::flyConnLost() {
  counterDown++;
  if (counterDown >= EPOC_FOR_DOWN) {
    counterDown = 0;
    if (POWER_IN_MAIN > MIN_POWER)
      POWER_IN_MAIN -= THR_SUB_POWER;
  }
}
