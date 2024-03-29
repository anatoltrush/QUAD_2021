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
  regulator_FR_RL.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF_XY),
                            POWER_FULL_DIFF * PID_LIM_COEFF_XY);
  regulator_FR_RL.setDt(TIME_PID_MS);
  regulator_FR_RL.setpoint = 0.0f;
  regulator_FR_RL.Kp = PID_KP_XY;
  regulator_FR_RL.Ki = PID_KI_XY;
  regulator_FR_RL.Kd = PID_KD_XY;

  regulator_FL_RR.setDirection(NORMAL);
  regulator_FL_RR.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF_XY),
                            POWER_FULL_DIFF * PID_LIM_COEFF_XY);
  regulator_FL_RR.setDt(TIME_PID_MS);
  regulator_FL_RR.setpoint = 0.0f;
  regulator_FL_RR.Kp = PID_KP_XY;
  regulator_FL_RR.Ki = PID_KI_XY;
  regulator_FL_RR.Kd = PID_KD_XY;

  regulator_D1_D2.setDirection(NORMAL);
  regulator_D1_D2.setLimits(-(POWER_FULL_DIFF * PID_LIM_COEFF_Z),
                            POWER_FULL_DIFF * PID_LIM_COEFF_Z);
  regulator_D1_D2.setDt(TIME_PID_MS);
  regulator_D1_D2.setpoint = 0.0f;
  regulator_D1_D2.Kp = 0.0f;
  regulator_D1_D2.Ki = PID_KI_Z;
  regulator_D1_D2.Kd = PID_KD_Z;

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

void WrapEng::setData(uint8_t* msg_data, bool isConnLost, uint32_t ms) {
  if (millis() - prevCmndMs >= ms) {
#ifdef DEBUG_ENG
    Serial.print(millis() - prevCmndMs);
    Serial.print("_");
    Serial.print(counter);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevCmndMs = millis();  // запоминаем момент времени
    //_________________________
    if (isConnLost) state = State::CONN_LOST;
    else state = State::OK;
    // --- --- --- --- ---
    if (state == State::OK) analyzeCommands(msg_data);
    if (state == State::CONN_LOST) connectionLost();
  }
}

void WrapEng::stabAndExec(uint32_t ms) {
  if (millis() - prevApplyMs >= ms) {
#ifdef DEBUG_ENG
    Serial.print(millis() - prevApplyMs);
    Serial.print("_");
    Serial.print(counter);
    Serial.print("_");
    Serial.println(__func__);
    counter++;
#endif
    prevApplyMs = millis();  // запоминаем момент времени
    //_________________________
    // --- --- --- --- ---> D1 <-o-> D2 <--- --- --- --- ---
    uint16_t pid_D1_D2 = 0;
    if (POWER_MAIN >= MIN_UP_POWER) {
      pid_D1_D2 = regulator_D1_D2.getResult();
    } else {
      pid_D1_D2 = 0;
      regulator_D1_D2.integral = 0.0f;
    }
    POWER_Diag_FRRL = POWER_MAIN + pid_D1_D2;
    POWER_Diag_FLRR = POWER_MAIN - pid_D1_D2;

    // --- --- --- --- ---> Diag FR <-o-> RL <--- --- --- --- ---
    uint16_t pid_FR_RL = 0;
    if (POWER_Diag_FRRL >= MIN_UP_POWER) {
      pid_FR_RL = (uint16_t)regulator_FR_RL.getResult();
    } else {
      pid_FR_RL = 0;
      regulator_FR_RL.integral = 0.0f;
    }
    POWER_FR = POWER_Diag_FRRL + pid_FR_RL;
    POWER_RL = POWER_Diag_FRRL - pid_FR_RL;

    // --- --- --- --- ---> Diag FL <-o-> RR <--- --- --- --- ---
    uint16_t pid_FL_RR = 0;
    if (POWER_Diag_FLRR >= MIN_UP_POWER) {
      pid_FL_RR = (uint16_t)regulator_FL_RR.getResult();
    } else {
      pid_FL_RR = 0;
      regulator_FL_RR.integral = 0.0f;
    }
    POWER_FL = POWER_Diag_FLRR - pid_FL_RR;
    POWER_RR = POWER_Diag_FLRR + pid_FL_RR;

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
  isMaxReached = (maxPower > WARN_ENG_POWER) ? true : false;
}

void WrapEng::analyzeCommands(uint8_t* msgData) {
#ifdef DEBUG_ENG
  Serial.print(millis() - prevCmndMs);
  Serial.print("_");
  Serial.println(__func__);
#endif
  // ---------- [1] THROTTLE ----------
  if (msgData[BT_MSG_THR] == DATA_MAX) {
    if (!isMaxReached && POWER_MAIN < WARN_MAIN_POWER)
      POWER_MAIN += THR_ADD_POWER;
  }
  if (msgData[BT_MSG_THR] == DATA_MIN) {
    if (POWER_MAIN > MIN_POWER)
      POWER_MAIN -= THR_SUB_POWER;
  }

  // ---------- [0] YAW ----------
  if (msgData[BT_MSG_YAW] == DATA_MAX) {
    setpointD1D2 += SET_YAW_ANG;
  }
  if (msgData[BT_MSG_YAW] == DATA_MIN) {
    setpointD1D2 -= SET_YAW_ANG;
  }

  // ---------- [3] ROLL + [4] PITCH ----------
  float setpointFRRL = 0.0f;
  float setpointFLRR = 0.0f;

  if (msgData[BT_MSG_PTCH] == DATA_MAX && msgData[BT_MSG_ROLL] == DATA_MAX) {  // Diag1+ (FRRL)
    setpointFRRL += SET_P_R_ANG;
  }
  if (msgData[BT_MSG_PTCH] == DATA_MIN && msgData[BT_MSG_ROLL] == DATA_MIN) {  // Diag1- (FRRL)
    setpointFRRL -= SET_P_R_ANG;
  }

  if (msgData[BT_MSG_PTCH] == DATA_MAX && msgData[BT_MSG_ROLL] == DATA_MIN) {  // Diag2+ (FLRR)
    setpointFLRR += SET_P_R_ANG;
  }
  if (msgData[BT_MSG_PTCH] == DATA_MIN && msgData[BT_MSG_ROLL] == DATA_MAX) {  // Diag2- (FLRR)
    setpointFLRR -= SET_P_R_ANG;
  }

  if (msgData[BT_MSG_PTCH] == DATA_AVRG && msgData[BT_MSG_ROLL] == DATA_MAX) {  // ROLL+
    setpointFRRL += SET_P_R_ANG;
    setpointFLRR += SET_P_R_ANG;
  }
  if (msgData[BT_MSG_PTCH] == DATA_AVRG && msgData[BT_MSG_ROLL] == DATA_MIN) {  // ROLL-
    setpointFRRL -= SET_P_R_ANG;
    setpointFLRR -= SET_P_R_ANG;
  }

  if (msgData[BT_MSG_PTCH] == DATA_MAX && msgData[BT_MSG_ROLL] == DATA_AVRG) {  // PITCH+
    setpointFRRL += SET_P_R_ANG;
    setpointFLRR -= SET_P_R_ANG;
  }
  if (msgData[BT_MSG_PTCH] == DATA_MIN && msgData[BT_MSG_ROLL] == DATA_AVRG) {  // PITCH-
    setpointFRRL -= SET_P_R_ANG;
    setpointFLRR += SET_P_R_ANG;
  }

  // ----- apply offsets -----
  regulator_D1_D2.setpoint = setpointD1D2;
  regulator_FR_RL.setpoint = setpointFRRL;
  regulator_FL_RR.setpoint = setpointFLRR;

  // CUSTOM COMMANDS
  if (msgData[BT_MSG_THR] == DATA_MIN && msgData[BT_MSG_AUX2] == DATA_MAX) {  // FULL DOWN
    POWER_MAIN = MIN_POWER;
  }
}

void WrapEng::connectionLost() {
#ifdef DEBUG_ENG
  Serial.print(millis() - prevCmndMs);
  Serial.print("_");
  Serial.println(__func__);
#endif
  if (POWER_MAIN >= MIN_UP_POWER) {
    countPowerDown++;
    if (countPowerDown >= EPOC_FOR_DOWN) {
      POWER_MAIN -= THR_SUB_POWER;
      countPowerDown = 0;
    }
  } else {
    POWER_MAIN = MIN_POWER;
  }
}