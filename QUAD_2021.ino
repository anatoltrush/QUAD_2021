#include "Wire.h"
#include "Extra.h"
#include "WrapRadio.h"
#include "WrapEng.h"
#include "WrapGyro.h"

Extra extra(PIN_FLASH, PIN_VOLT);
WrapRadio wrapradio;
WrapGyro wrapgyro;
WrapEng wrapengine;

uint32_t curr_time = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000ul);

  wrapgyro.init();
  wrapengine.init();
  wrapradio.init();

  Serial.begin(115200);
  Serial.println("AXIS_X_sm, AXIS_X_rl, OUTPUT_R, OUTPUT_L");
}

void loop() {
  extra.flash(TIME_FLASH_MS); // heart beat
  extra.get_volt(TIME_VOLT_MS);

  if (wrapradio.radio->available(&wrapradio.pipeNum)) {
    wrapradio.radio->read(&wrapradio.data_msg, SIZE_OF_DATA);
    wrapradio.ack_msg[0] = extra.output * 100;
    //wrapradio.ack_msg[1] = wrapengine.isMaxReached;
    wrapradio.ack_msg[1] = true;
    wrapradio.ack_msg[2] = wrapengine.numWarnEngine;
    wrapradio.radio->writeAckPayload(wrapradio.pipeNum, wrapradio.ack_msg, SIZE_OF_ACK);
  }

  /*Serial.print(wrapradio.data_msg[0]); Serial.print("|");
  Serial.print(wrapradio.data_msg[1]); Serial.print("|");
  Serial.print(wrapradio.data_msg[2]); Serial.print("|");
  Serial.print(wrapradio.data_msg[3]); Serial.print("|");
  Serial.print(wrapradio.data_msg[4]); Serial.print("|");
  Serial.print(wrapradio.data_msg[5]); Serial.print("|");
  //Serial.print(millis());
  Serial.println();*/

  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    if (val >= MIN_POWER && val <= MAX_POWER) { // ---> POWER <---
      wrapengine.POWER_IN_Diag_FRRL = wrapengine.POWER_IN_Diag_FLRR = val;
    }
  }

  switch (wrapradio.data_msg[3]) {
    case DATA_MIN:
      wrapengine.regulator_FR_RL.setpoint = -SET_ANGLE;
      wrapengine.regulator_FL_RR.setpoint = -SET_ANGLE;
      break;
    case DATA_AVRG:
      wrapengine.regulator_FR_RL.setpoint = OFFSET_FR_RL;
      wrapengine.regulator_FL_RR.setpoint = OFFSET_FL_RR;
      break;
    case DATA_MAX:
      wrapengine.regulator_FR_RL.setpoint = SET_ANGLE;
      wrapengine.regulator_FL_RR.setpoint = SET_ANGLE;
      break;
    default :
      break;
  }

  // ---> GYRO <---
  wrapgyro.getRealResultTimer(TIME_GYRO_MS);
  wrapgyro.getSmoothResultTimer(TIME_GYRO_MS);

  wrapengine.regulator_FR_RL.input = wrapgyro.ax_x_sm; // ВХОД регулятора угол DIAGONAL 1
  wrapengine.regulator_FL_RR.input = wrapgyro.ax_y_sm; // ВХОД регулятора угол DIAGONAL 2

  uint16_t pid_FR_RL = (uint16_t)wrapengine.regulator_FR_RL.getResultTimer(); // PID DIAGONAL 1
  uint16_t pid_FL_RR = (uint16_t)wrapengine.regulator_FL_RR.getResultTimer(); // PID DIAGONAL 2

  wrapengine.apply(pid_FR_RL, pid_FL_RR, TIME_ENGINE_MS);

  if (millis() - curr_time >= TIME_PID_MS) {
    curr_time = millis();

    /*Serial.print(wrapgyro.ax_x_sm);
      Serial.print(',');
      Serial.print(wrapgyro.ax_x_rl);
      Serial.print(',');*/
    /*Serial.print(wrapgyro.ax_y_sm);
      Serial.print(',');
      Serial.print(wrapgyro.ax_y_rl);
      Serial.print(',');*/

    /*Serial.print(wrapengine.POWER_FR);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RL);*/
    /*Serial.print(wrapengine.POWER_FL);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RR);*/
    //Serial.println();
  }
}
