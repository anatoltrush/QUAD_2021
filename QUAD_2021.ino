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

  wrapradio.getData(extra.output * 100, wrapengine.isMaxReached, wrapengine.numWarnEngine);

  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    if (val >= MIN_POWER && val <= MAX_POWER) { // ---> POWER <---
      if (wrapengine.isMaxReached && val < wrapengine.POWER_IN_Diag_FRRL && val < wrapengine.POWER_IN_Diag_FLRR) {
        wrapengine.POWER_IN_Diag_FRRL = wrapengine.POWER_IN_Diag_FLRR = val;
      }
      else if (!wrapengine.isMaxReached) {
        wrapengine.POWER_IN_Diag_FRRL = wrapengine.POWER_IN_Diag_FLRR = val;
      }
    }
  }

  switch (wrapradio.data_msg[3]) { // move to WrapEngine
    case DATA_MIN:
      wrapengine.regulator_FR_RL.setpoint = OFFSET_FR_RL - SET_ANGLE;
      wrapengine.regulator_FL_RR.setpoint = OFFSET_FL_RR - SET_ANGLE;
      //Serial.print(wrapengine.regulator_FR_RL.setpoint);Serial.print("_");Serial.println(wrapengine.regulator_FL_RR.setpoint);
      break;
    case DATA_AVRG:
      wrapengine.regulator_FR_RL.setpoint = OFFSET_FR_RL;
      wrapengine.regulator_FL_RR.setpoint = OFFSET_FL_RR;
      //Serial.print(wrapengine.regulator_FR_RL.setpoint);Serial.print("_");Serial.println(wrapengine.regulator_FL_RR.setpoint);
      break;
    case DATA_MAX:
      wrapengine.regulator_FR_RL.setpoint = OFFSET_FR_RL + SET_ANGLE;
      wrapengine.regulator_FL_RR.setpoint = OFFSET_FL_RR + SET_ANGLE;
      //Serial.print(wrapengine.regulator_FR_RL.setpoint);Serial.print("_");Serial.println(wrapengine.regulator_FL_RR.setpoint);
      break;
    default :
      break;
  }

  // ---> GYRO <---
  wrapgyro.getRealResultTimer(TIME_GYRO_MS);
  wrapgyro.getSmoothResultTimer(TIME_GYRO_MS);

  wrapengine.regulator_FR_RL.input = wrapgyro.ax_x_sm; // Enter for DIAGONAL 1
  wrapengine.regulator_FL_RR.input = wrapgyro.ax_y_sm; // Enter for DIAGONAL 2

  uint16_t pid_FR_RL = (wrapengine.POWER_IN_Diag_FRRL >= MIN_DIAG_POWER) ?
                       (uint16_t)wrapengine.regulator_FR_RL.getResultTimer() : 0; // PID DIAGONAL 1
  uint16_t pid_FL_RR = (wrapengine.POWER_IN_Diag_FLRR >= MIN_DIAG_POWER) ?
                       (uint16_t)wrapengine.regulator_FL_RR.getResultTimer() : 0; // PID DIAGONAL 2

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
    Serial.print(wrapengine.POWER_FL);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RR);
      Serial.println();
  }
}
// down power with PIDs
