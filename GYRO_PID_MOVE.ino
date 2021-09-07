#include "Wire.h"
#include "Extra.hpp"
#include "WrapRadio.hpp"
#include "WrapEngine.hpp"
#include "WrapGyroDMP.hpp"
#
Extra extra(PIN_FLASH, PIN_VOLT);
WrapRadio wrapradio;
WrapGyroDMP wrapgyroDMP;
WrapEngine wrapengine;

uint32_t curr_time = 0;
uint8_t data_cntrl[SIZE_OF_DATA] = {0, 0, 0, 0, 0, 0};

void setup() {
  Wire.begin();
  Wire.setClock(400000ul);

  wrapgyroDMP.init();
  wrapengine.init();
  wrapradio.init();

  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("AXIS_X_sm, AXIS_X_rl, OUTPUT_R, OUTPUT_L");
}

void loop() {
  extra.flash(TIME_FLASH_MS); // heart beat
  extra.get_volt(TIME_VOLT_MS);

  if (wrapradio.radio->available(&wrapradio.pipeNum)) {
    wrapradio.radio->read(&data_cntrl, sizeof(data_cntrl));
    wrapradio.ack_msg = extra.output * 100;
    wrapradio.radio->writeAckPayload(wrapradio.pipeNum, &wrapradio.ack_msg, SIZE_OF_ACK);
  }

  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    if (val >= 100 && val < 200) { // ---> P <---
      wrapengine.regulator_FL_RR.Kp =
        wrapengine.regulator_FR_RL.Kp =
          (float)((val - 100.0f) /*/ 10.0f*/);
    }
    if (val >= 200 && val < 300) { // ---> I <---
      wrapengine.regulator_FL_RR.Ki =
        wrapengine.regulator_FR_RL.Ki =
          (float)((val - 200.0f) /*/ 10.0f*/);
    }
    if (val >= 300 && val < 400) { // ---> D <---
      wrapengine.regulator_FL_RR.Kd =
        wrapengine.regulator_FR_RL.Kd =
          (float)((val - 300.0f) / 100.0f);
    }
    if (val >= MIN_POWER && val <= MAX_POWER) { // ---> POWER <---
      wrapengine.POWER_IN_Diag_FRRL =
        wrapengine.POWER_IN_Diag_FLRR =
          val;
    }
  }

  // ---> GYRO <---
  wrapgyroDMP.getRealResultTimer(TIME_GYRO_MS);
  wrapgyroDMP.getSmoothResultTimer(TIME_GYRO_MS);

  wrapengine.regulator_FR_RL.input = wrapgyroDMP.ax_x_sm; // ВХОД регулятора угол X
  wrapengine.regulator_FL_RR.input = wrapgyroDMP.ax_y_sm; // ВХОД регулятора угол Y
  //wrapengine.regulator_FR_RL.input = wrapgyroDMP.ax_x_rl; // ВХОД регулятора угол X
  //wrapengine.regulator_FL_RR.input = wrapgyroDMP.ax_y_rl; // ВХОД регулятора угол Y

  uint16_t pid_FR_RL = (uint16_t)wrapengine.regulator_FR_RL.getResultTimer(); // PID DIAGONAL 1
  uint16_t pid_FL_RR = (uint16_t)wrapengine.regulator_FL_RR.getResultTimer(); // PID DIAGONAL 2

  wrapengine.apply(pid_FR_RL, pid_FL_RR, TIME_ENGINE_MS);

  if (millis() - curr_time >= TIME_PID_MS) {
    curr_time = millis();

    /*Serial.print(wrapgyroDMP.ax_x_sm);
      Serial.print(',');
      Serial.print(wrapgyroDMP.ax_x_rl);
      Serial.print(',');*/
    Serial.print(wrapgyroDMP.ax_y_sm);
    Serial.print(',');
    Serial.print(wrapgyroDMP.ax_y_rl);
    Serial.print(',');

    /*Serial.print(wrapengine.POWER_FR);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RL);*/
    Serial.print(wrapengine.POWER_FL);
    Serial.print(',');
    Serial.print(wrapengine.POWER_RR);
    Serial.println();
  }
}
// TODO: 102 202 350 in code
