#include "Wire.h"
#include "Extra.h"
#include "WrapRadio.h"
#include "WrapEng.h"
#include "WrapGyro.h"

Extra extra(PIN_FLASH, PIN_VOLT, PIN_AUX_1, PIN_AUX_2);
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
  Serial.println("AXIS_Z, OUTPUT_R, OUTPUT_L");
}

void loop() {
  extra.flash(TIME_FLASH_MS); // heart beat
  extra.getVoltQuad(TIME_VOLT_MS);

  wrapgyro.getRealResultTimer(TIME_GYRO_MS);
  wrapgyro.getSmoothResultTimer(TIME_GYRO_MS);

  wrapengine.regulator_D1_D2.input = wrapgyro.ax_z_sm;// Enter for D1 & D2
  wrapengine.regulator_FR_RL.input = wrapgyro.ax_x_sm; // Enter for DIAGONAL 1
  wrapengine.regulator_FL_RR.input = wrapgyro.ax_y_sm; // Enter for DIAGONAL 2

  wrapradio.getData(extra.voltOutput * 10, wrapengine.isMaxReached, wrapengine.numWarnEngine);
  wrapengine.analyzeCommand(wrapradio.data_msg, TIME_CMD_UPD_MS);
  extra.customCommand(wrapradio.data_msg, TIME_CMD_UPD_MS);

  wrapengine.execute(TIME_ENGINE_MS);

  if (millis() - curr_time >= TIME_PID_MS) {
    curr_time = millis();
    /*Serial.print(wrapengine.POWER_FR);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RL);*/
    Serial.print(wrapgyro.ax_z_rl);
    Serial.print(',');
    Serial.print(wrapengine.POWER_FL);
    Serial.print(',');
    Serial.print(wrapengine.POWER_RR);
    Serial.println();
  }
}
