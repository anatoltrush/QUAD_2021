#include "Wire.h"
#include "WrapRadio.h"
#include "WrapEng.h"
#include "WrapGyro.h"

Extra extra;
WrapRadio wrapradio;
WrapGyro wrapgyro;
WrapEng wrapengine;

uint32_t curr_time = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000ul);

  wrapengine.init();
  wrapgyro.init();
  wrapradio.init();

  Serial.begin(115200);
  Serial.println("Y_real");
}

void loop() {
  extra.flash(TIME_FLASH_MS); // heart beat
  extra.getVoltQuad(TIME_VOLT_MS, wrapradio.ack_msg);
  extra.customCommand(wrapradio.data_msg, TIME_CMD_UPD_MS);

  wrapgyro.getRealResultTimer(TIME_GYRO_MS);
  wrapradio.getData(extra.voltOutput * 10, wrapengine.isMaxReached, wrapengine.numWarnEngine, wrapengine.POWER_MAIN);
  wrapengine.setGyroData(wrapgyro.ax_x_rl, wrapgyro.ax_y_rl, wrapgyro.ax_z_rl);
  wrapengine.analyzeCommand(wrapradio.data_msg, wrapradio.isConnLost, TIME_CMD_UPD_MS);
  wrapengine.stabAndExec(TIME_ENGINE_MS);

  if (millis() - curr_time >= TIME_PID_MS) {
    curr_time = millis();
    /*Serial.print(wrapengine.POWER_FR);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RL);
      Serial.print(',');*/
    /*Serial.print(wrapgyro.ax_x_rl);
      Serial.print(',');*/
    Serial.print(wrapgyro.ax_y_rl);
    /*Serial.print(',');
      Serial.print(wrapengine.POWER_FL);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RR);*/
    /*Serial.print(wrapengine.POWER_Diag_FRRL);
      Serial.print(',');
      Serial.print(wrapengine.POWER_Diag_FLRR);*/
    Serial.println();
  }
}
