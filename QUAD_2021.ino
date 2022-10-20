#include "Wire.h"
#include "WrapRadio.h"
#include "WrapEng.h"
#include "WrapGyro.h"

Extra extra;
WrapRadio wrapRadio;
WrapGyro wrapGyro;
WrapEng wrapEngine;

uint32_t curr_time = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000ul);

  wrapEngine.init();
  wrapGyro.init();
  wrapRadio.init();

  Serial.begin(115200);
  Serial.println("X_real");
}

void loop() {
  extra.flash(TIME_FLASH_MS); // heart beat
  extra.getVoltQuad(TIME_VOLT_MS, wrapRadio.ack_msg);
  extra.customCommand(wrapRadio.data_msg, TIME_CMD_UPD_MS);

  wrapGyro.getRealResultTimer(TIME_GYRO_MS);

  wrapRadio.getData(extra.voltOutput * 10, wrapEngine.isMaxReached, wrapEngine.numWarnEngine, wrapEngine.POWER_MAIN);

  wrapEngine.setGyroData(wrapGyro.ax_x_rl, wrapGyro.ax_y_rl, wrapGyro.ax_z_rl);
  wrapEngine.setData(wrapRadio.data_msg, wrapRadio.isConnLost, TIME_CMD_UPD_MS);
  wrapEngine.stabAndExec(TIME_ENGINE_MS);

  if (millis() - curr_time >= TIME_PID_MS) {
    curr_time = millis();
    /*Serial.print(wrapengine.POWER_FR);
      Serial.print(',');
      Serial.print(wrapengine.POWER_RL);
      Serial.print(',');*/
    Serial.print(wrapGyro.ax_x_rl);
    //Serial.print(',');
    //Serial.print(wrapGyro.ax_y_rl);
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
