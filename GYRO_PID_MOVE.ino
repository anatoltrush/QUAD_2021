#include "Flash.hpp"
#include "WrapRadio.hpp"
#include "WrapEngine.hpp"
#include "WrapGyro.hpp"
#include "WrapPID.hpp"

uint64_t curr_time = 0;

WrapRadio wrapradio;

Flasher flasher(PIN_FLASH, TIME_FLASH_MS, TIME_FLASH_MS);

// GYRO
WrapGyro wrapgyro;

// PID
WrapPID wrappid;

// ENGINES
WrapEngine wrapengine;
int POWER_IN = MIN_POWER;

void setup() {
  wrapengine.init();
  wrapgyro.init();
  wrappid.init();
  wrapradio.init();

  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("AXIS_X, OUTPUT_R, OUTPUT_L");
}

void loop() {
  // GYRO
  if (millis() - curr_time > TIME_GYRO) {
    curr_time = millis();

    if (Serial.available() > 0) {
      int val = Serial.parseInt();
      if (val > 0 && val < 200) { // P
        wrappid.regulator_FR_RL.Kp = (float)((val - 100.0f) / 10.0f);
      }
      if (val >= 200 && val < 300) { // I
        wrappid.regulator_FR_RL.Ki = (float)((val - 200.0f) / 10.0f);
      }
      if (val >= 300 && val < 800) { // D
        wrappid.regulator_FR_RL.Kd = (float)((val - 300.0f) / 10.0f);
      }
      if (val >= 800 && val < 2300) { // POWER
        POWER_IN = val;
      }
    }

    float smoothed_x = 0.0f, smoothed_y = 0.0f;
    wrapgyro.getSmoothResult(smoothed_x, smoothed_y, TIME_GYRO);
    wrappid.regulator_FR_RL.input = smoothed_x;     // ВХОД регулятора угол

    Serial.print(smoothed_x);
    Serial.print(',');

    // PID
    int pid_out = (int)wrappid.regulator_FR_RL.getResultTimer();
    wrapengine.POWER_FL = POWER_IN - pid_out;
    wrapengine.POWER_RR = POWER_IN + pid_out;

    //wrapengine.POWER_RL = POWER_IN ;
    //wrapengine.POWER_FR = POWER_IN ;
    //wrapengine.POWER_RR = POWER_IN ;

    Serial.print(wrapengine.POWER_RR);
    Serial.print(',');
    Serial.println(wrapengine.POWER_FL);
  }

  // MOVER
  wrapengine.apply(10);

  flasher.update();
}
// kp = 5...8, ki = 0.5...2.0, kd = 0.5
// kp = 110, ki = 215, kd = 301
