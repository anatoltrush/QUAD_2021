#include <GyverPID.h>

#include "Flash.hpp"
#include "WrapRadio.hpp"
#include "WrapEngine.hpp"
#include "WrapGyro.hpp"

WrapRadio wrapradio;

#define TIME_PID 25
uint64_t curr_time = 0;

Flasher flasher(PIN_FLASH, TIME_FLASH_MS, TIME_FLASH_MS);

// GYRO
WrapGyro wrapgyro;

// PID
GyverPID regulator_FR_RL;
GyverPID regulator_FL_RR;
GyverPID regulator_D1_D2;

// ENGINES
WrapEngine wrapengine;
int POWER_IN = MIN_POWER;

void setup() {
  // PID
  regulator_FR_RL.setDirection(NORMAL);
  regulator_FR_RL.setLimits(-(MAX_POWER - MIN_POWER), MAX_POWER - MIN_POWER);
  regulator_FR_RL.setDt(TIME_PID);
  regulator_FR_RL.setpoint = 0;  // УСТАНОВКА УГЛА

  regulator_FL_RR.setDirection(NORMAL);
  regulator_FL_RR.setLimits(-(MAX_POWER - MIN_POWER), MAX_POWER - MIN_POWER);
  regulator_FL_RR.setDt(TIME_PID);
  regulator_FL_RR.setpoint = 0;  // УСТАНОВКА УГЛА

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
        regulator_FR_RL.Kp = (float)((val - 100.0f) / 10.0f);
      }
      if (val >= 200 && val < 300) { // I
        regulator_FR_RL.Ki = (float)((val - 200.0f) / 10.0f);
      }
      if (val >= 300 && val < 800) { // D
        regulator_FR_RL.Kd = (float)((val - 300.0f) / 10.0f);
      }
      if (val >= 800 && val < 2300) { // POWER
        POWER_IN = val;
      }
    }

    float smoothed_x = 0.0f, smoothed_y = 0.0f;
    wrapgyro.getSmoothResult(smoothed_x, smoothed_y);
    regulator_FR_RL.input = smoothed_x;     // ВХОД регулятора угол

    Serial.print(smoothed_x);
    Serial.print(',');

    /*Serial.print(new_val_x);
      Serial.print(" ");
      Serial.println(new_val_y);*/

    // PID
    int pid_out = (int)regulator_FR_RL.getResultTimer();
    wrapengine.POWER_RR = POWER_IN - pid_out;
    wrapengine.POWER_RL = POWER_IN + pid_out;

    Serial.print(wrapengine.POWER_RR);
    Serial.print(',');
    Serial.println(wrapengine.POWER_RL);
  }

  // MOVER
  wrapengine.apply();

  flasher.update();
}
// kp = 5...8, ki = 0.5...2.0, kd = 0.5
// kp = 1...2, ki = 1, kd = 0.1
