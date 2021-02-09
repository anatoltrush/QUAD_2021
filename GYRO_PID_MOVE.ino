#include "Flash.hpp"
#include "WrapRadio.hpp"
#include "WrapEngine.hpp"
#include "WrapGyro.hpp"
#include "WrapPID.hpp"

#define SIZE_OF_DATA 6

uint64_t curr_time = 0;
uint8_t data_cntrl[SIZE_OF_DATA] = {0, 0, 0, 0 , 0 , 0};

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
  Serial.println("AXIS_X_sm, AXIS_X_rl, OUTPUT_R, OUTPUT_L");
}

void loop() {
  if (wrapradio.radio->available(&wrapradio.pipeNo)) {
    wrapradio.radio->read(&data_cntrl, sizeof(data_cntrl));
    //wrapradio.gotByte = volt.update() * 10;
    wrapradio.radio->writeAckPayload(wrapradio.pipeNo, &wrapradio.gotByte, 1); // îòïðàâëÿåì îáðàòíî òî ÷òî ïðèíÿëè
#ifdef DEBUG
    delay(100);
    Serial.print("DATA: ");
    Serial.println(data_cntrl[1]);
#endif // DEBUG
  }

  // GYRO
  if (millis() - curr_time > TIME_GYRO) {
    curr_time = millis();

    if (Serial.available() > 0) {
      int val = Serial.parseInt();
      if (val > 0 && val < 200) { // P
        wrappid.regulator_FL_RR.Kp = wrappid.regulator_FR_RL.Kp
                                     = (float)((val - 100.0f) / 10.0f);
      }
      if (val >= 200 && val < 300) { // I
        wrappid.regulator_FL_RR.Ki = wrappid.regulator_FR_RL.Ki
                                     = (float)((val - 200.0f) / 10.0f);
      }
      if (val >= 300 && val < 800) { // D
        wrappid.regulator_FL_RR.Kd = wrappid.regulator_FR_RL.Kd
                                     = (float)((val - 300.0f) / 100.0f);
      }
      if (val >= 800 && val < 2300) { // POWER
        POWER_IN = val;
      }
    }
    /*wrappid.regulator_FR_RL.Kp = 1.2f;
      wrappid.regulator_FR_RL.Ki = 1.5f;
      wrappid.regulator_FR_RL.Kd = 0.2f;
      POWER_IN = 950;*/

    float smoothed_x = 0.0f, smoothed_y = 0.0f;
    wrapgyro.getSmoothResult(smoothed_x, smoothed_y, TIME_GYRO);
    wrappid.regulator_FR_RL.input = smoothed_x; // ВХОД регулятора угол X
    wrappid.regulator_FL_RR.input = smoothed_y;// ВХОД регулятора угол Y

    Serial.print(smoothed_x);
    Serial.print(',');

    float real_x = 0.0f, real_y = 0.0f;
    wrapgyro.getRealResult(real_x, real_y, TIME_GYRO);
    Serial.print(real_x);
    Serial.print(',');

    // PID
    int pid_out_FR_RL = (int)wrappid.regulator_FR_RL.getResultTimer();
    wrapengine.POWER_FR = POWER_IN - pid_out_FR_RL;
    wrapengine.POWER_RL = POWER_IN + pid_out_FR_RL;

    int pid_out_FL_RR = (int)wrappid.regulator_FL_RR.getResultTimer();
    wrapengine.POWER_FL = POWER_IN - pid_out_FL_RR;
    wrapengine.POWER_RR = POWER_IN + pid_out_FL_RR;

    Serial.print(wrapengine.POWER_FR);
    Serial.print(',');
    Serial.println(wrapengine.POWER_RL);
  }

  // MOVER
  wrapengine.apply(5);

  flasher.update();
}
// kp = 5...8, ki = 0.5...2.0, kd = 0.5
// kp = 105, ki = 215, kd = 301?
// TODO: add Voltage
