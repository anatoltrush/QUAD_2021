#include "Flash.hpp"
#include "Voltage.hpp"
#include "WrapRadio.hpp"
#include "WrapEngine.hpp"
#include "WrapGyro.hpp"
#include "WrapPID.hpp"

Flasher flasher(PIN_FLASH, TIME_FLASH_MS, TIME_FLASH_MS);
Voltage volt(PIN_VOLT, TIME_VOLT_MS);

WrapRadio wrapradio;

// GYRO
WrapGyro wrapgyro;

// PID
WrapPID wrappid;

// ENGINES
WrapEngine wrapengine;

uint32_t curr_time = 0;
uint8_t data_cntrl[SIZE_OF_DATA] = {0, 0, 0, 0, 0, 0};

uint16_t POWER_MAIN = MIN_POWER;
uint16_t POWER_IN_DiagRL = POWER_MAIN;
uint16_t POWER_IN_DiagLR = POWER_MAIN;

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
  if (wrapradio.radio->available(&wrapradio.pipeNum)) {
    wrapradio.radio->read(&data_cntrl, sizeof(data_cntrl));
    wrapradio.gotByte = volt.update() * 100;
    wrapradio.radio->writeAckPayload(wrapradio.pipeNum, &wrapradio.gotByte, 1); // îòïðàâëÿåì îáðàòíî òî ÷òî ïðèíÿëè
#ifdef DEBUG
    delay(100);
    Serial.print("DATA: ");
    Serial.println(data_cntrl[1]);
#endif // DEBUG
  }

  // GYRO
  //if (millis() - curr_time > TIME_GYRO_MS) {
  //  curr_time = millis();

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
      if (val >= 300 && val < MIN_POWER) { // D
        wrappid.regulator_FL_RR.Kd = wrappid.regulator_FR_RL.Kd
                                     = (float)((val - 300.0f) / 100.0f);
      }
      if (val >= 800 && val < MAX_POWER) { // POWER
        POWER_IN_DiagRL = val;
      }
    }

    float smoothed_x = 0.0f, smoothed_y = 0.0f;
    wrapgyro.getSmoothResultTimer(smoothed_x, smoothed_y, TIME_GYRO_MS);
    wrappid.regulator_FR_RL.input = smoothed_x; // ВХОД регулятора угол X
    wrappid.regulator_FL_RR.input = smoothed_y;// ВХОД регулятора угол Y

    Serial.print(smoothed_x);
    Serial.print(',');

    float real_x = 0.0f, real_y = 0.0f;
    wrapgyro.getRealResultTimer(real_x, real_y, TIME_GYRO_MS);
    Serial.print(real_x);
    Serial.print(',');

    // PID DIAGONAL 1
    uint16_t pid_out_FR_RL = (uint16_t)wrappid.regulator_FR_RL.getResultTimer();
    wrapengine.POWER_FR = POWER_IN_DiagRL - pid_out_FR_RL;
    wrapengine.POWER_RL = POWER_IN_DiagRL + pid_out_FR_RL;

    // PID DIAGONAL 2
    uint16_t pid_out_FL_RR = (uint16_t)wrappid.regulator_FL_RR.getResultTimer();
    wrapengine.POWER_FL = POWER_IN_DiagLR - pid_out_FL_RR;
    wrapengine.POWER_RR = POWER_IN_DiagLR + pid_out_FL_RR;

    // PID D1 D2

    Serial.print(wrapengine.POWER_FR);
    Serial.print(',');
    Serial.println(wrapengine.POWER_RL);
  //}

  // MOVER
  wrapengine.apply(5);

  flasher.update();
}
// 108 208 302
