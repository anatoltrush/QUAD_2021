#include <Servo.h>
#include <MPU6050.h>
#include <GyverPID.h>

#include "Flash.hpp"
#define PIN_FLASH      8
#define TIME_FLASH_MS    500

// GYRO
#define TO_DEG 57.29577951308232087679815481410517033f
#define TIME_GYRO 50

// PID
#define TIME_PID 50

// MOVER
#define MIN_POWER 800
#define MAX_POWER 2300

#define FR_pin_out 4 // PIN_FR_SERV_OUT
#define FL_pin_out 6 // PIN_FL_SERV_OUT
#define RR_pin_out 3 // PIN_RR_SERV_OUT
#define RL_pin_out 5 // PIN_RL_SERV_OUT

Flasher flasher(PIN_FLASH, TIME_FLASH_MS, TIME_FLASH_MS);

// GYRO
MPU6050 accel;
float angle_ax = 0.0f, angle_ay = 0.0f;
float prev_ax = 0.0f, prev_ay = 0.0f;
float new_val_x = 0.0f, new_val_y = 0.0f;
float delta_x = 0.0f, delta_y = 0.0f;
uint64_t curr_time = 0;

// PID
GyverPID regulator;

// MOVER
Servo motorFR;
Servo motorFL;
Servo motorRR;
Servo motorRL;
int POWER = MIN_POWER;

void setup() {
  // GYRO
  accel.initialize(); // первичная настройка датчика
  accel.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  // PID
  regulator.setDirection(NORMAL);
  regulator.setLimits(-(MAX_POWER - MIN_POWER), MAX_POWER - MIN_POWER);
  regulator.setDt(TIME_PID);
  regulator.setpoint = 0;  // УСТАНОВКА регулятора, например необходимая температура

  // MOVER
  pinMode(FR_pin_out, OUTPUT);
  pinMode(FL_pin_out, OUTPUT);
  pinMode(RR_pin_out, OUTPUT);
  pinMode(RL_pin_out, OUTPUT);

  motorFR.attach(FR_pin_out);
  motorFL.attach(FL_pin_out);
  motorRR.attach(RR_pin_out);
  motorRL.attach(RL_pin_out);

  motorFR.writeMicroseconds(MAX_POWER);
  motorFL.writeMicroseconds(MAX_POWER);
  motorRR.writeMicroseconds(MAX_POWER);
  motorRL.writeMicroseconds(MAX_POWER);
  delay(2000);

  motorFR.writeMicroseconds(MIN_POWER);
  motorFL.writeMicroseconds(MIN_POWER);
  motorRR.writeMicroseconds(MIN_POWER);
  motorRL.writeMicroseconds(MIN_POWER);
  delay(2000);

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
        regulator.Kp = (float)((val - 100.0f) / 10.0f);
      }
      if (val >= 200 && val < 300) { // I
        regulator.Ki = (float)((val - 200.0f) / 10.0f);
      }
      if (val >= 300 && val < 800) { // D
        regulator.Kd = (float)((val - 300.0f) / 10.0f);
      }
      if (val >= 800 && val < 2300) { // POWER
        POWER = val;
      }
    }

    int16_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
    float ay = 0.0f, gx = 0.0f;
    float ax = 0.0f, gy = 0.0f;
    prev_ax = new_val_x;
    prev_ay = new_val_y;

    accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // сырые данные акселерометра нужно преобразовать в единицы гравитации
    // при базовых настройках 1G = 4096
    ay = ay_raw / 4096.0f;
    ax = ax_raw / 4096.0f;

    // функция acos возвращает угол в радианах, так что преобразуем
    // его в градусы при помощи коэффициента TO_DEG
    angle_ax = (ay >= 0) ? 90.0f - TO_DEG * acos(ay) : angle_ax = TO_DEG * acos(-ay) - 90.0f;
    angle_ay = (ax >= 0) ? 90.0f - TO_DEG * acos(ax) : angle_ay = TO_DEG * acos(-ax) - 90.0f;

    if (isnan(angle_ax) || isnan(angle_ay)) { // !!!
      Serial.println("NAN!");
      return;
    }

    delta_x = angle_ax - prev_ax;
    new_val_x = prev_ax + (delta_x * 0.3f); // config 0-low...1-full
    regulator.input = new_val_x;     // ВХОД регулятора угол

    Serial.print(new_val_x);
    Serial.print(',');

    delta_y = angle_ay - prev_ay;
    new_val_y = prev_ay + (delta_y * 0.3f); // config 0-low...1-full

    /*Serial.print(new_val_x);
      Serial.print(" ");
      Serial.println(new_val_y);*/

    // PID
    int pid_out = (int)regulator.getResultTimer();
    int RR = POWER + pid_out;
    int RL = POWER - pid_out;

    Serial.print(RR);
    Serial.print(',');
    Serial.println(RL);
  }

  // MOVER
  motorFR.writeMicroseconds(POWER);
  motorFL.writeMicroseconds(POWER);
  motorRR.writeMicroseconds(POWER);
  motorRL.writeMicroseconds(POWER);

  flasher.update();
}
// kp = 5...8, ki = 0.5...2.0, kd = 0.5
