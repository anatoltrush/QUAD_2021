#include "WrapGyro.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

MPU6050 mpu;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

void WrapGyro::init() {
  mpu.initialize(); // первичная настройка датчика

  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
}

void WrapGyro::getRealResultTimer(uint32_t ms) {
  if (millis() - prevRealMs >= ms) {
#ifdef DEBUG_GYRO
    Serial.print(millis() - prevRealMs); Serial.print("_");
    Serial.print(counter_rl); Serial.print("_");
    Serial.println(__func__);
    counter_rl++;
#endif
    prevRealMs = millis(); // запоминаем момент времени
    //_________________________
    getData(ax_x_rl, ax_y_rl, ax_z_rl);
  }
}

void WrapGyro::getSmoothResultTimer(uint32_t ms) {
  if (millis() - prevSmoothMs >= ms) {
#ifdef DEBUG_GYRO
    Serial.print(millis() - prevSmoothMs); Serial.print("_");
    Serial.print(counter_sm); Serial.print("_");
    Serial.println(__func__);
    counter_sm++;
#endif
    prevSmoothMs = millis(); // запоминаем момент времени
    //_________________________
    prev_x = new_val_x;
    prev_y = new_val_y;
    prev_z = new_val_z;

    float delta_x = ax_x_rl - prev_x; // >>>X<<<
    new_val_x = prev_x + (delta_x * SMOOTH_COEFF); // config 0-smooth...1-sharp
    ax_x_sm = new_val_x;

    float delta_y = ax_y_rl - prev_y; // >>>Y<<<
    new_val_y = prev_y + (delta_y * SMOOTH_COEFF); // config 0-smooth...1-sharp
    ax_y_sm = new_val_y;

    float delta_z = ax_z_rl - prev_z; // >>>Z<<<
    new_val_z = prev_z + (delta_z * SMOOTH_COEFF); // config 0-smooth...1-sharp
    ax_z_sm = new_val_z;
  }
}

void WrapGyro::getData(float &axis_x, float &axis_y, float &axis_z) {
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    mpu.resetFIFO();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  axis_x = ypr[2] * toDeg;
  axis_y = ypr[1] * toDeg;
  axis_z = ypr[0] * toDeg;
}
