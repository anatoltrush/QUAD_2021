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

  mpu.setXAccelOffset(OFFSET_A_X);
  mpu.setYAccelOffset(OFFSET_A_Y);
  mpu.setXGyroOffset(OFFSET_G_X);
  mpu.setYGyroOffset(OFFSET_G_Y);
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
