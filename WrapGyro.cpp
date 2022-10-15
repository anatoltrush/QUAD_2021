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

  mpu.setXAccelOffset(OFF_A_X);
  mpu.setYAccelOffset(OFF_A_Y);
  mpu.setZAccelOffset(OFF_A_Z);
  mpu.setXGyroOffset(OFF_G_X);
  mpu.setYGyroOffset(OFF_G_Y);
  mpu.setZGyroOffset(OFF_G_Z);
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

WrapKalman::WrapKalman(float est, float coeff) {
  filterX = new GKalman(est, coeff);
  filterY = new GKalman(est, coeff);
  filterZ = new GKalman(est, coeff);
}

WrapKalman::~WrapKalman() {
  delete filterX;
  delete filterY;
  delete filterZ;
}

void WrapKalman::setDataAndCalc(float inX, float inY, float inZ, uint32_t ms) {
  if (millis() - prevCalcMs >= ms) {
    valX = filterX->filtered(inX);
    valY = filterY->filtered(inY);
    valZ = filterZ->filtered(inZ);
  }
}
