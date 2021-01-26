#ifndef WRAPGYRO_HPP
#define WRAPGYRO_HPP

#include <Arduino.h>

#include <MPU6050.h>

#define TO_DEG    57.29577951308232087679815481410517033f
#define TIME_GYRO 25

class WrapGyro
{
  public:
    MPU6050 accel;
    float angle_ax = 0.0f, angle_ay = 0.0f;
    float prev_ax = 0.0f, prev_ay = 0.0f;
    float new_val_x = 0.0f, new_val_y = 0.0f;
    float delta_x = 0.0f, delta_y = 0.0f;    

    WrapGyro();
    void getSmoothResult(float &angle_x, float &angle_y);
};
#endif // WRAPGYRO_HPP
