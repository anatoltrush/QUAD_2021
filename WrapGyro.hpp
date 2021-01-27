#ifndef WRAPGYRO_HPP
#define WRAPGYRO_HPP

#include <Arduino.h>

#include <MPU6050.h>

#define TO_DEG    57.29577951308232087679815481410517033f
#define TIME_GYRO 25
#define SMOOTH  0.2f // config 0-low...1-full

class WrapGyro
{
  public:
    MPU6050 accel;
    float angle_ax = 0.0f, angle_ay = 0.0f;
    float prev_ax = 0.0f, prev_ay = 0.0f;
    float new_val_x = 0.0f, new_val_y = 0.0f;
    float delta_x = 0.0f, delta_y = 0.0f;

    uint64_t _prev_millis  = 0; // последний момент смены состояния

    void init();
    void getSmoothResult(float &angle_x, float &angle_y);
    void getSmoothResult(float &angle_x, float &angle_y, uint16_t ms);
};
#endif // WRAPGYRO_HPP
