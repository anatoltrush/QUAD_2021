#ifndef WRAPGYRO_HPP
#define WRAPGYRO_HPP

#include <Arduino.h>
#include <MPU6050.h>

#include "Defines.hpp"

class WrapGyro
{
  public:
    MPU6050 accel;
    float angle_ax = 0.0f, angle_ay = 0.0f;
    float prev_ax = 0.0f, prev_ay = 0.0f;
    float new_val_x = 0.0f, new_val_y = 0.0f;
    float delta_x = 0.0f, delta_y = 0.0f;

    uint32_t _prev_millis_sm = 0;
    uint32_t _prev_millis_rl = 0;

    void init();

    void getRealResult(float &angle_x, float &angle_y);
    void getRealResult(float &angle_x, float &angle_y, uint16_t ms);

    void getSmoothResult(float &angle_x, float &angle_y);
    void getSmoothResult(float &angle_x, float &angle_y, uint16_t ms);
};
#endif // WRAPGYRO_HPP
