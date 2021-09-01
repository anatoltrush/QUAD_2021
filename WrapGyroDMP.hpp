#ifndef WRAPGYRODMP_HPP
#define WRAPGYRODMP_HPP

#include "Defines.hpp"

//#define DEBUG_GYRO_DMP

class WrapGyroDMP
{
  public:
    void init();

    float ax_x_rl = 0.0f,  ax_x_sm = 0.0f;
    float ax_y_rl = 0.0f,  ax_y_sm = 0.0f;
    float ax_z_rl = 0.0f,  ax_z_sm = 0.0f;

    void getRealResultTimer(uint32_t ms);
    void getSmoothResultTimer(uint32_t ms);

  private:
    const float toDeg = 180.0 / M_PI;
    uint8_t mpuIntStatus = 0;   // holds actual interrupt status byte from MPU
    uint8_t devStatus = 0;      // return status after each device operation (0 = success, !0 = error)
    uint16_t fifoCount = 0;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];     // FIFO storage buffer
    float ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector*/

    uint32_t prev_millis_sm = 0;
    uint32_t prev_millis_rl = 0;

    float prev_x = 0.0f,    prev_y = 0.0f,      prev_z = 0.0f;
    float new_val_x = 0.0f, new_val_y = 0.0f,   new_val_z = 0.0f;

#ifdef DEBUG_GYRO_DMP
    uint16_t counter_rl = 0;
    uint16_t counter_sm = 0;
#endif

    void getData(float &axis_x, float &axis_y, float &axis_z);
};
#endif // WRAPGYRODMP_HPP
