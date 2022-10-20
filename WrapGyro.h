#ifndef WRAPGYRO_H
#define WRAPGYRO_H

#include "Defines.h"

//#define DEBUG_GYRO

class WrapGyro
{
  public:
    void init();

    float ax_x_rl = 0.0f;
    float ax_y_rl = 0.0f;
    float ax_z_rl = 0.0f;

    void getRealResultTimer(uint32_t ms);

  private:
    const float toDeg = 180.0 / M_PI;
    uint8_t mpuIntStatus = 0;   // holds actual interrupt status byte from MPU
    uint8_t devStatus = 0;      // return status after each device operation (0 = success, !0 = error)
    uint16_t fifoCount = 0;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];     // FIFO storage buffer
    float ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector*/

    uint32_t prevRealMs = 0;

#ifdef DEBUG_GYRO
    uint16_t counter_rl = 0;
#endif

    void getData(float &axis_x, float &axis_y, float &axis_z);
};
#endif // WRAPGYRO_H
