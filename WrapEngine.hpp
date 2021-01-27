#ifndef WRAPENGINE_HPP
#define WRAPENGINE_HPP

#include <Arduino.h>

#include <Servo.h>

#define FR_pin_out 4 // PIN_FR_SERV_OUT
#define FL_pin_out 6 // PIN_FL_SERV_OUT
#define RR_pin_out 3 // PIN_RR_SERV_OUT
#define RL_pin_out 5 // PIN_RL_SERV_OUT

#define MIN_POWER 800
#define MAX_POWER 2300

class WrapEngine
{
  public:

    Servo motorFR;
    Servo motorFL;
    Servo motorRR;
    Servo motorRL;

    int POWER_FR = MIN_POWER;
    int POWER_FL = MIN_POWER;
    int POWER_RR = MIN_POWER;
    int POWER_RL = MIN_POWER;

    uint64_t _prev_millis  = 0; // последний момент смены состояния

    WrapEngine();
    void apply();
    void apply(uint16_t ms);
};
#endif // WRAPENGINE_HPP
