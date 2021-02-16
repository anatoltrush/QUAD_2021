#ifndef VOLTAGE_HPP
#define VOLTAGE_HPP

#include <Arduino.h>
#include <LiquidCrystal.h>

#include "Defines.hpp"

//example---> Voltage volt(5, 1000); (pin, ms)
//--->float v = volt.update();

class Voltage
{
    uint8_t perc        = 0;    // voltage in percent
    uint8_t inPin       = 0;    // analog pin
    uint32_t period     = 0;    // period
    uint32_t prevMillis = 0;    //
    float signal        = 0.0f; // analog signal
    float output        = 0.0f; // output signal

  public:
    Voltage(uint8_t pin, uint32_t time_upd);
    float update();

};
#endif // VOLTAGE_HPP

// 12.6 -> 12100/7900 -> 4.98V
// 10.2 -> 12100/7900 -> 4.03V
