#ifndef VOLTAGE_HPP
#define VOLTAGE_HPP

#include <Arduino.h>
#include <LiquidCrystal.h>

#define PIN_VOLT      5.0
#define LOW_VOLT      10.2
#define MAX_VOLT      12.6
#define PIN_VOLT      A2
#define TIME_VOLT_MS  2000

#define R1  12500.0f
#define R2  7500.0f

//example---> Voltage volt(5, 1000);(pin, ms)
//--->float vvv = volt.update();

class Voltage
{
    uint8_t perc = 0;       //voltage in percent
    float signal = 0.0f;    //analog signal
    float output = 0.0f;    //output signal
    uint8_t inPin = 0;      //analog pin
    uint32_t period = 0;    //period
    uint32_t prevMillis = 0;

  public:
    Voltage(uint8_t pin, uint32_t time_upd);
    float update();

};
#endif // VOLTAGE_HPP

// 12.6 -> 12100/7900 -> 4.98V
// 10.2 -> 12100/7900 -> 4.03V
