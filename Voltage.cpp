#include "Voltage.hpp"

Voltage::Voltage(uint8_t pin, uint32_t time_upd):
  inPin(pin), period(time_upd)
{
  pinMode(inPin, INPUT);
}

float Voltage::update() {
  if (millis() - prevMillis >= period)
  {
    prevMillis = millis();

    signal = (analogRead(inPin) / 1024.0f) * PIN_VOLT;
    float div_koeff = R2 / (R1 + R2); // = 0.375
    output = signal / div_koeff;

    // in percs
    float diff_max_min = MAX_VOLT - LOW_VOLT;
    float diff_curr = output - LOW_VOLT;
    perc = (diff_curr / diff_max_min) * 100;

    return output;
  }
  else return output;
}
