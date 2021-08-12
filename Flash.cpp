#include "Flash.hpp"

Flasher::Flasher(uint8_t pin, uint32_t time_on, uint32_t time_off) :
  _led_pin(pin), _on_time(time_on), _off_time(time_off)
{
  pinMode(_led_pin, OUTPUT);
}

void Flasher::update() {
  if (millis() - _prev_millis >= _on_time) {
#ifdef DEBUG_FLASH
    Serial.print(millis() - _prev_millis);
    Serial.print("_");
    Serial.println(__func__);
#endif
    _prev_millis = millis(); // запоминаем момент времени

    (_led_state == LOW) ? _led_state = HIGH : _led_state = LOW;

    digitalWrite(_led_pin, _led_state); // реализуем новое состояние
  }
}
