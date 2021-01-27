#include "Flash.hpp"

Flasher::Flasher(uint8_t pin, uint64_t time_on, uint64_t time_off) :
  _led_pin(pin), _on_time(time_on), _off_time(time_off)
{
  pinMode(_led_pin, OUTPUT);
}

void Flasher::update() {
  uint64_t _curr_mills = millis(); // текущее время в миллисекундах

  if (_curr_mills - _prev_millis > _on_time) {
    (_led_state == LOW) ? _led_state = HIGH : _led_state = LOW;

    _prev_millis = _curr_mills; // запоминаем момент времени
    digitalWrite(_led_pin, _led_state); // реализуем новое состояние
  }
}
