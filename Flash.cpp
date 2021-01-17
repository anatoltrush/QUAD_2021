#include "FLASH.hpp"

Flasher::Flasher(uint8_t pin, uint64_t time_on, uint64_t time_off) :
	_led_pin(pin), _on_time(time_on), _off_time(time_off)
{
	pinMode(_led_pin, OUTPUT);
}

void Flasher::update() {
	uint64_t _curr_mills = millis(); // текущее время в миллисекундах

	if ((_led_state == HIGH) && (_curr_mills - _prev_millis >= _on_time)) {
		_led_state = LOW; // выключаем
		_prev_millis = _curr_mills; // запоминаем момент времени
		digitalWrite(_led_pin, _led_state); // реализуем новое состояние
	}

	if ((_led_state == LOW) && (_curr_mills - _prev_millis >= _off_time)) {

#ifdef DEBUG_FLASH
		Serial.print("curr: ");
		uint64_t _local_diff = _curr_mills - _prev_millis;
		uint32_t val = _local_diff;
		Serial.println(val);
#endif // DEBUG_FLASH

		_led_state = HIGH; // выключаем
		_prev_millis = _curr_mills; // запоминаем момент времени
		digitalWrite(_led_pin, _led_state); // реализуем новое состояние
	}
}