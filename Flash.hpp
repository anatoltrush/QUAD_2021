#ifndef FLASH_HPP
#define FLASH_HPP

#include <Arduino.h>

#define PIN_FLASH      8
#define TIME_FLASH_MS    500

/*example---> Flasher led1(5, 500, 400);
--->led1.Update();*/

class Flasher
{		
	uint8_t _led_pin		= 13; // номер пина со светодиодом

	uint32_t _on_time		= 0; // время включения в миллисекундах
	uint32_t _off_time		= 0; // время, когда светодиод выключен	
	uint32_t _prev_millis	= 0; // последний момент смены состояния

	bool _led_state			= false; // состояние ВКЛ/ВЫКЛ

public:
	Flasher(uint8_t pin, uint32_t time_on, uint32_t time_off);
	void update();
};
#endif // FLASH_HPP
