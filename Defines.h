#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

// <----- DIGITAL PINS ----->

#define PIN_RL          3 // rear-left motor
#define PIN_FR          4 // front-right motor
#define PIN_RR          5 // rear-right motor
#define PIN_FL          6 // front-left motor

#define PIN_FLASH       8

#define PIN_NRF_CE      9
#define PIN_NRF_CS      10
#define PIN_NRF_MOSI    11
#define PIN_NRF_MISO    12
#define PIN_NRF_SCK     13

// <----- ANALOG PINS ----->

#define PIN_VOLT        A2
// GYRO                 A4
// GYRO                 A5

// <----- PERIODS ----->

#define TIME_PID_MS     10
#define TIME_GYRO_MS    10
#define TIME_ENGINE_MS  10
#define TIME_FLASH_MS   500
#define TIME_VOLT_MS    2000

// <----- CONSTANT VALUES ----->

// DATA
#define SIZE_OF_DATA    6
#define SIZE_OF_ACK     3

#define SET_ANGLE       5.0f

#define DATA_MIN        0
#define DATA_AVRG       1
#define DATA_MAX        2

// ENGINES
#define MIN_POWER       800
#define MAX_POWER       2300
#define MIN_DIAG_POWER  1000  
#define WARN_POWER      1800

// RESISTANCE
#define RESIST_1        12500.0f
#define RESIST_2        7500.0f

// GYRO
#define TO_DEG          57.29577951308232087679815481410517033f
#define SMOOTH_COEFF    0.35f // config 0-smooth...1-sharp
#define OFFSET_FL_RR    -4.0f
#define OFFSET_FR_RL    0.0f

// VOLTAGE
#define MAX_INP_VOLT    5.0
#define LOW_VOLT        10.2
#define MAX_VOLT        12.6

// PID
#define PID_KP          1.5f
#define PID_KI          1.5f
#define PID_KD          0.6f
#define PID_LIM_COEFF   0.35f

#endif // DEFINES_H
