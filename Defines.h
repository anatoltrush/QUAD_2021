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

#define PIN_AUX_1       A0
#define PIN_AUX_2       A1

#define PIN_VOLT        A2
// GYRO                 A4
// GYRO                 A5

// <----- PERIODS ----->

#define TIME_PID_MS     10
#define TIME_GYRO_MS    10
#define TIME_ENGINE_MS  10
#define TIME_FLASH_MS   500
#define TIME_VOLT_MS    2000
#define TIME_CMD_UPD_MS 20

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
#define WARN_POWER      2000
#define THR_ADD_POWER   2

// GYRO
#define TO_DEG          57.295779513082320876798f
#define SMOOTH_COEFF    0.35f // config 0-smooth...1-sharp
#define OFFSET_FL_RR    -4.0f
#define OFFSET_FR_RL    0.0f

// VOLTAGE
#define VOLT_DIV        80.0f
#define VOLT_MIN        10.05f // 3 x 3.35V
#define VOLT_MAX        12.6f // 3 x 4.2V

// PID
#define PID_KP          1.5f
#define PID_KI          1.5f
#define PID_KD          0.6f
#define PID_LIM_COEFF   0.35f

#endif // DEFINES_H
