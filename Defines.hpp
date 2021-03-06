#ifndef DEFINES_HPP
#define DEFINES_HPP

// <----- DIGITAL PINS ----->

#define PIN_RL          3 // PIN_RL_SERV_OUT
#define PIN_FR          4 // PIN_FR_SERV_OUT
#define PIN_RR          5 // PIN_RR_SERV_OUT
#define PIN_FL          6 // PIN_FL_SERV_OUT

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

#define TIME_PID_MS     20
#define TIME_GYRO_MS    30
#define TIME_FLASH_MS   500
#define TIME_VOLT_MS    2000

// <----- CONSTANT VALUES ----->
// engines
#define MIN_POWER       800
#define MAX_POWER       2300

// resistance
#define RESIST_1        12500.0f
#define RESIST_2        7500.0f

// gyro
#define TO_DEG          57.29577951308232087679815481410517033f
#define SMOOTH_COEFF    0.25f // config 0-low...1-full

// voltage
#define MAX_INP_VOLT    5.0
#define LOW_VOLT        10.2
#define MAX_VOLT        12.6

// PID
#define PID_KP          0.8f
#define PID_KI          0.8f
#define PID_KD          0.02f

#endif // DEFINES_HPP
