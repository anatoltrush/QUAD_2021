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

#define TIME_PID_MS     20
#define TIME_GYRO_MS    20
#define TIME_ENGINE_MS  20
#define TIME_FLASH_MS   500
#define TIME_VOLT_MS    2000
#define TIME_CMD_UPD_MS 20
#define TIME_IS_LOST_MS 2000
#define TIME_NEW_RADIO  20000

#define EPOC_FOR_DOWN   40

// <----- CONSTANT VALUES ----->

// DATA
#define SIZE_OF_DATA    6
#define SIZE_OF_ACK     6

#define DATA_MIN        0
#define DATA_AVRG       1
#define DATA_MAX        2

// ENGINES
#define MIN_POWER       800
#define MAX_POWER       2300
#define MIN_UP_POWER    1000
#define WARN_MAIN_POWER 2100
#define WARN_ENG_POWER  2200
#define THR_ADD_POWER   2
#define THR_SUB_POWER   1

// GYRO
#define TO_DEG          57.295779513082320
#define SMOOTH_COEFF    1.0f // config 0-smooth...1-sharp
#define SET_P_R_ANG     8.0f
#define SET_YAW_ANG     0.2f

// VOLTAGE
#define VOLT_COEFF_K    81.49f // MAGIC VALUE
#define VOLT_COEFF_B    -3.47f // MAGIC VALUE
#define VOLT_MIN        10.05f // 3 x 3.35V
#define VOLT_MAX        12.6f // 3 x 4.2V

// PID
#define PID_KP_XY       2.0f
#define PID_KI_XY       2.0f
#define PID_KD_XY       0.8f // 0.7f
#define PID_KI_Z        2.0f
#define PID_KD_Z        0.8f // 0.7f
#define PID_LIM_COEFF   0.75f

// BYTES MSG
#define BT_MSG_YAW      0
#define BT_MSG_THR      1
#define BT_MSG_AUX1     2
#define BT_MSG_ROLL     3
#define BT_MSG_PTCH     4
#define BT_MSG_AUX2     5

// BYTES ACK
#define BT_ACK_VOLT     0
#define BT_ACK_WARN     1
#define BT_ACK_NUME     2
#define BT_ACK_POWR     3

#endif // DEFINES_H
