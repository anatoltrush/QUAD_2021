#include "WrapGyro.hpp"

void WrapGyro::init() {
  accel.initialize(); // первичная настройка датчика
  accel.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
}

void WrapGyro::getRealResultNow(float &angle_x, float &angle_y) {
  int16_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
  float ay = 0.0f, gx = 0.0f;
  float ax = 0.0f, gy = 0.0f;

  accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // сырые данные акселерометра нужно преобразовать в единицы гравитации
  // при базовых настройках 1G = 4096
  ay = ay_raw / 4096.0f;
  ax = ax_raw / 4096.0f;

  // функция acos возвращает угол в радианах, так что преобразуем
  // его в градусы при помощи коэффициента TO_DEG
  angle_x = (ay >= 0) ? 90.0f - TO_DEG * acos(ay) : angle_x = TO_DEG * acos(-ay) - 90.0f;
  angle_y = (ax >= 0) ? 90.0f - TO_DEG * acos(ax) : angle_y = TO_DEG * acos(-ax) - 90.0f;

  if (isnan(angle_x) || isnan(angle_ay)) { // !!!
    Serial.println("NAN!");
    return;
  }
}

// RELOAD
void WrapGyro::getRealResultTimer(float &angle_x, float &angle_y, uint16_t ms) {
  if (millis() - _prev_millis_rl > ms) {
    _prev_millis_rl = millis(); // запоминаем момент времени

#ifdef DEBUG_GYRO
    Serial.print(millis() + "_");
    Serial.println(__PRETTY_FUNCTION__);
#endif

    int16_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
    float ay = 0.0f, gx = 0.0f;
    float ax = 0.0f, gy = 0.0f;

    accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // сырые данные акселерометра нужно преобразовать в единицы гравитации
    // при базовых настройках 1G = 4096
    ay = ay_raw / 4096.0f;
    ax = ax_raw / 4096.0f;

    // функция acos возвращает угол в радианах, так что преобразуем
    // его в градусы при помощи коэффициента TO_DEG
    angle_x = (ay >= 0) ? 90.0f - TO_DEG * acos(ay) : angle_x = TO_DEG * acos(-ay) - 90.0f;
    angle_y = (ax >= 0) ? 90.0f - TO_DEG * acos(ax) : angle_y = TO_DEG * acos(-ax) - 90.0f;

    if (isnan(angle_x) || isnan(angle_y)) { // !!!
      Serial.println("NAN!");
      return;
    }
  }
}

void WrapGyro::getSmoothResultNow(float &angle_x, float &angle_y)
{
  int16_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
  float ay = 0.0f, gx = 0.0f;
  float ax = 0.0f, gy = 0.0f;
  prev_ax = new_val_x;
  prev_ay = new_val_y;

  accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // сырые данные акселерометра нужно преобразовать в единицы гравитации
  // при базовых настройках 1G = 4096
  ay = ay_raw / 4096.0f;
  ax = ax_raw / 4096.0f;

  // функция acos возвращает угол в радианах, так что преобразуем
  // его в градусы при помощи коэффициента TO_DEG
  angle_ax = (ay >= 0) ? 90.0f - TO_DEG * acos(ay) : angle_ax = TO_DEG * acos(-ay) - 90.0f;
  angle_ay = (ax >= 0) ? 90.0f - TO_DEG * acos(ax) : angle_ay = TO_DEG * acos(-ax) - 90.0f;

  if (isnan(angle_ax) || isnan(angle_ay)) { // !!!
    Serial.println("NAN!");
    return;
  }

  delta_x = angle_ax - prev_ax;
  new_val_x = prev_ax + (delta_x * SMOOTH_COEFF); // config 0-low...1-full
  angle_x = new_val_x;

  delta_y = angle_ay - prev_ay;
  new_val_y = prev_ay + (delta_y * SMOOTH_COEFF); // config 0-low...1-full
  angle_y = new_val_y;
}

// RELOAD
void WrapGyro::getSmoothResultTimer(float &angle_x, float &angle_y, uint16_t ms)
{
  if (millis() - _prev_millis_sm > ms) {
    _prev_millis_sm = millis(); // запоминаем момент времени

#ifdef DEBUG_GYRO
    Serial.print(millis() + "_");
    Serial.println(__PRETTY_FUNCTION__);
#endif

    int16_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
    float ay = 0.0f, gx = 0.0f;
    float ax = 0.0f, gy = 0.0f;
    prev_ax = new_val_x;
    prev_ay = new_val_y;

    accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // сырые данные акселерометра нужно преобразовать в единицы гравитации
    // при базовых настройках 1G = 4096
    ay = ay_raw / 4096.0f;
    ax = ax_raw / 4096.0f;

    // функция acos возвращает угол в радианах, так что преобразуем
    // его в градусы при помощи коэффициента TO_DEG
    angle_ax = (ay >= 0) ? 90.0f - TO_DEG * acos(ay) : angle_ax = TO_DEG * acos(-ay) - 90.0f;
    angle_ay = (ax >= 0) ? 90.0f - TO_DEG * acos(ax) : angle_ay = TO_DEG * acos(-ax) - 90.0f;

    if (isnan(angle_ax) || isnan(angle_ay)) { // !!!
      Serial.println("NAN!");
      return;
    }

    delta_x = angle_ax - prev_ax;
    new_val_x = prev_ax + (delta_x * SMOOTH_COEFF); // config 0-low...1-full
    angle_x = new_val_x;

    delta_y = angle_ay - prev_ay;
    new_val_y = prev_ay + (delta_y * SMOOTH_COEFF); // config 0-low...1-full
    angle_y = new_val_y;
  }
}
