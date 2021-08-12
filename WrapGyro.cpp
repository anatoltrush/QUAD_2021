#include "WrapGyro.hpp"

void WrapGyro::init() {
  accel.initialize(); // первичная настройка датчика
  accel.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
}

void WrapGyro::getRealResultTimer(float &angle_x, float &angle_y, uint16_t ms) {
  if (millis() - _prev_millis_rl >= ms) {
#ifdef DEBUG_GYRO
    Serial.print(millis() - _prev_millis_rl);
    Serial.print("_");
    Serial.println(__PRETTY_FUNCTION__);
#endif
    _prev_millis_rl = millis(); // запоминаем момент времени

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

void WrapGyro::getSmoothResultTimer(float &angle_x, float &angle_y, uint16_t ms)
{
  if (millis() - _prev_millis_sm >= ms) {
#ifdef DEBUG_GYRO
    Serial.print(millis() - _prev_millis_sm);
    Serial.print("_");
    Serial.println(__PRETTY_FUNCTION__);
#endif
    _prev_millis_sm = millis(); // запоминаем момент времени

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
