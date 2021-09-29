#include "Extra.h"

Extra::Extra(uint8_t led_pin, uint8_t volt_pin):
  flashPin(led_pin), voltPin(volt_pin) {
  pinMode(flashPin, OUTPUT);
  pinMode(voltPin, INPUT);
}

void Extra::flash(uint32_t ms) {
  if (millis() - prevFlashMs >= ms) {
#ifdef DEBUG_EXTRA
    Serial.print(millis() - prevFlashMs);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevFlashMs = millis();
    //_________________________
    (ledState == LOW) ? ledState = HIGH : ledState = LOW;

    digitalWrite(flashPin, ledState);
  }
}

void Extra::getVoltQuad(uint32_t ms) {
  if (millis() - prevVoltMs >= ms) {
#ifdef DEBUG_EXTRA
    Serial.print(millis() - prevVoltMs);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevVoltMs = millis();
    //_________________________
    uint16_t readSignal = analogRead(voltPin);
    voltOutput = (float)readSignal / VOLT_DIV;
    // in percents
    float diffCurr = voltOutput - VOLT_MIN;
    voltPercent = (diffCurr / diffMinMax) * 100;
  }
}

void Extra::customCommand(uint8_t* msg_data, uint32_t ms) {
  if (millis() - prevCmndMs >= ms) {
#ifdef DEBUG_EXTRA
    Serial.print(millis() - prevCmndMs);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevCmndMs = millis();
    //_________________________
    // implement
    if (msg_data[2] == DATA_MAX) // left
      ; // AUX 1
    if (msg_data[5] == DATA_MAX) // left
      ; // AUX 2
  }
}
