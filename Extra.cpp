#include "Extra.h"

Extra::Extra(uint8_t ledPin, uint8_t voltPin, uint8_t aux1pin, uint8_t aux2pin):
  PinFlash(ledPin), PinVolt(voltPin), PinAux1(aux1pin), PinAux2(aux2pin) {
  pinMode(PinFlash, OUTPUT);
  pinMode(PinVolt, INPUT);
  pinMode(PinAux1, OUTPUT);
  pinMode(PinAux2, OUTPUT);
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
    (flashState == LOW) ? flashState = HIGH : flashState = LOW;

    digitalWrite(PinFlash, flashState);
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
    uint16_t readSignal = analogRead(PinVolt);
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
    if (msg_data[BT_MSG_AUX1] == DATA_MAX) // left
      ; // AUX 1
    if (msg_data[BT_MSG_AUX2] == DATA_MAX) // left
      ; // AUX 2
  }
}
