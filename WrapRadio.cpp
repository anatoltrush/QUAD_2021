#include "WrapRadio.h"

WrapRadio::WrapRadio()
{
  radio = new RF24(PIN_NRF_CE, PIN_NRF_CS);
}

WrapRadio::~WrapRadio()
{
  delete radio;
}

void WrapRadio::init() {
  if (radio != NULL) {
    radio->begin();
    radio->setAutoAck(true);
    radio->enableAckPayload();
    radio->setPayloadSize(SIZE_OF_DATA);

    radio->openReadingPipe(1, 0x7878787878LL);
    radio->setChannel(0x60);

    radio->setPALevel(RF24_PA_MAX); // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio->setDataRate(RF24_1MBPS); // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

    radio->powerUp();
    radio->startListening();
  }
  else {
    Serial.println("Null pointer in WrapRadio");
  }
}

void WrapRadio::getData(uint8_t volt, bool isReached, uint8_t numEng) {
  ack_msg[BT_ACK_VOLT] = volt;
  ack_msg[BT_ACK_WARN] = isReached;
  ack_msg[BT_ACK_NUME] = numEng;
  if (radio->available(&pipeNum)) {
    radio->read(&data_msg, SIZE_OF_DATA);
    radio->writeAckPayload(pipeNum, ack_msg, SIZE_OF_ACK);
    // ____________
    lastGetData = millis();
  }
  else {
    noGetData = millis();
  }

  int32_t diffGetDataMs = noGetData - lastGetData;
  if (diffGetDataMs > TIME_IS_LOST_MS) {
    isConnLost = true;
    data_msg[BT_MSG_YAW] = DATA_AVRG;
    data_msg[BT_MSG_THR] = DATA_AVRG;
    data_msg[BT_MSG_AUX1] = DATA_MIN;
    data_msg[BT_MSG_ROLL] = DATA_AVRG;
    data_msg[BT_MSG_PTCH] = DATA_AVRG;
    data_msg[BT_MSG_AUX2] = DATA_MIN;
  }
  else {
    isConnLost = false;
  }
  // ---> Creating new radio for good ACK sending <---
  if (millis() - prevNewRadio >= TIME_NEW_RADIO) {
#ifdef DEBUG_RAD
    Serial.println("Creating new radio");
    Serial.print(millis() - prevNewRadio);
    Serial.print("_");
    Serial.println(__func__);
#endif
    prevNewRadio = millis();
    //_________________________
    delete radio;
    radio = new RF24(PIN_NRF_CE, PIN_NRF_CS);
    init();
  }
}

Extra::Extra():
  PinFlash(PIN_FLASH), PinVolt(PIN_VOLT),
  PinAux1(PIN_AUX_1), PinAux2(PIN_AUX_2) {
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
    Serial.println(readSignal);
    voltOutput = ((float)readSignal - VOLT_COEFF_B) / VOLT_COEFF_K;
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
