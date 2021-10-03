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
}

void WrapRadio::getData(uint8_t volt, bool isReached, uint8_t numEng) {
  ack_msg[BT_ACK_VOLT] = volt;
  ack_msg[BT_ACK_WARN] = isReached;
  ack_msg[BT_ACK_NUME] = numEng;
  if (radio->available(&pipeNum)) {
    radio->read(&data_msg, SIZE_OF_DATA);
    radio->writeAckPayload(pipeNum, ack_msg, SIZE_OF_ACK);
    // _____
    lastGetData = millis();
  }
  else {
    noGetData = millis();
  }
  
  int32_t diffGetDataMs = noGetData - lastGetData;
  if(diffGetDataMs > TIME_IS_LOST_MS){
    isConnLost = true;
    data_msg[BT_MSG_YAW] = DATA_AVRG;
    data_msg[BT_MSG_THR] = DATA_AVRG;
    data_msg[BT_MSG_AUX1] = DATA_MIN;
    data_msg[BT_MSG_ROLL] = DATA_AVRG;
    data_msg[BT_MSG_PTCH] = DATA_AVRG;
    data_msg[BT_MSG_AUX2] = DATA_MIN;
  }
  else{
    isConnLost = false;
  }
}
