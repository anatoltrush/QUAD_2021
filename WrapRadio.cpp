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
    //radio->setRetries(5, 1); // delay, count
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
  if (radio->available(&pipeNum)) {
    radio->read(&data_msg, SIZE_OF_DATA);
    ack_msg[0] = volt;
    ack_msg[1] = isReached;
    ack_msg[2] = numEng;
    radio->writeAckPayload(pipeNum, ack_msg, SIZE_OF_ACK);
  }
}
