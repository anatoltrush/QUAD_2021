#include "WrapRadio.hpp"

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
    byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"};
    
    radio->begin();   
    radio->setAutoAck(1);    
    radio->setRetries(0, 3);  
    radio->enableAckPayload();   
    radio->setPayloadSize(32); 

    radio->openReadingPipe(1, address[0]);     
    radio->setChannel(0x60);  

    radio->setPALevel(RF24_PA_MAX);  //óðîâåíü ìîùíîñòè ïåðåäàò÷èêà. Íà âûáîð RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio->setDataRate(RF24_1MBPS);  //ñêîðîñòü îáìåíà. Íà âûáîð RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

    radio->powerUp();
    radio->startListening();
  }
}
