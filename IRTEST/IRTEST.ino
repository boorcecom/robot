#include <IRremote.hpp>

const int irReceiverPin = 2; // Infrarouge connectÃ© Ã  la broche OUT 2

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(irReceiverPin, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode())
  { 
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.resume();
  }    
}
