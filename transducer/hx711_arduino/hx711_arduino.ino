// HX711 -> Ruggeduino: VCC->5V, GND->GND, DOUT->D3, SCK->D2, RATE->5V
// Ruggeduino -> ClearCore COM-0: TX->Pin8, RX->Pin5, GND->Pin4, 5V->Pin6
// Sends raw ADC at 115200 baud, 80Hz

#include "HX711.h"

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(3, 2); // DOUT, SCK
  delay(500);
}

void loop() {
  if (scale.is_ready()) Serial.println(scale.get_value()); 
}
