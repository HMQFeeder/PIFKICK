#include "Arduino.h"

const int gas_pin = A0;
double gas_val = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(gas_pin, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  gas_val = analogRead(gas_pin);
  
  if (gas_val > 25) {
    Serial.println("===BAO DONG PHAT HIEN KHI CHAY===");
    Serial.print("giá trị là ");
    Serial.println(gas_val);
  }
  else {
    Serial.print("giá trị là ");
    Serial.println(gas_val);
  }
}
