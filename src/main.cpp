#include <Arduino.h>
#include <deca_device_api.h>
#include <SPI.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello World");
  deca_sleep(1000);
  Serial.println("Goodbye World");
}

void loop() {
  // put your main code here, to run repeatedly:
}

