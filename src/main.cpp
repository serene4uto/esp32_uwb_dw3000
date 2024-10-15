#include <Arduino.h>
#include <deca_device_api.h>
#include <SPI.h>

#include "deca_probe_interface.h"
#include "deca_spi.h"
#include "port.h"

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  pinMode(DW_CS_PIN, OUTPUT);
  digitalWrite(DW_CS_PIN, HIGH);

  // Initialize other DW3000 pins
  pinMode(DW_IRQ_PIN, INPUT);
  pinMode(DW_RST_PIN, INPUT); // Reset pin is not used

}

void loop() {
  // put your main code here, to run repeatedly:
}

