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
  // pinMode(DW_IRQ_PIN, INPUT);
  // pinMode(DW_RST_PIN, OUTPUT);
  // digitalWrite(DW_RST_PIN, HIGH); // Assuming HIGH is inactive
  // pinMode(DW_WAKEUP_PIN, OUTPUT);
  // digitalWrite(DW_WAKEUP_PIN, HIGH); // Assuming HIGH is inactive

  // Initialize DW3000
  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
    Serial.println("Failed to probe DW3000");
    while (1);
  }

  
}

void loop() {
  // put your main code here, to run repeatedly:
}

