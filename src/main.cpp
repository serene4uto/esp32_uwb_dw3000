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
  // pinMode(DW_WAKEUP_PIN, OUTPUT);

  // reset_DW3000();

  // // Initialize DW3000
  // if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
  //   Serial.println("Failed to probe DW3000");
  //   while (1);
  // }

  // delay(100);

  // // intialize DW3000
  // if (dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT | DWT_READ_OTP_TMP) != DWT_SUCCESS) {
  //   Serial.println("Failed to initialize DW3000");
  //   while (1);
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
}

