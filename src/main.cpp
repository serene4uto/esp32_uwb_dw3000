#include <Arduino.h>
#include <SPI.h>


#include "deca_regs.h" 
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "port.h"

hw_timer_t * timer = NULL;

void setup() {

  Serial.begin(115200);
  while (!Serial);

  // spi
  SPI.begin();

  // gpio
  pinMode(DW_CS_PIN, OUTPUT);
  digitalWrite(DW_CS_PIN, HIGH);
  init_dw3000_irq();
  reset_DW3000();

  // timer
  timer = timerBegin(0, 80, true); // 80 prescaler, 1 tick = 1us

  // Initialize DW3000
  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
    Serial.println("Failed to probe DW3000");
    while (1);
  }

  delay(100);

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("DW IC is not in IDLE_RC state");
    while (100);
  }

  dwt_softreset();
  delay(200);

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("DW IC is not in IDLE_RC state");
    while (100);
  }
  Serial.println("IDLE OK");

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    Serial.println("INIT FAILED");
    while (100);
  }
  Serial.println("INIT OK");


  // hardware timer




  // task

  
}

void loop() {
  delay(10);
}
