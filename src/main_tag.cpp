#include "main.h"

#include "app.h"
#include "deca_regs.h" 
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "port.h"

#include "task_tag.h"

const param_block_t FConfig = DEFAULT_CONFIG;


void main_tag() 
{

  // serial
  Serial.begin(115200);
  while (!Serial);

  // timer
  hwtimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz frequency), count up

  // spi
  SPI.begin();

  // gpio
  pinMode(DW_CS_PIN, OUTPUT);
  digitalWrite(DW_CS_PIN, HIGH);
  pinMode(DW_RST_PIN, INPUT);
  init_dw3000_irq();

  memset(&app, 0, sizeof(app));

  app.pConfig = (param_block_t *)&FConfig;
  

  reset_DW3000();

  // Initialize DW3000
  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
    Serial.println("Failed to probe DW3000");
    while (1);
  }

  delay(100);

  dwt_softreset();
  delay(200);

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("DW IC is not in IDLE_RC state");
    while (1);
  }
  Serial.println("IDLE OK");

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    Serial.println("INIT FAILED");
    while (1);
  }
  Serial.println("INIT OK");

  
  tag_terminate();

  tag_helper(NULL);


  while (1) {

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}



