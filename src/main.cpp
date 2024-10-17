// #include <Arduino.h>
// #include <SPI.h>

#include "main.h"
#include "app.h"

#define DEV_SNIFFER       0
#define DEV_ANCHOR_MASTER 1
#define DEV_ANCHOR        2
#define DEV_TAG           3


#define DEVICE_ROLE    DEV_TAG

app_t app; // global application variables

void setup() {

#if(DEVICE_ROLE == DEV_TAG)
  main_tag();
#endif

}

void loop() {

}


