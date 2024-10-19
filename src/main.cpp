// #include <Arduino.h>
// #include <SPI.h>

#include "main.h"
#include "app.h"

#define DEV_SNIFFER       0

#define DEV_ANCHOR        1
#define DEV_TAG           2


// #define DEVICE_ROLE    DEV_TAG
#define DEVICE_ROLE    DEV_ANCHOR

app_t app; // global application variables

void setup() {

#if(DEVICE_ROLE == DEV_TAG)
  main_tag();
#elif(DEVICE_ROLE == DEV_ANCHOR2)
  main_anchor();
#endif

}

void loop() {

}


