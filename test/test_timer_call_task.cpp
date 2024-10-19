#include <Arduino.h>

// FreeRTOS task handle
TaskHandle_t TaskHandle1 = NULL;

// Hardware timer variables
hw_timer_t * timer = NULL;

// ISR-safe function to notify the task
void IRAM_ATTR onTimer() {
  // Notify the task. Using vTaskNotifyGiveFromISR ensures it is safe to call from ISR.
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle1, &xHigherPriorityTaskWoken);
  
  // Optionally, yield if the task has higher priority
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// The high-priority task that waits for the notification
void Task1(void * parameter) {
  while (1) {
    // Wait indefinitely for a notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // After being notified, perform the desired action
    Serial.println("Hello the wait");
  }
}

// Variables for the loop() function to manage 2-second interval
unsigned long previousMillis = 0;        // Stores the last time "loop" was printed
const unsigned long interval = 2000;     // Interval at which to print "loop" (2 seconds)

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }
  
  // Create the FreeRTOS task with highest priority
  BaseType_t result = xTaskCreate(
    Task1,                          // Task function
    "Task1",                        // Task name
    2048,                           // Stack size (in words)
    NULL,                           // Task input parameter
    configMAX_PRIORITIES - 1,       // Priority (highest possible)
    &TaskHandle1                    // Task handle
  );
  
  if (result != pdPASS) {
    Serial.println("Task creation failed!");
    while (1); // Halt
  }
  
  // Configure the hardware timer
  // Arguments:
  //   - timer number (0-3)
  //   - prescaler (divider). ESP32 has a 80MHz APB clock, so prescaler of 80 scales it to 1MHz
  //   - count up (true) or down (false)
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  
  // Attach the onTimer function as the ISR handler
  timerAttachInterrupt(timer, &onTimer, true);
  
  // Set the timer to trigger every 1 second (1,000,000 microseconds)
  timerAlarmWrite(timer, 1000000, true); // 1 second, auto-reload true
  
  // Start the timer
  timerAlarmEnable(timer);
  
  Serial.println("Hardware timer setup complete.");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("loop");
  }
  
  // Optional: Yield to other tasks to ensure smooth multitasking
  // This is generally handled by the FreeRTOS scheduler, but can be explicitly called if needed
  // vTaskDelay(0);
}