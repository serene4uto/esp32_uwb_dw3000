/**
 * @file      portDW.c
 *
 * @brief     Platform-dependent functions for current application are collected here
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put copyright
 *            All rights reserved.
 *
 */

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "port.h"

#include "deca_device_api.h"    

/**/
extern const dw_t *pDwChip;

/****************************************************************************//**
 * Sleep, usleep and bare sw_timer based on HAL tick
 */

/**
 * @brief Start a timer by saving the current timestamp in microseconds.
 * @param p_timestamp Pointer to store the current timestamp.
 */
void start_timer(volatile uint32_t *p_timestamp)
{
    *p_timestamp = esp_timer_get_time(); // Get time in microseconds
}

/**
 * @brief Check if a specified time interval has elapsed since the timestamp.
 * @param timestamp The starting timestamp in microseconds.
 * @param time The time interval to check against in microseconds.
 * @return true if the time interval has elapsed, false otherwise.
 */
bool check_timer(volatile uint32_t timestamp, uint32_t time)
{
    return (esp_timer_get_time() - timestamp) >= time;
}

/**
 * @brief Sleep for a specified number of milliseconds.
 * @param dwMs Number of milliseconds to sleep.
 */
void dwp_Sleep(uint32_t dwMs)
{
    // If using FreeRTOS, prefer vTaskDelay
    vTaskDelay(pdMS_TO_TICKS(dwMs));

    // If blocking delay is required without FreeRTOS:
    /*
    uint64_t dwStart;
    start_timer(&dwStart);
    uint64_t delay_us = (uint64_t)dwMs * 1000ULL; // Convert ms to us
    while (!check_timer(dwStart, delay_us))
    {
        // Optionally yield to other tasks or add a small delay
        taskYIELD();
    }
    */
}

/**
 * @brief Precise microsecond delay.
 * @param usec Number of microseconds to delay.
 */
void dwp_usleep(uint32_t usec)
{
    // Use the ROM function for microsecond delay
    ets_delay_us(usec);
}

/****************************************************************************//**
 *
 */

void init_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    pinMode(pDw->irqPin, INPUT_PULLDOWN);
}
void enable_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    attachInterrupt(pDw->irqPin, dwt_isr, RISING);
}

void disable_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    detachInterrupt(pDw->irqPin);
}

/*!
 * @fn reset_DW3000()
 * @brief
 * Low-level abstract function to reset the DW3000 device.
 * Configures the reset pin as output, asserts reset, then deconfigures as input.
 */
void reset_DW3000(void)
{
    const dw_t *pDw = pDwChip;

    pinMode(pDw->rstPin, OUTPUT);       // Set reset pin as OUTPUT
    digitalWrite(pDw->rstPin, LOW);     // Assert reset by pulling pin LOW
    usleep(200);                        
    pinMode(pDw->rstPin, INPUT);        // Set reset pin as INPUT (High Impedance)
    usleep(2000);                       
}

/****************************************************************************//**
 *
 */


void wakeup_device_with_io(void)
{
    port_wakeup_dw3000_fast();
}


/*!
 * @fn      port_wakeup_dw3000_fast
 * @brief   waking up of DW3000 using DW_CS pin
 *
 *          the fast wakeup takes ~1ms:
 *          500us to hold the CS  - TODO: this time can be reduced
 *          500us to the crystal to startup
 *          + ~various time 100us...10ms
 */
error_e port_wakeup_dw3000_fast(void)
{
    spi_handle_t *p = pDwChip->pSpi;

    // Pull CS low to initiate wakeup
    digitalWrite(DW_CS_PIN, LOW);
    usleep(500);  // Delay for 500 microseconds

    // Pull CS high to complete wakeup sequence
    digitalWrite(DW_CS_PIN, HIGH);
    usleep(500);  // Optional: Additional delay if necessary

    // Return success
    return _NO_ERR;
}

/*! 
 * @fn      port_wakeup_dw3000
 * @brief   waking up of DW3000 using DW_CS pin
 *
 * */
error_e port_wakeup_dw3000(void) {
    //TODO: implement the function
    return _NO_ERR;
}