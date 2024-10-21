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
#include "port.h"

#include "deca_device_api.h"    

/**/
extern const dw_t *pDwChip;

hw_timer_t * hwtimer = NULL;

portMUX_TYPE task_mux = portMUX_INITIALIZER_UNLOCKED;

static bool isDw3000InterruptAttached = false;
static bool isGpioIsrServiceInstalled = false;

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

// void init_dw3000_irq(void)
// {
//     const dw_t *pDw = pDwChip;

//     pinMode(pDw->irqPin, INPUT_PULLDOWN);
// }
// void enable_dw3000_irq(void)
// {
//     const dw_t *pDw = pDwChip;

//     attachInterrupt(pDw->irqPin, dwt_isr, RISING);

//     isDw3000InterruptAttached = true;
// }

// void disable_dw3000_irq(void)
// {
//     const dw_t *pDw = pDwChip;

//     // check if the interrupt is attached
//     if (isDw3000InterruptAttached)
//     {
//         detachInterrupt(pDw->irqPin);
//         isDw3000InterruptAttached = false;
//     }
// }


static void IRAM_ATTR isr_handler(void *arg)
{
    dwt_isr();
}

void init_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pDw->irqPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
    }
}

void enable_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    // Install GPIO ISR service if not already installed
    if (!isGpioIsrServiceInstalled) {
        esp_err_t ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            // ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
            return;
        }
        isGpioIsrServiceInstalled = true;
    }

    // Configure the interrupt type for the GPIO pin
    esp_err_t ret = gpio_set_intr_type((gpio_num_t)pDw->irqPin, GPIO_INTR_POSEDGE);
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Set interrupt type failed: %s", esp_err_to_name(ret));
        return;
    }

    // Add ISR handler for the GPIO pin
    ret = gpio_isr_handler_add((gpio_num_t)pDw->irqPin, isr_handler, NULL);
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "ISR handler add failed: %s", esp_err_to_name(ret));
        return;
    }

    isDw3000InterruptAttached = true;
    // ESP_LOGI(TAG, "DW3000 interrupt enabled");
}

void disable_dw3000_irq(void)
{
    const dw_t *pDw = pDwChip;

    if (isDw3000InterruptAttached) {
        esp_err_t ret = gpio_isr_handler_remove((gpio_num_t)pDw->irqPin);
        if (ret != ESP_OK) {
            // ESP_LOGE(TAG, "ISR handler remove failed: %s", esp_err_to_name(ret));
            return;
        }
        isDw3000InterruptAttached = false;
        // ESP_LOGI(TAG, "DW3000 interrupt disabled");
    }
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




/* @fn         port_stop_all_UWB(s)
 *
 * @brief     stop UWB activity
 */
void port_stop_all_UWB(void)
{
    port_disable_dw_irq_and_reset(1);
    dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL);
}


/*
 * @brief disable DW_IRQ, reset DW3000
 *        and set
 *        app.DwCanSleep = DW_CANNOT_SLEEP;
 *        app.DwEnterSleep = DW_NOT_SLEEPING;
 * */
error_e port_disable_dw_irq_and_reset(int reset)
{
    taskENTER_CRITICAL(&task_mux);

    disable_dw3000_irq(); /**< disable NVIC IRQ until we configure the device */

    //this is called to reset the DW device
    if (reset)
    {
        reset_DW3000();
    }

    // app.DwSpiReady = DW_SPI_READY; //SPI ready INT not used (disabled above) here we'll assume SPI is ready

    taskEXIT_CRITICAL(&task_mux);

    return _NO_ERR;
}



// void port_init_hwtimer(void)
// {
//     if (hwtimer != NULL)
//     {
//         return;
//     }
//     hwtimer = timerBegin(0, 80, true); // 80 MHz, 1 us per tick
// }

// void port_deinit_hwtimer(void)
// {
//     if (hwtimer == NULL)
//     {
//         return;
//     }
//     timerEnd(hwtimer);
//     hwtimer = NULL;
// }

// void port_enable_hwtimer(void)
// {
//     if (hwtimer == NULL)
//     {
//         return;
//     }
//     timerAlarmEnable(hwtimer);
// }

// void port_disable_hwtimer(void)
// {
//     if (hwtimer == NULL)
//     {
//         return;
//     }
//     timerDetachInterrupt(hwtimer);
// }