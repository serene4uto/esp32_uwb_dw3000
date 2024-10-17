/**
 * @file      port.h
 *
 * @brief     port headers file to ESP32
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put copyright
 *            All rights reserved.
 *
 */

#ifndef PORT__H_
#define PORT__H_ 1

#include <stdint.h>
#include <stdbool.h>

#include <Arduino.h>
#include <SPI.h>

#include "deca_spi.h"
#include "error_types.h"

#define DW_SPI_FAST_BAUDRATE 8000000L
#define DW_SPI_SLOW_BAUDRATE 2000000L

#define DW_IRQ_PIN 34
#define DW_RST_PIN 27
#define DW_WAKEUP_PIN 32
#define DW_CS_PIN 4


#ifdef __cplusplus
 extern "C" {
#endif


//-----------------------------------------------------------------------------
//    DWxxx description

/* description of spi interface to DW3000 chip */
struct spi_handle_s
{
    SPIClass *spi;                   // Pointer to SPI instance (e.g., &SPI)
    SPISettings fastSettings;        // SPI settings for fast rate
    SPISettings slowSettings;        // SPI settings for slow rate
    uint8_t csPin;                   // Chip Select pin number
    SemaphoreHandle_t spiMutex;      // Mutex for thread-safe access
};

typedef struct spi_handle_s spi_handle_t;

/* Description of connection to the DW3000 chip */
struct dw_s
{
    uint8_t         irqPin;       // Interrupt pin number
    uint8_t         rstPin;       // Reset pin number
    uint8_t         wakeUpPin;    // Wake-up pin number
    spi_handle_t    *pSpi;         // Pointer to SPIClass instance (e.g., &SPI)
};

typedef struct dw_s dw_t;

/* port functions prototypes
 *
 * */
void init_dw3000_irq(void);
void enable_dw3000_irq(void);
void disable_dw3000_irq(void);
void reset_DW3000(void);

error_e port_wakeup_dw3000_fast(void);
error_e port_wakeup_dw3000(void);
void wakeup_device_with_io(void);

/* Time section */
void start_timer(volatile uint32_t *p_timestamp);
bool check_timer(volatile uint32_t timestamp, uint32_t time);
void dwp_usleep(uint32_t usec);
void dwp_Sleep( volatile uint32_t );




#ifdef __cplusplus
}
#endif

#endif /* PORT__H_ */


