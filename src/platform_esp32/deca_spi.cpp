/**
 * @file      deca_spi.c
 *
 * @brief     SPI functions to interface to DW3000 chip's from ESP32.
 *
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put attention here
 *
 */

#include "deca_spi.h"
#include "port.h"
#include "assert.h"

//-----------------------------------------------------------------------------

extern SPIClass SPI;

// Define the SPI handler structure
static spi_handle_t spi_handle {
    .spi = &SPI,
    .fastSettings = SPISettings(DW_SPI_FAST_BAUDRATE, MSBFIRST, SPI_MODE0),
    .slowSettings = SPISettings(DW_SPI_SLOW_BAUDRATE, MSBFIRST, SPI_MODE0),
    .csPin = DW_CS_PIN,
    .spiMutex = xSemaphoreCreateMutex() // beginTransaction() and endTransaction() are already thread-safe, so this mutex is not strictly necessary
};

//------------------------------------------------------------------------------
// DW chip description

const dw_t dw_chip = {
    .irqPin = DW_IRQ_PIN,
    .rstPin = DW_RST_PIN,
    .wakeUpPin = DW_WAKEUP_PIN,
    .pSpi = &spi_handle
};

const dw_t *pDwChip = &dw_chip;

//------------------------------------------------------------------------------
static SPISettings spiSettings = spi_handle.slowSettings; // Default SPI settings

//-----------------------------------------------------------------------------

/*! 
 * @fn  set_dw_spi_slow_rate
 * @brief sets slow SPI clock speed for the DW chip
 *        left for compatibility.
 */
void set_dw_spi_slow_rate(void)
{
    spiSettings = spi_handle.slowSettings;
}

/*!
 * @fn      set_dw_spi_fast_rate
 * @brief   sets High SPI clock speed for the DW chip
 */
void set_dw_spi_fast_rate(void)
{
    spiSettings = spi_handle.fastSettings;
}



/****************************************************************************//**
 *
 *                              DWxxx SPI section
 *
 *******************************************************************************/
/*!
 * @fn  openspi()
 * @brief
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
}


/*!
 * @fn  closespi()
 * @brief
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
}

/*!
 * @fn  writetospi()
 * @brief
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t          headerLength,
               const    uint8_t *headerBuffer,
               uint16_t          bodyLength,
               const    uint8_t *bodyBuffer)
{
    /*  Input Validation */
    if ((headerLength > 0 && headerBuffer == NULL) ||
        (bodyLength > 0 && bodyBuffer == NULL)) {
        // Invalid input parameters
        return -1;
    }

    if (spi_handle.spi == NULL) {
        // SPI interface not initialized
        return -1;
    }

    /* SPI Transfer */
    spi_handle.spi->beginTransaction(spiSettings);
    digitalWrite(spi_handle.csPin, LOW); // Assert CS (active low)
    if (headerLength > 0) {
        // Transfer header data
        // Using transferBytes for efficient bulk transfer
        // Master sends data; ignoring received data by passing NULL
        spi_handle.spi->transferBytes(headerBuffer, NULL, headerLength);
    }
    if (bodyLength > 0) {
        // Transfer body data
        // Using transferBytes for efficient bulk transfer
        // Master sends data; ignoring received data by passing NULL
        spi_handle.spi->transferBytes(bodyBuffer, NULL, bodyLength);
    }
    digitalWrite(spi_handle.csPin, HIGH); // Deassert CS
    spi_handle.spi->endTransaction();

    return 0; // Success
}

/*!
 * @fn writetospiwithcrc()
 * @brief
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{
    /*  Input Validation */
    if ((headerLength > 0 && headerBuffer == NULL) ||
        (bodyLength > 0 && bodyBuffer == NULL)) {
        // Invalid input parameters
        return -1;
    }

    if (spi_handle.spi == NULL) {
        // SPI interface not initialized
        return -1;
    }

    /* SPI Transfer */
    spi_handle.spi->beginTransaction(spiSettings);
    digitalWrite(spi_handle.csPin, LOW); // Assert CS (active low)
    if (headerLength > 0) {
        // Transfer header data
        // Using transferBytes for efficient bulk transfer
        // Master sends data; ignoring received data by passing NULL
        spi_handle.spi->transferBytes(headerBuffer, NULL, headerLength);
    }
    if (bodyLength > 0) {
        // Transfer body data
        // Using transferBytes for efficient bulk transfer
        // Master sends data; ignoring received data by passing NULL
        spi_handle.spi->transferBytes(bodyBuffer, NULL, bodyLength);
    }
    // Transfer the CRC8 byte
    // Using transfer() as it's a single byte
    spi_handle.spi->transfer(crc8);
    digitalWrite(spi_handle.csPin, HIGH); // Deassert CS
    spi_handle.spi->endTransaction();

    return 0; // Success
}

/*!
 * @fn readfromspi()
 * @brief
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16_t          headerLength,
                /*const*/ uint8_t    *headerBuffer,
                uint16_t          readlength,
                uint8_t         *readBuffer)
{
    /* Input Validation */
    if ((headerLength > 0 && headerBuffer == NULL) ||
        (readlength > 0 && readBuffer == NULL)) {
        // Invalid input parameters
        return -1;
    }

    if (spi_handle.spi == NULL) {
        // SPI interface not initialized
        return -1;
    }

    /* SPI Transfer */
    spi_handle.spi->beginTransaction(spiSettings);
    digitalWrite(spi_handle.csPin, LOW); // Assert CS (active low)
    if (headerLength > 0) {
        // Transfer header data
        spi_handle.spi->transferBytes(headerBuffer, NULL, headerLength);
    }

    if (readlength > 0) {
        // Use transferBytes with NULL data to send dummy bytes (0xFF)
        spi_handle.spi->transferBytes(NULL, readBuffer, readlength);
    }
    digitalWrite(spi_handle.csPin, HIGH); // Deassert CS
    spi_handle.spi->endTransaction();

    return 0; // Success
}



