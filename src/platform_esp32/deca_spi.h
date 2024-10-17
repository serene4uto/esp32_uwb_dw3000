/**
 * @file      deca_spi.h
 *
 * @brief     SPI interface access function protypes
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put attention here
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
#include "driver/spi_master.h"


/**---------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void);

/**---------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void);

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer);
int writetospiwithcrc( uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8);
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer);
void set_dw_spi_fast_rate(void);
void set_dw_spi_slow_rate(void);


#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
