/*! ----------------------------------------------------------------------------
 * @file    deca_probe_interface.c
 * @brief   Interface structure. Provides external dependencies required by the driver
 *
 * @attention //TODO: put attention here
 *
 */
#include "deca_probe_interface.h"
#include "deca_interface.h"
#include "deca_spi.h"
#include "port.h"

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = readfromspi,
    .writetospi = writetospi,
    .writetospiwithcrc = writetospiwithcrc,
    .setslowrate = set_dw_spi_slow_rate,
    .setfastrate = set_dw_spi_fast_rate
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = wakeup_device_with_io
};
