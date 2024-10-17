/**-
 * @file    deca_sleep.c
 *
 * @brief   platform dependent sleep implementation
 *
 * @attention //TODO: put attention here
 *
 * @author Nguyen Ha Trung
 */

#include "deca_device_api.h"
#include "port.h"

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    dwp_Sleep(time_ms);
}

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
    dwp_usleep(time_us);
}



