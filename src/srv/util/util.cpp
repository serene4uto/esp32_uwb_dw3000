/**---------------------------------------
 * @file    util.c
 * @brief   utility functions:
 *          Seconds to/from DW1000 internal time conversions.
 *          1sy = 1us / 1.0256
 *          1dt  = 1s / 499.2e6 / 128.0
 *          sfd timeout calculation
 *
 * @author Decawave
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 */

#include "util.h"
#include "math.h"

uint64_t util_us_to_dev_time (double microsecu)
{
    uint64_t dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6;

    dt = (uint64_t) (dtime) ;

    return (dt);
}

double util_dev_time_to_sec(uint64_t dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return (f) ;
}

uint64_t util_sec_to_dev_time (double secu)
{
    uint64_t dt;
    double dtime;

    dtime = (secu / (double) DWT_TIME_UNITS);

    dt = 0x0FFFFFFFFFULL& (uint64_t) (dtime) ;

    return (dt);
}

double util_us_to_sy(double us)
{
    return (double)(us / 1.0256);
}



