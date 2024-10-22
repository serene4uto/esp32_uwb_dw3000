/**
 * @file    util.h
 *
 * @brief   Decawave Application Layer utility functions & Macros
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#ifndef __UTIL__H__
#define __UTIL__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include "deca_device_api.h"

// #define DWT_SYMBOL_UNIT_S             (1.0256e-6)         /**< Symbol time, in seconds. */
// #define DWT_SYMBOL_UNIT_US            (1.0256)            /**< Symbol time, in microseconds. */

// #define DWT_TIME_UNIT_S               (1.0/499.2e6/128.0) /**< = 15.65e-12 s */
// #define DWT_TIME_UNIT_US              (DWT_TIME_UNIT_S * 1e6) /**< = 15.65e-6 us */
// #define UUS_TO_DWT_TIME_UNITS(uus)    ((uint64_t)(uus / DWT_TIME_UNIT_US)) /**< Convert microseconds to device time units. */

#define SPEED_OF_LIGHT                (299702547.0)       /**< Speed of light in air, in metres per second. */
// ----------------------------------------------------------------------------

#define AR2U32(x)               (((uint32_t)x[3])<<24 |\
                                ((uint32_t)x[2])<<16  |\
                                ((uint32_t)x[1])<<8   |\
                                ((uint32_t)x[0]))

#define AR2U16(x)               ((x[1]<<8) | x[0])


#define TS2U64_MEMCPY(x,y) do{\
                            x = (uint64_t)(((uint64_t)y[4]<<32)|\
                                ((uint64_t)y[3]<<24)|\
                                ((uint64_t)y[2]<<16)|\
                                ((uint64_t)y[1]<<8) |\
                                y[0]); \
                         }while(0)

#define TS2TS_MEMCPY(x,y)  do {\
                            for(int i=0;i<TS_40B_SIZE;i++) {x[i] = y[i];}\
                          }while(0)

#define TS2TS_UWB_MEMCPY(x,y)  do {\
                            for(int i=0;i<TS_UWB_SIZE;i++) {x[i] = y[i];}\
                          }while(0)

#define U642TS_MEMCPY(x,y) do {\
                            for(int i=0;i<TS_40B_SIZE;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

#define U642TS_UWB_MEMCPY(x,y) do {\
                            for(int i=0;i<TS_UWB_SIZE;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

#define U32TOAR_MEMCPY(x,y) do {\
                            for(int i=0;i<4;i++) {x[i] = (uint8_t)((y>>(i*8)&0xFF));}\
                          }while(0)

// ----------------------------------------------------------------------------

uint64_t util_us_to_dev_time (double us);
double   util_dev_time_to_sec(uint64_t dt);
uint64_t util_sec_to_dev_time (double sec);
double util_us_to_sy(double us);


#ifdef __cplusplus
}
#endif

#endif /* __UTIL__H__ */
