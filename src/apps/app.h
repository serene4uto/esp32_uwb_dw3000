/**
 * @file      app.h
 *
 * @brief     Application Layer header contains all macros and structures related apllicaitions
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: Add attention if needed
 *
 */

#ifndef APP_H_
#define APP_H_ 1

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Application's global parameters structure */
struct app_s
{


};

typedef struct app_s app_t;

/* global application variables */
extern app_t app;


#ifdef __cplusplus
}
#endif //APP_H_ 1


#endif //__DECA_APP_H