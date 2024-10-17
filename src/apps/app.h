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

#include "error_types.h"
#include "task_tag.h"

#ifdef __cplusplus
 extern "C" {
#endif


typedef enum {
    Twr_Tx_Done,
    Twr_Tx_Blink_Sent,          //tag sends blink
    Twr_Tx_Ranging_Config_Sent, //node sends range init
    Twr_Tx_Poll_Sent,           //tag sends poll
    Twr_Tx_Resp_Sent,           //node sends response
    Twr_Tx_Final_Sent,          //tag sends final
}tx_states_e;

/* Application tasks handles & corresponded signals structure */
struct task_signal_s
{
    // osThreadId Handle;           /* Tasks handler */
    TaskHandle_t Handle;            /* Tasks handler */
    // osMutexId  MutexId;          /* Tasks mutex */
    SemaphoreHandle_t MutexId;      /* Tasks mutex */
    int32_t    Signal;              /* Tasks signal */
    void        *arg;               /* additional parameters for the Task */
};

typedef struct task_signal_s    task_signal_t;

/* System mode of operation. used to
 *
 * 1. indicate in which mode of operation system is running
 * 2. configure the access rights to command handler in control mode
 * */
typedef enum {
    mANY = 0,   /**< Used only for Commands: indicates the command can be executed in any modes below */
    mIDLE,      /**< IDLE mode */
    mPNODE,     /**< PDOA Slotted TWR (Node active) mode */
    mPTAG,      /**< PDoA Slotted TWR (Tag active ) mode */
    mMASK      = 0xFFFF
}mode_e;




/* Application's global parameters structure */
struct app_s
{

    void (* hw_time_cb)(void); // pointer to the hardware timer callback
};

typedef struct app_s app_t;

/* global application variables */
extern app_t app;


#ifdef __cplusplus
}
#endif //APP_H_ 1


#endif //__DECA_APP_H