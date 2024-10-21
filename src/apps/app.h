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
#include "default_config.h"


#ifdef __cplusplus
 extern "C" {
#endif

#define MASK_TXDTS            (0x00FFFFFFFE00ULL)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

/**
 * @brief Structure representing the context of a task.
 *
 * This structure encapsulates all necessary information required to manage and synchronize a task.
 * It includes the task's handle, a mutex for synchronization, and a pointer to any additional
 * parameters that the task might need.
 */
struct task_context_s
{
    TaskHandle_t Handle;            /* Tasks handler */
    SemaphoreHandle_t MutexId;      /* Tasks mutex */
    void        *arg;               /* additional parameters for the Task */
};

typedef struct task_context_s task_context_t;


/* Application's global parameters structure */
struct app_s
{
    param_block_t *pConfig;       /**< Current configuration */

    task_context_t anchor_rx_task; /* Anchor Rx task */
    task_context_t anchor_master_giving_turn_task; /* Anchor giving turn task */

    task_context_t tagRxTask;    /* Tag Rx task */
    task_context_t tagPollTask;  /* Tag Poll task */

    
    // void (* hw_time_cb)(void); // pointer to the hardware timer callback
};

typedef struct app_s app_t;

/* global application variables */
extern app_t app;


#ifdef __cplusplus
}
#endif //APP_H_ 1


#endif //__DECA_APP_H