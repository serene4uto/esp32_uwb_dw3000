/**
 * @file      task_tag.c
 * @brief     Tag Application Layer
 *            RTOS tag implementation
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: Add attention here
 *
 */

#include "task_tag.h"
#include "port.h"
#include "tag.h"


#define TAG_TASK_BLINK_PRIO    (5)
#define TAG_TASK_TWR_POLL_PRIO (10)
#define TAG_TASK_RX_PRIO       (10)

#define BLINK_PERIOD_MS            (500)    /* range init phase - Blink send period, ms */

portMUX_TYPE tagTaskMux = portMUX_INITIALIZER_UNLOCKED;

//-----------------------------------------------------------------------------

/*
 * @brief
 *     The thread is initiating the transmission of the blink
 *     on reception of app.blinkTask.Signal
 *
 * */
static void
TagBlinkTask(void * arg)
{
    tag_info_t     *pTagInfo;
    uint32_t notifyValue = 0;

    while(!(pTagInfo = getTagInfoPtr()))  //wait for initialisation of psTagInfo
    {
        vTaskDelay(100);
    }

    while(1)
    {
        xSemaphoreGive(app.blinkTask.MutexId);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the signal

        xSemaphoreTake(app.blinkTask.MutexId, portMAX_DELAY);  //we do not want the task can be deleted in the middle of operation

        Serial.println("Blinking");

        tag_send_blink(pTagInfo); // send the blink

        /* set the next blink ms period */
        uint32_t new_blink_period_ms;
        new_blink_period_ms = (uint32_t)((float)((TWR_TAG_BLINK_PERIOD_MS/3.0)*rand()/RAND_MAX)); 
        new_blink_period_ms += TWR_TAG_BLINK_PERIOD_MS;

        timerAlarmDisable(hwtimer);
        timerAlarmWrite(hwtimer, new_blink_period_ms*1000, false);
        timerAlarmEnable(hwtimer);

        
    }
}

/*
 * @brief
 *     The thread is initiating the TWR sequence
 *      on reception of .signal.twrTxPoll
 *
 * */
static void
TagPollTask(void * arg)
{
    
}


/* @brief DW3000 RX : RTOS implementation
 *
 * */
static void
TagRxTask(void * arg)
{
    
}


/* @brief Setup TWR tasks and timers for discovery phase.
 *         - blinking timer
 *         - blinking task
 *          - twr polling task
 *         - rx task
 * Only setup, do not start.
 * */
static void tag_setup_tasks(void)
{
    xTaskCreate(TagBlinkTask, "BlinkTask", 1024, NULL, TAG_TASK_BLINK_PRIO, &app.blinkTask.Handle);
    app.blinkTask.MutexId = xSemaphoreCreateMutex();

    // xTaskCreate(TagPollTask, "PollTask", 1024, NULL, TAG_TASK_TWR_POLL_PRIO, &app.pollTask.Handle);
    // app.pollTask.MutexId = xSemaphoreCreateMutex();

    // xTaskCreate(TagRxTask, "RxTask", 1024, NULL, TAG_TASK_RX_PRIO, &app.rxTask.Handle);
    // app.rxTask.MutexId = xSemaphoreCreateMutex();

    // if( (app.blinkTask.Handle == NULL)   ||\
    //     (app.pollTask. Handle == NULL)   ||\
    //     (app.rxTask.   Handle == NULL))
    // {
    //     //TODO: handle error
    // }

    // release the mutexes
    xSemaphoreGive(app.blinkTask.MutexId);
    // xSemaphoreGive(app.pollTask.MutexId);
    // xSemaphoreGive(app.rxTask.MutexId);
}

//-----------------------------------------------------------------------------

/* @brief
 *      Kill all task and timers related to tag/TWR if any
 *      DW3000's RX and IRQ shall be switched off before task termination,
 *      that IRQ will not produce unexpected Signal
 * */
void tag_terminate(void)
{
    timerAlarmDisable(hwtimer); // stop the timer

    // terminate the tasks
    TERMINATE_STD_TASK(app.blinkTask);
    TERMINATE_STD_TASK(app.pollTask);
    TERMINATE_STD_TASK(app.rxTask);

    tag_process_terminate();    //de-allocate Tag RAM Resources
}


/* @fn      tag_helper
 * @brief   this is a service function which starts the Tag
 *          top-level  application.
 *    Note: If the dynamic memory allocation is used, then the
 *          tag_process_init() will allocate the memory of sizeof(tag_info_t)
 *          from the <b>caller's</b> task stack, see _malloc_r() !
 * */
void tag_helper(void const *argument)
{
    error_e err;

    port_disable_dw_irq_and_reset(1);

    taskENTER_CRITICAL(&tagTaskMux);

    set_dw_spi_fast_rate();

    tag_setup_tasks();

    err = tag_process_init();

    if (err != _NO_ERR)
    {
        //TODO: handle error
    }
    
    taskEXIT_CRITICAL(&tagTaskMux);
    
    tag_process_start(); // should be called outside of the critical section
}