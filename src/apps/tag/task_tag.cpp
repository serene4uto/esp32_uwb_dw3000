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


#define TAG_TASK_BLINK_PRIO             (5)
#define TAG_TASK_END_TURN_PRIO          (10)
#define TAG_TASK_RX_PRIO                (10)

#define TAG_BLINK_PERIOD_MS             (500)    /* range init phase - Blink send period, ms */

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

}


static void
tag_end_turn_task(void * arg)
{
    tag_info_t *pTagInfo;

    while(!(pTagInfo = getTagInfoPtr()))  //wait for initialisation of psTagInfo
    {
        vTaskDelay(100);
    }

    while(1)
    {
        xSemaphoreGive(app.tagEndTurnTask.MutexId);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the signal

        xSemaphoreTake(app.tagEndTurnTask.MutexId, portMAX_DELAY);  //we do not want the task can be deleted in the middle of operation

        tag_send_end_turn(pTagInfo); // send the end turn message
    }
}


/* @brief DW3000 RX : RTOS implementation
 *
 * */
static void
tag_rx_task(void * arg)
{
    tag_info_t     *pTagInfo;
    tag_rx_pckt_t  rxPckt;

    while(!(pTagInfo = getTagInfoPtr()))  //wait for initialisation of psTagInfo
    {
        vTaskDelay(100);
    }

    while(1)
    {
        xSemaphoreGive(app.tagRxTask.MutexId);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the signal

        xSemaphoreTake(app.tagRxTask.MutexId, portMAX_DELAY);  //we do not want the task can be deleted in the middle of operation

        xQueueReceive(pTagInfo->rxPktQueue, &rxPckt, portMAX_DELAY);

        tag_process_rx_pkt(pTagInfo, &rxPckt);
    }
}


static void tag_setup_tasks(void)
{
    // xTaskCreate(TagBlinkTask, "BlinkTask", 1024, NULL, TAG_TASK_BLINK_PRIO, &app.blinkTask.Handle);
    // app.blinkTask.MutexId = xSemaphoreCreateMutex();

    xTaskCreate(tag_end_turn_task, "TagEndTurnTask", 1024, NULL, TAG_TASK_END_TURN_PRIO, &app.tagEndTurnTask.Handle);
    app.tagEndTurnTask.MutexId = xSemaphoreCreateMutex();

    xTaskCreate(tag_rx_task, "TagRxTask", 4096, NULL, TAG_TASK_RX_PRIO, &app.tagRxTask.Handle);
    app.tagRxTask.MutexId = xSemaphoreCreateMutex();

    if( (app.tagEndTurnTask.Handle == NULL)   ||
        (app.tagRxTask.Handle == NULL))
    {
        //TODO: handle error
    }

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
    TERMINATE_STD_TASK(app.tagRxTask);
    TERMINATE_STD_TASK(app.tagEndTurnTask);

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

    tag_setup_tasks();

    set_dw_spi_fast_rate();

    err = tag_process_init();

    if (err != _NO_ERR)
    {
        //TODO: handle error
    }
    
    tag_process_start(); // should be called outside of the critical section
}