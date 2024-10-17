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
    xTaskCreate(TagBlinkTask, "BlinkTask", 1024, NULL, TAG_TASK_BLINK_PRIO, NULL);

    xTaskCreate(TagPollTask, "TwrPollTask", 1024, NULL, TAG_TASK_TWR_POLL_PRIO, NULL);
    xTaskCreate(TagRxTask, "RxTask", 1024, NULL, TAG_TASK_RX_PRIO, NULL);
}

//-----------------------------------------------------------------------------

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



    taskEXIT_CRITICAL(&tagTaskMux);
}