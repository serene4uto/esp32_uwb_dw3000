/**
 * @brief Task for Anchor Master
 * 
 * @attention //TODO: Add attention here
 * 
 * @author Nguyen Ha Trung
 */


#include "port.h"   
#include "anchor.h"
#include "task_anchor.h"


#define ANCHOR_TASK_RX_PRIO (10)
#define ANCHOR_TASK_TOKEN_PRIO (5)

#define ANCHOR_GIVING_TURN_TIMEOUT_MS (1000) 

//-----------------------------------------------------------------------------
portMUX_TYPE anchorTaskMux = portMUX_INITIALIZER_UNLOCKED;


static
void anchor_master_giving_turn_task(void *arg)
{
    anchor_info_t *pAnchorInfo;

    while (!(pAnchorInfo = getAnchorInfoPtr())) // wait until the AnchorInfo is initialized
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        xSemaphoreGive(app.anchor_master_giving_turn_task.MutexId);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the signal

        xSemaphoreTake(app.anchor_master_giving_turn_task.MutexId, portMAX_DELAY); // we do not want the task can be deleted in the middle of operation

        anchor_master_give_turn(pAnchorInfo);

        // Serial.print("Giving turn to Tag: ");
        // for(int i = 0; i < sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes); i++)
        // {
        //     Serial.print(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
    }
}

static
void anchor_rx_task(void *arg)
{
    anchor_info_t *pAnchorInfo;
    anchor_rx_pckt_t rxPckt;

    while (!(pAnchorInfo = getAnchorInfoPtr())) // wait until the AnchorInfo is initialized
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        xSemaphoreGive(app.anchor_rx_task.MutexId);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the signal

        xSemaphoreTake(app.anchor_rx_task.MutexId, portMAX_DELAY); // we do not want the task can be deleted in the middle of operation

        xQueueReceive(pAnchorInfo->rxPcktQueue, &rxPckt, portMAX_DELAY);

        anchor_process_rx_pckt(pAnchorInfo, &rxPckt);

    }
}


static
void anchor_setup_tasks(void) 
{
    xTaskCreate(anchor_rx_task, "AnchorRxTask", 1024, NULL, ANCHOR_TASK_RX_PRIO, &app.anchor_rx_task.Handle);
    app.anchor_rx_task.MutexId = xSemaphoreCreateMutex();

    xTaskCreate(anchor_master_giving_turn_task, "AnchorMasterGivingTurnTask", 1024, NULL, ANCHOR_TASK_TOKEN_PRIO, &app.anchor_master_giving_turn_task.Handle);
    app.anchor_master_giving_turn_task.MutexId = xSemaphoreCreateMutex();

    if ((app.anchor_rx_task.Handle == NULL) || (app.anchor_master_giving_turn_task.Handle == NULL))
    {
        //TODO: handle error
    }

}


//-----------------------------------------------------------------------------

void anchor_helper(void const *argument)
{
    error_e err;

    port_disable_dw_irq_and_reset(1);

    anchor_setup_tasks();

    taskENTER_CRITICAL(&anchorTaskMux);

    set_dw_spi_fast_rate();

    err = anchor_process_init();

    if (err != _NO_ERR)
    {
        //TODO: handle error
    }

    taskEXIT_CRITICAL(&anchorTaskMux);

    anchor_process_start();
}



void anchor_terminate(void)
{
    
}