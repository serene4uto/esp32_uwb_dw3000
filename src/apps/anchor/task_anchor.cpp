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

//-----------------------------------------------------------------------------
portMUX_TYPE anchorTaskMux = portMUX_INITIALIZER_UNLOCKED;


static
void anchor_token_give_task(void *arg)
{

    
    while (1)
    {

    }
}

static
void anchor_rx_task(void *arg)
{
    while (1)
    {

    }
}


static
void anchor_setup_tasks(void) 
{

}


//-----------------------------------------------------------------------------

void anchor_helper(void const *argument)
{
    error_e err;

    port_disable_dw_irq_and_reset(1);

    taskENTER_CRITICAL(&anchorTaskMux);

    set_dw_spi_fast_rate();

    anchor_setup_tasks();

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