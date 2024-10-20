/**
 * @file    tag.c
 * @brief    Tag Application Layer
 *             TWR functions collection
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#include "tag.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_vals.h"
#include "common_n.h"
#include "util.h"

#define TWR_TAG_ENTER_CRITICAL()  taskENTER_CRITICAL(&task_mux)
#define TWR_TAG_EXIT_CRITICAL()   taskEXIT_CRITICAL(&task_mux)

//-----------------------------------------------------------------------------

// //dynamic allocation of TwrInfo
// static tag_info_t    *psTagInfo = NULL;

//static ("safe") implementation
static tag_info_t    sTagInfo;
static tag_info_t   *psTagInfo = &sTagInfo;

//-----------------------------------------------------------------------------
// Implementation

static void tag_hw_timer_cb();

//-----------------------------------------------------------------------------
// Support section

/*
 * @brief     get pointer to the twrInfo structure
 * */
tag_info_t *
getTagInfoPtr(void)
{
    return (psTagInfo);
}

//-----------------------------------------------------------------------------
//    DW3000 callbacks section :
//    if RTOS, the preemption priority of the dwt_isr() shall be such, that
//    allows signal to the thread.

/* @brief    ISR layer
 *             Real-time TWR application Tx callback
 *            to be called from dwt_isr()
 * */
static void
twr_tag_tx_cb(const dwt_cb_data_t *txd)
{

}

/* @brief     ISR layer
 *             TWR application Rx callback
 *             to be called from dwt_isr() as an Rx call-back
 * */
static void
twr_tag_rx_cb(const dwt_cb_data_t *rxd)
{

}

/*
 * @brief    ISR layer
 *
 * */
static void
twr_tag_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
    
}

static void
twr_tag_rx_error_cb(const dwt_cb_data_t *rxd)
{

}

/*
 * Called on SPI_RDY IRQ by deca_driver
 */
static void
tag_spi_rdy_cb(const dwt_cb_data_t *rxd)
{
    //TODO: implement
}


//-----------------------------------------------------------------------------

/* @brief     app layer
 *     RTOS independent application layer function.
 *     initialising of TWR from scratch.
 *     This MUST be executed in protected mode.
 *
 *     !!!! It is assumed DW IC is reset prior to calling this function !!!!!
 * 
 * */
error_e tag_process_init(void)
{
    tag_info_t *pTagInfo = getTagInfoPtr();

    if(!pTagInfo)
    {
        return(_ERR_Cannot_Alloc_Memory);
    }

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pTagInfo, 0 , sizeof(tag_info_t));

    if(pTagInfo->rx_pkt_queue == NULL)
    {
        pTagInfo->rx_pkt_queue = xQueueCreate(EVENT_BUF_TAG_SIZE, sizeof(tag_rx_pckt_t));
    }


    /* Hard code the known anchors for the demo */
    app.pConfig->known_anchor_list[0].eui16 = TWR_ANCHOR_MASTER_EUI16;
    app.pConfig->known_anchor_list[1].eui16 = TWR_ANCHOR_DEV1_EUI16;
    app.pConfig->known_anchor_list[2].eui16 = TWR_ANCHOR_DEV2_EUI16;
    app.pConfig->known_anchor_list[3].eui16 = TWR_ANCHOR_DEV3_EUI16;
    app.pConfig->known_anchor_list_size = 4;


    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    disable_dw3000_irq();

    TWR_TAG_ENTER_CRITICAL();

    if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID) != DWT_SUCCESS) /**< set callbacks to NULL inside dwt_initialise*/
    {
        TWR_TAG_EXIT_CRITICAL();
        Serial.println("INIT FAILED");
        return (_ERR_INIT);   // device initialise has failed
    }

    set_dw_spi_fast_rate();

    uint32_t dev_id = dwt_readdevid();

    if (dev_id != DWT_C0_DEV_ID) {
        TWR_TAG_EXIT_CRITICAL();
        Serial.println("Device ID is not correct");
        return (_ERR_INIT);
    }

    /* Configure receiver's UWB mode, set power and antenna delays for TWR mode */
    rxtx_configure_t p;
    p.pdwCfg      = &app.pConfig->dwt_config;
    p.frameFilter = DWT_FF_DISABLE;    //DWT_FF_ENABLE_802_15_4
    p.frameFilterMode   = (DWT_FF_DATA_EN | DWT_FF_ACK_EN); //FIXME
    p.txAntDelay  = app.pConfig->runtime_params.ant_tx_a;
    p.rxAntDelay  = app.pConfig->runtime_params.ant_rx_a;
    p.panId       = 0x5555;//PanID : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    p.shortadd    = 0xAAAA;//ShortAddr : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message

    rxtx_configure(&p);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(/*DWT_PA_ENABLE | DWT_LNA_ENABLE*/ DWT_TXRX_EN);  /**< DEBUG I/O 4&5&6 : configure LNA/PA, 0&1 TX/RX states to output on GPIOs */

    dwt_setcallbacks(
        twr_tag_tx_cb, 
        twr_tag_rx_cb, 
        twr_tag_rx_timeout_cb, 
        twr_tag_rx_error_cb, 
        NULL, 
        tag_spi_rdy_cb);

    dwt_setinterrupt(\
        (\
            DWT_INT_SPIRDY_BIT_MASK | \
            DWT_INT_ARFE_BIT_MASK   | \
            DWT_INT_TXFRS_BIT_MASK | \
            DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | \
            DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFTO_BIT_MASK
        ),\
        0,\
        DWT_ENABLE_INT_ONLY\
    );

    // dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_PRES_SLEEP| DWT_WAKE_CSN | DWT_SLP_EN);

    init_dw3000_irq();            /**< manually init EXTI DW3000 lines IRQs */

    /* configure non-zero initial values */
    pTagInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);


    /*
     * The dwt_initialize will read the default XTAL TRIM from the OTP or use the DEFAULT_XTAL_TRIM.
     * In this case we would apply the user-configured value.
     *
     * Bit 0x80 can be used to overwrite the OTP settings if any.
     * */
    if((dwt_getxtaltrim() == DEFAULT_XTAL_TRIM) || (app.pConfig->runtime_params.xtal_trim & ~XTAL_TRIM_BIT_MASK))
    {
        dwt_setxtaltrim(app.pConfig->runtime_params.xtal_trim& XTAL_TRIM_BIT_MASK);
        pTagInfo->xtaltrim = dwt_getxtaltrim();
    }

    pTagInfo->mode = tag_info_s::WAIT_FOR_TURN_MODE; // waiting for the turn

    TWR_TAG_EXIT_CRITICAL();
    
    return (_NO_ERR);
}


/*
 * */
void tag_process_start(void)
{
    enable_dw3000_irq();  /**< enable DW3000 IRQ to start  */

    // start timer
    timerAttachInterrupt(hwtimer, &tag_hw_timer_cb, true); 

    timerAlarmDisable(hwtimer);
    // hard-coding the start of the first blink in 10ms after starting of the Tag application
    timerAlarmWrite(hwtimer, 10000, false);
    timerAlarmEnable(hwtimer);
}

/* @brief   app level
 *          RTOS-independent application level function.
 *          deinitialize the pTwrInfo structure.
 *  This must be executed in protected mode.
 *
 * */
void tag_process_terminate(void)
{
    
}



//-----------------------------------------------------------------------------

/* @brief
 *          TWR : DISCOVERY PHASE
 *          Tag sends Blinks, waiting for a Ranging Config message
 *
 *          application layer function
 */
error_e tag_send_blink(tag_info_t *p)
{
    error_e       ret;


    return ret;
}


static void IRAM_ATTR tag_hw_timer_cb() {

    tag_info_t *pTagInfo = getTagInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // if (pTagInfo->mode == tag_info_s::BLINKING_MODE) 
    // {
    //     if(app.blinkTask.Handle)
    //     {
    //        vTaskNotifyGiveFromISR(app.blinkTask.Handle, &xHigherPriorityTaskWoken);
    //     }

    // }


    // Optionally, yield if the task has higher priority
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
}