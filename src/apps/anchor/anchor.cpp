/**
 * @brief Anchor Master bare implementation
 * 
 * @attention //TODO: Add attention here
 * 
 * @author Nguyen Ha Trung
 */



#include "anchor.h"
#include "deca_interface.h"
#include "common_n.h"
#include "util.h"

#define ANCHOR_ENTER_CRITICAL()  taskENTER_CRITICAL(&task_mux)
#define ANCHOR_EXIT_CRITICAL()   taskEXIT_CRITICAL(&task_mux)


static anchor_info_t    sAnchorInfo;
static anchor_info_t   *psAnchorInfo = &sAnchorInfo;

static void anchor_hw_timer_cb();


anchor_info_t * getAnchorInfoPtr(void)
{
    return (psAnchorInfo);
}

void anchor_tx_cb(const dwt_cb_data_t *txd){
    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();

    if (!pAnchorInfo) {
        return;
    }

    // Store the Tx timestamp of the sent packet
    if (pAnchorInfo->lastTxMsg == MSG_GIVING_TURN)
    {
        //TODO
    }

}

void anchor_rx_cb(const dwt_cb_data_t *rxd){

    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    anchor_rx_pckt_t rxPckt;

    if (!pAnchorInfo) {
        return;
    }

    dwt_readrxtimestamp(rxPckt.timeStamp);
    rxPckt.status = rxd->status;
    rxPckt.rxDataLen = rxd->datalength;

    dwt_readrxdata(rxPckt.msg.raw, rxd->datalength, 0);

    xQueueSendFromISR(pAnchorInfo->rxPcktQueue, &rxPckt, &xHigherPriorityTaskWoken);

    if(app.anchor_rx_task.Handle)
    {
        vTaskNotifyGiveFromISR(app.anchor_rx_task.Handle, &xHigherPriorityTaskWoken);
    }


}

void anchor_rx_timeout_cb(const dwt_cb_data_t *rxd){
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
}

void anchor_rx_error_cb(const dwt_cb_data_t *rxd){
    anchor_rx_timeout_cb(rxd);
}

error_e anchor_process_init(void) {
    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();

    if(!pAnchorInfo) 
    {
        return(_ERR_Cannot_Alloc_Memory);
    }

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pAnchorInfo, 0, sizeof(anchor_info_t));


    // Set the EUI16 for current anchor
    pAnchorInfo->eui16 = ANCHOR_ADDR_EUI16;

    if(pAnchorInfo->eui16 == TWR_ANCHOR_MASTER_EUI16)
    {
        pAnchorInfo->isMaster = true;
    }


    // Hardcoded EUI64s for demo 
    // TODO: fixed later
    if (pAnchorInfo->isMaster)
    {
        pAnchorInfo->tagList[0].eui16 = (uint16_t)TWR_TAG_DEV1_EUI16;
        pAnchorInfo->tagList[1].eui16 = (uint16_t)TWR_TAG_DEV2_EUI16;
        pAnchorInfo->curTagNum = 2;
        pAnchorInfo->curTagIdx = 0;
    }

    if(pAnchorInfo->rxPcktQueue == NULL)
    {
        pAnchorInfo->rxPcktQueue = xQueueCreate(ANCHOR_EVENT_BUF_SIZE, sizeof(anchor_rx_pckt_t));
    }

    pAnchorInfo->panID = ANCHOR_DEFAULT_PANID;


    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    disable_dw3000_irq();

    ANCHOR_ENTER_CRITICAL();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)  /**< set callbacks to NULL inside dwt_initialise*/
    {
        ANCHOR_EXIT_CRITICAL();
        return (_ERR_INIT);
    }

    set_dw_spi_fast_rate();

    uint32_t dev_id = dwt_readdevid();

    if (dev_id != DWT_C0_DEV_ID) {
        ANCHOR_EXIT_CRITICAL();
        return (_ERR_INIT);
    }

    /* read OTP Temperature calibration parameter */

    /* Configure DW IC's UWB mode, sets power and antenna delays for TWR mode */
    rxtx_configure_t p;
    p.pdwCfg      = &app.pConfig->dwt_config;
    p.frameFilter = DWT_FF_DISABLE;    //DWT_FF_ENABLE_802_15_4
    p.frameFilterMode   = (DWT_FF_DATA_EN | DWT_FF_ACK_EN); //FIXME
    p.txAntDelay  = app.pConfig->runtime_params.ant_tx_a;
    p.rxAntDelay  = app.pConfig->runtime_params.ant_rx_a;
    p.panId       = pAnchorInfo->panID;//PanID : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    p.shortadd    = pAnchorInfo->eui16;//ShortAddr

    rxtx_configure(&p);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_TXRX_EN);  /**< DEBUG I/O 0&1 : configure TX/RX states to output on GPIOs */

    dwt_setcallbacks(anchor_tx_cb, anchor_rx_cb, anchor_rx_timeout_cb, anchor_rx_error_cb, NULL, NULL);

    // Note: if no DWT_INT_SPIRDY_BIT_MASK, the system work unexpectedly
    // DWT_INT_SPIRDY_BIT_MASK is must-have
    dwt_setinterrupt(\
        (\
            DWT_INT_SPIRDY_BIT_MASK | \
            DWT_INT_TXFRS_BIT_MASK | \
            DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK | \
            DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFTO_BIT_MASK
        ),\
        0,\
        DWT_ENABLE_INT_ONLY\
    );

    init_dw3000_irq();            /**< manually init EXTI DW3000 lines IRQs */

    /* configure non-zero initial values */
    pAnchorInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);
    pAnchorInfo->expectedRxMsg = MSG_NONE;
    pAnchorInfo->lastTxMsg = MSG_NONE;


    /*
     * The dwt_initialize will read the default XTAL TRIM from the OTP or use the DEFAULT_XTAL_TRIM.
     * In this case we would apply the user-configured value.
     *
     * Bit 0x80 can be used to overwrite the OTP settings if any.
     * */
    if((dwt_getxtaltrim() == DEFAULT_XTAL_TRIM) || (app.pConfig->runtime_params.xtal_trim & ~XTAL_TRIM_BIT_MASK))
    {
        dwt_setxtaltrim(app.pConfig->runtime_params.xtal_trim& XTAL_TRIM_BIT_MASK);
    }

    ANCHOR_EXIT_CRITICAL();


    return _NO_ERR;
}

void anchor_process_start(void) {
    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();
    
    enable_dw3000_irq();  /**< enable DW3000 IRQ to start  */

    // start timer
    timerAttachInterrupt(hwtimer, &anchor_hw_timer_cb, true); 

    if(psAnchorInfo->isMaster)
    {
        pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE; // at the beginning, the Master Anchor gives the turn to the first Tag
        xTaskNotifyGive(app.anchor_master_giving_turn_task.Handle);
    }
    else
    {
        pAnchorInfo->mode = anchor_info_s::RANGING_MODE; // at the beginning, the Tag waits for the turn
        xTaskNotifyGive(app.anchor_rx_task.Handle);
    }
}


error_e anchor_master_give_turn(anchor_info_t *pAnchorInfo)
{
    error_e ret = _NO_ERR;

    tx_pckt_t txPckt;

    memset(&txPckt, 0, sizeof(txPckt));


    // setup transmit params
    txPckt.psduLen = sizeof(giving_turn_msg_t);
    txPckt.msg.giving_turn_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.giving_turn_msg.mac.frameCtrl[1] = FC_2_SHORT;

    txPckt.msg.giving_turn_msg.mac.seqNum = pAnchorInfo->seqNum;
    txPckt.msg.giving_turn_msg.mac.panID[0] = (uint8_t)(pAnchorInfo->panID & 0xff);
    txPckt.msg.giving_turn_msg.mac.panID[1] = (uint8_t)(pAnchorInfo->panID >> 8);

    // set the destination address
    txPckt.msg.giving_turn_msg.mac.destAddr[0] = (uint8_t)(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes[0]);
    txPckt.msg.giving_turn_msg.mac.destAddr[1] = (uint8_t)(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes[1]);

    // set the source address
    txPckt.msg.giving_turn_msg.mac.sourceAddr[0] = (uint8_t)(pAnchorInfo->eui16 & 0xff);
    txPckt.msg.giving_turn_msg.mac.sourceAddr[1] = (uint8_t)(pAnchorInfo->eui16 >> 8);

    // set the message type
    txPckt.msg.giving_turn_msg.message_type = MSG_GIVING_TURN;


    txPckt.txFlag               = ( DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED );
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(app.pConfig->runtime_params.rcDelay_us);  //Ranging Config: activate receiver this time SY after Blink Tx
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(app.pConfig->runtime_params.rcRxTo_us);   //Ranging Config: receiver will be active for this time, SY
    
    pAnchorInfo->seqNum++;
    pAnchorInfo->lastTxMsg = MSG_GIVING_TURN;
    pAnchorInfo->expectedRxMsg = MSG_ACK;    

    ANCHOR_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    ANCHOR_EXIT_CRITICAL();

    if (ret != _NO_ERR)
    {
        Serial.println("Error in tx_start");
    }
    
    return ret;
}

error_e anchor_process_rx_pckt(anchor_info_t *pAnchorInfo, anchor_rx_pckt_t *pRxPckt)
{
    error_e ret = _NO_ERR;

    if(pAnchorInfo->expectedRxMsg == MSG_ACK && pAnchorInfo->mode == anchor_info_s::GIVING_TURN_MODE)
    {
        if(pRxPckt->msg.ack_msg.message_type != MSG_ACK)
        {
            return _ERR; //TODO: handle error
        }

        Serial.println("ACK received");

        // disable ack timeout timer
        // timerAlarmDisable(hwtimer);
        // pAnchorInfo->mode == anchor_info_s::RANGING_MODE;
    
    }


    return ret;
}



static void IRAM_ATTR anchor_hw_timer_cb() {
    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(pAnchorInfo->isMaster && pAnchorInfo->mode == anchor_info_s::GIVING_TURN_MODE)
    {
        // if timeout occurs, the Master Anchor gives the turn to the next Tag
        pAnchorInfo->curTagIdx++;
        if (pAnchorInfo->curTagIdx >= pAnchorInfo->curTagNum)
        {
            pAnchorInfo->curTagIdx = 0; // reset the index
        }

        // signal the Anchor Master giving turn task
        if(app.anchor_master_giving_turn_task.Handle)
        {
            vTaskNotifyGiveFromISR(app.anchor_master_giving_turn_task.Handle, &xHigherPriorityTaskWoken);
        }
    }
}
