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

#define ANCHOR_GIVING_TURN_ACK_TIMEOUT_US               (1000000)          /* timeout for waiting ACK after sending Giving Turn message, us */
#define ANCHOR_MASTER_POLL_TIMEOUT_US                   (1000000)       /* timeout for waiting POLL after finishing Giving turn phase, us */
#define ANCHOR_POLL_TIMEOUT_US                          (0)          /* timeout for waiting POLL */

#define ANCHOR_ENTER_CRITICAL()  taskENTER_CRITICAL(&task_mux)
#define ANCHOR_EXIT_CRITICAL()   taskEXIT_CRITICAL(&task_mux)


static anchor_info_t    sAnchorInfo;
static anchor_info_t   *psAnchorInfo = &sAnchorInfo;

static void anchor_hw_timer_cb();


anchor_info_t * getAnchorInfoPtr(void)
{
    return (psAnchorInfo);
}

static
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


    if (pAnchorInfo->lastTxMsg == MSG_RESP)
    {
        //TODO
    }


}

static
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

static
void anchor_rx_timeout_cb(const dwt_cb_data_t *rxd){

    anchor_info_t *pAnchorInfo = getAnchorInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!pAnchorInfo) {
        return;
    }

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

static
void anchor_rx_error_cb(const dwt_cb_data_t *rxd){

    //TODO: implement this function
    anchor_rx_timeout_cb(rxd);
    
}

static
void anchor_spi_rdy_cb(const dwt_cb_data_t *rxd){

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
    pAnchorInfo->shortAddress.eui16 = ANCHOR_ADDR_EUI16;

    if(pAnchorInfo->shortAddress.eui16 == TWR_ANCHOR_MASTER_EUI16)
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
    p.panId       = pAnchorInfo->panID;                 //PanID : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    p.shortadd    = pAnchorInfo->shortAddress.eui16;    //ShortAddr

    rxtx_configure(&p);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_TXRX_EN);  /**< DEBUG I/O 0&1 : configure TX/RX states to output on GPIOs */

    dwt_setcallbacks(\
        anchor_tx_cb, \
        anchor_rx_cb, \
        anchor_rx_timeout_cb, \
        anchor_rx_error_cb, \
        NULL, \
        NULL);


    dwt_setinterrupt(\
        (
            DWT_INT_TXFRS_BIT_MASK 
            | DWT_INT_RXFCG_BIT_MASK 
            | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK 
            | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK 
            | DWT_INT_RXFTO_BIT_MASK
        ),\
        0,\
        DWT_ENABLE_INT_ONLY\
    );

    init_dw3000_irq();            /**< manually init EXTI DW3000 lines IRQs */

    /* configure non-zero initial values */
    pAnchorInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);


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

    pAnchorInfo->expectedRxMsg = MSG_NONE;
    pAnchorInfo->lastTxMsg = MSG_NONE;

    ANCHOR_EXIT_CRITICAL();

    // print current anchor address
    Serial.print("Current Anchor: ");
    for(int i = 0; i < sizeof(pAnchorInfo->shortAddress.bytes); i++)
    {
        Serial.print(pAnchorInfo->shortAddress.bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // print current target tag address
    Serial.print("Current Tag: ");
    for(int i = 0; i < sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes); i++)
    {
        Serial.print(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

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
    memcpy(txPckt.msg.giving_turn_msg.mac.destAddr, 
        pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes, 
        sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes)
    );

    // set the source address
    memcpy(txPckt.msg.giving_turn_msg.mac.sourceAddr, 
        pAnchorInfo->shortAddress.bytes, 
        sizeof(pAnchorInfo->shortAddress.bytes)
    );

    // set the message type
    txPckt.msg.giving_turn_msg.message_type = MSG_GIVING_TURN;


    // set transmission parameters
    // tx immediately --> rx immediately --> rx timeout
    txPckt.txFlag               = ( DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED );
    txPckt.delayedTxTimeH_dt    = (uint32_t)util_us_to_sy(0);   // delayed TX time
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(0);   // TX to RX delay
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(ANCHOR_GIVING_TURN_ACK_TIMEOUT_US);   // RX timeout
    
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

    // TODO: optimize conditions
    if( (pAnchorInfo->expectedRxMsg == MSG_ACK) && 
        (pAnchorInfo->mode == anchor_info_s::GIVING_TURN_MODE) &&
        pAnchorInfo->isMaster
    ) {
        // check conditions
        if( (pRxPckt->msg.ack_msg.message_type != MSG_ACK) ||
            (memcmp(pRxPckt->msg.ack_msg.mac.destAddr, 
                    pAnchorInfo->shortAddress.bytes, 
                    sizeof(pAnchorInfo->shortAddress.bytes)) != 0) ||
            (memcmp(pRxPckt->msg.ack_msg.mac.sourceAddr, 
                    pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes, 
                    sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes)) != 0)
        )
        {
            // Error handling: wait for ack again
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            dwt_setrxtimeout(util_us_to_sy(ANCHOR_GIVING_TURN_ACK_TIMEOUT_US)); 
            return _ERR; //TODO: handle error
        }
        
        Serial.println("ACK received");

        // print raw message
        for(int i = 0; i < pRxPckt->rxDataLen; i++)
        {
            Serial.print(pRxPckt->msg.raw[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // ranging using poll message
        pAnchorInfo->mode == anchor_info_s::RANGING_MODE;
        pAnchorInfo->expectedRxMsg = MSG_POLL;
        pAnchorInfo->lastTxMsg = MSG_NONE;

        // wait for POLL message
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        // dwt_setrxtimeout(util_us_to_sy(ANCHOR_MASTER_POLL_TIMEOUT_US));
        dwt_setrxtimeout(util_us_to_sy(0));
        
        return _NO_ERR;
    }

    // ----------------------------------------------------------------------------
    // Ranging phase

    // MSG_POLL
    if( (pAnchorInfo->mode == anchor_info_s::RANGING_MODE) && 
        (pAnchorInfo->expectedRxMsg == MSG_POLL)        
    ) {
        // check conditions
        if( (pRxPckt->msg.poll_msg.msgType != MSG_POLL) || 
            (memcmp(pRxPckt->msg.poll_msg.mac.destAddr,
                    pAnchorInfo->shortAddress.bytes, 
                    sizeof(pAnchorInfo->shortAddress.bytes)) != 0) ||
            (memcmp(pRxPckt->msg.poll_msg.mac.sourceAddr, 
                    pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes, 
                    sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes)) != 0)
        ) {
            // Error handling: wait for poll again
            if(pAnchorInfo->isMaster) {
                // back to giving turn mode
                pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE; 
                pAnchorInfo->expectedRxMsg = MSG_NONE;
                pAnchorInfo->lastTxMsg = MSG_NONE;
                // delay for a random 1-2 seconds
                vTaskDelay(pdMS_TO_TICKS(1000 + (rand() % 1000)));
                // give turn to the next tag
                pAnchorInfo->curTagIdx++;
                if (pAnchorInfo->curTagIdx >= pAnchorInfo->curTagNum) {
                    pAnchorInfo->curTagIdx = 0; // reset the index
                }
                // signal the Anchor Master giving turn task
                xTaskNotifyGive(app.anchor_master_giving_turn_task.Handle); 
            } 
            else {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                dwt_setrxtimeout(util_us_to_sy(ANCHOR_POLL_TIMEOUT_US));
            }

            Serial.println("Error in POLL message");

            return _ERR; //TODO: handle error
        }

        Serial.println("POLL received");
        // print raw message
        for(int i = 0; i < pRxPckt->rxDataLen; i++)
        {
            Serial.print(pRxPckt->msg.raw[i], HEX);
            Serial.print(" ");
        }

        return _NO_ERR;
    }

    // MSG_FINAL
    if( (pAnchorInfo->mode == anchor_info_s::RANGING_MODE) && 
        (pAnchorInfo->expectedRxMsg == MSG_FINAL)        
    ) {
        // check conditions
        // if( (pRxPckt->msg.final_msg.msgType != MSG_FINAL) || 
        //     (memcmp(pRxPckt->msg.final_msg.mac.destAddr,
        //             pAnchorInfo->shortAddress.bytes, 
        //             sizeof(pAnchorInfo->shortAddress.bytes)) != 0) ||
        //     (memcmp(pRxPckt->msg.final_msg.mac.sourceAddr, 
        //             pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes, 
        //             sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes)) != 0)
        // ) {
        //     // Error handling: wait for final again
        //     dwt_rxenable(DWT_START_RX_IMMEDIATE);
        //     dwt_setrxtimeout(util_us_to_sy(ANCHOR_POLL_TIMEOUT_US));
        //     return _ERR; //TODO: handle error
        // }

        // Serial.println("FINAL received");
        // // print raw message
        // for(int i = 0; i < pRxPckt->rxDataLen; i++)
        // {
        //     Serial.print(pRxPckt->msg.raw[i], HEX);
        //     Serial.print(" ");
        // }

        // return _NO_ERR;
    }

    // MSG_END_TURN
    if( (pAnchorInfo->mode == anchor_info_s::RANGING_MODE) && 
        (pAnchorInfo->expectedRxMsg == MSG_END_TURN)        
    ) {
        // check conditions
        // if( (pRxPckt->msg.end_turn_msg.msgType != MSG_END_TURN) || 
        //     (memcmp(pRxPckt->msg.end_turn_msg.mac.destAddr,
        //             pAnchorInfo->shortAddress.bytes, 
        //             sizeof(pAnchorInfo->shortAddress.bytes)) != 0) ||
        //     (memcmp(pRxPckt->msg.end_turn_msg.mac.sourceAddr, 
        //             pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes, 
        //             sizeof(pAnchorInfo->tagList[pAnchorInfo->curTagIdx].bytes)) != 0)
        // ) {
        //     // Error handling: wait for end turn again
        //     dwt_rxenable(DWT_START_RX_IMMEDIATE);
        //     dwt_setrxtimeout(util_us_to_sy(ANCHOR_POLL_TIMEOUT_US));
        //     return _ERR; //TODO: handle error
        // }

        // Serial.println("END_TURN received");
        // // print raw message
        // for(int i = 0; i < pRxPckt->rxDataLen; i++)
        // {
        //     Serial.print(pRxPckt->msg.raw[i], HEX);
        //     Serial.print(" ");
        // }

        // return _NO_ERR;
    }


    return _NO_ERR;
}



static void IRAM_ATTR anchor_hw_timer_cb() {
    // anchor_info_t *pAnchorInfo = getAnchorInfoPtr();
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // if(pAnchorInfo->isMaster && pAnchorInfo->mode == anchor_info_s::GIVING_TURN_MODE)
    // {
    //     // if timeout occurs, the Master Anchor gives the turn to the next Tag
    //     pAnchorInfo->curTagIdx++;
    //     if (pAnchorInfo->curTagIdx >= pAnchorInfo->curTagNum)
    //     {
    //         pAnchorInfo->curTagIdx = 0; // reset the index
    //     }

    //     // signal the Anchor Master giving turn task
    //     if(app.anchor_master_giving_turn_task.Handle)
    //     {
    //         vTaskNotifyGiveFromISR(app.anchor_master_giving_turn_task.Handle, &xHigherPriorityTaskWoken);
    //     }
    // }
}
