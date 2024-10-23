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


#define ANCHOR_MASTER_WAIT_POLL_BROADCAST_TIMEOUT_US        (1000000)           /* timeout for waiting POLL BROADCAST message, us */
#define ANCHOR_WAIT_POLL_BROADCAST_TIMEOUT_US               (0)                 /* timeout for waiting POLL BROADCAST message, us */
#define ANCHOR_WAIT_FINAL_MSG_TIMEOUT_US                    (1000000)           /* timeout for waiting the final message after sending the response message, us */

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
        pAnchorInfo->expectedRxMsg = MSG_POLL_BROADCAST;  
        pAnchorInfo->lastTxMsg = MSG_NONE;
    }


    if (pAnchorInfo->lastTxMsg == MSG_RESP)
    {
        if (pAnchorInfo->isMaster)
        {
            pAnchorInfo->expectedRxMsg = MSG_END_TURN;
        }
        else
        {
            pAnchorInfo->expectedRxMsg = MSG_POLL;
        }
        pAnchorInfo->lastTxMsg = MSG_NONE;

        Serial.println("Sent response message");
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

    if(pAnchorInfo->isMaster && pAnchorInfo->mode == anchor_info_s::RANGING_MODE)
    {
        // // if timeout occurs, the Master Anchor waits for the next turn
        // pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE;
        // pAnchorInfo->expectedRxMsg = MSG_GIVING_TURN;
        // pAnchorInfo->lastTxMsg = MSG_NONE;

        // // signal the Anchor Master giving turn task
        // if(app.anchor_master_giving_turn_task.Handle)
        // {
        //     vTaskNotifyGiveFromISR(app.anchor_master_giving_turn_task.Handle, &xHigherPriorityTaskWoken);
        // }
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
    // timerAttachInterrupt(hwtimer, &anchor_hw_timer_cb, true); 

    if(psAnchorInfo->isMaster)
    {
        pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE; // at the beginning, the Master Anchor gives the turn to the first Tag
        xTaskNotifyGive(app.anchor_master_giving_turn_task.Handle);
    }
    else
    {
        pAnchorInfo->mode = anchor_info_s::RANGING_MODE; // at the beginning, the Tag waits for the turn
        // start waiting for the POLL BROADCAST message
        pAnchorInfo->expectedRxMsg = MSG_POLL_BROADCAST;
        pAnchorInfo->lastTxMsg = MSG_NONE;

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        dwt_setrxtimeout(util_us_to_sy(0));
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
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(ANCHOR_MASTER_WAIT_POLL_BROADCAST_TIMEOUT_US);   // RX timeout
    
    pAnchorInfo->seqNum++;
    pAnchorInfo->lastTxMsg = MSG_GIVING_TURN;

    // setup remain params in tx done callback


    ANCHOR_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    ANCHOR_EXIT_CRITICAL();

    if (ret != _NO_ERR)
    {
        Serial.println("Error in tx_start");
    }
    
    return ret;
}

error_e anchor_send_resp(anchor_info_t *pAnchorInfo, anchor_rx_pckt_t *pRxPckt) {
    error_e ret = _NO_ERR;
    tx_pckt_t txPckt;
    uint64_t pollBroadcastRxTs = 0;
    uint16_t respDelay = 0;
    uint64_t respTxTs = 0;

    memset(&txPckt, 0, sizeof(txPckt));

    // get info from the received poll broadcast message
    for(uint8_t i=0;i<pRxPckt->msg.poll_broadcast_msg.numAnchors;i++)
    {
        if(memcmp(pRxPckt->msg.poll_broadcast_msg.anchorSchedule[i].shortAddr, 
                pAnchorInfo->shortAddress.bytes, 
                sizeof(pAnchorInfo->shortAddress.bytes)) == 0)
        {
            respDelay = (pRxPckt->msg.poll_broadcast_msg.anchorSchedule[i].respTime[1] << 8) | pRxPckt->msg.poll_broadcast_msg.anchorSchedule[i].respTime[0];
            break;
        }
    }

    // setup transmit params
    txPckt.psduLen = sizeof(resp_msg_t);
    txPckt.msg.resp_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.resp_msg.mac.frameCtrl[1] = FC_2_SHORT;

    txPckt.msg.resp_msg.mac.seqNum = pAnchorInfo->seqNum;
    txPckt.msg.resp_msg.mac.panID[0] = (uint8_t)(pAnchorInfo->panID & 0xff);
    txPckt.msg.resp_msg.mac.panID[1] = (uint8_t)(pAnchorInfo->panID >> 8);

    // set the destination address
    memcpy(txPckt.msg.resp_msg.mac.destAddr, 
        pRxPckt->msg.poll_broadcast_msg.mac.sourceAddr, 
        sizeof(pRxPckt->msg.poll_broadcast_msg.mac.sourceAddr)
    );

    // set the source address
    memcpy(txPckt.msg.resp_msg.mac.sourceAddr, 
        pAnchorInfo->shortAddress.bytes, 
        sizeof(pAnchorInfo->shortAddress.bytes)
    );

    // set the message type
    txPckt.msg.resp_msg.msgType = MSG_RESP;
    memcpy(txPckt.msg.resp_msg.pollRxTs, pRxPckt->timeStamp, sizeof(pRxPckt->timeStamp));

    // set transmission parameters
    // tx immediately --> rx immediately --> rx timeout
    txPckt.txFlag               = ( DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(0);   // TX to RX delay
    // txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(0);   // RX timeout 

    // calculate the delayed time to respond
    TS2U64_MEMCPY(pollBroadcastRxTs, pRxPckt->timeStamp);
    txPckt.delayedTxTimeH_dt    = (pollBroadcastRxTs + util_us_to_dev_time(respDelay) ) >> 8; // only high 32 bits

    respTxTs = (((uint64_t)(txPckt.delayedTxTimeH_dt & 0xFFFFFFFEUL)) << 8) + app.pConfig->runtime_params.ant_tx_a;
    for(int i = 0; i < sizeof(txPckt.msg.resp_msg.respTxTs); i++)
    {
        txPckt.msg.resp_msg.respTxTs[i] = (uint8_t)(respTxTs >> (i*8));
    }

    if(pAnchorInfo->isMaster)
    {
        // get the last delay time
        uint16_t longestDelay = (uint16_t)((pRxPckt->msg.poll_broadcast_msg.anchorSchedule[pRxPckt->msg.poll_broadcast_msg.numAnchors-1].respTime[1] << 8) | 
                                        pRxPckt->msg.poll_broadcast_msg.anchorSchedule[pRxPckt->msg.poll_broadcast_msg.numAnchors-1].respTime[0]);
        txPckt.delayedRxTimeout_sy = (uint32_t)util_us_to_sy(longestDelay + 7000);   // RX timeout for end turn message
        // txPckt.delayedRxTimeout_sy = (uint32_t)util_us_to_sy(1000000); 
    }
    else
    {
        txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(0);   // no RX timeout, just wait for the next poll broadcast
    }

    pAnchorInfo->seqNum++;
    pAnchorInfo->lastTxMsg = MSG_RESP;


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
    // MSG_POLL_BROADCAST
    if( (pAnchorInfo->mode == anchor_info_s::GIVING_TURN_MODE) && 
        (pAnchorInfo->expectedRxMsg == MSG_POLL_BROADCAST)        
    ) {

        // print raw message
        for(int i = 0; i < pRxPckt->rxDataLen; i++)
        {
            Serial.print(pRxPckt->msg.raw[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // check conditions
        if( (pRxPckt->msg.poll_broadcast_msg.msgType != MSG_POLL_BROADCAST) || 
            ( (pRxPckt->msg.poll_broadcast_msg.mac.destAddr[0] != 0xff) && 
              (pRxPckt->msg.poll_broadcast_msg.mac.destAddr[1] != 0xff) )
        ) {
            // TODO: Error handling: wait for poll broadcast again
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
                // abort the ranging process and wait for next poll broadcast
                pAnchorInfo->mode = anchor_info_s::RANGING_MODE;
                pAnchorInfo->expectedRxMsg = MSG_POLL_BROADCAST;
                pAnchorInfo->lastTxMsg = MSG_NONE;

                // re-enable RX with timeout (0)
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                dwt_setrxtimeout(util_us_to_sy(ANCHOR_WAIT_POLL_BROADCAST_TIMEOUT_US));
            }

            Serial.println("Error in POLL Broadcast message");

            return _ERR; //TODO: handle error
        }

        Serial.println("POLL Broadcast received");

        pAnchorInfo->mode = anchor_info_s::RANGING_MODE; // switch to ranging mode

        // send response message
        if(anchor_send_resp(pAnchorInfo, pRxPckt) != _NO_ERR) {
            // error handling
            return _ERR;
        }

        return _NO_ERR;
    }

    // MSG_END_TURN
    if( (pAnchorInfo->mode == anchor_info_s::RANGING_MODE) && 
        (pAnchorInfo->expectedRxMsg == MSG_END_TURN)        
    ) {

        // print raw message
        for(int i = 0; i < pRxPckt->rxDataLen; i++)
        {
            Serial.print(pRxPckt->msg.raw[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        if( (pRxPckt->msg.end_turn_msg.msgType != MSG_END_TURN) || 
            (memcmp(pRxPckt->msg.end_turn_msg.mac.destAddr, 
                    pAnchorInfo->shortAddress.bytes, 
                    sizeof(pAnchorInfo->shortAddress.bytes)) != 0)
        ) {
            //TODO: Error handling
            return _ERR;
        }

        Serial.println("END TURN received");

        // back to giving turn mode
        pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE;
        pAnchorInfo->expectedRxMsg = MSG_GIVING_TURN;
        pAnchorInfo->lastTxMsg = MSG_NONE;

        xTaskNotifyGive(app.anchor_master_giving_turn_task.Handle);

        return _NO_ERR;
    }

    // no message matched 
    if(pAnchorInfo->isMaster) {
        // back to giving turn mode ?? TODO: check this
        pAnchorInfo->mode = anchor_info_s::GIVING_TURN_MODE;
        pAnchorInfo->expectedRxMsg = MSG_NONE;
        pAnchorInfo->lastTxMsg = MSG_NONE;

        xTaskNotifyGive(app.anchor_master_giving_turn_task.Handle);
    } 
    else {
        // abort the ranging process and wait for next poll broadcast
        pAnchorInfo->mode = anchor_info_s::RANGING_MODE;
        pAnchorInfo->expectedRxMsg = MSG_POLL_BROADCAST;
        pAnchorInfo->lastTxMsg = MSG_NONE;

        // re-enable RX with timeout (0)
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        dwt_setrxtimeout(util_us_to_sy(ANCHOR_WAIT_POLL_BROADCAST_TIMEOUT_US));
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
