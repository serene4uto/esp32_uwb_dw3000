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


#define TAG_GIVING_TURN_ACK_DELAY_UUS     (1500)    /* delay before sending ACK after receiving Giving Turn message, us */


#define TAG_ENTER_CRITICAL()  taskENTER_CRITICAL(&task_mux)
#define TAG_EXIT_CRITICAL()   taskEXIT_CRITICAL(&task_mux)

//-----------------------------------------------------------------------------

// //dynamic allocation of TwrInfo
// static tag_info_t    *psTagInfo = NULL;

//static ("safe") implementation
static tag_info_t    sTagInfo;
static tag_info_t   *psTagInfo = NULL;

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
tag_tx_cb(const dwt_cb_data_t *txd)
{
    tag_info_t *pTagInfo = getTagInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(!pTagInfo)
    {
        return;
    }

    // Store the Tx timestamp of the sent packet
    if(pTagInfo->lastTxMsg == MSG_GIVING_TURN)
    {
        //TODO
    }

    if(pTagInfo->lastTxMsg == MSG_ACK)
    {
        pTagInfo->mode = tag_info_s::RANGING_MODE;
        pTagInfo->expectedRxMsg = MSG_NONE;
        pTagInfo->lastTxMsg = MSG_NONE;
        vTaskNotifyGiveFromISR(app.tagPollTask.Handle, &xHigherPriorityTaskWoken);
    }

    if(pTagInfo->lastTxMsg == MSG_POLL)
    {
        Serial.println("Poll sent");
    }

    
}

/* @brief     ISR layer
 *             TWR application Rx callback
 *             to be called from dwt_isr() as an Rx call-back
 * */
static void
tag_rx_cb(const dwt_cb_data_t *rxd)
{
    tag_info_t *pTagInfo = getTagInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    tag_rx_pckt_t rxPckt;

    if(!pTagInfo)
    {
        return;
    }

    dwt_readrxtimestamp(rxPckt.timeStamp);
    rxPckt.status = rxd->status;
    rxPckt.rxDataLen = rxd->datalength;
    dwt_readrxdata(rxPckt.msg.raw, rxd->datalength, 0);

    xQueueSendFromISR(pTagInfo->rxPktQueue, &rxPckt, &xHigherPriorityTaskWoken);

    if(app.tagRxTask.Handle)
    {
        vTaskNotifyGiveFromISR(app.tagRxTask.Handle, &xHigherPriorityTaskWoken);
    }
}

/*
 * @brief    ISR layer
 *
 * */
static void
tag_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
    tag_info_t *pTagInfo = getTagInfoPtr();

    if(!pTagInfo) {
        return;
    }

    if(pTagInfo->mode == tag_info_s::WAIT_FOR_TURN_MODE) {

    }

    if(pTagInfo->mode == tag_info_s::RANGING_MODE) {
        pTagInfo->mode = tag_info_s::WAIT_FOR_TURN_MODE;
    }

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

}

static void
tag_rx_error_cb(const dwt_cb_data_t *rxd)
{
    tag_rx_timeout_cb(rxd);
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
    tag_info_t *pTagInfo = NULL;

    // Allocate memory for the tag_info_t structure
    psTagInfo = &sTagInfo; //TODO: fix this

    pTagInfo = getTagInfoPtr(); // get pointer to the tag_info_t structure

    if(!pTagInfo)
    {
        return(_ERR_Cannot_Alloc_Memory);
    }

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pTagInfo, 0 , sizeof(tag_info_t));

    if(pTagInfo->rxPktQueue == NULL)
    {
        pTagInfo->rxPktQueue = xQueueCreate(TAG_EVENT_BUF_SIZE, sizeof(tag_rx_pckt_t));
    }


    /* Hard code the known anchors for the demo */
    pTagInfo->anchorList[0].eui16 = (uint16_t)TWR_ANCHOR_MASTER_EUI16;
    pTagInfo->anchorList[1].eui16 = (uint16_t)TWR_ANCHOR_DEV1_EUI16;
    pTagInfo->anchorList[2].eui16 = (uint16_t)TWR_ANCHOR_DEV2_EUI16;
    pTagInfo->anchorList[3].eui16 = (uint16_t)TWR_ANCHOR_DEV3_EUI16;
    pTagInfo->curAnchorNum = 4;
    pTagInfo->curAnchorIdx = 0; // anchor master position

    pTagInfo->panID = TAG_DEFAULT_PANID;

    /* set the short address */
    pTagInfo->shortAddress.eui16 = (uint16_t)TWR_TAG_DEV1_EUI16;

    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    disable_dw3000_irq();

    TAG_ENTER_CRITICAL();

    if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID) != DWT_SUCCESS) /**< set callbacks to NULL inside dwt_initialise*/
    {
        TAG_EXIT_CRITICAL();
        Serial.println("INIT FAILED");
        return (_ERR_INIT);   // device initialise has failed
    }

    set_dw_spi_fast_rate();

    uint32_t dev_id = dwt_readdevid();

    if (dev_id != DWT_C0_DEV_ID) {
        TAG_EXIT_CRITICAL();
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
    p.panId       = pTagInfo->panID;    //PanID : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    p.shortadd    = pTagInfo->shortAddress.eui16;   //ShortAddr : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message

    rxtx_configure(&p);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;     /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(/*DWT_PA_ENABLE | DWT_LNA_ENABLE*/ DWT_TXRX_EN);  /**< DEBUG I/O 4&5&6 : configure LNA/PA, 0&1 TX/RX states to output on GPIOs */

    dwt_setcallbacks(
        tag_tx_cb, 
        tag_rx_cb, 
        tag_rx_timeout_cb, 
        tag_rx_error_cb, 
        NULL, 
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

    pTagInfo->mode = tag_info_s::WAIT_FOR_TURN_MODE; // waiting for the turn at the beginning
    pTagInfo->expectedRxMsg = MSG_GIVING_TURN; 
    pTagInfo->lastTxMsg = MSG_NONE;

    TAG_EXIT_CRITICAL();
    
    return (_NO_ERR);
}


/*
 * */
void tag_process_start(void)
{
    enable_dw3000_irq();  /**< enable DW3000 IRQ to start  */

    // start timer
    // timerAttachInterrupt(hwtimer, &tag_hw_timer_cb, true); 

    // timerAlarmDisable(hwtimer);
    // // hard-coding the start of the first blink in 10ms after starting of the Tag application
    // timerAlarmWrite(hwtimer, 10000, false);
    // timerAlarmEnable(hwtimer);
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

error_e tag_respond_ack(tag_info_t *pTagInfo, tag_rx_pckt_t *pRxPckt)
{
    error_e       ret = _NO_ERR;
    uint64_t      u64RxTs;
    tx_pckt_t txPckt;

    memset(&txPckt, 0, sizeof(txPckt));

    txPckt.psduLen = sizeof(ack_msg_t);
    txPckt.msg.ack_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.ack_msg.mac.frameCtrl[1] = FC_2_SHORT;
    txPckt.msg.ack_msg.mac.seqNum = pTagInfo->seqNum;

    txPckt.msg.ack_msg.mac.panID[0] = (uint8_t)(pTagInfo->panID & 0xff);
    txPckt.msg.ack_msg.mac.panID[1] = (uint8_t)(pTagInfo->panID >> 8);

    // set the destination address as current target Anchor
    memcpy(txPckt.msg.ack_msg.mac.destAddr, pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes, sizeof(pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes));

    // set the source address as the Tag current address
    memcpy(txPckt.msg.ack_msg.mac.sourceAddr, pTagInfo->shortAddress.bytes, sizeof(pTagInfo->shortAddress.bytes));

    txPckt.msg.ack_msg.message_type = MSG_ACK;

    txPckt.txFlag               = ( DWT_START_TX_DELAYED );
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(0);
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(0);
    

    // calculate the delayed time to respond
    TS2U64_MEMCPY(u64RxTs, pRxPckt->timeStamp);

    // add delay
    txPckt.delayedTxTimeH_dt    = (u64RxTs + util_us_to_dev_time(TAG_GIVING_TURN_ACK_DELAY_UUS) ) >> 8; // only high 32 bits

    pTagInfo->seqNum++;
    pTagInfo->lastTxMsg = MSG_ACK;

    TAG_ENTER_CRITICAL();
    ret = tx_start(&txPckt);
    TAG_EXIT_CRITICAL();

    if(ret != _NO_ERR)
    {
        Serial.println("ACK TX failed");
    }

    return ret;
}

error_e tag_send_poll(tag_info_t *pTagInfo) {
    error_e ret = _NO_ERR;
    tx_pckt_t txPckt;

    memset(&txPckt, 0, sizeof(txPckt));

    txPckt.psduLen = sizeof(poll_msg_t);
    txPckt.msg.poll_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.poll_msg.mac.frameCtrl[1] = FC_2_SHORT;

    txPckt.msg.poll_msg.mac.seqNum = pTagInfo->seqNum;
    txPckt.msg.poll_msg.mac.panID[0] = (uint8_t)(pTagInfo->panID & 0xff);
    txPckt.msg.poll_msg.mac.panID[1] = (uint8_t)(pTagInfo->panID >> 8);

    // set the destination address
    memcpy(txPckt.msg.poll_msg.mac.destAddr, 
        pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes, 
        sizeof(pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes)
    );

    // set the source address
    memcpy(txPckt.msg.poll_msg.mac.sourceAddr, 
        pTagInfo->shortAddress.bytes, 
        sizeof(pTagInfo->shortAddress.bytes)
    );

    // set the message type
    txPckt.msg.poll_msg.msgType = MSG_POLL;

    // set transmission parameters
    // tx immediately --> rx immediately --> rx timeout

    txPckt.txFlag               = ( DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED );
    // txPckt.txFlag               = ( DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );
    txPckt.delayedTxTimeH_dt    = (uint32_t)util_us_to_sy(0);   // delayed TX time
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(0);   // TX to RX delay
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(1000000);   // RX timeout

    pTagInfo->seqNum++;
    pTagInfo->lastTxMsg = MSG_POLL;


    TAG_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    TAG_EXIT_CRITICAL();

    // if (ret != _NO_ERR)
    // {
    //     Serial.println("Error in tx_start");
    // }
    
    return ret;
}


error_e tag_process_rx_pkt(tag_info_t *pTagInfo, tag_rx_pckt_t *pRxPckt)
{
    // ----------------------------------------------------------------------------
    // Wait for turn phase
    if ((pTagInfo->mode == tag_info_t::WAIT_FOR_TURN_MODE) && 
        (pTagInfo->expectedRxMsg == MSG_GIVING_TURN)
    ) {
        // Check if the message type is MSG_GIVING_TURN
        if ((pRxPckt->msg.giving_turn_msg.message_type != MSG_GIVING_TURN ) ||
            (memcmp(pRxPckt->msg.giving_turn_msg.mac.destAddr, 
                    pTagInfo->shortAddress.bytes, 
                    sizeof(pTagInfo->shortAddress.bytes)) != 0) ||
            (memcmp(pRxPckt->msg.giving_turn_msg.mac.sourceAddr, 
                    pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes, 
                    sizeof(pTagInfo->anchorList[pTagInfo->curAnchorIdx].bytes)) != 0)
        ) {

            dwt_writefastCMD(CMD_TXRXOFF);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            dwt_setrxtimeout(0);
            return _ERR;
        }

        Serial.println("Giving Turn Received");
        // Print raw message
        for (int i = 0; i < pRxPckt->rxDataLen; i++) {
            Serial.print(pRxPckt->msg.raw[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Send ACK
        if (tag_respond_ack(pTagInfo, pRxPckt) != _NO_ERR) {
            // Error handling
            dwt_writefastCMD(CMD_TXRXOFF);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            dwt_setrxtimeout(0);
            return _ERR;
        }

        // Set up ranging phase in tx done callback


        return _NO_ERR;
    }

    // ----------------------------------------------------------------------------
    if (pTagInfo->expectedRxMsg == MSG_RESP) {
        // TODO: implement this

    }


    // ----------------------------------------------------------------------------
    if (pTagInfo->expectedRxMsg == MSG_REPORT) {
        //TODO: implement this
    }

    return _NO_ERR;
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