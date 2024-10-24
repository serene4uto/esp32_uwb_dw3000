/**
 * @file    tag.c
 * @brief    Tag Application Layer
 *             TWR functions collection
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#include "esp_log.h"

#include "tag.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "deca_vals.h"
#include "common_n.h"
#include "util.h"


// #define TAG_GIVING_TURN_ACK_DELAY_UUS     (1500)           /* delay before sending ACK after receiving Giving Turn message, us */
// #define TAG_TX_POLL_DELAY_US            (9000)                /* delay before sending POLL after receiving Giving Turn message, us */

#define TAG_DEFAULT_RESP_DELAY_TIME_US  (10000)                                         /* default response msg delay time, us */
#define TAG_DEFAULT_RESP_TIMEOUT_US     (TAG_DEFAULT_RESP_DELAY_TIME_US)         /* default response msg timeout, us */

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

    if(pTagInfo->lastTxMsg == MSG_POLL_BROADCAST)
    {
        dwt_readtxtimestamp(pTagInfo->lastPollBrdcastTxTs);

        pTagInfo->lastTxMsg = MSG_NONE;
        pTagInfo->expectedRxMsg = MSG_RESP;

        if(pTagInfo->mode != tag_info_s::RANGING_MODE) {
            pTagInfo->mode = tag_info_s::RANGING_MODE;
        }

        // ESP_LOGI(TAG_LOG_TAG, "Poll Broadcast sent");
    }

    if(pTagInfo->lastTxMsg == MSG_END_TURN)
    {
        // ESP_LOGI(TAG_LOG_TAG, "End Turn sent");
        tag_restart_for_new_turn(pTagInfo);
    }
}

static void tag_rx_cb(const dwt_cb_data_t *rxd)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    tag_info_t *pTagInfo = getTagInfoPtr();
    tag_rx_pckt_t rxPckt;

    if (pTagInfo == NULL) {
        return;
    }

    // Read the receive timestamp and data from the device
    dwt_readrxtimestamp(rxPckt.timeStamp);
    rxPckt.status = rxd->status;
    rxPckt.rxDataLen = rxd->datalength;

    // Read the received data into the packet buffer
    dwt_readrxdata(rxPckt.msg.raw, rxPckt.rxDataLen, 0);

    // Send the received packet to the queue from ISR
    if (xQueueSendFromISR(pTagInfo->rxPktQueue, &rxPckt, &xHigherPriorityTaskWoken) != pdPASS){
        // Handle the error if the queue is full
        // Optional: Log the event or increment a missed packet counter
    }

    // Notify the task that new data is available
    if (app.tagRxTask.Handle != NULL) {
        vTaskNotifyGiveFromISR(app.tagRxTask.Handle, &xHigherPriorityTaskWoken);
    }

    // Request a context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // -->switch to the higher priority task immediately
}

static void tag_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
    tag_info_t *pTagInfo = getTagInfoPtr();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pTagInfo == NULL) {
        return;
    }

    // Use direct comparisons with expected mode and message for faster checks
    if (pTagInfo->mode == tag_info_s::WAIT_FOR_TURN_MODE && 
        pTagInfo->expectedRxMsg == MSG_GIVING_TURN
    ) {
        // Log and restart for new turn if condition met
        // ESP_LOGI(TAG_LOG_TAG, "GIVING TURN RX Timeout");
        tag_restart_for_new_turn(pTagInfo);
    }
    else if (pTagInfo->mode == tag_info_s::RANGING_MODE && 
            pTagInfo->expectedRxMsg == MSG_RESP
    ) {
        // Log and process RX timeout for RANGING_MODE
        // ESP_LOGI(TAG_LOG_TAG, "RESP RX Timeout");

        // Inline the response wait count increment and check
        if (++pTagInfo->curRespWaitCount >= pTagInfo->curAnchorNum) {
            pTagInfo->curRespWaitCount = 0;

            // Directly notify the end turn task from the ISR
            vTaskNotifyGiveFromISR(app.tagEndTurnTask.Handle, &xHigherPriorityTaskWoken);
        }
    }

    // Context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void
tag_rx_error_cb(const dwt_cb_data_t *rxd)
{
    //TODO: implement
    ESP_LOGI(TAG_LOG_TAG, "RX Error");
}


//-----------------------------------------------------------------------------

error_e tag_process_init(void)
{
    esp_log_level_set(TAG_LOG_TAG, TAG_LOG_LEVEL);
    
    tag_info_t *pTagInfo = NULL;

    // Allocate memory for the tag_info_t structure
    psTagInfo = &sTagInfo; //TODO: fix this

    pTagInfo = getTagInfoPtr(); // get pointer to the tag_info_t structure

    if(!pTagInfo) {
        return(_ERR_Cannot_Alloc_Memory);
    }

    // clear the structure
    memset(pTagInfo, 0 , sizeof(tag_info_t));

    if(pTagInfo->rxPktQueue == NULL) {
        pTagInfo->rxPktQueue = xQueueCreate(TAG_EVENT_BUF_SIZE, sizeof(tag_rx_pckt_t));
    }

    /* Hard code the known anchors for the demo */
    pTagInfo->anchorList[0].shortAddr.eui16 = (uint16_t)TWR_ANCHOR_MASTER_EUI16;
    // pTagInfo->anchorList[1].shortAddr.eui16 = (uint16_t)TWR_ANCHOR_DEV1_EUI16;
    // pTagInfo->anchorList[2].shortAddr.eui16 = (uint16_t)TWR_ANCHOR_DEV2_EUI16;
    // pTagInfo->anchorList[3].shortAddr.eui16 = (uint16_t)TWR_ANCHOR_DEV3_EUI16;
    pTagInfo->curAnchorNum = 1;
    pTagInfo->anchorMasterIdx = 0; // index of the anchor master

    // fixed respDelay for each anchor
    for(uint8_t i=0; i<pTagInfo->curAnchorNum; i++)
    {
        pTagInfo->anchorList[i].respDelay = (2 * i + 1)* TAG_DEFAULT_RESP_DELAY_TIME_US; //DEFAULT_REPLY_DELAY_TIME_US
        // pTagInfo->anchorList[i].respDelay = (i + 1)* TAG_DEFAULT_RESP_DELAY_TIME_US; //DEFAULT_REPLY_DELAY_TIME_US // TODO: estimate the delay time
    }

    pTagInfo->panID = TAG_DEFAULT_PANID;
    
    /* set the short address */
    pTagInfo->shortAddress.eui16 = (uint16_t)TAG_ADDR_EUI16;

    /* dwt_xx calls in app level Must be in protected mode (DW3000 IRQ disabled) */
    disable_dw3000_irq();

    TAG_ENTER_CRITICAL();

    if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID) != DWT_SUCCESS) /**< set callbacks to NULL inside dwt_initialise*/
    {
        TAG_EXIT_CRITICAL();
        ESP_LOGE(TAG_LOG_TAG, "Init failed");
        return (_ERR_INIT);   // device initialise has failed
    }

    set_dw_spi_fast_rate();

    uint32_t dev_id = dwt_readdevid();

    if (dev_id != DWT_C0_DEV_ID) {
        TAG_EXIT_CRITICAL();
        ESP_LOGE(TAG_LOG_TAG, "Device ID check failed");
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

    pTagInfo->curRespWaitCount = 0;

    TAG_EXIT_CRITICAL();

    ESP_LOGI(TAG_LOG_TAG, "Tag init done");

    // print tag address 
    ESP_LOGI(TAG_LOG_TAG, "Current Tag: %02X:%02X", 
                pTagInfo->shortAddress.bytes[0], 
                pTagInfo->shortAddress.bytes[1]);

    // print anchor master address
    ESP_LOGI(TAG_LOG_TAG, "Anchor Master: %02X:%02X", 
                pTagInfo->anchorList[pTagInfo->anchorMasterIdx].shortAddr.bytes[0], 
                pTagInfo->anchorList[pTagInfo->anchorMasterIdx].shortAddr.bytes[1]);
    
    // print other anchors address except the master anchor
    for(uint8_t i=1; i<pTagInfo->curAnchorNum; i++)
    {
        ESP_LOGI(TAG_LOG_TAG, "Anchor %d: %02X:%02X", 
                    i, 
                    pTagInfo->anchorList[i].shortAddr.bytes[0], 
                    pTagInfo->anchorList[i].shortAddr.bytes[1]);
    }
    
    return (_NO_ERR);
}



void tag_process_start(void) {

    tag_info_t *pTagInfo = getTagInfoPtr();
    if(!pTagInfo) {
        return;
    }

    enable_dw3000_irq();

    tag_restart_for_new_turn(pTagInfo);
}


void tag_process_terminate(void) {
    //TODO: implement
}



//-----------------------------------------------------------------------------

error_e tag_send_blink(tag_info_t *p)
{
    error_e       ret;

    return ret;
}

error_e tag_send_poll_broadcast(tag_info_t *pTagInfo, tag_rx_pckt_t *pRxPckt) {
    
    error_e ret = _NO_ERR;
    tx_pckt_t txPckt;
    uint64_t rxTs;

    memset(&txPckt, 0, sizeof(txPckt));

    txPckt.psduLen = sizeof(poll_broadcast_msg_t);
    txPckt.msg.poll_broadcast_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.poll_broadcast_msg.mac.frameCtrl[1] = FC_2_SHORT;

    txPckt.msg.poll_broadcast_msg.mac.seqNum = pTagInfo->seqNum++;
    txPckt.msg.poll_broadcast_msg.mac.panID[0] = (uint8_t)(pTagInfo->panID & 0xff);
    txPckt.msg.poll_broadcast_msg.mac.panID[1] = (uint8_t)(pTagInfo->panID >> 8);

    // Set the destination address as broadcast 0xFFFF
    txPckt.msg.poll_broadcast_msg.mac.destAddr[0] = 0xFF;
    txPckt.msg.poll_broadcast_msg.mac.destAddr[1] = 0xFF;

    // Set the source address
    txPckt.msg.poll_broadcast_msg.mac.sourceAddr[0] = pTagInfo->shortAddress.bytes[0];
    txPckt.msg.poll_broadcast_msg.mac.sourceAddr[1] = pTagInfo->shortAddress.bytes[1];

    // Set the message type
    txPckt.msg.poll_broadcast_msg.msgType = MSG_POLL_BROADCAST;

    // Set scheduled anchors
    txPckt.msg.poll_broadcast_msg.numAnchors = pTagInfo->curAnchorNum;
    for (int i = 0; i < pTagInfo->curAnchorNum; i++) {
        txPckt.msg.poll_broadcast_msg.anchorSchedule[i].shortAddr[0] = pTagInfo->anchorList[i].shortAddr.bytes[0];
        txPckt.msg.poll_broadcast_msg.anchorSchedule[i].shortAddr[1] = pTagInfo->anchorList[i].shortAddr.bytes[1];

        txPckt.msg.poll_broadcast_msg.anchorSchedule[i].respTime[0] = (uint8_t)(pTagInfo->anchorList[i].respDelay & 0xFF);
        txPckt.msg.poll_broadcast_msg.anchorSchedule[i].respTime[1] = (uint8_t)(pTagInfo->anchorList[i].respDelay >> 8);
    }

    // Set transmission parameters
    // txPckt.txFlag              = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;

    txPckt.txFlag               = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED;
    txPckt.delayedTxTimeH_dt    = 0;
    txPckt.delayedRxTime_sy     = 0;
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(TAG_DEFAULT_RESP_TIMEOUT_US);
    
    // Calculate the delayed time to respond
    // TS2U64_MEMCPY(u64RxTs, prxPckt->timeStamp);
    // rxTs =  ((uint64_t)pRxPckt->timeStamp[4] << 32) |
    //         ((uint64_t)pRxPckt->timeStamp[3] << 24) |
    //         ((uint64_t)pRxPckt->timeStamp[2] << 16) |
    //         ((uint64_t)pRxPckt->timeStamp[1] << 8)  |
    //         pRxPckt->timeStamp[0];

    // Add delay
    // txPckt.delayedTxTimeH_dt = (rxTs + util_us_to_dev_time(TAG_TX_POLL_DELAY_US)) >> 8;

    pTagInfo->lastTxMsg = MSG_POLL_BROADCAST;

    TAG_ENTER_CRITICAL();
    ret = tx_start(&txPckt);
    TAG_EXIT_CRITICAL();

    if (ret != _NO_ERR) {
        ESP_LOGE(TAG_LOG_TAG, "Poll Broadcast TX failed");
        tag_restart_for_new_turn(pTagInfo);
    }
    
    return ret;
}


error_e tag_send_end_turn(tag_info_t *pTagInfo) {

    error_e ret = _NO_ERR;
    tx_pckt_t txPckt;

    memset(&txPckt, 0, sizeof(txPckt));

    txPckt.psduLen = sizeof(end_turn_msg_t);
    txPckt.msg.end_turn_msg.mac.frameCtrl[0] = FC_1;
    txPckt.msg.end_turn_msg.mac.frameCtrl[1] = FC_2_SHORT;
    txPckt.msg.end_turn_msg.mac.seqNum = pTagInfo->seqNum;

    txPckt.msg.end_turn_msg.mac.panID[0] = (uint8_t)(pTagInfo->panID & 0xff);
    txPckt.msg.end_turn_msg.mac.panID[1] = (uint8_t)(pTagInfo->panID >> 8);

    memcpy(txPckt.msg.end_turn_msg.mac.destAddr, 
        pTagInfo->anchorList[pTagInfo->anchorMasterIdx].shortAddr.bytes, 
        sizeof(pTagInfo->anchorList[pTagInfo->anchorMasterIdx].shortAddr.bytes)
    );

    memcpy(txPckt.msg.end_turn_msg.mac.sourceAddr, 
        pTagInfo->shortAddress.bytes, 
        sizeof(pTagInfo->shortAddress.bytes)
    );

    txPckt.msg.end_turn_msg.msgType = MSG_END_TURN;

    // add report
    txPckt.msg.end_turn_msg.numAnchors = pTagInfo->curAnchorNum;
    for (int i = 0; i < pTagInfo->curAnchorNum; i++) {
        memcpy(txPckt.msg.end_turn_msg.anchorReport[i].shortAddr, 
            pTagInfo->anchorList[i].shortAddr.bytes, 
            sizeof(pTagInfo->anchorList[i].shortAddr.bytes)
        );

        // add distance
        uint16_t dist = (uint16_t)(pTagInfo->anchorList[i].lastDist * 1000); // convert to mm
        txPckt.msg.end_turn_msg.anchorReport[i].dist[0] = (uint8_t)(dist & 0xff);
        txPckt.msg.end_turn_msg.anchorReport[i].dist[1] = (uint8_t)(dist >> 8);
    }

    // set transmission parameters
    txPckt.txFlag = (DWT_START_TX_IMMEDIATE);
    txPckt.delayedRxTime_sy = util_us_to_sy(0);
    txPckt.delayedRxTimeout_sy = util_us_to_sy(0); // no timeout
    txPckt.delayedTxTimeH_dt = util_us_to_dev_time(0);

    pTagInfo->seqNum++;
    pTagInfo->lastTxMsg = MSG_END_TURN;

    TAG_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    TAG_EXIT_CRITICAL();

    if (ret != _NO_ERR)
    {
        // less chance to fail here because send immediately, but still need to handle error for sure
        ESP_LOGE(TAG_LOG_TAG, "END TURN msg TX failed");
        // reset cycle, wait for the next turn
        tag_restart_for_new_turn(pTagInfo);
    }

    return ret;
}

void tag_restart_for_new_turn(tag_info_t *pTagInfo) {
    pTagInfo->mode = tag_info_s::WAIT_FOR_TURN_MODE;
    pTagInfo->expectedRxMsg = MSG_GIVING_TURN;
    pTagInfo->lastTxMsg = MSG_NONE;
    pTagInfo->curRespWaitCount = 0;

    dwt_writefastCMD(CMD_TXRXOFF);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
                    sizeof(pTagInfo->shortAddress.bytes)) != 0) 
        ) {

            //TODO: error handling
            ESP_LOGE(TAG_LOG_TAG, "GIVING TURN msg RX is not correct");
            dwt_writefastCMD(CMD_TXRXOFF);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            dwt_setrxtimeout(0);
            return _ERR;
        }

        ESP_LOGI(TAG_LOG_TAG, "GIVING TURN msg RX succeeded");

        // send broadcast poll message
        if (tag_send_poll_broadcast(pTagInfo, pRxPckt) != _NO_ERR) {
            // no need to handle error here, it will be handled in the function
            return _ERR;
        }

        return _NO_ERR;
    }

    if ((pTagInfo->mode == tag_info_t::RANGING_MODE) && 
        (pTagInfo->expectedRxMsg == MSG_RESP)
    ) {

        // TODO: checking if source anchor is in the list of anchors
        if ((pRxPckt->msg.resp_msg.msgType != MSG_RESP) ||
            (memcmp(pRxPckt->msg.resp_msg.mac.destAddr, 
                    pTagInfo->shortAddress.bytes, 
                    sizeof(pTagInfo->shortAddress.bytes)) != 0) 
        ) {
            //TODO: error handling
            ESP_LOGE(TAG_LOG_TAG, "RESP message RX failed");

            if(++pTagInfo->curRespWaitCount == pTagInfo->curAnchorNum) {
                pTagInfo->curRespWaitCount = 0;
                // send end turn message
                xTaskNotifyGive(app.tagEndTurnTask.Handle);
            }

            return _ERR;
        }

        ESP_LOGI(TAG_LOG_TAG, "RESP message RX succeeded");

        //TODO: calculate the distance --> check this again
        uint64_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;
        uint8_t curAnchorIdx;

        // get offset ratio
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

        // get timestamps
        TS2U64_MEMCPY(poll_tx_ts, pTagInfo->lastPollBrdcastTxTs);
        TS2U64_MEMCPY(resp_rx_ts, pRxPckt->timeStamp);
        TS2U64_MEMCPY(poll_rx_ts, pRxPckt->msg.resp_msg.pollRxTs); // get anchor poll rx ts
        TS2U64_MEMCPY(resp_tx_ts, pRxPckt->msg.resp_msg.respTxTs); // get anchor resp tx ts;

        // compute time of flight and distance
        rtd_init = (uint32_t)resp_rx_ts - (uint32_t)poll_tx_ts;
        rtd_resp = (uint32_t)resp_tx_ts - (uint32_t)poll_rx_ts;

        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        ESP_LOGI(TAG_LOG_TAG, "Distance: %f", distance);

        // check current anchor index
        for (int i = 0; i < pTagInfo->curAnchorNum; i++) {
            if (memcmp(pRxPckt->msg.resp_msg.mac.sourceAddr, 
                       pTagInfo->anchorList[i].shortAddr.bytes, 
                       sizeof(pTagInfo->anchorList[i].shortAddr.bytes)) == 0) {
                curAnchorIdx = i;
                break;
            }
        }

        // save the last distance 
        pTagInfo->anchorList[curAnchorIdx].lastDist = distance;

        // increment the response wait count when receiving a response message or timeout or error
        // TODO: error rx case ???
        if(++pTagInfo->curRespWaitCount == pTagInfo->curAnchorNum) {
            //send end turn message
            pTagInfo->curRespWaitCount = 0;

            // send end turn message
            xTaskNotifyGive(app.tagEndTurnTask.Handle);
        }

        return _NO_ERR;
    }


    return _ERR; // because this rx message is not recognized ??? maybe this is not necessary
}


//-----------------------------------------------------------------------------
static void IRAM_ATTR tag_hw_timer_cb() {
}