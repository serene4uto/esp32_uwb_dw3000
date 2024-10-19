/**
 * @brief Anchor Master bare implementation
 * 
 * @attention //TODO: Add attention here
 * 
 * @author Nguyen Ha Trung
 */



#include "anchor.h"

static anchor_info_t    sAnchorInfo;
static anchor_info_t   *psAnchorInfo = &sAnchorInfo;


anchor_info_t * getAnchorInfoPtr(void)
{
    return (psAnchorInfo);
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

    /* Hardcoded EUI64s for demo */
    app.pConfig->known_tag_list[0].eui16 = TWR_TAG_DEV1_EUI16;
    app.pConfig->known_tag_list[1].eui16 = TWR_TAG_DEV2_EUI16;
    app.pConfig->known_tag_list_size = 2;

    // Set the EUI16
    pAnchorInfo->eui16 = ANCHOR_ADDR_EUI16;

    if(pAnchorInfo->eui16 == TWR_ANCHOR_MASTER_EUI16)
    {
        pAnchorInfo->isMaster = true;
    }

    if(pAnchorInfo->rxPcktQueue == NULL)
    {
        pAnchorInfo->rxPcktQueue = xQueueCreate(EVENT_BUF_ANCHOR_SIZE, sizeof(anchor_rx_pckt_t));
    }


    


    return _NO_ERR;
}