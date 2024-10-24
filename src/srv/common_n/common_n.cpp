/*
 * @file      common_n.c
 * @brief     Decawave Application Layer
 *            Common functions for twr tag and node
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "app.h"
#include "common_n.h"
#include "port.h"

/*
 * @brief   configure tag/node application: frame filtering, PANID, address, antenna delays
 *
 *
 * @note
 * */
void
tn_app_config(rxtx_configure_t *p)
{
    /* set antenna delays */
    dwt_setrxantennadelay(p->rxAntDelay);
    dwt_settxantennadelay(p->txAntDelay);

    dwt_setrxaftertxdelay(0);    /**< no any delays set by default : part of config of receiver on Tx sending */
    dwt_setrxtimeout     (0);    /**< no any delays set by default : part of config of receiver on Tx sending */

    dwt_configureframefilter(p->frameFilter, p->frameFilterMode);

    dwt_setpanid(p->panId);

    dwt_setaddress16(p->shortadd);
}

/*
 * @brief   ISR level (need to be protected if called from APP level)
 *          low-level configuration for DW1000
 *
 *          if called from app, shall be performed with DW IRQ off &
 *          TAG_ENTER_CRITICAL(); / TAG_EXIT_CRITICAL();
 *
 *
 * @note
 * */
void
rxtx_configure(rxtx_configure_t *p)
{
    if(dwt_configure(p->pdwCfg))    /**< Configure the Physical Channel parameters (PLEN, PRF, etc) */
    {
        // error_handler(1, _ERR_INIT );
        //TODO: error handling
    }
    /* configure power */
    dwt_configuretxrf(&app.pConfig->runtime_params.tx_config);

    tn_app_config(p);

}


/**
 * @brief   ISR level (need to be protected if called from APP level)
 *          Transmit packet
 * */
error_e
tx_start(tx_pckt_t * pTxPckt)
{
    error_e ret = _NO_ERR;

    if(pTxPckt->psduLen)
    {
        //print message
        dwt_writetxdata(pTxPckt->psduLen, (uint8_t *) &pTxPckt->msg.raw, 0);
        dwt_writetxfctrl(pTxPckt->psduLen, 0, 1);
    }

    //Setup for delayed Transmit time (units are 4ns)
    if (pTxPckt->txFlag &
        (DWT_START_TX_DELAYED | DWT_START_TX_DLY_REF | DWT_START_TX_DLY_RS | DWT_START_TX_DLY_TS))
    {
        dwt_setdelayedtrxtime(pTxPckt->delayedTxTimeH_dt) ;
    }

    //Setup for delayed Receive after Tx (units are sy = 1.0256 us)
    if(pTxPckt->txFlag & DWT_RESPONSE_EXPECTED)
    {
        dwt_setrxaftertxdelay(pTxPckt->delayedRxTime_sy);
        dwt_setrxtimeout(pTxPckt->delayedRxTimeout_sy);
    }

    // Begin delayed TX of frame
    if(dwt_starttx(pTxPckt->txFlag) != DWT_SUCCESS)
    {
        ret = _ERR_DelayedTX_Late;
    }

    return (ret);
}


