/**
 * @file    common_n.h
 *
 * @brief   common TWR defines, types and fn()
 *
 * @attention
 *
 * Copyright 2016-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 */

#ifndef CORE_SRC_SRV_COMMON_N_H_
#define CORE_SRC_SRV_COMMON_N_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "deca_device_api.h"
#include "deca_interface.h"
#include "uwb_mac_frames.h"




#ifndef M_PI
#define M_PI    (3.141592654f)
#endif

#ifndef M_PI_2
#define M_PI_2  (1.570796327f)
#endif

#ifndef TWO_PI
#define TWO_PI  (2*M_PI)
#endif

// ----------------------------------------------------------------------------

/* TxPckt */
struct tx_pckt_s
{
    int16_t        psduLen;

    union {
        std_msg_t           stdMsg;
        twr_msg_t           twrMsg;

        blink_msg_t         blinkMsg;
    } msg;

    uint8_t        txFlag;              // Holds Tx sending parameters: extended set for DW3000

    uint32_t    delayedTxTimeH_sy;      // Delayed transmit time (in 4ns)
    uint32_t    delayedRxTime_sy;       // Delay after Tx when to switch on receiver (in SY=1.0256us)
    uint16_t    delayedRxTimeout_sy;    // How long the receiver will be switched on after Tx (in SY=1.0256us)

};

typedef struct tx_pckt_s tx_pckt_t;

// ----------------------------------------------------------------------------
//
void tn_app_config(rxtx_configure_t *p);
void rxtx_configure(rxtx_configure_t *p);
error_e tx_start(tx_pckt_t * pTxPckt);

#ifdef __cplusplus
}
#endif

#endif /* CORE_SRC_SRV_COMMON_N_H_ */
