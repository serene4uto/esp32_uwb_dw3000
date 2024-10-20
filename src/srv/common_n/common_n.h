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
typedef struct tx_pckt_s
{
    uint16_t psduLen;  /**< Length of the packet payload in bytes. */

    union {
        uint8_t             raw[STANDARD_FRAME_SIZE];  /**< Raw message buffer. */
        giving_turn_msg_t   giving_turn_msg;  /**< Giving turn message. */
        ack_msg_t           ack_msg;          /**< Ack message. */
    } msg;  /**< Union of possible message types to be transmitted. */

    uint8_t txFlag;  /**< Transmission flags indicating sending parameters (extended set for DW3000). */

    uint32_t delayedTxTimeH_sy;   /**< Delayed transmit time (units in 4ns). */
    uint32_t delayedRxTime_sy;    /**< Delay after transmission before switching on receiver (units in symbols, 1 symbol ≈ 1.0256µs). */
    uint16_t delayedRxTimeout_sy; /**< Duration the receiver remains on after transmission (units in symbols, 1 symbol ≈ 1.0256µs). */

} tx_pckt_t;

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
