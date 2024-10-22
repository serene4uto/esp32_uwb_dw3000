/**
 * @file    common_n.h
 *
 * @brief   common TWR defines, types and fn()
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#ifndef CORE_SRC_SRV_COMMON_N_H_
#define CORE_SRC_SRV_COMMON_N_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "deca_device_api.h"
#include "deca_interface.h"
#include "uwb_mac_frames.h"

// ----------------------------------------------------------------------------

/* TxPckt */
typedef struct tx_pckt_s
{
    uint16_t psduLen;  /**< Length of the packet payload in bytes. */

    union {
      uint8_t               raw[STANDARD_FRAME_SIZE];   /**< Raw message buffer. */
      giving_turn_msg_t     giving_turn_msg;            /**< Giving turn message. */
      ack_msg_t             ack_msg;                    /**< Ack message. */
      poll_msg_t            poll_msg;                   /**< Poll message. */
      poll_broadcast_msg_t  poll_broadcast_msg;         /**< Poll broadcast message. */
    } msg;  /**< Union of possible message types to be received. */

    uint8_t txFlag;  /**< Transmission flags indicating sending parameters (extended set for DW3000). */

    uint32_t delayedTxTimeH_dt;   /**< Delay before transmission in dt, the high 32 bits only. Check DW3000 User Manual, section 3.3. */
    uint32_t delayedRxTime_sy;    /**< Delay after transmission before switching on receiver in symbols. */
    uint32_t delayedRxTimeout_sy; /**< Duration the receiver remains on after transmission in symbols. */

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
