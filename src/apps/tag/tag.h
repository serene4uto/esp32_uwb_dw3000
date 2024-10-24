/**
 * @file    tag.h
 *
 * @brief   tag bare implementation
 *
 * @attention //TODO: Add attention here
 *
 * @author
 */


#ifndef __TAG__H__
#define __TAG__H__ 1

#include "uwb_mac_frames.h"
#include "app.h"
#include "port.h"


#ifdef __cplusplus
 extern "C" {
#endif

#define TAG_DEFAULT_PANID               (0xDECA) /* default PAN ID */

#define TAG_BLINK_PERIOD_MS            (500)    /* range init phase - Blink send period, ms */
#define TAG_MAX_ANCHORS                (8)      /* Maximum number of anchors */

/* Rx Events circular buffer.
 * 0x02, 0x04, 0x08, 0x10, etc.
 * The size of the buffer at any given time should be < 2 */
#define TAG_EVENT_BUF_SIZE        (0x02)

//-----------------------------------------------------------------------------
// Struct & Typedefs

/* rx packet structure of tag */
struct tag_rx_pckt_s
{
  uint16_t rxDataLen;

    union {
      uint8_t               raw[STANDARD_FRAME_SIZE];   /**< Raw message buffer. */
      giving_turn_msg_t     giving_turn_msg;            /**< Giving turn message. */
      ack_msg_t             ack_msg;                    /**< Ack message. */
      poll_msg_t            poll_msg;                   /**< Poll message. */
      poll_broadcast_msg_t  poll_broadcast_msg;         /**< Poll broadcast message. */
    } msg;  /**< Union of possible message types to be received. */

  uint8_t timeStamp[TS_40B_SIZE];   /* Full TimeStamp */

  uint32_t status;                  /* status of the received frame */

};

typedef struct tag_rx_pckt_s tag_rx_pckt_t;

/* This structure holds application parameters:
 * eui64
 * txAntennaDelay
 * rxAntennaDelay
 * timestamps for every phase's IRQ:
 *             initiator: blinkTx_ts, pollTx_ts, respRX_ts, finalTx_ts, (reportRx_ts)
 *             responder: blinkRx_ts, pollRx_ts, respTx_ts, finalRx_ts, (reportTx_ts)
 *
 * */
struct tag_info_s {

  /* Unique long Address, used at the discovery phase before Range Init reception */
  union {
    uint8_t  bytes[8];
    uint64_t eui64;
  } longAddress;

  /* Unique short Address, uses at the ranging phase
   * valid for low-endian compiler.
   * */
  union    {
    uint8_t     bytes[2];
    uint16_t    eui16;
  } shortAddress;


  uint16_t panID; // PAN ID



  QueueHandle_t rxPktQueue = NULL; // queue of received packets

  /* ranging variables */
  struct {
      /* MAC sequence number, increases on every tx_start */
      uint8_t seqNum;

      /* Application DW_TX_IRQ source indicator */
      uwb_msg_e_t lastTxMsg;
  };


  /* Tag's crystal clock offset trimming */
  int16_t     clkOffset_pphm;     // ???
  uint8_t     xtaltrim;           // Tag crystal trim value

  uint16_t    lateTX;             //used for Debug to count any lateTX

  enum
  {
    IDLE_MODE = 0x00,
    BLINKING_MODE,
    WAIT_FOR_TURN_MODE,
    RANGING_MODE
  } mode;

  uwb_msg_e_t expectedRxMsg; // expected message type

  // Anchor management
  dev_eui16_t anchorList[TAG_MAX_ANCHORS]; // list of anchors
  uint8_t     curAnchorNum;  // number of anchors
  uint8_t     curAnchorIdx;  // current anchor index to range with


};

typedef struct tag_info_s tag_info_t;


//-----------------------------------------------------------------------------
// exported functions prototypes

/* initiator (tag) */
tag_info_t * getTagInfoPtr(void);

error_e tag_process_init(void);
void    tag_process_start(void);
void    tag_process_terminate(void);

error_e tag_send_blink(tag_info_t *p);

error_e tag_process_rx_pkt(tag_info_t *pTagInfo, tag_rx_pckt_t *pRxPckt);
error_e tag_respond_ack(tag_info_t *pTagInfo, tag_rx_pckt_t *prxPckt);
error_e tag_send_poll(tag_info_t *pTagInfo);

#ifdef __cplusplus
}
#endif

#endif /* __TAG__H__ */
