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

#define TWR_TAG_BLINK_PERIOD_MS            (500)    /* range init phase - Blink send period, ms */

/* Rx Events circular buffer.
 * 0x02, 0x04, 0x08, 0x10, etc.
 * The size of the buffer at any given time should be < 2 */
#define EVENT_BUF_TAG_SIZE        (0x02)

#define POLL_ENTER_CRITICAL()     vPortEnterCritical()
#define POLL_EXIT_CRITICAL()      vPortExitCritical()

//-----------------------------------------------------------------------------
// Struct & Typedefs

/* rx packet structure of tag */
struct tag_rx_pckt_s
{
  uint16_t rx_data_len;  

  union {

  } msg;  

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
    uint8_t  euiLong[8];
    uint64_t eui64;
  };

  QueueHandle_t rx_pkt_queue = NULL; // queue of received packets

  /* ranging variables */
  struct {
      /* MAC sequence number, increases on every tx_start */
      uint8_t        seqNum;

      /* Application DW_TX_IRQ source indicator */
      tx_states_e     txState;
  };


  /* Tag's crystal clock offset trimming */
  int16_t     clkOffset_pphm;     //
  uint8_t     xtaltrim;           //Tag crystal trim value

  uint16_t    lateTX;             //used for Debug to count any lateTX

  enum
  {
    IDLE_MODE = 0x00,
    WAIT_FOR_TURN_MODE,
    BLINKING_MODE,
    RANGING_MODE
  } mode;
  
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

#ifdef __cplusplus
}
#endif

#endif /* __TAG__H__ */
