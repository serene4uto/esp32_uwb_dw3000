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


#define DW_TAG_NOT_SLEEPING        (0)

#define BLINK_PERIOD_MS            (500)    /* range init phase - Blink send period, ms */

#define DWT_DIAGNOSTIC_LOG_REV_5       (5)


/* Rx Events circular buffer.
 * 0x02, 0x04, 0x08, 0x10, etc.
 * The size of the buffer at any given time should be < 2 */
#define EVENT_BUF_TAG_SIZE        (0x02)

#define POLL_ENTER_CRITICAL()     vPortEnterCritical()
#define POLL_EXIT_CRITICAL()      vPortExitCritical()

//-----------------------------------------------------------------------------
// Struct & Typedefs




/* RxPckt */
struct rx_pckt_t_s
{
    int16_t        rxDataLen;

    union {
      std_msg_t           stdMsg;
      std_msg_ss_t        ssMsg;
      std_msg_ls_t        lsMsg;
      twr_msg_t           twrMsg;
      blink_msg_t         blinkMsg;
      rng_cfg_msg_t       rngCfgMsg;
      rng_cfg_upd_msg_t   rngCfgUpdMsg;
      resp_pdoa_msg_t      respExtMsg;
    } msg;

    uint8_t     timeStamp[TS_40B_SIZE];   /* Full TimeStamp */
    uint32_t    rtcTimeStamp;             /* MCU RTC timestamp */
    uint16_t    firstPath;                /* First path (raw 10.6) */
    int16_t     clock_offset;

};

typedef struct rx_pckt_t_s rx_pckt_t_t;

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

    /* circular Buffer of received Rx packets :
     * uses in transferring of the data from ISR to APP level.
     * */
    struct {
        rx_pckt_t_t   buf[EVENT_BUF_TAG_SIZE];
        uint16_t    head;
        uint16_t    tail;
    } rxPcktBuf;


    /* Environment - configured from Range init structure.
     * slotCorr_us is used to adjust slot every reception as part of Response
     */
    struct    env
    {

    } env;


    enum
    {
        BLINKING_MODE,
        RANGING_MODE
    } mode;


};


error_e tag_process_init(void);
// void    tag_process_start(void);
// void    tag_process_terminate(void);





#ifdef __cplusplus
}
#endif

#endif /* __TAG__H__ */
