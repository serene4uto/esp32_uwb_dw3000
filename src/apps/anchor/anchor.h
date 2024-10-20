/**
 * @brief Anchor Master bare implementation
 * 
 * @attention //TODO: Add attention here
 * 
 * @author Nguyen Ha Trung
 */



#ifndef __ANCHOR_M__H__
#define __ANCHOR_M__H__ 1

#include "error_types.h"
#include "app.h"
#include "port.h"
#include "default_config.h"
#include "uwb_mac_frames.h"

#define ANCHOR_DEFAULT_PANID 0xDECA
#define ANCHOR_ADDR_EUI16 TWR_ANCHOR_MASTER_EUI16

#define ANCHOR_EVENT_BUF_SIZE        (0x02)

#define ANCHOR_MASTER_MAX_TAGS       (4)

/* RxPckt is the structure is for the current reception */
struct anchor_rx_pckt_s
{
    uint16_t        rxDataLen;

    union {
      uint8_t             raw[STANDARD_FRAME_SIZE];   /**< Raw message buffer. */
      giving_turn_msg_t   giving_turn_msg;            /**< Giving turn message. */
      ack_msg_t           ack_msg;                    /**< Ack message. */
    } msg;  /**< Union of possible message types to be received. */

    uint8_t     timeStamp[TS_40B_SIZE]; /* Timestamp of the received frame */

    dev_eui16_t *tag;                /* the tag, to which the current range exchange is performing */


    uint32_t     status;              /* status of the received frame */
};

typedef struct anchor_rx_pckt_s anchor_rx_pckt_t;

struct anchor_info_s
{
    /* Unique long Address, used at the discovery phase before Range Init reception */
    union {
        uint8_t  euiLong[8];
        uint64_t eui64;
    };

    /* Unique short Address, uses at the ranging phase
     * valid for low-endian compiler.
     * */
    union    {
       uint8_t     euiShort[2];
       uint16_t    eui16;
    };

    uint16_t    panID;          // PAN ID

    QueueHandle_t rxPcktQueue = NULL;  // circular Buffer of received Rx packets

    uint8_t seqNum; // sequence number, increases on every tx_start
    uwb_msg_e_t lastTxMsg; // last transmitted message type
    uwb_msg_e_t expectedRxMsg; // expected next rx message type
    
    bool   isMaster = false;  // true if this is the Master Anchor

    enum {
        IDLE_MODE = 0x00,
        GIVING_TURN_MODE,
        RANGING_MODE,
    } mode;

    // master info
    uint8_t curTagNum; // number of tags
    dev_eui16_t tagList[ANCHOR_MASTER_MAX_TAGS]; // list of tags
    uint8_t curTagIdx; // current tag index
};

typedef struct anchor_info_s anchor_info_t; 



anchor_info_t * getAnchorInfoPtr(void);

error_e anchor_process_init(void);
void    anchor_process_start(void);
void    anchor_process_terminate(void);


error_e anchor_master_give_turn(anchor_info_t *pAnchorInfo);

error_e anchor_process_rx_pckt(anchor_info_t *pAnchorInfo, anchor_rx_pckt_t *pRxPckt);

#endif /* __ANCHOR_M__H__ */