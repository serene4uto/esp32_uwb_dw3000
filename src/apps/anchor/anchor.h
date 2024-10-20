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
    int16_t        rxDataLen;

    union {
        
        uint8_t     raw[STANDARD_FRAME_SIZE];
    } msg;

    uint8_t     timeStamp[TS_40B_SIZE]; /* Timestamp of the received frame */

    dev_eui16_t *tag;                /* the tag, to which the current range exchange is performing */


    uint32_t     status;              /* status of the received frame */
};

typedef struct anchor_rx_pckt_s anchor_rx_pckt_t;

struct anchor_info_s
{
    /* Unique short Address, uses at the ranging phase
     * valid for low-endian compiler.
     * */
    union    {
       uint8_t     euiShort[2];
       uint16_t    eui16;
    };

    uint16_t    panID;          // PAN ID


    QueueHandle_t rxPcktQueue = NULL;  // circular Buffer of received Rx packets

    /* ranging variables */
    struct {
        /* MAC sequence number, increases on every tx_start */
        uint8_t        seqNum;

        /* Application DW_TX_IRQ source indicator */
        tx_states_e     txState;
    };

    
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

#endif /* __ANCHOR_M__H__ */