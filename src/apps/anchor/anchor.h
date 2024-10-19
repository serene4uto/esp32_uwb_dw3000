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

#define ANCHOR_ADDR_EUI16 TWR_ANCHOR_MASTER_EUI16

#define EVENT_BUF_ANCHOR_SIZE        (0x02)

/* RxPckt is the structure is for the current reception */
struct anchor_rx_pckt_s
{
    int16_t        rxDataLen;

    union {
        std_msg_t           stdMsg;
        std_msg_ss_t        ssMsg;
        std_msg_ls_t        lsMsg;
        twr_msg_t           twrMsg;
        blink_msg_t         blinkMsg;
    } msg;

    dev_eui16_t *tag;                /* the tag, to which the current range exchange is performing */
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


    QueueHandle_t rxPcktQueue = NULL;  // circular Buffer of received Rx packets


    bool   isMaster = false;  // true if this is the Master Anchor

};

typedef struct anchor_info_s anchor_info_t; 



anchor_info_t * getAnchorInfoPtr(void);

error_e anchor_process_init(void);
void    anchor_process_start(void);
void    anchor_process_terminate(void);

#endif /* __ANCHOR_M__H__ */