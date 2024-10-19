/**
 * @file      uwb_frames.h
 *
 * @brief     UWB message frames definitions and typedefs
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef UWB_FRAMES_H_
#define UWB_FRAMES_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define TS_40B_SIZE                 (5)
#define TS_UWB_SIZE                 (5)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC                   2
#define FRAME_SOURCE_ADDRESS_S      (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L      (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP                 (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)         /* 5 */
#define FRAME_CTRL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)     /* 21 bytes for 64-bit addresses) */
#define FRAME_CTRL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     /* 9 bytes for 16-bit addresses) */
#define FRAME_CTRL_AND_ADDRESS_LS   (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     /* 15 bytes for 1 16-bit address and 1 64-bit address) */
#define MAX_USER_PAYLOAD_STRING_LL  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_L-FRAME_CRC)          /* 127 - 21 - 2 = 104 */
#define MAX_USER_PAYLOAD_STRING_SS  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_S-FRAME_CRC)          /* 127 - 9 - 2 = 116 */
#define MAX_USER_PAYLOAD_STRING_LS  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_LS-FRAME_CRC)         /* 127 - 15 - 2 = 110 */

#define FRAME_DEST_ADDRESS_S_IDX    (FRAME_CTRLP)
#define FRAME_SRC_ADDRESS_S_IDX     (FRAME_CTRLP + ADDR_BYTE_SIZE_S)
//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING     MAX_USER_PAYLOAD_STRING_LL

#define RC_VERSION_PDOA            (3)
#define RC_VERSION_DR              (4)


enum {
    Head_Msg_BLINK      =   0xC5,
    Head_Msg_STD        =   (0x40 | 0x01),
    Head_Msg_STD_AR     =   (0x40 | 0x20 | 0x01),
    Frame_Ctrl_SS       =   (0x80 | 0x08),    //Message addressing: destination short (16-bit), source short (16-bit)
    Frame_Ctrl_LS       =   (0x80 | 0x0C),    //Message addressing: destination long (64-bit), source short (16-bit)
    Frame_Ctrl_MASK     =   0xCC
};


/* enumeration of function codes used in TWR protocol */
typedef enum {
    Twr_Fcode_Not_Defined       = 0xFF, // Special : nothing
    Twr_Fcode_Blink             = 0xEE, // Special : Blink
    Twr_Fcode_Rng_Config        = 0x20, // Responder (Node) Ranging Config message          : reply to blink
    Twr_Fcode_Tag_Poll          = 0x84, // Initiator (Tag)  Poll message                    : twr start message
    Twr_Fcode_Tag_Final         = 0x88, // Initiator (Tag)  Final message back to Responder : reply to Response
}fcode_e;


//TDOA TWR
enum {
    RTLS_TWR_MSG_RNG_INIT       =   Twr_Fcode_Rng_Config,   // Ranging initiation message
    RTLS_TWR_MSG_TAG_POLL       =   0x81,   // Initiator (Tag) poll message
    RTLS_TWR_MSG_ANCH_RESP      =   0x70,   // Responder (Anchor) response to poll
    RTLS_TWR_MSG_TAG_FINAL      =   0x82,   // Initiator (Tag) final message back to Responder
    RTLS_TWR_MSG_ANCH_TOFR      =   0x71,   // Responder (Anchor) TOF Report message to Initiator

    RTLS_MSG_ANCH_CLK_SYNC      =   0x2C,   // Anchor CLK SYNC message (broadcast)

    HEAD_MSG_BLINK              =   Head_Msg_BLINK,       // Tag standard Blink message
    HEAD_MSG_STD                =   Head_Msg_STD,         // Std message header

    APP_MSG_DATA                =   0x7A,   // Data Transfer Message (this is Communications Test function code)

    APP_MSG_UWB_BH_DWNSTREAM    =   0x7B,   // UWB Command from CLE via Master to Slave
    APP_MSG_UWB_BH_UPSTREAM     =   0x7C,   // UWB Data Backhaul back from Slave to CLE via Master
};


/* UWB packet types : MAC headers */
typedef struct
{
        uint8_t frameCtrl[2];                   //  frame control bytes 00-01
        uint8_t seqNum;                         //  sequence_number 02
        uint8_t panID[2];                       //  PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_S];     //  05-06
        uint8_t sourceAddr[ADDR_BYTE_SIZE_S];   //  07-08
}__attribute__((packed))
mac_header_ss_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_L];     // 05-12 or using 64 bit addresses (05-12)
        uint8_t sourceAddr[ADDR_BYTE_SIZE_L];   // 13-20 or using 64 bit addresses (13-20)
}__attribute__((packed))
mac_header_ll_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_L];     // 05-12 using 64 bit addresses
        uint8_t sourceAddr[ADDR_BYTE_SIZE_S];   // 13-14
}__attribute__((packed))
mac_header_ls_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_S];     // 05-06
        uint8_t sourceAddr[ADDR_BYTE_SIZE_L];   // 7-14 using 64 bit addresses
}__attribute__((packed))
mac_header_sl_t;

/* General UWB packet types : Messages */
typedef struct
{
    mac_header_ll_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_LL];// 21-124 (application data and any user payload)
    uint8_t         fcs[2]; // 125-126  we allow space for the CRC as it is logically part of the message.
                            // DW1000 calculates and adds these bytes.
}__attribute__((packed))
std_msg_ll_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_SS];// 09-124 (application data and any user payload)
    uint8_t         fcs[2];
}__attribute__((packed))
std_msg_ss_t;

typedef struct
{
    mac_header_ls_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_LS];// 15-124 (application data and any user payload)
    uint8_t         fcs[2];
}__attribute__((packed))
std_msg_ls_t;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8_t         frameCtrl[1];           //  frame control bytes 00
    uint8_t         seqNum;                 //  sequence_number 01
    uint8_t         tagID[ADDR_BYTE_SIZE_L];//  02-09 64 bit addresses
    uint8_t         fcs[2];
}__attribute__((packed))
blink_msg_t;

typedef std_msg_ll_t std_msg_t;
typedef std_msg_ss_t twr_msg_t;

#ifdef __cplusplus
}
#endif

#endif /* UWB_FRAMES_H_ */
