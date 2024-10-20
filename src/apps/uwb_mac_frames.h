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

typedef enum {
    MSG_GIVING_TURN = 0x01,
} uwb_msg_e_t;

typedef enum {
    FC_1        = 0x41,
    FC_1_BLINK  = 0xC5,
    FC_2        = 0x8C,
    FC_2_SHORT  = 0x88,
} uwb_frame_control_e_t;


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
    mac_header_ss_t mac;
    struct
    {
        uint8_t message_type;
    } body;
}__attribute__((packed))
giving_turn_msg_t;


#ifdef __cplusplus
}
#endif

#endif /* UWB_FRAMES_H_ */
