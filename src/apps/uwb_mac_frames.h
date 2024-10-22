/**
 * @file      uwb_frames.h
 *
 * @brief     UWB message frames definitions and typedefs
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: Add attention here
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


#define POLL_BROADCAST_MAX_ANCHORS  (8)

typedef enum {
    MSG_NONE = 0x00,
    MSG_GIVING_TURN = 0x01,
    MSG_ACK = 0x02,
    MSG_POLL = 0x03,
    MSG_POLL_BROADCAST = 0x04,
    MSG_RESP = 0x05,
    MSG_FINAL = 0x06,
    MSG_REPORT = 0x07,
    MSG_END_TURN = 0x08,
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
    uint8_t message_type;
    uint8_t fcs[2];
}__attribute__((packed))
giving_turn_msg_t;


typedef struct
{
    mac_header_ss_t mac;
    uint8_t message_type;
    uint8_t fcs[2];  
}__attribute__((packed))
ack_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t msgType;
    uint8_t respTime[2];
    uint8_t fcs[2];
}__attribute__((packed))
poll_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t msgType;
    uint8_t numAnchors;
    struct {
        uint8_t shortAddr[ADDR_BYTE_SIZE_S];
        uint8_t respTime[2];
    } anchorSchedule[POLL_BROADCAST_MAX_ANCHORS];
    uint8_t fcs[2];
}__attribute__((packed))
poll_broadcast_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t msgType;
    uint8_t pollRxTs[4];
    uint8_t respTxTs[4];
    uint8_t fcs[2];
}__attribute__((packed))
resp_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t msgType;
    struct {
        uint8_t shortAddr[ADDR_BYTE_SIZE_S];
        uint8_t dist[4];
    } anchorReport[POLL_BROADCAST_MAX_ANCHORS];
    uint8_t fcs[2];
}__attribute__((packed))
end_turn_msg_t;


#ifdef __cplusplus
}
#endif

#endif /* UWB_FRAMES_H_ */
