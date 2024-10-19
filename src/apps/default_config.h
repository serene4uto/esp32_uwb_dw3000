


#ifndef __DEFAULT_CONFIG_H__H__
#define __DEFAULT_CONFIG_H__H__ 1


#ifdef __cplusplus
 extern "C" {
#endif

#include "deca_device_api.h"
#include "tag_list.h"

/* UWB config */
#define DEFAULT_CHANNEL             9
#define DEFAULT_TXPREAMBLENGTH      DWT_PLEN_64
#define DEFAULT_RXPAC               DWT_PAC8
#define DEFAULT_PCODE               9
#define DEFAULT_NSSFD               1  //! SFD type 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type
#define DEFAULT_DATARATE            DWT_BR_6M8
#define DEFAULT_PHRMODE             DWT_PHRMODE_STD
#define DEFAULT_PHRRATE             DWT_PHRRATE_STD
#define DEFAULT_SFDTO               (64 + 1 + 8 - 8)
#define DEFAULT_STS_MODE            (DWT_STS_MODE_1 | DWT_STS_MODE_SDC) //!< STS mode
#define DEFAULT_STS_LENGTH          DWT_STS_LEN_256  //!< STS length
#define DEFAULT_PDOA_MODE           DWT_PDOA_M3      //!< pdoa mode: on SP1/3 Ipatov_64 + STS_256->PDoA_M3; if Ipatov_64 + STS_64 -> PDoA Mode 1

#define DEFAULT_STS_STATIC          1 //! 1 to re-load STS Key & IV after each Rx & Tx:: TCFM, Listener

/* run-time config */
#define DEFAULT_NODE_ADDR           0x0001  /**< Addr16    */
#define DEFAULT_PANID               0xDECA  /**< PanID */
#define DEFAULT_ANTD                (513.484f * 1e-9 / DWT_TIME_UNITS) /*Total antenna delay*/



/* This configures the delay between end of Tag's Blink_TX and Tag start Rx of Ranging Config message.
 * From Node's view this is a delay between end of reception of Blink's data and start of transmission of preamble of Ranging Config.
 * Should be the same for Node and Tag.
 * */
#define DEFAULT_TAG_BLINK_TX_RC_RX_US       (150*1000) // Changed to 150 ms to support compatibility with DecaRanging Application

 /* This configures the RX timeout while waiting for Ranging Config message */
#define DEFAULT_TAG_RC_RX_TIMEOUT_US        (500)



/* Default configuration initialization */
#define DEFAULT_CONFIG \
{ \
    /* Initialize Decawave driver configuration */ \
    .dwt_config = { \
        .chan            = DEFAULT_CHANNEL, \
        .txPreambLength  = DEFAULT_TXPREAMBLENGTH, \
        .rxPAC           = DEFAULT_RXPAC, \
        .txCode          = DEFAULT_PCODE, \
        .rxCode          = DEFAULT_PCODE, \
        .sfdType         = DEFAULT_NSSFD, \
        .dataRate        = DEFAULT_DATARATE, \
        .phrMode         = DEFAULT_PHRMODE, \
        .phrRate         = DEFAULT_PHRRATE, \
        .sfdTO           = DEFAULT_SFDTO, \
        .stsMode         = DEFAULT_STS_MODE, \
        .stsLength       = DEFAULT_STS_LENGTH, \
        .pdoaMode        = DEFAULT_PDOA_MODE, \
    }, \
    \
    /* Initialize run-time parameters to zero or as needed */ \
    .runtime_params = { \
        .tx_config = { \
            .PGdly = 0x34, \
            .power = 0xfdfdfdfdUL, \
            .PGcount = 0x0, \
        }, \
        .rcDelay_us = DEFAULT_TAG_BLINK_TX_RC_RX_US,\
        .rcRxTo_us  = DEFAULT_TAG_RC_RX_TIMEOUT_US,\
        .ant_rx_a   = (uint16_t)(0.5* DEFAULT_ANTD), \
        .ant_tx_a   = (uint16_t)(0.5* DEFAULT_ANTD), \
        .ant_rx_b   = (uint16_t)(0.5* DEFAULT_ANTD), \
        .xtal_trim  = DEFAULT_XTAL_TRIM, \
    }, \
    \
    /* Initialize knownTagList to zero or as needed */ \
    .known_tag_list = { \
        {0, {0x00, 0x00}, {0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 0}}, \
        {1, {0x00, 0x01}, {0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 1}}, \
    },\
}


/**
 * @brief Structure representing runtime parameters.
 *
 * This structure holds configuration parameters that are determined or modified during
 * the execution of the application. It includes TX configurations, antenna delays, and
 * crystal trim values.
 */
typedef struct runtime_params_s
{
    dwt_txconfig_t tx_config;    /**< TX power, PG delay, TX mode configuration. */

    uint32_t    rcDelay_us;     /**< Node&Tag delay between end reception of UWB blink and start transmission of UWB Ranging Config message */
    uint16_t    rcRxTo_us;      /**< Tag's Receiver timeout only to save power on non-reception of the Ranging Config */

    uint16_t ant_rx_a;           /**< Antenna delay value for the left RX port. */
    uint16_t ant_tx_a;           /**< Antenna delay value for the left TX port. */
    uint16_t ant_rx_b;           /**< Antenna delay value for the right RX port. */

    uint8_t  xtal_trim;          /**< XTAL trim value for frequency calibration. */

} __attribute__((__packed__)) runtime_params_t;


/**
 * @brief Structure holding the configurable parameters of the application.
 *
 * This structure encapsulates the standard Decawave driver configuration, a list of known
 * tags, and the runtime parameters that can be altered during application execution.
 */
typedef struct param_block_s 
{
    dwt_config_t        dwt_config;                    /**< Standard Decawave driver configuration. */
    runtime_params_t    runtime_params;                /**< Runtime parameters for dynamic configuration. */
    tag_addr_slot_t     known_tag_list[MAX_KNOWN_TAG_LIST_SIZE]; /**< Array of known tag addresses and slots. */
    

} __attribute__((__packed__)) param_block_t;


#ifdef __cplusplus
}
#endif

#endif // __DEFAULT_CONFIG_H__H__