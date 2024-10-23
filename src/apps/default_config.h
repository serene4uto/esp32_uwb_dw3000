


#ifndef __DEFAULT_CONFIG_H__H__
#define __DEFAULT_CONFIG_H__H__ 1


#ifdef __cplusplus
 extern "C" {
#endif

#include "deca_device_api.h"

/* Hardcoded EUI64s for demo */
/* ANCHOR EUI64s */ 
// #define TWR_ANCHOR_MASTER_EUI64             {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x08, 0x0F} /* Anchor Master's EUI64 */
// #define TWR_ANCHOR_DEV1_EUI64               {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x08, 0x01} /* Anchor Device 1's EUI64 */
// #define TWR_ANCHOR_DEV2_EUI64               {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x08, 0x02} /* Anchor Device 2's EUI64 */
// #define TWR_ANCHOR_DEV3_EUI64               {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x08, 0x03} /* Anchor Device 3's EUI64 */
// #define TWR_ANCHOR_DEV4_EUI64               {0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x08, 0x04} /* Anchor Device 4's EUI64 */

/* Tag EUI64 */
// #define TWR_TAG_DEV1_EUI64                  {0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x07, 0x01} /* Tag Device 1's EUI64 */
// #define TWR_TAG_DEV2_EUI64                  {0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x07, 0x02} /* Tag Device 2's EUI64 */


#define TWR_ANCHOR_MASTER_EUI16             0x080F /* Anchor Master's EUI16 */
#define TWR_ANCHOR_DEV1_EUI16               0x0801 /* Anchor Device 1's EUI16 */
#define TWR_ANCHOR_DEV2_EUI16               0x0802 /* Anchor Device 2's EUI16 */
#define TWR_ANCHOR_DEV3_EUI16               0x0803 /* Anchor Device 3's EUI16 */
#define TWR_ANCHOR_DEV4_EUI16               0x0804 /* Anchor Device 4's EUI16 */


#define TWR_TAG_DEV1_EUI16                  0x0701 /* Tag Device 1's EUI16 */
#define TWR_TAG_DEV2_EUI16                  0x0702 /* Tag Device 2's EUI16 */


/* UWB config */
#define DEFAULT_CHANNEL             5                   //!< Channel number (5 to 9)
#define DEFAULT_TXPREAMBLENGTH      DWT_PLEN_128        //!< Preamble length. Used in TX only.
#define DEFAULT_RXPAC               DWT_PAC8            //!< Preamble acquisition chunk size. Used in RX only.
#define DEFAULT_PCODE               9                   //!< TX RX preamble code. Used in TX RX.
#define DEFAULT_NSSFD               1                   //! SFD type 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type
#define DEFAULT_DATARATE            DWT_BR_6M8          //!< Data rate. 
#define DEFAULT_PHRMODE             DWT_PHRMODE_STD     //!< PHY header mode.
#define DEFAULT_PHRRATE             DWT_PHRRATE_STD     //!< PHY header rate.
#define DEFAULT_SFDTO               (128 + 1 + 8 - 8)   //!< SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
#define DEFAULT_STS_MODE            DWT_STS_MODE_OFF    //!< STS mode
#define DEFAULT_STS_LENGTH          DWT_STS_LEN_64      //!< STS length
#define DEFAULT_PDOA_MODE           DWT_PDOA_M0         //!< pdoa mode: on SP1/3 Ipatov_64 + STS_256->PDoA_M3; if Ipatov_64 + STS_64 -> PDoA Mode 1

#define DEFAULT_STS_STATIC          1 //! 1 to re-load STS Key & IV after each Rx & Tx:: TCFM, Listener

/* run-time config */
// #define DEFAULT_NODE_ADDR           0x0001  /**< Addr16    */
// #define DEFAULT_PANID               0xDECA  /**< PanID */
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


typedef union 
{
    uint8_t     bytes[2];
    uint16_t    eui16;
} __attribute__((__packed__)) dev_eui16_t;


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
} __attribute__((__packed__)) param_block_t;


#ifdef __cplusplus
}
#endif

#endif // __DEFAULT_CONFIG_H__H__