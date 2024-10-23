/**
 * @file      error.h
 *
 * @brief     Header file for errors
 *
 * @author    Decawave
 *
 * @attention //TODO: put attention here
 *
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_
// Include necessary headers
#include "esp_err.h"

// Define a base value for your custom errors to avoid conflict with ESP-IDF error codes
#define ESP_ERR_CUSTOM_BASE (0x7000)

// Define your custom error enumeration
typedef enum {
    _NO_ERR = ESP_OK,                  // 0
    _ERR = ESP_FAIL,                    // -1
    _ERR_Busy = ESP_ERR_TIMEOUT,        // -3
    _ERR_Timeout = ESP_ERR_TIMEOUT,     // -3

    // Starting from a custom base to avoid conflict
    _ERR_DEVID = ESP_ERR_CUSTOM_BASE + 1,          // 0x7001
    _ERR_IWDG,                                      // 0x7002
    _ERR_INSTANCE,                                  // 0x7003
    _ERR_INIT,                                      // 0x7004
    _ERR_IMU_INIT,                                  // 0x7005
    _ERR_TxBuf_Overflow,                            // 0x7006
    _ERR_RxBuf_Overflow,                            // 0x7007
    _ERR_Usb_Tx,                                    // 0x7008
    _ERR_Flash_Ob,                                  // 0x7009
    _ERR_Flash_Prog,                                // 0x700A
    _ERR_Flash_Erase,                               // 0x700B
    _ERR_Flash_Error,                               // 0x700C
    _ERR_Flash_Verify,                              // 0x700D
    _ERR_Flash_Protected,                           // 0x700E
    _ERR_LSM_R,                                     // 0x700F
    _ERR_LSM_W,                                     // 0x7010
    _ERR_SPI,                                       // 0x7011
    _ERR_SPI_RRX,                                   // 0x7012
    _ERR_SPI_WTX,                                   // 0x7013
    _ERR_SPI_DMA,                                   // 0x7014
    _ERR_UART_DMA,                                  // 0x7015
    _ERR_UART_INIT,                                 // 0x7016
    _ERR_UART_RX,                                   // 0x7017
    _ERR_UART_TX,                                   // 0x7018
    _ERR_UART_RxCplt,                               // 0x7019
    _ERR_UART_RxCplt_Overflow,                      // 0x701A
    _ERR_USB_UART_RX,                               // 0x701B
    _ERR_TCFM,                                      // 0x701C
    _ERR_TWR_CANNOT_START,                          // 0x701D
    _ERR_MEM_CORRUPTED,                             // 0x701E
    _ERR_Configure_WKUP_Timer,                      // 0x701F
    _ERR_PLL,                                       // 0x7020

    /* TWR */
    _ERR_Twr_Bad_State,                             // 0x7021
    _ERR_Not_Twr_Frame,                             // 0x7022
    _ERR_Unknown_Tag,                               // 0x7023
    _ERR_DelayedTX_Late,                            // 0x7024
    _ERR_Range_Calculation,                         // 0x7025
    _ERR_Ranging_Config,                            // 0x7026
    _ERR_RC_Version_Unknown,                        // 0x7027
    _ERR_Non_Compatible_TWR_Parameters,             // 0x7028
    _NO_Err_New_Tag,                                // 0x7029
    _NO_Err_Tx_Sent,                                // 0x702A
    _NO_Err_Start_Rx,                               // 0x702B
    _NO_Err_Final,                                  // 0x702C
    _NO_Err_Ranging_Config,                         // 0x702D
    _NO_Err_Ranging_Update,                         // 0x702E
    _NO_Err_Response,                               // 0x702F
    _NO_Err_Idata,                                  // 0x7030
    _NO_Err_Rdata,                                  // 0x7031
    _NO_Err_Can_Sleep,                              // 0x7032

    /* USB2SPI */
    _ERR_Usb2Spi_ptr_busy,                          // 0x7033
    _ERR_Usb2Spi_ptr_alloc,                         // 0x7034

    /* RTOS */
    _ERR_General_Error,                             // 0x7035
    _ERR_Create_Task_Bad,                           // 0x7036
    _ERR_Timer_Create_Bad,                           // 0x7037
    _ERR_Timer_Start_Bad,                            // 0x7038
    _ERR_Signal_Bad,                                // 0x7039
    _ERR_Cannot_Delete_Timer,                       // 0x703A
    _ERR_Cannot_Delete_Task,                        // 0x703B
    _ERR_Cannot_Delete_usb2spiTask,                 // 0x703C
    _ERR_Cannot_Delete_tcfmTask,                    // 0x703D
    _ERR_Cannot_Delete_tcwmTask,                    // 0x703E
    _ERR_Cannot_Delete_imuTask,                     // 0x703F
    _ERR_Cannot_Delete_rtlsTask,                    // 0x7040
    _ERR_Cannot_Delete_rxTask,                      // 0x7041
    _ERR_Cannot_Delete_calcTask,                    // 0x7042
    _ERR_Cannot_Delete_twrTask,                     // 0x7043
    _ERR_Cannot_Delete_commTask,                    // 0x7044
    _ERR_Cannot_Send_Mail,                          // 0x7045
    _ERR_Cannot_Alloc_Mail,                          // 0x7046
    _ERR_Cannot_Alloc_Memory,                        // 0x7047
    _ERR_Cannot_Alloc_NodeMemory,                    // 0x7048
    _ERR_Malloc_Failed,                             // 0x7049
    _ERR_Stack_Overflow,                            // 0x704A
    _ERR_No_pTwrInfo                                // 0x704B
} error_e;



#endif /* INC_ERROR_H_ */
