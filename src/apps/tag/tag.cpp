/**
 * @file    tag.c
 * @brief    Tag Application Layer
 *             TWR functions collection
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#include "tag.h"



//-----------------------------------------------------------------------------

/* @brief     app layer
 *     RTOS independent application layer function.
 *     initialising of TWR from scratch.
 *     This MUST be executed in protected mode.
 *
 *     !!!! It is assumed DW IC is reset prior to calling this function !!!!!
 *
 *     This will setup the process of:
 *     1. broadcast blink / wait for Ranging Config response;
 *     2. receive setup parameters from Ranging Config;
 *     3. if version of Ranging Config is not compatible, keep blinking;
 *     4. otherwise setup slot, new panID, framefiltering, address, TWR timings;
 *     6. switch off blinking timer and switch on precise WUP timer;
 *     5. range to the Node addr from MAC of Ranging Config
 * */
error_e tag_process_init(void)
{
    error_e err = _NO_ERR;


    return err;
}




void tag_hw_timer_cb() {
    
}