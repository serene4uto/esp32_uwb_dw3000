/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "port.h"
#include "deca_device_api.h"


// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------

extern const dw_t *pDwChip;

// Critical section handling
portMUX_TYPE deca_mux = portMUX_INITIALIZER_UNLOCKED;

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function should disable interrupts. This is called at the start of a critical section
 * It returns the IRQ state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW3000 interrupt
 */
decaIrqStatus_t decamutexon(void)
{
    // Enter critical section (disables interrupts on the current core)
    portENTER_CRITICAL(&deca_mux);

    // Disable the DW3000 interrupt
    // disable_dw3000_irq(); --> crashes the system

    // Since we're in a critical section, no need to return a value here.
    return 0;  // Dummy return to match the signature
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW3000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW3000 interrupt
 */
void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
    // Re-enable the DW3000 interrupt
    // enable_dw3000_irq(); --> crashes the system

    // Exit critical section (re-enables interrupts on the current core)
    portEXIT_CRITICAL(&deca_mux);
}
