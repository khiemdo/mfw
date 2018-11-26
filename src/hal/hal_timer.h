/* Copyright (C) 2015-2017
 *
 * hal_timer.h
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
 *
 * Systems Control and Optimization Laboratory - www.syscop.de
 * Institute for Microsystems Engineering - www.imtek.de
 * Faculty of Engineering - www.tf.uni-freiburg.de
 * University of Freiburg - www.uni-freiburg.de
 *
 * This file is part of the TOPCORE platform - topcore.syscop.de
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

#ifndef HAL_HAL_TIMER_H_
#define HAL_HAL_TIMER_H_

//*****************************************************************************
//
//! \example test-app-spi.c
//! \example test-app-uart.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_timer_api HAL Timer API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

/*!
 * \brief Initialize the HAL timer module.
 *
 * \details A call to this function is required prior to use any timers!
 *          After a call to this function, a HW timer is initialized to work
 *          as a time base with a resolution of 1 millisecond.
 *          Following operations are performed by this function:
 *
 *          - Initialize a MCU internal timer with a resolution of 1ms.
 *          - Enable timer interrupt.
 *
 * \return TRUE if initialized successfully. FALSE otherwise.
 */
bool hal_timer_init(void);

/*!
 * \brief Initializes and starts a single timer by returning a timeout value.
 *
 * \details The user gives the timeout value that is the relative future time
 *          when the timer expires.
 *          See hal_timer_isTimedOut for an example usage of this API.
 *
 * \param ui32_timeoutMs  Time to expire (in milliseconds).
 *
 * \return Timer instance as reference to the requested timeout.
 *         The returned value can be passed to \ref hal_timer_isTimedOut to check
 *         if the timrout occurred or not.
 */
uint32_t hal_timer_getTimeout(uint32_t ui32_timeoutMs);

/*!
 * \brief Checks wether an existing timer expired or not.
 *
 * \param ui32_tmr  Timer that shall be checked. This value shall be returned
 *                  by a previous call to function \ref hal_timer_getTimeout.
 *
 * \return TRUE if timer expired. FALSE otherwise.
 *
 * \code
 * // === Example usage ===
 *
 * // Initialize the timer module first.
 * hal_timer_init();
 *
 * // Request a timer by specifing a timeout value (e.g. 50 milliseconds here).
 * uint32_t myTimer = hal_timer_getTimeout( 50 );
 *
 * // Check if the timer expired.
 * if( hal_timer_isTimedOut( myTimer ) )
 * {
 *    // The timer expired!
 * }
 * \endcode
 */
bool hal_timer_isTimedOut(uint32_t ui32_tmr);

/* TODO: docu */
bool hal_timer_registerCallback(uint32_t intervall, void (*pfn_callback)(void));

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_TIMER_H_ */
