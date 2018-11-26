/* Copyright (C) 2015-2017
 *
 * timing.h
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Elias Rosch         <eliasrosch@gmail.com>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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

#ifndef UTILS_TIMING_H_
#define UTILS_TIMING_H_

//*****************************************************************************
//
//! \addtogroup utils
//! @{
//!
//! \addtogroup utils_timing Timing
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

/* Protobuf specific includes for timing_getTicks() */
#include "Types.pb.h"


/*!
 * \brief Initialize the timing module.
 *
 * \return TRUE if initialized successfully. FALSE otherwise.
 */
bool timing_init(void);


/*!
 * \brief Initializes and starts a single timer by returning a timeout value.
 *
 * \details The user gives the timeout value that is the relative future time
 *          when the timer expires.
 *          See \ref timing_isTimedOut for an example usage of this API.
 *
 * \param ui32_timeoutMs  Time to expire (in milliseconds).
 *
 * \return Timer instance as reference to the requested timeout.
 *         The returned value can be passed to \ref timing_isTimedOut to check
 *         if the timrout occurred or not.
 */
uint32_t timing_getTimeout(uint32_t ui32_timeoutMs);


/*!
 * \brief Checks wether an existing timer expired or not.
 *
 * \param ui32_tmr  Timer that shall be checked. This value shall be returned
 *                  by a previous call to function \ref timing_getTimeout.
 *
 * \return TRUE if timer expired. FALSE otherwise.
 *
 * \code
 * // === Example usage ===
 *
 * // Initialize the timer module first.
 * timing_init();
 *
 * // Request a timer by specifing a timeout value (e.g. 50 milliseconds here).
 * uint32_t myTimer = timing_getTimeout( 50 );
 *
 * // Check if the timer expired.
 * if( timing_isTimedOut( myTimer ) )
 * {
 *    // The timer expired!
 * }
 * \endcode
 */
bool timing_isTimedOut(uint32_t ui32_tmr);


/*!
 * \brief Gets a timestamp (i.e. ticks since system startup).
 *
 * \param p_tick Pointer to protobuf struct "BetCOM_Ticks".
 *
 * \return TRUE if timestamp set successfully. FALSE otherwise.
 */
bool timing_getTimestamp(Timestamp * const p_tick);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* UTILS_TIMING_H_ */
