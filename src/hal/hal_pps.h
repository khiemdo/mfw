/* Copyright (C) 2015-2017
 *
 * hal_pps.h
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
 * Martin Dold         <martin.dold@gmx.net>
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
 
#ifndef HAL_HAL_PPS_H_
#define HAL_HAL_PPS_H_

//*****************************************************************************
//
//! \example test-app-1pps-isr
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_pps_api HAL PPS API
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "Types.pb.h"
#include "Pps.pb.h"

/*!
 * \brief Initialize the PPS driver module.
 *
 * \details This function shall be called prior to any PPS use.
 *
 * \return TRUE if initialized successfully. FALSE otherwise.
 */
bool hal_pps_init(void);

/*!
 * \brief Interrupt service routine for timestamping.
 *
 * \details Is executed immediately on interrupt for shortest possible delay.
 *          Declared inline to avoid unnecessary jumps in code. Compiler should
 *          be called with highest possible -O option. The function reads the current
 *          counter value of the assigned timer and resets it. The PPS counter is
 *          increased. Both values are stored in internal variables.
 *
 * \return None.
 */
inline void hal_pps_isr(void);

/*!
 * \brief Provides external access to PPS information.
 *
 * \details Return the PPS information from the latest interrupt.
 *
 * \param pps_msg Pointer to return message, where data is inserted.
 *
 * \return TRUE if data was available and returned. FALSE otherwise.
 */
inline bool hal_pps_getCurrentPPS(Pps *const pps_msg);

/*!
 * \brief Provides HAL function for precise timestamp.
 *
 * \details Returns the current counter value and current PPS count.
 *
 * \param timestamp_msg Pointer to return message, where data is inserted.
 *
 * \return TRUE if data was available and returned. FALSE otherwise.
 */
inline bool hal_pps_getTimestamp(Timestamp *const timestamp_msg);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_PPS_H_ */
