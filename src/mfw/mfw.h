/* Copyright (C) 2015-2017
 *
 * mfw.h
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

#ifndef MFW_H_
#define MFW_H_

//*****************************************************************************/
//
//! \addtogroup mfw MFW
//! @{
//!
//! \addtogroup mfw_api API
//! @{
//
//*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Initializes the measurement framework.
 *
 * @return True if initialized successfully, false otherwise.
 */
bool mfw_init(void);

/*!
 * @brief Runs the measurement framework.
 *
 * @return None.
 */
void mfw_run(void);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* MFW_H_ */
