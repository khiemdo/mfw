/* Copyright (C) 2015-2017
 *
 * satellady.h
 *
 * Elias Rosch         <eliasrosch@gmail.com>
 * Martin Dold         <martin.dold@gmx.net>
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

#ifndef SATELLADY_SATELLADY_H_
#define SATELLADY_SATELLADY_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_satellady Satellady
//! @{
//
//*****************************************************************************

#include <stdbool.h>

#include "FurunoGps.pb.h"

/*!
 * \brief Initialize the satellady-sensor.
 *
 * \return Returns TRUE, iff satellady-sensor is initialized
 *         successfully. FALSE otherwise.
 */
bool satellady_init();

/*!
 * \brief Handle the satellady-sensor state.
 *
 * \return None.
 */
void satellady_run();

/*!
 * \brief Gets the current information of the satellady and writes the values
 *        to the provided protobuf gps-msg.
 *
 * \details Be aware, that this function has to be used in combination with the
 *          *_run() function, as that function handles the actual
 *          UART transmission.
 *
 * \param p_gps_msg Pointer to gps-msg struct.
 *
 * \return TRUE if new sensor data is written into the message. FALSE otherwise.
 */
bool satellady_get(FurunoGps* const p_gps_msg);

/*!
 *  \brief Handles a Gps configuration message and performs the requested
 *         settings within the module.
 *
 *  \returns TRUE if config is set successfully. FALSE otherwise.
 */
bool satellady_setConf();

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SATELLADY_SATELLADY_H_ */
