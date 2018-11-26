/* Copyright (C) 2015-2017
 *
 * pps.h
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Martin Dold         <martin.dold@gmx.net>
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

#ifndef SENSORS_PPS_PPS_H_
#define SENSORS_PPS_PPS_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_pps PPS
//! @{
//
//*****************************************************************************

#include <stdbool.h>

#include "Pps.pb.h"


/*!
 * \brief Initialize the PPS module.
 *
 * \return None.
 */
bool pps_init(void);

/*!
 * \brief Handle PPS module state.
 *
 * \return None.
 */
void pps_run(void);

/*!
 * \brief gets the information about the last PPS signal.
 *
 * \param pps_msg Pointer to pps-msg struct.
 *
 * \return TRUE if data is set to the given message. FALSE otherwise.
 */
bool pps_get(Pps *const pps_msg);

/*!
 *  \brief Handles a PPS configuration message and performs the requested
 *         settings within the module.
 *
 *  \param ppsconf_msg Read only pointer to a configuration message.
 *
 *  \returns TRUE if config is set successfully. FALSE otherwise.
 */
bool pps_setConf(PpsConf *const ppsconf_msg);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SENSORS_PPS_PPS_H_ */
