/* Copyright (C) 2015-2017
 *
 * mpl.h
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

#ifndef MPL_MPL_H_
#define MPL_MPL_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_mpl MPL
//! @{
//
//*****************************************************************************

#include <stdbool.h>

#include "Barometer.pb.h"


#define MPL_SLAVE_ADDR 0x60

typedef enum
{
  E_MPL_STATE_INIT1,
  E_MPL_STATE_INIT2,
  E_MPL_STATE_IDLE,
  E_MPL_STATE_PENDING,
  E_MPL_STATE_READY,
  E_MPL_STATE_TIMEOUT,
}E_MPL_STATE_T;

/*!
 * \brief Initialize the mpl-sensor.
 *
 *
 * \return Returns true if tube-angle-sensor is initialized
 *         successfully, else otherwise.
 */
bool MPL_init();

/*!
 * \brief Handle the mpl-sensor state.
 *
 * \return None.
 */
void MPL_run();

/*!
 * \brief gets the current barometer values and writes them to protobuf msg.
 *
 * \details Be aware, that this function has to be used in combination with the
 *          *_run() function, as that function handles the actual sensor readout.
 *          This function only copies local new values to msg.
 *
 * \param p_baro_msg	Pointer to msg struct.
 *
 * \return TRUE if new sensor data is written into the message. FALSE otherwise.
 */
bool MPL_get(Barometer* const p_baro_msg);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* MPL_MPL_H_ */
