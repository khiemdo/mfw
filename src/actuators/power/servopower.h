/* Copyright (C) 2015-2017
 *
 * servopower.h
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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

#ifndef POWER_SERVOPOWER_H_
#define POWER_SERVOPOWER_H_

//*****************************************************************************/
//
//! \addtogroup actuators
//! @{
//!
//! \addtogroup actuators_servopower Servo-Power
//! @{
///
//*****************************************************************************/

#include <stdbool.h>

/*!
 * \brief Activate servo power control.
 *
 * \details This function activates the onboard servo power control.
 *
 * \return TRUE if activated successfully. FALSE otherwise.
 */
bool servopower_init(void);

/*!
 * \brief Returns current setting of power source.
 *
 * \details This function gives the current status of the power source control.
 *
 * \return TRUE if active. FALSE otherwise.
 */
bool servopower_status(void);

/*!
 * \brief Activate servo power source.
 *
 * \details This function activates the onboard servo power source.
 *          Default/reset state is ON.
 *
 * \return TRUE if activated successfully. FALSE otherwise.
 */
bool servopower_on(void);

/*!
 * \brief Deactivate servo power source.
 *
 * \details This function deactivates the onboard servo power source.
 *          Default/reset state is ON.
 *
 * \return TRUE if activated successfully. FALSE otherwise.
 */
bool servopower_off(void);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* POWER_SERVOPOWER_H_ */
