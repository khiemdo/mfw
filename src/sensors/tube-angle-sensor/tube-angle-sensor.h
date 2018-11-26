/* Copyright (C) 2015-2017
 *
 * tube-angle-sensor.h
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

#ifndef SENSORS_TUBE_ANGLE_SENSOR_TUBE_ANGLE_SENSOR_H_
#define SENSORS_TUBE_ANGLE_SENSOR_TUBE_ANGLE_SENSOR_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_tas Tube Angle Sensor (TAS)
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "AngleEncoder.pb.h"

/*!
 * \brief Initialize the tube-angle-sensor.
 *
 * \details Please note that the module internally is disabled by default after
 *          initialization. To enable this module a call to
 *          \ref tube_angle_sensor_setConf() is required with parameter
 *          "activated" set to true.
 *
 * \param ui32_spiClock Defines the spi-clk speed to be intialized.
 *
 * \return Returns pointer to context variable if tube-angle-sensor is initialized
 *         successfully. NULL otherwise.
 */
bool tube_angle_sens_init(uint32_t ui32_spiClock);

/*!
 * \brief Handle the tube-angle-sensor state.
 *
 * \return None.
 */
void tube_angle_sens_run();

/*!
 * \brief gets the current elevation and rotation of the tube-angle-sensor
 * and writes the values to the provided protobuf tas-msg.
 *
 * \param tas_msg	Pointer to tas-msg struct.
 *
 * \return TRUE if new sensor data is written into the message. FALSE otherwise.
 */
bool tube_angle_sensor_get_blocking(AngleEncoder * const tas_msg);

/*!
 * \brief Gets the current elevation and rotation of the tube-angle-sensor
 *        and writes the values to the provided protobuf tas-msg.
 *
 * \details Be aware, that this function has to be used in combination with the
 *          *_run() function, as that function handles the actual SPI
 *          transmission. This function only copies local new values to msg.
 *
 * \return TRUE if new sensor data is written into the message. FALSE otherwise.
 */
bool tube_angle_sensor_get(AngleEncoder* const p_ae_el_msg, AngleEncoder* const p_ae_ro_msg);

bool tube_angle_sensor_get_el(AngleEncoder* const p_ae_el_msg);

bool tube_angle_sensor_get_ro(AngleEncoder* const p_ae_ro_msg);

/*!
 *  \brief Handles a TubeAngle configuration message and performs the requested
 *         settings within the module.
 *
 *  \param p_tasConf Read only pointer to a tube anle configuration message.
 *
 *  \returns TRUE if config is set successfully. FALSE otherwise.
 */
bool tube_angle_sensor_setConf(AngleEncoderConf * const p_tasConf);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SENSORS_TUBE_ANGLE_SENSOR_TUBE_ANGLE_SENSOR_H_ */
