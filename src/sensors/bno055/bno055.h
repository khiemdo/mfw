/* Copyright (C) 2015-2017
 *
 * bno055.h
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

#ifndef SENSORS_BNO055_BNO055_H_
#define SENSORS_BNO055_BNO055_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_bno055 BNO055
//! @{
//
//*****************************************************************************

#include "hal_i2c_ti_drv.h"
#include "Imu.pb.h"


#define BNO055_I2C_PORT   E_HAL_I2C_PORT_2


/*!
 * \brief Initialize the BNO055-sensor.
 *
 * \details Please note that the module internally is disabled by default after
 *          initialization. To enable this module a call to
 *          \ref bno055_sensor_setConf() is required with parameter
 *          "activated" set to true.
 *
 * \return Returns true if tube-angle-sensor is initialized
 *         successfully. FALSE otherwise.
 */
bool bno055_init();

/*!
 * \brief Handle the bno055-sensor state.
 *
 * \return None.
 */
void bno055_run();

/*!
 * \brief gets the current measurements of the bno055-sensor
 * and writes the values to the provided protobuf imu-msg.
 *
 * \param imu_msg	Pointer to imu-msg struct.
 *
 * \return TRUE if new sensor data is set to the message. FALSE otherwise.
 */
bool bno055_get(Imu* const imu_msg);

/*!
 *  \brief Handles a IMU configuration message and performs the requested
 *         settings within the module.
 *
 *  \param p_imuConf Read only pointer to a tube anle configuration message.
 *
 *  \returns TRUE if config is set successfully. FALSE otherwise.
 */
bool bno055_sensor_setConf(ImuConf * const p_imuConf);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SENSORS_BNO055_BNO055_H_ */

