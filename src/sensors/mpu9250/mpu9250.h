/* Copyright (C) 2015-2017
 *
 * mpu9250.h
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

#ifndef SENSORS_MPU9250_H_
#define SENSORS_MPU9250_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_mpu9250 MPU9250
//! @{
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>

#include "hal_spi.h"

#include "Imu.pb.h"

/*!
 * \brief Initialize the MPU9250-sensor.
 *
 * \param e_hal_spi_port Defines the SPI port to be intialized.
 * \param ui32_spiClock Defines the SPI clock speed.
 *
 * \return TRUE if init was succesful. FALSE otherwise.
 */
bool mpu9250_init(E_HAL_SPI_PORT_t e_hal_spi_port, uint32_t ui32_spiClock);

/*!
 * \brief Provides the MFW run API.
 *
 * \details Handles the state machine and timing of read and write functions. Triggers and fetches measurements
 * according to set resolution and refresh rate.
 *
 * \return None.
 */
void mpu9250_run(void);

/*!
 * \brief gets the current IMU measurements of the MPU9250
 * and writes the values to the provided protobuf mpu-msg.
 *
 * \param mpu_msg	Pointer to MPU msg struct.
 *
 * \return TRUE if data is set to the given message. FALSE otherwise.
 */
bool mpu9250_get(Imu *const mpu_msg);

/*!
 *  \brief Handles a MPU9250 configuration message and performs the requested
 *         settings within the module.
 *
 *  \param p_imuConf Read only pointer to a configuration message.
 *
 *  \returns TRUE if config is set successfully. FALSE otherwise.
 */
bool mpu9250_setConf(ImuConf *const p_imuConf);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SENSORS_TUBE_ANGLE_SENSOR_TUBE_ANGLE_SENSOR_H_ */
