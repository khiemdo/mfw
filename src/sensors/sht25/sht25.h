/* Copyright (C) 2015-2017
 *
 * sht25.h
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

#ifndef SENSORS_SHT25_SHT25_H_
#define SENSORS_SHT25_SHT25_H_

//*****************************************************************************
//
//! \addtogroup sensors
//! @{
//!
//! \addtogroup sensors_sht25 SHT25
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "Humidity.pb.h"

typedef uint16_t SHT25_DATA_t;

typedef uint8_t SHT25_CRC_t;

/*!
 * \brief Provides the MFW init API.
 *
 * \details Initialises necessary internal variables, registers I2C context with
 *          I2C bus manager, sets state machines and issues a reset of the
 *          sensor.
 *
 * \return TRUE if succesfully initialized. FALSE otherwise.
 *
 */
bool SHT25_init(void);

/*!
 * \brief Provides the MFW run API.
 *
 * \details Handles the state machine and timing of read and write functions.
 *          Triggers and fetches the measurements according to set resolution
 *          and refresh rate.
 *
 * \return None.
 */
void SHT25_run(void);

/*!
 * \brief Provides the MFW get API.
 *
 * \details Handles requests for data and returns data if available.
 *
 * \param humidity_msg Pointer to return message where data is inserted.
 *
 * \return TRUE if data was available and returned. FALSE otherwise.
 */
bool SHT25_get(Humidity *const humidity_msg);

/*!
 * \brief Provides the MFW setConf API.
 *
 * \details Handles received BetCONF messages and sets refresh rate.
 *
 * \param humidityConf_msg Pointer to the configuration message.
 *
 * \return None.
 */
void SHT25_setConf(HumidityConf *const humidityConf_msg);

void SHT25_reset(void);

uint8_t *SHT25_getStatus();
bool SHT25_enableHeater(void);
bool SHT25_disableHeater(void);
bool SHT25_enableOTPReload(void);
bool SHT25_disableOTPReload(void);
bool SHT25_setResolution(uint8_t ui8_resolution);
void SHT25_triggerMeasurement(void);
void SHT25_fetchMeasurement(void);
void SHT25_getMeasurement(void);

/*!
 * \brief Checks the CRC8 checksum of the measurement.
 *
 * \details This function can be used for checking the validity of the
 *          measurement data. It calculates the CRC8 checksum with the
 *          polynomial 0x131 and compares it with the checksum received
 *          from the sensor.
 *
 * \param ui16_data Raw date received from the sensor.
 * \param ui8_crc CRC8 value received from the sensor.
 *
 * \return TRUE if checksum matches. FALSE otherwise.
 */
bool SHT25_checkCRC(SHT25_DATA_t ui16_data, SHT25_CRC_t ui8_crc);


//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SENSORS_SHT25_SHT25_H_ */
