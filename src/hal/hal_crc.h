/* Copyright (C) 2015-2017
 *
 * hal_crc.h
 *
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

#ifndef HAL_HAL_CRC_H_
#define HAL_HAL_CRC_H_

//*****************************************************************************
//
//! \example test-app-sproto.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_crc_api HAL CRC API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

/*! \brief   Initializes CRC controller of the microcontroller.
 *
 *  \details A call to this function is required prior to any CRC operations!
 *           Following operations are performed by this function:
 *
 *           - Enable the CRC controller at the system level.
 *           - Configure the CRC controller for CRC-CCITT that uses
 *             CRC polynom "0x1021".
 *           - Fallback to software CRC if hardware module is not available.
 *
 *  \return  TRUE, if initialization was done successfully. FALSE otherwise.
 */
bool hal_crc_init(void);


/*! \brief   Compute the CRC of a given message.
 *
 *  \details The complete message must be given in a single call to compute
 *           the CRC correctly.
 *
 *  \warning A call to \ref hal_crc_init() is required prior to any CRC
 *           calculations!
 *
 *  \param   pc_data  Pointer to the data to calculate the CRC for.
 *  \param   ui16_len Length of the given data.
 *
 *  \return  CRC result of the given message.
 */
uint16_t hal_crc_calculate(const uint8_t * pc_data, uint16_t ui16_len);


//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_CRC_H_ */
