/* Copyright (C) 2015-2017
 *
 * hal_mcu.h
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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

#ifndef HAL_HAL_MCU_H_
#define HAL_HAL_MCU_H_

//*****************************************************************************
//
//! \example test-app-spi.c
//! \example test-app-uart.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_mcu_api HAL MCU API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

/*!
 * \brief Initialize the microcontroller driver module.
 *
 *  \details A call to this function is required prior to any HAL operations
 *           (e.g. to use the HAL SPI driver for SPI transmissions)!
 *           Following operations are performed by this function:
 *
 *           - Initialize the system clock module (default: 120MHz system clock).
 *           - Enable processor interrupts.
 *
 * \return TRUE if initialized successfully. FALSE otherwise.
 */
bool hal_mcu_init(void);


/*!
 * \brief Returns the current system clock in MHz.
 *
 * \return Current system clock speed in MHz.
 */
uint32_t hal_mcu_getSysClock(void);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_MCU_H_ */
