/* Copyright (C) 2015-2017
 *
 * hal.h
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

#ifndef HAL_HAL_H_
#define HAL_HAL_H_

//*****************************************************************************
//
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_init HAL Init
//! @{
//
//*****************************************************************************

#include <stdbool.h>


/*!
 * \brief Main function to initialize the basic hardware abstraction layer.
 *
 * \details This function initializes the generic lowest level HAL
 *          functionalities that are not binded to a specific SW component
 *          using it. In other words, it calls all the init() functions that do
 *          not return a context (pointer) to be used later on, such as:
 *          - \ref hal_mcu_init()
 *          - \ref hal_dma_init()
 *          - \ref hal_gpio_init()
 *          - \ref hal_crc_init()
 *
 * \return Returns true if HAL is initialized successfully. false otherwise.
 */
bool hal_init(void);


//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_H_ */
