/* Copyright (C) 2015-2017
 *
 * hal_dma.h
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

#ifndef HAL_HAL_DMA_H_
#define HAL_HAL_DMA_H_

//*****************************************************************************
//
//! \example test-app-spi.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_dma_api HAL DMA API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

/*! \brief Initializes DMA controller of the microcontroller.
 *
 *  \details A call to this function is required prior to any DMA operations
 *           (e.g. to use the DMA controller for SPI transmissions)!
 *           Following operations are performed by this function:
 *
 *           - Enable the uDMA controller at the system level.
 *           - Enable the uDMA controller.
 *           - Initialize control table to use for channel control structures.
 *
 *  \return TRUE, if initialization was done successfully. FALSE otherwise.
 */
bool hal_dma_init(void);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_DMA_H_ */
