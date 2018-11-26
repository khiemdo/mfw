/* Copyright (C) 2015-2017
 *
 * hal_ethernet.h
 *
 * Martin Dold    <martin.dold@gmx.net>
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

#ifndef HAL_ETH_H_
#define HAL_ETH_H_

//*****************************************************************************
//
//! \addtogroup hal_api HAL ETH API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* TODO: docu */
bool hal_eth_init(void);

/* TODO: docu */
const uint8_t *hal_eth_getMacAddress(void);


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif /* HAL_ETH_H_ */
