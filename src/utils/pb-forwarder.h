/* Copyright (C) 2015-2017
 *
 * pb-forwarder.h
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

#ifndef UTILS_PB_FORWARDER_H_
#define UTILS_PB_FORWARDER_H_

//*****************************************************************************
//
//! \addtogroup utils
//! @{
//!
//! \addtogroup utils_pb_forward Protobuf Forwarder
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

bool pb_forwarder_init( bool (*pfnForward)(uint8_t *pc_data, uint16_t i_len) );

void pb_forwarder_run(void);

bool forwardProtobufToArm(uint8_t *pc_data, uint16_t i_len);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* UTILS_PB_FORWARDER_H_ */
