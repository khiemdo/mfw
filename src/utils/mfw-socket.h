/* Copyright (C) 2015-2017
 *
 * mfw-socket.h
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

#ifndef MFW_SOCKET_H_
#define MFW_SOCKET_H_

//*****************************************************************************
//
//! \example test-app-ethernet.c
//!
//! \addtogroup utils Utility modules
//! @{
//!
//! \addtogroup utils_mfw_socket UDP socket for MFW communication
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

/* TODO: docu */
bool mfw_socket_init( uint32_t ui32IPAddr,
                      uint32_t ui32NetMask,
                      uint32_t ui32GWAddr
                    );

/* TODO: docu */
bool mfw_socket_udpStart( uint16_t srcPort,
                          uint32_t destIPAddr,
                          uint16_t destPort,
                          uint8_t *pc_rxDataBuf,
                          uint16_t rxDataBufLen,
                          void (*callbackRxDataRdy)(uint8_t *pc_data, uint16_t ui16_len)
                        );

/* TODO: docu */
void mfw_socket_udpRun(void);

/* TODO: docu */
bool mfw_socket_udpSend(uint8_t *data, uint16_t dataLen);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* MFW_SOCKET_H_ */
