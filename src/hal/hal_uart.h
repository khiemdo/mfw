/* Copyright (C) 2015-2017
 *
 * hal_uart.h
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Elias Rosch         <eliasrosch@gmail.com>
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

#ifndef HAL_HAL_UART_H_
#define HAL_HAL_UART_H_

//*****************************************************************************
//! \example test-app-uart.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_uart_api HAL UART API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*! Available UART ports provided and handled by the UART module. */
typedef enum
{
  /*! UART0: Rx = PA0, Tx = PA1 */
  E_HAL_UART_PORT_0,

  /*! UART1: Rx = PB0, Tx = PB1 */
  E_HAL_UART_PORT_1,

  #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
      defined(TARGET_IS_TM4C129_RA1) ||                                         \
      defined(TARGET_IS_TM4C129_RA2)
  /*! UART2: Rx = PD4, Tx = PD5 */
  E_HAL_UART_PORT_2,

  /*! UART3: Rx = PJ0, Tx = PJ1 */
  E_HAL_UART_PORT_3,

  /*! UART4: Rx = PK0, Tx = PK1 */
  E_HAL_UART_PORT_4,

  /*! UART5: Rx = PC6, Tx = PC7 */
  E_HAL_UART_PORT_5,

  /*! UART6: Rx = PP0, Tx = PP1
   *
   * \warning UART6 is not capable of DMA operation as the DMA channel
   *          conflicts with DMA SPI0!
   */
  E_HAL_UART_PORT_6,

  /*! UART7: Rx = PC4, Tx = PC5 */
  E_HAL_UART_PORT_7
  #endif

} E_HAL_UART_PORT_t;

/*! Typedef of structure defining the context of an UART port. */
typedef struct S_HAL_UART_CTX_T s_hal_uart_ctx_t;


/*!
 * \brief Initialize the UART.
 *
 * \param e_port          Defines the UART port to be intialized.
 * \param ui32_baudRate   Defines the UART baud rate for port to be intialized.
 *
 * \return Returns pointer to context variable if UART is initialized
 *         successfully. NULL otherwise.
 */
s_hal_uart_ctx_t * hal_uart_init(E_HAL_UART_PORT_t e_port, uint32_t ui32_baudRate);

/*!
 * \brief Reads data from the UART.
 *
 * \param ps_ctx   Pointer to context variable.
 * \param pc_data  Pointer to destination memory where data will be copied to.
 * \param dataLen  Length of the given destination memory.
 *
 * \return Returns the number of read bytes.
 */
uint16_t hal_uart_read(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_data, uint16_t dataLen);

/*!
 * \brief Writes data to the UART.
 *
 * \param ps_ctx   Pointer to context variable.
 * \param pc_data  Pointer to the data to write to the UART.
 * \param dataLen  Length of the given data.
 *
 * \return Returns the number of written bytes.
 */
uint16_t hal_uart_write(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_data, uint16_t dataLen);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_UART_H_ */
