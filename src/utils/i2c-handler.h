/* Copyright (C) 2015-2017
 *
 * i2c-handler.h
 *
 * Elias Rosch         <eliasrosch@gmail.com>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Martin Dold         <martin.dold@gmx.net>
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
#ifndef UTILS_I2C_HANDLER_H_
#define UTILS_I2C_HANDLER_H_

//*****************************************************************************
//
//! \addtogroup utils
//! @{
//!
//! \addtogroup utils_i2c_handler I2C Handler (Generic Bus Manager)
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include "hal_i2c_ti_drv.h"

#define I2C_HAND_QUEUE_LENGTH 10

typedef struct
{
  uint8_t priority;
  uint8_t slave_address;
  uint8_t *write_data;
  uint8_t write_length;
  void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status);
}s_i2c_hand_wtask_t;

typedef struct
{
  uint8_t priority;
  s_hal_i2c_ctx_t *ps_ctx;
  uint8_t slave_address;
  uint8_t *write_data;
  uint8_t write_length;
  uint8_t *read_data;
  uint8_t read_length;
  void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status);
}s_i2c_hand_rtask_t;

typedef struct
{
	s_hal_i2c_ctx_t * i2c_ctx;
    s_i2c_hand_wtask_t *wqueue[I2C_HAND_QUEUE_LENGTH];
    bool wtask_active[I2C_HAND_QUEUE_LENGTH];
    uint8_t wqueue_rptr;
    s_i2c_hand_rtask_t *rqueue[I2C_HAND_QUEUE_LENGTH];
    bool rtask_active[I2C_HAND_QUEUE_LENGTH];
    uint8_t rqueue_rptr;
}s_i2c_hand_t;

/*!
 * \brief Initialize the i2c_handler: Initializes i2c-port (if not already initialized)
 *
 * \param e_port Defines the I2C port to be initialized.
 *
 * \return Returns context to port-specific handler struct.
 *
 */
s_i2c_hand_t *i2c_handler_init(E_HAL_I2C_PORT_t e_port);

/*!
 * \brief Add a write task to the write-queue.
 *
 * \param e_port Context to handler of specific port.
 * \param Pointer to write task.
 *
 * \return Returns true if task was appended, else otherwise.
 *
 */
bool i2c_handler_write(s_i2c_hand_t * i2c_hand_ctx, s_i2c_hand_wtask_t *write_task);

/*!
 * \brief Add a read task to the read-queue.
 *
 * \param e_port Context to handler of specific port.
 * \param Pointer to read task.
 *
 * \return Returns true if task was appended, else otherwise.
 *
 */
bool i2c_handler_read(s_i2c_hand_t * i2c_hand_ctx, s_i2c_hand_rtask_t *read_task);

/*!
 * \brief Start transmissions for every port if available and port is idle.
 *
 *
 */
void i2c_handler_run();

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* UTILS_I2C_HANDLER_H_ */
