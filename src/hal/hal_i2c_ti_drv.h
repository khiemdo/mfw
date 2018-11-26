/* Copyright (C) 2015-2017
 *
 * hal_i2c_ti_drv.h
 *
 * Elias Rosch         <eliasrosch@gmail.com>
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

#ifndef HAL_HAL_I2C_H_
#define HAL_HAL_I2C_H_

//*****************************************************************************
//
//! \example test-app-i2c.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_i2c_api HAL I2C API
//! @{
//
//*****************************************************************************

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"

/*! Available I2C ports provided and handled by the I2C module. */
typedef enum
{
  /*! I2C0: SCL = PB2, SDA = PB3 */
  E_HAL_I2C_PORT_0,

  /*! I2C1: SCL = PG0, SDA = PG1 */
  E_HAL_I2C_PORT_1,

  /*! I2C2: SCL = PN5, SDA = PN4 */
  E_HAL_I2C_PORT_2,
  
  /*! I2C3: SCL = PK4, SDA = PK5 */
  E_HAL_I2C_PORT_3,

  /*! I2C4: SCL = PK6, SDA = PK7 */
  E_HAL_I2C_PORT_4,

  /*! I2C6: SCL = PA6, SDA = PA7 */
  E_HAL_I2C_PORT_5,

} E_HAL_I2C_PORT_t;

/*! Available I2C states provided and handled by the I2C module. */
typedef enum
{
  /*! Nothing to do */
  E_HAL_I2C_STATE_IDLE,
  /*! Transmission pending */
  E_HAL_I2C_STATE_PENDING,
  /*! Transmission timed out */
  E_HAL_I2C_STATE_TIMEOUT,
} E_HAL_I2C_STATE_t;

/*! Structure defining the context of an I2C port. */
struct S_HAL_I2C_CTX_T
{
	tI2CMInstance *gs_i2c_inst;
	E_HAL_I2C_STATE_t state;
	void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status);
};


/*! Typedef of structure defining the context of an I2C port. */
typedef struct S_HAL_I2C_CTX_T s_hal_i2c_ctx_t;

/*!
 * \brief Initialize the I2C.
 *
 * \param e_port          Defines the I2C port to be intialized.
 * \return Returns pointer to context variable if I2C is initialized
 *         successfully. NULL otherwise.
 */
s_hal_i2c_ctx_t *hal_i2c_ti_drv_init(E_HAL_I2C_PORT_t e_port);

/*!
 * \brief Reads data from the I2C.
 *
 * \param ps_ctx   Pointer to context variable.
 * \param slave_address to node where data will be received from.
 * \param data  Pointer to destination memory where data will be copied to.
 * \param length  Length of the given destination memory.
 *
 * \return None.
 */
void hal_i2c_read(s_hal_i2c_ctx_t *ps_ctx,
                  uint8_t slave_address,
                  uint8_t *writeData,
                  uint8_t writeLength,
                  uint8_t *readData,
                  uint8_t readLength,
                  void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status)
                  );


/*!
 * \brief Writes data to the I2C.
 *
 * \param ps_ctx   Pointer to context variable.
 * \param slave_address to node where data will be written to.
 * \param data Pointer to the data to write to the I2C.
 * \param length Length of the given data.
 *
 * \return None.
 */
void hal_i2c_write(s_hal_i2c_ctx_t *ps_ctx,
                   uint8_t slave_address,
                   uint8_t *data,
                   uint8_t length,
                   void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status));


//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_I2C_H_ */
