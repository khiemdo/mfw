/* Copyright (C) 2015-2017
 *
 * i2c-bus-manager.h
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
#ifndef UTILS_I2C_BUS_MANAGER_H_
#define UTILS_I2C_BUS_MANAGER_H_

//*****************************************************************************
//
//! \addtogroup utils
//! @{
//!
//! \addtogroup utils_i2c_bus_manager I2C Bus Manager
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include "hal_i2c_ti_drv.h"

#define NUMBER_OF_NODES 6
#define NODE_SHT25_PRIORITY 10
#define NODE_PRESSURE_PRIORITY 30
#define NODE_LED_DRV_PRIORITY 10
#define NODE_ADC1_PRIORITY 10
#define NODE_ADC2_PRIORITY 10
#define NODE_ADC3_PRIORITY 10

//! enum defining node names
typedef enum
{
	E_I2C_BUS_NODE_SHT25,
	E_I2C_BUS_NODE_PRESSURE,
	E_I2C_BUS_NODE_LED_DRV,
	E_I2C_BUS_NODE_ADC1,
	E_I2C_BUS_NODE_ADC2,
	E_I2C_BUS_NODE_ADC3,
	E_I2C_BUS_NODE_NONE
}E_I2C_BUS_NODE_T;

//! struct containing priorities of nodes
typedef struct
{
	uint8_t node_sht25_prio;
	uint8_t node_pressure_prio;
	uint8_t node_led_drv_prio;
	uint8_t node_adc1_prio;
	uint8_t node_adc2_prio;
	uint8_t node_adc3_prio;
}s_i2c_bus_mgr_prio_t;

//! struct defining write task
typedef struct
{
  bool write_queued;	//!< Flag indicating whether task is active
  s_hal_i2c_ctx_t *ps_ctx; //!< Context to used i2c port
  uint8_t slave_address; //!< slave addr for write task
  uint8_t *write_data;	//!< write data for write task
  uint8_t write_length; //!< write length for write task
  void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status); //!< callback for node
}s_i2c_bus_mgr_write_t;

//! struct defining read task
typedef struct
{
  bool read_queued;		//!< Flag indicating whether task is active
  s_hal_i2c_ctx_t *ps_ctx;	//!< Context to used i2c port
  uint8_t slave_address;	//!< slave addr for read task
  uint8_t *write_data;	//!< write data for write task
  uint8_t write_length; //!< write length for write task
  uint8_t *read_data;	//!< read data for write task
  uint8_t read_length;	//!< read length for write task
  void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status); //!< callback for node
}s_i2c_bus_mgr_read_t;

//! struct defining write queue (contains entry for every node)
typedef struct
{
  s_i2c_bus_mgr_write_t *node_sht25_write_ctx;
  s_i2c_bus_mgr_write_t *node_pressure_write_ctx;
  s_i2c_bus_mgr_write_t *node_led_drv_write_ctx;
  s_i2c_bus_mgr_write_t *node_adc1_write_ctx;
  s_i2c_bus_mgr_write_t *node_adc2_write_ctx;
  s_i2c_bus_mgr_write_t *node_adc3_write_ctx;
}s_i2c_bus_mgr_write_queue_t;

//! struct defining read queue (contains entry for every node)
typedef struct
{
  s_i2c_bus_mgr_read_t *node_sht25_read_ctx;
  s_i2c_bus_mgr_read_t *node_pressure_read_ctx;
  s_i2c_bus_mgr_read_t *node_led_drv_read_ctx;
  s_i2c_bus_mgr_read_t *node_adc1_read_ctx;
  s_i2c_bus_mgr_read_t *node_adc2_read_ctx;
  s_i2c_bus_mgr_read_t *node_adc3_read_ctx;
}s_i2c_bus_mgr_read_queue_t;




/*!
 * \brief Initialize the i2c_bus_manager. With initial priorities set to lowest value.
 *
 * \return Returns true if init was succesful.
 *
 */
bool i2c_bus_mgr_init(void);

/*!
 * \brief Request access from busmanager
 *
 * \param node_id Defines the node that requests access.
 *
 *
 */
void i2c_bus_mgr_request_access(E_I2C_BUS_NODE_T node_id);

/*!
 * \brief Returns access to busmanager
 *
 * \param node_id Defines the node that returns access.
 *
 *
 */
void i2c_bus_mgr_return_access(E_I2C_BUS_NODE_T node_id);

/*!
 * \brief Appends a write task to queue
 *
 * \param node_id Defines the node that appends task.
 *
 * \param write_ctx pointer to write-task.
 *
 *
 */
void i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_T node_id ,s_i2c_bus_mgr_write_t * write_ctx);

/*!
 * \brief Appends a read-task to queue
 *
 * \param node_id Defines the node that appends task.
 *
 * \param write_ctx pointer to read-task.
 *
 *
 */
void i2c_bus_mgr_queue_read(E_I2C_BUS_NODE_T node_id ,s_i2c_bus_mgr_read_t * read_ctx);

/*!
 * \brief Handles the State (Read/write tasks) of the manager
 *
 */
void i2c_bus_mgr_run();

/*!
 * \brief Get the node id that currently has access to bus
 *
 * \return Node-ID that currently has access to bus
 */
E_I2C_BUS_NODE_T i2c_bus_mgr_current_access();

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* UTILS_I2C_BUS_MANAGER_H_ */
