/* Copyright (C) 2015-2017
 *
 * ads1115.h
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

#ifndef ADS1115_ADS1115_H_
#define ADS1115_ADS1115_H_

//*****************************************************************************
//
//! \addtogroup sensors Sensors
//! @{
//!
//! \addtogroup sensors_ads1115 ADS1115
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include "i2c-bus-manager.h"
#include "Adc.pb.h"


#define ADS1115_SLAVE_ADDR_1 0x48
#define ADS1115_SLAVE_ADDR_2 0x49
#define ADS1115_SLAVE_ADDR_3 0x4A

typedef enum
{
  E_ADS1115_ID_1,
  E_ADS1115_ID_2,
  E_ADS1115_ID_3,
}E_ADS1115_ID_T;

/*!
 * \brief state-space of a ADS-instance.
 */
typedef enum
{
  E_ADS1115_STATE_IDLE,
  E_ADS1115_STATE_PENDING,
  E_ADS1115_STATE_MEASURING,
  E_ADS1115_STATE_READY,
  E_ADS1115_STATE_TIMEOUT,
}E_ADS1115_STATE_T;

/*!
 * \brief Context of a ADS-instance.
 */
typedef struct
{
  E_I2C_BUS_NODE_T ads_id;	/*!< Identifier of ADS-chip on FC for i2c-busmanager*/
  E_ADS1115_STATE_T ads_state; /*!< Current state of instance */
  uint8_t ads_channel; 	/*!< current measuring channel */
  uint8_t ads_slave_address;	/*!< I2C address */
  uint8_t ads_tx_data[3];	/*!< write data (fetched by i2c-busmanager) */
  uint8_t ads_rx_data[3];	/*!< read data (filled by i2c-busmanager) */
  bool ads_new_data_av;		/*!< flag indicating whether data has changed */
  uint32_t measure_timeout; /*!< measure timer reference */
  uint32_t timeout_timer;	/*!< timeout timer reference */
}s_ads1115_ctx_t;

/*!
 * \brief Initialize the ADS-sensor.
 *
 * \param ads_id Defines the identifier of the ADS on the flight-controller to be intialized.
 * \return Returns pointer to context variable if ADS-instance is initialized
 *         successfully. NULL otherwise.
 */
s_ads1115_ctx_t *ads1115_init(E_ADS1115_ID_T ads_id);

/*!
 * \brief Handle the ADS-sensor state.
 *
 * \param ads_ctx Defines the context to the ADS-instance that should be handled.
 *
 * \return None.
 */
void ads1115_run(s_ads1115_ctx_t * ads_ctx);

/*!
 * \brief Gets the current ADS-values and writes them into the provided Adc-msg.
 *
 * \param p_adc_v_msg Pointer to adc-msg struct.
 *
 * \return TRUE if new sensor data is written into the message. FALSE otherwise.
 */
bool ads1115_get(Adc* const p_adc_v_msg);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* ADS1115_ADS1115_H_ */
