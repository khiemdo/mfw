/* Copyright (C) 2015-2017
 *
 * mpl.c
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


#include "mpl.h"
#include "hal_pps.h"
#include "hal_timer.h"

#include "i2c-bus-manager.h"

#define MEASURE_TIMEOUT 1
#define TIMEOUT_DELAY 20

#define MPL_REG_CTRL1 0x26
#define MPL_REG_DATA_CFG 0x13

/* ===========================================================================*/
/*                         global variables                                   */
/* ===========================================================================*/
s_hal_i2c_ctx_t *mpl_i2c_ctx;
E_MPL_STATE_T mpl_state;
bool mpl_new_data_av;
uint32_t mpl_timeout_timer;
E_I2C_BUS_NODE_T mpl_i2c_node;
s_i2c_bus_mgr_write_t mpl_write_ctx;
s_i2c_bus_mgr_read_t mpl_read_ctx;
uint8_t mpl_tx_data[2];
uint8_t mpl_rx_data[5];
Barometer barometer_msg;

/* ===========================================================================*/
/*                         local function implementations                     */
/* ===========================================================================*/
//! Callbacks to used in init-procedure
//! (could be modified to use only one callback that handles all state-switchings,
//! but too lazy right now)
void loc_mpl_callback_to_init1(void *pvData, uint_fast8_t ui8Status)
{
  mpl_state = E_MPL_STATE_INIT1;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_PRESSURE);
  mpl_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}

void loc_mpl_callback_to_init2(void *pvData, uint_fast8_t ui8Status)
{
  mpl_state = E_MPL_STATE_INIT2;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_PRESSURE);
  mpl_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}

void loc_mpl_callback_to_init_finish(void *pvData, uint_fast8_t ui8Status)
{
  mpl_state = E_MPL_STATE_IDLE;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_PRESSURE);
  mpl_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}

//! Callback after measurement request
void loc_mpl_callback_to_ready(void *pvData, uint_fast8_t ui8Status)
{
  // Check for DataReady-bit
  if (mpl_rx_data[0] & 0x08) {
    mpl_state = E_MPL_STATE_READY;
  } else {
    mpl_state = E_MPL_STATE_IDLE;
  }
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_PRESSURE);
  mpl_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}

//! Callback after measurement fetch.
//!	Copy fetched data into local msg
void loc_mpl_callback_to_idle(void *pvData, uint_fast8_t ui8Status)
{
  barometer_msg.has_generated = hal_pps_getTimestamp( &(barometer_msg.generated) );
  barometer_msg.pressure_raw = (mpl_rx_data[0] << 16) | (mpl_rx_data[1] << 8) | (mpl_rx_data[2]);
  barometer_msg.has_pressure_raw = true;
  barometer_msg.temperature_raw = (mpl_rx_data[3] << 8) | (mpl_rx_data[4]);
  barometer_msg.has_temperature_raw = true;
  mpl_new_data_av = true;
  mpl_state = E_MPL_STATE_IDLE;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_PRESSURE);
  mpl_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}

/* ===========================================================================*/
/*                         global function implementations                    */
/* ===========================================================================*/
bool MPL_init() {
  bool b_return = false;
  mpl_i2c_ctx = hal_i2c_ti_drv_init(E_HAL_I2C_PORT_5);
  if (mpl_i2c_ctx) {
    memset(&mpl_write_ctx, 0, sizeof(mpl_write_ctx));
    mpl_write_ctx.ps_ctx = mpl_i2c_ctx;
    memset(&mpl_read_ctx, 0, sizeof(mpl_read_ctx));
    mpl_read_ctx.ps_ctx = mpl_i2c_ctx;
    b_return = i2c_bus_mgr_init();
    mpl_state = E_MPL_STATE_INIT1;
    mpl_new_data_av = false;
    mpl_i2c_node = E_I2C_BUS_NODE_PRESSURE;
    mpl_write_ctx.slave_address = MPL_SLAVE_ADDR;
    mpl_read_ctx.slave_address = MPL_SLAVE_ADDR;
    mpl_write_ctx.write_data = mpl_tx_data;
    mpl_read_ctx.write_data = mpl_tx_data;
    mpl_read_ctx.read_data = mpl_rx_data;
  }

  mpl_tx_data[0] = MPL_REG_CTRL1;
  // Set to Barometer with an OSR = 128
  mpl_tx_data[1] = 0x38;
  mpl_write_ctx.write_length = 2;
  mpl_write_ctx.callback_pointer = &loc_mpl_callback_to_init1;
  mpl_state = E_MPL_STATE_PENDING;
  mpl_timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
  i2c_bus_mgr_queue_write(mpl_i2c_node, &mpl_write_ctx);
  i2c_bus_mgr_run();
  return b_return;
}

void MPL_run() {
  i2c_bus_mgr_run();

  switch(mpl_state){
    case E_MPL_STATE_INIT1:
      mpl_tx_data[0] = MPL_REG_DATA_CFG;
      // Raise events on temperature & pressure
      mpl_tx_data[1] = 0x07;
      mpl_write_ctx.write_length = 2;
      mpl_write_ctx.callback_pointer = &loc_mpl_callback_to_init2;
      mpl_state = E_MPL_STATE_PENDING;
      mpl_timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
      i2c_bus_mgr_queue_write(mpl_i2c_node,&mpl_write_ctx);
      break;
    case E_MPL_STATE_INIT2:
      mpl_tx_data[0] = MPL_REG_CTRL1;
      // Set to Barometer with an OSR = 128 (standby off)
      mpl_tx_data[1] = 0x39;
      mpl_write_ctx.write_length = 2;
      mpl_write_ctx.callback_pointer = &loc_mpl_callback_to_init_finish;
      mpl_state = E_MPL_STATE_PENDING;
      mpl_timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
      i2c_bus_mgr_queue_write(mpl_i2c_node,&mpl_write_ctx);
      break;
    case E_MPL_STATE_IDLE:
      mpl_tx_data[0] = 0x00;
      mpl_read_ctx.write_length = 1;
      mpl_rx_data[0] = 0x00;
      mpl_read_ctx.read_length = 1;
      mpl_read_ctx.callback_pointer = &loc_mpl_callback_to_ready;
      mpl_state = E_MPL_STATE_PENDING;
      mpl_timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
      i2c_bus_mgr_queue_read(mpl_i2c_node,&mpl_read_ctx);
      break;

      case E_MPL_STATE_PENDING:
        if (hal_timer_isTimedOut(mpl_timeout_timer)) {
          mpl_state = E_MPL_STATE_TIMEOUT;
        }
        break;
      case E_MPL_STATE_READY:
        mpl_tx_data[0] = 0x01;
        mpl_read_ctx.write_length = 1;
        mpl_rx_data[0] = 0x00;
        mpl_rx_data[1] = 0x00;
        mpl_rx_data[2] = 0x00;
        mpl_rx_data[3] = 0x00;
        mpl_rx_data[4] = 0x00;
        mpl_read_ctx.read_length = 5;
        mpl_read_ctx.callback_pointer = &loc_mpl_callback_to_idle;
        mpl_state = E_MPL_STATE_PENDING;
        mpl_timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
        i2c_bus_mgr_queue_read(mpl_i2c_node,&mpl_read_ctx);
        break;
       case E_MPL_STATE_TIMEOUT:
         break;
  }
}

//! Copy content of local msg into provided msg
bool MPL_get(Barometer* const p_baro_msg) {
  bool b_return = false;
  if (mpl_new_data_av) {
    (*p_baro_msg) = barometer_msg;
    mpl_new_data_av = false;
    b_return = true;
  }
  return b_return;
}
