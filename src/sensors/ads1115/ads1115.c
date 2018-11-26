/* Copyright (C) 2015-2017
 *
 * ads1115.c
 *
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

#include "ads1115.h"
#include "hal_pps.h"
#include "hal_timer.h"

#define MEASURE_TIMEOUT 2
#define TIMEOUT_DELAY 20


/* ===========================================================================*/
/*                         enums                                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*                         global variables                                   */
/* ===========================================================================*/
s_hal_i2c_ctx_t *ads1115_i2c_ctx;
s_ads1115_ctx_t gs_ads1115_id1;
s_ads1115_ctx_t gs_ads1115_id2;
s_ads1115_ctx_t gs_ads1115_id3;
s_i2c_bus_mgr_write_t write_ctx1;
s_i2c_bus_mgr_write_t write_ctx2;
s_i2c_bus_mgr_write_t write_ctx3;
s_i2c_bus_mgr_read_t read_ctx1;
s_i2c_bus_mgr_read_t read_ctx2;
s_i2c_bus_mgr_read_t read_ctx3;
Adc ads1115_msg;
Adc ads1115_def_msg = AdcCurrent_init_default;

/* ===========================================================================*/
/*                         local function prototypes                          */
/* ===========================================================================*/

/* ===========================================================================*/
/*                         local function implementations                     */
/* ===========================================================================*/
//! Callback after measurement request
void loc_ads1115_id1_callback_to_ready(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id1.ads_state = E_ADS1115_STATE_MEASURING;
  gs_ads1115_id1.measure_timeout = hal_timer_getTimeout(MEASURE_TIMEOUT);
  //i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC1);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
//! Callback after measurement request
void loc_ads1115_id2_callback_to_ready(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id2.ads_state = E_ADS1115_STATE_MEASURING;
  gs_ads1115_id2.measure_timeout = hal_timer_getTimeout(MEASURE_TIMEOUT);
  //i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC2);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
//! Callback after measurement request
void loc_ads1115_id3_callback_to_ready(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id3.ads_state = E_ADS1115_STATE_MEASURING;
  gs_ads1115_id3.measure_timeout = hal_timer_getTimeout(MEASURE_TIMEOUT);
  //i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC3);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
//! Callback after fetching measurements (copy fetched data into message struct)
void loc_ads1115_id1_callback_to_idle(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id1.ads_state = E_ADS1115_STATE_IDLE;

  switch (gs_ads1115_id1.ads_channel) {
	  case 0:
		  ads1115_msg.current_12V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.current_12V_rail.generated));
		  ads1115_msg.current_12V_rail.has_current_raw = true;
		  ads1115_msg.current_12V_rail.current_raw = (gs_ads1115_id1.ads_rx_data[0] << 8)| gs_ads1115_id1.ads_rx_data[1];
		  ads1115_msg.has_current_12V_rail = true;
		  break;
	  case 1:
		  ads1115_msg.current_7V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.current_7V_rail.generated));
		  ads1115_msg.current_7V_rail.has_current_raw = true;
		  ads1115_msg.current_7V_rail.current_raw = (gs_ads1115_id1.ads_rx_data[0] << 8)| gs_ads1115_id1.ads_rx_data[1];
		  ads1115_msg.has_current_7V_rail = true;
		  break;
	  case 2:
		  ads1115_msg.current_5V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.current_5V_rail.generated));
		  ads1115_msg.current_5V_rail.has_current_raw = true;
		  ads1115_msg.current_5V_rail.current_raw = (gs_ads1115_id1.ads_rx_data[0] << 8)| gs_ads1115_id1.ads_rx_data[1];
		  ads1115_msg.has_current_5V_rail = true;
		  break;
	  case 3:
		  ads1115_msg.voltage_ref_1.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_ref_1.generated));
		  ads1115_msg.voltage_ref_1.has_voltage_raw = true;
		  ads1115_msg.voltage_ref_1.voltage_raw = (gs_ads1115_id1.ads_rx_data[0] << 8)| gs_ads1115_id1.ads_rx_data[1];
		  ads1115_msg.has_voltage_ref_1 = true;
		  break;
  }


  gs_ads1115_id1.ads_channel = (gs_ads1115_id1.ads_channel + 1) % 4;
  gs_ads1115_id1.ads_new_data_av = true;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC1);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
void loc_ads1115_id2_callback_to_idle(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id2.ads_state = E_ADS1115_STATE_IDLE;

  switch (gs_ads1115_id2.ads_channel) {
	  case 0:
		  ads1115_msg.current_3_3V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.current_3_3V_rail.generated));
		  ads1115_msg.current_3_3V_rail.has_current_raw = true;
		  ads1115_msg.current_3_3V_rail.current_raw = (gs_ads1115_id2.ads_rx_data[0] << 8)| gs_ads1115_id2.ads_rx_data[1];
		  ads1115_msg.has_current_3_3V_rail = true;
		  break;
	  case 1:
		  ads1115_msg.voltage_12V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_12V_rail.generated));
		  ads1115_msg.voltage_12V_rail.has_voltage_raw = true;
		  ads1115_msg.voltage_12V_rail.voltage_raw = (gs_ads1115_id2.ads_rx_data[0] << 8)| gs_ads1115_id2.ads_rx_data[1];
		  ads1115_msg.has_voltage_12V_rail = true;
		  break;
	  case 2:
		  ads1115_msg.voltage_7V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_7V_rail.generated));
		  ads1115_msg.voltage_7V_rail.has_voltage_raw = true;
		  ads1115_msg.voltage_7V_rail.voltage_raw = (gs_ads1115_id2.ads_rx_data[0] << 8)| gs_ads1115_id2.ads_rx_data[1];
		  ads1115_msg.has_voltage_7V_rail = true;
		  break;
	  case 3:
		  ads1115_msg.voltage_ref_2.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_ref_2.generated));
		  ads1115_msg.voltage_ref_2.has_voltage_raw = true;
		  ads1115_msg.voltage_ref_2.voltage_raw = (gs_ads1115_id2.ads_rx_data[0] << 8)| gs_ads1115_id2.ads_rx_data[1];
		  ads1115_msg.has_voltage_ref_2 = true;
		  break;
  }

  gs_ads1115_id2.ads_channel = (gs_ads1115_id2.ads_channel + 1) % 4;
  gs_ads1115_id2.ads_new_data_av = true;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC2);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
void loc_ads1115_id3_callback_to_idle(void *pvData, uint_fast8_t ui8Status)
{
  gs_ads1115_id3.ads_state = E_ADS1115_STATE_IDLE;

  switch (gs_ads1115_id3.ads_channel) {
	  case 0:
		  ads1115_msg.voltage_5V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_5V_rail.generated));
		  ads1115_msg.voltage_5V_rail.has_voltage_raw = true;
		  ads1115_msg.voltage_5V_rail.voltage_raw = (gs_ads1115_id3.ads_rx_data[0] << 8)| gs_ads1115_id3.ads_rx_data[1];
		  ads1115_msg.has_voltage_5V_rail = true;
		  break;
	  case 1:
		  ads1115_msg.voltage_3_3V_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_3_3V_rail.generated));
		  ads1115_msg.voltage_3_3V_rail.has_voltage_raw = true;
		  ads1115_msg.voltage_3_3V_rail.voltage_raw = (gs_ads1115_id3.ads_rx_data[0] << 8)| gs_ads1115_id3.ads_rx_data[1];
		  ads1115_msg.has_voltage_3_3V_rail = true;
		  break;
	  case 2:
		  ads1115_msg.voltage_3_3V_uC_rail.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_3_3V_uC_rail.generated));
		  ads1115_msg.voltage_3_3V_uC_rail.has_voltage_raw = true;
		  ads1115_msg.voltage_3_3V_uC_rail.voltage_raw = (gs_ads1115_id3.ads_rx_data[0] << 8)| gs_ads1115_id3.ads_rx_data[1];
		  ads1115_msg.has_voltage_3_3V_uC_rail = true;
		  break;
	  case 3:
		  ads1115_msg.voltage_ref_3.has_generated = hal_pps_getTimestamp(&(ads1115_msg.voltage_ref_3.generated));
		  ads1115_msg.voltage_ref_3.has_voltage_raw = true;
		  ads1115_msg.voltage_ref_3.voltage_raw = (gs_ads1115_id3.ads_rx_data[0] << 8)| gs_ads1115_id3.ads_rx_data[1];
		  ads1115_msg.has_voltage_ref_3 = true;
		  break;
  }

  gs_ads1115_id3.ads_channel = (gs_ads1115_id3.ads_channel + 1) % 4;
  gs_ads1115_id3.ads_new_data_av = true;
  i2c_bus_mgr_return_access(E_I2C_BUS_NODE_ADC3);
  ads1115_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
}
/* ===========================================================================*/
/*                         global function implementations                    */
/* ===========================================================================*/
s_ads1115_ctx_t *ads1115_init(E_ADS1115_ID_T ads_id) {
  ads1115_i2c_ctx = hal_i2c_ti_drv_init(E_HAL_I2C_PORT_5);
  memset(&write_ctx1, 0, sizeof(write_ctx1));
  memset(&write_ctx2, 0, sizeof(write_ctx2));
  memset(&write_ctx3, 0, sizeof(write_ctx3));
  write_ctx1.ps_ctx = ads1115_i2c_ctx;
  write_ctx2.ps_ctx = ads1115_i2c_ctx;
  write_ctx3.ps_ctx = ads1115_i2c_ctx;
  memset(&read_ctx1, 0, sizeof(read_ctx1));
  memset(&read_ctx2, 0, sizeof(read_ctx2));
  memset(&read_ctx3, 0, sizeof(read_ctx3));
  read_ctx1.ps_ctx = ads1115_i2c_ctx;
  read_ctx2.ps_ctx = ads1115_i2c_ctx;
  read_ctx3.ps_ctx = ads1115_i2c_ctx;
  i2c_bus_mgr_init();
  s_ads1115_ctx_t * ps_return;
  switch (ads_id) {
    case E_ADS1115_ID_1:
      memset(&gs_ads1115_id1, 0, sizeof(gs_ads1115_id1));
      gs_ads1115_id1.ads_id = E_I2C_BUS_NODE_ADC1;
      gs_ads1115_id1.ads_state = E_ADS1115_STATE_IDLE;
      gs_ads1115_id1.ads_channel = 0;
      gs_ads1115_id1.ads_tx_data[0] = 0x00;
      gs_ads1115_id1.ads_tx_data[1] = 0x00;
      gs_ads1115_id1.ads_tx_data[2] = 0x00;
      gs_ads1115_id1.ads_rx_data[0] = 0x00;
      gs_ads1115_id1.ads_rx_data[1] = 0x00;
      gs_ads1115_id1.ads_rx_data[2] = 0x00;
      gs_ads1115_id1.ads_slave_address = ADS1115_SLAVE_ADDR_1;
      gs_ads1115_id1.ads_new_data_av = false;
      ps_return = &gs_ads1115_id1;
      break;
    case E_ADS1115_ID_2:
      memset(&gs_ads1115_id2, 0, sizeof(gs_ads1115_id2));
      gs_ads1115_id2.ads_id = E_I2C_BUS_NODE_ADC2;
      gs_ads1115_id2.ads_state = E_ADS1115_STATE_IDLE;
      gs_ads1115_id2.ads_channel = 0;
      gs_ads1115_id2.ads_tx_data[0] = 0x00;
      gs_ads1115_id2.ads_tx_data[1] = 0x00;
      gs_ads1115_id2.ads_tx_data[2] = 0x00;
      gs_ads1115_id2.ads_rx_data[0] = 0x00;
      gs_ads1115_id2.ads_rx_data[1] = 0x00;
      gs_ads1115_id2.ads_rx_data[2] = 0x00;
      gs_ads1115_id2.ads_slave_address = ADS1115_SLAVE_ADDR_2;
      gs_ads1115_id2.ads_new_data_av = false;
      ps_return = &gs_ads1115_id2;
      break;
    case E_ADS1115_ID_3:
      memset(&gs_ads1115_id3, 0, sizeof(gs_ads1115_id3));
      gs_ads1115_id3.ads_id = E_I2C_BUS_NODE_ADC3;
      gs_ads1115_id3.ads_state = E_ADS1115_STATE_IDLE;
      gs_ads1115_id3.ads_channel = 0;
      gs_ads1115_id3.ads_tx_data[0] = 0x00;
      gs_ads1115_id3.ads_tx_data[1] = 0x00;
      gs_ads1115_id3.ads_tx_data[2] = 0x00;
      gs_ads1115_id3.ads_rx_data[0] = 0x00;
      gs_ads1115_id3.ads_rx_data[1] = 0x00;
      gs_ads1115_id3.ads_rx_data[2] = 0x00;
      gs_ads1115_id3.ads_slave_address = ADS1115_SLAVE_ADDR_3;
      gs_ads1115_id3.ads_new_data_av = false;
      ps_return = &gs_ads1115_id3;
      break;
  }

  return ps_return;
}

void ads1115_run(s_ads1115_ctx_t * ads_ctx) {
  //! Update the busmanager
  i2c_bus_mgr_run();
  E_I2C_BUS_NODE_T i2c_node;
  i2c_node = ads_ctx->ads_id;
  //!Set the write/read-ctx according to the ads-ctx.
  switch(ads_ctx->ads_id){
    case E_I2C_BUS_NODE_ADC1:
    	  write_ctx1.slave_address = ads_ctx->ads_slave_address;
    	  read_ctx1.slave_address = ads_ctx->ads_slave_address;
    	  write_ctx1.write_data = ads_ctx->ads_tx_data;
    	  read_ctx1.write_data = ads_ctx->ads_tx_data;
    	  read_ctx1.read_data = ads_ctx->ads_rx_data;
      break;
    case E_I2C_BUS_NODE_ADC2:
  	  write_ctx2.slave_address = ads_ctx->ads_slave_address;
  	  read_ctx2.slave_address = ads_ctx->ads_slave_address;
  	  write_ctx2.write_data = ads_ctx->ads_tx_data;
  	  read_ctx2.write_data = ads_ctx->ads_tx_data;
  	  read_ctx2.read_data = ads_ctx->ads_rx_data;
      break;
    case E_I2C_BUS_NODE_ADC3:
  	  write_ctx3.slave_address = ads_ctx->ads_slave_address;
  	  read_ctx3.slave_address = ads_ctx->ads_slave_address;
  	  write_ctx3.write_data = ads_ctx->ads_tx_data;
  	  read_ctx3.write_data = ads_ctx->ads_tx_data;
  	  read_ctx3.read_data = ads_ctx->ads_rx_data;
      break;
  }
  //! Perform actions depending on current state of instance
  //! IDLE: Send measurement instruction -> PENDING
  //! PENDING: Wait for callback ->	MEASURING
  //! MEASURING: Wait for measurement timer to run out -> READY
  //! READY: Read measured data -> PENDING
  //! PENDING: Wait for callback -> IDLE
  //! (If timeout while PENDING -> TIMEOUT)
  switch(ads_ctx->ads_state){
      case E_ADS1115_STATE_IDLE:
        ads_ctx->ads_tx_data[0] = 0x01; // Write to config register
        ads_ctx->ads_tx_data[1] = 0xC5 | ((ads_ctx->ads_channel & 0x03) << 4); // Start single conv, channel 0, +-2.048 V, single shot (Select channel here!)
        ads_ctx->ads_tx_data[2] = 0xE3; // 860 SPS, Trad. comp, active low comp, no latch, disable comp.
        switch(ads_ctx->ads_id){
          case E_I2C_BUS_NODE_ADC1:
            write_ctx1.write_length = 3;
            write_ctx1.callback_pointer = &loc_ads1115_id1_callback_to_ready;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_write(i2c_node,&write_ctx1);
            break;
          case E_I2C_BUS_NODE_ADC2:
            write_ctx2.write_length = 3;
            write_ctx2.callback_pointer = &loc_ads1115_id2_callback_to_ready;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_write(i2c_node,&write_ctx2);
            break;
          case E_I2C_BUS_NODE_ADC3:
            write_ctx3.write_length = 3;
            write_ctx3.callback_pointer = &loc_ads1115_id3_callback_to_ready;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_write(i2c_node,&write_ctx3);
            break;
        }

        break;
      case E_ADS1115_STATE_PENDING:
        if (hal_timer_isTimedOut(ads_ctx->timeout_timer)) {
          //ads_ctx->ads_state = E_ADS1115_STATE_TIMEOUT;
        }
        break;
      case E_ADS1115_STATE_MEASURING:
        if (hal_timer_isTimedOut(ads_ctx->measure_timeout)) {
          ads_ctx->ads_state = E_ADS1115_STATE_READY;
        }
        break;
      case E_ADS1115_STATE_READY:
    	ads_ctx->ads_tx_data[0] = 0x00;
        read_ctx1.write_length = 1;
        read_ctx2.write_length = 1;
        read_ctx3.write_length = 1;
        ads_ctx->ads_rx_data[0] = 0x00;
        ads_ctx->ads_rx_data[1] = 0x00;
        ads_ctx->ads_rx_data[2] = 0x00;
        read_ctx1.read_length = 2;
        read_ctx2.read_length = 2;
        read_ctx3.read_length = 2;
        switch(ads_ctx->ads_id){
          case E_I2C_BUS_NODE_ADC1:
            read_ctx1.callback_pointer = &loc_ads1115_id1_callback_to_idle;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_read(i2c_node,&read_ctx1);
            break;
          case E_I2C_BUS_NODE_ADC2:
            read_ctx2.callback_pointer = &loc_ads1115_id2_callback_to_idle;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_read(i2c_node,&read_ctx2);
            break;
          case E_I2C_BUS_NODE_ADC3:
            read_ctx3.callback_pointer = &loc_ads1115_id3_callback_to_idle;
            ads_ctx->ads_state = E_ADS1115_STATE_PENDING;
            gs_ads1115_id2.timeout_timer = hal_timer_getTimeout(TIMEOUT_DELAY);
            i2c_bus_mgr_queue_read(i2c_node,&read_ctx3);
          break;
        }
        break;
       case E_ADS1115_STATE_TIMEOUT:
         break;
  }
}

//! Simply copy content of local measurement msg into provided msg
bool ads1115_get(Adc* const p_adc_v_msg) {
  bool b_return = false;
  if (gs_ads1115_id1.ads_new_data_av | gs_ads1115_id2.ads_new_data_av | gs_ads1115_id3.ads_new_data_av) {
	  gs_ads1115_id1.ads_new_data_av = false;
	  gs_ads1115_id2.ads_new_data_av = false;
	  gs_ads1115_id3.ads_new_data_av = false;
	  (*p_adc_v_msg) = ads1115_msg;
	  ads1115_msg = ads1115_def_msg;
	  b_return = true;
  }
  return b_return;
}
