/* Copyright (C) 2015-2017
 *
 * i2c-bus-manager.c
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

#include <string.h>
#include "i2c-bus-manager.h"
/* ===========================================================================*/
/*                    Global variables				                          */
/* ===========================================================================*/

s_i2c_bus_mgr_prio_t g_node_priorities;
E_I2C_BUS_NODE_T g_current_access;
s_i2c_bus_mgr_write_queue_t g_write_queue;
s_i2c_bus_mgr_read_queue_t g_read_queue;


/* ===========================================================================*/
/*                    Local function declarations	                          */
/* ===========================================================================*/

void loc_i2c_bus_mgr_assign_access();

/* ===========================================================================*/
/*                    Local function implementations                         */
/* ===========================================================================*/

void loc_i2c_bus_mgr_assign_access(){
	if (g_current_access == E_I2C_BUS_NODE_NONE) {
		uint8_t values[NUMBER_OF_NODES];
		values[0] = g_node_priorities.node_sht25_prio;
    values[1] = g_node_priorities.node_pressure_prio;
		values[2] = g_node_priorities.node_led_drv_prio;
		values[3] = g_node_priorities.node_adc1_prio;
    values[4] = g_node_priorities.node_adc2_prio;
    values[5] = g_node_priorities.node_adc3_prio;

		uint8_t min = 255;
		uint8_t min_ind = NUMBER_OF_NODES;
		uint8_t i;
		for (i = 0; i < NUMBER_OF_NODES; i++) {
			if (values[i] < min) {
				min = values[i];
				min_ind = i;
			}
		}

		switch (min_ind) {
			case 0:
				g_current_access = E_I2C_BUS_NODE_SHT25;
				break;
			case 1:
				g_current_access = E_I2C_BUS_NODE_PRESSURE;
				break;
			case 2:
				g_current_access = E_I2C_BUS_NODE_LED_DRV;
				break;
			case 3:
				g_current_access = E_I2C_BUS_NODE_ADC1;
				break;
			case 4:
				g_current_access = E_I2C_BUS_NODE_ADC2;
				break;
			case 5:
				g_current_access = E_I2C_BUS_NODE_ADC3;
				break;
			case NUMBER_OF_NODES:
				g_current_access = E_I2C_BUS_NODE_NONE;
				break;
		}

	}
}

/* ===========================================================================*/
/*                    Global function implementations                         */
/* ===========================================================================*/

bool i2c_bus_mgr_init() {
  bool b_return;
  b_return = true;
  memset( &g_write_queue, 0, sizeof(g_write_queue) );
  memset( &g_read_queue, 0, sizeof(g_read_queue) );
  g_node_priorities.node_sht25_prio = 255;
  g_node_priorities.node_pressure_prio = 255;
  g_node_priorities.node_led_drv_prio = 255;
  g_node_priorities.node_adc1_prio = 255;
  g_node_priorities.node_adc2_prio = 255;
  g_node_priorities.node_adc3_prio = 255;

  g_current_access = E_I2C_BUS_NODE_NONE;

  return b_return;
}

void i2c_bus_mgr_request_access(E_I2C_BUS_NODE_T node_id) {
	switch(node_id){
		case E_I2C_BUS_NODE_SHT25:
		  g_node_priorities.node_sht25_prio = (NODE_SHT25_PRIORITY < g_node_priorities.node_sht25_prio ? NODE_SHT25_PRIORITY : g_node_priorities.node_sht25_prio);
      break;
		case E_I2C_BUS_NODE_PRESSURE:
      g_node_priorities.node_pressure_prio = (NODE_PRESSURE_PRIORITY < g_node_priorities.node_pressure_prio ? NODE_PRESSURE_PRIORITY : g_node_priorities.node_pressure_prio);
			break;
		case E_I2C_BUS_NODE_LED_DRV:
		  g_node_priorities.node_led_drv_prio = (NODE_LED_DRV_PRIORITY < g_node_priorities.node_led_drv_prio ? NODE_LED_DRV_PRIORITY : g_node_priorities.node_led_drv_prio);
			break;
		case E_I2C_BUS_NODE_ADC1:
		  g_node_priorities.node_adc1_prio = (NODE_ADC1_PRIORITY < g_node_priorities.node_adc1_prio ? NODE_ADC1_PRIORITY : g_node_priorities.node_adc1_prio);
			break;
		case E_I2C_BUS_NODE_ADC2:
		  g_node_priorities.node_adc2_prio = (NODE_ADC2_PRIORITY < g_node_priorities.node_adc2_prio ? NODE_ADC2_PRIORITY : g_node_priorities.node_adc2_prio);
			break;
		case E_I2C_BUS_NODE_ADC3:
		  g_node_priorities.node_adc3_prio = (NODE_ADC3_PRIORITY < g_node_priorities.node_adc3_prio ? NODE_ADC3_PRIORITY : g_node_priorities.node_adc3_prio);
			break;
	}
	loc_i2c_bus_mgr_assign_access();
}


void i2c_bus_mgr_return_access(E_I2C_BUS_NODE_T node_id) {
  switch(node_id){
    case E_I2C_BUS_NODE_SHT25:
      if (g_write_queue.node_sht25_write_ctx) g_write_queue.node_sht25_write_ctx->write_queued = false;
      if (g_read_queue.node_sht25_read_ctx) g_read_queue.node_sht25_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = 255;
      g_node_priorities.node_pressure_prio = (g_node_priorities.node_pressure_prio == 255 ? 255 : g_node_priorities.node_pressure_prio -1);
      g_node_priorities.node_led_drv_prio = (g_node_priorities.node_led_drv_prio == 255 ? 255 : g_node_priorities.node_led_drv_prio -1);
      g_node_priorities.node_adc1_prio = (g_node_priorities.node_adc1_prio == 255 ? 255 : g_node_priorities.node_adc1_prio -1);
      g_node_priorities.node_adc2_prio = (g_node_priorities.node_adc2_prio == 255 ? 255 : g_node_priorities.node_adc2_prio -1);
      g_node_priorities.node_adc3_prio = (g_node_priorities.node_adc3_prio == 255 ? 255 : g_node_priorities.node_adc3_prio -1);
      break;
    case E_I2C_BUS_NODE_PRESSURE:
      if (g_write_queue.node_pressure_write_ctx) g_write_queue.node_pressure_write_ctx->write_queued = false;
      if (g_read_queue.node_pressure_read_ctx) g_read_queue.node_pressure_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = (g_node_priorities.node_sht25_prio == 255 ? 255 : g_node_priorities.node_sht25_prio -1);
      g_node_priorities.node_pressure_prio = 255;
      g_node_priorities.node_led_drv_prio = (g_node_priorities.node_led_drv_prio == 255 ? 255 : g_node_priorities.node_led_drv_prio -1);
      g_node_priorities.node_adc1_prio = (g_node_priorities.node_adc1_prio == 255 ? 255 : g_node_priorities.node_adc1_prio -1);
      g_node_priorities.node_adc2_prio = (g_node_priorities.node_adc2_prio == 255 ? 255 : g_node_priorities.node_adc2_prio -1);
      g_node_priorities.node_adc3_prio = (g_node_priorities.node_adc3_prio == 255 ? 255 : g_node_priorities.node_adc3_prio -1);
      break;
    case E_I2C_BUS_NODE_LED_DRV:
      if (g_write_queue.node_led_drv_write_ctx) g_write_queue.node_led_drv_write_ctx->write_queued = false;
      if (g_read_queue.node_led_drv_read_ctx) g_read_queue.node_led_drv_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = (g_node_priorities.node_sht25_prio == 255 ? 255 : g_node_priorities.node_sht25_prio -1);
      g_node_priorities.node_pressure_prio = (g_node_priorities.node_pressure_prio == 255 ? 255 : g_node_priorities.node_pressure_prio -1);
      g_node_priorities.node_led_drv_prio = 255;
      g_node_priorities.node_adc1_prio = (g_node_priorities.node_adc1_prio == 255 ? 255 : g_node_priorities.node_adc1_prio -1);
      g_node_priorities.node_adc2_prio = (g_node_priorities.node_adc2_prio == 255 ? 255 : g_node_priorities.node_adc2_prio -1);
      g_node_priorities.node_adc3_prio = (g_node_priorities.node_adc3_prio == 255 ? 255 : g_node_priorities.node_adc3_prio -1);
      break;
    case E_I2C_BUS_NODE_ADC1:
      if (g_write_queue.node_adc1_write_ctx) g_write_queue.node_adc1_write_ctx->write_queued = false;
      if (g_read_queue.node_adc1_read_ctx) g_read_queue.node_adc1_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = (g_node_priorities.node_sht25_prio == 255 ? 255 : g_node_priorities.node_sht25_prio -1);
      g_node_priorities.node_pressure_prio = (g_node_priorities.node_pressure_prio == 255 ? 255 : g_node_priorities.node_pressure_prio -1);
      g_node_priorities.node_led_drv_prio = (g_node_priorities.node_led_drv_prio == 255 ? 255 : g_node_priorities.node_led_drv_prio -1);
      g_node_priorities.node_adc1_prio = 255;
      g_node_priorities.node_adc2_prio = (g_node_priorities.node_adc2_prio == 255 ? 255 : g_node_priorities.node_adc2_prio -1);
      g_node_priorities.node_adc3_prio = (g_node_priorities.node_adc3_prio == 255 ? 255 : g_node_priorities.node_adc3_prio -1);
      break;
    case E_I2C_BUS_NODE_ADC2:
      if (g_write_queue.node_adc2_write_ctx) g_write_queue.node_adc2_write_ctx->write_queued = false;
      if (g_read_queue.node_adc2_read_ctx) g_read_queue.node_adc2_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = (g_node_priorities.node_sht25_prio == 255 ? 255 : g_node_priorities.node_sht25_prio -1);
      g_node_priorities.node_pressure_prio = (g_node_priorities.node_pressure_prio == 255 ? 255 : g_node_priorities.node_pressure_prio -1);
      g_node_priorities.node_led_drv_prio = (g_node_priorities.node_led_drv_prio == 255 ? 255 : g_node_priorities.node_led_drv_prio -1);
      g_node_priorities.node_adc1_prio = (g_node_priorities.node_adc1_prio == 255 ? 255 : g_node_priorities.node_adc1_prio -1);
      g_node_priorities.node_adc2_prio = 255;
      g_node_priorities.node_adc3_prio = (g_node_priorities.node_adc3_prio == 255 ? 255 : g_node_priorities.node_adc3_prio -1);
      break;
    case E_I2C_BUS_NODE_ADC3:
      if (g_write_queue.node_adc3_write_ctx) g_write_queue.node_adc3_write_ctx->write_queued = false;
      if (g_read_queue.node_adc3_read_ctx) g_read_queue.node_adc3_read_ctx->read_queued = false;
      g_node_priorities.node_sht25_prio = (g_node_priorities.node_sht25_prio == 255 ? 255 : g_node_priorities.node_sht25_prio -1);
      g_node_priorities.node_pressure_prio = (g_node_priorities.node_pressure_prio == 255 ? 255 : g_node_priorities.node_pressure_prio -1);
      g_node_priorities.node_led_drv_prio = (g_node_priorities.node_led_drv_prio == 255 ? 255 : g_node_priorities.node_led_drv_prio -1);
      g_node_priorities.node_adc1_prio = (g_node_priorities.node_adc1_prio == 255 ? 255 : g_node_priorities.node_adc1_prio -1);
      g_node_priorities.node_adc2_prio = (g_node_priorities.node_adc2_prio == 255 ? 255 : g_node_priorities.node_adc2_prio -1);
      g_node_priorities.node_adc3_prio = 255;
      break;
  }
  g_current_access = E_I2C_BUS_NODE_NONE;
  loc_i2c_bus_mgr_assign_access();
}

void i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_T node_id ,s_i2c_bus_mgr_write_t * write_ctx) {
  switch (node_id) {
    case E_I2C_BUS_NODE_SHT25:
      g_write_queue.node_sht25_write_ctx = write_ctx;
      g_write_queue.node_sht25_write_ctx->write_queued = true;
      //g_write_queue.node_sht25_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_sht25_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_sht25_write_ctx.data = write_ctx->data;
      //g_write_queue.node_sht25_write_ctx.length = write_ctx->length;
      //g_write_queue.node_sht25_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_PRESSURE:
      g_write_queue.node_pressure_write_ctx = write_ctx;
      g_write_queue.node_pressure_write_ctx->write_queued = true;
      //g_write_queue.node_pressure_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_pressure_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_pressure_write_ctx.data = write_ctx->data;
      //g_write_queue.node_pressure_write_ctx.length = write_ctx->length;
      //g_write_queue.node_pressure_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_LED_DRV:
      g_write_queue.node_led_drv_write_ctx = write_ctx;
      g_write_queue.node_led_drv_write_ctx->write_queued = true;
      //g_write_queue.node_led_drv_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_led_drv_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_led_drv_write_ctx.data = write_ctx->data;
      //g_write_queue.node_led_drv_write_ctx.length = write_ctx->length;
      //g_write_queue.node_led_drv_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC1:
      g_write_queue.node_adc1_write_ctx = write_ctx;
      g_write_queue.node_adc1_write_ctx->write_queued = true;
      //g_write_queue.node_adc1_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_adc1_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_adc1_write_ctx.data = write_ctx->data;
      //g_write_queue.node_adc1_write_ctx.length = write_ctx->length;
      //g_write_queue.node_adc1_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC2:
      g_write_queue.node_adc2_write_ctx = write_ctx;
      g_write_queue.node_adc2_write_ctx->write_queued = true;
      //g_write_queue.node_adc2_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_adc2_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_adc2_write_ctx.data = write_ctx->data;
      //g_write_queue.node_adc2_write_ctx.length = write_ctx->length;
      //g_write_queue.node_adc2_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC3:
      g_write_queue.node_adc3_write_ctx = write_ctx;
      g_write_queue.node_adc3_write_ctx->write_queued = true;
      //g_write_queue.node_adc3_write_ctx.ps_ctx = write_ctx->ps_ctx;
      //g_write_queue.node_adc3_write_ctx.slave_address = write_ctx->slave_address;
      //g_write_queue.node_adc3_write_ctx.data = write_ctx->data;
      //g_write_queue.node_adc3_write_ctx.length = write_ctx->length;
      //g_write_queue.node_adc3_write_ctx.callback_pointer = write_ctx->callback_pointer;
      break;
  }
  i2c_bus_mgr_request_access(node_id);
}

void i2c_bus_mgr_queue_read(E_I2C_BUS_NODE_T node_id ,s_i2c_bus_mgr_read_t * read_ctx) {
  switch (node_id) {
    case E_I2C_BUS_NODE_SHT25:
      g_read_queue.node_sht25_read_ctx = read_ctx;
      g_read_queue.node_sht25_read_ctx->read_queued = true;
      //g_read_queue.node_sht25_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_sht25_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_sht25_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_sht25_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_sht25_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_sht25_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_sht25_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_PRESSURE:
      g_read_queue.node_pressure_read_ctx = read_ctx;
      g_read_queue.node_pressure_read_ctx->read_queued = true;
      //g_read_queue.node_pressure_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_pressure_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_pressure_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_pressure_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_pressure_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_pressure_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_pressure_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_LED_DRV:
      g_read_queue.node_led_drv_read_ctx = read_ctx;
      g_read_queue.node_led_drv_read_ctx->read_queued = true;
      //g_read_queue.node_led_drv_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_led_drv_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_led_drv_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_led_drv_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_led_drv_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_led_drv_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_led_drv_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC1:
      g_read_queue.node_adc1_read_ctx = read_ctx;
      g_read_queue.node_adc1_read_ctx->read_queued = true;
      //g_read_queue.node_adc1_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_adc1_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_adc1_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_adc1_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_adc1_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_adc1_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_adc1_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC2:
      g_read_queue.node_adc2_read_ctx = read_ctx;
      g_read_queue.node_adc2_read_ctx->read_queued = true;
      //g_read_queue.node_adc2_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_adc2_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_adc2_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_adc2_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_adc2_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_adc2_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_adc2_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
    case E_I2C_BUS_NODE_ADC3:
      g_read_queue.node_adc3_read_ctx = read_ctx;
      g_read_queue.node_adc3_read_ctx->read_queued = true;
      //g_read_queue.node_adc3_read_ctx->ps_ctx = read_ctx->ps_ctx;
      //g_read_queue.node_adc3_read_ctx->slave_address = read_ctx->slave_address;
      //g_read_queue.node_adc3_read_ctx->write_data = read_ctx->write_data;
      //g_read_queue.node_adc3_read_ctx->write_length = read_ctx->write_length;
      //g_read_queue.node_adc3_read_ctx->read_data = read_ctx->read_data;
      //g_read_queue.node_adc3_read_ctx->read_length = read_ctx->read_length;
      //g_read_queue.node_adc3_read_ctx->callback_pointer = read_ctx->callback_pointer;
      break;
  }
  i2c_bus_mgr_request_access(node_id);
}

void i2c_bus_mgr_run() {
  switch(g_current_access){
    case E_I2C_BUS_NODE_SHT25:
      // write queued
      if ((g_write_queue.node_sht25_write_ctx->write_queued) && ( g_write_queue.node_sht25_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_write(g_write_queue.node_sht25_write_ctx->ps_ctx, g_write_queue.node_sht25_write_ctx->slave_address, g_write_queue.node_sht25_write_ctx->write_data, g_write_queue.node_sht25_write_ctx->write_length, g_write_queue.node_sht25_write_ctx->callback_pointer);
      // read queued
      } else if ((g_read_queue.node_sht25_read_ctx->read_queued) && ( g_read_queue.node_sht25_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_sht25_read_ctx->ps_ctx, g_read_queue.node_sht25_read_ctx->slave_address, g_read_queue.node_sht25_read_ctx->write_data, g_read_queue.node_sht25_read_ctx->write_length, g_read_queue.node_sht25_read_ctx->read_data, g_read_queue.node_sht25_read_ctx->read_length,g_read_queue.node_sht25_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_PRESSURE:
      if ((g_write_queue.node_pressure_write_ctx->write_queued) && ( g_write_queue.node_pressure_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_write(g_write_queue.node_pressure_write_ctx->ps_ctx, g_write_queue.node_pressure_write_ctx->slave_address, g_write_queue.node_pressure_write_ctx->write_data, g_write_queue.node_pressure_write_ctx->write_length, g_write_queue.node_pressure_write_ctx->callback_pointer);
      } else if ((g_read_queue.node_pressure_read_ctx->read_queued) && ( g_read_queue.node_pressure_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_pressure_read_ctx->ps_ctx, g_read_queue.node_pressure_read_ctx->slave_address, g_read_queue.node_pressure_read_ctx->write_data, g_read_queue.node_pressure_read_ctx->write_length, g_read_queue.node_pressure_read_ctx->read_data, g_read_queue.node_pressure_read_ctx->read_length, g_read_queue.node_pressure_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_LED_DRV:
      if ((g_write_queue.node_led_drv_write_ctx->write_queued) && ( g_write_queue.node_led_drv_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_write(g_write_queue.node_led_drv_write_ctx->ps_ctx, g_write_queue.node_led_drv_write_ctx->slave_address, g_write_queue.node_led_drv_write_ctx->write_data, g_write_queue.node_led_drv_write_ctx->write_length, g_write_queue.node_led_drv_write_ctx->callback_pointer);
      } else if ((g_read_queue.node_led_drv_read_ctx->read_queued) && ( g_read_queue.node_led_drv_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_led_drv_read_ctx->ps_ctx, g_read_queue.node_led_drv_read_ctx->slave_address, g_read_queue.node_led_drv_read_ctx->write_data, g_read_queue.node_led_drv_read_ctx->write_length, g_read_queue.node_led_drv_read_ctx->read_data, g_read_queue.node_led_drv_read_ctx->read_length, g_read_queue.node_led_drv_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_ADC1:
      if ((g_write_queue.node_adc1_write_ctx->write_queued) && ( g_write_queue.node_adc1_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)){
        hal_i2c_write(g_write_queue.node_adc1_write_ctx->ps_ctx, g_write_queue.node_adc1_write_ctx->slave_address, g_write_queue.node_adc1_write_ctx->write_data, g_write_queue.node_adc1_write_ctx->write_length, g_write_queue.node_adc1_write_ctx->callback_pointer);
      } else if ((g_read_queue.node_adc1_read_ctx->read_queued) && ( g_read_queue.node_adc1_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_adc1_read_ctx->ps_ctx, g_read_queue.node_adc1_read_ctx->slave_address, g_read_queue.node_adc1_read_ctx->write_data, g_read_queue.node_adc1_read_ctx->write_length, g_read_queue.node_adc1_read_ctx->read_data, g_read_queue.node_adc1_read_ctx->read_length, g_read_queue.node_adc1_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_ADC2:
      if ((g_write_queue.node_adc2_write_ctx->write_queued) && ( g_write_queue.node_adc2_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)){
        hal_i2c_write(g_write_queue.node_adc2_write_ctx->ps_ctx, g_write_queue.node_adc2_write_ctx->slave_address, g_write_queue.node_adc2_write_ctx->write_data, g_write_queue.node_adc2_write_ctx->write_length, g_write_queue.node_adc2_write_ctx->callback_pointer);
      } else if ((g_read_queue.node_adc2_read_ctx->read_queued) && ( g_read_queue.node_adc2_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_adc2_read_ctx->ps_ctx, g_read_queue.node_adc2_read_ctx->slave_address, g_read_queue.node_adc2_read_ctx->write_data, g_read_queue.node_adc2_read_ctx->write_length, g_read_queue.node_adc2_read_ctx->read_data, g_read_queue.node_adc2_read_ctx->read_length, g_read_queue.node_adc2_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_ADC3:
      if ((g_write_queue.node_adc3_write_ctx->write_queued) && ( g_write_queue.node_adc3_write_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)){
        hal_i2c_write(g_write_queue.node_adc3_write_ctx->ps_ctx, g_write_queue.node_adc3_write_ctx->slave_address, g_write_queue.node_adc3_write_ctx->write_data, g_write_queue.node_adc3_write_ctx->write_length, g_write_queue.node_adc3_write_ctx->callback_pointer);
      } else if ((g_read_queue.node_adc3_read_ctx->read_queued) && ( g_read_queue.node_adc3_read_ctx->ps_ctx->state == E_HAL_I2C_STATE_IDLE)) {
        hal_i2c_read(g_read_queue.node_adc3_read_ctx->ps_ctx, g_read_queue.node_adc3_read_ctx->slave_address, g_read_queue.node_adc3_read_ctx->write_data, g_read_queue.node_adc3_read_ctx->write_length, g_read_queue.node_adc3_read_ctx->read_data, g_read_queue.node_adc3_read_ctx->read_length, g_read_queue.node_adc3_read_ctx->callback_pointer);
      }
      break;
    case E_I2C_BUS_NODE_NONE:
      loc_i2c_bus_mgr_assign_access();
      break;
  }
}

E_I2C_BUS_NODE_T i2c_bus_mgr_current_access() {
	return g_current_access;
}


