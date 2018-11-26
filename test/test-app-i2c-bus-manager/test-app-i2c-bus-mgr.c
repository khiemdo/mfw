/* Copyright (C) 2015-2017
 *
 * test-app-i2c-bus-mgr.c
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
#include "i2c-bus-manager.h"
#include <stdbool.h>

E_I2C_BUS_NODE_T g_current_access;

bool loc_test_init() {
	i2c_bus_mgr_init();
	return true;
}

void loc_test_run(){
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_SHT25);
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_PRESSURE);
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_LED_DRV);
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_ADC1);
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_ADC2);
	i2c_bus_mgr_request_access(E_I2C_BUS_NODE_ADC3);

	g_current_access = i2c_bus_mgr_current_access();

	i2c_bus_mgr_return_access(g_current_access);

	g_current_access = i2c_bus_mgr_current_access();

}

int main(void) {
	if (!loc_test_init()) return(0);

    while(true) {
    	loc_test_run();
    }
	return 0;
}
