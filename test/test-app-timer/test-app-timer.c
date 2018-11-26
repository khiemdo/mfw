/* Copyright (C) 2015-2017
 *
 * test-app-timer.c
 *
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
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "hal_mcu.h"

uint32_t current_val = 0;

int main(void) {
	
	hal_mcu_init();
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC_UP);   // 32 bits Timer
	//TimerUpdateMode(TIMER0_BASE, TIMER_A, TIMER_UP_LOAD_IMMEDIATE);

	TimerClockSourceSet(TIMER5_BASE, TIMER_CLOCK_SYSTEM);

	TimerEnable(TIMER5_BASE, TIMER_A);


	uint16_t counter = 0;
	while (1) {
		current_val = TimerValueGet(TIMER5_BASE, TIMER_A);
		__delay_cycles(1200000);
		counter += 1;
		if (counter == 1000) {
			counter = 0;
			HWREG(TIMER5_BASE + 0x050) = 0x00000000;
			//TimerLoadSet(TIMER5_BASE, TIMER_A, 2000000000);
		}
	}

	return 0;
}
