/* Copyright (C) 2015-2017
 *
 * main.c
 *
 * Fabian Girrbach     <fabiangirrbach@gmail.com>
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

#include "imu_xsens/imu_xsens.h"
#include "hal_timer.h"
#include "hal_mcu.h"
#include "betcall.pb.h"

//---------------------------------------------------------
//				GLOBAL VARIABLES
//---------------------------------------------------------
static BetCALL_Sensors sensors = BetCALL_Sensors_init_zero;
const  BetCALL_Sensors sensors_init = BetCALL_Sensors_init_default;




void delay(const uint32_t timeout)
{
	uint32_t timer = hal_timer_getTimeout(timeout);
	while (hal_timer_isTimedOut(timer));
}


int main(void)
{

	if (!hal_mcu_init())
	{
		return 0;
	}

	if (!hal_timer_init()){
		return 0;
	}

	sensors = sensors_init;

	s_imu_xsens_ctx_t* imu = xsens_createImu(xsensUART, 115200);
	if (imu != NULL)
	{
		xsens_configureMotionTracker(imu);
		if (xsens_goToMeasurmentMode(imu))
		{
			uint32_t timer = hal_timer_getTimeout(1000);
			while (hal_timer_getTimeout(timer))
			{
				sensors.imu_count = xsens_getSensorData(imu, &sensors.imu[0]);
			}
		}



		return 1;
	}


	return 0;
}
