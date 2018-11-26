/* Copyright (C) 2015-2017
 *
 * test-app-bno055.c
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
#include <stdint.h>
#include <stdbool.h>

#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_pps.h"
#include "hal_i2c_ti_drv.h"
#include "bno055.h"

#include "Imu.pb.h"

#define SLAVE_ADDRESS 0x28


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
Imu imu_msg = {};
const Imu imu_msg_default = Imu_init_default;
int16_t accel_x, accel_y, accel_z;
/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
bool loc_hal_init(void);

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    b_return = hal_pps_init();
  }
  if(b_return)
  {
    b_return = bno055_init();
  }
  if(b_return)
  {
    b_return = hal_timer_init();
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
	__delay_cycles(100000000);
	bno055_run();
	imu_msg = imu_msg_default;
	if ( bno055_get(&imu_msg) )
	{
		accel_x = imu_msg.accel_raw.x;
		accel_y = imu_msg.accel_raw.y;
		accel_z = imu_msg.accel_raw.z;
	}
}

int main(void)
{
	if( loc_hal_init() )
	{

      while(true)
      {
        loc_test_run();
      }
    }
	while(1);
}
