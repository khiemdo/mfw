/* Copyright (C) 2015-2017
 *
 * test-app-mpu9250.c
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

/*
 * test-tube-angle.c
 */
#include <stdlib.h>
#include "hal_spi.h"
#include "hal_gpio.h"
#include "hal_mcu.h"
#include "hal_dma.h"
#include "hal_timer.h"
#include "mpu9250.h"

#include "imu.pb.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the amount of repititions/loops for the test.
 *  For each repitition the amount of test data is \ref TEST_APP_TEST_DATA_LEN.
 *  This number shall not exceed 0xFFFFFFFF as 32bit variables are used. */
#define TEST_APP_TEST_REPETITIONS   100

/*! Port ID of spi module. */
#define TEST_APP_SPI_PORT E_HAL_SPI_PORT_0

/*! Clockfrequency of SCK line in Hz. */
#define TEST_APP_SPI_SPEED 3000000
/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
BetCOM_Imu mpu_msg = {};
int value_x;
/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  if(!hal_mcu_init()){
	  return false;
  }

  hal_timer_init();

  hal_dma_init();

  // Initialize SPI port
  if(!mpu9250_init(TEST_APP_SPI_PORT, TEST_APP_SPI_SPEED)){
      return false;
  }
  // initialize the timer
  return true;
}/* loc_hal_init() */

/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{

  /* Fill in our test data. */
	return true;
}/* loc_test_init() */

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
	uint32_t timer;
	int i_written = 0;
	for (i_written = 0; i_written < TEST_APP_TEST_REPETITIONS; i_written++) {
		timer = hal_timer_getTimeout(100);
		while(!hal_timer_isTimedOut(timer));
		mpu9250_get(&mpu_msg);
		mpu9250_run();
		value_x = mpu_msg.measurements[0].accel_raw.x;
	}
}

int main(void) {
	if (!loc_hal_init()) return(0);

		if (!loc_test_init()) return(0);

	    while(true) {
	    	loc_test_run();
	    }
}
