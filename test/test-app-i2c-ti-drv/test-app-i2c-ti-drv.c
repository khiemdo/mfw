/* Copyright (C) 2015-2017
 *
 * test-app-i2c-ti-drv.c
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
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_i2c_ti_drv.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static bool loc_test_init(void);
static void loc_test_run(void);

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the i2c-slave address. */
#define SLAVE_ADDRESS 0x3A
/*! Defines the i2c-port. */
#define TEST_APP_I2C_PORT E_HAL_I2C_PORT_1


/*! Delay between SENDs of the test app in milliseconds. */
#define TEST_APP_TEST_DELAY  100


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
s_hal_i2c_ctx_t *gps_i2c;
uint8_t gc_txData[5];
uint8_t gc_rxData[5];
uint32_t timer_txInterval;
uint16_t value = 0;

/* ===========================================================================*/
/*                  loc_test_cb() - callback function                         */
/* ===========================================================================*/
void loc_test_cb(void *pvData, uint_fast8_t ui8Status) {
	gps_i2c->state = E_HAL_I2C_STATE_IDLE;
}

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    b_return = hal_timer_init();
  }

  if(b_return)
  {
	gps_i2c = hal_i2c_ti_drv_init(TEST_APP_I2C_PORT);
    if(gps_i2c == NULL)
    {
      b_return = false;
    }
  }

  return b_return;
}/* loc_hal_init() */

/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
  gc_txData[0] = 0;
  gc_txData[1] = 0;
  gc_txData[2] = 0;
  gc_txData[3] = 0;

  gc_rxData[0] = 0;
  gc_rxData[1] = 0;
  gc_rxData[2] = 0;
  gc_rxData[3] = 0;


  /* All global variables must be initialized within an init function! */
  timer_txInterval = 0;

  return true;
}/* loc_test_init() */

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{

  	if (hal_timer_isTimedOut(timer_txInterval))
	{
  	  value = (value + 10) % 1000;

  	  gc_txData[0] = 0x00;
  	  gc_txData[1] = ((value+1000)>>8) & 0xFF;
  	  gc_txData[2] = (value+1000) & 0xFF;
  	  hal_i2c_write(gps_i2c, SLAVE_ADDRESS, gc_txData, 3, &loc_test_cb);
  	  //hal_i2c_read(gps_i2c, SLAVE_ADDRESS, gc_txData, 1, gc_rxData, 1, &loc_test_cb);
  	  timer_txInterval = hal_timer_getTimeout(TEST_APP_TEST_DELAY);
	}

}/* loc_test_run() */

int main(void) {
	if (!loc_hal_init()) return(0);

	if (!loc_test_init()) return(0);

    while(true) {
    	loc_test_run();
    }
   return(0);
}
