/* Copyright (C) 2015-2017
 *
 * test-app-i2c.c
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


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "hal_i2c.h"
#include "hal_timer.h"
#include "hal_mcu.h"

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
#define SLAVE_ADDRESS 0x0A

/*! Defines the amount of test data bytes transfered in each test loop.
 *  This number shall not exceed 0xFFFF as 16bit variables are used. */
#define TEST_APP_TEST_DATA_LEN      1

/*! Defines the amount of repititions/loops for the test.
 *  For each repitition the amount of test data is \ref TEST_APP_TEST_DATA_LEN.
 *  This number shall not exceed 0xFFFFFFFF as 32bit variables are used. */
#define TEST_APP_TEST_REPETITIONS   2000

/*! Delay between SENDs of the test app in milliseconds. */
#define TEST_APP_TEST_DELAY   200

#define TEST_APP_I2C_PORT E_HAL_I2C_PORT_2


/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */
typedef enum TEST_APP_STATE
{
  E_TEST_APP_SEND_DATA,

  E_TEST_APP_RECEIVE_DATA,

  E_TEST_APP_ANALYZE_DATA

} e_test_app_state_t ;


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
uint8_t gc_testData[TEST_APP_TEST_DATA_LEN];
uint8_t gc_recvData[TEST_APP_TEST_DATA_LEN];
e_test_app_state_t ge_testState;
s_hal_i2c_ctx_t *gps_i2c;


/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
	gps_i2c = hal_i2c_init(TEST_APP_I2C_PORT);
    if(gps_i2c == NULL)
    {
      b_return = false;
    }
  }

  if(b_return)
  {
    b_return = hal_timer_init();
  }

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
  /* After startup of the test-app we transmit data. */
  ge_testState = E_TEST_APP_SEND_DATA;
  /* Fill in our test data. */
  int i;
  for (i = 0; i < TEST_APP_TEST_DATA_LEN; i++)
  {
	  gc_testData[i] = i;
	  gc_recvData[i] = 0;
  }
  return true;
}/* loc_test_init() */


/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
  uint8_t i_written = 0;
  uint32_t timer = 0;

  switch (ge_testState)
  {
    case E_TEST_APP_SEND_DATA:
      /* Call the I2C API until all data is transfered. Blocking! */
      for (i_written = 0 ; i_written<TEST_APP_TEST_REPETITIONS; i_written++) {
    	gc_testData[0] = (gc_testData[0] + 1)%255;
        hal_i2c_write_mul(gps_i2c, SLAVE_ADDRESS, gc_testData, TEST_APP_TEST_DATA_LEN);
        timer = hal_timer_getTimeout(TEST_APP_TEST_DELAY);
        while( !hal_timer_isTimedOut(timer) );
      }
      ge_testState = E_TEST_APP_RECEIVE_DATA;

      break;


    case E_TEST_APP_RECEIVE_DATA:
		hal_i2c_read_mul(gps_i2c, SLAVE_ADDRESS, gc_testData,1);
        hal_i2c_write_mul(gps_i2c, SLAVE_ADDRESS, gc_testData, TEST_APP_TEST_DATA_LEN);
        timer = hal_timer_getTimeout(TEST_APP_TEST_DELAY);
        ge_testState = E_TEST_APP_ANALYZE_DATA;
      break;


    case E_TEST_APP_ANALYZE_DATA:
    	if (gc_recvData[0] == gc_testData[0]){
    		/*To be implemented */
    	} else {
    		/*To be implemented */
    	}
    	ge_testState = E_TEST_APP_SEND_DATA;
      break;

    default:
    	/*We should never reach this. */
      break;
  }/* switch() */

}/* loc_test_run() */


int main(void) {
	if (!loc_hal_init()) return(0);
	
	if (!loc_test_init()) return(0);

    while(true) {
    	loc_test_run();
    }
	return 0;
}
