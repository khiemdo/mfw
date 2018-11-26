/* Copyright (C) 2015-2017
 *
 * test-app-precise-timing.c
 *
 * Tobias Paxian       <tobias.paxian@gmx.de>
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

#include<stdint.h>
#include<stdbool.h>
#include "hal_mcu.h"
#include "hal_gpio.h"
#include "hal_timer.h"


static bool loc_hal_init(void);
static void loc_test_led(void);

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
	b_return = hal_gpio_init();
  }

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_led()                                        */
/* ===========================================================================*/
void loc_test_led(void)
{
  uint8_t i;

  uint32_t timer = 0;

  uint32_t nextTimeOut = 100000;
  uint32_t actualTimer = 0;
  uint32_t maxTimeOut = 0;

  uint32_t ledTimeOut[4] = {330, 600, 850, 232};

  bool ledChange[4] = {false, false, false, false};

  uint8_t ledPin1 = HAL_GPIO_PIN_1;
  uint8_t ledPin2 = HAL_GPIO_PIN_0;
  uint8_t ledPin3 = HAL_GPIO_PIN_4;
  uint8_t ledPin4 = HAL_GPIO_PIN_0;

  hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, E_HAL_GPIO_DIR_OUT);
  hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
  hal_gpio_setDir(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_4, E_HAL_GPIO_DIR_OUT);
  hal_gpio_setDir(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);

  //
  // Turn on the LED as a heartbeat
  //
  //ledPin2 ^= HAL_GPIO_PIN_0;
  //hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin2);
  //hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, ledPin1);


  /* Set the timout at beginning to min of LED's milliseconds. */
  for ( i = 0; i < 4; i = i + 1 )
  {
      if ( ledTimeOut[i] > maxTimeOut)
    	maxTimeOut = ledTimeOut[i];

	  if ( ledTimeOut[i] == nextTimeOut ) {
		  ledChange[i] = true;
	  }
	  else if ( ledTimeOut[i] < nextTimeOut ) {
		  ledChange[0] = false;
		  ledChange[1] = false;
		  ledChange[2] = false;
		  ledChange[3] = false;
		  ledChange[i] = true;
		  nextTimeOut = ledTimeOut[i];
	  }
  }

  timer = hal_timer_getTimeout(nextTimeOut);
  while(true)
  {
    /* Toggle LED every 500ms to signal we passed the test successfully. */
    if( hal_timer_isTimedOut(timer) )
    {
      actualTimer = actualTimer + nextTimeOut;

      if ( ledChange[0] ) {
        ledPin1 ^= HAL_GPIO_PIN_1;
        hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, ledPin1);
      }
      if ( ledChange[1] ) {
    	ledPin2 ^= HAL_GPIO_PIN_0;
    	hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin2);
      }
      if ( ledChange[2] ) {
    	ledPin3 ^= HAL_GPIO_PIN_4;
    	hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_4, ledPin3);
      }
      if ( ledChange[3] ) {
    	ledPin4 ^= HAL_GPIO_PIN_0;
    	hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, ledPin4);
      }

      nextTimeOut = maxTimeOut;
      for ( i = 0; i < 4; i = i + 1 )
      {

    	  if ( ledTimeOut[i] - (actualTimer % ledTimeOut[i]) == nextTimeOut ) {
    		  ledChange[i] = true;
    		  nextTimeOut = ledTimeOut[i] - (actualTimer % ledTimeOut[i]);
    	  }
    	  else if ( ledTimeOut[i] - (actualTimer % ledTimeOut[i]) < nextTimeOut ) {
    		  ledChange[0] = false;
    		  ledChange[1] = false;
    		  ledChange[2] = false;
    		  ledChange[3] = false;
    		  ledChange[i] = true;
    		  nextTimeOut = ledTimeOut[i] - (actualTimer % ledTimeOut[i]);
    	  }
      }
      timer = hal_timer_getTimeout(nextTimeOut);
    }
  }
}/* loc_test_led() */

int main(void) {
	
	loc_hal_init();
	hal_timer_init();
	loc_test_led();

	return 0;
}
