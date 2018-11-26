/* Copyright (C) 2015-2017
 *
 * test-app-1pps-isr.c
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
#include <stdlib.h>
#include "hal_gpio.h"
#include "hal_mcu.h"
#include "hal_pps.h"

/*Don't do this !*/
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"



#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"

#include "inc/hw_types.h"
#include "inc/hw_nvic.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static bool loc_test_init(void);
static void loc_test_run(void);

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
bool last = 0;
Dummy_Timestamp PPS_Message;
Dummy_Timestamp Dummy_Message1;
Dummy_Timestamp Dummy_Message2;
Dummy_Timestamp Dummy_Message3;

void loc_cb(void)
{
  hal_gpio_write(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_4, HAL_GPIO_PIN_4);
  hal_pps_getTimestamp(&Dummy_Message1);
  hal_pps_getTimestamp(&Dummy_Message2);
  hal_pps_getCurrentPPS(&PPS_Message);
  hal_pps_getTimestamp(&Dummy_Message3);
  __delay_cycles(1000000);

  return;
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
    b_return = hal_gpio_init();
  }

  if(b_return)
  {
    b_return = hal_pps_init();
  }

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
  bool b_return = false;

  b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_M, HAL_GPIO_PIN_7, E_HAL_GPIO_DIR_IN);

  if(b_return)
  {
    b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
  }

  if(b_return)
  {
    b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_4, E_HAL_GPIO_DIR_OUT);
  }

  if(b_return)
  {
    b_return = hal_gpio_setInt(E_HAL_GPIO_PORT_M, HAL_GPIO_PIN_7, E_HAL_GPIO_INT_TYPE_RISING_EDGE, &loc_cb);
  }

  return b_return;
}/* loc_test_init() */


/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{

  while(1)
  {
  hal_gpio_write(E_HAL_GPIO_PORT_L, (HAL_GPIO_PIN_0 | HAL_GPIO_PIN_4), 0x0);
  }


}/* loc_test_run() */


int main(void) {
	if (!loc_hal_init()) return(0);
	
	if (!loc_test_init()) return(0);

    while(true) {
    	loc_test_run();
    }
	return false;
}
