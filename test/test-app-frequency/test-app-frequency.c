/* Copyright (C) 2015-2017
 *
 * test-app-frequency.c
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
#include "hal_timer.h"
#include "hal_mcu.h"

/*Don't do this !*/
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
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
#define DELAY_CYCLES 59981          //must be 40 or greater
                                    //59981 for 1kHz @ FC3

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/

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

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
  bool b_return = false;

  b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_M, HAL_GPIO_PIN_7, E_HAL_GPIO_DIR_OUT);

  return b_return;
}/* loc_test_init() */


/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{

  while(1)
  {
    HWREG(GPIO_PORTM_BASE + 0x3FC) ^= 0x80;   // Toggle 1PPS-Pin (14 cycles including while loop)
                                              // ldr r1    -> 2/8
                                              // ldr r0    -> 2/2
                                              // eor r0,r0 -> 1/1
                                              // str r0    -> 2/1
    __asm(" NOP\n");                          // nop       -> 1/1 (clear pipeline)
                                              // b         -> 1/1
    __delay_cycles(DELAY_CYCLES - 14);

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
