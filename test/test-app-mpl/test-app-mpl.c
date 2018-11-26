/* Copyright (C) 2015-2017
 *
 * test-app-mpl.c
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

#include "hal.h"
#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_pps.h"
#include "mpl/mpl.h"



/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
Barometer baro_msg;
/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
bool loc_test_init(void);

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_test_init(void)
{
  bool b_return = hal_init();

  if (b_return) {
    b_return = hal_timer_init();
  }

  if (b_return) {
    b_return = mpl_init();
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
  mpl_run();
  mpl_get(&baro_msg);

}

/* ===========================================================================*/
/*                              MAIN                                          */
/* ===========================================================================*/
int main(void)
{
  if( hal_init() )
  {
    if (loc_test_init()) {
      while(true)
      {
        loc_test_run();
      }
    }
  }
}
