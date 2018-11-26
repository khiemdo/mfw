/* Copyright (C) 2015-2017
 *
 * hal_mcu.c
 *
 * Martin Dold         <martin.dold@gmx.net>
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

#include "inc/hw_ints.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "hal_mcu.h"

uint32_t g_ui32SysClock;

bool hal_mcu_init(void)
{
  bool b_return = false;
  //
  // Set the clocking to run directly from the crystal at 120MHz/80MHz.
  //
  #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
      defined(TARGET_IS_TM4C129_RA1) ||                                         \
      defined(TARGET_IS_TM4C129_RA2)
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                           SYSCTL_USE_PLL |
                                           SYSCTL_CFG_VCO_480), 120000000);
  b_return = true;
  #endif
  #if defined(TARGET_IS_TM4C123_RA0) ||                                         \
      defined(TARGET_IS_TM4C123_RA1) ||                                         \
      defined(TARGET_IS_TM4C123_RA2)
  SysCtlClockSet(SYSCTL_XTAL_25MHZ |
                 SYSCTL_OSC_MAIN |
                 SYSCTL_USE_PLL |
                 SYSCTL_SYSDIV_2_5 );
  g_ui32SysClock = SysCtlClockGet();
  b_return = true;
  #endif

  //
  // Enable processor interrupts.
  //
  MAP_IntMasterEnable();

  return b_return;
}


uint32_t hal_mcu_getSysClock(void)
{
  return g_ui32SysClock;
}
