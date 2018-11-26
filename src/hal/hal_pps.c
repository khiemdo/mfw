/* Copyright (C) 2015-2017
 *
 * hal_pps.c
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
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

#include <stdbool.h>
#include <stdint.h>

#include "hal_pps.h"

#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/cpu.h"

static volatile uint32_t g_pps_current_counter; //Replace those two by the new BetCOM_PPS_message with version 2
static volatile uint32_t g_pps_ticks_since_last;

bool hal_pps_init(void)
{

  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC_UP);   // 32 bits Timer
  TimerClockSourceSet(TIMER5_BASE, TIMER_CLOCK_SYSTEM);
  TimerEnable(TIMER5_BASE, TIMER_A);

  g_pps_current_counter = 0;
  g_pps_ticks_since_last = 0;

  return true;
}

inline void hal_pps_isr(void)
{
  //Earliest point to add interrupt handling code.

  g_pps_ticks_since_last = HWREG(TIMER5_BASE + TIMER_O_TAR);
  HWREG(TIMER5_BASE + TIMER_O_TAV) = 0x00000000;
  g_pps_current_counter++;
}

inline bool hal_pps_getCurrentPPS(Pps *const pps_msg)
{
  bool b_return = false;

  if (pps_msg)
  {
    pps_msg->ticks_since_last = g_pps_ticks_since_last;
    pps_msg->has_ticks_since_last = true;
    pps_msg->current_counter = g_pps_current_counter;
    pps_msg->has_current_counter = true;

    b_return = true;
  }

  return b_return;
}

inline bool hal_pps_getTimestamp(Timestamp *const timestamp_msg)
{
  bool b_return = false;
  if (timestamp_msg) {
    //CPUcpsid();  // Disable Interrupt
    //__asm("    cpsid   i\n");
    timestamp_msg->nsec = HWREG(TIMER5_BASE + TIMER_O_TAR);
    timestamp_msg->has_nsec = true;
    timestamp_msg->sec = g_pps_current_counter;
    timestamp_msg->has_sec = true;
    //CPUcpsie();  // Enable Interrupt
    //__asm("    cpsie   i\n");
    b_return = true;
  }
  return b_return;
}
