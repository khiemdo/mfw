/* Copyright (C) 2015-2017
 *
 * hal_timer.c
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
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
#include "hal_timer.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"

#include "hal_mcu.h"

//*****************************************************************************
//
// The number of SysTick ticks per second used for the SysTick interrupt.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1000

typedef struct
{
  void (*pfn_callback)(void);
  uint32_t intervall;
  uint32_t timeout;
} timer ;

void SysTickHandler(void);

static volatile uint32_t g_ui32TickCount;
static timer g_timer;

bool hal_timer_init(void)
{
  //
  // Configure SysTick to occur 1000 times per second, to use as a time
  // reference.  Enable SysTick to generate interrupts.
  //
  SysTickIntRegister( SysTickHandler );
  MAP_SysTickPeriodSet( hal_mcu_getSysClock() / SYSTICKS_PER_SECOND);
  MAP_SysTickIntEnable();
  MAP_SysTickEnable();

  g_ui32TickCount = 0;
  memset( (void*)&g_timer, 0U, sizeof(g_timer));
  g_timer.pfn_callback = NULL;

  return true;
}

uint32_t hal_timer_getTimeout(uint32_t ui32_timeoutMs)
{
  return (g_ui32TickCount + ui32_timeoutMs);
}

bool hal_timer_isTimedOut(uint32_t ui32_tmr)
{
  bool b_return = false;

  if( g_ui32TickCount >= ui32_tmr)
  {
    b_return = true;
  }

  return b_return;
}

bool hal_timer_registerCallback(uint32_t intervall, void (*pfn_callback)(void))
{
  bool b_return = false;

  if(pfn_callback)
  {
    g_timer.pfn_callback = pfn_callback;
    g_timer.intervall = intervall;
    /* Immediately start the timer. */
    g_timer.timeout = g_ui32TickCount + intervall;
    b_return = true;
  }

  return b_return;
}

//*****************************************************************************
//
// The interrupt handler for the SysTick timer.  This handler will increment a
// milliseconds counter in each tick (in each call respectively.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    //
    // Increment the tick counter.
    //
    g_ui32TickCount++;

    /* A single timer with callback can be requested by this module.
     * Serve it if present/requested. */
    if( g_timer.pfn_callback )
    {
      if( g_ui32TickCount >= g_timer.timeout )
      {
        g_timer.timeout += g_timer.intervall;
        g_timer.pfn_callback();
      }
    }

    return;
}
