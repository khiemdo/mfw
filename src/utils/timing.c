/* Copyright (C) 2015-2017
 *
 * timing.c
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

#include "timing.h"

#include "hal_timer.h"
#include "hal_pps.h"

/*! \brief Compile time switch: Use the low precision HAL timer implementation
 *         that has a resolution of 1ms. */
#define TIMING_USE_LOW_PRECISION_TIMER    1

/*! \brief Compile time switch: Use the high precision HAL timer implementation.
 *         Not implemented yet! */
#define TIMING_USE_HIGH_PRECISION_TIMER   0

/* Validate the compile time settings! */
#if ( TIMING_USE_LOW_PRECISION_TIMER && TIMING_USE_HIGH_PRECISION_TIMER )
#error Low and high precision timer can not be enabled at the same time! \
Check macros TIMING_USE_LOW_PRECISION_TIMER and TIMING_USE_HIGH_PRECISION_TIMER.
#endif

#if ( (TIMING_USE_LOW_PRECISION_TIMER + TIMING_USE_HIGH_PRECISION_TIMER) == 0)
#error Low and high precision timer can not be disabled at the same time! \
Check macros TIMING_USE_LOW_PRECISION_TIMER and TIMING_USE_HIGH_PRECISION_TIMER.
#endif


/* ===========================================================================*/
/*                  Defines/Macros                                            */
/* ===========================================================================*/


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/


/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/


/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/


/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  timing_init()                                             */
/* ===========================================================================*/
bool timing_init(void)
{
  bool b_return = false;

  #if TIMING_USE_LOW_PRECISION_TIMER
  /* As long as we have no high precision timer, we use the existing
   * hal_timer module for our notion of time. */
  b_return = hal_timer_init();
  #elif TIMING_USE_HIGH_PRECISION_TIMER
  /* High precision timer implementation here. */
  #endif

  return b_return;
}


/* ===========================================================================*/
/*                  timing_getTimeout()                                       */
/* ===========================================================================*/
uint32_t timing_getTimeout(uint32_t ui32_timeoutMs)
{
  #if TIMING_USE_LOW_PRECISION_TIMER
  return hal_timer_getTimeout(ui32_timeoutMs);
  #elif TIMING_USE_HIGH_PRECISION_TIMER
  /* High precision timer implementation here. */
  #endif
}


/* ===========================================================================*/
/*                  timing_isTimedOut()                                       */
/* ===========================================================================*/
bool timing_isTimedOut(uint32_t ui32_tmr)
{
  #if TIMING_USE_LOW_PRECISION_TIMER
  return hal_timer_isTimedOut(ui32_tmr);
  #elif TIMING_USE_HIGH_PRECISION_TIMER
  /* High precision timer implementation here. */
  #endif
}


/* ===========================================================================*/
/*                  timing_getTicks()                                         */
/* ===========================================================================*/
bool timing_getTimestamp(Timestamp * const timestamp_msg)
{
  bool b_return = false;

  if(timestamp_msg)
  {
    #if TIMING_USE_LOW_PRECISION_TIMER
    uint32_t currentTimeMs = 0;

    /* Requesting the timeout for '0' milliseconds actually returns the current
     * time since last startup. */
    currentTimeMs = hal_timer_getTimeout(0);

    /* Convert from milliseconds to seconds. */
    timestamp_msg->sec = (currentTimeMs/1000);
    timestamp_msg->has_sec          = true;

    /* Convert from milliseconds to nanoseconds within last second. */
    timestamp_msg->nsec          = ((currentTimeMs % 1000) * 8000);
    timestamp_msg->has_nsec          = true;

    b_return = true;

    #elif TIMING_USE_HIGH_PRECISION_TIMER
    /* High precision timer implementation here. */
    b_return = hal_pps_getTimestamp(timestamp_msg);

    #endif
  }

  return b_return;
}


