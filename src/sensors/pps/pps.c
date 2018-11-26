/* Copyright (C) 2015-2017
 *
 * pps.c
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

#include <stdlib.h>
#include <string.h>

#include "mfw-global-config.h"

#include "pps.h"

#include "hal_gpio.h"
#include "hal_pps.h"
#include "hal_timer.h"

#define PPS_TIMER_LENGTH 1000
#define PPS_TIMEOUT 2000
#define PPS_LED_ON  20

typedef enum
{
  E_PPS_STATE_IDLE,
  E_PPS_STATE_DATA_READY,
} E_PPS_STATE_t;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
static E_PPS_STATE_t PPS_state;
static Pps pps;
static uint32_t PPS_timer;
static volatile uint32_t PPS_timeout;
#if MFW_PPS_LED_EN
static volatile uint32_t PPS_led_on;
#endif
static PpsConf_Mode current_mode;
static PpsConf_Mode last_mode;
static const Pps g_ppsDefault = Pps_init_default;

/* ===========================================================================*/
/*                  LOCAL FUNCTION IMPLEMENTATIONS and PROTOTYPES             */
/* ===========================================================================*/
static void loc_isr_cb(void);
static inline bool loc_generate_pps_tick(void);

/* ===========================================================================*/
/*                         loc_isr_cb after gyro read                       */
/* ===========================================================================*/
void loc_isr_cb()
{
  hal_pps_getCurrentPPS(&pps);
  PPS_state = E_PPS_STATE_DATA_READY;
  PPS_timeout = hal_timer_getTimeout(PPS_TIMEOUT);

  #if MFW_PPS_LED_EN
  hal_gpio_write(MFW_PPS_LED_PORT, MFW_PPS_LED_PIN, MFW_PPS_LED_PIN);
  PPS_led_on = hal_timer_getTimeout(PPS_LED_ON);
  #endif
}

static inline bool loc_generate_pps_tick(void)
{
  return hal_gpio_write(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, MFW_PPS_INPUT_PIN);
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       pps_init() Initializes PPS                                           */
/* ===========================================================================*/
bool pps_init()
{
  bool b_return = true;
  PPS_state = E_PPS_STATE_IDLE;
  current_mode = PpsConf_Mode_GPS_TRIG_MODE;
  last_mode = PpsConf_Mode_GPS_TRIG_MODE;
  pps = g_ppsDefault;
  PPS_timer = hal_timer_getTimeout(PPS_TIMER_LENGTH);
  PPS_timeout = hal_timer_getTimeout(PPS_TIMEOUT);
  #if MFW_PPS_LED_EN
  PPS_led_on = 0;
  #endif

  #if (FC_GPS || ARM_GPS)
  if(b_return)
  {
    b_return = hal_gpio_unlock(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN);
  }
  #endif

  if(b_return)
  {
    b_return = hal_gpio_setDir(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_DIR_IN);
  }

  if(b_return)
  {
    b_return = hal_gpio_setInt(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_INT_TYPE_RISING_EDGE, &loc_isr_cb);
  }

  #if (FC_GPS || ARM_GPS)
  if(b_return)
  {
    b_return = hal_gpio_unlock(MFW_PPS_LED_PORT, MFW_PPS_LED_PIN);
  }
  #endif

  #if MFW_PPS_LED_EN
  if(b_return)
  {
    b_return = hal_gpio_setDir(MFW_PPS_LED_PORT, MFW_PPS_LED_PIN, E_HAL_GPIO_DIR_OUT);
  }
  #endif

  return b_return;
}

void pps_run()
{
  switch(PPS_state)
  {
  case E_PPS_STATE_IDLE:
    if(hal_timer_isTimedOut(PPS_timer) && current_mode == PpsConf_Mode_FC_TRIG_MODE)
    {
      PPS_timer = hal_timer_getTimeout(PPS_TIMER_LENGTH);
      loc_generate_pps_tick();
    }
    if(hal_timer_isTimedOut(PPS_timeout))
    {
      current_mode = last_mode = PpsConf_Mode_FC_TRIG_MODE;
      hal_gpio_setDir(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_DIR_OUT);
    }
    if(current_mode != PpsConf_Mode_GPS_TRIG_MODE)
    {
      hal_gpio_write(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, 0x0);
    }
    #if MFW_PPS_LED_EN
    if(hal_timer_isTimedOut(PPS_led_on))
    {
      hal_gpio_write(MFW_PPS_LED_PORT, MFW_PPS_LED_PIN, 0x0);
    }
    #endif
    break;
  case E_PPS_STATE_DATA_READY:
    break;
  default:
    break;
  }
  return;
}

bool pps_get(Pps *const pps_msg)
{
  bool b_return = false;
  if(PPS_state == E_PPS_STATE_DATA_READY)
  {
    *pps_msg = pps;
    pps = g_ppsDefault;
    PPS_state = E_PPS_STATE_IDLE;
    b_return = true;
  }
  return b_return;
}

bool pps_setConf(PpsConf *const ppsConf_msg)
{
  bool b_return = false;

  if(ppsConf_msg->has_mode)
  {
    current_mode = ppsConf_msg->mode;

    if(current_mode == last_mode)
    {
      if(current_mode == PpsConf_Mode_GS_TRIG_MODE)
      {
        b_return = loc_generate_pps_tick();
      }
      else
      {
        b_return = true;
      }
    }
    else
    {
      switch(current_mode)
      {
      case PpsConf_Mode_FC_TRIG_MODE:
        b_return = hal_gpio_setDir(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_DIR_OUT);
        break;
      case PpsConf_Mode_GS_TRIG_MODE:
        b_return = hal_gpio_setDir(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_DIR_OUT);
        if(b_return)
        {
          b_return = loc_generate_pps_tick();
        }
        break;
      case PpsConf_Mode_GPS_TRIG_MODE:
        b_return = hal_gpio_setDir(MFW_PPS_INPUT_PORT, MFW_PPS_INPUT_PIN, E_HAL_GPIO_DIR_IN);
        break;
      default:
        break;
      }
    }

    last_mode = current_mode;
  }

  return b_return;
}
