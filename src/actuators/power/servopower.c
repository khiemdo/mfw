/* Copyright (C) 2015-2017
 *
 * servopower.c
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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

#include "servopower.h"

#include "hal_gpio.h"

bool servopower_init(void)
{
  bool b_return = false;
  b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_3, E_HAL_GPIO_DIR_OUT);
  return b_return;
}

bool servopower_status(void)
{
  bool b_return = false;
  uint8_t ui8_status = 0;
  ui8_status = hal_gpio_read(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_3);
  /* If pin is low, power is on. */
  if(!(ui8_status & HAL_GPIO_PIN_3))
  {
    b_return = true;
  }
  /* If pin is high, power is off. */
  else
  {
    b_return = false;
  }
  return b_return;

}

bool servopower_on(void)
{
  bool b_return = false;
  b_return = hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_3, ~HAL_GPIO_PIN_3);
  return b_return;
}

bool servopower_off(void)
{
  bool b_return = false;
  b_return = hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_3, HAL_GPIO_PIN_3);
  return b_return;
}
