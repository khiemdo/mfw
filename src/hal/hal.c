/* Copyright (C) 2015-2017
 *
 * hal.c
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

#include "hal.h"

#include "hal_mcu.h"
#include "hal_dma.h"
#include "hal_gpio.h"
#include "hal_crc.h"
#include "hal_pps.h"

bool hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    b_return = hal_dma_init();
  }

  if(b_return)
  {
    b_return = hal_gpio_init();
  }

  if(b_return)
  {
    b_return = hal_crc_init();
  }

  if(b_return)
  {
    b_return = hal_pps_init();
  }

  return b_return;
}
