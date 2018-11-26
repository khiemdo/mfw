/* Copyright (C) 2015-2017
 *
 * test-app-adc.c
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
#include <stdbool.h>
#include "hal.h"
#include "hal_timer.h"
#include "i2c-bus-manager.h"
#include "ads1115.h"

s_hal_i2c_ctx_t *i2c_ctx;
s_ads1115_ctx_t *gs_ads_ctx1;
s_ads1115_ctx_t *gs_ads_ctx2;
s_ads1115_ctx_t *gs_ads_ctx3;
Adc volt_msg1;
Adc volt_msg2;
Adc volt_msg3;

bool loc_test_init() {
  bool b_return = hal_init();

  if (b_return) {
    b_return = hal_timer_init();
  }

  if (b_return) {
    gs_ads_ctx1 = ads1115_init(E_ADS1115_ID_1);
    gs_ads_ctx2 = ads1115_init(E_ADS1115_ID_2);
    gs_ads_ctx3 = ads1115_init(E_ADS1115_ID_3);
  }

  if (gs_ads_ctx1 && gs_ads_ctx2 && gs_ads_ctx3) {
    i2c_ctx = hal_i2c_ti_drv_init(E_HAL_I2C_PORT_5);
  }

  return b_return;
}

void loc_test_run(){
  ads1115_run(gs_ads_ctx1);
  ads1115_run(gs_ads_ctx2);
  ads1115_get( &volt_msg1);
  ads1115_get(&volt_msg2);
}

int main(void) {
  if (!loc_test_init()) return(0);

    while(true) {
      loc_test_run();
    }
  return 0;
}


