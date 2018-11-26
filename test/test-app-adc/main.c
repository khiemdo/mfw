/* Copyright (C) 2015-2017
 *
 * main.c
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
#include "hal.h"
#include "i2c-bus-manager.h"
#include "ads1115.h"
#include <stdbool.h>

E_I2C_BUS_NODE_T g_current_access;
s_hal_i2c_ctx_t * i2c_ctx;
bool sent = false;
uint8_t rx_data[12][2] = {
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00},
                   {0x00, 0x00}
                   };

void loc_test_cb(void *pvData, uint_fast8_t ui8Status) {
  i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
  sent = true;
}

bool loc_test_init() {
  bool b_return = hal_init();

  //if (b_return) {
  //  b_return = i2c_bus_mgr_init();
  //}

  //if (b_return) {
  //  b_return = ads1115_init(E_ADS1115_ID_1);
  //}
  if (b_return) {
    i2c_ctx = hal_i2c_ti_drv_init(E_HAL_I2C_PORT_5);
  }

  return b_return;
}

void loc_readChan(uint8_t chan)
{
  uint8_t tx_data[3] = {0x01, 0xC5, 0xE3};
  uint8_t addr = 0;
  if(chan <= 3)
    addr = ADS1115_SLAVE_ADDR_1;
  if(chan > 3 & chan <= 7)
    addr = ADS1115_SLAVE_ADDR_2;
  if(chan > 7 & chan <= 11)
    addr = ADS1115_SLAVE_ADDR_3;

  if(chan % 4 == 0)
    tx_data[1] = 0xC5;
  if(chan % 4 == 1)
    tx_data[1] = 0xD5;
  if(chan % 4 == 2)
    tx_data[1] = 0xE5;
  if(chan % 4 == 3)
    tx_data[1] = 0xF5;

  sent = false;
  hal_i2c_write(i2c_ctx, addr, tx_data, 3, &loc_test_cb);
  while(!sent);
  __delay_cycles(1200000);
  sent = false;
  tx_data[0] = 0x00;
  hal_i2c_read(i2c_ctx, addr, tx_data, 1, rx_data[chan], 2, &loc_test_cb);
  while(!sent);
    __delay_cycles(1200000);
}

void loc_test_run(){

  int i;
  for(i = 0; i < 12; i++){
    loc_readChan(i);
  }

}

int main(void) {
  if (!loc_test_init()) return(0);

    while(true) {
      loc_test_run();
    }
  return 0;
}
