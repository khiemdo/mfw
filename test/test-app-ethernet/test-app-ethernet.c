/* Copyright (C) 2015-2017
 *
 * test-app-ethernet.c
 *
 * Martin Dold    <martin.dold@gmx.net>
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
#include <string.h>

#include "hal_mcu.h"
#include "hal_dma.h"
#include "hal_gpio.h"
#include "hal_timer.h"

#include "mfw-socket.h"
#include "eth-benchmark/eth-benchmark.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

/*! IP settings: */
#define IP_ADDR_1       192
#define IP_ADDR_2       168
#define IP_ADDR_3         2
#define IP_ADDR_4        42

#define NET_MASK_1      255
#define NET_MASK_2      255
#define NET_MASK_3      255
#define NET_MASK_4        0

#define GW_IP_ADDR_1    192
#define GW_IP_ADDR_2    168
#define GW_IP_ADDR_3      2
#define GW_IP_ADDR_4      1

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/


/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_halInit(void);
static bool loc_testInit(void);
static void loc_testRun(void);


/* ===========================================================================*/
/*                  loc_halInit()                                             */
/* ===========================================================================*/
static bool loc_halInit(void)
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
    b_return = hal_timer_init();
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_testInit()                                           */
/* ===========================================================================*/
static bool loc_testInit(void)
{
  bool b_return = false;

  uint32_t ipAddr   = (   (IP_ADDR_1 << 24) | \
                          (IP_ADDR_2 << 16) | \
                          (IP_ADDR_3 <<  8) | \
                          (IP_ADDR_4      )
                      );
  uint32_t netmask  = (   (NET_MASK_1 << 24) | \
                          (NET_MASK_2 << 16) | \
                          (NET_MASK_3 <<  8) | \
                           NET_MASK_4
                      );
  uint32_t gwIpAddr = (   (GW_IP_ADDR_1 << 24) | \
                          (GW_IP_ADDR_2 << 16) | \
                          (GW_IP_ADDR_3 <<  8) | \
                          (GW_IP_ADDR_4      )
                      );

  /* Init low-level ETH MAC/PHY and IP stack. */
  b_return = mfw_socket_init(ipAddr, netmask, gwIpAddr);

  /* The MFW would now proceed with calling mfw_socket_udpStart() to initialize
   * a UDP socket that is used for communication between FC and GS. For the
   * purpose of this test-app, we initialize our benchmark app instead.
   */

  if(b_return)
  {
    /* Finally, setup our ETH benchmark app. */
    eth_benchmark_init();
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_testRun()                                             */
/* ===========================================================================*/
static void loc_testRun(void)
{
  //
  // All the lwIP work is done in interrupt handlers.
  //
}

/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void)
{
  if( loc_halInit() )
  {
    if( loc_testInit() )
    {
      while(1)
      {
        loc_testRun();
      }
    }
  }
	
	return 0;
}
