/* Copyright (C) 2015-2017
 *
 * hal_ethernet.c
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
#include "hal_ethernet.h"

#if !(defined(TARGET_IS_TM4C129_RA0) ||                                       \
      defined(TARGET_IS_TM4C129_RA1) ||                                       \
      defined(TARGET_IS_TM4C129_RA2))
#error Ethernet is available for TM4C129 devices only.
#else

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
static uint8_t g_macAddress[8] = { 0U };
static bool g_isEthernetInitialized = false;

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

bool hal_eth_init(void)
{
  bool b_ret = true;
  uint32_t ui32User0 = 0U;
  uint32_t ui32User1 = 0U;

  //
  // Make sure the main oscillator is enabled because this is required by
  // the PHY.  The system must have a 25MHz crystal attached to the OSC
  // pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
  // frequency is 10MHz or higher.
  //
  SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

  /* Init ETH LEDs */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinConfigure(GPIO_PF0_EN0LED0);
  ROM_GPIOPinConfigure(GPIO_PF4_EN0LED1);

  GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);


  /* Get and store pre-programmed MAC adress */
  MAP_FlashUserGet(&ui32User0, &ui32User1);
  if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
  {
    //
    // We should never get here.  This is an error if the MAC address has
    // not been programmed into the device.  Exit the program.
    // Let the user know there is no MAC address
    //
    b_ret = false;
  }
  else
  {
    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    g_macAddress[0] = ((ui32User0 >>  0) & 0xff);
    g_macAddress[1] = ((ui32User0 >>  8) & 0xff);
    g_macAddress[2] = ((ui32User0 >> 16) & 0xff);
    g_macAddress[3] = ((ui32User1 >>  0) & 0xff);
    g_macAddress[4] = ((ui32User1 >>  8) & 0xff);
    g_macAddress[5] = ((ui32User1 >> 16) & 0xff);
  }

  if(b_ret)
  {
    /* TI demo sets priorities after low level initialization of ETH MAC/PHY
     * but this seems to be working fine too. */

    //
    // Set the interrupt priorities.  We set the SysTick interrupt to a higher
    // priority than the Ethernet interrupt to ensure that the file system
    // tick is processed if SysTick occurs while the Ethernet handler is being
    // processed.  This is very likely since all the TCP/IP and HTTP work is
    // done in the context of the Ethernet interrupt.
    //
    MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);


    /* Finally, mark that the module is initialized properly. */
    g_isEthernetInitialized = true;
  }

  return b_ret;
}

const uint8_t *hal_eth_getMacAddress(void)
{
  uint8_t *p_ret = NULL;

  if(g_isEthernetInitialized)
  {
    p_ret = g_macAddress;
  }

  return p_ret;
}

#endif /* #if defined(TARGET_IS_ ... */
