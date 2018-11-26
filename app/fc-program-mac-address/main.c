/* Copyright (C) 2015-2017
 *
 * main.c
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
#if !(defined(TARGET_IS_TM4C129_RA0) ||                                       \
      defined(TARGET_IS_TM4C129_RA1) ||                                       \
      defined(TARGET_IS_TM4C129_RA2))
#error Ethernet is available for TM4C129 devices only.
#else

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

/*! @brief MAC address part written to user_reg0
 *
 * @warning
 * Contact your team leader to ensure that you set a unique MAC address within
 * the setup! To make sure what you do, the macro is empty and the file does not
 * compile without your MAC address set.
 *
 * @details
 * When running the MFW framework using Ethernet connection, your MAC address
 * setting is used as follows:
 * - #define MAC_ADDR_TO_PROGRAM_0    0x00280008
 * - #define MAC_ADDR_TO_PROGRAM_1    0x0063855A
 * - Resulting MAC address: 08:00:28:5A:85:63
 *
 * Note that "08:00:28" is the MAC address space of Texas Instruments.
 * You should consider setting the "Universal/Local" (U/L) bit within your MAC
 * address.
 *
 * @sa MAC_ADDR_TO_PROGRAM_1
 */
#define MAC_ADDR_TO_PROGRAM_0

/*! @brief MAC address part written to user_reg1
 *
 * See @ref MAC_ADDR_TO_PROGRAM_0 for details.
 */
#define MAC_ADDR_TO_PROGRAM_1

/*! @brief Forces to write the MAC address even if one is present. Default: Off. */
#define MAC_ADDR_FORCE_PROGRAM  0

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/

/*
 * Use global variables for easy debugging in watch window.
 */
uint32_t ui32User0 = 0U;
uint32_t ui32User1 = 0U;
uint32_t ui32UserNew0 = MAC_ADDR_TO_PROGRAM_0;
uint32_t ui32UserNew1 = MAC_ADDR_TO_PROGRAM_1;

void main(void)
{
  /*! @warning
   * !!! WARNING !!!
   * The register used to store the MAC address (USER_REG0, USER_REG1), accessed
   * by FlashUserSet() is a WRITE-ONLY-ONCE register!
   * Once you programmed this register using this app, you cannot update it
   * (without some effort)!
   * So make sure your MAC address is really unique within the setup!
   * Check the datasheet for details:
   * - Tiva TM TM4C129ENCPDT Microcontroller,
   * - Rev: June 18, 2014,
   * - page 685
   * - section "USER_REG0"
   * !!! WARNING !!!
   */

  /* Get pre-programmed MAC address */
  MAP_FlashUserGet(&ui32User0, &ui32User1);
  /* Check if its valid or not. */
  if((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
  {
    /* There is no MAC address programmed yet, so program user MAC. */
    FlashUserSet(ui32UserNew0, ui32UserNew1);

    /* Stop here for debugging purpose. */
    while(1);
  }
  else
  {
    #if MAC_ADDR_FORCE_PROGRAM
    /* Force to program MAC address. */
    if(FlashUserSet(ui32UserNew0, ui32UserNew1) < 0)
    {
      /* Error writing MAC address! */
      while(1);
    }

    FlashUserSave();

    MAP_FlashUserGet(&ui32User0, &ui32User1);

    /* Stop here for debugging purpose. */
    while(1);
    #endif /* MAC_ADDR_FORCE_PROGRAM */
  }

  /* Stop here for debugging purpose. */
  while(1);
}

#endif /* !defined(TARGET...) */
