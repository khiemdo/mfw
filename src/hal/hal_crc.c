/* Copyright (C) 2015-2017
 *
 * hal_crc.c
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

#include "hal_crc.h"

#include "inc/hw_memmap.h"

#include "driverlib/crc.h"
#include "driverlib/sysctl.h"

#include "crc16.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Seed value to be used for CRC-CCITT calculation (polynom = 0x1021) */
#define HAL_CRC_CCITT_SEED 0xFFFFFFFF

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
volatile static bool g_hw_crc_av = false;

/* ===========================================================================*/
/*                    API function implementations                            */
/* ===========================================================================*/

bool hal_crc_init(void)
{
  bool b_return = false;

  if (SysCtlPeripheralPresent(SYSCTL_PERIPH_CCM0))
  {
    //
    // Enable the CRC module.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);

    //
    // Wait for the CRC module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0));

    //
    // Configure the CRC module.
    //
    CRCConfigSet(CCM0_BASE,
                 CRC_CFG_INIT_SEED |
                 CRC_CFG_TYPE_P1021 |
                 CRC_CFG_SIZE_8BIT);
    g_hw_crc_av = true;
    b_return = true;
  }
  else
  {
    // Use software crc
    g_hw_crc_av = false;
    b_return = true;
  }
  return b_return;
}


uint16_t hal_crc_calculate(const uint8_t *pc_data, uint16_t ui16_len)
{
  uint16_t crc_result = 0;

  if(g_hw_crc_av)
  {
    /* Seed must be set for each new CRC calculation! */
    CRCSeedSet(CCM0_BASE, HAL_CRC_CCITT_SEED);
    /* Enable post-operation to finalize the CRC calculation within this single
     * call. If process() shall be called several times for one CRC calculation
     * this value shall be set to false for the first calls. */
    crc_result = CRCDataProcess(CCM0_BASE, (uint32_t *) pc_data, ui16_len, true);
  }
  else
  {
    /* If no hardware CRC module is available, use software CRC for API functionality. */
    crc_result = crcFast(pc_data, (uint32_t) ui16_len);
  }

  return crc_result;
}
