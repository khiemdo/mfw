/* Copyright (C) 2015-2017
 *
 * hal_dma.c
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

#include "hal_dma.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

uint32_t ui32_dmaErrorCount;


/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
void uDMAErrorHandler(void);



/* ===========================================================================*/
/*                  hal_dma_init()                                            */
/* ===========================================================================*/
bool hal_dma_init(void)
{
  //
  // Enable the uDMA controller at the system level.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

  //
  // Enable the uDMA controller error interrupt.  This interrupt will occur
  // if there is a bus error during a transfer.
  //
  uDMAIntRegister(UDMA_INT_ERR, uDMAErrorHandler); /* No ROM_() available! */
  MAP_IntEnable(INT_UDMAERR);


  //
  // Enable the uDMA controller.
  //
  MAP_uDMAEnable();

  //
  // Point at the control table to use for channel control structures.
  //
  MAP_uDMAControlBaseSet(pui8ControlTable);

  ui32_dmaErrorCount = 0;

  return true;
}


//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    //
    // Check for uDMA error bit
    //
    ui32Status = MAP_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        MAP_uDMAErrorStatusClear();
        ui32_dmaErrorCount++;
    }
}

