/* Copyright (C) 2015-2017
 *
 * hal_spi.c
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

#include "hal_spi.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/ssi.h"

#include "hal_mcu.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
#ifndef SSI_MAX_NUM
#define SSI_MAX_NUM   4
#endif


#ifndef SSI_LOOPBACK_ENABLED
/* By default loopback is disabled. */
#define SSI_LOOPBACK_ENABLED  0
#endif

#define HAL_SPI_DMA_TX_STATUS_FIN   0x01
#define HAL_SPI_DMA_RX_STATUS_FIN   0x02

#define HAL_SPI_EMPTY_RX_FIFO(baseAddr)                         \
  uint32_t trashBin[1] = {0};                                   \
  while (MAP_SSIDataGetNonBlocking(baseAddr, &trashBin[0]))

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
struct S_HAL_SPI_CTX_T
{
  uint32_t ui32_baseAddr;
  uint32_t ui32_dmaTxChannel;
  uint32_t ui32_dmaRxChannel;
  uint32_t ui32_interrupt;

  uint8_t ui8_dmaStatusFlags;

  uint8_t *pc_txBuf;
  uint8_t *pc_rxBuf;
  uint16_t ui16_len;

  pfn_spi_callback pf_cb;
};

uint8_t gc_spiDummyByte = 0xFF;

s_hal_spi_ctx_t gs_ssi[SSI_MAX_NUM];

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static s_hal_spi_ctx_t *loc_initSSI0(uint32_t ui32_spiClock);
static s_hal_spi_ctx_t *loc_initSSI1(uint32_t ui32_spiClock);
static s_hal_spi_ctx_t *loc_initSSI2(uint32_t ui32_spiClock);
static s_hal_spi_ctx_t *loc_initSSI3(uint32_t ui32_spiClock);
static void loc_spiTx(s_hal_spi_ctx_t *ps_ctx);
static void loc_spiRx(s_hal_spi_ctx_t *ps_ctx);
static void loc_spiRxTx(s_hal_spi_ctx_t *ps_ctx);
static void loc_handleInterrupts(s_hal_spi_ctx_t *ps_ctx);

static void SSI0IntHandler(void);
static void SSI1IntHandler(void);
static void SSI2IntHandler(void);
static void SSI3IntHandler(void);


/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_initSSI0()                                            */
/* ===========================================================================*/
s_hal_spi_ctx_t *loc_initSSI0(uint32_t ui32_spiClock)
{
  s_hal_spi_ctx_t *ps_return = NULL;
  uint32_t ui32SysClock = 0;

  if(ui32_spiClock > 0 && ui32_spiClock < 10000000)
  {
    ui32SysClock = hal_mcu_getSysClock();

    //
    // Enable the SSI0 Peripheral.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure GPIO Pins for SSI0 mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);	  // Clk
    MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS);   // CS
    MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0); // Tx
    MAP_GPIOPinConfigure(GPIO_PA5_SSI0XDAT1); // Rx
    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
            SSI_MODE_MASTER, ui32_spiClock, 8);

    SSIEnable(SSI0_BASE);
    SSIIntRegister( SSI0_BASE, SSI0IntHandler );

    HAL_SPI_EMPTY_RX_FIFO(SSI0_BASE);

    memset((void *) &gs_ssi[0], 0, sizeof(s_hal_spi_ctx_t));
    ps_return = &gs_ssi[0];

    ps_return->ui32_baseAddr = SSI0_BASE;
    ps_return->ui32_dmaTxChannel = UDMA_CHANNEL_SSI0TX;
    ps_return->ui32_dmaRxChannel = UDMA_CHANNEL_SSI0RX;
    ps_return->ui32_interrupt = INT_SSI0;

    /* After startup none of the transfers are finished. */
    ps_return->ui8_dmaStatusFlags = 0;

  }

  return ps_return;
}

/* ===========================================================================*/
/*                  loc_initSSI1()                                            */
/* ===========================================================================*/
s_hal_spi_ctx_t *loc_initSSI1(uint32_t ui32_spiClock)
{
  s_hal_spi_ctx_t *ps_return = NULL;
  uint32_t ui32SysClock = 0;

  if(ui32_spiClock > 0 && ui32_spiClock < 10000000)
  {
    ui32SysClock = hal_mcu_getSysClock();

    //
    // Enable the SSI1 Peripheral.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); /* CLK -> PB5 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); /* MOSI/MISO -> PE4/5*/
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // Configure GPIO Pins for SSI1 mode.
    //
    MAP_GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    MAP_GPIOPinConfigure(GPIO_PE4_SSI1XDAT0); // Tx
    MAP_GPIOPinConfigure(GPIO_PE5_SSI1XDAT1); // Rx
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
            SSI_MODE_MASTER, ui32_spiClock, 8);

    SSIEnable(SSI1_BASE);
    SSIIntRegister( SSI1_BASE, SSI1IntHandler );

    HAL_SPI_EMPTY_RX_FIFO(SSI1_BASE);

    memset((void *) &gs_ssi[1], 0, sizeof(s_hal_spi_ctx_t));
    ps_return = &gs_ssi[1];

    ps_return->ui32_baseAddr = SSI1_BASE;
    ps_return->ui32_dmaTxChannel = UDMA_CHANNEL_SSI1TX;
    ps_return->ui32_dmaRxChannel = UDMA_CHANNEL_SSI1RX;
    ps_return->ui32_interrupt = INT_SSI1;

    /* After startup none of the transfers are finished. */
    ps_return->ui8_dmaStatusFlags = 0;

  }

  return ps_return;
}

/* ===========================================================================*/
/*                  loc_initSSI2()                                            */
/* ===========================================================================*/
s_hal_spi_ctx_t *loc_initSSI2(uint32_t ui32_spiClock)
{
  s_hal_spi_ctx_t *ps_return = NULL;
  uint32_t ui32SysClock = 0;

  if(ui32_spiClock > 0 && ui32_spiClock < 10000000)
  {
    ui32SysClock = hal_mcu_getSysClock();

    //
    // Enable the SSI2 Peripheral.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    //
    // Configure GPIO Pins for SSI2 mode.
    //
    MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0); // Tx
    MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1); // Rx
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

    SSIConfigSetExpClk(SSI2_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
            SSI_MODE_MASTER, ui32_spiClock, 8);

    SSIEnable(SSI2_BASE);
    SSIIntRegister( SSI2_BASE, SSI2IntHandler );

    HAL_SPI_EMPTY_RX_FIFO(SSI2_BASE);

    memset((void *) &gs_ssi[2], 0, sizeof(s_hal_spi_ctx_t));
    ps_return = &gs_ssi[2];

    ps_return->ui32_baseAddr = SSI2_BASE;
    uDMAChannelAssign(UDMA_CH13_SSI2TX);
    ps_return->ui32_dmaTxChannel = 13;
    uDMAChannelAssign(UDMA_CH12_SSI2RX);
    ps_return->ui32_dmaRxChannel = 12;
    ps_return->ui32_interrupt = INT_SSI2;

    /* After startup none of the transfers are finished. */
    ps_return->ui8_dmaStatusFlags = 0;

  }

  return ps_return;
}

/* ===========================================================================*/
/*                  loc_initSSI3()                                            */
/* ===========================================================================*/
s_hal_spi_ctx_t *loc_initSSI3(uint32_t ui32_spiClock)
{
  s_hal_spi_ctx_t *ps_return = NULL;
  uint32_t ui32SysClock = 0;

  if(ui32_spiClock > 0 && ui32_spiClock < 10000000)
  {
    ui32SysClock = hal_mcu_getSysClock();

    //
    // Enable the SSI3 Peripheral.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    //
    // Configure GPIO Pins for SSI3 mode.
    //
    MAP_GPIOPinConfigure(GPIO_PF3_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PF1_SSI3XDAT0); // Tx
    MAP_GPIOPinConfigure(GPIO_PF0_SSI3XDAT1); // Rx
    MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

    SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
            SSI_MODE_MASTER, ui32_spiClock, 8);

    SSIEnable(SSI3_BASE);
    SSIIntRegister( SSI3_BASE, SSI3IntHandler );

    HAL_SPI_EMPTY_RX_FIFO(SSI3_BASE);

    memset((void *) &gs_ssi[3], 0, sizeof(s_hal_spi_ctx_t));
    ps_return = &gs_ssi[3];

    ps_return->ui32_baseAddr = SSI3_BASE;
    uDMAChannelAssign(UDMA_CH15_SSI3TX);
    ps_return->ui32_dmaTxChannel = 15;
    uDMAChannelAssign(UDMA_CH14_SSI3RX);
    ps_return->ui32_dmaRxChannel = 14;
    ps_return->ui32_interrupt = INT_SSI3;

    /* After startup none of the transfers are finished. */
    ps_return->ui8_dmaStatusFlags = 0;

  }

  return ps_return;
}


/* ===========================================================================*/
/*                  loc_spiTx()                                               */
/* ===========================================================================*/
void loc_spiTx(s_hal_spi_ctx_t *ps_ctx)
{
  //
  // Enable the uDMA interface for TX.
  //
  MAP_SSIDMAEnable(ps_ctx->ui32_baseAddr, SSI_DMA_TX);

  //
  // Put the attributes in a known state for the uDMA SSI0TX channel.  These
  // should already be disabled by default.
  //
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaTxChannel,
                                  UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK);

  //
  // Set the USEBURST attribute for the uDMA SSI0TX channel.  This will
  // force the controller to always use a burst when transferring data from
  // the TX buffer to the SSI0.  This is somewhat more effecient bus usage
  // than the default which allows single or burst transfers.
  //
  MAP_uDMAChannelAttributeEnable(ps_ctx->ui32_dmaTxChannel, UDMA_ATTR_USEBURST);

  //
  // Configure the control parameters for the SSI0 TX.
  //
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  //
  // Set up the transfer parameters for the uDMA SSI0 TX channel.  This will
  // configure the transfer source and destination and the transfer size.
  // Basic mode is used because the peripheral is making the uDMA transfer
  // request.  The source is the TX buffer and the destination is theUART0
  // data register.
  //
  MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   ps_ctx->pc_txBuf,
                                   (void *)(ps_ctx->ui32_baseAddr + SSI_O_DR),
                                   ps_ctx->ui16_len
                         );
  //
  // Now both the uDMA SSI0 TX and RX channels are primed to start a
  // transfer.  As soon as the channels are enabled, the peripheral will
  // issue a transfer request and the data transfers will begin.
  //
  MAP_uDMAChannelEnable(ps_ctx->ui32_dmaTxChannel);

  //
  // Enable the SSI0 DMA TX interrupts.
  //
  MAP_SSIIntEnable(ps_ctx->ui32_baseAddr, SSI_DMATX);

  //
  // Enable the SSI0 peripheral interrupts.
  //
  MAP_IntEnable(ps_ctx->ui32_interrupt);
}

/* ===========================================================================*/
/*                  loc_spiRx()                                               */
/* ===========================================================================*/
void loc_spiRx(s_hal_spi_ctx_t *ps_ctx)
{
  //
  // Enable the uDMA interface for both TX and RX channels.
  // TX to send dummy bytes!
  //
  MAP_SSIDMAEnable(ps_ctx->ui32_baseAddr, SSI_DMA_RX | SSI_DMA_TX);

  /* -------------------- Set RX DMA attributes ---------------------- */

  //
  // Put the attributes in a known state for the uDMA SSI0RX channel.  These
  // should already be disabled by default.
  //
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaRxChannel,
                                  UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                  (UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK));

  //
  // Configure the control parameters for the primary control structure for
  // the SSIORX channel.
  //
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaRxChannel | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                            UDMA_ARB_4);
  //
  // Set up the transfer parameters for the SSI0RX Channel
  //
  MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaRxChannel | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC,
                             (void *)(ps_ctx->ui32_baseAddr + SSI_O_DR),
                             ps_ctx->pc_rxBuf,
                             ps_ctx->ui16_len
                            );

  /* -------------------- Set TX DMA attributes ---------------------- */

  /* Configure the TX DMA to send the dummy byte. */
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaTxChannel,
                                  UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK);

  MAP_uDMAChannelAttributeEnable(ps_ctx->ui32_dmaTxChannel, UDMA_ATTR_USEBURST);

  /* When sending the dummy byte, the DMA source does not increment! */
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  /* Provide the dummy byte as source buffer. */
  MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   &gc_spiDummyByte,
                                   (void *)(ps_ctx->ui32_baseAddr + SSI_O_DR),
                                   ps_ctx->ui16_len
                         );

  //
  // Now both the uDMA SSI TX and RX channels are primed to start a
  // transfer.  As soon as the channels are enabled, the peripheral will
  // issue a transfer request and the data transfers will begin.
  //
  MAP_uDMAChannelEnable(ps_ctx->ui32_dmaRxChannel);
  MAP_uDMAChannelEnable(ps_ctx->ui32_dmaTxChannel);

  //
  // Enable the SSI DMA RX and TX interrupts.
  //
  MAP_SSIIntEnable(ps_ctx->ui32_baseAddr, SSI_DMARX | SSI_DMATX);

  //
  // Enable the SSI peripheral interrupts.
  //
  MAP_IntEnable(ps_ctx->ui32_interrupt);
}

/* ===========================================================================*/
/*                  loc_spiRxTx()                                             */
/* ===========================================================================*/
void loc_spiRxTx(s_hal_spi_ctx_t *ps_ctx)
{
  //
  // Enable the uDMA interface for both TX and RX channels.
  //
  MAP_SSIDMAEnable(ps_ctx->ui32_baseAddr, SSI_DMA_RX | SSI_DMA_TX);

  #if SSI_LOOPBACK_ENABLED
  //
  // This register write will set the SSI to operate in loopback mode.  Any
  // data sent on the TX output will be received on the RX input.
  //
  HWREG(ps_ctx->ui32_baseAddr + SSI_O_CR1) |= SSI_CR1_LBM;
  #endif


  /* -------------------- Set RX DMA attributes ---------------------- */

  //
  // Put the attributes in a known state for the uDMA SSI0RX channel.  These
  // should already be disabled by default.
  //
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaRxChannel,
                                  UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                  (UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK));

  //
  // Configure the control parameters for the primary control structure for
  // the SSIORX channel.
  //
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaRxChannel | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                            UDMA_ARB_4);
  //
  // Set up the transfer parameters for the SSI0RX Channel
  //
  MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaRxChannel | UDMA_PRI_SELECT,
                             UDMA_MODE_BASIC,
                             (void *)(ps_ctx->ui32_baseAddr + SSI_O_DR),
                             ps_ctx->pc_rxBuf,
                             ps_ctx->ui16_len
                            );

  /* -------------------- Set TX DMA attributes ---------------------- */

  /* Configure the TX DMA to send the dummy byte. */
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaTxChannel,
                                  UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK);

  MAP_uDMAChannelAttributeEnable(ps_ctx->ui32_dmaTxChannel, UDMA_ATTR_USEBURST);

  /* When sending the dummy byte, the DMA source does not increment! */
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  /* Provide the dummy byte as source buffer. */
  MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaTxChannel | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   ps_ctx->pc_txBuf,
                                   (void *)(ps_ctx->ui32_baseAddr + SSI_O_DR),
                                   ps_ctx->ui16_len
                         );

  //
  // Now both the uDMA SSI TX and RX channels are primed to start a
  // transfer.  As soon as the channels are enabled, the peripheral will
  // issue a transfer request and the data transfers will begin.
  //
  MAP_uDMAChannelEnable(ps_ctx->ui32_dmaRxChannel);
  MAP_uDMAChannelEnable(ps_ctx->ui32_dmaTxChannel);

  //
  // Enable the SSI DMA RX and TX interrupts.
  //
  MAP_SSIIntEnable(ps_ctx->ui32_baseAddr, SSI_DMARX | SSI_DMATX);

  //
  // Enable the SSI peripheral interrupts.
  //
  MAP_IntEnable(ps_ctx->ui32_interrupt);

  return;
}


/* ===========================================================================*/
/*                  hal_spi_init()                                            */
/* ===========================================================================*/
s_hal_spi_ctx_t *hal_spi_init(E_HAL_SPI_PORT_t e_port, E_HAL_SPI_MODE_t e_mode, uint32_t ui32_spiClock)
{
  s_hal_spi_ctx_t *ps_return = NULL;

  /* Currently only SPI master is supported! */
  if(e_mode == E_HAL_SPI_MODE_MASTER)
  {
    switch (e_port)
    {
      case E_HAL_SPI_PORT_0:
        ps_return = loc_initSSI0(ui32_spiClock);
        break;
      case E_HAL_SPI_PORT_1:
        ps_return = loc_initSSI1(ui32_spiClock);
        break;
      case E_HAL_SPI_PORT_2:
        ps_return = loc_initSSI2(ui32_spiClock);
        break;
      case E_HAL_SPI_PORT_3:
        ps_return = loc_initSSI3(ui32_spiClock);
        break;
      default:
        break;
    }
  }

  return ps_return;
}


/* ===========================================================================*/
/*                  hal_spi_config()                                          */
/* ===========================================================================*/
bool hal_spi_config(s_hal_spi_ctx_t *ps_ctx, E_HAL_SPI_CLK_POL_t e_pol,
            E_HAL_SPI_CLK_PHA_t e_phase, uint32_t ui32_spiClock)
{
  bool b_return = false;
  /* Default mode: SPO = 0; SPH = 0; */
  uint32_t ui32_protocol = SSI_FRF_MOTO_MODE_0;
  uint32_t ui32SysClock = 0;

  if(ps_ctx)
  {
    /* Disable SSI prior to do any configuration on it. */
    SSIDisable(ps_ctx->ui32_baseAddr);

    ui32SysClock = hal_mcu_getSysClock();

    if(e_pol   == E_HAL_SPI_CLK_POL_HIGH &&
       e_phase == E_HAL_SPI_CLK_PHA_EDGE_FIRST)
    {
      /* SPO = 1; SPH = 0; */
      ui32_protocol = SSI_FRF_MOTO_MODE_2;
    }
    else if(e_pol == E_HAL_SPI_CLK_POL_HIGH &&
            e_phase == E_HAL_SPI_CLK_PHA_EDGE_SCND )
    {
      /* SPO = 1; SPH = 1; */
      ui32_protocol = SSI_FRF_MOTO_MODE_3;
    }
    else if(e_pol == E_HAL_SPI_CLK_POL_LOW &&
            e_phase == E_HAL_SPI_CLK_PHA_EDGE_FIRST )
    {
      /* SPO = 0; SPH = 0; */
      ui32_protocol = SSI_FRF_MOTO_MODE_0;
    }
    else if(e_pol == E_HAL_SPI_CLK_POL_LOW &&
            e_phase == E_HAL_SPI_CLK_PHA_EDGE_SCND )
    {
      /* SPO = 0; SPH = 1; */
      ui32_protocol = SSI_FRF_MOTO_MODE_1;
    }

    SSIConfigSetExpClk(ps_ctx->ui32_baseAddr, ui32SysClock,
        ui32_protocol, SSI_MODE_MASTER, ui32_spiClock, 8);

    /* Enable SSI after configuration is done. */
    SSIEnable(ps_ctx->ui32_baseAddr);

    b_return = true;

  }/* if() */

  return b_return;
}


/* ===========================================================================*/
/*                  hal_spi_xfer()                                            */
/* ===========================================================================*/
void hal_spi_xfer(s_hal_spi_ctx_t *ps_ctx, uint8_t *pc_txData, uint8_t *pc_rxData, uint16_t ui16_len, pfn_spi_callback pf_cb)
{

  if(ps_ctx && pf_cb && ui16_len > 0)
  {
    ps_ctx->pc_txBuf = pc_txData;
    ps_ctx->pc_rxBuf = pc_rxData;
    ps_ctx->ui16_len = ui16_len;
    ps_ctx->pf_cb = pf_cb;

    if(pc_txData != NULL && pc_rxData != NULL)
    {
      /* Clear RX fifo in advance. */
      HAL_SPI_EMPTY_RX_FIFO(ps_ctx->ui32_baseAddr);
      /* TX + RX data */
      loc_spiRxTx(ps_ctx);
    }
    else if(pc_txData)
    {
      /* TX data */
      loc_spiTx(ps_ctx);
      /* If we TX only, the RX is implicitely finished already. */
      ps_ctx->ui8_dmaStatusFlags |= HAL_SPI_DMA_RX_STATUS_FIN;
    }
    else if(pc_rxData)
    {
      /* Clear RX fifo in advance. */
      HAL_SPI_EMPTY_RX_FIFO(ps_ctx->ui32_baseAddr);
      /* RX data */
      loc_spiRx(ps_ctx);
    }

  }

  return;
}


/* ===========================================================================*/
/*                  hal_spi_xferBlocking()                                    */
/* ===========================================================================*/
void hal_spi_xferBlocking(s_hal_spi_ctx_t *ps_ctx, uint8_t *pc_txData, uint8_t *pc_rxData, uint16_t ui16_len)
{
  uint16_t i = 0;
  uint32_t ui32_rxDummyBuf = 0;

  if(ps_ctx && ui16_len > 0)
  {
    /* Empty the RX FIFO before any SPI operation. */
    HAL_SPI_EMPTY_RX_FIFO(ps_ctx->ui32_baseAddr);

    #if SSI_LOOPBACK_ENABLED
    //
    // This register write will set the SSI to operate in loopback mode.  Any
    // data sent on the TX output will be received on the RX input.
    //
    HWREG(ps_ctx->ui32_baseAddr + SSI_O_CR1) |= SSI_CR1_LBM;
    #endif

    if(pc_txData != NULL && pc_rxData != NULL)
    {
      /* TX + RX data */

      for (i = 0; i < ui16_len; ++i)
      {

        //
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        //
        SSIDataPut(ps_ctx->ui32_baseAddr, (uint32_t) pc_txData[i] );

        //
        // Wait until SSI is done transferring all the data in the transmit FIFO.
        //
        while( SSIBusy(ps_ctx->ui32_baseAddr) );

        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        SSIDataGet(ps_ctx->ui32_baseAddr, &ui32_rxDummyBuf );

        /* Mask the 32bit RX value as we use the lowest 8bit only. */
        ui32_rxDummyBuf &= 0x000000FF;
        pc_rxData[i]  = (uint8_t) ui32_rxDummyBuf;

      }/* for() */

    }
    else if(pc_txData)
    {
      /* TX only */

      for (i = 0; i < ui16_len; ++i)
      {
        SSIDataPut(ps_ctx->ui32_baseAddr, (uint32_t) pc_txData[i] );

        while( SSIBusy(ps_ctx->ui32_baseAddr) );

        SSIDataGet(ps_ctx->ui32_baseAddr, (uint32_t *) &ui32_rxDummyBuf );

      }/* for() */

    }
    else if(pc_rxData)
    {
      /* RX only */

      for (i = 0; i < ui16_len; ++i)
      {

        SSIDataPut(ps_ctx->ui32_baseAddr, (uint32_t) gc_spiDummyByte );

        while( SSIBusy(ps_ctx->ui32_baseAddr) );

        SSIDataGet(ps_ctx->ui32_baseAddr, &ui32_rxDummyBuf );

        /* Mask the 32bit RX value as we use the lowest 8bit only. */
        ui32_rxDummyBuf &= 0x000000FF;
        pc_rxData[i]  = (uint8_t) ui32_rxDummyBuf;

      }/* for() */

    }
  }

  return;
}/* hal_spi_xferBlocking() */


/* ===========================================================================*/
/*                  loc_handleInterrupts()                                    */
/* ===========================================================================*/
void loc_handleInterrupts(s_hal_spi_ctx_t *ps_ctx)
{
  uint32_t ui32Status = 0;
  uint32_t ui32Mode = 0;

  ui32Status = MAP_SSIIntStatus(ps_ctx->ui32_baseAddr, true);

  MAP_SSIIntClear(ps_ctx->ui32_baseAddr, ui32Status);

  ui32Mode = MAP_uDMAChannelModeGet(ps_ctx->ui32_dmaRxChannel | UDMA_PRI_SELECT);

  if(ui32Mode == UDMA_MODE_STOP)
  {
    /* Signal new data available! */
    ps_ctx->ui8_dmaStatusFlags |= HAL_SPI_DMA_RX_STATUS_FIN;
  }

  if(!MAP_uDMAChannelIsEnabled(ps_ctx->ui32_dmaTxChannel))
  {
    /* TX finished! */
    ps_ctx->ui8_dmaStatusFlags |= HAL_SPI_DMA_TX_STATUS_FIN;
  }

  /* If both TX and RX transmissions are done we signal it to the caller. */
  if( (ps_ctx->ui8_dmaStatusFlags & HAL_SPI_DMA_TX_STATUS_FIN) &&
      (ps_ctx->ui8_dmaStatusFlags & HAL_SPI_DMA_RX_STATUS_FIN)  )
  {
    ps_ctx->pf_cb();
    /* Reset our flags right after. */
    ps_ctx->ui8_dmaStatusFlags = 0;

    MAP_SSIDMADisable(ps_ctx->ui32_baseAddr, SSI_DMA_RX | SSI_DMA_TX);

    //
    // Enable the SSI DMA RX and TX interrupts.
    //
    MAP_SSIIntDisable(ps_ctx->ui32_baseAddr, SSI_DMARX | SSI_DMATX);

    //
    // Enable the SSI peripheral interrupts.
    //
    MAP_IntDisable(ps_ctx->ui32_interrupt);
  }
}


//*****************************************************************************
//
// SSI0 Interrupt Handler
//
//*****************************************************************************
void
SSI0IntHandler(void)
{
  loc_handleInterrupts(&gs_ssi[0]);
}

//*****************************************************************************
//
// SSI1 Interrupt Handler
//
//*****************************************************************************
void
SSI1IntHandler(void)
{
  loc_handleInterrupts(&gs_ssi[1]);
}

//*****************************************************************************
//
// SSI2 Interrupt Handler
//
//*****************************************************************************
void
SSI2IntHandler(void)
{
  loc_handleInterrupts(&gs_ssi[2]);
}

//*****************************************************************************
//
// SSI3 Interrupt Handler
//
//*****************************************************************************
void
SSI3IntHandler(void)
{
  loc_handleInterrupts(&gs_ssi[3]);
}
