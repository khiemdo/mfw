/* Copyright (C) 2015-2017
 *
 * hal_uart.c
 *
 * Martin Dold         <martin.dold@gmx.net>
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

#include "hal_uart.h"
#include "inc/hw_uart.h"
#include "driverlib/udma.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "hal_mcu.h"

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
#define UART_RX_DATA_BUF_SIZE   2048
#define UART_TX_DATA_BUF_SIZE   2048


#ifndef HAL_UART0_USE_DMA
/*! Compile time option to enable DMA operation for \b UART0. */
#define HAL_UART0_USE_DMA     1
#endif

#ifndef HAL_UART1_USE_DMA
/*! Compile time option to enable DMA operation for \b UART1. */
#define HAL_UART1_USE_DMA     1
#endif

#ifndef HAL_UART2_USE_DMA
/*! Compile time option to enable DMA operation for \b UART2. */
#define HAL_UART2_USE_DMA     1
#endif

#ifndef HAL_UART3_USE_DMA
/*! Compile time option to enable DMA operation for \b UART3. */
#define HAL_UART3_USE_DMA     1
#endif

#ifndef HAL_UART4_USE_DMA
/*! Compile time option to enable DMA operation for \b UART4. */
#define HAL_UART4_USE_DMA     1
#endif

#ifndef HAL_UART5_USE_DMA
/*! Compile time option to enable DMA operation for \b UART5. */
#define HAL_UART5_USE_DMA     1
#endif

#ifndef HAL_UART6_USE_DMA
/*! Compile time option to enable DMA operation for \b UART6.
 *
 * \warning UART6 is not capable of DMA operation as the DMA channel
 *          conflicts with DMA SPI0!
 */
#define HAL_UART6_USE_DMA     0
#endif
#if HAL_UART6_USE_DMA
#error UART6 is not capable of DMA operation as the DMA channel conflicts with DMA SPI0!
#endif

#ifndef HAL_UART7_USE_DMA
/*! Compile time option to enable DMA operation for \b UART7. */
#define HAL_UART7_USE_DMA     1
#endif

#define IS_BUF_EMPTY(x, y)   (x == y)
#define INVALID_DMA_CHANNEL   0xFFFFFFFF


/*! Structure defining the context of an UART port. */
struct S_HAL_UART_CTX_T
{
  uint32_t ui32_baseAddr;
  uint32_t ui32_dmaChannelTx;

  uint8_t volatile ac_rxData[UART_RX_DATA_BUF_SIZE];
  uint8_t volatile *pc_rxDataWrite;
  uint8_t volatile *pc_rxDataRead;

  uint8_t ac_txData[UART_TX_DATA_BUF_SIZE];
  uint8_t volatile *pc_txDataWrite;
  uint8_t volatile *pc_txDataRead;

  uint32_t volatile txDataTransmitCount;
  bool volatile txEnabled;
};



/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
s_hal_uart_ctx_t gs_uart0;
s_hal_uart_ctx_t gs_uart1;
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
s_hal_uart_ctx_t gs_uart2;
s_hal_uart_ctx_t gs_uart3;
s_hal_uart_ctx_t gs_uart4;
s_hal_uart_ctx_t gs_uart5;
s_hal_uart_ctx_t gs_uart6;
s_hal_uart_ctx_t gs_uart7;
#endif
/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static uint32_t loc_countTxDataBytesLinear(s_hal_uart_ctx_t *ps_ctx);
static void     loc_incrementBy(s_hal_uart_ctx_t *ps_ctx);
static bool     loc_isRxBufFull(s_hal_uart_ctx_t *ps_ctx);
static bool     loc_isTxBufFull(s_hal_uart_ctx_t *ps_ctx);
static bool     loc_writeToTxFIFO(s_hal_uart_ctx_t *ps_ctx);
static bool     loc_readByteFromBuf(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_byte);
#if !( HAL_UART0_USE_DMA & HAL_UART1_USE_DMA & HAL_UART2_USE_DMA & HAL_UART3_USE_DMA &  \
       HAL_UART4_USE_DMA & HAL_UART5_USE_DMA & HAL_UART7_USE_DMA )
static void     loc_handleInterrupt(s_hal_uart_ctx_t *ps_ctx);
#endif
#if( HAL_UART0_USE_DMA | HAL_UART1_USE_DMA | HAL_UART2_USE_DMA | HAL_UART3_USE_DMA |  \
     HAL_UART4_USE_DMA | HAL_UART5_USE_DMA | HAL_UART7_USE_DMA )
static void     loc_handleDmaInterrupt(s_hal_uart_ctx_t *ps_ctx);
static void     loc_initDmaTx(s_hal_uart_ctx_t *ps_ctx, uint32_t ui32_dmaChanAssignTx);
#endif
static void     loc_initUartStruct(s_hal_uart_ctx_t *ps_ctx, uint32_t ui32_baseAddr, uint32_t ui32_dmaChannelTx);
static s_hal_uart_ctx_t *loc_initUart0(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t *loc_initUart1(uint32_t ui32_sysClock, uint32_t ui32_baudRate);

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
static s_hal_uart_ctx_t* loc_initUart2(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t* loc_initUart3(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t* loc_initUart4(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t* loc_initUart5(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t* loc_initUart6(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
static s_hal_uart_ctx_t* loc_initUart7(uint32_t ui32_sysClock, uint32_t ui32_baudRate);
#endif

/* Prototypes of UART interrupt handlers that are registered in corresponding
 * init() functions, e.g. loc_initUart0(). */
static void UART0IntHandler(void);
static void UART1IntHandler(void);

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
static void UART2IntHandler(void);
static void UART3IntHandler(void);
static void UART4IntHandler(void);
static void UART5IntHandler(void);
static void UART6IntHandler(void);
static void UART7IntHandler(void);
#endif

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

static uint32_t loc_countTxDataBytesLinear(s_hal_uart_ctx_t *ps_ctx)
{
  uint32_t l_return = 0;

  if( IS_BUF_EMPTY(ps_ctx->pc_txDataRead, ps_ctx->pc_txDataWrite) )
  {
    l_return = 0;
  }
  else if( ps_ctx->pc_txDataRead < ps_ctx->pc_txDataWrite)
  {
    l_return = ps_ctx->pc_txDataWrite - ps_ctx->pc_txDataRead;
  }
  else
  {
    l_return = &(ps_ctx->ac_txData[UART_TX_DATA_BUF_SIZE-1]) - ps_ctx->pc_txDataRead;
  }

  return l_return;
}

static void loc_incrementBy(s_hal_uart_ctx_t *ps_ctx)
{
  ps_ctx->pc_txDataRead += ps_ctx->txDataTransmitCount;
  ps_ctx->txDataTransmitCount = 0;

  /* Ring buf! Therefore check and do wrap around if required. */
  if( ps_ctx->pc_txDataRead == &ps_ctx->ac_txData[UART_TX_DATA_BUF_SIZE-1] )
  {
    ps_ctx->pc_txDataRead = ps_ctx->ac_txData;
  }
}

static bool loc_isRxBufFull(s_hal_uart_ctx_t *ps_ctx)
{
  bool b_return = false;

  /* buf is full if read pointer is 1 byte ahead the write pointer. */
  if( (ps_ctx->pc_rxDataWrite + 1) == ps_ctx->pc_rxDataRead)
  {
    b_return = true;
  }
  /* Due to the ringbuf it might be, that the read pointer was already wrapped around.
   * We must check with "SIZE-2" here, as the write pointer will wrap around at
   * "SIZE-1" already and thereby point to start again. */
  else if( (ps_ctx->pc_rxDataWrite == &(ps_ctx->ac_rxData[UART_RX_DATA_BUF_SIZE-2]) ) &&
           (ps_ctx->pc_rxDataRead  == &(ps_ctx->ac_rxData[0]) ) )
  {
    b_return = true;
  }

  return b_return;
}

static bool loc_isTxBufFull(s_hal_uart_ctx_t *ps_ctx)
{
  bool b_return = false;

  /* buf is full if read pointer is 1 byte ahead the write pointer. */
  if( (ps_ctx->pc_txDataWrite + 1) == ps_ctx->pc_txDataRead)
  {
	  b_return = true;
  }
  /* Due to the ringbuf it might be, that the read pointer was already wrapped around.
   * We must check with "SIZE-2" here, as the write pointer will wrap around at
   * "SIZE-1" already and thereby point to start again. */
  else if( (ps_ctx->pc_txDataWrite == &(ps_ctx->ac_txData[UART_TX_DATA_BUF_SIZE-2]) ) &&
		   (ps_ctx->pc_txDataRead  == &(ps_ctx->ac_txData[0]) ) )
  {
	  b_return = true;
  }

  return b_return;
}

static bool loc_writeToTxFIFO(s_hal_uart_ctx_t *ps_ctx)
{
  bool b_return = false;

  /* Verify that space if left in the MCU TX FIFO and our buffer is not empty */
  while( MAP_UARTSpaceAvail(ps_ctx->ui32_baseAddr) &&
         !IS_BUF_EMPTY(ps_ctx->pc_txDataWrite, ps_ctx->pc_txDataRead) )
  {
    MAP_UARTCharPutNonBlocking(ps_ctx->ui32_baseAddr, *ps_ctx->pc_txDataRead++);
    b_return = true;

    /* Ring buf! Therefore check and do wrap around if required. */
    if( ps_ctx->pc_txDataRead == &ps_ctx->ac_txData[UART_TX_DATA_BUF_SIZE-1] )
    {
      ps_ctx->pc_txDataRead = ps_ctx->ac_txData;
    }
  }/* while() */

  return b_return;
}

static bool loc_readByteFromBuf(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_byte)
{
  bool b_return = false;

  if( !IS_BUF_EMPTY(ps_ctx->pc_rxDataWrite, ps_ctx->pc_rxDataRead) )
  {
    *pc_byte = *ps_ctx->pc_rxDataRead++;

    /* Ring buf! Therefore check and do wrap around if required. */
    if( ps_ctx->pc_rxDataRead == &ps_ctx->ac_rxData[UART_RX_DATA_BUF_SIZE-1] )
    {
      ps_ctx->pc_rxDataRead = ps_ctx->ac_rxData;
    }

    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_handleInterrupt()                                     */
/* ===========================================================================*/
#if !( HAL_UART0_USE_DMA & HAL_UART1_USE_DMA & HAL_UART2_USE_DMA & HAL_UART3_USE_DMA &  \
       HAL_UART4_USE_DMA & HAL_UART5_USE_DMA & HAL_UART7_USE_DMA )
static void loc_handleInterrupt(s_hal_uart_ctx_t *ps_ctx)
{
  uint32_t ui32Status;

  //
  // Get the interrrupt status.
  //
  ui32Status = MAP_UARTIntStatus(ps_ctx->ui32_baseAddr, true);

  //
  // Clear the asserted interrupts.
  //
  MAP_UARTIntClear(ps_ctx->ui32_baseAddr, ui32Status);

  if( (ui32Status & UART_INT_RX) || (ui32Status & UART_INT_RT) )
  {
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(MAP_UARTCharsAvail(ps_ctx->ui32_baseAddr))
    {
      /* Only write into RX ringbuf if not full! Otherwise, the writePointer
       * will overrun readPointer and hal_uart_read() may return old/outdated
       * data again. This is seen more critical than missing/ommit the bytes we
       * can't store anymore.
       */
      if( !loc_isRxBufFull(ps_ctx) )
      {
        *ps_ctx->pc_rxDataWrite++ = MAP_UARTCharGetNonBlocking(ps_ctx->ui32_baseAddr);

        /* Ring buf! Therefore check and do wrap around if required. */
        if( ps_ctx->pc_rxDataWrite == &ps_ctx->ac_rxData[UART_RX_DATA_BUF_SIZE-1] )
        {
          ps_ctx->pc_rxDataWrite = ps_ctx->ac_rxData;
        }
      }
      else
      {
        /* Break the while() here to not stay in ISR for inifinity. */
        break;
      }
    }
  }

  /* Check if TX interrupt occurred. */
  if(ui32Status & UART_INT_TX)
  {
    if( IS_BUF_EMPTY(ps_ctx->pc_txDataWrite, ps_ctx->pc_txDataRead) )
    {
      /* No further data to send so disable TX interrupt. */
      MAP_UARTIntDisable(ps_ctx->ui32_baseAddr, UART_INT_TX);
    }
    else
    {
      /* We still have data to write through the UART. */
      loc_writeToTxFIFO(ps_ctx);
    }/* if() ... else */
  }

  return;
}
#endif

/* ===========================================================================*/
/*                  loc_handleDmaInterrupt()                                  */
/* ===========================================================================*/
#if( HAL_UART0_USE_DMA | HAL_UART1_USE_DMA | HAL_UART2_USE_DMA | HAL_UART3_USE_DMA |  \
     HAL_UART4_USE_DMA | HAL_UART5_USE_DMA | HAL_UART7_USE_DMA )
static void loc_handleDmaInterrupt(s_hal_uart_ctx_t *ps_ctx)
{
  uint32_t ui32Status = 0;

  //
  // Get the interrrupt status.
  //
  ui32Status = MAP_UARTIntStatus(ps_ctx->ui32_baseAddr, true);

  //
  // Clear the asserted interrupts.
  //
  MAP_UARTIntClear(ps_ctx->ui32_baseAddr, ui32Status);

  if( (ui32Status & UART_INT_RX) || (ui32Status & UART_INT_RT) )
  {
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(MAP_UARTCharsAvail(ps_ctx->ui32_baseAddr))
    {
      /* Only write into RX ringbuf if not full! Otherwise, the writePointer
       * will overrun readPointer and hal_uart_read() may return old/outdated
       * data again. This is seen more critical than missing/ommit the bytes we
       * can't store anymore.
       */
      if( !loc_isRxBufFull(ps_ctx) )
      {
        *ps_ctx->pc_rxDataWrite++ = MAP_UARTCharGetNonBlocking(ps_ctx->ui32_baseAddr);

        /* Ring buf! Therefore check and do wrap around if required. */
        if( ps_ctx->pc_rxDataWrite == &ps_ctx->ac_rxData[UART_RX_DATA_BUF_SIZE-1] )
        {
          ps_ctx->pc_rxDataWrite = ps_ctx->ac_rxData;
        }
      }
      else
      {
        /* Break the while() here to not stay in ISR for inifinity. */
        break;
      }
    }
  }

  //
  // If the UART1 DMA TX channel is disabled, that means the TX DMA transfer
  // is done.
  //
  if( ui32Status & UART_INT_DMATX )
  {
    if(!MAP_uDMAChannelIsEnabled(ps_ctx->ui32_dmaChannelTx))
    {
      /* Increment pointers first. */
      loc_incrementBy(ps_ctx);
      ps_ctx->txDataTransmitCount = loc_countTxDataBytesLinear(ps_ctx);

      if( ps_ctx->txDataTransmitCount )
      {
        //
        // Start another DMA transfer to UART TX.
        //
        MAP_uDMAChannelTransferSet(ps_ctx->ui32_dmaChannelTx | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, (void *)ps_ctx->pc_txDataRead,
                                   (void *)(ps_ctx->ui32_baseAddr + UART_O_DR),
                                   ps_ctx->txDataTransmitCount
                                   );

        //
        // The uDMA TX channel must be re-enabled.
        //
        MAP_uDMAChannelEnable(ps_ctx->ui32_dmaChannelTx);
      }
      else
      {
        ps_ctx->txEnabled = false;
        //
        // Disable the UART DMA TX interrupt.
        //
        MAP_UARTIntDisable(ps_ctx->ui32_baseAddr, UART_INT_DMATX );
      }
    }
  }

  return;
}
#endif

/* ===========================================================================*/
/*                  loc_initDmaTx()                                           */
/* ===========================================================================*/
#if( HAL_UART0_USE_DMA | HAL_UART1_USE_DMA | HAL_UART2_USE_DMA | HAL_UART3_USE_DMA |  \
     HAL_UART4_USE_DMA | HAL_UART5_USE_DMA | HAL_UART7_USE_DMA )
static void loc_initDmaTx(s_hal_uart_ctx_t *ps_ctx, uint32_t ui32_dmaChanAssignTx)
{
  //
  // Enable the UART for operation, and enable the uDMA interface for TX.
  //
  MAP_UARTEnable(ps_ctx->ui32_baseAddr);
  MAP_UARTDMAEnable(ps_ctx->ui32_baseAddr, UART_DMA_TX);
  uDMAChannelEnable(ps_ctx->ui32_dmaChannelTx);
  uDMAChannelAssign(ui32_dmaChanAssignTx);

  //
  // Put the attributes in a known state for the uDMA UART1TX channel.  These
  // should already be disabled by default.
  //
  MAP_uDMAChannelAttributeDisable(ps_ctx->ui32_dmaChannelTx,
                                  UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_HIGH_PRIORITY |
                                  UDMA_ATTR_REQMASK);

  //
  // Set the USEBURST attribute for the uDMA UART TX channel.  This will
  // force the controller to always use a burst when transferring data from
  // the TX buffer to the UART.  This is somewhat more effecient bus usage
  // than the default which allows single or burst transfers.
  //
  MAP_uDMAChannelAttributeEnable(ps_ctx->ui32_dmaChannelTx, UDMA_ATTR_USEBURST);

  //
  // Configure the control parameters for the UART TX.  The uDMA UART TX
  // channel is used to transfer a block of data from a buffer to the UART.
  // The data size is 8 bits.  The source address increment is 8-bit bytes
  // since the data is coming from a buffer.  The destination increment is
  // none since the data is to be written to the UART data register.  The
  // arbitration size is set to 4, which matches the UART TX FIFO trigger
  // threshold.
  //
  MAP_uDMAChannelControlSet(ps_ctx->ui32_dmaChannelTx | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                            UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  return;
}
#endif

/* ===========================================================================*/
/*                  loc_initUartStruct()                                      */
/* ===========================================================================*/
static void loc_initUartStruct(s_hal_uart_ctx_t *ps_ctx, uint32_t ui32_baseAddr, uint32_t ui32_dmaChannelTx)
{
  memset( ps_ctx, 0, sizeof(s_hal_uart_ctx_t) );
  ps_ctx->ui32_baseAddr = ui32_baseAddr;
  ps_ctx->pc_rxDataRead = ps_ctx->ac_rxData;
  ps_ctx->pc_rxDataWrite = ps_ctx->ac_rxData;
  ps_ctx->pc_txDataRead = ps_ctx->ac_txData;
  ps_ctx->pc_txDataWrite = ps_ctx->ac_txData;
  /* Invalid DMA channels are stored too. Required for check in hal_uart_write(). */
  ps_ctx->ui32_dmaChannelTx = ui32_dmaChannelTx;
  ps_ctx->txEnabled = false;
  return;
}

/* ===========================================================================*/
/*                  loc_initUart0()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart0(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Set GPIO A0 and A1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART0_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART0_USE_DMA
  loc_initUartStruct( &gs_uart0, UART0_BASE, UDMA_CH9_UART0TX );
  loc_initDmaTx(&gs_uart0, UDMA_CH9_UART0TX);
  #else
  loc_initUartStruct( &gs_uart0, UART0_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART0_BASE, UART0IntHandler );
  MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART0);

  return &gs_uart0;
}

/* ===========================================================================*/
/*                  loc_initUart1()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart1(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  //
  // Set GPIO PB0 and PB1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinConfigure(GPIO_PB1_U1TX);
  MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART1_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART1_USE_DMA
  loc_initUartStruct( &gs_uart1, UART1_BASE, UDMA_CH23_UART1TX );
  loc_initDmaTx(&gs_uart1, UDMA_CH23_UART1TX);
  #else
  loc_initUartStruct( &gs_uart1, UART1_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART1_BASE, UART1IntHandler );
  MAP_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART1_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART1);

  return &gs_uart1;
}

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
/* ===========================================================================*/
/*                  loc_initUart2()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart2(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

  //
  // Set GPIO D4 and D5 as UART pins.
  //
  GPIOPinConfigure(GPIO_PD4_U2RX);
  GPIOPinConfigure(GPIO_PD5_U2TX);
  MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART2_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART2_USE_DMA
  loc_initUartStruct( &gs_uart2, UART2_BASE, UDMA_CH1_UART2TX );
  loc_initDmaTx(&gs_uart2, UDMA_CH1_UART2TX);
  #else
  loc_initUartStruct( &gs_uart2, UART2_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART2_BASE, UART2IntHandler );
  MAP_UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART2_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART2);

  return &gs_uart2;
}

/* ===========================================================================*/
/*                  loc_initUart3()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart3(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

  //
  // Set GPIO J0 and J1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PJ0_U3RX);
  GPIOPinConfigure(GPIO_PJ1_U3TX);
  MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART3_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART3_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART3_USE_DMA
  loc_initUartStruct( &gs_uart3, UART3_BASE, UDMA_CH17_UART3TX );
  loc_initDmaTx(&gs_uart3, UDMA_CH17_UART3TX);
  #else
  loc_initUartStruct( &gs_uart3, UART3_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART3_BASE, UART3IntHandler );
  MAP_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART3_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART3);

  return &gs_uart3;
}


/* ===========================================================================*/
/*                  loc_initUart4()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart4(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

  //
  // Set GPIO K0 and K1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PK0_U4RX);
  GPIOPinConfigure(GPIO_PK1_U4TX);
  MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART4_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART4_USE_DMA
  loc_initUartStruct( &gs_uart4, UART4_BASE, UDMA_CH19_UART4TX );
  loc_initDmaTx(&gs_uart4, UDMA_CH19_UART4TX);
  #else
  loc_initUartStruct( &gs_uart4, UART4_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART4_BASE, UART4IntHandler );
  MAP_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART4_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART4);

  return &gs_uart4;
}

/* ===========================================================================*/
/*                  loc_initUart5()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart5(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  //
  // Set GPIO PC60 and PC7 as UART pins.
  //
  GPIOPinConfigure(GPIO_PC6_U5RX);
  GPIOPinConfigure(GPIO_PC7_U5TX);
  MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART5_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART5_USE_DMA
  loc_initUartStruct( &gs_uart5, UART5_BASE, UDMA_CH7_UART5TX );
  loc_initDmaTx(&gs_uart5, UDMA_CH7_UART5TX);
  #else
  loc_initUartStruct( &gs_uart5, UART5_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART5_BASE, UART5IntHandler );
  MAP_UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART5_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART5);

  return &gs_uart5;
}


/* ===========================================================================*/
/*                  loc_initUart6()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart6(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

  //
  // Set GPIO J0 and J1 as UART pins.
  //
  GPIOPinConfigure(GPIO_PP0_U6RX);
  GPIOPinConfigure(GPIO_PP1_U6TX);
  MAP_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART6_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  /* Initialize our context struct. */
  memset( &gs_uart6, 0, sizeof(gs_uart6) );
  gs_uart6.ui32_baseAddr = UART6_BASE;
  gs_uart6.pc_rxDataRead  = gs_uart6.ac_rxData;
  gs_uart6.pc_rxDataWrite = gs_uart6.ac_rxData;
  gs_uart6.pc_txDataRead  = gs_uart6.ac_txData;
  gs_uart6.pc_txDataWrite = gs_uart6.ac_txData;

  MAP_UARTFIFOLevelSet(UART6_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART6_BASE, UART6IntHandler );
  MAP_IntEnable(INT_UART6);
  MAP_UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART6_BASE, UART_INT_TX);

  return &gs_uart6;
}

/* ===========================================================================*/
/*                  loc_initUart7()                                           */
/* ===========================================================================*/
static s_hal_uart_ctx_t* loc_initUart7(uint32_t ui32_sysClock, uint32_t ui32_baudRate)
{
  //
  // Enable the peripherals used by this example.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

  //
  // Set GPIO C4 and C5 as UART pins.
  //
  GPIOPinConfigure(GPIO_PC4_U7RX);
  GPIOPinConfigure(GPIO_PC5_U7TX);
  MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

  //
  // Configure the UART for 8-N-1 operation with user defined baud rate.
  //
  MAP_UARTConfigSetExpClk(UART7_BASE, ui32_sysClock, ui32_baudRate,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  MAP_UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

  /* Initialize our context struct. */
  #if HAL_UART7_USE_DMA
  loc_initUartStruct( &gs_uart7, UART7_BASE, UDMA_CH21_UART7TX );
  loc_initDmaTx(&gs_uart7, UDMA_CH21_UART7TX);
  #else
  loc_initUartStruct( &gs_uart7, UART7_BASE, INVALID_DMA_CHANNEL );
  #endif

  //
  // Enable the UART interrupt.
  //
  UARTIntRegister( UART7_BASE, UART7IntHandler );
  MAP_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
  MAP_UARTIntDisable(UART7_BASE, UART_INT_TX);
  MAP_IntEnable(INT_UART7);

  return &gs_uart7;
}
#endif
/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  hal_uart_init()                                           */
/* ===========================================================================*/
s_hal_uart_ctx_t *hal_uart_init(E_HAL_UART_PORT_t e_port, uint32_t ui32_baudRate)
{
  s_hal_uart_ctx_t *ps_return = NULL;

  /* Get the System clock (settings). */
  uint32_t ui32SysClock = hal_mcu_getSysClock();

  switch (e_port)
  {
    case E_HAL_UART_PORT_0:
      ps_return = loc_initUart0(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_1:
      ps_return = loc_initUart1(ui32SysClock, ui32_baudRate);
      break;
    #if defined(TARGET_IS_TM4C129_RA0) ||                                         \
        defined(TARGET_IS_TM4C129_RA1) ||                                         \
        defined(TARGET_IS_TM4C129_RA2)
    case E_HAL_UART_PORT_2:
      ps_return = loc_initUart2(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_3:
      ps_return = loc_initUart3(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_4:
      ps_return = loc_initUart4(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_5:
      ps_return = loc_initUart5(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_6:
      ps_return = loc_initUart6(ui32SysClock, ui32_baudRate);
      break;

    case E_HAL_UART_PORT_7:
      ps_return = loc_initUart7(ui32SysClock, ui32_baudRate);
      break;
    #endif
    /* Other UART ports are not supported => ps_return remains NULL. */
    default:
      break;
  }

  return ps_return;
}

/* ===========================================================================*/
/*                  hal_uart_read()                                           */
/* ===========================================================================*/
uint16_t hal_uart_read(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_data, uint16_t dataLen)
{
  uint16_t i_ret = 0;

  for (i_ret = 0; i_ret < dataLen; ++i_ret)
  {
    if( !loc_readByteFromBuf(ps_ctx, &pc_data[i_ret] ) )
    {
      /* No further data to read! */
      break;
    }
  }

  return i_ret;
}


/* ===========================================================================*/
/*                  hal_uart_write()                                          */
/* ===========================================================================*/
uint16_t hal_uart_write(s_hal_uart_ctx_t *ps_ctx, uint8_t *pc_data, uint16_t dataLen)
{
  uint16_t i_ret = 0;

  if( ps_ctx && pc_data && (dataLen > 0) )
  {
    for (i_ret = 0; i_ret < dataLen; ++i_ret)
    {
      if(false == loc_isTxBufFull(ps_ctx))
      {
       *ps_ctx->pc_txDataWrite++ = *pc_data++;

       /* Ring buf! Therefore check and do wrap around if required. */
       if( ps_ctx->pc_txDataWrite == &ps_ctx->ac_txData[UART_TX_DATA_BUF_SIZE-1] )
       {
         ps_ctx->pc_txDataWrite = ps_ctx->ac_txData;
       }
      }
      else
      {
       break; /* No further space left in TX buffer. */
      }
    }/* for() */

    if( ps_ctx->ui32_dmaChannelTx != INVALID_DMA_CHANNEL)
    {
      /* Make sure we wrote data to the TX buffer and that currently no
      * TX transmission is ongoing. */
      if( (i_ret > 0) && !ps_ctx->txEnabled )
      {
        ps_ctx->txEnabled = true;

        //
        // Enable the UART DMA TX interrupt.
        //
        MAP_UARTIntEnable(ps_ctx->ui32_baseAddr, UART_INT_DMATX );
      }
    }
    else
    {
      /*
       * This UART port does not use DMA!
       */
      if( (i_ret > 0) && ( !UARTBusy(ps_ctx->ui32_baseAddr) ) )
      {

        if( loc_writeToTxFIFO(ps_ctx) )
        {
          /* We need to trigger the TX ISR in case we wrote new data to the TX ring buf. */
          MAP_UARTIntEnable(ps_ctx->ui32_baseAddr, UART_INT_TX);
        }
      }
    }
  }

  return i_ret;
}


//*****************************************************************************
//
// The UART0 interrupt handler.
//
//*****************************************************************************
void
UART0IntHandler(void)
{
#if HAL_UART0_USE_DMA
  loc_handleDmaInterrupt(&gs_uart0);
#else
  loc_handleInterrupt(&gs_uart0);
#endif
}

//*****************************************************************************
//
// The UART1 interrupt handler.
//
//*****************************************************************************
void
UART1IntHandler(void)
{
#if HAL_UART1_USE_DMA
  loc_handleDmaInterrupt(&gs_uart1);
#else
  loc_handleInterrupt(&gs_uart1);
#endif
}

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
//*****************************************************************************
//
// The UART2 interrupt handler.
//
//*****************************************************************************
void
UART2IntHandler(void)
{
  #if HAL_UART2_USE_DMA
  loc_handleDmaInterrupt(&gs_uart2);
  #else
  loc_handleInterrupt(&gs_uart2);
  #endif
}

//*****************************************************************************
//
// The UART3 interrupt handler.
//
//*****************************************************************************
void
UART3IntHandler(void)
{
  #if HAL_UART3_USE_DMA
  loc_handleDmaInterrupt(&gs_uart3);
  #else
  loc_handleInterrupt(&gs_uart3);
  #endif
}

//*****************************************************************************
//
// The UART4 interrupt handler.
//
//*****************************************************************************
void
UART4IntHandler(void)
{
  #if HAL_UART4_USE_DMA
  loc_handleDmaInterrupt(&gs_uart4);
  #else
  loc_handleInterrupt(&gs_uart4);
  #endif
}

//*****************************************************************************
//
// The UART5 interrupt handler.
//
//*****************************************************************************
void
UART5IntHandler(void)
{
  #if HAL_UART5_USE_DMA
  loc_handleDmaInterrupt(&gs_uart5);
  #else
  loc_handleInterrupt(&gs_uart5);
  #endif
}

//*****************************************************************************
//
// The UART6 interrupt handler.
//
//*****************************************************************************
void
UART6IntHandler(void)
{
  loc_handleDmaInterrupt(&gs_uart6);
}

//*****************************************************************************
//
// The UART7 interrupt handler.
//
//*****************************************************************************
void
UART7IntHandler(void)
{
  #if HAL_UART7_USE_DMA
  loc_handleDmaInterrupt(&gs_uart7);
  #else
  loc_handleInterrupt(&gs_uart7);
  #endif
}
#endif
