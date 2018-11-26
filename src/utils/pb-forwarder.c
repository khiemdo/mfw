/* Copyright (C) 2015-2017
 *
 * pb-forwarder.c
 *
 * Martin Dold         <martin.dold@gmx.net>
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

#include "pb-forwarder.h"

#include "sproto.h"

#include "hal_uart.h"
#include "hal_crc.h"
/* Include to access level shifter pin setting and enable it. */
#include "mfw-global-config.h"
#include "hal_gpio.h"


typedef bool (*pfnForward)(uint8_t *pc_data, uint16_t i_len);

pfnForward pfn_forwardData;

s_hal_uart_ctx_t *gps_uart1Listener;
s_sproto_ctx_t gs_sprotoListenerUart1;
uint8_t gac_uart1RxBuf[4096];

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static uint16_t loc_cbReadFromUart1(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_cbWriteToUart1(const uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbUart1RxDataRdy(uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbSerialToUart1Error(E_SERIAL_PROTOCOL_ERROR_t e_errType);
static uint16_t loc_cbSerialCrcCalculate(const uint8_t *pc_data, uint16_t ui16_len);


/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_cbReadFromUart1()                                     */
/* ===========================================================================*/
static uint16_t loc_cbReadFromUart1(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_read(gps_uart1Listener, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbWriteToUart1()                                      */
/* ===========================================================================*/
static uint16_t loc_cbWriteToUart1(const uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_write(gps_uart1Listener, (uint8_t *)pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbUart1RxDataRdy()                                    */
/* ===========================================================================*/
static void loc_cbUart1RxDataRdy(uint8_t *pc_data, uint16_t ui16_len)
{
  /* New data is available. Skip any checks but just forward it to GS. */
  pfn_forwardData( pc_data, ui16_len );

  return;
}

/* ===========================================================================*/
/*                  loc_cbSerialToUart1Error()                                */
/* ===========================================================================*/
static void loc_cbSerialToUart1Error(E_SERIAL_PROTOCOL_ERROR_t e_errType)
{
  switch (e_errType)
  {
    case E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID:
      /* TODO: implement me */
      break;
    case E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW:
      /* TODO: implement me */
      break;
    default:
      break;
  }
  return;
}

/* ===========================================================================*/
/*                  loc_cbSerialCrcCalculate()                                */
/* ===========================================================================*/
static uint16_t loc_cbSerialCrcCalculate(const uint8_t *pc_data, uint16_t ui16_len)
{
  /* This function can be used for all UART ports as it only computes CRCs. */
  return hal_crc_calculate(pc_data, ui16_len);
}


bool forwardProtobufToArm(uint8_t *pc_data, uint16_t i_len)
{
  bool b_return = false;

  /* Simply forward given data (that is actually protobuf message) to the GS. */
  if( sproto_sendFrame(&gs_sprotoListenerUart1, pc_data, i_len) )
  {
    b_return = true;
  }

  return b_return;
}


/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

bool pb_forwarder_init( bool (*pfnForward)(uint8_t *pc_data, uint16_t i_len) )
{
  bool b_return = false;

  /* 1st, set OE high for TXS0108 Level Shifter. */
  hal_gpio_setDir(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN, E_HAL_GPIO_DIR_OUT);
  hal_gpio_write(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN, MFW_LSHIFT_EN_PIN);

  if(pfnForward)
  {
    /* Store the function pointer given to forward data. */
    pfn_forwardData = pfnForward;

    /* Initialize the UART 1 to listen for data to be forwarded. */
    gps_uart1Listener = hal_uart_init( E_HAL_UART_PORT_1, 921600 );
    if(gps_uart1Listener != NULL)
    {
      b_return = true;
    }

    /* Init the serial protocol module used for reading from UART 1. */
    if( b_return )
    {
      b_return = sproto_init(&gs_sprotoListenerUart1,           /* context   */
          loc_cbWriteToUart1, loc_cbReadFromUart1, loc_cbUart1RxDataRdy,  /* callbacks */
          loc_cbSerialToUart1Error, loc_cbSerialCrcCalculate,
          gac_uart1RxBuf, sizeof(gac_uart1RxBuf) );           /* buffer    */
    }
  }

  return b_return;
}

void pb_forwarder_run(void)
{
  /* Run the serial protocol for UART 1. */
  while( sproto_run( &gs_sprotoListenerUart1 ) );

  return;
}
