/* Copyright (C) 2015-2017
 *
 * test-app-simple-nanopb.c
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_uart.h"
#include "hal_timer.h"
#include "hal_mcu.h"

#include "sproto.h"


#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "simple.pb.h"

#ifndef TEST_APP_USE_SERIAL_PROTOCOL
/* By default: disable the serial protocol and access the HAL UART API directly. */
#define TEST_APP_USE_SERIAL_PROTOCOL   0
#endif

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static void loc_serialRxCallback(uint8_t *pc_data, uint16_t ui16_len);

#if !TEST_APP_USE_SERIAL_PROTOCOL
static void loc_simpleNanopbTestRun(void);
pb_ostream_t pb_ostream_from_uart(s_hal_uart_ctx_t *ps_uart);
pb_istream_t pb_istream_from_uart(s_hal_uart_ctx_t *ps_uart);
static bool write_callback(pb_ostream_t *stream, const uint8_t *buf, size_t count);
static bool read_callback(pb_istream_t *stream, uint8_t *buf, size_t count);

#else

static void loc_simpleNanopbTestViaSerialProtocolRun(void);
pb_ostream_t pb_ostream_from_serial(s_serial_protocol_ctx_t *ps_serial);
static bool write_serialCallback(pb_ostream_t *stream, const uint8_t *buf, size_t count);
#endif

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
#if !TEST_APP_USE_SERIAL_PROTOCOL
/*! Defines the UART port to be tested. Must be of type \ref E_HAL_UART_PORT_t. */
#define TEST_APP_UART_PORT          E_HAL_UART_PORT_0

/*! Defnies the baud rate to be used for the UART port under test. */
#define TEST_APP_UART_BAUD_RATE     115200
#endif

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */


/* ===========================================================================*/
/*                  Global variables                                          */
/* ===========================================================================*/
#if TEST_APP_USE_SERIAL_PROTOCOL
s_serial_protocol_ctx_t gs_serial;
#else
s_hal_uart_ctx_t *gps_uart;
#endif

s_serial_protocol_ctx_t gs_serial;
static uint32_t errorCount = 0;
static uint32_t successCount = 0;

/* ===========================================================================*/
/*                  FUNCTION IMPLEMENTATIONS                                  */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

#if !TEST_APP_USE_SERIAL_PROTOCOL
  if(b_return)
  {
    gps_uart = hal_uart_init(TEST_APP_UART_PORT, TEST_APP_UART_BAUD_RATE);
    if(gps_uart == NULL)
    {
      b_return = false;
    }
  }
#endif

  if(b_return)
  {
    b_return = hal_timer_init();
  }

  return b_return;
}/* loc_hal_init() */



void loc_serialRxCallback(uint8_t *pc_data, uint16_t ui16_len)
{
  if(pc_data[0] == 0 && pc_data[1] == 1 && pc_data[2] == 2 )
  {
    successCount++;
  }

  return;
}


/* ===========================================================================*/
/*                  loc_simpleNanopbTestRun()                                 */
/* ===========================================================================*/
#if !TEST_APP_USE_SERIAL_PROTOCOL
static void loc_simpleNanopbTestRun(void)
{
  size_t message_length;
  bool status;

//  /* Encode our message */
//  {
//      /* Allocate space on the stack to store the message data.
//       *
//       * Nanopb generates simple struct definitions for all the messages.
//       * - check out the contents of simple.pb.h!
//       * It is a good idea to always initialize your structures
//       * so that you do not have garbage data from RAM in there.
//       */
//      SimpleMessage message = SimpleMessage_init_zero;
//
//      /* Create a stream that will write to our buffer. */
//      pb_ostream_t ostream = pb_ostream_from_uart(gps_uart);
//
//      /* Fill in the lucky number */
////      message.lucky_number = 13;
//      message.lucky_number = 0xFFEEFFEE;
//
//      /* Now we are ready to encode the message! */
//      status = pb_encode_delimited(&ostream, SimpleMessage_fields, &message);
//      message_length = ostream.bytes_written;
//
//      /* Then just check for any errors.. */
//      if (!status)
//      {
//          /* printf("Encoding failed: %s\n", PB_GET_ERROR(&ostream)); */
//      }
//  }

  /* Now we could transmit the message over network, store it in a file or
   * wrap it to a pigeon's leg.
   */

  pb_istream_t istream;

  /* But because we are lazy, we will just decode it immediately. */

  {
      /* Allocate space for the decoded message. */
      SimpleMessage messageRx = SimpleMessage_init_zero;

      /* Create a stream that reads from the buffer. */
      istream = pb_istream_from_uart(gps_uart);

      /* Now we are ready to decode the message. */
      status = pb_decode_delimited(&istream, SimpleMessage_fields, &messageRx);

      /* Check for errors... */
      if (!status)
      {
        errorCount++;
      }
      else
      {
        successCount++;
      }

      /* Print the data contained in the message. */
      /* printf("Your lucky number was %d!\n", message.lucky_number); */
  }

  uint32_t delayTimer = 0;
  delayTimer = hal_timer_getTimeout(5000);
  while( !hal_timer_isTimedOut(delayTimer) );

  return;

}/* loc_simpleNanopbTestRun() */


pb_ostream_t pb_ostream_from_uart(s_hal_uart_ctx_t *ps_uart)
{
    pb_ostream_t stream = {&write_callback, (void*)ps_uart, SIZE_MAX, 0};
    return stream;
}

pb_istream_t pb_istream_from_uart(s_hal_uart_ctx_t *ps_uart)
{
    pb_istream_t stream = {&read_callback, (void*)ps_uart, SIZE_MAX};
    return stream;
}

static bool write_callback(pb_ostream_t *stream, const uint8_t *buf, size_t count)
{
  bool b_return = false;
  s_hal_uart_ctx_t *ps_ctx = stream->state;


  if( hal_uart_write( ps_ctx, (uint8_t *)buf, (uint16_t) count) == count )
  {
    b_return = true;
  }

  return b_return;
}

static bool read_callback(pb_istream_t *stream, uint8_t *buf, size_t count)
{
  bool b_return = false;
  s_hal_uart_ctx_t *ps_ctx = stream->state;
  uint16_t u16_received = 0;
  uint32_t timer = 0;
  bool b_timeout = false;

  /* Set the initial timout of 50 milliseconds. */
  timer = hal_timer_getTimeout(50);

  do
  {

    u16_received += hal_uart_read( ps_ctx, buf, (uint16_t) count);

    if( u16_received )
    {
      /* Update the timeout timer in case we received some data. */
      timer = hal_timer_getTimeout(50);
    }

    b_timeout = hal_timer_isTimedOut(timer);

  }
  while( !b_timeout && (u16_received < count) );

  if( u16_received )
  {
    /* At least we received some data, not necessarily all the data. */
    b_return = true;
  }

  if( b_timeout || u16_received == count )
  {
    /*
     * Two conditions for "end of file" respectively "end of data":
     * 1) We waited for some time for new data, but nothing happend.
     * 2) We actually received all the requested data.
     */
    stream->bytes_left = 0;
  }

  return b_return;
}
#endif


#if TEST_APP_USE_SERIAL_PROTOCOL
/* ===========================================================================*/
/*                  loc_simpleNanopbTestRun()                                 */
/* ===========================================================================*/
static void loc_simpleNanopbTestViaSerialProtocolRun(void)
{
  size_t message_length;
  bool status;

  /* Encode our message */
  {
      /* Allocate space on the stack to store the message data.
       *
       * Nanopb generates simple struct definitions for all the messages.
       * - check out the contents of simple.pb.h!
       * It is a good idea to always initialize your structures
       * so that you do not have garbage data from RAM in there.
       */
      SimpleMessage message = SimpleMessage_init_zero;

      /* Create a stream that will write to our buffer. */
      pb_ostream_t ostream = pb_ostream_from_serial(&gs_serial);

      /* Fill in the lucky number */
//      message.lucky_number = 13;
      message.lucky_number = 0xFFEEFFEE;

      /* Now we are ready to encode the message! */
      status = pb_encode_delimited(&ostream, SimpleMessage_fields, &message);
      message_length = ostream.bytes_written;

      /* Then just check for any errors.. */
      if (!status)
      {
          /* printf("Encoding failed: %s\n", PB_GET_ERROR(&ostream)); */
      }
  }

  {
      /* Allocate space for the decoded message. */
      SimpleMessage message = SimpleMessage_init_zero;

//      pb_istream_t istream = pb_istream_from_uart(gps_uart);

      /* Now we are ready to decode the message. */
//      status = pb_decode_delimited(&istream, SimpleMessage_fields, &message);

      /* Check for errors... */
//      if (!status)
      {
          /* printf("Decoding failed: %s\n", PB_GET_ERROR(&istream)); */
      }

      /* Print the data contained in the message. */
      /* printf("Your lucky number was %d!\n", message.lucky_number); */
  }

}/* loc_simpleNanopbTestRun() */


pb_ostream_t pb_ostream_from_serial(s_serial_protocol_ctx_t *ps_serial)
{
  /* Initialize our serial context first. */
  serial_protocol_init(ps_serial, E_SERIAL_PROTOCOL_UART_PORT_0, 115200);
  pb_ostream_t stream = {&write_serialCallback, (void*)ps_serial, SIZE_MAX, 0};
  return stream;
}


static bool write_serialCallback(pb_ostream_t *stream, const uint8_t *buf, size_t count)
{
  bool b_return = false;
  s_serial_protocol_ctx_t *ps_ctx = stream->state;
  static uint8_t ui8_bytesToWrite = 0;
  static uint8_t ui8_bytesWritten = 0;

  if( ui8_bytesToWrite == 0 )
  {
    /* First call to this callback is always passing the length of the
     * protobuf message length plus the length byte itself.
     */
    ui8_bytesToWrite = *buf + 1;
    ui8_bytesWritten = 0;

    if( serial_protocol_txStart(ps_ctx, (uint16_t) ui8_bytesToWrite) )
    {
      if( serial_protocol_txWrite(ps_ctx, (uint8_t *)buf, (uint16_t) count) )
      {
        b_return = true;
        ui8_bytesWritten++;
      }
    }
  }
  else
  {
    if( serial_protocol_txWrite(ps_ctx, (uint8_t *)buf, (uint16_t) count) )
    {
      b_return = true;
      ui8_bytesWritten += (uint8_t) count;

      if(ui8_bytesWritten >= ui8_bytesToWrite)
      {
        /* Protobuf message transmitted completely. */
        ui8_bytesToWrite = 0;
      }
    }
  }

  return b_return;
}
#endif


int main(void)
{

  /* Continue only if HAL is initialized successfully. */
  if( loc_hal_init() )
  {
    #if TEST_APP_USE_SERIAL_PROTOCOL
    loc_simpleNanopbTestViaSerialProtocolRun();
    #else
//    loc_simpleNanopbTestRun();
    #endif

    gs_serial.ps_uart = gps_uart;
    serial_protocol_init(&gs_serial, loc_serialRxCallback, E_SERIAL_PROTOCOL_UART_PORT_0, 115200);

    while(true)
    {
//      loc_simpleNanopbTestRun();
      serial_protocol_run(&gs_serial);

    }/* while() */
  }/* if() */

}
