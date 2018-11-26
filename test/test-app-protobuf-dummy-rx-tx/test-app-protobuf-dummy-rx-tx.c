/* Copyright (C) 2015-2017
 *
 * test-app-protobuf-dummy-rx-tx.c
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
#include "hal_gpio.h"

#include "sproto.h"

#include <pb_encode.h>
#include <pb_decode.h>

#include "betconf.pb.h"
#include "betcall.pb.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static void loc_firmware_init(void);
static void loc_firmware_run(void);
static void loc_firmware_error(void);
/* Callback functions used by the serial protocol module.  */
static uint16_t loc_cbSerialRead(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_cbSerialWrite(uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbSerialRxDataRdy(uint8_t *pc_data, uint16_t ui16_len);

/* Local function that generates us the output stream (context). */
static pb_ostream_t pb_ostream_from_serial(s_sproto_ctx_t *ps_serial);
/* Callback functions used by the nanopb module. */
static bool loc_cbProtobufWrite(pb_ostream_t *stream, const uint8_t *buf, size_t count);


static bool loc_sendMsgBetConf(void);
static bool loc_sendMsgBetCall(void);

static void loc_getPbMsgBasicConf(void * p_cnf);
static void loc_getPbMsgBetComTicks(void * p);
static void loc_getPbMsgBetComTimestamp(void * p);
static void loc_getPbMsgBetComTime(void * p);

static float loc_getPbMsgFloatValue(void);
static uint32_t loc_getPbMsgUint32Value(void);
static uint64_t loc_getPbMsgUint64Value(void);

//*****************************************************************************
//
//! \addtogroup test_app_params Test APP protobuf dummy - Compile time options
//! @{
//
//*****************************************************************************

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the UART port to be tested. Must be of type \ref E_HAL_UART_PORT_t. */
#define TEST_APP_UART_PORT          E_HAL_UART_PORT_0

/*! Defnies the baud rate to be used for the UART port. Default: 115200 kBaud */
#define TEST_APP_UART_BAUD_RATE     115200

/*! As the test-app provides the buffer to the serial protocol module,
 * this number defines the buffer length of the serial protocol RX buffer.
 * Make sure this value fits to the maximum legth of a protobuf message! */
#define TEST_APP_SERIAL_PROTOCOL_BUF_LEN      1024

/*! Transmission intervall for protocol buffer message in milliseconds.
 *  Default: 1000ms. */
#define TEST_APP_TX_INTERVALL_MS     1000

/*! Timeout the RX_SPROTO_LED is turned on (in ms) after successfull reception
 *  of a serial protocol frame. */
#define TEST_APP_RX_SPROTO_LED_ON_TIME_MS     50

/*! Timeout the RX_PAYLOAD_LED is turned on (in ms) after reception of a valid
 *  data payload within a serial protocol frame. */
#define TEST_APP_RX_PAYLOAD_LED_ON_TIME_MS     50

/*! Timeout the TX_LED is turned on (in ms) after successfull transmission
 *  of a protobuf message. */
#define TEST_APP_TX_LED_ON_TIME_MS     100

/*! Macro that turns off TX_LED that is \b D4 on crypto-launchpad. */
#define TEST_APP_TX_LED_OFF() \
            hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, ~HAL_GPIO_PIN_0)

/*! Macro that turns on TX_LED that is \b D4 on crypto-launchpad. */
#define TEST_APP_TX_LED_ON() \
            hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, HAL_GPIO_PIN_0)

/*! Macro that turns off RX_SPROTO_LED that is \b D2 on crypto-launchpad. */
#define TEST_APP_RX_SPROTO_LED_OFF() \
            hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ~HAL_GPIO_PIN_0)

/*! Macro that turns on RX_SPROTO_LED that is \b D2 on crypto-launchpad. */
#define TEST_APP_RX_SPROTO_LED_ON() \
            hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, HAL_GPIO_PIN_0)

/*! Macro that turns off RX_PAYLOAD_LED that is \b D1 on crypto-launchpad. */
#define TEST_APP_RX_PAYLOAD_LED_OFF() \
            hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, ~HAL_GPIO_PIN_1)

/*! Macro that turns on RX_PAYLOAD_LED that is \b D1 on crypto-launchpad. */
#define TEST_APP_RX_PAYLOAD_LED_ON() \
            hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, HAL_GPIO_PIN_1)

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//
//*****************************************************************************

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */


/* ===========================================================================*/
/*                  Global variables                                          */
/* ===========================================================================*/
/*! Pointer to the context to be used for HAL UART operations. */
s_hal_uart_ctx_t *gps_uart;

/*! Global context of the serial protocol (= our instance of the protocol). */
s_sproto_ctx_t gs_serial;

/*! Buffer that is passed to the serial protocol module (as its RX data memory). */
uint8_t gac_serialProtocolBuffer[TEST_APP_SERIAL_PROTOCOL_BUF_LEN];

/*! Global context of the output stream to be used by protocol buffers (nanopb) */
pb_ostream_t g_ostream;

/*! Global context of the input stream to be used by protocol buffers (nanopb) */
pb_istream_t g_istream;

/*! Global, static variable for transmission of pb BetCONF message in
 *  \ref loc_sendMsgBetConf(). If you use a local variable make sure the stack
 *  size is configured properly within project settings!
 *  However, pb BetCONF are not transmitted by this app by default! */
BetCONF_Configuration g_txBetConf;

/*! Global, static variable for reception of pb BetCONF message in
 *  \ref loc_cbSerialRxDataRdy(). If you use a local variable make sure the stack
 *  size is configured properly within project settings!
 */
BetCONF_Configuration g_rxBetConf;

/*! Global, static constant variable for initialization of pb BetCONF using
 * default values as defined in proto file.
 */
const BetCONF_Configuration g_txBetConfDefault = BetCONF_Configuration_init_default;

/*! Global, static variable for transmission of pb BetCALL message in
 *  \ref loc_sendMsgBetCall(). If you use a local variable make sure the stack
 *  size is configured properly within project settings! */
BetCALL_Sensors g_txBetCall;

/*! Global, static constant variable for initialization of pb BetCALL using
 * default values as defined in proto file.
 */
const BetCALL_Sensors g_txBetCallDefault = BetCALL_Sensors_init_default;

/*! Dummy payload value to be used for float variables in pb message. */
float g_pbMsgFloatValue;

/*! Dummy payload value to be used for uint32 variables in pb message. */
uint32_t g_pbMsgUint32Value;

/*! Dummy payload value to be used for uint64 variables in pb message. */
uint64_t g_pbMsgUint64Value;

/*! Timer used to measure the time the RX_SPROTO_LED is turned on (in ms). */
uint32_t g_rxSprotoLedTimer;

/*! Timer used to measure the time the RX_PAYLOAD_LED is turned on (in ms). */
uint32_t g_rxPayloadLedTimer;

/*! Timer used to measure the time the TX_LED is turned on (in ms). */
uint32_t g_txLedTimer;

/* ===========================================================================*/
/*                  FUNCTION IMPLEMENTATIONS                                  */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
static bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    gps_uart = hal_uart_init(TEST_APP_UART_PORT, TEST_APP_UART_BAUD_RATE);
    if(gps_uart == NULL)
    {
      b_return = false;
    }
  }

  if(b_return)
  {
    b_return = hal_timer_init();
  }

  if(b_return)
  {
    /* Init all LEDs with state = OFF. */
    hal_gpio_setDir(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
    TEST_APP_TX_LED_OFF();

    hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
    TEST_APP_RX_SPROTO_LED_OFF();

    hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, E_HAL_GPIO_DIR_OUT);
    TEST_APP_RX_PAYLOAD_LED_OFF();
  }

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_firmware_init()                                       */
/* ===========================================================================*/
static void loc_firmware_init(void)
{
  bool status = false;

  /* Init the serial protocol module. */
  status = sproto_init(&gs_serial,                                 /* context   */
      loc_cbSerialWrite, loc_cbSerialRead, loc_cbSerialRxDataRdy,  /* callbacks */
      gac_serialProtocolBuffer, TEST_APP_SERIAL_PROTOCOL_BUF_LEN); /* buffer    */

  if( status )
  {
    /* Lets ignore the return here for now. This function is active only if
     * SERIAL_PROTOCOL_RX_TIMEOUT_ENABLE is enabled for sproto module.
     * It returns false, in case macro is discabled. To not distinguish the
     * cases here and not increase complexity, return val is ignore for now. */
    sproto_setRxTimeout(&gs_serial, 2);

    /* Init our protobuf output stream. */
    g_ostream = pb_ostream_from_serial( &gs_serial );

    /* Init our protobuf input stream. It is intialized using the same buffer
     * we gave to the serial protocol module, s.t. protobuf works on the same
     * memory and we don't need to copy the data (again). */
    g_istream = pb_istream_from_buffer( gac_serialProtocolBuffer,
                                        TEST_APP_SERIAL_PROTOCOL_BUF_LEN );

    /* Initialize all our dummy values that we transmit in our pb messages. */
    g_pbMsgFloatValue = 0;
    g_pbMsgUint32Value = 0;
    g_pbMsgUint64Value = 0;

    /* Init the LED timers. */
    g_rxSprotoLedTimer = 0;
    g_txLedTimer = 0;
    g_rxPayloadLedTimer = 0;
  }
  else
  {
    loc_firmware_error();
  }

  return;
}

/* ===========================================================================*/
/*                  loc_firmware_init()                                       */
/* ===========================================================================*/
static void loc_firmware_run(void)
{
  static uint32_t txTimer = 0;

  /* Run the serial protocol. This shall be done as often as possible! */
  sproto_run(&gs_serial);

  /* Hnadle transmission of protobuf message. */
  if( hal_timer_isTimedOut(txTimer) )
  {
    if( !loc_sendMsgBetCall() )
    {
      loc_firmware_error();
    }

    /* Increment our dummy payload values after transmission. */
    g_pbMsgFloatValue++;
    g_pbMsgUint32Value++;
    g_pbMsgUint64Value++;

    /* Dummy value overflow protection. */
    if( g_pbMsgUint32Value == 0xFFFFFFFF )
    {
      g_pbMsgFloatValue = 0;
      g_pbMsgUint32Value = 0;
      g_pbMsgUint64Value = 0;
    }

    /* Set timout for next transmission. */
    txTimer = hal_timer_getTimeout(TEST_APP_TX_INTERVALL_MS);

    /* Turn on LED and start a timer to turn off the TX_LED after some time. */
    TEST_APP_TX_LED_ON();
    g_txLedTimer = hal_timer_getTimeout(TEST_APP_TX_LED_ON_TIME_MS);
  }

  /* Turn off the TX_LED some time after transmission. */
  if( hal_timer_isTimedOut(g_txLedTimer) )
  {
    TEST_APP_TX_LED_OFF();
  }

  /* If the timeout occurred, turn of the RX_SPROTO_LED that was turned on in
   * loc_cbSerialRxDataRdy(). */
  if( hal_timer_isTimedOut(g_rxSprotoLedTimer) )
  {
    TEST_APP_RX_SPROTO_LED_OFF();
  }

  /* If the timeout occurred, turn of the RX_PAYLOAD_LED that was turned on in
   * loc_cbSerialRxDataRdy(). */
  if( hal_timer_isTimedOut(g_rxPayloadLedTimer) )
  {
    TEST_APP_RX_PAYLOAD_LED_OFF();
  }

  return;
}


/* ===========================================================================*/
/*                  loc_firmware_error()                                      */
/* ===========================================================================*/
static void loc_firmware_error(void)
{
  uint32_t timer = 0;
  uint8_t ledPin = HAL_GPIO_PIN_0;

  hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);

  /* Toggle D1 as error LED. */
  hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);

  /* Set the timout of 200 milliseconds. */
  timer = hal_timer_getTimeout(200);

  /* Loop for infinity. */
  while(true)
  {
    /* Toggle LED every 200ms to signal there was some formware error. */
    if( hal_timer_isTimedOut(timer) )
    {
      ledPin ^= HAL_GPIO_PIN_0;
      hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);
      timer = hal_timer_getTimeout(200);
    }
  }

}


/* ===========================================================================*/
/*                  loc_cbSerialRead()                                        */
/* ===========================================================================*/
static uint16_t loc_cbSerialRead(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_read(gps_uart, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbSerialWrite()                                       */
/* ===========================================================================*/
static uint16_t loc_cbSerialWrite(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_write(gps_uart, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbSerialRxDataRdy()                                   */
/* ===========================================================================*/
static void loc_cbSerialRxDataRdy(uint8_t *pc_data, uint16_t ui16_len)
{
  bool isPayloadValid = false;

  /*
   * Getting this callback called signals that a serial protocol message is
   * received successfully. Therfore, give visual feedback by first let the
   * RX_SPROTO_LED blink.
   */
  TEST_APP_RX_SPROTO_LED_ON();
  g_rxSprotoLedTimer = hal_timer_getTimeout( TEST_APP_RX_SPROTO_LED_ON_TIME_MS );


  /* Workaround: Initialize the istream for decode that overcomes the
   * error/bug, that decode() fails after 876 messages. */
  g_istream = pb_istream_from_buffer( pc_data, TEST_APP_SERIAL_PROTOCOL_BUF_LEN );

  /*
   * Now evaluate the transported data, i.e. the data transported within the
   * serial protocol frame. We let another LED blink upon successfull reception
   * of the following data patterns:
   *
   * a) payload = 0x13 0x37
   * b) payload = BetCONF protocol buffer message.
   */
  if( (ui16_len == 2) && (pc_data[0] == 0x13) && (pc_data[1] == 0x37) )
  {
    isPayloadValid = true;
  }
  else if( pb_decode_delimited(&g_istream, BetCONF_Configuration_fields, &g_rxBetConf) )
  {
    isPayloadValid = true;
  }

  if( isPayloadValid )
  {
    /* We successfully received a valid payload we listen for. Let another LED
     * blink as visual feedback. */
    TEST_APP_RX_PAYLOAD_LED_ON();
    g_rxPayloadLedTimer = hal_timer_getTimeout( TEST_APP_RX_PAYLOAD_LED_ON_TIME_MS );
  }

  return;
}



static pb_ostream_t pb_ostream_from_serial(s_sproto_ctx_t *ps_serial)
{
  pb_ostream_t stream = {&loc_cbProtobufWrite, (void*)ps_serial, SIZE_MAX, 0};
  return stream;
}

static bool loc_cbProtobufWrite(pb_ostream_t *stream, const uint8_t *buf, size_t count)
{
  bool b_return = false;
  s_sproto_ctx_t *ps_ctx = stream->state;
  static uint8_t ui8_bytesToWrite = 0;
  static uint8_t ui8_bytesWritten = 0;

  if( ui8_bytesToWrite == 0 )
  {
    /* First call to this callback is always passing the length of the
     * protobuf message length plus the length byte itself.
     *
     * Don't forget to parse the varint value!!!!
     */
    ui8_bytesToWrite = *buf + 1;
    ui8_bytesWritten = 0;

    if( sproto_txStart(ps_ctx, (uint16_t) ui8_bytesToWrite) )
    {
      if( sproto_txWrite(ps_ctx, (uint8_t *)buf, (uint16_t) count) )
      {
        b_return = true;
        ui8_bytesWritten++;
      }
    }
  }
  else
  {
    if( sproto_txWrite(ps_ctx, (uint8_t *)buf, (uint16_t) count) )
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

/* ===========================================================================*/
/*                  loc_sendMsgBetConf()                                      */
/* ===========================================================================*/
static bool loc_sendMsgBetConf(void)
{
  g_txBetConf = g_txBetConfDefault;

  loc_getPbMsgBasicConf(&g_txBetConf.airspeed.basic_conf);
  g_txBetConf.has_airspeed = true;
  g_txBetConf.airspeed.has_speed_offset               = true;
  g_txBetConf.airspeed.speed_offset                   = loc_getPbMsgFloatValue();


  loc_getPbMsgBasicConf(&g_txBetConf.angle_of_attack.basic_conf);
  g_txBetConf.has_angle_of_attack                     = true;
  g_txBetConf.angle_of_attack.has_horizontal_offset   = true;
  g_txBetConf.angle_of_attack.horizontal_offset       = loc_getPbMsgUint32Value();
  g_txBetConf.angle_of_attack.has_vertical_offset     = true;
  g_txBetConf.angle_of_attack.vertical_offset         = loc_getPbMsgUint32Value();


  loc_getPbMsgBasicConf(&g_txBetConf.barometer.basic_conf);
  g_txBetConf.has_barometer                           = true;
  g_txBetConf.barometer.has_altitude_offset           = true;
  g_txBetConf.barometer.altitude_offset               = loc_getPbMsgFloatValue();
  g_txBetConf.barometer.has_pressure_offset           = true;
  g_txBetConf.barometer.pressure_offset               = loc_getPbMsgFloatValue();
  g_txBetConf.barometer.has_temperature_offset        = true;
  g_txBetConf.barometer.temperature_offset            = loc_getPbMsgFloatValue();


  loc_getPbMsgBasicConf(&g_txBetConf.dms.basic_conf);
  g_txBetConf.has_dms                                 = true;
  g_txBetConf.dms.strain_offset                       = loc_getPbMsgFloatValue();


  loc_getPbMsgBasicConf(&g_txBetConf.gps.basic_conf);
  g_txBetConf.has_gps = true;
  g_txBetConf.gps.has_velocity_axial_system    = true;
  g_txBetConf.gps.velocity_axial_system        = BetCOM_GpsAxialSystem_NED;
  g_txBetConf.gps.has_position_axial_system    = true;
  g_txBetConf.gps.position_axial_system        = BetCOM_GpsAxialSystem_NED;
  g_txBetConf.gps.has_baseline_axial_system    = true;
  g_txBetConf.gps.baseline_axial_system        = BetCOM_GpsAxialSystem_NED;
  g_txBetConf.gps.has_send_time                     = true;
  g_txBetConf.gps.send_time                         = true;
  g_txBetConf.gps.has_send_accuracy                 = true;
  g_txBetConf.gps.send_accuracy                     = true;
  g_txBetConf.gps.has_send_dilution_of_precision    = true;
  g_txBetConf.gps.send_dilution_of_precision        = true;


  loc_getPbMsgBasicConf(&g_txBetConf.line_angle.basic_conf);
  g_txBetConf.has_line_angle              = true;
  g_txBetConf.line_angle.azimuth_offset   = loc_getPbMsgFloatValue();
  g_txBetConf.line_angle.elevation_offset = loc_getPbMsgFloatValue();


  loc_getPbMsgBasicConf(&g_txBetConf.temperature.basic_conf);
  g_txBetConf.has_temperature = true;


  loc_getPbMsgBasicConf(&g_txBetConf.tube_angle.basic_conf);
  g_txBetConf.has_tube_angle                  = true;
  g_txBetConf.tube_angle.has_elevation_offset = true;
  g_txBetConf.tube_angle.has_rotation_offset  = true;
  g_txBetConf.tube_angle.elevation_offset     = loc_getPbMsgFloatValue();
  g_txBetConf.tube_angle.rotation_offset      = loc_getPbMsgFloatValue();

  return pb_encode_delimited(&g_ostream, BetCONF_Configuration_fields, &g_txBetConf);
}


/* ===========================================================================*/
/*                  loc_sendMsgBetCall()                                      */
/* ===========================================================================*/
static bool loc_sendMsgBetCall(void)
{
  g_txBetCall = g_txBetCallDefault;

  loc_getPbMsgBetComTime( (void *) &g_txBetCall.time );

  g_txBetCall.has_temperature = true;
  g_txBetCall.temperature.has_temperature       = true;
  g_txBetCall.temperature.has_temperature_raw   = true;
  g_txBetCall.temperature.temperature           = loc_getPbMsgFloatValue();
  g_txBetCall.temperature.temperature_raw       = loc_getPbMsgUint32Value();

  g_txBetCall.has_temperature_arm = true;
  g_txBetCall.temperature_arm.has_temperature     = true;
  g_txBetCall.temperature_arm.has_temperature_raw = true;
  g_txBetCall.temperature_arm.temperature         = loc_getPbMsgFloatValue();
  g_txBetCall.temperature_arm.temperature_raw     = loc_getPbMsgUint32Value();
  loc_getPbMsgBetComTime( (void *) &g_txBetCall.temperature_arm.time );

  return pb_encode_delimited(&g_ostream, BetCALL_Sensors_fields, &g_txBetCall);
}

/* ===========================================================================*/
/*                  loc_getPbMsgBasicConf()                                   */
/* ===========================================================================*/
static void loc_getPbMsgBasicConf(void * p_cnf)
{
  BetCOM_BasicConf *p_config = (BetCOM_BasicConf *)p_cnf;

  p_config->activated = true;
  p_config->rate      = loc_getPbMsgUint32Value();
  p_config->raw       = true;

  return;
}

/* ===========================================================================*/
/*                  loc_getPbMsgBasicConf()                                   */
/* ===========================================================================*/
static void loc_getPbMsgBetComTicks(void * p)
{
  BetCOM_Ticks *p_tick = (BetCOM_Ticks *)p;

  p_tick->onepps_counter = loc_getPbMsgUint32Value();
  p_tick->ticks          = loc_getPbMsgUint32Value();

  return;
}

/* ===========================================================================*/
/*                  loc_getPbMsgBasicConf()                                   */
/* ===========================================================================*/
static void loc_getPbMsgBetComTimestamp(void * p)
{
  BetCOM_Timestamp *p_timestamp = (BetCOM_Timestamp *)p;

  p_timestamp->tsec  = loc_getPbMsgUint64Value();
  p_timestamp->tnsec = loc_getPbMsgUint64Value();

  return;
}

/* ===========================================================================*/
/*                  loc_getPbMsgBasicConf()                                   */
/* ===========================================================================*/
static void loc_getPbMsgBetComTime(void * p)
{
  BetCOM_Time *p_time = (BetCOM_Time *)p;

  p_time->has_ticks_measured      = true;
  p_time->has_ticks_received      = true;
  p_time->has_ticks_sent          = true;
  loc_getPbMsgBetComTicks( &(p_time->ticks_measured) );
  loc_getPbMsgBetComTicks( &(p_time->ticks_received) );
  loc_getPbMsgBetComTicks( &(p_time->ticks_sent    ) );
  p_time->has_timestamp_measured  = true;
  p_time->has_timestamp_received  = true;
  p_time->has_timestamp_sent      = true;
  loc_getPbMsgBetComTimestamp( &(p_time->timestamp_measured) );
  loc_getPbMsgBetComTimestamp( &(p_time->timestamp_received) );
  loc_getPbMsgBetComTimestamp( &(p_time->timestamp_sent    ) );

  return;
}

/* ===========================================================================*/
/*                  loc_getPbMsgFloatValue()                                  */
/* ===========================================================================*/
static float loc_getPbMsgFloatValue(void)
{
  return g_pbMsgFloatValue;
}

/* ===========================================================================*/
/*                  loc_getPbMsgUint32Value()                                 */
/* ===========================================================================*/
static uint32_t loc_getPbMsgUint32Value(void)
{
  return g_pbMsgUint32Value;
}

/* ===========================================================================*/
/*                  loc_getPbMsgUint64Value()                                 */
/* ===========================================================================*/
static uint64_t loc_getPbMsgUint64Value(void)
{
  return g_pbMsgUint64Value;
}


/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void)
{
  if( loc_hal_init() )
  {
    /* Initialize our firmware / test application */
    loc_firmware_init();

    while(1)
    {
      /* Run our firmware continously and as often as possible! */
      loc_firmware_run();

    }
  }
	return 0;
}
