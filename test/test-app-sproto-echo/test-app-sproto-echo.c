/* Copyright (C) 2015-2017
 *
 * test-app-sproto-echo.c
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

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static void loc_firmware_init(void);
static void loc_firmware_run(void);

/* Callback functions used by the serial protocol module.  */
static uint16_t loc_cbSerialRead(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_cbSerialWrite(uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbSerialRxDataRdy(uint8_t *pc_data, uint16_t ui16_len);

//*****************************************************************************
//
//! \addtogroup test_app_sproto_echo_params Test APP Serial Protocol Echo - Compile time options
//! @{
//
//*****************************************************************************

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the UART port to be tested. Must be of type \ref E_HAL_UART_PORT_t.
 *  \warning The TI development board:
 *           UART port 0 is shared with the USB/Serial connection to the
 *           host PC, i.e. the RX/TX pins of port 0 might be influenced by the
 *           USB-to-serial converter chip. It is therefore recommended to use
 *           another port when connecting to the groundstation! */
#define TEST_APP_UART_PORT          E_HAL_UART_PORT_1

/*! Defnies the baud rate to be used for the UART port. Default: 115200 kBaud */
#define TEST_APP_UART_BAUD_RATE     115200

/*! As the test-app provides the buffer to the serial protocol module,
 * this number defines the buffer length of the serial protocol RX buffer.
 * Make sure this value fits to the maximum legth of a protobuf message! */
#define TEST_APP_SERIAL_PROTOCOL_BUF_LEN      1024

/*! Time in milliseconds the heartbeat LED is turned ON. */
#define TEST_APP_HEARTBEAT_LED_ON_TIME_MS      500
/*! Time in milliseconds the heartbeat LED is turned OFF. */
#define TEST_APP_HEARTBEAT_LED_OFF_TIME_MS     1000

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

/*! Macro that turns off HEARTBEAT_LED that is \b D4 on crypto-launchpad. */
#define TEST_APP_HEARTBEAT_LED_OFF() \
            hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, ~HAL_GPIO_PIN_0)

/*! Macro that turns on HEARTBEAT_LED that is \b D4 on crypto-launchpad. */
#define TEST_APP_HEARTBEAT_LED_ON() \
            hal_gpio_write(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, HAL_GPIO_PIN_0)

/*! Macro that return the HEARTBEAT_LED state that is \b D4 on crypto-launchpad. */
#define TEST_APP_HEARTBEAT_LED_STATE() \
            hal_gpio_read(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0)

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
    /* Init all LEDs (of TI development board!) with state = OFF. */
    hal_gpio_setDir(E_HAL_GPIO_PORT_F, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
    hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);
    hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_1, E_HAL_GPIO_DIR_OUT);
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
     * It returns false, in case macro is disabled. To not distinguish the
     * cases here and not increase complexity, return val is ignore for now. */
    sproto_setRxTimeout(&gs_serial, 2);
  }

  return;
}

/* ===========================================================================*/
/*                  loc_firmware_init()                                       */
/* ===========================================================================*/
static void loc_firmware_run(void)
{
  static uint32_t heartbeatTimer = 0;

  /* Run the serial protocol. This shall be done as often as possible! */
  sproto_run(&gs_serial);

  /* Lets turn on/off some heartbeat LED to show we are fine. */
  if( hal_timer_isTimedOut(heartbeatTimer) )
  {
    if( TEST_APP_HEARTBEAT_LED_STATE() )
    {
      /* LED is ON. Turn it off! */
      heartbeatTimer = hal_timer_getTimeout(TEST_APP_HEARTBEAT_LED_OFF_TIME_MS);
      TEST_APP_HEARTBEAT_LED_OFF();
    }
    else
    {
      /* LED is OFF. Turn it on! */
      heartbeatTimer = hal_timer_getTimeout(TEST_APP_HEARTBEAT_LED_ON_TIME_MS);
      TEST_APP_HEARTBEAT_LED_ON();
    }
  }

  return;
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
  /* Catch the received data here and immediately forward/echo it here. */
  sproto_txStart(&gs_serial, ui16_len);
  sproto_txWrite(&gs_serial, pc_data, ui16_len);

  return;
}


/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void)
{
  if( loc_hal_init() )
  {

    loc_firmware_init();

    while(1)
    {
      /* Run our firmware continously and as often as possible! */
      loc_firmware_run();
    }
  }
  return 0;
}
