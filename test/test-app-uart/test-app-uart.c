/* Copyright (C) 2015-2017
 *
 * test-app-uart.c
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Fabian Girrbach     <fabiangirrbach@gmail.com>
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
#include "hal_uart.h"
#include "hal_dma.h"
#include "hal_timer.h"
#include "hal_mcu.h"
#include "hal_gpio.h"


/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static void loc_test_init(void);
static void loc_test_run(void);
static void loc_test_success(void);
static void loc_test_fail(void);
static void loc_test_sendData(void);
static bool loc_test_rcvData(void);
static void loc_test_verifyRxData(void);


/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the amount of test data bytes transfered in each test loop.
 *  This number shall not exceed 0xFFFF as 16bit variables are used. */
#define TEST_APP_TEST_DATA_LEN      250

/*! Defines the amount of repititions/loops for the test.
 *  For each repitition the amount of test data is \ref TEST_APP_TEST_DATA_LEN.
 *  This number shall not exceed 0xFFFFFFFF as 32bit variables are used. */
#define TEST_APP_TEST_REPETITIONS   10

#define TEST_APP_START_LEN_TX      240
#define TEST_APP_START_LEN_RX      240

/*! Defines the UART port to be tested. Must be of type \ref E_HAL_UART_PORT_t. */
#define TEST_APP_UART_PORT          E_HAL_UART_PORT_0

/*! Defnies the baud rate to be used for the UART port under test. */
#define TEST_APP_UART_BAUD_RATE     115200

#define TEST_APP_UART_TX_INTERVAL_MS   20

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */
typedef enum TEST_APP_STATE
{
  E_TEST_APP_TX_DATA,

  E_TEST_APP_RX_DATA,

  E_TEST_APP_ANALYZE_DATA

} e_test_app_state_t ;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
uint8_t gc_testData[TEST_APP_TEST_DATA_LEN];
uint8_t gc_testDataRx[TEST_APP_TEST_DATA_LEN];
e_test_app_state_t ge_testState;
uint32_t gi_testRepetitionCounterTx;
uint32_t gi_testRepetitionCounterRx;

uint16_t g_currentTestDataLenTx;
uint16_t g_currentTestDataLenRx;

volatile uint16_t g_readBytes;

s_hal_uart_ctx_t *gps_uart;

uint32_t g_uartTxTimer;

uint32_t count;

int errorIndex;

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    b_return = hal_dma_init();
  }

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

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
void loc_test_init(void)
{
  uint16_t i = 0;

  /* Fill our test data array with simple data pattern. */
  for (i = 0; i < TEST_APP_TEST_DATA_LEN; ++i)
  {
    gc_testData[i] = (uint8_t) (i % 256);
  }

  /* Reset RX data array before startup. */
  memset(gc_testDataRx, 0, TEST_APP_TEST_DATA_LEN);

  /* After startup of the test-app we transmit data. */
  ge_testState = E_TEST_APP_TX_DATA;
  /* Lets count our test repetitions. We start counting with 1 here. */
  gi_testRepetitionCounterTx = 0;
  gi_testRepetitionCounterRx = 1;

  g_currentTestDataLenTx = TEST_APP_START_LEN_TX;
  g_currentTestDataLenRx = TEST_APP_START_LEN_RX;

  g_readBytes = 0;

  g_uartTxTimer = 0;

  count = 0;
  errorIndex = 0;

}/* loc_test_init() */

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{

  if( hal_timer_isTimedOut(g_uartTxTimer) )
  {
    g_uartTxTimer = hal_timer_getTimeout(TEST_APP_UART_TX_INTERVAL_MS);
    loc_test_sendData();
  }

  if( loc_test_rcvData() )
  {
    loc_test_verifyRxData();
  }

}/* loc_test_run() */


/* ===========================================================================*/
/*                  loc_test_success()                                        */
/* ===========================================================================*/
void loc_test_success(void)
{
  uint32_t timer = 0;
  uint8_t ledPin = HAL_GPIO_PIN_0;

  hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);

  //
  // Turn on the LED as a heartbeat
  //
  hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);

  /* Set the timout of 500 milliseconds. */
  timer = hal_timer_getTimeout(500);

  while(true)
  {
    /* Toggle LED every 500ms to signal we passed the test successfully. */
    if( hal_timer_isTimedOut(timer) )
    {
      ledPin ^= HAL_GPIO_PIN_0;
      hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);
      timer = hal_timer_getTimeout(500);
    }
  }

}/* loc_test_success() */

/* ===========================================================================*/
/*                  loc_test_fail()                                           */
/* ===========================================================================*/
void loc_test_fail(void)
{
  uint32_t timer = 0;
  uint8_t ledPin = HAL_GPIO_PIN_0;

  hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT);

  //
  // Turn on the LED as a heartbeat
  //
  hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);

  /* Set the timout of 500 milliseconds. */
  timer = hal_timer_getTimeout(100);

  while(true)
  {
    /* Toggle LED every 500ms to signal we passed the test successfully. */
    if( hal_timer_isTimedOut(timer) )
    {
      ledPin ^= HAL_GPIO_PIN_0;
      hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_0, ledPin);
      timer = hal_timer_getTimeout(100);
    }
  }
}

/* ===========================================================================*/
/*                  loc_test_sendData()                                       */
/* ===========================================================================*/
void loc_test_sendData(void)
{
  uint16_t i_written = 0;
  bool b_send = false;

  if(gi_testRepetitionCounterTx < TEST_APP_TEST_REPETITIONS)
  {
    gi_testRepetitionCounterTx++;
    b_send = true;
  }
  else
  {
    if( g_currentTestDataLenTx < TEST_APP_TEST_DATA_LEN )
    {
      g_currentTestDataLenTx++;
      gi_testRepetitionCounterTx = 1;
      b_send = true;
    }
  }

  if( b_send )
  {
    do
    {
      i_written += hal_uart_write( gps_uart,
            (gc_testData + i_written) , g_currentTestDataLenTx - i_written);

    } while (i_written < g_currentTestDataLenTx);
  }

  return;
}

/* ===========================================================================*/
/*                  loc_test_rcvData()                                        */
/* ===========================================================================*/
bool loc_test_rcvData(void)
{
  bool b_return = false;

  g_readBytes += hal_uart_read( gps_uart,
        (gc_testDataRx + g_readBytes) , g_currentTestDataLenRx - g_readBytes);

  if( g_readBytes == g_currentTestDataLenRx )
  {
    b_return = true;
    count++;
  }

  if( (g_currentTestDataLenRx == TEST_APP_TEST_DATA_LEN) &&
      (gi_testRepetitionCounterRx == TEST_APP_TEST_REPETITIONS) )
  {
    /* Last piece of data received! */
    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_test_verifyRxData()                                   */
/* ===========================================================================*/
void loc_test_verifyRxData(void)
{
  if( memcmp(gc_testData, gc_testDataRx, g_currentTestDataLenRx) == 0 )
  {
    if(gi_testRepetitionCounterRx < TEST_APP_TEST_REPETITIONS)
    {
      gi_testRepetitionCounterRx++;
    }
    else
    {
      if( g_currentTestDataLenRx == TEST_APP_TEST_DATA_LEN)
      {
        /* All tests passed successfully! */
        loc_test_success();
      }
      else
      {
        /* Restart test loop with new test data len. */
        g_currentTestDataLenRx++;
        gi_testRepetitionCounterRx = 1;
      }
    }

    memset(gc_testDataRx, 0, TEST_APP_TEST_DATA_LEN);
    g_readBytes = 0;
  }
  else
  {
    if( (g_currentTestDataLenRx == TEST_APP_TEST_DATA_LEN) &&
        (gi_testRepetitionCounterRx == TEST_APP_TEST_REPETITIONS) )
    {
      /* All tests passed successfully! */
      loc_test_success();
    }
    else
    {
      errorIndex = memcmp(gc_testData, gc_testDataRx, g_currentTestDataLenRx);
      /* Test failed! */
      loc_test_fail();
    }
  }/* if() ... else */
}

/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void) {

  /* Continue only if HAL is initialized successfully. */
  if( loc_hal_init() )
  {

    /* Initialize the test app. */
    loc_test_init();

    while(true)
    {

      /* Continuously run the test app. */
      loc_test_run();

    }/* while() */
  }/* if() */

}/* main() */
