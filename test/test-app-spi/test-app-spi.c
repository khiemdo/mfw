/* Copyright (C) 2015-2017
 *
 * test-app-spi.c
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

#include "hal_mcu.h"
#include "hal_spi.h"
#include "hal_timer.h"
#include "hal_gpio.h"
#include "hal_dma.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static void loc_test_init(void);
static void loc_test_run(void);
static void loc_test_rxTxDataBlockingMode(void);
static void loc_test_success(void);
static void loc_test_fail(void);

static void loc_callbackAfterRx(void);
static void loc_callbackAfterTxRx(void);


/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

#define TEST_APP_SPI_PORT           E_HAL_SPI_PORT_0
#define TEST_APP_SPI_MODE           E_HAL_SPI_MODE_MASTER
/*! Default SPI speed within this test-app: 1MHz */
#define TEST_APP_SPI_CLOCK_SPEED    1000000

/*! Number of test data bytes to transmit in one test loop. */
#define TEST_APP_SPI_TEST_DATA_LEN  1000

#define TEST_APP_SPI_TEST_REPETITIONS 10000


#define TEST_APP_SPI_CLEAR_RX_BUF() \
    memset( (void *)&gac_rxTestData, 0, sizeof(gac_rxTestData) )



/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */
typedef enum
{
  E_TEST_APP_STATE_DATA_TX_RX_BLOCKING,
  E_TEST_APP_STATE_DATA_TX_RX,
  E_TEST_APP_STATE_DATA_RX,
  E_TEST_APP_STATE_WAIT,
  E_TEST_APP_STATE_DATA_TX_RX_ANALYZE,
  E_TEST_APP_STATE_DATA_RX_ANALYZE
} E_TEST_APP_STATE_t;


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
s_hal_spi_ctx_t *gps_spi;
E_TEST_APP_STATE_t ge_testAppState;

uint8_t gac_txTestData[TEST_APP_SPI_TEST_DATA_LEN];
uint8_t gac_rxTestData[TEST_APP_SPI_TEST_DATA_LEN];

uint32_t gi_testRepetitionCounter;


/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    /* Call this function before SPI DMA operations are done!
     * Furthermore, this function shall be called before SPI initialization to
     * make sure that the DMA controller is initialized prior to any SPI
     * transfer or initialization. */
    b_return = hal_dma_init();
  }

  if(b_return)
  {
    gps_spi = hal_spi_init(TEST_APP_SPI_PORT, TEST_APP_SPI_MODE, TEST_APP_SPI_CLOCK_SPEED);
    if(gps_spi == NULL)
    {
      b_return = false;
    }
  }

  if(b_return)
  {
    b_return = hal_spi_config(gps_spi, E_HAL_SPI_CLK_POL_HIGH,
        E_HAL_SPI_CLK_PHA_EDGE_FIRST, TEST_APP_SPI_CLOCK_SPEED);
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

  /* Clear buffers. */
  TEST_APP_SPI_CLEAR_RX_BUF();
  memset( (void *)&gac_txTestData, 0, sizeof(gac_txTestData) );

  /* Fill our TX test data array with simple data pattern. */
  for (i = 0; i < TEST_APP_SPI_TEST_DATA_LEN; ++i)
  {
    gac_txTestData[i] = (uint8_t) (i % 256);
  }

  /* Reset state machine. Start with blocking SPI tests first. */
  ge_testAppState = E_TEST_APP_STATE_DATA_TX_RX_BLOCKING;

  gi_testRepetitionCounter = 0;

  return;
}/* loc_test_init() */


void loc_callbackAfterRx(void)
{

  ge_testAppState = E_TEST_APP_STATE_DATA_RX_ANALYZE;
  return;
}


void loc_callbackAfterTxRx(void)
{
  /* This callback gets called after SPI TX/RX transmission is finished.
   * Therefore switch to next state that checks the data pattern when
   * SPI TX/RX was used. */
  ge_testAppState = E_TEST_APP_STATE_DATA_TX_RX_ANALYZE;
  return;
}

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
  uint16_t i = 0;

  switch (ge_testAppState)
  {

    case E_TEST_APP_STATE_DATA_TX_RX_BLOCKING:
      loc_test_rxTxDataBlockingMode();
      /* Immediately switch to next state with non-blocking tests. */
      ge_testAppState = E_TEST_APP_STATE_DATA_RX;
      break;

    case E_TEST_APP_STATE_DATA_RX:
      hal_spi_xfer(gps_spi, NULL, gac_rxTestData,
                        TEST_APP_SPI_TEST_DATA_LEN, loc_callbackAfterRx);
      ge_testAppState = E_TEST_APP_STATE_WAIT;
      break;

    case E_TEST_APP_STATE_DATA_RX_ANALYZE:
      /* Verify the received data. */
      for (i = 0; i < TEST_APP_SPI_TEST_DATA_LEN; ++i)
      {
        if( gac_rxTestData[i] != 0xFF)
        {
          loc_test_fail();
        }
      }
      TEST_APP_SPI_CLEAR_RX_BUF();

      if(gi_testRepetitionCounter < TEST_APP_SPI_TEST_REPETITIONS)
      {
        /* Lets start another test round of RX only. */
        gi_testRepetitionCounter++;
        ge_testAppState = E_TEST_APP_STATE_DATA_RX;
      }
      else
      {
        /* All non-blocking RX tests passed! Now test TX/RX. */
        ge_testAppState = E_TEST_APP_STATE_DATA_TX_RX;
        /* Reset counter for TX/RX tests. */
        gi_testRepetitionCounter = 0;
      }
      break;

    case E_TEST_APP_STATE_DATA_TX_RX:
      hal_spi_xfer(gps_spi, gac_txTestData, gac_rxTestData,
                        TEST_APP_SPI_TEST_DATA_LEN, loc_callbackAfterTxRx);
      ge_testAppState = E_TEST_APP_STATE_WAIT;
      break;

    case E_TEST_APP_STATE_WAIT:
      /* Do nothing until the callback function updates ge_testAppState. */
      break;

    case E_TEST_APP_STATE_DATA_TX_RX_ANALYZE:
      /* Time to analzye this test run. The received data must match the
       * transmitted data. */
      if( memcmp(gac_txTestData, gac_rxTestData, TEST_APP_SPI_TEST_DATA_LEN) == 0 )
      {
        if(gi_testRepetitionCounter < TEST_APP_SPI_TEST_REPETITIONS)
        {
          /* Lets start another test round. */
          gi_testRepetitionCounter++;
          ge_testAppState = E_TEST_APP_STATE_DATA_TX_RX;
        }
        else
        {
          /* All tests passed successfully! */
          loc_test_success();
        }
      }
      else
      {
        /* Test failed! */
        loc_test_fail();
      }/* if() ... else */
      break;

    default:
      break;
  }

  return;
}/* loc_test_run() */

/* ===========================================================================*/
/*                  loc_test_rxTxDataBlockingMode()                           */
/* ===========================================================================*/
static void loc_test_rxTxDataBlockingMode(void)
{
  uint16_t i = 0;
  uint16_t u16_testCounter = 0;

  /* User defines the number of test cycles. */
  for (u16_testCounter = 0; u16_testCounter < TEST_APP_SPI_TEST_REPETITIONS; ++u16_testCounter)
  {

    /* Transmit and receive data at the same time. */
    hal_spi_xferBlocking(gps_spi, gac_txTestData, gac_rxTestData, TEST_APP_SPI_TEST_DATA_LEN);

    /* We expect to receive the same data pattern we just transmitted. */
    if( memcmp(gac_txTestData, gac_rxTestData, TEST_APP_SPI_TEST_DATA_LEN) == 0 )
    {
      /* Clear/Reset RX buffer for the next run. */
      TEST_APP_SPI_CLEAR_RX_BUF();
    }
    else
    {
      loc_test_fail();
    }

    /* Transmit data only. Unfortunately, we cannot verify the transmitted
     * test data pattern. But let's use this function anyway for the sake of
     * completeness.
     */
    hal_spi_xferBlocking(gps_spi, gac_txTestData, NULL, TEST_APP_SPI_TEST_DATA_LEN);


    /* Receive data only. For this test we use the idea that the SPI master
     * transmits dummy bytes to allow the SPI slave to send its data.
     * In case this test app is used in SPI loopback mode, make sure the comparison
     * below matches to what is confiured as TX dummy byte in HAL SPI (see
     * variable "gc_spiDummyByte").
     */
    hal_spi_xferBlocking(gps_spi, NULL, gac_rxTestData, TEST_APP_SPI_TEST_DATA_LEN);

    /* Verify the received data. */
    for (i = 0; i < TEST_APP_SPI_TEST_DATA_LEN; ++i)
    {
      if( gac_rxTestData[i] != 0xFF)
      {
        loc_test_fail();
      }
    }

    /* If we reached this point the RX only test was passed successfully
     * and we can reset RX buffer for the next run. */
    TEST_APP_SPI_CLEAR_RX_BUF();

  }/* for() */

  return;
}/* loc_test_rxTxDataBlockingMode() */

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
  while(true);
}


/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void)
{
  /* Continue only if HAL is initialized successfully. */
  if( loc_hal_init() )
  {

    /* Initialize the test app. */
    loc_test_init();

    /* This test app requires:
     *
     * a) HAL SPI is compiled for loopback mode (see SSI_LOOPBACK_ENABLED macro)
     *
     * or
     *
     * b) a matching SPI slave is connected to the SPI pins that answers with
     *    similar test data patterns as expected in this test loop.
     */

    while(true)
    {
      /* Continuously run the test app. */
      loc_test_run();
    }/* while() */

  }/* if() */

}/* main() */
