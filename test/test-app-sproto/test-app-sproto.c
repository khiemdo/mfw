/* Copyright (C) 2015-2017
 *
 * test-app-sproto.c
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
static void loc_test_init(void);
static void loc_test_run(void);
static void loc_test_success(void);
static void loc_test_fail(void);
static bool loc_testCrcCalculation(void);
static uint16_t loc_serialRead(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_serialWrite(const uint8_t *pc_data, uint16_t ui16_len);
static void loc_serialRxDataRdy(uint8_t *pc_data, uint16_t ui16_len);
static void loc_serialError(E_SERIAL_PROTOCOL_ERROR_t e_errType);
static uint16_t loc_serialCrc(const uint8_t *pc_data, uint16_t ui16_len);



/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! As the test-app provides the buffer to the serial protocol module,
 * this number defines the buffer length of the serial protocol RX buffer.
 * Make sure this is not smaller than \ref TEST_APP_TEST_DATA_LEN. */
#define TEST_APP_SERIAL_PROTOCOL_BUF_LEN      1024

/*! Defines the amount of test data bytes transfered in each test loop.
 *  This number shall not exceed 0xFFFF as 16bit variables are used. */
#define TEST_APP_TEST_DATA_LEN      50

/*! Defines the amount of repititions/loops for the test.
 *  For each repitition the amount of test data is \ref TEST_APP_TEST_DATA_LEN.
 *  This number shall not exceed 0xFFFFFFFF as 32bit variables are used. */
#define TEST_APP_TEST_REPETITIONS   2000

/*! Defines the UART port to be tested. Must be of type \ref E_HAL_UART_PORT_t. */
#define TEST_APP_UART_PORT          E_HAL_UART_PORT_0

/*! Defines the baud rate to be used for the UART port under test. */
#define TEST_APP_UART_BAUD_RATE     115200

/*! \brief Compile time option: Perform CRC calculation in HW if set to '1'.
 *
 * \details Compile time option of this test app:
 *          Use HAL_CRC module to perform CRC calculation if set to '1'.
 *          Disable this option by setting to '0'.
 *          This option must be in sync with \ref TEST_APP_ENABLE_SW_CRC.
 */
#define TEST_APP_ENABLE_HW_CRC  1

/*! \brief Compile time option: Perform CRC calculation in SW if set to '1'.
 *
 * \details Compile time option of the serial protocol module:
 *          Use "crc16" module to perform CRC calculation if set to '1'.
 *          Disable this option by setting to '0'.
 *          This option must be in sync with \ref TEST_APP_ENABLE_HW_CRC.
 */
#define TEST_APP_ENABLE_SW_CRC  0

#if (TEST_APP_ENABLE_HW_CRC && TEST_APP_ENABLE_SW_CRC)
#error "Both HW and SW CRC enabled. Choose only one of the possibilities!"
#elif (TEST_APP_ENABLE_HW_CRC == 0) && (TEST_APP_ENABLE_SW_CRC == 0)
#error "Both HW and SW CRC disabled. Choose at least one of the possibilities!"
#endif

/* Do the required includes according to compile time option choosen by user. */
#if TEST_APP_ENABLE_SW_CRC
#include "crc16.h"
#elif TEST_APP_ENABLE_HW_CRC
#include "hal_crc.h"
#endif

/* ===========================================================================*/
/*                  Enums/typedefs                                            */
/*=========================================================================== */
typedef enum TEST_APP_STATE
{
  E_TEST_APP_TX_DATA,

  E_TEST_APP_RX_DATA

} e_test_app_state_t ;

/* ===========================================================================*/
/*                  Global variables                                          */
/* ===========================================================================*/
s_hal_uart_ctx_t *gps_uart;
s_sproto_ctx_t gs_serial;
uint8_t gc_testData[TEST_APP_TEST_DATA_LEN];
e_test_app_state_t ge_testState;
uint32_t gi_testRepetitionCounter;
uint8_t gac_serialProtocolBuffer[TEST_APP_SERIAL_PROTOCOL_BUF_LEN];
uint32_t g_errorCounter;

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

#if TEST_APP_ENABLE_HW_CRC
  if(b_return)
  {
    /* In case HW CRC is used, make sure that this is called prior to any
     * sproto action! sproto expects the HAL_CRC module ready to be used! */
    b_return = hal_crc_init();
  }
#endif

  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
static void loc_test_init(void)
{
  uint16_t i = 0;

  /* Fill our test data array with simple data pattern. */
  for (i = 0; i < TEST_APP_TEST_DATA_LEN; ++i)
  {
    gc_testData[i] = (uint8_t) (i % 256);
  }

  /* After startup of the test-app we transmit data. */
  ge_testState = E_TEST_APP_TX_DATA;
  /* Lets count our test repetitions. We start counting with 1 here. */
  gi_testRepetitionCounter = 1;

  sproto_init(&gs_serial,                                   /* sproto context */
      loc_serialWrite, loc_serialRead, loc_serialRxDataRdy, /* Serial Data Cb */
      loc_serialError,                                      /* Error Cb */
      loc_serialCrc,                                        /* CRC Cb */
      gac_serialProtocolBuffer, TEST_APP_SERIAL_PROTOCOL_BUF_LEN);

  g_errorCounter = 0;

  /* For the sake of this test app, lets test our CRC algorithm(s) with some
   * test data patterns. */
  if( !loc_testCrcCalculation() )
  {
    loc_test_fail();
  }

  return;
}/* loc_test_init() */


/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
  /* To run the test we allways need to run() the serial protocol module! */
  while( sproto_run(&gs_serial) );

  switch (ge_testState)
  {
    case E_TEST_APP_TX_DATA:
      if( sproto_sendFrame(&gs_serial, gc_testData, TEST_APP_TEST_DATA_LEN) )
      {
        ge_testState = E_TEST_APP_RX_DATA;
      }
      else
      {
        loc_test_fail();
      }
      break;

    case E_TEST_APP_RX_DATA:
      /* We actually do nothing here and wait for the RX event to come that
       * is served in loc_serialRxDataRdy(). */
      break;

    default:
      break;
  }


  return;
}


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


static bool loc_testCrcCalculation(void)
{
  bool b_return = false;
  uint16_t crc_result = 0;

  /* Our test data patterns. */
  uint8_t const testData0[] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39 };
  uint8_t const testData1[] = { 0x12, 0x34, 0x56, 0x70 };
  uint8_t const testData2[] = { 0x5A, 0x26, 0x19, 0x77 };

#if TEST_APP_ENABLE_SW_CRC
  /*
   * Test the SW implementation that is provided by crc16.h
   */
  {
    crc_result = crcFast(testData0, sizeof(testData0));
    if( !(crc_result == 0x29B1) )
    {
      return b_return;
    }

    crc_result = crcFast(testData1, sizeof(testData1));
    if( !(crc_result == 0xB1E4) )
    {
      return b_return;
    }

    crc_result = crcFast(testData2, sizeof(testData2));
    if( !(crc_result == 0x1AAD) )
    {
      return b_return;
    }
  }

#elif TEST_APP_ENABLE_HW_CRC
  /*
   * Test the HW implementation that is provided by hal_crc.h
   */
  {
    crc_result = hal_crc_calculate(testData0, sizeof(testData0));
    if( !(crc_result == 0x29B1) )
    {
      return b_return;
    }

    crc_result = hal_crc_calculate(testData1, sizeof(testData1));
    if( !(crc_result == 0xB1E4) )
    {
      return b_return;
    }

    crc_result = hal_crc_calculate(testData2, sizeof(testData2));
    if( !(crc_result == 0x1AAD) )
    {
      return b_return;
    }
  }
#endif

  /* CRC implementation works fine if we end up here! */
  b_return = true;
  return b_return;
}


/* ===========================================================================*/
/*                  loc_serialRead()                                          */
/* ===========================================================================*/
static uint16_t loc_serialRead(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_read(gps_uart, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_serialWrite()                                         */
/* ===========================================================================*/
static uint16_t loc_serialWrite(const uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_write(gps_uart, (uint8_t *)pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_serialRxDataRdy()                                     */
/* ===========================================================================*/
static void loc_serialRxDataRdy(uint8_t *pc_data, uint16_t ui16_len)
{
  /* Time to analzye this test run. The received data must match the
   * transmitted data. */
  if( ui16_len == TEST_APP_TEST_DATA_LEN )
  {
    if( memcmp(gc_testData, pc_data, ui16_len) == 0 )
    {
      if(gi_testRepetitionCounter < TEST_APP_TEST_REPETITIONS)
      {
        /* Lets start another test round. */
        gi_testRepetitionCounter++;
        ge_testState = E_TEST_APP_TX_DATA;
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
  }
  else
  {
    /* Test failed! */
    loc_test_fail();
  }/* if() ... else */

  return;
}

/* ===========================================================================*/
/*                  loc_serialError()                                         */
/* ===========================================================================*/
static void loc_serialError(E_SERIAL_PROTOCOL_ERROR_t e_errType)
{
  switch (e_errType)
  {
    case E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID:
      g_errorCounter++;
      break;
    case E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW:
      loc_test_fail();
      break;
    default:
      break;
  }
  return;
}

/* ===========================================================================*/
/*                  loc_serialCrc()                                           */
/* ===========================================================================*/
static uint16_t loc_serialCrc(const uint8_t *pc_data, uint16_t ui16_len)
{
  uint16_t crc_result = 0;
  #if TEST_APP_ENABLE_SW_CRC
  crc_result = crcFast(pc_data, ui16_len);
  #elif TEST_APP_ENABLE_HW_CRC
  crc_result = hal_crc_calculate(pc_data, ui16_len);
  #endif
  return crc_result;
}

/* ===========================================================================*/
/*                  main()                                                    */
/* ===========================================================================*/
int main(void)
{
  if( loc_hal_init() )
  {
    loc_test_init();

    while(1)
    {
      loc_test_run();
    }
  }

	return 0;
}
