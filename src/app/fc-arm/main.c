/* Copyright (C) 2015-2017
 *
 * main.c
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
#include <stdlib.h>
#include "hal_spi.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_dma.h"
#include "hal_crc.h"
#include "hal_pps.h"
#include "tube-angle-sensor.h"

/* Protobuf includes */
#include <pb_encode.h>
#include <pb_decode.h>

#include "sproto.h"

#include "BetCALL.pb.h"
#include "BetCONF.pb.h"

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_hal_init(void);
static bool loc_test_init(void);
static void loc_test_run(void);

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */
/*! Defines the amount of repititions/loops for the test.
 *  For each repitition the amount of test data is \ref TEST_APP_TEST_DATA_LEN.
 *  This number shall not exceed 0xFFFFFFFF as 32bit variables are used. */
#define TEST_APP_TEST_REPETITIONS   100

/*! Delay between SENDs of the test app in milliseconds. */
#define TEST_APP_TEST_DELAY   10

/*! Clockfrequency of SCK line in MHz. */
#define TEST_APP_SPI_SPEED 2000000

/*! Defines the UART port the GS is connected to. Must be of type
 *  \ref E_HAL_UART_PORT_t. */
#define MFW_UART_PORT_TO_FC		E_HAL_UART_PORT_1
/*! Defines the UART baudrate to be used between GS and FC. */
#define MFW_UART_BAUD_RATE_TO_FC    921600
/*! Defines the port of the enable pin of TXS0108 Level Shifter. */
#define MFW_LSHIFT_EN_PORT		E_HAL_GPIO_PORT_K
/*! Defines the the enable pin of TXS0108 Level Shifter. */
#define MFW_LSHIFT_EN_PIN_UART    HAL_GPIO_PIN_2
/*! Defines the the enable pin of SPI Level Shifter. */
#define MFW_LSHIFT_EN_PIN_SPI    HAL_GPIO_PIN_3
/*! As mfw provides the buffer to the serial protocol module (for communication
 *  with groundstation), this number defines the buffer length of the serial
 *  protocol RX buffer. Make sure this value fits to the maximum legth of a
 *  protobuf message transmitted from GS to FC! */
#define MFW_FLIGHTCONT_RX_BUF_LEN      1024

#define TEST_APP_ERROR_LED_INIT() \
    hal_gpio_setDir(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_0, E_HAL_GPIO_DIR_OUT)
#define TEST_APP_ERROR_LED_OFF() \
    hal_gpio_write(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_0, ~HAL_GPIO_PIN_0)
#define TEST_APP_ERROR_LED_ON() \
    hal_gpio_write(E_HAL_GPIO_PORT_L, HAL_GPIO_PIN_0, HAL_GPIO_PIN_0)

/* ============================================================================
 * Configuration for protocol buffers.
 */
/*! Size of the output stream buffer, i.e. the buffer where nanopb writes the
 *  encoded protocol buffer message to. */
#define MFW_PB_OSTREAM_BUF_LEN 2000


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
/*! Global context of the serial protocol (= our instance of the protocol) that
 *  is used for communication with groundstation. In detail, this sproto
 *  instance uses the HAL UART port that is connected to groundstation,
 *  see \ref gps_uartToGs. */
s_sproto_ctx_t gs_flightcont;
/*! Buffer that is passed to the serial protocol module (as its RX data memory)
 *  for communication with groundstation. */
uint8_t gac_flightcontRxBuf[MFW_FLIGHTCONT_RX_BUF_LEN];
/*! Global context of the output stream to be used by protocol buffers (nanopb) */
pb_ostream_t g_ostream;
/*! The output stream buffer, i.e. the buffer where nanopb writes the
 *  encoded protocol buffer message to. */
uint8_t gac_ostreamBuf[MFW_PB_OSTREAM_BUF_LEN];
/*! Global context pointer to for HAL UART port (RS232 port) that is connected
 *  to groundstation. */
s_hal_uart_ctx_t *gps_uartToFC;

/*! Global context of the input stream to be used by protocol buffers (nanopb) */
pb_istream_t g_istream;

BetCONF g_rxBetConf;
/*! Global, static constant variable for initialization of pb BetCONF using
 * default values as defined in proto file.
 */
const BetCONF g_betConfDefault = BetCONF_init_default;
/*! Global boolean signaling that a betCONF message was received. */
bool gb_betConfReceived;

BetCALL tas_msg = {};

const BetCALL g_tubeAngleDefault = BetCALL_init_default;

uint32_t g_errorCounter;
uint32_t g_errorIncalidCrcCounter;

uint32_t elevation;
uint32_t rotation;

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
bool loc_hal_init(void);
bool loc_test_init(void);
void loc_test_run(void);
static uint16_t loc_cbReadFromFC(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_cbWriteToFC(const uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbFCRxDataRdy(uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbSerialToFcError(E_SERIAL_PROTOCOL_ERROR_t e_errType);
static uint16_t loc_cbSerialToFcCrc(const uint8_t *pc_data, uint16_t ui16_len);
bool loc_initCom(void);
static bool loc_decodeBetConf(uint8_t *pc_data, uint16_t i_len, BetCONF *p_pbConf);
bool loc_writeTubeAngleToFC(BetCALL *p_txTubeAngle);

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
bool loc_hal_init(void)
{
  if(!hal_mcu_init()) {
	  return false;
  }

  if(!hal_dma_init()) {
      return false;
  }

  if(!hal_crc_init()) {
      return false;
  }

  if(!hal_pps_init()){
    return false;
  }

  if(!hal_timer_init()){
    return false;
  }

  // Initialize SPI port
  if(!tube_angle_sens_init(TEST_APP_SPI_SPEED)) {
	  return false;
  }

  /* LED1 (Yellow-Green): PL0 */
  if( !TEST_APP_ERROR_LED_INIT() )
  {
	  return false;
  }
  TEST_APP_ERROR_LED_OFF();

  // initialize the timer
  return true;
}/* loc_hal_init() */

/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
  /* Fill in our test data. */
	bool b_return = loc_initCom();
	memset( (void*) gac_ostreamBuf, 0, MFW_PB_OSTREAM_BUF_LEN);
  g_errorCounter = 0;
  tas_msg = g_tubeAngleDefault;
  g_errorIncalidCrcCounter = 0;

	return b_return;
}/* loc_test_init() */

/* ===========================================================================*/
/*                  loc_cbReadFromGs()                                        */
/* ===========================================================================*/
static uint16_t loc_cbReadFromFC(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_read(gps_uartToFC, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbWriteToGs()                                         */
/* ===========================================================================*/
static uint16_t loc_cbWriteToFC(const uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_write(gps_uartToFC, (uint8_t *)pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbGsRxDataRdy()                                       */
/* ===========================================================================*/
static void loc_cbFCRxDataRdy(uint8_t *pc_data, uint16_t ui16_len)
{
  /*
   * Getting this callback called signals that a serial protocol message
   * is received successfully. Let's decode it!
   */
  if( loc_decodeBetConf(pc_data, ui16_len, &g_rxBetConf) )
  {
    gb_betConfReceived = true;
  }

  return;
}


/* ===========================================================================*/
/*                  loc_cbSerialToFcError()                                   */
/* ===========================================================================*/
static void loc_cbSerialToFcError(E_SERIAL_PROTOCOL_ERROR_t e_errType)
{
  switch (e_errType)
  {
    case E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID:
      g_errorIncalidCrcCounter++;
      break;
    case E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW:
      break;
    default:
      break;
  }
  return;
}


/* ===========================================================================*/
/*                  loc_cbSerialToFcCrc()                                     */
/* ===========================================================================*/
static uint16_t loc_cbSerialToFcCrc(const uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_crc_calculate(pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_decodeBetConf()                                       */
/* ===========================================================================*/
static bool loc_decodeBetConf(uint8_t *pc_data, uint16_t i_len, BetCONF *p_pbConf)
{
  bool b_return = false;

  /* Initialize the istream context for each decoding! */
  g_istream = pb_istream_from_buffer( pc_data, i_len );

  /* Initialize betconf struct with default values before decoding. */
  *p_pbConf = g_betConfDefault;

  if( pb_decode( &g_istream, BetCONF_fields, p_pbConf) )
  {
    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_initUartComToGs()                                     */
/* ===========================================================================*/
bool loc_initCom(void)
{
  bool b_return = true;

  /* 1st, set OE high for TXS0108 Level Shifter. */
  hal_gpio_setDir(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN_UART, E_HAL_GPIO_DIR_OUT);
  hal_gpio_write(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN_UART, MFW_LSHIFT_EN_PIN_UART);
  /* 1st, set OE high for SPI Level Shifter. */
  hal_gpio_setDir(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN_SPI, E_HAL_GPIO_DIR_OUT);
  hal_gpio_write(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN_SPI, MFW_LSHIFT_EN_PIN_SPI);


  /* 2nd, initialize the corresponding UART port for communication to GS. */
  gps_uartToFC = hal_uart_init( MFW_UART_PORT_TO_FC, MFW_UART_BAUD_RATE_TO_FC );
  if(gps_uartToFC == NULL)
  {
    b_return = false;
  }

  /* 3rd, init the serial protocol module used for communciation to FC. */
  if( b_return )
  {
    b_return = sproto_init(&gs_flightcont,                       /* context   */
        loc_cbWriteToFC, loc_cbReadFromFC, loc_cbFCRxDataRdy,  /* callbacks */
        loc_cbSerialToFcError, loc_cbSerialToFcCrc,
        gac_flightcontRxBuf, MFW_FLIGHTCONT_RX_BUF_LEN);           /* buffer    */
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_writeBetCallToGs()                                    */
/* ===========================================================================*/
bool loc_writeTubeAngleToFC(BetCALL *p_txTubeAngle)
{
  bool b_return = false;

  /* Initialize the output context for each encoding! */
  g_ostream = pb_ostream_from_buffer( gac_ostreamBuf, MFW_PB_OSTREAM_BUF_LEN );

  if( pb_encode( &g_ostream, BetCALL_fields, p_txTubeAngle) )
  {
    /* Write the encoded buffer through serial protocol to GS. */
    if( sproto_sendFrame(&gs_flightcont, gac_ostreamBuf, g_ostream.bytes_written) )
    {
      b_return = true;
    }
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_test_run()                                            */
/* ===========================================================================*/
void loc_test_run(void)
{
	tube_angle_sens_run();
	if (tube_angle_sensor_get(&(tas_msg.TAS_elevation), &(tas_msg.TAS_pitch)))
	{
		elevation = tas_msg.TAS_elevation.angle_raw;
		rotation = tas_msg.TAS_pitch.angle_raw;
		elevation +=0;
		rotation += 0;
		if( !loc_writeTubeAngleToFC(&tas_msg) )
		{
		  /* Error occured! Turn on LED D3 as error indicator. */
		  g_errorCounter++;
		  TEST_APP_ERROR_LED_ON();
		}
	}
}

int main(void)
{
	if( loc_hal_init() )
	{
    if( loc_test_init() )
    {
      while(true)
      {
        loc_test_run();
      }
    }
	}
	
	TEST_APP_ERROR_LED_ON();
	while(1);
}
