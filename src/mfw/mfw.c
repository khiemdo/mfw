/* Copyright (C) 2015-2017
 *
 * mfw.c
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

#include <stdio.h>

#include "mfw.h"

/* First include the config header that en/disables mfw components. */
#include "mfw-global-config.h"

#include "hal_uart.h"
#include "hal_crc.h"
#include "hal_pps.h"
#include "hal_gpio.h"

/* Serial protocol is used for RS232/RS485 communication to groundstation. */
#include "sproto.h"
/* Timing module for timestamps and timers in general. */
#include "timing.h"
/* Include software CRC module. */
#include "crc16.h"
/* PPS module for precision timestamps. */
#include "pps/pps.h"
/* Bus manager module for managing multiple devices on one I�C port. */
#include "i2c-bus-manager.h"
/* Logging module for writing string to protobuf. */
#include "log.h"
/* Build string from version file. */
#include "version.h"

/* Protobuf includes */
#include <pb_encode.h>
#include <pb_decode.h>
#include "BetCOM.pb.h"
#include "Pps.pb.h"

/* Actuator includes */
#include "power/servopower.h"
#include "servo-dude/servo-dude.h"
#include "ncp5623/ncp5623.h"

/* Sensor includes */
#include "mpu9250/mpu9250.h"
#include "bno055/bno055.h"
#include "sht25/sht25.h"
#include "ads1115/ads1115.h"
#include "mpl/mpl.h"
#include "tube-angle-sensor/tube-angle-sensor.h"
#include "satellady/satellady.h"

#if MFW_PB_FORWARDER_EN
/* Include only required if pb forwarder is enabled. */
#include "pb-forwarder.h"
#endif

#if MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
#include "mfw-socket.h"
#endif

/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/

#if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
/*! Global context of the serial protocol (= our instance of the protocol) that
 *  is used for communication with groundstation. In detail, this sproto
 *  instance uses the HAL UART port that is connected to groundstation,
 *  see \ref gps_uartToGs. */
s_sproto_ctx_t gs_groundst;

/*! Buffer that is passed to the serial protocol module (as its RX data memory)
 *  for communication with groundstation. */
uint8_t gac_groundstRxBuf[MFW_GROUNDST_RX_BUF_LEN];

/*! Global context pointer to for HAL UART port (RS232 port) that is connected
 *  to groundstation. */
s_hal_uart_ctx_t *gps_uartToGs;

uint32_t g_serialInvalidCrcCounter;
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_UART_EN */

#if MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
/*! Buffer that is passed to the mfw socket module (as its RX data memory)
 *  for communication with groundstation. */
uint8_t gac_groundstRxBuf[MFW_GROUNDST_RX_BUF_LEN];
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN */

/*
 * Global protocol buffer variables
 */
/*! Global context of the output stream to be used by protocol buffers (nanopb) */
pb_ostream_t g_ostream;

/*! Global context for a debug output stream. */
#if MFW_DEBUG_ENCODE_PB_ONLY_ONCE
pb_ostream_t g_ostreamDebug;
bool b_UseOstreamDebug;
#endif

/*! The output stream buffer, i.e. the buffer where nanopb writes the
 *  encoded protocol buffer message to. */
uint8_t gac_ostreamBuf[MFW_PB_OSTREAM_BUF_LEN];

/*! Global, static variable for transmission of pb BetCOM message in
 *  function loc_writeBetComToGs(). If you use a local variable make sure the
 *  stack size is configured properly within project settings!
 */
BetCOM g_txBetCom;

/*! Global, static variable containing BetCOM message for debugging.*/
#if MFW_DEBUG_REPEAT_FIRST_MESSAGE
BetCOM g_txBetComDebug;
bool b_UseBetComDebug;
#endif

/*! Global, static constant variable for initialization of pb BetCom using
 * default values as defined in proto file.
 */
const BetCOM g_betComDefault = BetCOM_init_default;

/*! Global, static constant variables for software version using default values
 * as defined in proto file.
 */
const Firmware g_FirmwareDefault = Firmware_init_set;
#if FC
const Version g_VersionDefault = FC_Version_init_set;
#endif
#if FC_GPS
const Version g_VersionDefault = FC_GPS_Version_init_set;
#endif
#if ARM
const Version g_VersionDefault = ARM_Version_init_set;
#endif
#if ARM_GPS
const Version g_VersionDefault = ARM_GPS_Version_init_set;
#endif

/*! Global context of the input stream to be used by protocol buffers (nanopb) */
pb_istream_t g_istream;

/*! Global, static variable for reception/decodeing of pb BetCom message in
 *  function loc_decodeBetCom(). If you use a local variable make sure the stack
 *  size is configured properly within project settings!
 */
BetCOM g_rxBetCom;

Report g_ReportDefault = Report_init_default;
Timestamp g_TimestampDefault = Timestamp_init_default;

/*! Global boolean signaling that a BetCALL message was received. */
bool gb_betCallReceived;

/*! Global boolean signaling that a BetCONF message was received. */
bool gb_betConfReceived;

/*! Global boolean signaling that a BetPUSH message was received. */
bool gb_betPushReceived;

/*! Global counter used for transmission of "package_counter" in PackageId subfield. */
uint32_t g_BetcomPackageCounter;
uint32_t g_BetcallPackageCounter;
uint32_t g_BetconfPackageCounter;

/*
 * Global actuator variables
 */
#if MFW_ACT_SERVO_DUDE_ELEVATOR_EN
s_servo_dude_ctx_t *gps_servoElevator;
#endif
#if MFW_ACT_SERVO_DUDE_RUDDER_EN
s_servo_dude_ctx_t *gps_servoRudder;
#endif
#if MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN
s_servo_dude_ctx_t *gps_servoFlapLeft;
#endif
#if MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN
s_servo_dude_ctx_t *gps_servoFlapRight;
#endif
#if MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN
s_servo_dude_ctx_t *gps_servoAileronLeft;
#endif
#if MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN
s_servo_dude_ctx_t *gps_servoAileronRight;
#endif
#if MFW_ACT_NCP5623_EN
bool gps_ncp5623 = false;
#endif
#if MFW_SENS_ADS1115_EN
s_ads1115_ctx_t *gs_ads1_ctx;
s_ads1115_ctx_t *gs_ads2_ctx;
s_ads1115_ctx_t *gs_ads3_ctx;
#endif

/*! Timer used as intervall of sending BetCALL messages to GS. */
uint32_t g_timerSendBetCall;

/*! Counts the situation that a mesasge sent from GS to FC was not decoded
 * successfully by nanopb.*/
uint32_t g_pbDecodeFailCounter;

#if MFW_SYSINFO_MESSAGE_EN
  bool g_Sysinfo_data_available;
  Sysinfo pb_Sysinfo;
  const Sysinfo g_SysinfoDefault = Sysinfo_init_default;

  #if MFW_LOOP_COUNTER_EN
    uint32_t g_mainLoopCounter;
    uint32_t g_timerLoopCounter;
  #endif
#endif

HumidityConf humconf; //Debug

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static void loc_mfw_error(void);

#if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
static bool loc_initUartComToGs(void);
static uint16_t loc_cbReadFromGs(uint8_t *pc_data, uint16_t ui16_len);
static uint16_t loc_cbWriteToGs(const uint8_t *pc_data, uint16_t ui16_len);
static void loc_cbSerialToGsError(E_SERIAL_PROTOCOL_ERROR_t e_errType);
static uint16_t loc_cbSerialToGsCrc(const uint8_t *pc_data, uint16_t ui16_len);
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_UART_EN */

static void loc_cbGsRxDataRdy(uint8_t *pc_data, uint16_t ui16_len);

#if MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
static bool loc_initEthernetToGs(void);
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN */

static bool loc_setPackageId(PackageId *p_packageId, uint32_t *packageCounter);
static bool loc_writeBetComToGs(BetCOM *p_txBetCom, Timestamp *sent_last);
static bool loc_decodeBetCom(uint8_t *pc_data, uint16_t i_len, BetCOM *p_pbBetCom);
static bool loc_writeString(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
#if MFW_PB_FORWARDER_EN
static bool loc_forwardProtobufToGs(uint8_t *pc_data, uint16_t i_len);
#endif

static bool loc_initSensors(void);
static void loc_runSensors(void);
static void loc_setSensorsConf(void);
static void loc_getSensorData(void);

#if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
static bool loc_initActuators(void);
static void loc_runActuators(void);
static void loc_setActuators(void);
#endif

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/
static void loc_mfw_error(void)
{
  static uint32_t errorCount = 0;

  /* For now just increase an error counter as error handling. */
  errorCount++;
}


#if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
/* ===========================================================================*/
/*                  loc_initUartComToGs()                                     */
/* ===========================================================================*/
static bool loc_initUartComToGs(void)
{
  bool b_return = true;

  /* 1st, set OE high for TXS0108 Level Shifter. */
  hal_gpio_setDir(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN, E_HAL_GPIO_DIR_OUT);
  hal_gpio_write(MFW_LSHIFT_EN_PORT, MFW_LSHIFT_EN_PIN, MFW_LSHIFT_EN_PIN);

  /* 2nd, initialize the corresponding UART port for communication to GS. */
  gps_uartToGs = hal_uart_init( MFW_UART_PORT_TO_GS, MFW_UART_BAUD_RATE_TO_GS );
  if(gps_uartToGs == NULL)
  {
    b_return = false;
  }

  /* 3rd, init the serial protocol module used for communication to GS. */
  if( b_return )
  {
    b_return = sproto_init(&gs_groundst,                       /* context   */
        loc_cbWriteToGs, loc_cbReadFromGs, loc_cbGsRxDataRdy,  /* callbacks */
        loc_cbSerialToGsError, loc_cbSerialToGsCrc,
        gac_groundstRxBuf, MFW_GROUNDST_RX_BUF_LEN);           /* buffer    */
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_cbReadFromGs()                                        */
/* ===========================================================================*/
static uint16_t loc_cbReadFromGs(uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_read(gps_uartToGs, pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbWriteToGs()                                         */
/* ===========================================================================*/
static uint16_t loc_cbWriteToGs(const uint8_t *pc_data, uint16_t ui16_len)
{
  return hal_uart_write(gps_uartToGs, (uint8_t *)pc_data, ui16_len);
}

/* ===========================================================================*/
/*                  loc_cbSerialToGsError()                                   */
/* ===========================================================================*/
static void loc_cbSerialToGsError(E_SERIAL_PROTOCOL_ERROR_t e_errType)
{
  switch (e_errType)
  {
    case E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID:
      g_serialInvalidCrcCounter++;
      pb_Sysinfo.has_crc_error_counter = true;
      pb_Sysinfo.crc_error_counter = g_serialInvalidCrcCounter;
      g_Sysinfo_data_available |= true;
      break;
    case E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW:
      loc_mfw_error();
      break;
    default:
      break;
  }
  return;
}

#if MFW_SOFTWARE_CRC_EN
/* ===========================================================================*/
/*                  loc_cbSerialToGsCrc()                                     */
/* ===========================================================================*/
static uint16_t loc_cbSerialToGsCrc(const uint8_t *pc_data, uint16_t ui16_len)
{
  uint16_t ui16_return;

  //CPUcpsid();
  ui16_return = crcFast(pc_data, ui16_len);
  //CPUcpsie();

  return ui16_return;
}

#else
/* ===========================================================================*/
/*                  loc_cbSerialToGsCrc()                                     */
/* ===========================================================================*/
static uint16_t loc_cbSerialToGsCrc(const uint8_t *pc_data, uint16_t ui16_len)
{
  uint16_t ui16_return;

  //CPUcpsid();
  ui16_return = hal_crc_calculate(pc_data, ui16_len);
  //CPUcpsie();

  return ui16_return;
}
#endif /* MFW_SOFTWARE_CRC */
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_UART_EN */

/* ===========================================================================*/
/*                  loc_cbGsRxDataRdy()                                       */
/* ===========================================================================*/
static void loc_cbGsRxDataRdy(uint8_t *pc_data, uint16_t ui16_len)
{
  /*
   * Getting this callback called signals that a serial protocol message (sent
   * by the groundstation) is received successfully. Let's decode it!
   */
#if MFW_PB_FORWARDER_EN
  forwardProtobufToArm(pc_data, ui16_len);
#endif

  if( loc_decodeBetCom(pc_data, ui16_len, &g_rxBetCom) )
  {
    if( g_rxBetCom.has_betpush )
    {
      gb_betPushReceived = true;
    }
    if( g_rxBetCom.has_betconf )
    {
      gb_betConfReceived = true;
    }
    if( g_rxBetCom.has_betcall )
    {
      gb_betCallReceived = true;
    }
  }
  else
  {
    g_pbDecodeFailCounter++;
  }

  return;
}

#if MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
/* ===========================================================================*/
/*                  loc_initEthernetToGs()                                    */
/* ===========================================================================*/
static bool loc_initEthernetToGs(void)
{
  bool b_return = false;

  uint32_t ipAddr     = (   (MFW_IP_ADDR_1 << 24) | \
                            (MFW_IP_ADDR_2 << 16) | \
                            (MFW_IP_ADDR_3 <<  8) | \
                            (MFW_IP_ADDR_4      )
                        );
  uint32_t netmask    = (   (MFW_NET_MASK_1 << 24) | \
                            (MFW_NET_MASK_2 << 16) | \
                            (MFW_NET_MASK_3 <<  8) | \
                             MFW_NET_MASK_4
                        );
  uint32_t gwIpAddr   = (   (MFW_GW_IP_ADDR_1 << 24) | \
                            (MFW_GW_IP_ADDR_2 << 16) | \
                            (MFW_GW_IP_ADDR_3 <<  8) | \
                            (MFW_GW_IP_ADDR_4      )
                        );
  uint32_t destIpAddr = ( (MFW_DEST_IP_ADDR_1 << 24) | \
                          (MFW_DEST_IP_ADDR_2 << 16) | \
                          (MFW_DEST_IP_ADDR_3 <<  8) | \
                          (MFW_DEST_IP_ADDR_4      )
                        );

  /* Init low-level ETH MAC/PHY and IP stack. */
  b_return = mfw_socket_init(ipAddr, netmask, gwIpAddr);

  if(b_return)
  {
    b_return = mfw_socket_udpStart( MFW_UDP_PORT,
                                    destIpAddr, MFW_UDP_PORT,
                                    gac_groundstRxBuf, MFW_GROUNDST_RX_BUF_LEN,
                                    loc_cbGsRxDataRdy
                                  );
  }

  return b_return;
}

#endif /* MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN */

/* ===========================================================================*/
/*                  loc_writeString()                                         */
/* ===========================================================================*/
bool loc_writeString(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    if (!pb_encode_tag_for_field(stream, field))
        return false;

    return pb_encode_string(stream, *arg, strlen(*arg));
}

/* ===========================================================================*/
/*                  loc_setPackageId()                                        */
/* ===========================================================================*/
static bool loc_setPackageId(PackageId *p_packageId, uint32_t *packageCounter)
{
  bool b_return = false;

  if( p_packageId )
  {
    /* Set property "package counter" */
    p_packageId->has_package_counter = true;
    p_packageId->package_counter = *packageCounter;
    /* Increment counter right after transmission. */
    (*packageCounter)++;

    /* Set property "origin" */
    p_packageId->has_origin = true;
    #if FC
    p_packageId->origin = PackageId_Origin_FC;
    #elif FC_GPS
    p_packageId->origin =  PackageId_Origin_FC_GPS;
    #elif ARM
    p_packageId->origin = PackageId_Origin_ARM;
    #elif ARM_GPS
    p_packageId->origin =  PackageId_Origin_ARM_GPS;
    #endif
    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_writeBetComToGs()                                     */
/* ===========================================================================*/
static bool loc_writeBetComToGs(BetCOM *p_txBetCom, Timestamp *sent_last)
{
  bool b_return = false;

  #if MFW_DEBUG_ENCODE_PB_ONLY_ONCE
  if(b_UseOstreamDebug)
  {
    if(sent_last)
      hal_pps_getTimestamp(sent_last);
    #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
    /* Write the encoded buffer through serial protocol to GS. */
    b_return = sproto_sendFrame(&gs_groundst, gac_ostreamBuf, g_ostreamDebug.bytes_written);

    #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
    b_return = mfw_socket_udpSend(gac_ostreamBuf, g_ostreamDebug.bytes_written);
    #endif
  }
  else
  {
    g_ostreamDebug = pb_ostream_from_buffer( gac_ostreamBuf, MFW_PB_OSTREAM_BUF_LEN );
    b_UseOstreamDebug = true;

    if( pb_encode( &g_ostreamDebug, BetCOM_fields, p_txBetCom) )
    {
      if(sent_last)
        hal_pps_getTimestamp(sent_last);
      #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
      /* Write the encoded buffer through serial protocol to GS. */
      b_return = sproto_sendFrame(&gs_groundst, gac_ostreamBuf, g_ostreamDebug.bytes_written);

      #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
      b_return = mfw_socket_udpSend(gac_ostreamBuf, g_ostreamDebug.bytes_written);
      #endif
    }
  }
  #else
  /* Initialize the output context for each encoding! */
  g_ostream = pb_ostream_from_buffer( gac_ostreamBuf, MFW_PB_OSTREAM_BUF_LEN );

  if( pb_encode( &g_ostream, BetCOM_fields, p_txBetCom) )
  {
    if(sent_last)
      hal_pps_getTimestamp(sent_last);
    #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
    /* Write the encoded buffer through serial protocol to GS. */
    b_return = sproto_sendFrame(&gs_groundst, gac_ostreamBuf, g_ostream.bytes_written);

    #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
    b_return = mfw_socket_udpSend(gac_ostreamBuf, g_ostream.bytes_written);
    #endif
  }
  #endif /* MFW_DEBUG_ENCODE_PB_ONLY_ONCE */

  return b_return;
}

/* ===========================================================================*/
/*                  loc_decodeBetCom()                                        */
/* ===========================================================================*/
static bool loc_decodeBetCom(uint8_t *pc_data, uint16_t i_len, BetCOM *p_pbBetCom)
{
  bool b_return = false;

  /* Initialize the istream context for each decoding! */
  g_istream = pb_istream_from_buffer( pc_data, i_len );

  /* Initialize betcom struct with default values before decoding. */
  *p_pbBetCom = g_betComDefault;

  if( pb_decode( &g_istream, BetCOM_fields, p_pbBetCom) )
  {
    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*                  loc_forwardProtobufToGs()                                 */
/* ===========================================================================*/
#if MFW_PB_FORWARDER_EN
static bool loc_forwardProtobufToGs(uint8_t *pc_data, uint16_t i_len)
{
  bool b_return = false;
  #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
  /* Simply forward given data (that is actually protobuf message) to the GS. */
  if( sproto_sendFrame(&gs_groundst, pc_data, i_len) )
  {
    b_return = true;
  }
  #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
  b_return = mfw_socket_udpSend(pc_data, i_len);
  #endif

  return b_return;
}
#endif

/* ===========================================================================*/
/*                  loc_initSensors()                                         */
/* ===========================================================================*/
static bool loc_initSensors(void)
{
  bool b_return = true;

  #if MFW_SENS_MPU9250_EN
  if(b_return)
  {
    b_return = mpu9250_init( MFW_SENS_MPU9250_SPI_PORT,
                           MFW_SENS_MPU9250_SPI_SPEED );
  }
  #endif

  #if MFW_SENS_BNO055_EN
  if(b_return)
  {
    b_return = bno055_init();
  }
  #endif

  #if SATELLADY_EN
  if(b_return)
  {
    b_return = satellady_init();
  }
  #endif

  #if MFW_SENS_PPS_EN
  if(b_return)
  {
    b_return = pps_init();
  }
  #endif

  #if MFW_SENS_TUBE_ANGLE_EN
  if(b_return)
  {
    b_return = tube_angle_sens_init(MFW_SENS_TUBE_ANGLE_SPI_SPEED);
  }
  #endif

  #if MFW_SENS_SHT25_EN
  if(b_return)
  {
    b_return = SHT25_init();
    //Testcode
    humconf.has_basic_conf = true;

    humconf.basic_conf.has_activated = true;
    humconf.basic_conf.activated = true;

    humconf.basic_conf.has_rate = true;
    humconf.basic_conf.rate = 100;
    // END Testcode

  }
  #endif

#if MFW_SENS_MPL_EN
if(b_return)
{
  b_return = MPL_init();

}
#endif

#if MFW_SENS_ADS1115_EN
  if (b_return)
  {
	gs_ads1_ctx = ads1115_init(E_ADS1115_ID_1);
	if (!gs_ads1_ctx)
	{
	  b_return = false;
	}
  gs_ads2_ctx = ads1115_init(E_ADS1115_ID_2);
	if (!gs_ads2_ctx)
	{
	  b_return = false;
	}
  gs_ads3_ctx = ads1115_init(E_ADS1115_ID_3);
	if (!gs_ads3_ctx)
	{
	  b_return = false;
	}
  }
#endif

  /* Initialize all sensore modules here! */

  return b_return;
}

/* ===========================================================================*/
/*                  loc_runSensors()                                          */
/* ===========================================================================*/
static void loc_runSensors(void)
{
  #if SATELLADY_EN
  satellady_run();
  #endif

  #if MFW_SENS_PPS_EN
  pps_run();
  #endif

  #if MFW_SENS_MPU9250_EN
  mpu9250_run();
  #endif

  #if MFW_SENS_BNO055_EN
  bno055_run();
  #endif

  #if MFW_SENS_SHT25_EN
  SHT25_run();
  #endif

  #if MFW_SENS_MPL_EN
  MPL_run();
  #endif


  #if MFW_SENS_TUBE_ANGLE_EN
  tube_angle_sens_run();
  #endif

  #if MFW_SENS_ADS1115_EN
  ads1115_run(gs_ads1_ctx);
  ads1115_run(gs_ads2_ctx);
  ads1115_run(gs_ads3_ctx);
  #endif

  /* Call all the sensor run() functions here! */
  return;
}

/* ===========================================================================*/
/*                  loc_setSensorsConf()                                      */
/* ===========================================================================*/
static void loc_setSensorsConf(void)
{
  /* We need to call setConf() only in case a betCONF message is received. */
  if( gb_betConfReceived )
  {
    gb_betConfReceived = false;

    #if MFW_SENS_PPS_EN
      #if FC
      if(g_rxBetCom.betconf.has_FC_pps)
      {
      pps_setConf(&g_rxBetCom.betconf.FC_pps);
      }
      #endif
      #if FC_GPS
      if(g_rxBetCom.betconf.has_FC_GPS_pps)
      {
      pps_setConf(&g_rxBetCom.betconf.FC_GPS_pps);
      }
      #endif
      #if ARM
      if(g_rxBetCom.betconf.has_ARM_pps)
      {
      pps_setConf(&g_rxBetCom.betconf.ARM_pps);
      }
      #endif
      #if ARM_GPS
      if(g_rxBetCom.betconf.has_ARM_GPS_pps)
      {
      pps_setConf(&g_rxBetCom.betconf.ARM_GPS_pps);
      }
      #endif
    #endif

    #if MFW_SENS_MPU9250_EN
    /* Add call to MPU setConf() here! */
    #endif

    #if MFW_SENS_BNO055_EN
    /* Add call to BNO setConf() here! */
    #endif

    #if MFW_SENS_SHT25_EN
    SHT25_setConf (&humconf);
    #endif

    #if FC
    g_rxBetCom.betconf.has_FC_firmware = true;
    g_rxBetCom.betconf.FC_firmware = g_FirmwareDefault;
    g_rxBetCom.betconf.FC_firmware.version = g_VersionDefault;
    #endif
    #if FC_GPS
    g_rxBetCom.betconf.has_FC_GPS_firmware = true;
    g_rxBetCom.betconf.FC_GPS_firmware = g_FirmwareDefault;
    g_rxBetCom.betconf.FC_GPS_firmware.version = g_VersionDefault;
    #endif
    #if ARM
    g_rxBetCom.betconf.has_ARM_firmware = true;
    g_rxBetCom.betconf.ARM_firmware = g_FirmwareDefault;
    g_rxBetCom.betconf.ARM_firmware.version = g_VersionDefault;
    #endif
    #if ARM_GPS
    g_rxBetCom.betconf.has_ARM_GPS_firmware = true;
    g_rxBetCom.betconf.ARM_GPS_firmware = g_FirmwareDefault;
    g_rxBetCom.betconf.ARM_GPS_firmware.version = g_VersionDefault;
    #endif

    g_rxBetCom.betconf.has_generated = hal_pps_getTimestamp( &(g_rxBetCom.betconf.generated) );
    g_rxBetCom.betconf.has_id = loc_setPackageId( &(g_rxBetCom.betconf.id), &g_BetconfPackageCounter );
    g_txBetCom.betconf = g_rxBetCom.betconf;
    g_txBetCom.has_betconf = true;

  }

  return;
}

/* ===========================================================================*/
/*                  loc_getSensorData()                                       */
/* ===========================================================================*/
static void loc_getSensorData(void)
{
  if( timing_isTimedOut(g_timerSendBetCall) )
  {
    /* Restart timer for sending the next betCOM message down to GS. We need
     * to limit the amount of data to not generate more data than the UART
     * can send out.
     */
    g_timerSendBetCall = timing_getTimeout( MFW_INTERVALL_BETCALL_MSG_TO_GS );
    bool b_data_available = false;

    #if MFW_SENS_MPU9250_EN
      #if FC
      if( mpu9250_get( &(g_txBetCom.betcall.FC_MPU) ) )
      {
        /* MPU data was present to we have at least one IMU data set. */
        g_txBetCom.betcall.has_FC_MPU = true;
        b_data_available |= true;
      }
      #endif
      #if ARM
      if( mpu9250_get( &(g_txBetCom.betcall.ARM_MPU) ) )
      {
        /* MPU data was present to we have at least one IMU data set. */
        g_txBetCom.betcall.has_ARM_MPU = true;
        b_data_available |= true;
      }
      #endif
    #endif

    #if MFW_SENS_BNO055_EN
      #if FC
      if( bno055_get( &(g_txBetCom.betcall.FC_BNO) ) )
      {
        /* MPU data was present to we have at least one IMU data set. */
        g_txBetCom.betcall.has_FC_BNO = true;
        b_data_available |= true;
      }
      #endif
      #if ARM
      if( bno055_get( &(g_txBetCom.betcall.ARM_BNO) ) )
      {
        /* MPU data was present to we have at least one IMU data set. */
        g_txBetCom.betcall.has_ARM_BNO = true;
        b_data_available |= true;
      }
      #endif
    #endif

    #if SATELLADY_EN
      #if FC_GPS
      if( satellady_get( &(g_txBetCom.betcall.FC_GPS_gps) ) )
      {
        /* SatelLady data was present. */
        g_txBetCom.betcall.has_FC_GPS_gps = true;
        b_data_available |= true;
      }
      #endif
      #if ARM_GPS
      if( satellady_get( &(g_txBetCom.betcall.ARM_GPS_gps) ) )
      {
        /* SatelLady data was present. */
        g_txBetCom.betcall.has_ARM_GPS_gps = true;
        b_data_available |= true;
      }
      #endif
    #endif


    #if MFW_SENS_PPS_EN
      #if FC
      if(pps_get( &(g_txBetCom.betcall.FC_pps) ))
      {
        g_txBetCom.betcall.has_FC_pps = true;
        b_data_available |= true;
      }
      #endif
      #if FC_GPS
      if(pps_get( &(g_txBetCom.betcall.FC_GPS_pps) ))
      {
        g_txBetCom.betcall.has_FC_GPS_pps = true;
        b_data_available |= true;
      }
      #endif
      #if ARM
      if(pps_get( &(g_txBetCom.betcall.ARM_pps) ))
      {
        g_txBetCom.betcall.has_ARM_pps = true;
        b_data_available |= true;
      }
      #endif
      #if ARM_GPS
      if(pps_get( &(g_txBetCom.betcall.ARM_GPS_pps) ))
      {
        g_txBetCom.betcall.has_ARM_GPS_pps = true;
        b_data_available |= true;
      }
      #endif
    #endif

    #if MFW_SENS_TUBE_ANGLE_EN
    if (tube_angle_sensor_get_el(&(g_txBetCom.betcall.TAS_elevation)))
    {
      g_txBetCom.betcall.has_TAS_elevation = true;
      b_data_available |= true;
    }

	if (tube_angle_sensor_get(&(g_txBetCom.betcall.TAS_elevation), &(g_txBetCom.betcall.TAS_pitch)))
	{
	g_txBetCom.betcall.has_TAS_elevation = true;
    g_txBetCom.betcall.has_TAS_pitch = true;
    b_data_available |= true;
	}
    #endif

    #if MFW_SENS_SHT25_EN
      #if FC
      if(SHT25_get( &(g_txBetCom.betcall.FC_humidity)))
      {
        g_txBetCom.betcall.has_FC_humidity = true;
        b_data_available |= true;
      }
      #endif
      #if ARM
      if(SHT25_get( &(g_txBetCom.betcall.ARM_humidity)))
      {
        g_txBetCom.betcall.has_ARM_humidity = true;
        b_data_available |= true;
      }
      #endif
    #endif

	#if MFW_SENS_ADS1115_EN
	  #if FC
	  if (ads1115_get(&(g_txBetCom.betcall.FC_adc)))
	  {
		  g_txBetCom.betcall.has_FC_adc = true;
		  b_data_available |= true;
	  }
	  #endif
	  #if ARM
	  if (ads1115_get(&(g_txBetCom.betcall.ARM_adc)))
	  {
		  g_txBetCom.betcall.has_ARM_adc = true;
		  b_data_available |= true;
	  }
	  #endif
	#endif

    #if MFW_SENS_MPL_EN
      #if FC
      if(MPL_get( &(g_txBetCom.betcall.FC_barometer)))
      {
        g_txBetCom.betcall.has_FC_barometer = true;
        b_data_available |= true;
      }
      #endif
      #if ARM
      if(MPL_get( &(g_txBetCom.betcall.ARM_barometer)))
      {
        g_txBetCom.betcall.has_ARM_barometer = true;
        b_data_available |= true;
      }
      #endif
    #endif

    #if MFW_SYSINFO_MESSAGE_EN
    pb_Sysinfo.has_report = log_getError(&pb_Sysinfo.report);
    if(g_Sysinfo_data_available || pb_Sysinfo.has_report)
    {
      g_Sysinfo_data_available = false;
      #if FC
      g_txBetCom.betcall.FC_sysinfo = pb_Sysinfo;
      g_txBetCom.betcall.has_FC_sysinfo = true;
      #endif
      /*#if FC
      g_txBetCom.betcall.FC_GPS_sysinfo = pb_Sysinfo;
      g_txBetCom.betcall.has_FC_GPS_sysinfo = true;
      #endif*/
      #if ARM
      g_txBetCom.betcall.ARM_sysinfo = pb_Sysinfo;
      g_txBetCom.betcall.has_ARM_sysinfo = true;
      #endif
      #if ARM_GPS
      g_txBetCom.betcall.ARM_GPS_sysinfo = pb_Sysinfo;
      g_txBetCom.betcall.has_ARM_GPS_sysinfo = true;
      #endif
      pb_Sysinfo = g_SysinfoDefault;

      b_data_available |= true;
    }
    #endif

    /*
     * Read all sensor data from all modules here!
     */

    if(b_data_available)
    {
      /* Update the internal PackageID submessage. */
      g_txBetCom.has_generated = hal_pps_getTimestamp( &(g_txBetCom.generated) );
      g_txBetCom.has_id = loc_setPackageId( &(g_txBetCom.id), &g_BetcomPackageCounter );

      /* This message contains a betCall submessage. */
      g_txBetCom.has_betcall = true;

      /* Set timestamp of sensor message. */
      g_txBetCom.betcall.has_generated = hal_pps_getTimestamp( &(g_txBetCom.betcall.generated) );
      g_txBetCom.betcall.has_id = loc_setPackageId( &(g_txBetCom.betcall.id), &g_BetcallPackageCounter );

      /* Finally, send the meassge down to groundstation. */
      #if MFW_DEBUG_REPEAT_FIRST_MESSAGE
      if(!b_UseBetComDebug)
      {
        g_txBetComDebug = g_txBetCom;
        b_UseBetComDebug = true;
      }
      if(!loc_writeBetComToGs( &g_txBetComDebug ) )
      {
        loc_mfw_error();
      }

      #else

      /* Take timestamp between encoding and sending, because lwip stack, opposite to sproto from RS485
       * communication, doesn't return after filling the buffer, but after sending the packet. Thus,
       * time differences can get negative, because sent_last may be taken after sending and receiving
       * the package.*/
      Timestamp sent_last = Timestamp_init_default;

      if(!loc_writeBetComToGs( &g_txBetCom, &sent_last ) )
      {
        loc_mfw_error();
      }
      #endif

      /* Reset the betcom message after each transmission. */
      g_txBetCom = g_betComDefault;
      g_txBetCom.has_sent_last = true;
      g_txBetCom.sent_last = sent_last;
      g_txBetCom.betcall.has_sent_last = true;
      g_txBetCom.betcall.sent_last = sent_last;
    }
  }/* if( timing_isTimedOut(g_timerSendBetCall) ) */

  return;
}

/* ===========================================================================*/
/*                  loc_initActuators()                                       */
/* ===========================================================================*/
#if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
static bool loc_initActuators(void)
{
  bool b_return = false;

  b_return = servopower_init();

  #if MFW_ACT_IS_ANY_SERVO_DUDE_ENABLED
  if( b_return )
  {
    b_return = servopower_on();
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_ELEVATOR_EN
  if( b_return )
  {
    gps_servoElevator = servo_dude_init(E_SERVO_DUDE_ELEVATOR);
    if( gps_servoElevator )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_RUDDER_EN
  if( b_return )
  {
    gps_servoRudder = servo_dude_init(E_SERVO_DUDE_RUDDER);
    if( gps_servoRudder )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN
  if( b_return )
  {
    gps_servoFlapLeft = servo_dude_init(E_SERVO_DUDE_FLAP_LEFT);
    if( gps_servoFlapLeft )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN
  if( b_return )
  {
    gps_servoFlapRight = servo_dude_init(E_SERVO_DUDE_FLAP_RIGHT);
    if( gps_servoFlapRight )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN
  if( b_return )
  {
    gps_servoAileronLeft = servo_dude_init(E_SERVO_DUDE_AILERON_LEFT);
    if( gps_servoAileronLeft )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN
  if( b_return )
  {
    gps_servoAileronRight = servo_dude_init(E_SERVO_DUDE_AILERON_RIGHT);
    if( gps_servoAileronRight )
    {
      b_return = true;
    }
  }
  #endif

  #if MFW_ACT_NCP5623_EN
  if( b_return )
  {
    gps_ncp5623 = NCP5623_init();
    if( gps_ncp5623 )
    {
      b_return = true;
    }
  }
  #endif
  /* Initialize further actuator modules here! */

  return b_return;
}
#endif

/* ===========================================================================*/
/*                  loc_runActuators()                                        */
/* ===========================================================================*/
#if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
static void loc_runActuators(void)
{
  #if MFW_ACT_SERVO_DUDE_ELEVATOR_EN
  servo_dude_run( gps_servoElevator );
  #endif
  
  #if MFW_ACT_SERVO_DUDE_RUDDER_EN
  servo_dude_run( gps_servoRudder );
  #endif

  #if MFW_ACT_SERVO_DUDE_RUDDER_EN
  servo_dude_run( gps_servoRudder );
  #endif

  #if MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN
  servo_dude_run( gps_servoFlapLeft );
  #endif

  #if MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN
  servo_dude_run( gps_servoFlapRight );
  #endif

  #if MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN
  servo_dude_run( gps_servoAileronLeft );
  #endif

  #if MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN
  servo_dude_run( gps_servoAileronRight );
  #endif

  #if MFW_ACT_NCP5623_EN
  NCP5623_run();
  #endif

  /* Call all the actuator run() functions here! */
  return;
}
#endif

/* ===========================================================================*/
/*                  loc_setActuators()                                        */
/* ===========================================================================*/
#if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
static void loc_setActuators(void)
{
  /* We need to call set() only in case a betPUSH message is received. */
  if( gb_betPushReceived )
  {
    /* Reset the boolean immediately as we will handle it now. */
    gb_betPushReceived = false;

    #if MFW_ACT_SERVO_DUDE_ELEVATOR_EN
    if (g_rxBetCom.betpush.has_elevator) {
      servo_dude_set( gps_servoElevator, &(g_rxBetCom.betpush.elevator));
    }
    #endif

    #if MFW_ACT_SERVO_DUDE_RUDDER_EN
    if (g_rxBetCom.betpush.has_rudder) {
      servo_dude_set( gps_servoRudder, &(g_rxBetCom.betpush.rudder) );
    }
    #endif

    #if MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN
    if (g_rxBetCom.betpush.has_flap_left) {
      servo_dude_set( gps_servoFlapLeft, &(g_rxBetCom.betpush.flap_left) );
    }
    #endif

    #if MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN
    if (g_rxBetCom.betpush.has_flap_right) {
      servo_dude_set( gps_servoFlapRight, &(g_rxBetCom.betpush.flap_right) );
    }
    #endif

    #if MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN
    if (g_rxBetCom.betpush.has_aileron_left) {
      servo_dude_set( gps_servoAileronLeft, &(g_rxBetCom.betpush.aileron_left) );
    }
    #endif

    #if MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN
    if (g_rxBetCom.betpush.has_aileron_right) {
      servo_dude_set( gps_servoAileronRight, &(g_rxBetCom.betpush.aileron_right) );
    }
    #endif
    /* Call all the actuators set() functions here! */
  }

  return;
}
#endif


/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  mfw_init()                                                */
/* ===========================================================================*/
bool mfw_init(void)
{
  bool b_return = true;

  /* Initiailze global variables for protobuf handling. */
  gb_betCallReceived = false;
  gb_betConfReceived = false;
  gb_betPushReceived = false;

  #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
/* Initiailze UART communication channel to GS. */
  memset( (void*) gac_groundstRxBuf, 0, MFW_GROUNDST_RX_BUF_LEN);
  memset( (void*) gac_ostreamBuf, 0, MFW_PB_OSTREAM_BUF_LEN);
  #endif

  /* Initialize here, otherwise the first package will contain nonsense. */
  g_txBetCom = g_betComDefault;

  /* Initialize debug variables */
  #if MFW_DEBUG_REPEAT_FIRST_MESSAGE
  g_txBetComDebug = g_betComDefault;
  bool b_UseBetComDebug = false;
  #endif

  #if MFW_DEBUG_ENCODE_PB_ONLY_ONCE
  b_UseOstreamDebug = false;
  #endif

  #if MFW_LOGGER_EN
  if(b_return)
  {
    b_return = log_init(&loc_writeString);

    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, BUILD_STRING);
    }

    #if MFW_DEBUG_VERBOSE_LOGGER
    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, "Logging module successfully initialised.");
    }
    #endif
  }
  #endif

  #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
  /* Initiailze UART communication channel to GS. */
  b_return = loc_initUartComToGs();

  #if MFW_DEBUG_VERBOSE_LOGGER
  if(b_return)
  {
    b_return = log_writeError(Report_ReportType_INFO, "UART to GS successfully initialised.");
  }
  #endif /* MFW_GROUNDST_COMMUNICATION_VIA_UART_EN */

  #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
  b_return = loc_initEthernetToGs();

  #if MFW_DEBUG_VERBOSE_LOGGER
  if(b_return)
  {
    b_return = log_writeError(Report_ReportType_INFO, "Ethernet to GS successfully initialised.");
  }
  #endif
  #endif /* MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN */

  #if MFW_PB_FORWARDER_EN
  /* After initialization of UART com to GS we can init the pb forwarder
   * and pass the callback that forwards/writes data to the UART to GS.
   * Do not init before to avoid issues of non-initialized UART port to GS! */
  if(b_return)
  {
    b_return = pb_forwarder_init( loc_forwardProtobufToGs );
  }

  #if MFW_DEBUG_VERBOSE_LOGGER
  if(b_return)
  {
    b_return = log_writeError(Report_ReportType_INFO, "Protobuf forwarder successfully initialised.");
  }
  #endif

  #endif

  /* Initialize our utilities modules before sensors/actuators. */
  if( b_return )
  {
    b_return = timing_init();

    #if MFW_DEBUG_VERBOSE_LOGGER
    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, "Timing module successfully initialised.");
    }
    #endif
  }

  #if MFW_I2C_BUSMANAGER_EN
  if(b_return)
  {
    b_return = i2c_bus_mgr_init();

    #if MFW_DEBUG_VERBOSE_LOGGER
    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, "I2C busmanager successfully initialised.");
    }
    #endif
  }
  #endif

  #if MFW_ACT_IS_ANY_SERVO_DUDE_ENABLED
  if( b_return )
  {
    /* Initialize all the actuator modules. */
    b_return = loc_initActuators();

    #if MFW_DEBUG_VERBOSE_LOGGER
    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, "Actuators successfully initialised.");
    }
    #endif
  }
  #endif

  if( b_return )
  {
    /* Initialize all the sensor modules. */
    b_return = loc_initSensors();

    #if MFW_DEBUG_VERBOSE_LOGGER
    if(b_return)
    {
      b_return = log_writeError(Report_ReportType_INFO, "Sensors successfully initialised.");
    }
    #endif
  }

  /* Start the timer for sending out betCALL message down to GS. */
  g_timerSendBetCall = timing_getTimeout( MFW_INTERVALL_BETCALL_MSG_TO_GS );

  g_pbDecodeFailCounter = 0;

  g_BetcomPackageCounter = 0;
  g_BetcallPackageCounter = 0;
  g_BetconfPackageCounter = 0;

  #if MFW_SYSINFO_MESSAGE_EN
    pb_Sysinfo = g_SysinfoDefault;
    g_Sysinfo_data_available = false;
    #if MFW_LOOP_COUNTER_EN
      g_mainLoopCounter = 0;
      g_timerLoopCounter = timing_getTimeout (1000);
    #endif
  #endif

  if(b_return)
  {
    b_return = log_writeError(Report_ReportType_INFO, "MFW successfully initialised.");
  }

  return b_return;
}

/* ===========================================================================*/
/*                  mfw_run()                                                 */
/* ===========================================================================*/
void mfw_run(void)
{
  #if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN
  /* Run the serial protocol. */
  while( sproto_run( &gs_groundst ) );
  #elif MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN
  /* Run the MFW socket module that handles IP/ethernet communication to GS. */
  mfw_socket_udpRun();
  #endif

  /* Handle BetCONF messages. */
  loc_setSensorsConf();

  #if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
  /* Run all the actuator modules. */
  loc_runActuators();
  #endif

  #if MFW_ACT_IS_ANY_ACTUATOR_ENABLED
  /* Handle all the actuator modules, i.e. handle the latest betPUSH message. */
  loc_setActuators();
  #endif

  /* Run all the sensor modules. */
  loc_runSensors();

  /* Catch up all available sensor data and send it to ground station. */
  loc_getSensorData();

  #if MFW_I2C_BUSMANAGER_EN
  i2c_bus_mgr_run();
  #endif

  #if MFW_LOOP_COUNTER_EN
  g_mainLoopCounter++;

  if(timing_isTimedOut(g_timerLoopCounter))
  {
    pb_Sysinfo.main_loops = g_mainLoopCounter;
    pb_Sysinfo.has_main_loops = true;
    g_Sysinfo_data_available |= true;
    g_mainLoopCounter = 0;
    g_timerLoopCounter = timing_getTimeout (1000);
  }
  #endif

  #if MFW_PB_FORWARDER_EN
  /* Run the pb-forwarder. */
  pb_forwarder_run();
  #endif

  return;
}
