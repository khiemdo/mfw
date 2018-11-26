/* Copyright (C) 2015-2017
 *
 * mfw-global-config.h
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
 * Martin Dold         <martin.dold@gmx.net>
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

#ifndef MFW_GLOBAL_CONFIG_H_
#define MFW_GLOBAL_CONFIG_H_

//*****************************************************************************/
//
//! \addtogroup mfw
//! @{
//!
//! \addtogroup mfw_cfg Config
//! @{
//
//*****************************************************************************/

#include "mfw-config.h"

#if (FC + ARM + FC_GPS + ARM_GPS) > 1
#error The board cannot simultaneously be more than one board. \
       Please choose only one.
#endif


#if (FC || ARM_GPS || FC_GPS)
#define MFW_GROUNDST_COMMUNICATION_VIA_UART_EN  1
#define MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN   0
#else
/*! @brief If enabled, the communication to groundstation is done through UART (RS232)
 *         interface.
 */
#define MFW_GROUNDST_COMMUNICATION_VIA_UART_EN  0
/*! @brief If enabled, the communication to groundstation is done through ethernet
 *         interface.
 */
#define MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN   1
#endif



#if (MFW_GROUNDST_COMMUNICATION_VIA_UART && MFW_GROUNDST_COMMUNICATION_VIA_ETH)
#error Define the communication port to groundstation. Communication via\
 UART and Ethernet at the same time is not possible. Check macros\
 MFW_GROUNDST_COMMUNICATION_VIA_UART_EN and MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN.
#endif

/* ============================================================================
 * Configuration for on-board LEDs
 */
#if(FC || ARM)
#define LED1_PORT E_HAL_GPIO_PORT_L
#define LED1_PIN  HAL_GPIO_PIN_0
#define LED2_PORT E_HAL_GPIO_PORT_L
#define LED2_PIN  HAL_GPIO_PIN_4
#define LED3_PORT E_HAL_GPIO_PORT_L
#define LED3_PIN  HAL_GPIO_PIN_5
#else
#define LED1_PORT E_HAL_GPIO_PORT_F
#define LED1_PIN  HAL_GPIO_PIN_0
#endif

/* ============================================================================
 * Configuration for UART communication to groundstation.
 */
#if (FC || ARM)
/*! Defines the port and pin of the enable pin of TXS0108 Level Shifter. */
  #define MFW_LSHIFT_EN_PORT   E_HAL_GPIO_PORT_K
  #define MFW_LSHIFT_EN_PIN    HAL_GPIO_PIN_2
#else
  #define MFW_LSHIFT_EN_PORT   E_HAL_GPIO_PORT_F
  #define MFW_LSHIFT_EN_PIN    HAL_GPIO_PIN_4
#endif

#if MFW_GROUNDST_COMMUNICATION_VIA_UART_EN

/*! Defines the UART port the GS is connected to. Must be of type
 *  \ref E_HAL_UART_PORT_t. */
#define MFW_UART_PORT_TO_GS         E_HAL_UART_PORT_0

/*! Defines the UART baudrate to be used between GS and FC. */
#define MFW_UART_BAUD_RATE_TO_GS    921600

#if (FC || ARM)
/*! As mfw provides the buffer to the serial protocol module (for communication
 *  with groundstation), this number defines the buffer length of the serial
 *  protocol RX buffer. Make sure this value fits to the maximum length of a
 *  protobuf message transmitted from GS to FC! */
  #define MFW_GROUNDST_RX_BUF_LEN      8192
#else
  #define MFW_GROUNDST_RX_BUF_LEN      1024
#endif

/*! \brief Defines the intervall (in ms) of BetCALL messages sent down to GS.
 *
 *  \details This value can be set to '0' to send out messages as fast as
 *           possible. But respect the warning stated below!
 *
 *  \warning We need to limit the amount of data (we sent down to GS) to not
 *           generate more data than the UART can send out. Therfore, this
 *           value has a strong relation to \ref MFW_UART_BAUD_RATE_TO_GS.
 */
#define MFW_INTERVALL_BETCALL_MSG_TO_GS   0

#endif /* MFW_GROUNDST_COMMUNICATION_VIA_UART_EN */

#if (FC || ARM)
/*! In case of FC we need the pb forwarder to forwarder TAS messages to GS. */
  #define MFW_PB_FORWARDER_EN  1
#else
  #define MFW_PB_FORWARDER_EN  0
#endif

#if (FC || ARM)
  /*! Use software CRC instead of hardware CRC. */
  #define MFW_SOFTWARE_CRC_EN 0
#else
  /*! Use software CRC instead of hardware CRC. */
  #define MFW_SOFTWARE_CRC_EN 1
#endif

/* ============================================================================
 * Configuration for ethernet communication to groundstation.
 */
#if MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN

#warning Ethernet not fully supported yet!
/* Defines for the IP address of flight controller. */
#define MFW_IP_ADDR_1       192
#define MFW_IP_ADDR_2       168
#define MFW_IP_ADDR_3         2
#define MFW_IP_ADDR_4        42

#define MFW_NET_MASK_1      255
#define MFW_NET_MASK_2      255
#define MFW_NET_MASK_3      255
#define MFW_NET_MASK_4        0

#define MFW_GW_IP_ADDR_1    192
#define MFW_GW_IP_ADDR_2    168
#define MFW_GW_IP_ADDR_3      2
#define MFW_GW_IP_ADDR_4      1

/*! \brief Local UDP port for reception and destination port for transmission. */
#define MFW_UDP_PORT       8080

#define MFW_DEST_IP_ADDR_1       192
#define MFW_DEST_IP_ADDR_2       168
#define MFW_DEST_IP_ADDR_3         2
#define MFW_DEST_IP_ADDR_4         1

#if (FC || ARM)
  #define MFW_GROUNDST_RX_BUF_LEN      8192
#else
  #define MFW_GROUNDST_RX_BUF_LEN      1024
#endif

/* \brief Defines the intervall (in ms) of BetCALL messages sent down to GS in
 *        case Ethernet is used for communication between FC and GS. */
#define MFW_INTERVALL_BETCALL_MSG_TO_GS         4

/* Add configs/settings for ETH communciation here like IP address and port. */
#endif /* MFW_GROUNDST_COMMUNICATION_VIA_ETH_EN */


/* ============================================================================
 * Configuration for protocol buffers.
 */
/*! Size of the output stream buffer, i.e. the buffer where nanopb writes the
 *  encoded protocol buffer message to. */
#if (FC || ARM)
  #define MFW_PB_OSTREAM_BUF_LEN       8192
#else
  #define MFW_PB_OSTREAM_BUF_LEN        1024
#endif

/* ============================================================================
 * Configuration for actuator modules.
 */
#if FC
/*! Enables/disables the servo dude elevator. */
#define MFW_ACT_SERVO_DUDE_ELEVATOR_EN      1
/*! Enables/disables the servo dude rudder. */
#define MFW_ACT_SERVO_DUDE_RUDDER_EN        1
/*! Enables/disables the servo dude flap-left. */
#define MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN     0
/*! Enables/disables the servo dude flap-right. */
#define MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN    1
/*! Enables/disables the servo dude aileron-left. */
#define MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN  0
/*! Enables/disables the servo dude aileron-right. */
#define MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN 1
#endif
/*! Checks if any of the servo dudes (MFW_ACT_SERVO_DUDE_xxx) is enabled. */
#define MFW_ACT_IS_ANY_SERVO_DUDE_ENABLED                                      \
  (MFW_ACT_SERVO_DUDE_ELEVATOR_EN      || MFW_ACT_SERVO_DUDE_RUDDER_EN      || \
   MFW_ACT_SERVO_DUDE_FLAP_LEFT_EN     || MFW_ACT_SERVO_DUDE_FLAP_RIGHT_EN  || \
   MFW_ACT_SERVO_DUDE_AILERON_LEFT_EN  || MFW_ACT_SERVO_DUDE_AILERON_RIGHT_EN)
/*! Checks if any actuator at all is enabled. */
#define MFW_ACT_IS_ANY_ACTUATOR_ENABLED (MFW_ACT_IS_ANY_SERVO_DUDE_ENABLED)

/* ============================================================================
 * Configuration for sensor modules.
 */
#if ARM
/*! Enables/disables the tube angle module. */
#define MFW_SENS_TUBE_ANGLE_EN          1
#else
#define MFW_SENS_TUBE_ANGLE_EN          0
#endif
#if MFW_SENS_TUBE_ANGLE_EN
/*! Defines the SPI port speed (Hz) for communication with tube angle sensor. */
#define MFW_SENS_TUBE_ANGLE_SPI_SPEED   2000000
#endif /* MFW_SENS_TUBE_ANGLE_EN */

#if (FC || ARM)
/*! Enables/disables the MPU9250 module. */
#define MFW_SENS_MPU9250_EN             1
#else
#define MFW_SENS_MPU9250_EN             0
#endif
#if MFW_SENS_MPU9250_EN
/*! Defines the SPI port the MPU9250 is connected to. */
#define MFW_SENS_MPU9250_SPI_PORT    E_HAL_SPI_PORT_0
/*! Defines the SPI port speed (Hz) for communication with MPU9250. */
#define MFW_SENS_MPU9250_SPI_SPEED   3000000
#endif /* MFW_SENS_MPU9250_EN */

#if (ARM)
/*! Enables/disables the BNO055 module. */
#define MFW_SENS_BNO055_EN              1
#else
#define MFW_SENS_BNO055_EN              0
#endif

#if (FC_GPS || ARM_GPS)
/*! Enables/disables the SATELLADY module. */
#define SATELLADY_EN                     1
#else
#define SATELLADY_EN                     0
#endif

#if (FC || ARM || FC_GPS || ARM_GPS)
/*! Enables/disables the PPS module. */
#define MFW_SENS_PPS_EN                 1
#else
#define MFW_SENS_PPS_EN                 0
#endif
#if MFW_SENS_PPS_EN


  #if(FC || ARM)
  /*! Defines the port and pin of the respective PPS input. */
    #define MFW_PPS_INPUT_PORT    E_HAL_GPIO_PORT_M
    #define MFW_PPS_INPUT_PIN     HAL_GPIO_PIN_7
  #endif
  #if(FC_GPS || ARM_GPS)
    #define MFW_PPS_INPUT_PORT    E_HAL_GPIO_PORT_D
    #define MFW_PPS_INPUT_PIN     HAL_GPIO_PIN_7
  #endif
  #define MFW_PPS_LED_EN                1
  #if MFW_PPS_LED_EN
    #define MFW_PPS_LED_PORT      LED1_PORT
    #define MFW_PPS_LED_PIN       LED1_PIN
  #endif
#endif

/* ============================================================================
 * I2C based devices need an activated bus manager, so if one of these sensors
 * is active, you also need to enable the bus manager.
 */
#if (FC || ARM)
#define MFW_I2C_BUSMANAGER_EN           1
#else
#define MFW_I2C_BUSMANAGER_EN           0
#endif

#if MFW_I2C_BUSMANAGER_EN
/*! Enables/disables the SHT25 module. */
#define MFW_SENS_SHT25_EN               1
/*! Enables/disables the MPL module. */
#define MFW_SENS_MPL_EN                 1
/*! Enabales disables the NCP5623 module */
#define MFW_ACT_NCP5623_EN              1
/*! Enabales disables the NCP5623 module */
#define MFW_SENS_ADS1115_EN             1
#endif

/* ============================================================================
 * Logger
 */
#if (FC || ARM || ARM_GPS)
#define MFW_LOGGER_EN             1
#define MFW_DEBUG_VERBOSE_LOGGER  1
#else
#define MFW_LOGGER_EN             0
#endif

/* ============================================================================
 * Debug options
 */
#define MFW_DEBUG_REPEAT_FIRST_MESSAGE  0
#define MFW_DEBUG_ENCODE_PB_ONLY_ONCE   0

/*! Enables/disables the Loop Counter. */
#define MFW_LOOP_COUNTER_EN             1

#define MFW_SYSINFO_MESSAGE_EN MFW_LOOP_COUNTER_EN

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* MFW_GLOBAL_CONFIG_H_ */
