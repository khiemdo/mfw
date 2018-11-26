/* Copyright (C) 2015-2017
 *
 * tube-angle-sensor.c
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

#include <stdlib.h>

#include "tube-angle-sensor.h"
#include "hal_timer.h"
#include "hal_pps.h"

/* Include for getting the timestamps for TAS measurement data. */
#include "timing.h"

#include "hal_spi.h"
#include "hal_gpio.h"

#ifndef TUBE_ANGLE_ENABLE_AFTER_START
#define TUBE_ANGLE_ENABLE_AFTER_START   1
#warning Tube angle module is (internally) enabled after init() and does not\
 wait for a setConf() message to be enabled.
#endif

//! Define sensor update rate in ms
#define TAS_REFRESH_PERIOD 4
//! Define timeout delay in ms
#define TIMEOUT_DELAY 20

//! Port ID of SPI ports.
#define SPI_PORT_ELEVATION E_HAL_SPI_PORT_1
#define SPI_PORT_ROTATION  E_HAL_SPI_PORT_3
//! Port ID of GPIO module for CS pins.
#define GPIO_CS0_PORT E_HAL_GPIO_PORT_B
#define GPIO_CS1_PORT E_HAL_GPIO_PORT_F
//! Mask for both CS-pins on the port(elevation, rotation).
#define GPIO_CS0_MASK 0x10
//! Mask for CS1.
#define GPIO_CS1_MASK 0x04

//! Mask for HIGH/LOW values.
#define GPIO_LOW 0x00
#define GPIO_HIGH 0xFF

/*! Available tube-angle-sensor-states provided and handled by the global_run() function. */
typedef enum
{
	E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_IDLE,
	E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_PENDING,
	E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_TIMEOUT,

} E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_t;

typedef enum
{
	E_TUBE_ANGLE_SENSOR_ROTATION_STATE_IDLE,
	E_TUBE_ANGLE_SENSOR_ROTATION_STATE_PENDING,
	E_TUBE_ANGLE_SENSOR_ROTATION_STATE_TIMEOUT,

} E_TUBE_ANGLE_SENSOR_ROTATION_STATE_t;
/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
static uint8_t tas_el_data[2];
static uint8_t tas_ro_data[2];
static uint8_t tas_el_data_dummy[2];
static uint8_t tas_ro_data_dummy[2];
static bool el_data_avail;
static bool ro_data_avail;

s_hal_spi_ctx_t *elevation_spi_ctx;
s_hal_spi_ctx_t *rotation_spi_ctx;
static E_HAL_GPIO_PORT_t gpio_port_cs0;
static E_HAL_GPIO_PORT_t gpio_port_cs1;
static E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_t elevation_state;
static E_TUBE_ANGLE_SENSOR_ROTATION_STATE_t rotation_state;
static uint32_t elevation_rate;
static uint32_t rotation_rate;
static uint32_t elevation_timeout;
static uint32_t rotation_timeout;
static bool gb_tasEnabled;

static const Timestamp g_timestampDefault = Timestamp_init_default;

static const AngleEncoder g_AngleEncoderDefault = AngleEncoder_init_default;
static AngleEncoder g_lastElData;
static AngleEncoder g_lastRoData;
//const BetCOM_TubeAngle g_tasDefault = BetCOM_TubeAngle_init_default;



static void loc_copyLastElDataToMsg(AngleEncoder* const ae_msg)
{
  ae_msg->generated = g_timestampDefault;
  ae_msg->has_generated = hal_pps_getTimestamp( &(ae_msg->generated) );
  ae_msg->angle_raw = ((uint16_t)tas_el_data[0])<<8 | (uint16_t)tas_el_data[1];;
  ae_msg->has_angle_raw = true;
}

static void loc_copyLastRoDataToMsg(AngleEncoder* const ae_msg)
{
  ae_msg->generated = g_timestampDefault;
  ae_msg->has_generated = hal_pps_getTimestamp( &(ae_msg->generated) );
  ae_msg->angle_raw = ((uint16_t)tas_ro_data[0])<<8 | (uint16_t)tas_ro_data[1];;
  ae_msg->has_angle_raw = true;
}

/* ===========================================================================*/
/*                         loc_callback if elevation is idle()						  */
/* ===========================================================================*/
static void loc_callbackToElevationIdle(void)
{
	hal_gpio_write(gpio_port_cs0, GPIO_CS0_MASK, GPIO_HIGH);
  /* Copy the data from array to our local Imu struct variable.
   * Furthermore, this function creates the timestamp for the measured data!
   * I.e. the timestamp is now applied right after the last byte from MPU is
   * received through SPI interface.
   */
  el_data_avail = true;
  loc_copyLastElDataToMsg( &g_lastElData );
	elevation_state = E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_IDLE;
	return;
}

/* ===========================================================================*/
/*                         loc_callback if rotation is idle()						  */
/* ===========================================================================*/
static void loc_callbackToRotationIdle(void)
{
	hal_gpio_write(gpio_port_cs1, GPIO_CS1_MASK, GPIO_HIGH);
  /* Copy the data from array to our local Imu struct variable.
   * Furthermore, this function creates the timestamp for the measured data!
   * I.e. the timestamp is now applied right after the last byte from MPU is
   * received through SPI interface.
   */
  ro_data_avail = true;
  loc_copyLastRoDataToMsg( &g_lastRoData );
	rotation_state = E_TUBE_ANGLE_SENSOR_ROTATION_STATE_IDLE;
	return;
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       servo_dude_init() Initializes a ServoDude and returns context        */
/* ===========================================================================*/
bool tube_angle_sens_init(uint32_t ui32_spiClock) {
	bool b_return = false;
	//! Initialize our context struct and init SPI port.
	elevation_spi_ctx = hal_spi_init(SPI_PORT_ELEVATION, E_HAL_SPI_MODE_MASTER, ui32_spiClock);
	rotation_spi_ctx = hal_spi_init(SPI_PORT_ROTATION, E_HAL_SPI_MODE_MASTER, ui32_spiClock);
	elevation_state = E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_IDLE;
	rotation_state = E_TUBE_ANGLE_SENSOR_ROTATION_STATE_IDLE;
	elevation_rate = hal_timer_getTimeout(TAS_REFRESH_PERIOD);
	rotation_rate = hal_timer_getTimeout(TAS_REFRESH_PERIOD);
    //! Configure the SPI port
	b_return = hal_spi_config(elevation_spi_ctx, E_HAL_SPI_CLK_POL_LOW, E_HAL_SPI_CLK_PHA_EDGE_SCND, ui32_spiClock);
	if (!b_return) return false;
	b_return = hal_spi_config(rotation_spi_ctx, E_HAL_SPI_CLK_POL_LOW, E_HAL_SPI_CLK_PHA_EDGE_SCND, ui32_spiClock);
	if (!b_return) return false;

    //! initialize GPIO module with two GPIO-pins for CS0 and CS1 (elevation, rotation)
	b_return = hal_gpio_init();
  b_return = hal_gpio_setDir(GPIO_CS0_PORT, GPIO_CS0_MASK, E_HAL_GPIO_DIR_OUT);
  b_return = hal_gpio_setDir(GPIO_CS1_PORT, GPIO_CS1_MASK, E_HAL_GPIO_DIR_OUT);
  gpio_port_cs0 = GPIO_CS0_PORT;
  gpio_port_cs1 = GPIO_CS1_PORT;

  //! Set both CS to idle
  b_return = hal_gpio_write(gpio_port_cs0, GPIO_CS0_MASK, GPIO_HIGH);
  b_return = hal_gpio_write(gpio_port_cs1, GPIO_CS1_MASK, GPIO_HIGH);

  //! No new data available at this point
  g_lastElData = g_AngleEncoderDefault;
  g_lastRoData = g_AngleEncoderDefault;

  el_data_avail = false;
  ro_data_avail = false;

  #if TUBE_ANGLE_ENABLE_AFTER_START
  gb_tasEnabled = true;
  #else
  /*! By default this module is disabled after init(). It must be enabled
   * by a call to \ref setConf() with corresponding member set to true. */
  gb_tasEnabled = false;
  #endif

  return (b_return);
}



/* ===========================================================================*/
/*  servo_dude_run() Handles the ServoDude-StateMachine         			  */
/* ===========================================================================*/

void tube_angle_sens_run() {

  if( gb_tasEnabled )
  {
	//! Handle state machine of elevation
    switch(elevation_state){
      case E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_IDLE:
    	  if (hal_timer_isTimedOut(elevation_rate)) {
    		  elevation_rate = hal_timer_getTimeout(TAS_REFRESH_PERIOD);
    		  elevation_timeout = hal_timer_getTimeout(TIMEOUT_DELAY);
    		  elevation_state = E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_PENDING;
    		  hal_gpio_write(gpio_port_cs0, GPIO_CS0_MASK, GPIO_LOW);
    		  hal_spi_xfer(elevation_spi_ctx, tas_el_data_dummy, tas_el_data, 2, loc_callbackToElevationIdle);
    	  }
    	  break;
      case E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_PENDING:
    	  if (hal_timer_isTimedOut(elevation_timeout)) {
    		  elevation_state = E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_TIMEOUT;
    	  }
    	  break;
      case E_TUBE_ANGLE_SENSOR_ELEVATION_STATE_TIMEOUT:
    	  break;
    }
	//! Handle state machine of rotation
    switch(rotation_state){
      case E_TUBE_ANGLE_SENSOR_ROTATION_STATE_IDLE:
    	  if (hal_timer_isTimedOut(rotation_rate)) {
    		  rotation_rate = hal_timer_getTimeout(TAS_REFRESH_PERIOD);
    		  rotation_timeout = hal_timer_getTimeout(TIMEOUT_DELAY);
    		  rotation_state = E_TUBE_ANGLE_SENSOR_ROTATION_STATE_PENDING;
    		  hal_gpio_write(gpio_port_cs1, GPIO_CS1_MASK, GPIO_LOW);
    		  hal_spi_xfer(rotation_spi_ctx, tas_ro_data_dummy, tas_ro_data, 2, loc_callbackToRotationIdle);
    	  }
    	  break;
      case E_TUBE_ANGLE_SENSOR_ROTATION_STATE_PENDING:
    	  if (hal_timer_isTimedOut(rotation_timeout)) {
    		  rotation_state = E_TUBE_ANGLE_SENSOR_ROTATION_STATE_TIMEOUT;
    	  }
          break;
      case E_TUBE_ANGLE_SENSOR_ROTATION_STATE_TIMEOUT:
    	  break;
    }
  }

	return;
}

/* ===========================================================================*/
/*    tube_angle_sensor_get() 									              */
/* ===========================================================================*/
bool tube_angle_sensor_get(AngleEncoder* const p_ae_el_msg, AngleEncoder* const p_ae_ro_msg) {
  bool b_return = false;
  //! Proceed only if module is enabled and a valid pointer is given.
  if( gb_tasEnabled && p_ae_el_msg &&  p_ae_ro_msg && el_data_avail && ro_data_avail)
  {
	//! get elevation value
	el_data_avail = false;
	ro_data_avail = false;
	*p_ae_el_msg = g_lastElData;
	*p_ae_ro_msg = g_lastRoData;
	b_return = true;
  }

  return b_return;
}

bool tube_angle_sensor_get_el(AngleEncoder* const p_ae_el_msg)
{
  bool b_return = false;

  if( gb_tasEnabled && p_ae_el_msg && el_data_avail)
  {
    el_data_avail = false;
    *p_ae_el_msg = g_lastElData;
    b_return = true;
  }

  return b_return;
}

bool tube_angle_sensor_get_ro(AngleEncoder* const p_ae_ro_msg)
{
  bool b_return = false;

  if( gb_tasEnabled && p_ae_ro_msg && ro_data_avail)
  {
    ro_data_avail = false;
    *p_ae_ro_msg = g_lastRoData;
    b_return = true;
  }

  return b_return;
}

/* ===========================================================================*/
/*    tube_angle_sensor_setConf()                                             */
/* ===========================================================================*/
bool tube_angle_sensor_setConf(AngleEncoderConf * const p_tasConf)
{
  bool b_return = false;

  if( p_tasConf )
  {
    gb_tasEnabled = p_tasConf->basic_conf.activated;

    /* Parse further conf members here and perform corresponding setting. */
  }

  return b_return;
}
