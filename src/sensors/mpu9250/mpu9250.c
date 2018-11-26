/* Copyright (C) 2015-2017
 *
 * mpu9250.c
 *
 * Elias Rosch         <eliasrosch@gmail.com>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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
#include <string.h>

#include "mpu9250.h"

#include "hal_dma.h"
#include "hal_timer.h"
#include "hal_pps.h"
#include "hal_gpio.h"

#include "timing.h"

/* Define sensor update rate in ms*/
#define MPU_REFRESH_PERIOD 10
/* Define timeout delay in ms*/
#define TIMEOUT_DELAY 50

/*! Port ID of GPIO module for CS pins. */
#define GPIO_CS_PORT E_HAL_GPIO_PORT_A

/*! Mask for CS-pins. */
#define GPIO_CS_MASK 0x8
#define GPIO_LOW 0x00
#define GPIO_HIGH 0xFF

/*! Addresses and Masks for MPU-registers. */
#define MPU9250_REG_PWR_MGMT_1 0x6B
#define MPU9250_REG_WHO_AM_I 0x75
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_GYRO_XOUT_H 0x43
#define MPU9250_REG_MAG_XOUT_H 0x3H
#define MPU9250_REG_MPU_READ_FLAG 0X80

/*! Available MPU9250-states provided and handled by the run() function. */
typedef enum
{
  E_MPU9250_STATE_IDLE,
  E_MPU9250_STATE_INIT_PENDING,
  E_MPU9250_STATE_READ_PENDING,
  E_MPU9250_STATE_DATA_READY,
  E_MPU9250_STATE_TIMEOUT
} E_MPU9250_STATE_t;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
static volatile uint8_t MPU9250_meas_data[15];
static volatile uint8_t MPU9250_addr_data[15];

static s_hal_spi_ctx_t *MPU9250_spi_ctx;
static E_HAL_GPIO_PORT_t MPU9250_gpio_port;
static E_MPU9250_STATE_t MPU9250_state;
static uint32_t MPU9250_update_timer;
static uint32_t MPU9250_mpu_timeout;

static uint8_t MPU9250_wake_up_data[2];
static uint8_t MPU9250_wake_up_dataRX[2];

static const Imu g_mpu_imuDefault = Imu_init_default;
static Imu MPU9250_lastData;
static volatile bool MPU9250_isDataReady;

/* ===========================================================================*/
/*                  LOCAL FUNCTION IMPLEMENTATIONS and PROTOTYPES             */
/* ===========================================================================*/
static void loc_mpu_callbackToIdle(void);
static void loc_copyLastDataToMsg(Imu *const mpu_msg);

/* ===========================================================================*/
/*                         loc_callback after gyro read 	     			  */
/* ===========================================================================*/
static void loc_mpu_callbackToIdle(void)
{
  /* Pull up CS pin to finalize SPI transmission. */
  hal_gpio_write(MPU9250_gpio_port, GPIO_CS_MASK, GPIO_HIGH);

  /* Copy the data from array to our local Imu struct variable.
   * Furthermore, this function creates the timestamp for the measured data!
   * I.e. the timestamp is now applied right after the last byte from MPU is
   * received through SPI interface. Makes also sure, that no MPU_get is in
   * progress.
   */
  loc_copyLastDataToMsg(&MPU9250_lastData);
  MPU9250_isDataReady = true;
  
  MPU9250_state = E_MPU9250_STATE_DATA_READY;
  return;
}

static void loc_copyLastDataToMsg(Imu *const mpu_msg)
{

  mpu_msg->has_generated = hal_pps_getTimestamp(&(mpu_msg->generated));

  // get accel value
  mpu_msg->accel_raw.x = (int16_t)((MPU9250_meas_data[1] << 8) | (MPU9250_meas_data[2]));
  mpu_msg->accel_raw.has_x = true;
  mpu_msg->accel_raw.y = (int16_t)((MPU9250_meas_data[3] << 8) | (MPU9250_meas_data[4]));
  mpu_msg->accel_raw.has_y = true;
  mpu_msg->accel_raw.z = (int16_t)((MPU9250_meas_data[5] << 8) | (MPU9250_meas_data[6]));
  mpu_msg->accel_raw.has_z = true;
  mpu_msg->has_accel_raw = true;

  mpu_msg->gyro_raw.x = (int16_t)((MPU9250_meas_data[9] << 8) | (MPU9250_meas_data[10]));
  mpu_msg->gyro_raw.has_x = true;
  mpu_msg->gyro_raw.y = (int16_t)((MPU9250_meas_data[11] << 8) | (MPU9250_meas_data[12]));
  mpu_msg->gyro_raw.has_y = true;
  mpu_msg->gyro_raw.z = (int16_t)((MPU9250_meas_data[13] << 8) | (MPU9250_meas_data[14]));
  mpu_msg->gyro_raw.has_z = true;
  mpu_msg->has_gyro_raw = true;
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       mpu9250_init() Initializes a MPU9250      and returns context        */
/* ===========================================================================*/
bool mpu9250_init(E_HAL_SPI_PORT_t e_hal_spi_port, uint32_t ui32_spiClock)
{
  bool b_return = false;

  MPU9250_lastData = g_mpu_imuDefault;
  MPU9250_isDataReady = false;

  memset((void *)MPU9250_meas_data, 0, sizeof(MPU9250_meas_data));
  memset((void *)MPU9250_addr_data, 0, sizeof(MPU9250_addr_data));
  MPU9250_addr_data[0] = (MPU9250_REG_MPU_READ_FLAG | MPU9250_REG_ACCEL_XOUT_H);

  MPU9250_spi_ctx = hal_spi_init(e_hal_spi_port, E_HAL_SPI_MODE_MASTER, ui32_spiClock);

  /* hal_spi_init() may return a NULL pointer if init fails! */ 
  if(MPU9250_spi_ctx)
  {
    // Configure the SPI port
    b_return = hal_spi_config(MPU9250_spi_ctx, E_HAL_SPI_CLK_POL_HIGH, E_HAL_SPI_CLK_PHA_EDGE_SCND, ui32_spiClock);

    /* If initialization of SPI fails we should not continue. */
    if(b_return)
    {
      // initialize GPIO module with GPIO-pin as CS
      b_return = hal_gpio_init();
      b_return = hal_gpio_setDir(GPIO_CS_PORT, GPIO_CS_MASK, E_HAL_GPIO_DIR_OUT);
      MPU9250_gpio_port = GPIO_CS_PORT;

      /* If we were not able to pull down the CS pin we can not wake up the MPU. */
      if(b_return)
      {
        // Wake up the imu
        MPU9250_wake_up_data[0] = MPU9250_REG_PWR_MGMT_1;
        MPU9250_wake_up_data[1] = 0x00;
        MPU9250_mpu_timeout = hal_timer_getTimeout(TIMEOUT_DELAY);
        MPU9250_update_timer = hal_timer_getTimeout(MPU_REFRESH_PERIOD);
        MPU9250_state = E_MPU9250_STATE_INIT_PENDING;
        hal_gpio_write(MPU9250_gpio_port, GPIO_CS_MASK, GPIO_LOW);
        hal_spi_xfer(MPU9250_spi_ctx, MPU9250_wake_up_data, MPU9250_wake_up_dataRX, 2, loc_mpu_callbackToIdle);
      }
    }
  }

  return b_return;
}

/* ===========================================================================*/
/*       mpu9250_global_run() Handles the global MPU9250-StateMachine         */
/* ===========================================================================*/

void mpu9250_run()
{
  switch(MPU9250_state)
  {
  case E_MPU9250_STATE_IDLE:
    if(hal_timer_isTimedOut(MPU9250_update_timer))
    {
      MPU9250_update_timer = hal_timer_getTimeout(MPU_REFRESH_PERIOD);
      MPU9250_mpu_timeout = hal_timer_getTimeout(TIMEOUT_DELAY);
      MPU9250_state = E_MPU9250_STATE_READ_PENDING;
      hal_gpio_write(MPU9250_gpio_port, GPIO_CS_MASK, GPIO_LOW);
      hal_spi_xfer(MPU9250_spi_ctx, (uint8_t *)MPU9250_addr_data, (uint8_t *)MPU9250_meas_data, 15,
                   loc_mpu_callbackToIdle);
    }
    break;
  case E_MPU9250_STATE_READ_PENDING:
    if(hal_timer_isTimedOut(MPU9250_mpu_timeout))
    {
      MPU9250_state = E_MPU9250_STATE_TIMEOUT;
    }
    break;
  case E_MPU9250_STATE_INIT_PENDING:
    if(hal_timer_isTimedOut(MPU9250_mpu_timeout))
    {
      MPU9250_state = E_MPU9250_STATE_TIMEOUT;
    }
    break;
  case E_MPU9250_STATE_DATA_READY:
    if(hal_timer_isTimedOut(MPU9250_mpu_timeout))
    {
      MPU9250_state = E_MPU9250_STATE_TIMEOUT;
    }
    break;
  default:
    break;
  }
}

/* ===========================================================================*/
/*              mpu9250_get()                                                 */
/* ===========================================================================*/

bool mpu9250_get(Imu *const p_mpu_msg)
{
  bool b_return = false;

  while(MPU9250_isDataReady)
  {
    MPU9250_isDataReady = false;
    *p_mpu_msg = MPU9250_lastData;
    b_return = true;
    MPU9250_state = E_MPU9250_STATE_IDLE;
  }

  return b_return;
}

bool mpu9250_setConf(ImuConf *const p_imuConf)
{
  bool b_return = false;

  return b_return;
}
