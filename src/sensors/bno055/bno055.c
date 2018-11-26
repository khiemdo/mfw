/* Copyright (C) 2015-2017
 *
 * bno055.c
 *
 * Elias Rosch         <eliasrosch@gmail.com>
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
#include <stdbool.h>
#include "bno055.h"
#include "hal_timer.h"
#include "hal_pps.h"
#include "hal_gpio.h"

#include "Types.pb.h"

/* Define sensor update rate in ms. */
#define BNO_REFRESH_PERIOD 10
/* Define timeout delay in ms. */
#define BNO_TIMEOUT_DELAY 50
/* Define startup delay. */
#define BNO_STARTUP_DELAY 650

#define SLAVE_ADDRESS (0x28)

#define REG_OPR_MODE 0x3D
#define OPR_MODE_AMG 0x07
#define OPR_MODE_NDOF 0x0C

#define REG_ACC_DATA_X_LSB 0x08
#define REG_QUA_DATA_W_LSB 0x20

#ifndef BNO055_ENABLE_AFTER_START
#define BNO055_ENABLE_AFTER_START   1
#warning BNO055 module is (internally) enabled after init() and does not\
 wait for a setConf() message to be enabled.
#endif

/*! Available BNO055-states provided and handled by the run() function. */
typedef enum
{
	E_BNO055_STATE_IDLE_WRITE,
	E_BNO055_STATE_IDLE_READ,
	E_BNO055_STATE_INIT_PENDING,
	E_BNO055_STATE_PENDING_WRITE,
	E_BNO055_STATE_PENDING_READ,
	E_BNO055_STATE_DATA_READY,
	E_BNO055_STATE_TIMEOUT,

} E_BNO055_STATE_t;

/*! Available BNO055-states provided and handled by the run() function. */
typedef enum
{
	E_BNO055_MODE_NORMAL,
	E_BNO055_MODE_FUSION,
	E_BNO055_MODE_ALL,
} E_BNO055_MODE_t;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
static uint8_t rx_data[45];

s_hal_i2c_ctx_t *bno055_i2c_ctx;
static E_BNO055_STATE_t ge_bno_state;
static E_BNO055_MODE_t ge_operation_mode = E_BNO055_MODE_NORMAL;

static uint32_t BNO055_update_timer;
static uint32_t BNO055_bno_timeout;

static bool gb_bnoEnabled;
static bool gb_new_data_av;
static const Imu g_bno_imuDefault = Imu_init_default;
static Imu g_bno_lastData;

/* ===========================================================================*/
/*                  LOCAL FUNCTION IMPLEMENTATIONS and PROTOTYPES             */
/* ===========================================================================*/
static void loc_bno_copyLastDataToMsg(Imu* const mpu_msg);



/* ===========================================================================*/
/*                         loc_callbackToIdle()							  	              */
/* ===========================================================================*/
void loc_bno_callbackToIdle(void *pvData, uint_fast8_t ui8Status)
{
	bno055_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	ge_bno_state = E_BNO055_STATE_IDLE_WRITE;
	return;
}

/* ===========================================================================*/
/*                         loc_callbackToIdle()							   	              */
/* ===========================================================================*/
void loc_bno_callbackToIdleWrite(void *pvData, uint_fast8_t ui8Status)
{
	gb_new_data_av = true;
    loc_bno_copyLastDataToMsg( &g_bno_lastData );
	bno055_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	ge_bno_state = E_BNO055_STATE_DATA_READY;
	return;
}

/* ===========================================================================*/
/*                         loc_callbackToIdle()							  	              */
/* ===========================================================================*/
void loc_bno_callbackToIdleRead(void *pvData, uint_fast8_t ui8Status)
{
	bno055_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	ge_bno_state = E_BNO055_STATE_IDLE_READ;
	return;
}

void loc_bno_copyLastDataToMsg(Imu* const bno_msg)
{
  /* Proceed only if module is enabled and a valid pointer is given. */
  if( gb_bnoEnabled && bno_msg)
  {
    bno_msg->has_generated = hal_pps_getTimestamp(&(bno_msg->generated));

    switch (ge_operation_mode) {
      case E_BNO055_MODE_NORMAL:

        // get accel value
        bno_msg->has_accel_raw = true;
        bno_msg->accel_raw.x = (int16_t)((rx_data[1] << 8) | rx_data[0]);
        bno_msg->accel_raw.has_x = true;
        bno_msg->accel_raw.y = (int16_t)((rx_data[3] << 8) | rx_data[2]);
        bno_msg->accel_raw.has_y = true;
        bno_msg->accel_raw.z = (int16_t)((rx_data[5] << 8) | rx_data[4]);
        bno_msg->accel_raw.has_z = true;

        // get mag value
        bno_msg->has_mag_raw = true;
        bno_msg->mag_raw.x = (int16_t)((rx_data[7] << 8) | rx_data[6]);
        bno_msg->mag_raw.has_x = true;
        bno_msg->mag_raw.y = (int16_t)((rx_data[9] << 8) | rx_data[8]);
        bno_msg->mag_raw.has_y = true;
        bno_msg->mag_raw.z = (int16_t)((rx_data[11] << 8) | rx_data[10]);
        bno_msg->mag_raw.has_z = true;

        // get gyro value
        bno_msg->has_gyro_raw = true;
        bno_msg->gyro_raw.x = (int16_t)((rx_data[13] << 8)| rx_data[12]);
        bno_msg->gyro_raw.has_x = true;
        bno_msg->gyro_raw.y = (int16_t)((rx_data[15] << 8)| rx_data[14]);
        bno_msg->gyro_raw.has_y = true;
        bno_msg->gyro_raw.z = (int16_t)((rx_data[17] << 8)| rx_data[16]);
        bno_msg->gyro_raw.has_z = true;
        break;

      case E_BNO055_MODE_FUSION:
        // get quaternion value
        bno_msg->has_quaternion_raw = true;
        bno_msg->quaternion_raw.w = (int16_t)((rx_data[1] << 8) | rx_data[0]);
        bno_msg->quaternion_raw.has_w = true;
        bno_msg->quaternion_raw.x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
        bno_msg->quaternion_raw.has_x = true;
        bno_msg->quaternion_raw.y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
        bno_msg->quaternion_raw.has_y = true;
        bno_msg->quaternion_raw.z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
        bno_msg->quaternion_raw.has_z = true;

        // get mag value
        bno_msg->has_linear_accel_raw = true;
        bno_msg->linear_accel_raw.x = (int16_t)((rx_data[9] << 8) | rx_data[8]);
        bno_msg->linear_accel_raw.has_x = true;
        bno_msg->linear_accel_raw.y = (int16_t)((rx_data[11] << 8) | rx_data[10]);
        bno_msg->linear_accel_raw.has_y = true;
        bno_msg->linear_accel_raw.z = (int16_t)((rx_data[13] << 8) | rx_data[12]);
        bno_msg->linear_accel_raw.has_z = true;

        // get gyro value
        bno_msg->has_gravity_vector_raw = true;
        bno_msg->gravity_vector_raw.x = (int16_t)((rx_data[15] << 8)| rx_data[14]);
        bno_msg->gravity_vector_raw.has_x = true;
        bno_msg->gravity_vector_raw.y = (int16_t)((rx_data[17] << 8)| rx_data[16]);
        bno_msg->gravity_vector_raw.has_y = true;
        bno_msg->gravity_vector_raw.z = (int16_t)((rx_data[19] << 8)| rx_data[18]);
        bno_msg->gravity_vector_raw.has_z = true;
        break;

      case E_BNO055_MODE_ALL:
        // get accel value
        bno_msg->has_accel_raw = true;
        bno_msg->accel_raw.x = (int16_t)((rx_data[1] << 8) | rx_data[0]);
        bno_msg->accel_raw.has_x = true;
        bno_msg->accel_raw.y = (int16_t)((rx_data[3] << 8) | rx_data[2]);
        bno_msg->accel_raw.has_y = true;
        bno_msg->accel_raw.z = (int16_t)((rx_data[5] << 8) | rx_data[4]);
        bno_msg->accel_raw.has_z = true;

        // get mag value
        bno_msg->has_mag_raw = true;
        bno_msg->mag_raw.x = (int16_t)((rx_data[7] << 8) | rx_data[6]);
        bno_msg->mag_raw.has_x = true;
        bno_msg->mag_raw.y = (int16_t)((rx_data[9] << 8) | rx_data[8]);
        bno_msg->mag_raw.has_y = true;
        bno_msg->mag_raw.z = (int16_t)((rx_data[11] << 8) | rx_data[10]);
        bno_msg->mag_raw.has_z = true;

        // get gyro value
        bno_msg->has_gyro_raw = true;
        bno_msg->gyro_raw.x = (int16_t)((rx_data[13] << 8)| rx_data[12]);
        bno_msg->gyro_raw.has_x = true;
        bno_msg->gyro_raw.y = (int16_t)((rx_data[15] << 8)| rx_data[14]);
        bno_msg->gyro_raw.has_y = true;
        bno_msg->gyro_raw.z = (int16_t)((rx_data[17] << 8)| rx_data[16]);
        bno_msg->gyro_raw.has_z = true;

        // get quaternion value
        bno_msg->has_quaternion_raw = true;
        bno_msg->quaternion_raw.w = (int16_t)((rx_data[25] << 8) | rx_data[24]);
        bno_msg->quaternion_raw.has_w = true;
        bno_msg->quaternion_raw.x = (int16_t)((rx_data[27] << 8) | rx_data[26]);
        bno_msg->quaternion_raw.has_x = true;
        bno_msg->quaternion_raw.y = (int16_t)((rx_data[29] << 8) | rx_data[28]);
        bno_msg->quaternion_raw.has_y = true;
        bno_msg->quaternion_raw.z = (int16_t)((rx_data[31] << 8) | rx_data[30]);
        bno_msg->quaternion_raw.has_z = true;

        // get linear accel value
        bno_msg->has_linear_accel_raw = true;
        bno_msg->linear_accel_raw.x = (int16_t)((rx_data[33] << 8) | rx_data[32]);
        bno_msg->linear_accel_raw.has_x = true;
        bno_msg->linear_accel_raw.y = (int16_t)((rx_data[35] << 8) | rx_data[34]);
        bno_msg->linear_accel_raw.has_y = true;
        bno_msg->linear_accel_raw.z = (int16_t)((rx_data[37] << 8)| rx_data[36]);
        bno_msg->linear_accel_raw.has_z = true;

        // get gravity value
        bno_msg->has_gravity_vector_raw = true;
        bno_msg->gravity_vector_raw.x = (int16_t)((rx_data[39] << 8)| rx_data[38]);
        bno_msg->gravity_vector_raw.has_x = true;
        bno_msg->gravity_vector_raw.y = (int16_t)((rx_data[41] << 8)| rx_data[40]);
        bno_msg->gravity_vector_raw.has_y = true;
        bno_msg->gravity_vector_raw.z = (int16_t)((rx_data[43] << 8)| rx_data[42]);
        bno_msg->gravity_vector_raw.has_z = true;
        // get temperature
        break;
    }
  }
}

/* ===========================================================================*/
/*                         loc_setOperationMode()						  	  */
/* ===========================================================================*/
bool loc_setOperationMode(E_BNO055_MODE_t mode) {
	bool b_return = false;
	if ((ge_bno_state == E_BNO055_STATE_IDLE_WRITE)||(ge_bno_state == E_BNO055_STATE_IDLE_READ)) {
		ge_operation_mode = mode;
		ge_bno_state = E_BNO055_STATE_INIT_PENDING;
		uint8_t data[2];
		data[0] = REG_OPR_MODE;
		switch(mode) {
			case E_BNO055_MODE_NORMAL:
				data[1] = OPR_MODE_AMG;
				break;
			case E_BNO055_MODE_FUSION:
				data[1] = OPR_MODE_NDOF;
				break;
			case E_BNO055_MODE_ALL:
				data[1] = OPR_MODE_NDOF;
				break;
			default:
				data[1] = OPR_MODE_AMG;
				break;
		}
    BNO055_bno_timeout = hal_timer_getTimeout(BNO_TIMEOUT_DELAY);
    BNO055_update_timer = hal_timer_getTimeout(BNO_REFRESH_PERIOD);
		hal_i2c_write(bno055_i2c_ctx, SLAVE_ADDRESS, data,2, loc_bno_callbackToIdle);

		// Needed for init maybe better solution later!
		__delay_cycles (1000000);

		b_return = true;
	} else {
		b_return = false;
	}
	return b_return;
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       bno055_init() Initializes bno055 and returns true iff success        */
/* ===========================================================================*/
bool bno055_init() {
	bool b_return = false;

	g_bno_lastData = g_bno_imuDefault;
	/* Initialize our context struct and init I2C port. */
	ge_bno_state = E_BNO055_STATE_IDLE_WRITE;
	bno055_i2c_ctx = hal_i2c_ti_drv_init(BNO055_I2C_PORT);
	if (bno055_i2c_ctx != NULL) b_return = true;


  if(b_return)
  {
    b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_Q, HAL_GPIO_PIN_4, E_HAL_GPIO_DIR_OUT);
  }
  if(b_return)
  {
    b_return = hal_gpio_write(E_HAL_GPIO_PORT_Q, HAL_GPIO_PIN_4, HAL_GPIO_PIN_4);
  }
  if(b_return)
  {
    b_return = hal_gpio_setDir(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_2, E_HAL_GPIO_DIR_OUT);
  }
  if(b_return)
  {
    b_return = hal_gpio_write(E_HAL_GPIO_PORT_N, HAL_GPIO_PIN_2, HAL_GPIO_PIN_2);
  }

  if(b_return)
  {
    b_return = hal_gpio_write(E_HAL_GPIO_PORT_Q, HAL_GPIO_PIN_4, ~HAL_GPIO_PIN_4);
    __delay_cycles(10);
  }
  if(b_return)
  {
    b_return = hal_gpio_write(E_HAL_GPIO_PORT_Q, HAL_GPIO_PIN_4, HAL_GPIO_PIN_4);
  }
  __delay_cycles(78000000);




	/* Set the operation mode of the bno055. */
	loc_setOperationMode(E_BNO055_MODE_ALL);

	gb_new_data_av = false;

  #if BNO055_ENABLE_AFTER_START
	gb_bnoEnabled = true;
  #else
	/*! By default this module is disabled after init(). It must be enabled
	 * by a call to \ref setConf() with corresponding member set to true. */
	gb_bnoEnabled = false;
  #endif
    /* By default this module is disabled after init(). It must be enabled
     * by a call to \ref setConf() with corresponding member set to true. */
  BNO055_update_timer = hal_timer_getTimeout(BNO_STARTUP_DELAY);

    return (b_return);
}

/* ===========================================================================*/
/*  servo_dude_global_run() Handles the global ServoDude-StateMachine         */
/* ===========================================================================*/

void bno055_run() {
	if( gb_bnoEnabled ) {
		uint8_t read_addr;
		uint8_t read_len;
		switch (ge_operation_mode) {
			case E_BNO055_MODE_NORMAL:
				read_addr = REG_ACC_DATA_X_LSB;
				read_len = 18;
				break;
			case E_BNO055_MODE_FUSION:
				read_addr = REG_QUA_DATA_W_LSB;
				read_len = 20;
				break;
			case E_BNO055_MODE_ALL:
				read_addr = REG_ACC_DATA_X_LSB;
				read_len = 45;
				break;
			default:
				read_addr = REG_ACC_DATA_X_LSB;
				read_len = 18;
				break;
		}
		switch(ge_bno_state){
	  	  	  case E_BNO055_STATE_IDLE_WRITE:
	  	  	    if (hal_timer_isTimedOut(BNO055_update_timer)) {
	  	  	      BNO055_update_timer = hal_timer_getTimeout(BNO_REFRESH_PERIOD);
                BNO055_bno_timeout = hal_timer_getTimeout(BNO_TIMEOUT_DELAY);
	  	  	      ge_bno_state = E_BNO055_STATE_PENDING_WRITE;
	  	  	      hal_i2c_write(bno055_i2c_ctx, SLAVE_ADDRESS, &read_addr, 1, loc_bno_callbackToIdleRead);
	  	  	    }
	  	  		  break;
	  	  	  case E_BNO055_STATE_IDLE_READ:
	  	  		  ge_bno_state = E_BNO055_STATE_PENDING_READ;
	  	  		  hal_i2c_read(bno055_i2c_ctx, SLAVE_ADDRESS, &read_addr, 0, rx_data, read_len, loc_bno_callbackToIdleWrite);
	  	  		  break;
	  	  	  case E_BNO055_STATE_INIT_PENDING:
	  	  	    if (hal_timer_isTimedOut(BNO055_bno_timeout)) {
	  	  	      ge_bno_state = E_BNO055_STATE_TIMEOUT;
	  	  	    }
	  	  		  break;
	  	  	  case E_BNO055_STATE_PENDING_WRITE:
              if (hal_timer_isTimedOut(BNO055_bno_timeout)) {
                ge_bno_state = E_BNO055_STATE_TIMEOUT;
              }
              break;
              case E_BNO055_STATE_PENDING_READ:
              if (hal_timer_isTimedOut(BNO055_bno_timeout)) {
                ge_bno_state = E_BNO055_STATE_TIMEOUT;
              }
                  break;
              case E_BNO055_STATE_DATA_READY:
              if (hal_timer_isTimedOut(BNO055_bno_timeout)) {
                ge_bno_state = E_BNO055_STATE_TIMEOUT;
              }
                  break;
	  	  	  case E_BNO055_STATE_TIMEOUT:
	  	  	    ge_bno_state = E_BNO055_STATE_TIMEOUT;
	  	  	    break;
	  	  	  default:
	  	  		  break;
		}
  }
	return;
}

/* ===========================================================================*/
/*    BNO055_get() 									                              */
/* ===========================================================================*/
bool bno055_get(Imu* const bno_msg) {
  bool b_return = false;

  /* Proceed only if module is enabled and a valid pointer is given. */
  if( gb_bnoEnabled && bno_msg && gb_new_data_av) {
    gb_new_data_av = false;
    *bno_msg = g_bno_lastData;
    b_return = true;
    ge_bno_state = E_BNO055_STATE_IDLE_WRITE;
  }

  return b_return;
}

/* ===========================================================================*/
/*    BNO055_setConf()               			                              */
/* ===========================================================================*/
bool BNO055_setConf(ImuConf * const p_bnoConf)
{
  bool b_return = false;

  if( p_bnoConf )
  {
    gb_bnoEnabled = p_bnoConf->basic_conf.activated;
    b_return = gb_bnoEnabled;
    /* Parse further conf members here and perform corresponding setting. */
  }

  return b_return;
}
