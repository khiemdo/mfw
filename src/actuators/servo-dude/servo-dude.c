/* Copyright (C) 2015-2017
 *
 * servo-dude.c
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

#include "servo-dude.h"

#include "hal_timer.h"
#include "i2c-handler.h"

/* ===========================================================================*/
/*                    MACROS                                                  */
/* ===========================================================================*/
/*! Define servodude-i2c-parameters for elevator*/
#define SD_ELEVATOR_PORT E_HAL_I2C_PORT_0
#define SD_ELEVATOR_ADDR 0x0A
#define SD_RUDDER_PORT E_HAL_I2C_PORT_0
#define SD_RUDDER_ADDR 0x0B
#define SD_AILERON_RIGHT_PORT E_HAL_I2C_PORT_1
#define SD_AILERON_RIGHT_ADDR 0x0C
#define SD_FLAP_RIGHT_PORT E_HAL_I2C_PORT_1
#define SD_FLAP_RIGHT_ADDR 0x0D
#define SD_AILERON_LEFT_PORT E_HAL_I2C_PORT_2
#define SD_AILERON_LEFT_ADDR 0x0E
#define SD_FLAP_LEFT_PORT E_HAL_I2C_PORT_2
#define SD_FLAP_LEFT_ADDR 0x0F

/*! Time in ms where servodude-values are updated*/
#define SRV_DUD_REFRESH_PERIOD 5
/*! Time in ms where timeout error occurrs (when i2c-transmission fails)*/
#define SRV_DUD_TIMEOUT_DELAY 30

/* ===========================================================================*/
/*                    TYPEDEFS                                                */
/* ===========================================================================*/
/*! Available servo-dude-states provided and handled by the run() function. */
typedef enum
{
  E_SERVO_DUDE_STATE_INACTIVE,
  E_SERVO_DUDE_STATE_IDLE,
  E_SERVO_DUDE_STATE_PENDING,
  E_SERVO_DUDE_STATE_TIMEOUT,

} E_SERVO_DUDE_STATE_t;

/*! Structure defining the context of a servo-dude. */
struct S_SERVO_DUDE_CTX_T
{
  s_i2c_hand_t *i2c_ctx;
  E_SERVO_DUDE_ID_t servo_id;
  E_SERVO_DUDE_STATE_t state;
  uint16_t set_point;
  uint8_t set_point_buffer[2];
  bool set_point_av;
  uint32_t timeout;
  uint32_t update_rate;
  s_i2c_hand_wtask_t write_task;
};

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
/*! Global servo-dude instances*/
s_servo_dude_ctx_t gs_sd_el;
s_servo_dude_ctx_t gs_sd_rd;
s_servo_dude_ctx_t gs_sd_fl;
s_servo_dude_ctx_t gs_sd_fr;
s_servo_dude_ctx_t gs_sd_al;
s_servo_dude_ctx_t gs_sd_ar;

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_el();
static s_servo_dude_ctx_t *loc_init_sd_rd();
static s_servo_dude_ctx_t *loc_init_sd_fl();
static s_servo_dude_ctx_t *loc_init_sd_fr();
static s_servo_dude_ctx_t *loc_init_sd_al();
static s_servo_dude_ctx_t *loc_init_sd_ar();

/* ===========================================================================*/
/*                         loc_el_callback() - callback	                      */
/* ===========================================================================*/
void loc_el_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_el.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_rd_callback() - callback	                      */
/* ===========================================================================*/
void loc_rd_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_rd.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_fl_callback() - callback	                      */
/* ===========================================================================*/
void loc_fl_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_fl.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_fr_callback() - callback	                      */
/* ===========================================================================*/
void loc_fr_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_fr.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_al_callback() - callback	                      */
/* ===========================================================================*/
void loc_al_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_al.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_ar_callback() - callback	                      */
/* ===========================================================================*/
void loc_ar_callback(void *pvData, uint_fast8_t ui8Status)
{
  gs_sd_ar.state = E_SERVO_DUDE_STATE_IDLE;
}

/* ===========================================================================*/
/*                         loc_init_sd_el() - Init Elevator                   */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_el()
{
  /* Initialize our context struct. */
  memset(&gs_sd_el, 0, sizeof(gs_sd_el));
  gs_sd_el.i2c_ctx = i2c_handler_init(SD_ELEVATOR_PORT);
  gs_sd_el.servo_id = E_SERVO_DUDE_ELEVATOR;
  gs_sd_el.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_el.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_el.set_point_av = false;
  gs_sd_el.write_task.priority = 0x00;
  gs_sd_el.write_task.slave_address = SD_ELEVATOR_ADDR;
  gs_sd_el.write_task.write_data = gs_sd_el.set_point_buffer;
  gs_sd_el.write_task.write_length = 2;
  gs_sd_el.write_task.callback_pointer = &loc_el_callback;
  return &gs_sd_el;
}

/* ===========================================================================*/
/*                         loc_init_sd_rd() - Init Rudder                     */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_rd()
{
  /* Initialize our context struct. */
  memset(&gs_sd_rd, 0, sizeof(gs_sd_rd));
  gs_sd_rd.i2c_ctx = i2c_handler_init(SD_RUDDER_PORT);
  gs_sd_rd.servo_id = E_SERVO_DUDE_RUDDER;
  gs_sd_rd.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_rd.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_rd.set_point_av = false;
  gs_sd_rd.write_task.priority = 0x00;
  gs_sd_rd.write_task.slave_address = SD_RUDDER_ADDR;
  gs_sd_rd.write_task.write_data = gs_sd_rd.set_point_buffer;
  gs_sd_rd.write_task.write_length = 2;
  gs_sd_rd.write_task.callback_pointer = &loc_rd_callback;
  return &gs_sd_rd;
}

/* ===========================================================================*/
/*                         loc_init_sd_fl() - Init Flap Left                  */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_fl()
{
  /* Initialize our context struct. */
  memset(&gs_sd_fl, 0, sizeof(gs_sd_fl));
  gs_sd_fl.i2c_ctx = i2c_handler_init(SD_FLAP_LEFT_PORT);
  gs_sd_fl.servo_id = E_SERVO_DUDE_FLAP_LEFT;
  gs_sd_fl.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_fl.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_fl.set_point_av = false;
  gs_sd_fl.write_task.priority = 0x00;
  gs_sd_fl.write_task.slave_address = SD_FLAP_LEFT_ADDR;
  gs_sd_fl.write_task.write_data = gs_sd_fl.set_point_buffer;
  gs_sd_fl.write_task.write_length = 2;
  gs_sd_fl.write_task.callback_pointer = &loc_fl_callback;
  return &gs_sd_fl;
}

/* ===========================================================================*/
/*                         loc_init_sd_fr() - Init Flap Right                 */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_fr()
{
  /* Initialize our context struct. */
  memset(&gs_sd_fr, 0, sizeof(gs_sd_fr));
  gs_sd_fr.i2c_ctx = i2c_handler_init(SD_FLAP_RIGHT_PORT);
  gs_sd_fr.servo_id = E_SERVO_DUDE_FLAP_RIGHT;
  gs_sd_fr.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_fr.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_fr.set_point_av = false;
  gs_sd_fr.write_task.priority = 0x00;
  gs_sd_fr.write_task.slave_address = SD_FLAP_RIGHT_ADDR;
  gs_sd_fr.write_task.write_data = gs_sd_fr.set_point_buffer;
  gs_sd_fr.write_task.write_length = 2;
  gs_sd_fr.write_task.callback_pointer = &loc_fr_callback;
  return &gs_sd_fr;
}

/* ===========================================================================*/
/*                         loc_init_sd_al() - Init Aileron Left               */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_al()
{
  /* Initialize our context struct. */
  memset(&gs_sd_al, 0, sizeof(gs_sd_al));
  gs_sd_al.i2c_ctx = i2c_handler_init(SD_AILERON_LEFT_PORT);
  gs_sd_al.servo_id = E_SERVO_DUDE_AILERON_LEFT;
  gs_sd_al.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_al.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_al.set_point_av = false;
  gs_sd_al.write_task.priority = 0x00;
  gs_sd_al.write_task.slave_address = SD_AILERON_LEFT_ADDR;
  gs_sd_al.write_task.write_data = gs_sd_al.set_point_buffer;
  gs_sd_al.write_task.write_length = 2;
  gs_sd_al.write_task.callback_pointer = &loc_al_callback;
  return &gs_sd_al;
}

/* ===========================================================================*/
/*                         loc_init_sd_ar() - Init Aileron Right                 */
/* ===========================================================================*/
static s_servo_dude_ctx_t *loc_init_sd_ar()
{
  /* Initialize our context struct. */
  memset(&gs_sd_ar, 0, sizeof(gs_sd_ar));
  gs_sd_ar.i2c_ctx = i2c_handler_init(SD_AILERON_RIGHT_PORT);
  gs_sd_ar.servo_id = E_SERVO_DUDE_AILERON_RIGHT;
  gs_sd_ar.state = E_SERVO_DUDE_STATE_IDLE;
  gs_sd_ar.update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
  gs_sd_ar.set_point_av = false;
  gs_sd_ar.write_task.priority = 0x00;
  gs_sd_ar.write_task.slave_address = SD_AILERON_RIGHT_ADDR;
  gs_sd_ar.write_task.write_data = gs_sd_ar.set_point_buffer;
  gs_sd_ar.write_task.write_length = 2;
  gs_sd_ar.write_task.callback_pointer = &loc_ar_callback;
  return &gs_sd_ar;
}
/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       servo_dude_init() Initializes a ServoDude and returns context        */
/* ===========================================================================*/
s_servo_dude_ctx_t *servo_dude_init(E_SERVO_DUDE_ID_t e_servo_dude_id)
{
  s_servo_dude_ctx_t *ps_return = NULL;
  switch(e_servo_dude_id)
  {
  case E_SERVO_DUDE_ELEVATOR:
    ps_return = loc_init_sd_el();
    break;
  case E_SERVO_DUDE_RUDDER:
    ps_return = loc_init_sd_rd();
    break;
  case E_SERVO_DUDE_FLAP_LEFT:
    ps_return = loc_init_sd_fl();
    break;
  case E_SERVO_DUDE_FLAP_RIGHT:
    ps_return = loc_init_sd_fr();
    break;
  case E_SERVO_DUDE_AILERON_LEFT:
    ps_return = loc_init_sd_al();
    break;
  case E_SERVO_DUDE_AILERON_RIGHT:
    ps_return = loc_init_sd_ar();
    break;
  /* Other ServoDude's are not supported => ps_return remains NULL. */
  default:
    break;
  }

  return ps_return;
}

/* ===========================================================================*/
/*       servo_dude_run() Handles the ServoDude-StateMachine                  */
/* ===========================================================================*/
void servo_dude_run(s_servo_dude_ctx_t *const servodude_ctx)
{
  //! Transform setpoint to 8-bit array*/
  if(servodude_ctx->set_point_av)
  {
    //! Handle the statemachine
    switch(servodude_ctx->state)
    {
    case E_SERVO_DUDE_STATE_IDLE:
      //! Check if servodude needs to be updated
      if(hal_timer_isTimedOut(servodude_ctx->update_rate))
      {
        //! Get new timeout values
        servodude_ctx->update_rate = hal_timer_getTimeout(SRV_DUD_REFRESH_PERIOD);
        servodude_ctx->timeout = hal_timer_getTimeout(SRV_DUD_TIMEOUT_DELAY);
        //! Switch to pending state
        servodude_ctx->state = E_SERVO_DUDE_STATE_PENDING;
        //! Set servodude setpoint according to its ID
        servodude_ctx->set_point_av = false;
        i2c_handler_write(servodude_ctx->i2c_ctx, &(servodude_ctx->write_task));
      }
      break;
    case E_SERVO_DUDE_STATE_PENDING:
      //! Check if i2c-error-timeout is already over.
      i2c_handler_run();
      if(hal_timer_isTimedOut(servodude_ctx->timeout))
      {
        servodude_ctx->state = E_SERVO_DUDE_STATE_TIMEOUT;
      }
      break;
    case E_SERVO_DUDE_STATE_TIMEOUT:
      break;
    case E_SERVO_DUDE_STATE_INACTIVE:
      break;
    }
  }
}

/* ===========================================================================*/
/*    servo_dude_set() Reads actuator message, sends value to servo-dude      */
/* ===========================================================================*/
void servo_dude_set(s_servo_dude_ctx_t *const servodude_ctx, const Actuator *const actuator_msg)
{
  if(actuator_msg->has_set_point)
  {
    servodude_ctx->set_point = (uint16_t)actuator_msg->set_point;
    servodude_ctx->set_point_buffer[0] = (uint8_t)(servodude_ctx->set_point >> 8) & 0x3F;
    servodude_ctx->set_point_buffer[1] = (uint8_t)servodude_ctx->set_point;
    servodude_ctx->set_point_av = true;
  }
}
