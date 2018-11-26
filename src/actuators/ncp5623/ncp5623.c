/* Copyright (C) 2015-2017
 *
 * ncp5623.c
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Martin Dold         <martin.dold@gmx.net>
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

#include <string.h>
#include <stdlib.h>

#include "ncp5623.h"
#include "i2c-bus-manager.h"

#include "hal_timer.h"

/*! Adress of the LED controller. This value is hard-coded into the hardware and cannot be changed.
 *  Binary representation from the datasheet: \b 0b0111000 */
#define NCP5623_ADDR 0x38
/*! I2C-Port used by the LED controller. */
#define NCP5623_I2C_PORT E_HAL_I2C_PORT_5
/*! Bitmap for masking data in combination with command.  Binary representation from
 *  the datasheet: \b 0bXXX 11111 */
#define NCP5623_CMD_MASK 0x1F
/*! Command for soft reset of the LED controller. Binary representation from the datasheet:
 *  \b 0b000 XXXXX */
#define NCP5623_CMD_SHUT_DOWN 0x00
/*! Command for setting current step of the LED controller. Binary representation from the datasheet:
 *  \b 0b001 XXXXX */
#define NCP5623_CMD_ILED 0x20
/*! Command for setting Red PWM of the LED controller. Binary representation from the datasheet:
 *  \b 0b010 XXXXX */
#define NCP5623_CMD_RED_PWM 0x40
/*! Command for setting Green PWM of the LED controller. Binary representation from the datasheet:
 *  \b 0b011 XXXXX */
#define NCP5623_CMD_GREEN_PWM 0x60
/*! Command for setting Blue PWM of the LED controller. Binary representation from the datasheet:
 *  \b 0b100 XXXXX */
#define NCP5623_CMD_BLUE_PWM 0x80
/*! Command for setting Dimm Up Target of the LED controller. Binary representation from the datasheet:
 *  \b 0b101 XXXXX */
#define NCP5623_CMD_DIMM_UP 0xA0
/*! Command for setting Dimm Down Target of the LED controller. Binary representation from the datasheet:
 *  \b 0b110 XXXXX */
#define NCP5623_CMD_DIMM_DOWN 0xC0
/*! Command for setting Dimm Time of the LED controller. Binary representation from the datasheet:
 *  \b 0b111 XXXXX */
#define NCP5623_CMD_DIMM_TIME 0xE0

/* ===========================================================================*/
/*                    States of the state machines                            */
/* ===========================================================================*/
/*! State machine of the LED controller. */
typedef enum
{
	E_NCP5623_STATE_PENDING, /*!< In this state, a transmission via I2C is pending. */
	E_NCP5623_STATE_DIMMING, /*!< In this state, dimming is in progress. */
    E_NCP5623_STATE_IDLE,    /*!< In this state, LED controller is ready for new data. */
	E_NCP5623_STATE_TIMEOUT  /*!< In this state, the LED controller hasn't responded in time and may be missing. */
} E_NCP5623_STATE_t;

/*! State machine of the set function. */
typedef enum
{
    E_NCP5623_LAST_RED_PWM,  /*!< Last sent value was Red PWM. */
    E_NCP5623_LAST_GREEN_PWM,/*!< Last sent value was Green PWM. */
    E_NCP5623_LAST_BLUE_PWM, /*!< Last sent value was Blue PWM. */
    E_NCP5623_LAST_ILED,     /*!< Last sent value was current step. */
    E_NCP5623_LAST_DIMM_TIME /*!< Last sent value was dimming time. */
} E_NCP5623_LAST_t;

/*! State machine of the rainbow generator. */
typedef enum
{
    E_NCP5623_RAINBOW_P1,    /*!< Phase 1: Ramping down Blue PWM. */
    E_NCP5623_RAINBOW_P2,    /*!< Phase 2: Ramping up Green PWM. */
    E_NCP5623_RAINBOW_P3,    /*!< Phase 3: Ramping down Red PWM. */
    E_NCP5623_RAINBOW_P4,    /*!< Phase 4: Ramping up Blue PWM. */
    E_NCP5623_RAINBOW_P5,    /*!< Phase 5: Ramping down Green PWM. */
    E_NCP5623_RAINBOW_P6     /*!< Phase 6: Ramping up Red PWM. */
} E_NCP5623_RAINBOW_t;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
s_hal_i2c_ctx_t *ncp5623_i2c_ctx;           /*!< I2C context for the LED controller. */
static s_NCP5623_DATA_t NCP5623_data;              /*!< Data known to have been written to the LED controller. */
static s_NCP5623_DATA_t NCP5623_data_new;          /*!< Data that should be written to the LED controller. */
static E_NCP5623_STATE_t NCP5623_state;            /*!< State variable for the main state machine. */
static E_NCP5623_LAST_t NCP5623_last;              /*!< State variable for the set function state machine. */
static E_NCP5623_RAINBOW_t NCP5623_rainbow_state;  /*!< State variable for the rainbow state machine. */
static uint32_t NCP5623_rainbow_timer;             /*!< Timer for generating the rainbow. */
static uint16_t NCP5623_rainbow_speed;             /*!< Time delay between two steps of the rainbow function. */
static uint32_t NCP5623_dimming_finished;          /*!< Timer for the dimming. */

static uint8_t NCP5623_write_data[1];
static s_i2c_bus_mgr_write_t NCP5623_bm_write;

/* ===========================================================================*/
/*                         loc_callback()						        	  */
/* ===========================================================================*/
/*!
 * /brief Callback function for I2C library.
 *
 * /detail This callback function is handed to the I2C-HAL and called when the transmission has finished.
 *         It then set the main state machine back to IDLE state.
 *
 * /param pvData Void pointer for handing back data structures. Not used in our case.
 * /param ui8Status Status information from the I2C library
 *
 * /return None.
 */
static void loc_NCP5623_cb(void *pvData, uint_fast8_t ui8Status) {
	NCP5623_state = E_NCP5623_STATE_IDLE;
	ncp5623_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	i2c_bus_mgr_return_access(E_I2C_BUS_NODE_LED_DRV);
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       NCP5623_init() Initializes NCP5623 and returns true iff success      */
/* ===========================================================================*/
bool NCP5623_init()
{
  bool b_return = false;
  NCP5623_data.ILED = 0;
  NCP5623_data.time = 0;
  NCP5623_data.R = 0;
  NCP5623_data.G = 0;
  NCP5623_data.B = 0;

  NCP5623_data_new.ILED = 1;
  NCP5623_data_new.time = 0;
  NCP5623_data_new.R = 0;
  NCP5623_data_new.G = 0;
  NCP5623_data_new.B = 0;

  NCP5623_dimming_finished = 0;
  ncp5623_i2c_ctx = hal_i2c_ti_drv_init(NCP5623_I2C_PORT);

  if(ncp5623_i2c_ctx != NULL)
  {
    b_return = true;
  }

  NCP5623_bm_write.ps_ctx = ncp5623_i2c_ctx;
  NCP5623_bm_write.slave_address = NCP5623_ADDR;
  NCP5623_bm_write.write_length = 1;
  NCP5623_bm_write.write_data = NCP5623_write_data;
  NCP5623_bm_write.callback_pointer = &loc_NCP5623_cb;

  NCP5623_state = E_NCP5623_STATE_IDLE;
  NCP5623_last = E_NCP5623_LAST_DIMM_TIME;
  NCP5623_rainbow_state = E_NCP5623_RAINBOW_P1;
  NCP5623_rainbow_timer = 0;
  NCP5623_rainbow_speed = 100;

  NCP5623_shutdown();

  return b_return;
}

/* ===========================================================================*/
/*       NCP5623_run() handles the state machine of NCP5623				      */
/* ===========================================================================*/
void NCP5623_run()
{
	switch(NCP5623_state){
	      case E_NCP5623_STATE_PENDING:
              break;
	      case E_NCP5623_STATE_DIMMING:
	          if(hal_timer_isTimedOut(NCP5623_dimming_finished)){
	              NCP5623_state = E_NCP5623_STATE_IDLE;
	          }
	          break;
	  	  case E_NCP5623_STATE_IDLE:
	  	      if(hal_timer_isTimedOut(NCP5623_rainbow_timer))
	  	        NCP5623_rainbow(NCP5623_rainbow_speed);
	  		  NCP5623_set();
	  		  break;
	  	  case E_NCP5623_STATE_TIMEOUT:
	  		  break;
	  	  default:
	  	      break;
	}
}

void NCP5623_shutdown()
{
  //uint8_t data[1];
  NCP5623_write_data[0] =  NCP5623_CMD_SHUT_DOWN;
  NCP5623_state = E_NCP5623_STATE_PENDING;
  i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
  //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
}
/* ===========================================================================*/
/*       NCP5623_get()  returns the value of NCP5623					      */
/* ===========================================================================*/
void NCP5623_set()
{
  bool b_stay = true;
  //uint8_t data[1];
  while(b_stay)
  {
	switch(NCP5623_last){
	  case E_NCP5623_LAST_DIMM_TIME:
	    NCP5623_last = E_NCP5623_LAST_BLUE_PWM;
	    if(NCP5623_data.B != NCP5623_data_new.B){
	      NCP5623_state = E_NCP5623_STATE_PENDING;
	      b_stay = false;
	      NCP5623_write_data[0] = NCP5623_CMD_RED_PWM | ((NCP5623_data_new.B >> 3) & NCP5623_CMD_MASK); //Red and Blue are swapped
	      NCP5623_data.B = NCP5623_data_new.B;
	      i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
	      //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
	    }
	    break;
	  case E_NCP5623_LAST_BLUE_PWM:
	    NCP5623_last = E_NCP5623_LAST_GREEN_PWM;
        if(NCP5623_data.G != NCP5623_data_new.G){
          NCP5623_state = E_NCP5623_STATE_PENDING;
          b_stay = false;
          NCP5623_write_data[0] = NCP5623_CMD_GREEN_PWM | ((NCP5623_data_new.G >> 3) & NCP5623_CMD_MASK);
          NCP5623_data.G = NCP5623_data_new.G;
          i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
          //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
        }
        break;
      case E_NCP5623_LAST_GREEN_PWM:
        NCP5623_last = E_NCP5623_LAST_RED_PWM;
        if(NCP5623_data.R != NCP5623_data_new.R){
          NCP5623_state = E_NCP5623_STATE_PENDING;
          b_stay = false;
          NCP5623_write_data[0] = NCP5623_CMD_BLUE_PWM | ((NCP5623_data_new.R >> 3) & NCP5623_CMD_MASK); //Red and Blue are swapped
          NCP5623_data.R = NCP5623_data_new.R;
          i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
          //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
        }
        break;
      case E_NCP5623_LAST_RED_PWM:
        NCP5623_last = E_NCP5623_LAST_DIMM_TIME;
        b_stay = false;
        if(NCP5623_data.ILED != NCP5623_data_new.ILED){
          if(NCP5623_data_new.time == 0)
          {
            NCP5623_state = E_NCP5623_STATE_PENDING;
            NCP5623_write_data[0] = NCP5623_CMD_ILED | (NCP5623_data_new.ILED & NCP5623_CMD_MASK);
            NCP5623_data.ILED = NCP5623_data_new.ILED;
            NCP5623_data.time = NCP5623_data_new.time;
            i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
            //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
          }
          else
          {
            NCP5623_last = E_NCP5623_LAST_ILED;
            NCP5623_state = E_NCP5623_STATE_PENDING;
            if(NCP5623_data_new.ILED > NCP5623_data.ILED)
            {
              NCP5623_write_data[0] = NCP5623_CMD_DIMM_UP | (NCP5623_data_new.ILED & NCP5623_CMD_MASK);
              NCP5623_data.ILED = NCP5623_data_new.ILED;
              i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
              //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
            }
            else
            {
              NCP5623_write_data[0] = NCP5623_CMD_DIMM_DOWN | (NCP5623_data_new.ILED & NCP5623_CMD_MASK);
              NCP5623_data.ILED = NCP5623_data_new.ILED;
              i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
              //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
            }
          }
        }
        break;
      case E_NCP5623_LAST_ILED:
        NCP5623_last = E_NCP5623_LAST_DIMM_TIME;
        NCP5623_state = E_NCP5623_STATE_PENDING;
        NCP5623_dimming_finished = hal_timer_getTimeout(NCP5623_data_new.time);
        b_stay = false;
        NCP5623_write_data[0] = NCP5623_CMD_DIMM_TIME | ((NCP5623_data_new.time >> 3) & NCP5623_CMD_MASK);
        NCP5623_data.time = NCP5623_data_new.time;
        i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_LED_DRV, &NCP5623_bm_write);
        //hal_i2c_write(ncp5623_i2c_ctx, NCP5623_addr, data, 1, &loc_NCP5623_cb);
        break;
      default:
        break;

	}
  }
}

void NCP5623_new(s_NCP5623_DATA_t data){
  NCP5623_data_new = data;
}

void NCP5623_rainbow(uint16_t ui16_speed){
  NCP5623_rainbow_timer = hal_timer_getTimeout(ui16_speed);
  switch(NCP5623_rainbow_state)
  {
  case E_NCP5623_RAINBOW_P1:
    if(NCP5623_data_new.B != 0)
      NCP5623_data_new.B -= 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P2;
    break;
  case E_NCP5623_RAINBOW_P2:
    if(NCP5623_data_new.G != 248)
      NCP5623_data_new.G += 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P3;
    break;
  case E_NCP5623_RAINBOW_P3:
    if(NCP5623_data_new.R != 0)
      NCP5623_data_new.R -= 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P4;
    break;
  case E_NCP5623_RAINBOW_P4:
    if(NCP5623_data_new.B != 248)
      NCP5623_data_new.B += 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P5;
    break;
  case E_NCP5623_RAINBOW_P5:
    if(NCP5623_data_new.G != 0)
      NCP5623_data_new.G -= 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P6;
    break;
  case E_NCP5623_RAINBOW_P6:
    if(NCP5623_data_new.R != 248)
      NCP5623_data_new.R += 8;
    else
      NCP5623_rainbow_state = E_NCP5623_RAINBOW_P1;
    break;
  default:
    break;

  }
}

