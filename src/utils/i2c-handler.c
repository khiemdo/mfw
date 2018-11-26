/* Copyright (C) 2015-2017
 *
 * i2c-handler.c
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

#include <string.h>
#include "i2c-handler.h"


/* ===========================================================================*/
/*                    Global variables				                          */
/* ===========================================================================*/

s_i2c_hand_t gs_handler_port0;
s_i2c_hand_t gs_handler_port1;
s_i2c_hand_t gs_handler_port2;
s_i2c_hand_t gs_handler_port3;
s_i2c_hand_t gs_handler_port4;
s_i2c_hand_t gs_handler_port5;

bool port0_initialized = false;
bool port1_initialized = false;
bool port2_initialized = false;
bool port3_initialized = false;
bool port4_initialized = false;
bool port5_initialized = false;

/* ===========================================================================*/
/*                    Local function declarations	                          */
/* ===========================================================================*/

// These local callbacks are wrappers for the callback functions
// provided by the actual modules. They return access to I2C-port and increment the read-pointers of fifo.
void loc_i2c_handler_port0_wcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port1_wcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port2_wcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port3_wcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port4_wcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port5_wcallback(void *pvData, uint_fast8_t ui8Status);

void loc_i2c_handler_port0_rcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port1_rcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port2_rcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port3_rcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port4_rcallback(void *pvData, uint_fast8_t ui8Status);
void loc_i2c_handler_port5_rcallback(void *pvData, uint_fast8_t ui8Status);


void loc_i2c_handler_run_port(s_i2c_hand_t *i2c_port_handler, void (*wcallback_pointer)(void *pvData, uint_fast8_t ui8Status), void (*rcallback_pointer)(void *pvData, uint_fast8_t ui8Status));


/* ===========================================================================*/
/*                    Local function implementations                         */
/* ===========================================================================*/
// Local callback for write-operations
void loc_i2c_handler_port0_wcallback(void *pvData, uint_fast8_t ui8Status) {
	// Call the actual module's callbacks wrapped by the local callback.
	(gs_handler_port0.wqueue[gs_handler_port0.wqueue_rptr]->callback_pointer)(pvData, ui8Status);

	gs_handler_port0.wtask_active[gs_handler_port0.wqueue_rptr] = false;
}

// Local callback for read-operations
void loc_i2c_handler_port0_rcallback(void *pvData, uint_fast8_t ui8Status) {
	// Call the actual module's callbacks wrapped by the local callback.
	(gs_handler_port0.rqueue[gs_handler_port0.rqueue_rptr]->callback_pointer)(pvData, ui8Status);

	gs_handler_port0.rtask_active[gs_handler_port0.rqueue_rptr] = false;
}

// Local callback for write-operations
void loc_i2c_handler_port1_wcallback(void *pvData, uint_fast8_t ui8Status) {
    // Call the actual module's callbacks wrapped by the local callback.
    (gs_handler_port1.wqueue[gs_handler_port1.wqueue_rptr]->callback_pointer)(pvData, ui8Status);

    gs_handler_port1.wtask_active[gs_handler_port1.wqueue_rptr] = false;
}

// Local callback for read-operations
void loc_i2c_handler_port1_rcallback(void *pvData, uint_fast8_t ui8Status) {
    // Call the actual module's callbacks wrapped by the local callback.
    (gs_handler_port1.rqueue[gs_handler_port1.rqueue_rptr]->callback_pointer)(pvData, ui8Status);

    gs_handler_port1.rtask_active[gs_handler_port1.rqueue_rptr] = false;
}

// Local callback for write-operations
void loc_i2c_handler_port2_wcallback(void *pvData, uint_fast8_t ui8Status) {
    // Call the actual module's callbacks wrapped by the local callback.
    (gs_handler_port2.wqueue[gs_handler_port2.wqueue_rptr]->callback_pointer)(pvData, ui8Status);

    gs_handler_port2.wtask_active[gs_handler_port2.wqueue_rptr] = false;
}

// Local callback for read-operations
void loc_i2c_handler_port2_rcallback(void *pvData, uint_fast8_t ui8Status) {
    // Call the actual module's callbacks wrapped by the local callback.
    (gs_handler_port2.rqueue[gs_handler_port2.rqueue_rptr]->callback_pointer)(pvData, ui8Status);

    gs_handler_port2.rtask_active[gs_handler_port2.rqueue_rptr] = false;
}

// Function that returns the write-task-ID with the lowest Priority value (highest priority rank)
// of the provided write-task list.
uint8_t loc_i2c_handler_wpriorityID(s_i2c_hand_wtask_t **queue, bool * task_activity) {
	uint8_t lowest_prio = 0xFF;
	uint8_t lowest_id = 0xFF;
	uint8_t i;
	for (i = 0; i < I2C_HAND_QUEUE_LENGTH; i++) {
		if (task_activity[i]) {
			if (queue[i]->priority < lowest_prio) {
				lowest_prio = queue[i]->priority;
				lowest_id = i;
			}
			// Lower the priority value of the task
			// (ensures that every task is executed eventually)
			if (queue[i]->priority > 0) {
				queue[i]->priority -= 1;
			}
		}
	}
	return lowest_id;
}

// Function that returns the read-task-ID with the lowest Priority value (highest priority rank)
// of the provided read-task list.
uint8_t loc_i2c_handler_rpriorityID(s_i2c_hand_rtask_t **queue, bool * task_activity) {
	uint8_t lowest_prio = 0xFF;
	uint8_t lowest_id = 0xFF;
	uint8_t i;
	for (i = 0; i < I2C_HAND_QUEUE_LENGTH; i++) {
		if (task_activity[i]) {
			if (queue[i]->priority < lowest_prio) {
				lowest_prio = queue[i]->priority;
				lowest_id = i;
			}
			// Lower the priority value of the task
			// (ensures that every task is executed eventually)
			if (queue[i]->priority > 0) {
				queue[i]->priority -= 1;
			}
		}
	}
	return lowest_id;
}

void loc_i2c_handler_run_port(s_i2c_hand_t *i2c_port_handler, void (*wcallback_pointer)(void *pvData, uint_fast8_t ui8Status), void (*rcallback_pointer)(void *pvData, uint_fast8_t ui8Status)) {
    if (i2c_port_handler->i2c_ctx->state == E_HAL_I2C_STATE_IDLE) {

        // Find active task with lowest priority value
        i2c_port_handler->wqueue_rptr = loc_i2c_handler_wpriorityID(i2c_port_handler->wqueue, i2c_port_handler->wtask_active);
        if (i2c_port_handler->wqueue_rptr < 255) {
            s_hal_i2c_ctx_t * tmp_i2c_ctx = i2c_port_handler->i2c_ctx;
            uint8_t tmp_slave_addr = i2c_port_handler->wqueue[i2c_port_handler->wqueue_rptr]->slave_address;
            uint8_t *tmp_data = i2c_port_handler->wqueue[i2c_port_handler->wqueue_rptr]->write_data;
            uint8_t tmp_length = i2c_port_handler->wqueue[i2c_port_handler->wqueue_rptr]->write_length;

            hal_i2c_write(tmp_i2c_ctx, tmp_slave_addr, tmp_data, tmp_length, wcallback_pointer);
        } else {

            // Find active task with lowest priority value
            i2c_port_handler->rqueue_rptr = loc_i2c_handler_rpriorityID(i2c_port_handler->rqueue, i2c_port_handler->rtask_active);
            if (i2c_port_handler->rqueue_rptr < 255) {
                s_hal_i2c_ctx_t * tmp_i2c_ctx = i2c_port_handler->i2c_ctx;
                uint8_t tmp_slave_addr = i2c_port_handler->rqueue[i2c_port_handler->rqueue_rptr]->slave_address;
                uint8_t *tmp_wdata = i2c_port_handler->rqueue[i2c_port_handler->rqueue_rptr]->write_data;
                uint8_t tmp_wlength = i2c_port_handler->rqueue[i2c_port_handler->rqueue_rptr]->write_length;
                uint8_t *tmp_rdata = i2c_port_handler->rqueue[i2c_port_handler->rqueue_rptr]->read_data;
                uint8_t tmp_rlength = i2c_port_handler->rqueue[i2c_port_handler->rqueue_rptr]->read_length;

                hal_i2c_read(tmp_i2c_ctx, tmp_slave_addr, tmp_wdata, tmp_wlength, tmp_rdata, tmp_rlength, rcallback_pointer);
            }
        }
    }
}


/* ===========================================================================*/
/*                    Global function implementations                         */
/* ===========================================================================*/
s_i2c_hand_t *i2c_handler_init(E_HAL_I2C_PORT_t e_port) {
	s_i2c_hand_t * hand_ctx;
	switch (e_port) {
	  case E_HAL_I2C_PORT_0:
		  if(!port0_initialized) {
			  memset( &gs_handler_port0, 0, sizeof(gs_handler_port0));
			  gs_handler_port0.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port0_initialized = true;
		  }
          hand_ctx = &gs_handler_port0;
		  break;
	  case E_HAL_I2C_PORT_1:
		  if(!port1_initialized) {
			  memset( &gs_handler_port1, 0, sizeof(gs_handler_port1));
			  gs_handler_port1.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port1_initialized = true;
		  }
          hand_ctx = &gs_handler_port1;
		  break;
	  case E_HAL_I2C_PORT_2:
		  if(!port2_initialized) {
			  memset( &gs_handler_port2, 0, sizeof(gs_handler_port2));
			  gs_handler_port2.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port2_initialized = true;
		  }
          hand_ctx = &gs_handler_port2;
		  break;
	  case E_HAL_I2C_PORT_3:
		  if(!port3_initialized) {
			  memset( &gs_handler_port3, 0, sizeof(gs_handler_port3));
			  gs_handler_port3.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port3_initialized = true;
		  }
          hand_ctx = &gs_handler_port3;
		  break;
	  case E_HAL_I2C_PORT_4:
		  if(!port4_initialized) {
			  memset( &gs_handler_port4, 0, sizeof(gs_handler_port4));
			  gs_handler_port4.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port4_initialized = true;
		  }
          hand_ctx = &gs_handler_port4;
		  break;
	  case E_HAL_I2C_PORT_5:
		  if(!port5_initialized) {
			  memset( &gs_handler_port5, 0, sizeof(gs_handler_port5));
			  gs_handler_port5.i2c_ctx = hal_i2c_ti_drv_init(e_port);
			  port5_initialized = true;
		  }
          hand_ctx = &gs_handler_port5;
		  break;
	}
	return hand_ctx;
}

bool i2c_handler_write(s_i2c_hand_t * i2c_hand_ctx, s_i2c_hand_wtask_t *write_task) {
	uint8_t i;
	for(i = 0; i < I2C_HAND_QUEUE_LENGTH; i++) {
		if (!i2c_hand_ctx->wtask_active[i]) {
			i2c_hand_ctx->wtask_active[i] = true;
			i2c_hand_ctx->wqueue[i] = write_task;
			return true;
		}
	}
	return false;
}

bool i2c_handler_read(s_i2c_hand_t * i2c_hand_ctx, s_i2c_hand_rtask_t *read_task) {
	uint8_t i;
	for(i = 0; i < I2C_HAND_QUEUE_LENGTH; i++) {
		if (!i2c_hand_ctx->rtask_active[i]) {
			i2c_hand_ctx->rtask_active[i] = true;
			i2c_hand_ctx->rqueue[i] = read_task;
			return true;
		}
	}
	return false;
}

void i2c_handler_run() {
	if (port0_initialized) {
		loc_i2c_handler_run_port(&gs_handler_port0, loc_i2c_handler_port0_wcallback, loc_i2c_handler_port0_rcallback);
	}
	if (port1_initialized) {
        loc_i2c_handler_run_port(&gs_handler_port1, loc_i2c_handler_port1_wcallback, loc_i2c_handler_port1_rcallback);
	}
	if (port2_initialized) {
        loc_i2c_handler_run_port(&gs_handler_port2, loc_i2c_handler_port2_wcallback, loc_i2c_handler_port2_rcallback);
	}
//	if (port3_initialized) {
//		loc_i2c_handler_run_port3();
//	}
//	if (port4_initialized) {
//		loc_i2c_handler_run_port4();
//	}
//	if (port5_initialized) {
//		loc_i2c_handler_run_port5();
//	}
}
