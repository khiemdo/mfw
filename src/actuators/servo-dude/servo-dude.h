/* Copyright (C) 2015-2017
 *
 * servo-dude.h
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

#ifndef ACTUATORS_SERVO_DUDE_SERVO_DUDE_H_
#define ACTUATORS_SERVO_DUDE_SERVO_DUDE_H_

//*****************************************************************************/
//
//! \addtogroup actuators
//! @{
//!
//! \addtogroup actuators_servodude Servo-Dude
//! @{
///
//*****************************************************************************/

#include <stdint.h>

#include "BetPUSH.pb.h"

/*! Available servo-dude-ids provided and handled by the servo-dude module. */
typedef enum
{
  E_SERVO_DUDE_ELEVATOR,
  E_SERVO_DUDE_RUDDER,
  E_SERVO_DUDE_FLAP_LEFT,
  E_SERVO_DUDE_FLAP_RIGHT,
  E_SERVO_DUDE_AILERON_LEFT,
  E_SERVO_DUDE_AILERON_RIGHT,

} E_SERVO_DUDE_ID_t;


/*! Typedef of structure defining the context of an servo-dude. */
typedef struct S_SERVO_DUDE_CTX_T s_servo_dude_ctx_t;

/*!
 * \brief Initialize the servo-dude.
 *
 * \param e_servo_dude_id	Defines the servo-dude-id to be intialized.
 *
 * \return Returns pointer to context variable if servo-dude is initialized
 *         successfully. NULL otherwise.
 */
s_servo_dude_ctx_t *servo_dude_init(E_SERVO_DUDE_ID_t e_servo_dude_id);

/*!
 * \brief Handle the servo-dude statemachine and sends commands over i2c.
 *
 * \param servodude_ctx   Pointer to context variable.
 *
 * \return None.
 */
void servo_dude_run(s_servo_dude_ctx_t *const servodude_ctx);

/*!
 * \brief Sets the specified servo-dude (in its context) to a position given by the protobuf actuator-msg.
 *
 * \param servodude_ctx	Pointer to context variable.
 * \param actuator_msg	Pointer to actuator-protostruct.
 *
 * \return None.
 */
void servo_dude_set(s_servo_dude_ctx_t *const servodude_ctx, const Actuator *const actuator_msg);

/*
 * \brief Sets the limits of a specified servo-dude (in its context) to a position given by the protobuf
 * actuatorconf-msg, and set flag that new limit is available.
 *
 *
 * \param servodude_ctx	Pointer to context variable.
 * \param actuator_msg	Pointer to actuatorconf-protostruct.
 *
 * \return None.
 */
// void servo_dude_set_limits(s_servo_dude_ctx_t* const servodude_ctx, const BetPUSH_ActuatorsConf* const actuator_msg);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* ACTUATORS_SERVO_DUDE_SERVO_DUDE_H_ */
