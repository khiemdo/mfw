/* Copyright (C) 2015-2017
 *
 * ncp5623.h
 *
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

#ifndef ACTUATORS_NCP5623_NCP5623_H_
#define ACTUATORS_NCP5623_NCP5623_H_

//*****************************************************************************/
//
//! \addtogroup actuators Actuators
//! @{
//!
//! \addtogroup actuators_NCP5623 NCP5623
//! @{
//
//*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*! @brief Struct containing all necessary data for the LED controller. */
typedef struct tags_NCP5623_DATA_t
{
  uint8_t ILED; /*!< Current step set, ranging from 0-31. */
  uint8_t time; /*!< Time for dimming in multiples of 8 ms, ranging from 0-31. */
  uint8_t R;    /*!< Red PWM value. Ranging from 0-255, but is mapped internally to 0-31. */
  uint8_t G;    /*!< Green PWM value. Ranging from 0-255, but is mapped internally to 0-31. */
  uint8_t B;    /*!< Blue PWM value. Ranging from 0-255, but is mapped internally to 0-31. */

} s_NCP5623_DATA_t;


/*!
 * \brief Initializes the LED controller.
 *
 * \details This function initializes all variables, gets an I2C context and
 *          issues a shutdown.
 *
 * \return TRUE if initialization successful. FALSE otherwise.
 */
bool NCP5623_init();

/*!
 * \brief Run function for measurement framework.
 *
 * \details This function is repeatedly called by the measurement framework.
 *          It realizes a state machine for the LED controller. If the LED
 *          controller is idle, it calls the set function. If a transmission
 *          is pending, it does nothing. If the controller is in the progress of
 *          dimming up or down, it checks if the controller has finished.
 *          If not, it also exits, otherwise state is changed to idle.
 *
 * \return None.
 */
void NCP5623_run();

/*!
 * \brief Shuts the LED controller down.
 *
 * \details This function shuts down all output and essentially deactivates the
 *          LEDs. It is not a reset. All values stored in the LED controllers
 *          registers are still present. As soon as a value is written
 *          into one of the registers, all outputs are activated again.
 *
 * \return None.
 */
void NCP5623_shutdown();

/*!
 * \brief Writes the new values stored in memory to the LED controller.
 *
 * \details This function implements the logic for writing new values to the
 *          LED controller. It checks if new values are present and if so,
 *          writes them to the LED controller. It makes sure, that all changed
 *          values are written in a row, before the next new data is written.
 *          The function also start a timer for the dimming time, if dimming is
 *          wanted and determines the dimming direction and writes the
 *          values accordingly.
 *
 * \return None.
 */
void NCP5623_set();

/*!
 * \brief Generates a rainbow.
 *
 * \details This function continuously changes the data written to the LED
 *          controller in a way, that all possible colours are generated. It
 *          automatically ramps up and down the PWM values with a given
 *          speed per step.
 *
 * \param speed  Delay in ms between two consecutive steps of the rainbow
 *               function.
 *
 * \return None.
 */
void NCP5623_rainbow(uint16_t speed);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* ACTUATORS_NCP5623_NCP5623_H_ */
