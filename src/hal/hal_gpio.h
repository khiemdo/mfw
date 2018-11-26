/* Copyright (C) 2015-2017
 *
 * hal_gpio.h
 *
 * Martin Dold         <martin.dold@gmx.net>
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

#ifndef HAL_HAL_GPIO_H_
#define HAL_HAL_GPIO_H_

//*****************************************************************************
//
//! \example test-app-spi.c
//! \example test-app-uart.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_gpio_api HAL GPIO API
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 0.*/
#define HAL_GPIO_PIN_0 0x01
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 1.*/
#define HAL_GPIO_PIN_1 0x02
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 2.*/
#define HAL_GPIO_PIN_2 0x04
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 3.*/
#define HAL_GPIO_PIN_3 0x08
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 4.*/
#define HAL_GPIO_PIN_4 0x10
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 5.*/
#define HAL_GPIO_PIN_5 0x20
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 6.*/
#define HAL_GPIO_PIN_6 0x40
/*! Bitmap reflecting a pin of a GPIO port for this GPIO module: \b Pin \b 7.*/
#define HAL_GPIO_PIN_7 0x80

/*! Available GPIO ports supported and handled by the GPIO module. */
typedef enum {
  
  /*! GPIO port A. */
  E_HAL_GPIO_PORT_A,
  
  /*! GPIO port B. */
  E_HAL_GPIO_PORT_B,
  
  /*! GPIO port C. */
  E_HAL_GPIO_PORT_C,

  /*! GPIO port D. */
  E_HAL_GPIO_PORT_D,

  /*! GPIO port E. */
  E_HAL_GPIO_PORT_E,

  /*! GPIO port F. */
  E_HAL_GPIO_PORT_F,
  
  /*! GPIO port G. */
  E_HAL_GPIO_PORT_G,

  /*! GPIO port H. */
  E_HAL_GPIO_PORT_H,
  
  /*! GPIO port J. */
  E_HAL_GPIO_PORT_J,

  /*! GPIO port K. */
  E_HAL_GPIO_PORT_K,
  
  /*! GPIO port L. */
  E_HAL_GPIO_PORT_L,

  /*! GPIO port M. */
  E_HAL_GPIO_PORT_M,
  
  /*! GPIO port N. */
  E_HAL_GPIO_PORT_N,
  
  /*! GPIO port P. */
  E_HAL_GPIO_PORT_P,
  
  /*! GPIO port Q. */
  E_HAL_GPIO_PORT_Q,  

} E_HAL_GPIO_PORT_t;

/*! Available GPIO ports directions supported and handled by the GPIO module. */
typedef enum {

  /*! GPIO pin direction: input. */
  E_HAL_GPIO_DIR_IN,

  /*! GPIO pin direction: output. */
  E_HAL_GPIO_DIR_OUT

} E_HAL_GPIO_DIR_t;

/*! Available GPIO interrput types supported and handled by the GPIO module. */
typedef enum {

  /*! Trigger to wait for is: falling edge. */
  E_HAL_GPIO_INT_TYPE_FALLING_EDGE,

  /*! Trigger to wait for is: rising edge. */
  E_HAL_GPIO_INT_TYPE_RISING_EDGE,

  /*! Trigger to wait for is: rising \b and falling edge. */
  E_HAL_GPIO_INT_TYPE_BOTH_EDGES,

  /*! Trigger to wait for is: low level. */
  E_HAL_GPIO_INT_TYPE_LOW_LEVEL,

  /*! Trigger to wait for is: high level. */
  E_HAL_GPIO_INT_TYPE_HIGH_LEVEL,

  /*! Trigger to wait for is: special type of discrete interrupt for dedicated
     pins only. */
  E_HAL_GPIO_INT_TYPE_DISCRETE_INT

} E_HAL_GPIO_INT_TYPE_t;

/*! Typedef of function pointer used as callback by \ref hal_gpio_setInt. */
typedef void (*pfn_gpio_callback)(void);

/*!
 * \brief Initialize the GPIO driver module.
 *
 * \details This function shall be called prior to any GPIO configuration.
 *
 * \return TRUE if initialized successfully. FALSE otherwise.
 */
bool hal_gpio_init(void);

/*!
 * \brief Defines the direction of the given port pin, i.e. defines wether pin
 *        is used as input \ref E_HAL_GPIO_DIR_IN or output \ref
 * E_HAL_GPIO_DIR_OUT.
 *
 * \attention Please note that this function \b does \b not verify if the user
 *            given pin is used already in a different manner, e.g. the pin
 *            might be used as UART TX pin. It is up to the caller to make sure
 *            that the pin usage as GPIO does not conflict with other HAL module
 *            configuration.
 *            A wrong and/or duplicated pin configuration (e.g. as UART TX pin
 *            and GPIO input pin) may lead to unexpected behaviour of the system
 *            and must be detected by the caller.
 *
 * \param e_port    Port that shall be configured.
 * \param ui8_pins  Pin that shall be configured.
 * \param e_dir     Direction the given pin shall be configured for.
 *
 * \return TRUE if configured successfully. FALSE otherwise.
 */
bool hal_gpio_setDir(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, E_HAL_GPIO_DIR_t e_dir);

/*!
 * \brief Assigns the user given value to the pin(s).
 *
 * \details As the parameters ui8_pins and ui8_value are used as bitmap
 * internally,
 *          it is possible to pass multiple pins and multiple pin assignments in
 *          a single call. However, all pins must be of the same port.
 *
 * \attention Please note that this function \b does \b not verify if the user
 *            given pin is used already in a different manner, e.g. the pin
 *            might be used as UART TX pin. It is up to the caller to make sure
 *            that the pin usage as GPIO does not conflict with other HAL module
 *            configuration.
 *            A wrong and/or duplicated pin configuration (e.g. as UART TX pin
 *            and GPIO input pin) may lead to unexpected behaviour of the system
 *            and must be detected by the caller.
 *
 * \param e_port     Single port that shall be configured.
 * \param ui8_pins   Pin(s) that shall be configured.
 * \param ui8_value  Value to be applied to the pin(s).
 *
 * \return TRUE if assigned successfully. FALSE otherwise.
 */
bool hal_gpio_write(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, uint8_t ui8_value);

/*!
 * \brief Reads and returns the current state of user given pin(s).
 *
 * \details As the parameter ui8_pins is used as bitmap internally,
 *          it is possible to pass multiple pins in a single call.
 *          However, all pins must be of the same port.
 *
 * \attention Please note that this function \b does \b not verify if the user
 *            given pin is used already in a different manner, e.g. the pin
 *            might be used as UART TX pin. It is up to the caller to make sure
 *            that the pin usage as GPIO does not conflict with other HAL module
 *            configuration.
 *            A wrong and/or duplicated pin configuration (e.g. as UART TX pin
 *            and GPIO input pin) may lead to unexpected behaviour of the system
 *            and must be detected by the caller.
 *
 * \param e_port     Single port that shall be configured.
 * \param ui8_pins   Pin(s) that shall be configured.
 *
 * \return 8 bit bitmap that reflects the current states ('0' or '1') of the
 *         requested port.
 */
uint8_t hal_gpio_read(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins);

/*!
 * \brief Unlocks special function port pin(s).
 *
 * \details Some port pins are used as special function port pins, e. g. JTAG
 * and
 *          NMI. These pins are protected from accidential writes by a lock.
 * This
 *          function selectivly unlocks those port pins. It has no function on
 *          unprotected pins and ports.
 *
 * \attention This function is DANGEROUS, so only use with good cause. If you
 * overwrite
 *            the standard pin function of the JTAG pins, you WILL LOOSE all
 *            debug access to your device, and essentially BRICK your
 * microcontroller.
 *
 * \param e_port     Single port that shall be configured.
 * \param ui8_pins   Pin(s) that shall be configured.
 *
 * \return TRUE if assigned successfully. FALSE otherwise.
 */
bool hal_gpio_unlock(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins);

/*!
 *
 * \brief Configures user given pin as interrupt pin.
 *
 * \details The port pin that is passed to this function is configured as
 *          interrupt pin and reacts on the user given event in
 *          \ref E_HAL_GPIO_INT_TYPE_t. If the corresponding event occurs, the
 *          callback pointer \ref pfn_gpio_callback will be called.
 *
 * \code
 * // === Example usage ===
 *
 * // Callback function defined by the caller.
 * void loc_cb(void)
 * {
 *   // Do something usefull here when interrupt occurs.
 * }
 *
 * // Configure the port and pin first.
 * hal_gpio_setDir(E_HAL_GPIO_PORT_H, HAL_GPIO_PIN_1, E_HAL_GPIO_DIR_IN);
 *
 * // Configure the type of interrupt and provide the corresponding callback
 * pointer.
 * hal_gpio_setInt(E_HAL_GPIO_PORT_H, HAL_GPIO_PIN_1,
 * E_HAL_GPIO_INT_TYPE_FALLING_EDGE, loc_cb);
 * \endcode
 *
 * \attention Please note that this function \b does \b not verify if the user
 *            given pin is used already in a different manner, e.g. the pin
 *            might be used as UART TX pin. It is up to the caller to make sure
 *            that the pin usage as GPIO does not conflict with other HAL module
 *            configuration.
 *            A wrong and/or duplicated pin configuration (e.g. as UART TX pin
 *            and GPIO input pin) may lead to unexpected behaviour of the system
 *            and must be detected by the caller.
 *
 * \param e_port     Single port that shall be configured.
 * \param ui8_pins   Pin(s) that shall be configured.
 * \param e_intType  Interrupts type that shall be configured.
 * \param pfn_cb     Function pointer that is called when specified interrupt
 *                   event occurs.
 *
 * \return TRUE if configured successfully. FALSE otherwise.
 */
bool hal_gpio_setInt(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, E_HAL_GPIO_INT_TYPE_t e_intType, pfn_gpio_callback pfn_cb);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_GPIO_H_ */
