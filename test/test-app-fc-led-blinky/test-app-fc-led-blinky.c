/* Copyright (C) 2015-2017
 *
 * test-app-fc-led-blinky.c
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbj√∂rn J√∂rger    <thorbjoern.joerger@web.de>
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
#include <stdint.h>
#include <stdbool.h>
#include "hal.h"
#include "hal_mcu.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "power/servopower.h"
#include "sht25/sht25.h"
#include "i2c-bus-manager.h"

#define TEST_APP_TIMEOUT_LED_TOGGLE_MS     100
#define TEST_APP_TIMEOUT_POWER_TOGGLE_MS   10000

#if defined(TARGET_IS_TM4C123_RA1) ||                                         \
    defined(TARGET_IS_TM4C123_RA3) ||                                         \
    defined(TARGET_IS_TM4C123_RB1)
#define FC_GPS 1
#define LED0_PORT E_HAL_GPIO_PORT_F
#define LED0_PIN  HAL_GPIO_PIN_0
#else
#define FC 1
#define LED0_PORT E_HAL_GPIO_PORT_D
#define LED0_PIN HAL_GPIO_PIN_7
#define LED1_PORT E_HAL_GPIO_PORT_L
#define LED1_PIN  HAL_GPIO_PIN_0
#define LED2_PORT E_HAL_GPIO_PORT_L
#define LED2_PIN  HAL_GPIO_PIN_4
#define LED3_PORT E_HAL_GPIO_PORT_L
#define LED3_PIN  HAL_GPIO_PIN_5
#endif

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static void loc_ledsOn(void);
static void loc_ledsOff(void);
static bool loc_hal_init(void);

/* ===========================================================================*/
/*                  loc_ledsOn()                                              */
/* ===========================================================================*/
static void loc_ledsOn(void)
{
  /* µCLED (Orange): PD7/PF0 */
  hal_gpio_write(LED0_PORT, LED0_PIN, LED0_PIN);

  #if FC
  /* LED1 (Yellow-Green): PL0 */
  hal_gpio_write(LED1_PORT, LED1_PIN, LED1_PIN);

  /* LED2 (Yellow-Green): PL4 */
  hal_gpio_write(LED2_PORT, LED2_PIN, LED2_PIN);

  /* LED3 (Yellow-Green): PL5 */
  hal_gpio_write(LED3_PORT, LED3_PIN, LED3_PIN);
  #endif

  return;
}

/* ===========================================================================*/
/*                  loc_ledsOff()                                             */
/* ===========================================================================*/
static void loc_ledsOff(void)
{
  /* µCLED (Orange): PD7/PF0 */
  hal_gpio_write(LED0_PORT, LED0_PIN, (uint8_t) ~LED0_PIN);

  #if FC
  /* LED1 (Yellow-Green): PL0 */
  hal_gpio_write(LED1_PORT, LED1_PIN, (uint8_t) ~LED1_PIN);

  /* LED2 (Yellow-Green): PL4 */
  hal_gpio_write(LED2_PORT, LED2_PIN, (uint8_t) ~LED2_PIN);

  /* LED3 (Yellow-Green): PL5 */
  hal_gpio_write(LED3_PORT, LED3_PIN, (uint8_t) ~LED3_PIN);
  #endif

  return;
}

/* ===========================================================================*/
/*                  loc_hal_init()                                            */
/* ===========================================================================*/
static bool loc_init(void)
{
  bool b_return = false;

  b_return = hal_mcu_init();

  if(b_return)
  {
    b_return = hal_timer_init();
  }

  if(b_return)
  {
    b_return = hal_gpio_init();
  }

  if(b_return)
  {
    b_return = hal_pps_init();
  }

  /* 3rd challenge:
   * Initialize the pins that have LEDs mounted on the flight controller
   * hardware (PCB). Unlock µCLED.*/
  #if FC
  if(b_return)
  {
    /* LED1 (Yellow-Green): PL0 */
    b_return = hal_gpio_setDir(LED1_PORT, LED1_PIN, E_HAL_GPIO_DIR_OUT);
  }

  if(b_return)
  {
   /* LED2 (Yellow-Green): PL4 */
    b_return = hal_gpio_setDir(LED2_PORT, LED2_PIN, E_HAL_GPIO_DIR_OUT);
  }

  if(b_return)
  {
    /* LED3 (Yellow-Green): PL5 */
    b_return = hal_gpio_setDir(LED3_PORT, LED3_PIN, E_HAL_GPIO_DIR_OUT);
  }
  #endif

  if(b_return)
  {
    /* µCLED (Orange): PD7/PF0 */
    b_return = hal_gpio_unlock(LED0_PORT, LED0_PIN);
  }

  if(b_return)
  {
    /* µCLED (Orange): PD7/PF0 */
    b_return = hal_gpio_setDir(LED0_PORT, LED0_PIN, E_HAL_GPIO_DIR_OUT);
  }

  /* 4th challenge:
   * Initialize the pins that controls the servo power supply.*/
  #if FC
  if(b_return)
  {
    /* Power-OFF: PN3 */
    b_return = servopower_init();
  }
  #endif



  return b_return;
}/* loc_hal_init() */


/* ===========================================================================*/
/*                  main()                                                    */
/* =========================================================,==================*/
int main(void)
{
  uint32_t timer_leds = 0;
  uint32_t timer_power = 0;

  bool b_return = loc_init();

  if(b_return)
  {
    /* Request some timeout in milliseconds. */
    timer_leds = hal_timer_getTimeout( TEST_APP_TIMEOUT_LED_TOGGLE_MS );
    #if FC
    timer_power = hal_timer_getTimeout( TEST_APP_TIMEOUT_POWER_TOGGLE_MS );
    #endif

    while(b_return)
    {

      if( hal_timer_isTimedOut(timer_leds) )
      {
        /* Check LED0 to determine if all LEDs are on or off. */
        if( hal_gpio_read(LED0_PORT, LED0_PIN) )
        {
          /* LED is ON. Turn all LEDs off! */
          loc_ledsOff();
        }
        else
        {
          /* LED is OFF. Turn all LEDs on! */
          loc_ledsOn();
        }

        /* Request new timeout in milliseconds. */
        timer_leds = hal_timer_getTimeout( TEST_APP_TIMEOUT_LED_TOGGLE_MS );
      }
      #if FC
      if (hal_timer_isTimedOut(timer_power))
      {
        /* Servo power is on, switch off! */
        if(servopower_status())
        {
          servopower_off();
        }
        /* Servo power is off, switch on! */
        else
        {
          servopower_on();
        }

        /* Request new timeout in miliseconds. */
        timer_power = hal_timer_getTimeout( TEST_APP_TIMEOUT_POWER_TOGGLE_MS );
      }
      #endif
    }
  }
  return 0;
}
