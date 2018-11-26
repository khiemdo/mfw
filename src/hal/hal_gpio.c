/* Copyright (C) 2015-2017
 *
 * hal_gpio.c
 *
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "hal_gpio.h"
#include "hal_pps.h"

#define NUM_ISR_MAX 8

static bool loc_validatePort(E_HAL_GPIO_PORT_t e_port, uint32_t *pui32_port);
static bool loc_validateIntType(E_HAL_GPIO_INT_TYPE_t e_intType, uint32_t *pui32_intType);
static void loc_setCallbackEnableInt(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, pfn_gpio_callback pfn_cb);
static void loc_clearInt(uint32_t ui32_port, uint32_t *pui32_status);

/* Prototypes of GPIO port interrupt handlers that are registered in
 * loc_setCallbackEnableInt(). */
static void GPIOPortA_IntHandler(void);
static void GPIOPortB_IntHandler(void);
static void GPIOPortC_IntHandler(void);
static void GPIOPortD_IntHandler(void);
static void GPIOPortE_IntHandler(void);
static void GPIOPortF_IntHandler(void);
static void GPIOPortG_IntHandler(void);
static void GPIOPortH_IntHandler(void);
static void GPIOPortJ_IntHandler(void);
static void GPIOPortK_IntHandler(void);
static void GPIOPortL_IntHandler(void);
static void GPIOPortM_IntHandler(void);
static void GPIOPortN_IntHandler(void);
static void GPIOPortP_IntHandler(void);
static void GPIOPortQ_IntHandler(void);

/* All callbacks for port A (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portA[NUM_ISR_MAX];

/* All callbacks for port B (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portB[NUM_ISR_MAX];

/* All callbacks for port C (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portC[NUM_ISR_MAX];

/* All callbacks for port D (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portD[NUM_ISR_MAX];

/* All callbacks for port E (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portE[NUM_ISR_MAX];

/* All callbacks for port F (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portF[NUM_ISR_MAX];

/* All callbacks for port G (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portG[NUM_ISR_MAX];

/* All callbacks for port H (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portH[NUM_ISR_MAX];

/* All callbacks for port J (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portJ[NUM_ISR_MAX];

/* All callbacks for port K (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portK[NUM_ISR_MAX];

/* All callbacks for port L (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portL[NUM_ISR_MAX];

/* All callbacks for port M (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portM[NUM_ISR_MAX];

/* All callbacks for port N (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portN[NUM_ISR_MAX];

/* All callbacks for port P (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portP[NUM_ISR_MAX];

/* All callbacks for port Q (pin 0 - pin 7) */
static pfn_gpio_callback gpfn_portQ[NUM_ISR_MAX];

bool loc_validatePort(E_HAL_GPIO_PORT_t e_port, uint32_t *pui32_port)
{
  bool b_return = true;

  switch (e_port)
  {
    case E_HAL_GPIO_PORT_A:
      *pui32_port = GPIO_PORTA_BASE;
      break;

    case E_HAL_GPIO_PORT_B:
      *pui32_port = GPIO_PORTB_BASE;
      break;

    case E_HAL_GPIO_PORT_C:
      *pui32_port = GPIO_PORTC_BASE;
      break;

    case E_HAL_GPIO_PORT_D:
      *pui32_port = GPIO_PORTD_BASE;
      break;

    case E_HAL_GPIO_PORT_E:
      *pui32_port = GPIO_PORTE_BASE;
      break;

    case E_HAL_GPIO_PORT_F:
      *pui32_port = GPIO_PORTF_BASE;
      break;

    case E_HAL_GPIO_PORT_G:
      *pui32_port = GPIO_PORTG_BASE;
      break;

    case E_HAL_GPIO_PORT_H:
      *pui32_port = GPIO_PORTH_BASE;
      break;

    case E_HAL_GPIO_PORT_J:
      *pui32_port = GPIO_PORTJ_BASE;
      break;

    case E_HAL_GPIO_PORT_K:
      *pui32_port = GPIO_PORTK_BASE;
      break;

    case E_HAL_GPIO_PORT_L:
      *pui32_port = GPIO_PORTL_BASE;
      break;

    case E_HAL_GPIO_PORT_M:
      *pui32_port = GPIO_PORTM_BASE;
      break;

    case E_HAL_GPIO_PORT_N:
      *pui32_port = GPIO_PORTN_BASE;
      break;

    case E_HAL_GPIO_PORT_P:
      *pui32_port = GPIO_PORTP_BASE;
      break;

    case E_HAL_GPIO_PORT_Q:
      *pui32_port = GPIO_PORTQ_BASE;
      break;

    default:
      b_return = false;
      break;
  }

  return b_return;
}

bool loc_validateIntType(E_HAL_GPIO_INT_TYPE_t e_intType, uint32_t *pui32_intType)
{
  bool b_return = true;

  switch (e_intType)
  {
    case E_HAL_GPIO_INT_TYPE_BOTH_EDGES:
      *pui32_intType = GPIO_BOTH_EDGES;
      break;
    case E_HAL_GPIO_INT_TYPE_FALLING_EDGE:
      *pui32_intType = GPIO_FALLING_EDGE;
      break;
    case E_HAL_GPIO_INT_TYPE_RISING_EDGE:
      *pui32_intType = GPIO_RISING_EDGE;
      break;
    case E_HAL_GPIO_INT_TYPE_HIGH_LEVEL:
      *pui32_intType = GPIO_HIGH_LEVEL;
      break;
    case E_HAL_GPIO_INT_TYPE_LOW_LEVEL:
      *pui32_intType = GPIO_LOW_LEVEL;
      break;
    case E_HAL_GPIO_INT_TYPE_DISCRETE_INT:
      *pui32_intType = GPIO_DISCRETE_INT;
      break;
    default:
      b_return = false;
      break;
  }

  return b_return;
}

static inline void loc_setCallbackEnableInt(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, pfn_gpio_callback pfn_cb)
{
  uint8_t i = 0;

  /* First convert the bitmap to the array index. */
  for (i = 0; i < NUM_ISR_MAX; ++i)
  {
    if ((1 << i) & ui8_pins)
    {
      break;
    }
  }

  /* Then check which array/port is used. */
  switch (e_port)
  {
    case E_HAL_GPIO_PORT_A:
      gpfn_portA[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTA_BASE, GPIOPortA_IntHandler);
      IntEnable(INT_GPIOA);
      break;
    case E_HAL_GPIO_PORT_B:
      gpfn_portB[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTB_BASE, GPIOPortB_IntHandler);
      IntEnable(INT_GPIOB);
      break;
    case E_HAL_GPIO_PORT_C:
      gpfn_portC[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTC_BASE, GPIOPortC_IntHandler);
      IntEnable(INT_GPIOC);
      break;
    case E_HAL_GPIO_PORT_D:
      gpfn_portD[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTD_BASE, GPIOPortD_IntHandler);
      IntEnable(INT_GPIOD);
      break;
    case E_HAL_GPIO_PORT_E:
      gpfn_portE[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTE_BASE, GPIOPortE_IntHandler);
      IntEnable(INT_GPIOE);
      break;
    case E_HAL_GPIO_PORT_F:
      gpfn_portF[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortF_IntHandler);
      IntEnable(INT_GPIOF);
      break;
    case E_HAL_GPIO_PORT_G:
      gpfn_portG[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTG_BASE, GPIOPortG_IntHandler);
      IntEnable(INT_GPIOG);
      break;
    case E_HAL_GPIO_PORT_H:
      gpfn_portH[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTH_BASE, GPIOPortH_IntHandler);
      IntEnable(INT_GPIOH);
      break;
    case E_HAL_GPIO_PORT_J:
      gpfn_portJ[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTJ_BASE, GPIOPortJ_IntHandler);
      IntEnable(INT_GPIOJ);
      break;
    case E_HAL_GPIO_PORT_K:
      gpfn_portK[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTK_BASE, GPIOPortK_IntHandler);
      IntEnable(INT_GPIOK);
      break;
    case E_HAL_GPIO_PORT_L:
      gpfn_portL[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTL_BASE, GPIOPortL_IntHandler);
      IntEnable(INT_GPIOL);
      break;
    case E_HAL_GPIO_PORT_M:
      gpfn_portM[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTM_BASE, GPIOPortM_IntHandler);
      IntEnable(INT_GPIOM);
	  break;
    case E_HAL_GPIO_PORT_N:
      gpfn_portN[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTN_BASE, GPIOPortN_IntHandler);
      IntEnable(INT_GPION);
	  break;
    case E_HAL_GPIO_PORT_P:
      gpfn_portP[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTP_BASE, GPIOPortP_IntHandler);
      IntEnable(INT_GPIOP0);
	  break;
    case E_HAL_GPIO_PORT_Q:
      gpfn_portQ[i] = pfn_cb;
      GPIOIntRegister(GPIO_PORTQ_BASE, GPIOPortQ_IntHandler);
      IntEnable(INT_GPIOQ0);
	  break;
    default:
      break;
  }

  return;
}

static inline void loc_clearInt(uint32_t ui32_port, uint32_t *pui32_status)
{
  *pui32_status = GPIOIntStatus(ui32_port, true);
  GPIOIntClear(ui32_port, *pui32_status);
}

bool hal_gpio_init(void) { return true; }
bool hal_gpio_setDir(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, E_HAL_GPIO_DIR_t e_dir)
{
  uint32_t ui32_port = 0;
  uint32_t ui32_sysClt = 0;
  bool b_return = true;

  /* Validate inputs first. Pins are valid, because input is only 8 bit wide.*/
  switch (e_port)
  {
    case E_HAL_GPIO_PORT_A:
      ui32_port = GPIO_PORTA_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOA;
      break;

    case E_HAL_GPIO_PORT_B:
      ui32_port = GPIO_PORTB_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOB;
      break;
	  
    case E_HAL_GPIO_PORT_C:
      ui32_port = GPIO_PORTC_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOC;
      break;

    case E_HAL_GPIO_PORT_D:
      ui32_port = GPIO_PORTD_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOD;
      break;

    case E_HAL_GPIO_PORT_E:
      ui32_port = GPIO_PORTE_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOE;
      break;

    case E_HAL_GPIO_PORT_F:
      ui32_port = GPIO_PORTF_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOF;
      break;

    case E_HAL_GPIO_PORT_G:
      ui32_port = GPIO_PORTG_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOG;
      break;

    case E_HAL_GPIO_PORT_H:
      ui32_port = GPIO_PORTH_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOH;
      break;

    case E_HAL_GPIO_PORT_J:
      ui32_port = GPIO_PORTJ_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOJ;
      break;

    case E_HAL_GPIO_PORT_K:
      ui32_port = GPIO_PORTK_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOK;
      break;

    case E_HAL_GPIO_PORT_L:
      ui32_port = GPIO_PORTL_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOL;
      break;

    case E_HAL_GPIO_PORT_M:
      ui32_port = GPIO_PORTM_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOM;
      break;

    case E_HAL_GPIO_PORT_N:
      ui32_port = GPIO_PORTN_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPION;
      break;

    case E_HAL_GPIO_PORT_P:
      ui32_port = GPIO_PORTP_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOP;
      break;

    case E_HAL_GPIO_PORT_Q:
      ui32_port = GPIO_PORTQ_BASE;
      ui32_sysClt = SYSCTL_PERIPH_GPIOQ;
      break;

    default:
      b_return = false;
      break;
  }

  if (e_dir == E_HAL_GPIO_DIR_IN || e_dir == E_HAL_GPIO_DIR_OUT)
  {
    b_return = true;
  }
  else
  {
    b_return = false;
  }

  if (b_return)
  {
    //
    // Enable the GPIO port.
    //
    MAP_SysCtlPeripheralEnable(ui32_sysClt);

    //
    // Enable the GPIO pins.
    //
    if (e_dir == E_HAL_GPIO_DIR_IN)
    {
      MAP_GPIOPinTypeGPIOInput(ui32_port, ui8_pins);
      // Enable weak pullup resistor. Throws error when using ROM_* API.
      GPIOPadConfigSet(ui32_port, ui8_pins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    }
    else if (e_dir == E_HAL_GPIO_DIR_OUT)
    {
      MAP_GPIOPinTypeGPIOOutput(ui32_port, ui8_pins);
    }
  }

  return b_return;
}

bool hal_gpio_write(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, uint8_t ui8_value)
{
  bool b_return = false;
  uint32_t ui32_port = 0;

  if (loc_validatePort(e_port, &ui32_port))
  {
    MAP_GPIOPinWrite(ui32_port, ui8_pins, ui8_value);
    b_return = true;
  }

  return b_return;
}

uint8_t hal_gpio_read(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins)
{
  uint8_t ui8_return = 0;
  uint32_t ui32_port = 0;

  if (loc_validatePort(e_port, &ui32_port))
  {
    ui8_return = MAP_GPIOPinRead(ui32_port, ui8_pins);
  }

  return ui8_return;
}

bool hal_gpio_unlock(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins)
{
  bool b_return = false;
  uint32_t ui32_port = 0;
  uint32_t ui32_sysClt = 0;

  /* Validate inputs first. Check for correct pins for unlocking.*/
  switch (e_port)
  {
      // clang-format off
    /* Are you ABSOLUTELY SURE you want to brick your device?
    case E_HAL_GPIO_PORT_C:
      if (!(ui8_pins & ~(HAL_GPIO_PIN_3 + HAL_GPIO_PIN_2 + HAL_GPIO_PIN_1 + HAL_GPIO_PIN_0)) && ui8_pins) //Only pins 0-3 are unlockable
      {
        ui32_port = GPIO_PORTC_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOC;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;*/  // clang-format on

    case E_HAL_GPIO_PORT_D:
      if (!(ui8_pins & ~HAL_GPIO_PIN_7) && ui8_pins) //Only pin 7 is unlockable
      {
        ui32_port = GPIO_PORTD_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOD;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;

#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)

    case E_HAL_GPIO_PORT_E:
      if (!(ui8_pins & ~HAL_GPIO_PIN_7) && ui8_pins) //Only pin 7 is unlockable
      {
        ui32_port = GPIO_PORTE_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOE;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;
#endif

#if defined(TARGET_IS_TM4C123_RA1) ||                                         \
    defined(TARGET_IS_TM4C123_RA3) ||                                         \
    defined(TARGET_IS_TM4C123_RB1)

    case E_HAL_GPIO_PORT_A:
      if (!(ui8_pins & ~(HAL_GPIO_PIN_5 + HAL_GPIO_PIN_4 + HAL_GPIO_PIN_3 + HAL_GPIO_PIN_2 + HAL_GPIO_PIN_1 + HAL_GPIO_PIN_0)) && ui8_pins) //Only pins 0-5 are unlockable
      {
        ui32_port = GPIO_PORTA_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOA;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;

      case E_HAL_GPIO_PORT_B:
      if (!(ui8_pins & ~(HAL_GPIO_PIN_5 + HAL_GPIO_PIN_4 + HAL_GPIO_PIN_3 + HAL_GPIO_PIN_2)) && ui8_pins) //Only pins 2-5 are unlockable
      {
        ui32_port = GPIO_PORTB_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOB;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;

    case E_HAL_GPIO_PORT_F:
      if (!(ui8_pins & ~HAL_GPIO_PIN_0) && ui8_pins) //Only pin 7 is unlockable
      {
        ui32_port = GPIO_PORTF_BASE;
        ui32_sysClt = SYSCTL_PERIPH_GPIOF;
        b_return = true;
      }
      else
      {
        b_return = false;
      }
      break;
#endif

    default:
      b_return = false;
      break;
  }
  if (b_return)
  {
    MAP_SysCtlPeripheralEnable(ui32_sysClt);
    HWREG(ui32_port + GPIO_O_LOCK) = GPIO_LOCK_KEY;  // Unlock the port
    HWREG(ui32_port + GPIO_O_CR) |= ui8_pins;        // Unlock the Pin(s)
    HWREG(ui32_port + GPIO_O_LOCK) = 0;              // Lock the port
  }

  return b_return;
}

bool hal_gpio_setInt(E_HAL_GPIO_PORT_t e_port, uint8_t ui8_pins, E_HAL_GPIO_INT_TYPE_t e_intType, pfn_gpio_callback pfn_cb)
{
  bool b_return = false;
  uint32_t ui32_port = 0;
  uint32_t ui32_intType = 0;

  if (loc_validatePort(e_port, &ui32_port))
  {
    if (loc_validateIntType(e_intType, &ui32_intType))
    {
      loc_setCallbackEnableInt(e_port, ui8_pins, pfn_cb);

      MAP_GPIOIntTypeSet(ui32_port, ui8_pins, ui32_intType);
      MAP_GPIOIntEnable(ui32_port, (uint32_t)ui8_pins);

      b_return = true;
    }
  }

  return b_return;
}

void GPIOPortA_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTA_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portA[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portA[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portA[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portA[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portA[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portA[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portA[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portA[7]();
  }

  return;
}

void GPIOPortB_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTB_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portB[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portB[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portB[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portB[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portB[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portB[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portB[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portB[7]();
  }

  return;
}

void GPIOPortC_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTC_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portC[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portC[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portC[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portC[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portC[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portC[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portC[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portC[7]();
  }

  return;
}

void GPIOPortD_IntHandler(void)
{
#if defined(TARGET_IS_TM4C123_RA1) ||                                         \
    defined(TARGET_IS_TM4C123_RA3) ||                                         \
    defined(TARGET_IS_TM4C123_RB1)
  hal_pps_isr();
#endif
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTD_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portD[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portD[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portD[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portD[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portD[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portD[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portD[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portD[7]();
  }

  return;
}

void GPIOPortE_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTE_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portE[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portE[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portE[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portE[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portE[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portE[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portE[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portE[7]();
  }

  return;
}

void GPIOPortF_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTF_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portF[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portF[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portF[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portF[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portF[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portF[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portF[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portF[7]();
  }

  return;
}

void GPIOPortG_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTG_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portG[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portG[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portG[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portG[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portG[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portG[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portG[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portG[7]();
  }

  return;
}

void GPIOPortH_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTH_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portH[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portH[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portH[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portH[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portH[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portH[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portH[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portH[7]();
  }

  return;
}

void GPIOPortJ_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTJ_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portJ[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portJ[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portJ[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portJ[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portJ[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portJ[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portJ[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portJ[7]();
  }

  return;
}

void GPIOPortK_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTK_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portK[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portK[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portK[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portK[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portK[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portK[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portK[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portK[7]();
  }

  return;
}

void GPIOPortL_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTL_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portL[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portL[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portL[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portL[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portL[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portL[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portL[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portL[7]();
  }

  return;
}

void GPIOPortM_IntHandler(void)
{
#if defined(TARGET_IS_TM4C129_RA0) ||                                       \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
  hal_pps_isr();
#endif

  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTM_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portM[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portM[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portM[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portM[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portM[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portM[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portM[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portM[7]();
  }

  return;
}

void GPIOPortN_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTN_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portN[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portN[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portN[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portN[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portN[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portN[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portN[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portN[7]();
  }

  return;
}

void GPIOPortP_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTP_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portP[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portP[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portP[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portP[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portP[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portP[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portP[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portP[7]();
  }

  return;
}

void GPIOPortQ_IntHandler(void)
{
  uint32_t ui32_intStatus = 0;

  /* Clear HW interrupt flag */
  loc_clearInt(GPIO_PORTQ_BASE, &ui32_intStatus);

  /* There might multiple interrupts at the same time, therefore check all.
*For the sake of performance don't do it in a loop. */
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_0)
  {
    gpfn_portQ[0]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_1)
  {
    gpfn_portQ[1]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_2)
  {
    gpfn_portQ[2]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_3)
  {
    gpfn_portQ[3]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_4)
  {
    gpfn_portQ[4]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_5)
  {
    gpfn_portQ[5]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_6)
  {
    gpfn_portQ[6]();
  }
  if ((uint8_t)ui32_intStatus & HAL_GPIO_PIN_7)
  {
    gpfn_portQ[7]();
  }

  return;
}
