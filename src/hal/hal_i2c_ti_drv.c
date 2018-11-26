/* Copyright (C) 2015-2017
 *
 * hal_i2c_ti_drv.c
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
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "hal_mcu.h"

#include "hal_i2c_ti_drv.h"


/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C0(uint32_t ui32_sysClock);
static s_hal_i2c_ctx_t *loc_initI2C1(uint32_t ui32_sysClock);
static s_hal_i2c_ctx_t *loc_initI2C2(uint32_t ui32_sysClock);
static s_hal_i2c_ctx_t *loc_initI2C3(uint32_t ui32_sysClock);
static s_hal_i2c_ctx_t *loc_initI2C4(uint32_t ui32_sysClock);
static s_hal_i2c_ctx_t *loc_initI2C5(uint32_t ui32_sysClock);
void IntI2C0Handler(void);
void IntI2C1Handler(void);
void IntI2C2Handler(void);
void IntI2C3Handler(void);
void IntI2C4Handler(void);
void IntI2C6Handler(void);

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
tI2CMInstance gs_i2c0_inst;
tI2CMInstance gs_i2c1_inst;
tI2CMInstance gs_i2c2_inst;
tI2CMInstance gs_i2c3_inst;
tI2CMInstance gs_i2c4_inst;
tI2CMInstance gs_i2c5_inst;

s_hal_i2c_ctx_t gs_i2c0_ctx;
s_hal_i2c_ctx_t gs_i2c1_ctx;
s_hal_i2c_ctx_t gs_i2c2_ctx;
s_hal_i2c_ctx_t gs_i2c3_ctx;
s_hal_i2c_ctx_t gs_i2c4_ctx;
s_hal_i2c_ctx_t gs_i2c5_ctx;

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

void loc_I2C0_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c0_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c0_ctx.state = E_HAL_I2C_STATE_IDLE;
}

void loc_I2C1_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c1_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c1_ctx.state = E_HAL_I2C_STATE_IDLE;
}

void loc_I2C2_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c2_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c2_ctx.state = E_HAL_I2C_STATE_IDLE;
}

void loc_I2C3_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c3_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c3_ctx.state = E_HAL_I2C_STATE_IDLE;
}

void loc_I2C4_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c4_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c4_ctx.state = E_HAL_I2C_STATE_IDLE;
}

void loc_I2C5_callback(void *pvData, uint_fast8_t ui8Status)
{
    gs_i2c5_ctx.callback_pointer(pvData, ui8Status);
	gs_i2c5_ctx.state = E_HAL_I2C_STATE_IDLE;
}

/* ===========================================================================*/
/*                  i2c-interrupt handlers                                    */
/* ===========================================================================*/
void IntI2C0Handler(void)
{
	I2CMIntHandler(&gs_i2c0_inst);
}

void IntI2C1Handler(void)
{
	I2CMIntHandler(&gs_i2c1_inst);
}

void IntI2C2Handler(void)
{
	I2CMIntHandler(&gs_i2c2_inst);
}

void IntI2C3Handler(void)
{
	I2CMIntHandler(&gs_i2c3_inst);
}

void IntI2C4Handler(void)
{
	I2CMIntHandler(&gs_i2c4_inst);
}

void IntI2C6Handler(void)
{
	I2CMIntHandler(&gs_i2c5_inst);
}
/* ===========================================================================*/
/*                         loc_initI2C0()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C0(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
  //
  // Set GPIO B2 and B3 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C0_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c0_ctx, 0, sizeof(gs_i2c0_ctx) );
  gs_i2c0_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c0_inst, 0, sizeof(gs_i2c0_inst) );
  I2CMInit(&gs_i2c0_inst, I2C0_BASE, INT_I2C0, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c0_ctx.gs_i2c_inst = & gs_i2c0_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C0_BASE, IntI2C0Handler);

  return &gs_i2c0_ctx;
}

/* ===========================================================================*/
/*                         loc_initI2C1()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C1(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

  //
  // Set GPIO G0 and G1 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PG0_I2C1SCL);
  GPIOPinConfigure(GPIO_PG1_I2C1SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);
  GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C1_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c1_ctx, 0, sizeof(gs_i2c1_ctx) );
  gs_i2c1_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c1_inst, 0, sizeof(gs_i2c1_inst) );
  I2CMInit(&gs_i2c1_inst, I2C1_BASE, INT_I2C1, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c1_ctx.gs_i2c_inst = & gs_i2c1_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C1_BASE, IntI2C1Handler);

  return &gs_i2c1_ctx;
}

/* ===========================================================================*/
/*                         loc_initI2C2()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C2(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

  //
  // Set GPIO N4 and N5 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PN4_I2C2SDA);
  GPIOPinConfigure(GPIO_PN5_I2C2SCL);
  GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
  GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C2_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c2_ctx, 0, sizeof(gs_i2c2_ctx) );
  gs_i2c2_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c2_inst, 0, sizeof(gs_i2c2_inst) );
  I2CMInit(&gs_i2c2_inst, I2C2_BASE, INT_I2C2, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c2_ctx.gs_i2c_inst = & gs_i2c2_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C2_BASE, IntI2C2Handler);

  return &gs_i2c2_ctx;
}

/* ===========================================================================*/
/*                         loc_initI2C3()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C3(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

  //
  // Set GPIO K4 and K5 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PK4_I2C3SCL);
  GPIOPinConfigure(GPIO_PK5_I2C3SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_4);
  GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_5);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C3_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c3_ctx, 0, sizeof(gs_i2c3_ctx) );
  gs_i2c3_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c3_inst, 0, sizeof(gs_i2c3_inst) );
  I2CMInit(&gs_i2c3_inst, I2C3_BASE, INT_I2C3, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c3_ctx.gs_i2c_inst = & gs_i2c3_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C3_BASE, IntI2C3Handler);

  return &gs_i2c3_ctx;
}

/* ===========================================================================*/
/*                         loc_initI2C4()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C4(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

  //
  // Set GPIO K6 and K7 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PK6_I2C4SCL);
  GPIOPinConfigure(GPIO_PK7_I2C4SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_6);
  GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_7);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C4_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c4_ctx, 0, sizeof(gs_i2c4_ctx) );
  gs_i2c4_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c4_inst, 0, sizeof(gs_i2c4_inst) );
  I2CMInit(&gs_i2c4_inst, I2C4_BASE, INT_I2C4, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c4_ctx.gs_i2c_inst = & gs_i2c4_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C4_BASE, IntI2C4Handler);

  return &gs_i2c4_ctx;
}

/* ===========================================================================*/
/*                         loc_initI2C5()                                     */
/* ===========================================================================*/
static s_hal_i2c_ctx_t *loc_initI2C5(uint32_t ui32_sysClock)
{

  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Set GPIO A6 and A7 as I2C pins.
  //
  GPIOPinConfigure(GPIO_PA6_I2C6SCL);
  GPIOPinConfigure(GPIO_PA7_I2C6SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
  GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

  //
  // init i2c clock
  //
  I2CMasterInitExpClk(I2C6_BASE, ui32_sysClock, true);

  /* Initialize our context struct. */
  memset( &gs_i2c5_ctx, 0, sizeof(gs_i2c5_ctx) );
  gs_i2c5_ctx.state = E_HAL_I2C_STATE_IDLE;
  memset( &gs_i2c5_inst, 0, sizeof(gs_i2c5_inst) );
  I2CMInit(&gs_i2c5_inst, I2C6_BASE, INT_I2C6, 0xFF, 0xFF,ui32_sysClock);
  gs_i2c5_ctx.gs_i2c_inst = & gs_i2c5_inst;

  /* Finally, register our interrupt handler implementation to the MCU here.
   * When interrupt occurs, this function gets called and forwards the call
   * to I2CMIntHandler() that is provided by i2cm_drv.h */
  I2CIntRegister(I2C6_BASE, IntI2C6Handler);

  return &gs_i2c5_ctx;
}

/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*         hal_i2c_init() Initializes a I2C Port and returns context          */
/* ===========================================================================*/
s_hal_i2c_ctx_t *hal_i2c_ti_drv_init(E_HAL_I2C_PORT_t e_port)
{
	s_hal_i2c_ctx_t *ps_return = NULL;

  /* Get the System clock (settings). */
  uint32_t ui32SysClock = hal_mcu_getSysClock();

  switch (e_port)
  {
    case E_HAL_I2C_PORT_0:
      ps_return = loc_initI2C0(ui32SysClock);
      break;

    case E_HAL_I2C_PORT_1:
      ps_return = loc_initI2C1(ui32SysClock);
      break;

    case E_HAL_I2C_PORT_2:
      ps_return = loc_initI2C2(ui32SysClock);
      break;
	  
    case E_HAL_I2C_PORT_3:
      ps_return = loc_initI2C3(ui32SysClock);
      break;
	  
    case E_HAL_I2C_PORT_4:
      ps_return = loc_initI2C4(ui32SysClock);
      break;
	  
    case E_HAL_I2C_PORT_5:
      ps_return = loc_initI2C5(ui32SysClock);
      break;

    /* Other I2C ports are not supported => ps_return remains NULL. */
    default:
      break;
  }

  return ps_return;
}

/* ===========================================================================*/
/*          hal_i2c_write() - Writes to I2C Line                              */
/* ===========================================================================*/
void hal_i2c_write(s_hal_i2c_ctx_t *ps_ctx, uint8_t slave_address, uint8_t *data, uint8_t length, void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status))
{
	if (ps_ctx->state == E_HAL_I2C_STATE_IDLE){
		ps_ctx->state = E_HAL_I2C_STATE_PENDING;
		ps_ctx->callback_pointer = callback_pointer;
		switch (ps_ctx->gs_i2c_inst->ui32Base) {
		    case I2C0_BASE:
		        I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C0_callback, NULL);
		        break;
            case I2C1_BASE:
                I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C1_callback, NULL);
                break;
            case I2C2_BASE:
                I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C2_callback, NULL);
                break;
            case I2C3_BASE:
                I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C3_callback, NULL);
                break;
            case I2C4_BASE:
                I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C4_callback, NULL);
                break;
            case I2C6_BASE:
                I2CMWrite(ps_ctx->gs_i2c_inst, slave_address, data, length, &loc_I2C5_callback, NULL);
                break;
		}

	}

}
/* ===========================================================================*/
/*          hal_i2c_read() - Reads from the I2C Line                          */
/* ===========================================================================*/
void hal_i2c_read(s_hal_i2c_ctx_t *ps_ctx, uint8_t slave_address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength, void (*callback_pointer)(void *pvData, uint_fast8_t ui8Status))
{
	if (ps_ctx->state == E_HAL_I2C_STATE_IDLE){
		ps_ctx->state = E_HAL_I2C_STATE_PENDING;
        ps_ctx->callback_pointer = callback_pointer;
        switch (ps_ctx->gs_i2c_inst->ui32Base) {
            case I2C0_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
	             readData, readLength, loc_I2C0_callback, NULL);
                break;
            case I2C1_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
                 readData, readLength, loc_I2C1_callback, NULL);
                break;
            case I2C2_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
                 readData, readLength, loc_I2C2_callback, NULL);
                break;
            case I2C3_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
                 readData, readLength, loc_I2C3_callback, NULL);
                break;
            case I2C4_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
                 readData, readLength, loc_I2C4_callback, NULL);
                break;
            case I2C6_BASE:
                I2CMRead(ps_ctx->gs_i2c_inst, slave_address, writeData, writeLength,
                 readData, readLength, loc_I2C5_callback, NULL);
                break;
        }
	}
}
