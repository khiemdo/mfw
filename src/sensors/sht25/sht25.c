/* Copyright (C) 2015-2017
 *
 * sht25.c
 *
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Elias Rosch         <eliasrosch@gmail.com>
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

#include "sht25.h"

#include "hal_timer.h"
#include "hal_pps.h"

#include "i2c-bus-manager.h"

/*! Adress of sensor. This value is hard-coded into the hardware and cannot be changed.
 *  Binary representation from the datasheet: \b 0b1000000 */
#define SHT25_ADDR 0x40

#define SHT25_I2C_PORT E_HAL_I2C_PORT_5
/*! Bitmap for write acess. Add this to the adress when writing to the sensor. Binary
 *  representation from the datasheet: \b 0b00000000 */
#define SHT25_WRITE 0x00
/*! Bitmap for read access. Add this to the adress when reading from the sensor. Binary
 *  representation from the datasheet: \b 0b00000001 */
#define SHT25_READ 0x01
/*! Command for reading from the user register of the sensor. Binary representation
 *  from the datasheet: \b 0b11100111 */
#define SHT25_CMD_READ_REG 0xE7
/*! Command for writing to the user register of the sensor. Binary representation from
 *  the datasheet: \b 0b11100110 */
#define SHT25_CMD_WRITE_REG 0xE6
/*! Command for soft reset of the sensor. Binary representation from the datasheet:
 *  \b 0b11111110 */
#define SHT25_CMD_SOFT_RESET 0xFE
/*! Command for triggering a temperature measurement of the sensor in hold mode. Sensor
 *  pulls down SCL during the measuring time (66-85 ms)to force the master into wait
 *  state. Binary representation from the datasheet: \b 0b11100011 */
#define SHT25_CMD_TRIG_T_MEASUREMENT 0xE3
/*! Command for triggering a relative humidity measurement of the sensor in hold mode.
 *  Sensor pulls down SCL during the measuring time (22-29 ms)to force the master into
 *  wait state. Binary representation from the datasheet: \b 0b11100101 */
#define SHT25_CMD_TRIG_RH_MEASUREMENT 0xE5
/*! Bitmap for no hold measurement. This is added to the corresponding commands for
 *  hold measurements. Binary representation from the datasheet: \b 0b00010000 */
#define SHT25_CMD_NOHOLD 0x10
/*! Command for triggering a temperature measurement of the sensor in no hold mode.
 *  Repeated reads during the measuring time (66-85 ms) are disacknowledged. Binary
 *  representation from the datasheet: \b 0b11110011 */
#define SHT25_CMD_TRIG_T_MEASUREMENT_NOHOLD (SHT25_CMD_TRIG_T_MEASUREMENT + SHT25_CMD_NOHOLD)
/*! Command for triggering a relative humidity measurement of the sensor in no hold mode.
 *  Repeated reads during the measuring time (22-29 ms ms) are disacknowledged. Binary
 *  representation from the datasheet: \b 0b11110101 */
#define SHT25_CMD_TRIG_RH_MEASUREMENT_NOHOLD (SHT25_CMD_TRIG_RH_MEASUREMENT + SHT25_CMD_NOHOLD)
/*! Bitmap for masking measurment data and extracting status bits. Binary representation from
 *  the datasheet: \b 0b00000011 */
#define SHT25_STATUS_MASK 0x3
/*! Bitmap for status bit that is representing a temperature measurement. Binary represenation
 * from the datasheet: \b 0b00000000 */
#define SHT25_STATUS_MEASUREMENT_TYPE_T 0x0
/*! Bitmap for status bit that is representing a relative humidity measurement. Binary
 * representation from the datasheet: \b 0b00000010 */
#define SHT25_STATUS_MEASUREMENT_TYPE_RH 0x2
/*! Bitmap for measurement resolution RH (12 bit) and T (14 bit). Binary representation from
 *  the datasheet: \b 0b00000000 */
#define SHT25_RESOLUTION_12_14 0x0
/*! Bitmap for measurement resolution RH (8 bit) and T (12 bit). Binary representation from
 *  the datasheet: \b 0b01000000 */
#define SHT25_RESOLUTION_8_12 0x01
/*! Bitmap for measurement resolution RH (10 bit) and T (13 bit). Binary representation from
 *  the datasheet: \b 0b10000000 */
#define SHT25_RESOLUTION_10_13 0x80
/*! Bitmap for measurement resolution RH (11 bit) and T (11 bit). Binary representation from
 *  the datasheet: \b 0b11000000 */
#define SHT25_RESOLUTION_11_11 0x81
/*! Bitmap for enabling the on-chip heater.  Binary representation from
 *  the datasheet: \b 0b00000010 */
#define SHT25_ENABLE_HEATER 0x4
/*! Bitmap for disabling OTP reload.  Binary representation from
 *  the datasheet: \b 0b00000001 */
#define SHT25_DISABLE_OTP_RELOAD 0x2
/*! Bitmap for masking write access to user register.  Binary representation from
 *  the datasheet: \b 0b11000000 */
#define SHT25_RESOLUTION_MASK 0x81
/*! Default refresh rate in ms. Minimum is resolution dependent and for highest resolution
 * 200 ms with a small security margin. */
#define SHT25_DEFAULT_REFRESH_RATE 1000
/*! CRC8 Polynomial for calculating the checksum of measurement data. Binary representation
 *  from the datasheet: \b 0b100110001 */
#define SHT25_CRC8_POLYNOMIAL 0x131


/* ===========================================================================*/
/*                    States of the state machine                             */
/* ===========================================================================*/
typedef enum
{
  E_SHT25_STATE_PENDING,
  E_SHT25_STATE_IDLE,
  E_SHT25_STATE_MEASURE,
  E_SHT25_STATE_RESET,
  E_SHT25_STATE_OFF,
  E_SHT25_STATE_TIMEOUT
} E_SHT25_STATE_t;

typedef enum
{
  E_SHT25_LAST_T,
  E_SHT25_LAST_RH
} E_SHT25_LAST_t;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
s_hal_i2c_ctx_t *sht25_i2c_ctx;
static uint8_t slave_address;
static E_SHT25_STATE_t SHT25_state;
static E_SHT25_LAST_t SHT25_last_measurement;
static bool SHT25_active;
static uint8_t regValue[3];
static uint8_t SHT25_RH_data[3];
static uint8_t SHT25_T_data[3];
static uint8_t SHT25_reset_data[1];
static uint8_t SHT25_trigger_data[1];
static uint8_t SHT25_fetch_data[1];
static uint32_t SHT25_measurement_ready;
static uint32_t SHT25_reset_finished;
static uint32_t SHT25_next_cycle;
static uint32_t SHT25_measurement_time;
static uint8_t SHT25_measurement_time_RH[4];
static uint8_t SHT25_measurement_time_T[4];
static uint8_t SHT25_reset_time;
static uint16_t SHT25_refresh_rate;
static uint16_t SHT25_cycle_time;
static uint8_t SHT25_resolution;
static bool SHT25_hasT;
static bool SHT25_hasRH;
static Timestamp SHT25_T_time;
static Timestamp SHT25_RH_time;
const static Timestamp g_timestampDefault = Timestamp_init_default;
static s_i2c_bus_mgr_write_t SHT25_bm_trigger;
static s_i2c_bus_mgr_write_t SHT25_bm_reset;
static s_i2c_bus_mgr_read_t SHT25_bm_fetchT;
static s_i2c_bus_mgr_read_t SHT25_bm_fetchRH;

/* ===========================================================================*/
/*                         loc_callback()							  */
/* ===========================================================================*/
static void loc_tempcb(void *pvData, uint_fast8_t ui8Status) {
    sht25_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
    i2c_bus_mgr_return_access(E_I2C_BUS_NODE_SHT25);
    SHT25_state = E_SHT25_STATE_IDLE;
    if(hal_pps_getTimestamp(&SHT25_T_time))
    {
      SHT25_hasT = true;
    }
    return;
}

static void loc_humicb(void *pvData, uint_fast8_t ui8Status) {
    sht25_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
    i2c_bus_mgr_return_access(E_I2C_BUS_NODE_SHT25);
    SHT25_state = E_SHT25_STATE_IDLE;
    if(hal_pps_getTimestamp(&SHT25_RH_time))
    {
      SHT25_hasRH = true;
    }
    return;
}

static void loc_triggercb(void *pvData, uint_fast8_t ui8status) {
	sht25_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	i2c_bus_mgr_return_access(E_I2C_BUS_NODE_SHT25);
    SHT25_state = E_SHT25_STATE_MEASURE;
    SHT25_measurement_ready = hal_timer_getTimeout(SHT25_measurement_time);
    return;
}

static void loc_resetcb(void *pvData, uint_fast8_t ui8status) {
	sht25_i2c_ctx->state = E_HAL_I2C_STATE_IDLE;
	i2c_bus_mgr_return_access(E_I2C_BUS_NODE_SHT25);
    SHT25_state = E_SHT25_STATE_RESET;
    SHT25_reset_finished = hal_timer_getTimeout(SHT25_reset_time);
    return;
}

/* Not implemented yet
static void loc_readReg()
{
  uint8_t data[1];
  data[0] = SHT25_CMD_TRIG_T_MEASUREMENT;
  i2c_bus_mgr_request_access(E_I2C_BUS_NODE_SHT25);
  if (i2c_bus_mgr_current_access() == E_I2C_BUS_NODE_SHT25) {
	  SHT25_state = E_SHT25_STATE_PENDING;
	  hal_i2c_read(sht25_i2c_ctx, slave_address, data, 1, regValue, 3, &loc_callback);
  }
  return;
}
static bool loc_writeReg(E_HAL_I2C_PORT_t e_port)
{
  bool b_return = false;
  return b_return;
}

static SHT25_DATA_t loc_readData(E_HAL_I2C_PORT_t e_port)
{
  SHT25_DATA_t ui16_data = 0;
  return ui16_data;
}*/
/* ===========================================================================*/
/*                  API FUNCTION IMPLEMENTATIONS                              */
/* ===========================================================================*/

/* ===========================================================================*/
/*       sht25_init() Initializes sht25 and returns true iff success        */
/* ===========================================================================*/
bool SHT25_init()
{
  bool b_return = false;
  regValue[0] = 0;
  regValue[1] = 0;
  regValue[2] = 0;
  SHT25_RH_data[0] = 0;
  SHT25_RH_data[1] = 0;
  SHT25_RH_data[2] = 0;
  SHT25_T_data[0] = 0;
  SHT25_T_data[1] = 0;
  SHT25_T_data[2] = 0;
  SHT25_measurement_time_RH[0] = 29;
  SHT25_measurement_time_RH[1] = 4;
  SHT25_measurement_time_RH[2] = 9;
  SHT25_measurement_time_RH[3] = 15;
  SHT25_measurement_time_T[0] = 85;
  SHT25_measurement_time_T[1] = 22;
  SHT25_measurement_time_T[2] = 43;
  SHT25_measurement_time_T[3] = 11;
  SHT25_measurement_time = 0;
  SHT25_reset_time = 15;
  SHT25_next_cycle = 0;
  SHT25_resolution = 0;
  SHT25_T_time = g_timestampDefault;
  SHT25_RH_time = g_timestampDefault;
  SHT25_refresh_rate = SHT25_DEFAULT_REFRESH_RATE;
  SHT25_cycle_time = SHT25_refresh_rate - (SHT25_measurement_time_RH[SHT25_resolution] + SHT25_measurement_time_T[SHT25_resolution]);
  sht25_i2c_ctx = hal_i2c_ti_drv_init(SHT25_I2C_PORT);
  slave_address = SHT25_ADDR;
  SHT25_state = E_SHT25_STATE_IDLE;
  SHT25_last_measurement = E_SHT25_LAST_T;
  SHT25_active = true;
  SHT25_hasT = false;
  SHT25_hasRH = false;

  if(sht25_i2c_ctx != NULL)
  {
    b_return = true;
    SHT25_bm_trigger.ps_ctx = sht25_i2c_ctx;
    SHT25_bm_reset.ps_ctx = sht25_i2c_ctx;
    SHT25_bm_fetchT.ps_ctx = sht25_i2c_ctx;
    SHT25_bm_fetchRH.ps_ctx = sht25_i2c_ctx;
  }

  SHT25_bm_trigger.slave_address = slave_address;
  SHT25_bm_reset.slave_address = slave_address;
  SHT25_bm_fetchT.slave_address = slave_address;
  SHT25_bm_fetchRH.slave_address = slave_address;

  SHT25_bm_trigger.write_length = 1;
  SHT25_bm_reset.write_length = 1;
  SHT25_bm_fetchT.write_length = 0;
  SHT25_bm_fetchRH.write_length = 0;

  SHT25_bm_fetchT.read_length = 3;
  SHT25_bm_fetchRH.read_length = 3;

  SHT25_bm_fetchT.read_data = SHT25_T_data;
  SHT25_bm_fetchRH.read_data = SHT25_RH_data;

  SHT25_bm_trigger.callback_pointer = &loc_triggercb;
  SHT25_bm_reset.callback_pointer = &loc_resetcb;
  SHT25_bm_fetchT.callback_pointer = &loc_tempcb;
  SHT25_bm_fetchRH.callback_pointer = &loc_humicb;

  SHT25_reset();

  return b_return;
}

/* ===========================================================================*/
/*       sht25_run() handles the state machine of sht25				          */
/* ===========================================================================*/
void SHT25_run()
{
	switch(SHT25_state){
	      case E_SHT25_STATE_PENDING:
              break;
	  	  case E_SHT25_STATE_IDLE:
	  	      if(hal_timer_isTimedOut(SHT25_next_cycle)){
	  		    SHT25_triggerMeasurement();
	  	      }
	  		  break;
	      case E_SHT25_STATE_MEASURE:
	          if(hal_timer_isTimedOut(SHT25_measurement_ready)){
	            SHT25_fetchMeasurement();
	          }
	          break;
	      case E_SHT25_STATE_RESET:
  	          if(hal_timer_isTimedOut(SHT25_reset_finished)){
	            SHT25_state = E_SHT25_STATE_IDLE;
	          }
	        break;
	      case E_SHT25_STATE_OFF:
	          break;
	  	  case E_SHT25_STATE_TIMEOUT:
	  		  break;
	  	  default:
	  	      break;
	}
}

/* ===========================================================================*/
/*       sht25_get()  returns the value of sht25                              */
/* ===========================================================================*/
bool SHT25_get(Humidity *const humidity_msg)
{
  bool b_return = false;
  if(humidity_msg)
    {
    if(SHT25_hasT || SHT25_hasRH)
    {
      b_return = true;

      if(SHT25_hasT)
      {
        SHT25_hasT = false;
        humidity_msg->generated = SHT25_T_time;
        humidity_msg->has_generated = true;
        humidity_msg->temperature_raw = (SHT25_T_data[0] << 8) + (SHT25_T_data[1] & 0xFC);
        humidity_msg->has_temperature_raw = true;
      }

      if(SHT25_hasRH)
      {
        SHT25_hasRH = false;
        humidity_msg->generated = SHT25_RH_time;
        humidity_msg->has_generated = true;
        humidity_msg->humidity_raw = (SHT25_RH_data[0] << 8) + (SHT25_RH_data[1] & 0xFC);
        humidity_msg->has_humidity_raw = true;
      }
    }
  }
  return b_return;
}

/* ===========================================================================*/
/*       sht25_setConf()  Handles the configuration message                   */
/* ===========================================================================*/
void SHT25_setConf(HumidityConf *const humidityConf_msg)
{
  if(humidityConf_msg)
  {
    if(humidityConf_msg->has_basic_conf)
    {
      if(humidityConf_msg->basic_conf.has_activated)
      {
        if(humidityConf_msg->basic_conf.activated)
        {
          if(!SHT25_active)
          {
            SHT25_state = E_SHT25_STATE_IDLE;
            SHT25_active = true;
          }
        }
        else
        {
          SHT25_active = false;      // Deactivate, but finish running measurements
        }
      }
      if(humidityConf_msg->basic_conf.has_rate && humidityConf_msg->basic_conf.rate != 0)
      {
        if(humidityConf_msg->basic_conf.rate > 8)
        {
          humidityConf_msg->basic_conf.rate = 8;
        }
        SHT25_refresh_rate = 1000 / humidityConf_msg->basic_conf.rate;
        SHT25_cycle_time = SHT25_refresh_rate - (SHT25_measurement_time_RH[SHT25_resolution] + SHT25_measurement_time_T[SHT25_resolution]);
      }
    }
  }
}

void SHT25_reset()
{

  SHT25_reset_data[0] = SHT25_CMD_SOFT_RESET;
  SHT25_state = E_SHT25_STATE_PENDING;
  SHT25_bm_reset.write_data = SHT25_reset_data;
  i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_SHT25, &SHT25_bm_reset);
}

uint8_t *SHT25_getStatus()
{
	return regValue;
}


bool SHT25_enableHeater(void)
{
  bool b_return = false;
  return b_return;
}

bool SHT25_disableHeater(void)
{
  bool b_return = false;
  return b_return;
}

bool SHT25_enableOTPReload(void)
{
  bool b_return = false;
  return b_return;
}

bool SHT25_disableOTPReload(void)
{
  bool b_return = false;
  return b_return;
}

bool SHT25_setResolution(uint8_t ui8_resolution)
{
  bool b_return = false;
  return b_return;
}

void SHT25_triggerMeasurement(void)
{
  if(SHT25_active)
  {
    switch(SHT25_last_measurement)
    {
      case E_SHT25_LAST_T:
        SHT25_trigger_data[0] = SHT25_CMD_TRIG_RH_MEASUREMENT_NOHOLD;
        SHT25_measurement_time = SHT25_measurement_time_RH[SHT25_resolution];
        SHT25_last_measurement = E_SHT25_LAST_RH;
        break;
      case E_SHT25_LAST_RH:
        SHT25_trigger_data[0] = SHT25_CMD_TRIG_T_MEASUREMENT_NOHOLD;
        SHT25_measurement_time = SHT25_measurement_time_T[SHT25_resolution];
        SHT25_last_measurement = E_SHT25_LAST_T;
        break;
      default:
        break;
     }
     SHT25_state = E_SHT25_STATE_PENDING;
     SHT25_bm_trigger.write_data = SHT25_trigger_data;
     i2c_bus_mgr_queue_write(E_I2C_BUS_NODE_SHT25, &SHT25_bm_trigger);
  }
  else
  {
    SHT25_state = E_SHT25_STATE_OFF;
  }
}

void SHT25_fetchMeasurement(void)
{
  SHT25_fetch_data[0] = 0;
  //i2c_bus_mgr_request_access(E_I2C_BUS_NODE_SHT25);
  SHT25_state = E_SHT25_STATE_PENDING;

    switch(SHT25_last_measurement){
        case E_SHT25_LAST_T:
            SHT25_bm_fetchT.write_data = SHT25_fetch_data;
            i2c_bus_mgr_queue_read(E_I2C_BUS_NODE_SHT25, &SHT25_bm_fetchT);
            break;
        case E_SHT25_LAST_RH:
            SHT25_bm_fetchRH.write_data = SHT25_fetch_data;
            SHT25_next_cycle = hal_timer_getTimeout(SHT25_cycle_time);
            i2c_bus_mgr_queue_read(E_I2C_BUS_NODE_SHT25, &SHT25_bm_fetchRH);
            break;
        default:
            break;
  }

}

// We don't need blocking access
/*void SHT25_getMeasurement(void)
{
  uint8_t data[1];
  data[0] = SHT25_CMD_TRIG_T_MEASUREMENT;
  i2c_bus_mgr_request_access(E_I2C_BUS_NODE_SHT25);
  if (i2c_bus_mgr_current_access() == E_I2C_BUS_NODE_SHT25) {
	  SHT25_state = E_SHT25_STATE_PENDING;
	  hal_i2c_read(sht25_i2c_ctx, slave_address, data, 1, regValue, 3, &loc_callback);
  }
}*/

bool SHT25_checkCRC(SHT25_DATA_t ui16_data, SHT25_CRC_t ui8_crc)
{
  bool b_return = false;
  SHT25_CRC_t ui8_checksum = 0;
  uint8_t ui8_high = 0;
  uint8_t ui8_low = 0;
  uint8_t ui8_bit = 0;

  ui8_low = ui16_data & 0x00FF;
  ui8_high = (ui16_data & 0xFF00) >> 8;

  ui8_checksum ^= ui8_high;
  for(ui8_bit = 8; ui8_bit > 0; ui8_bit--)
  {
    if(ui8_checksum & 0x80)
    {
      ui8_checksum = (ui8_checksum << 1) ^ SHT25_CRC8_POLYNOMIAL;
    }
    else
    {
      ui8_checksum = (ui8_checksum << 1);
    }
  }
  ui8_checksum ^= ui8_low;
  for(ui8_bit = 8; ui8_bit > 0; ui8_bit--)
  {
    if(ui8_checksum & 0x80)
    {
      ui8_checksum = (ui8_checksum << 1) ^ SHT25_CRC8_POLYNOMIAL;
    }
    else
    {
      ui8_checksum = (ui8_checksum << 1);
    }
  }

  if (ui8_crc == ui8_checksum)
  {
    b_return = true;
  }
  return b_return;
}


