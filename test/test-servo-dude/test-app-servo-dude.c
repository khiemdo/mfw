/* Copyright (C) 2015-2017
 *
 * test-app-servo-dude.c
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

#include <stdbool.h>
#include <stdint.h>
#include "hal_timer.h"
#include "hal_mcu.h"
#include "hal_gpio.h"
#include "servo-dude/servo-dude.h"
#include "BetPUSH.pb.h"
#include "power/servopower.h"



/*! Delay between SENDs of the test app in milliseconds. */
#define TEST_APP_TEST_DELAY   10

#define TEST_APP_SRV_DUD_ID E_SERVO_DUDE_ELEVATOR
#define TEST_APP_SRV_DUD_ID2 E_SERVO_DUDE_RUDDER
#define TEST_APP_SRV_DUD_ID3 E_SERVO_DUDE_AILERON_RIGHT
#define TEST_APP_SRV_DUD_ID4 E_SERVO_DUDE_FLAP_RIGHT
/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
uint32_t test_interval_timer;
s_servo_dude_ctx_t *gps_srv_dude;
s_servo_dude_ctx_t *gps_srv_dude2;
s_servo_dude_ctx_t *gps_srv_dude3;
s_servo_dude_ctx_t *gps_srv_dude4;

Actuator act_msg = {};
Actuator act_msg2 = {};
//BetPUSH_ActuatorsConf act_conf_msg = {};
uint16_t error;
int16_t counter;
bool sign;
/* ===========================================================================*/
/*                  loc_servo_dude_init()                                            */
/* ===========================================================================*/
bool loc_srv_dud_init(E_SERVO_DUDE_ID_t e_dude, s_servo_dude_ctx_t **p_gps_srv_dude)
{

  bool b_return = false;
  *p_gps_srv_dude = servo_dude_init(e_dude);
  if(p_gps_srv_dude != NULL)
  {
    b_return = true;
  }
  return b_return;
}/* loc_srv_dud_init() */

/* ===========================================================================*/
/*                  loc_test_init()                                           */
/* ===========================================================================*/
bool loc_test_init(void)
{
    bool b_return = hal_mcu_init();

    if(b_return)
    {
      b_return = hal_timer_init();
    }

	test_interval_timer = hal_timer_getTimeout(TEST_APP_TEST_DELAY);
  /* Fill in our test data. */
	act_msg.has_set_point = true;
	act_msg.set_point = 0xFFFF;
    act_msg2.has_set_point = true;
    act_msg2.set_point = 0xFFFF;
    counter = 0;
    sign = 0;
	return b_return;
}/* loc_test_init() */

void loc_update_msg()
{
    if (!sign)
    {
        counter = (counter + 1);
    }
    else
    {
        counter = (counter - 1);
    }
    if (counter > 999)
    {
        sign = 1;
        counter = 1000;
    }
    if (counter < 0)
    {
        sign = 0;
        counter = 0;
    }
    act_msg.set_point = counter+1000;
    act_msg2.set_point = 2000-counter;
}

bool loc_test_run(void)
{
    servo_dude_run(gps_srv_dude);
    servo_dude_run(gps_srv_dude2);
    servo_dude_run(gps_srv_dude3);
    servo_dude_run(gps_srv_dude4);
    if (hal_timer_isTimedOut(test_interval_timer)) {
    	loc_update_msg();
        servo_dude_set(gps_srv_dude,  &act_msg);
        servo_dude_set(gps_srv_dude2,  &act_msg2);
        servo_dude_set(gps_srv_dude3,  &act_msg);
        servo_dude_set(gps_srv_dude4,  &act_msg2);
    	test_interval_timer = hal_timer_getTimeout(TEST_APP_TEST_DELAY);
    }
	return true;
}/* loc_test_run() */

int main(void) {
    if (!servopower_init()) return(0);
    servopower_off();
    servopower_on();
    if (!loc_test_init()) return(0);
    if (!loc_srv_dud_init(TEST_APP_SRV_DUD_ID,  &gps_srv_dude))  return(0);
    if (!loc_srv_dud_init(TEST_APP_SRV_DUD_ID2,  &gps_srv_dude2))  return(0);
    if (!loc_srv_dud_init(TEST_APP_SRV_DUD_ID3,  &gps_srv_dude3))  return(0);
    if (!loc_srv_dud_init(TEST_APP_SRV_DUD_ID4,  &gps_srv_dude4))  return(0);

    while(true) {
    	loc_test_run();
    }
}
