/* Copyright (C) 2015-2017
 *
 * imu_xsens.h
 *
 * Fabian Girrbach     <fabiangirrbach@gmail.com>
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

#ifndef SENSORS_IMU_XSENS_H_
#define SENSORS_IMU_XSENS_H_


#include "Imu.pb.h"


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>



typedef enum e_Conntype{
	xsensSPI = 1,
	xsensI2C = 2,
	xsensUART = 3,
} XsensConnType;

typedef struct S_IMU_XSENS_CTX_T s_imu_xsens_ctx_t;




s_imu_xsens_ctx_t* xsens_createImu(XsensConnType connType, uint32_t speed);
void xsens_wakeUp(s_imu_xsens_ctx_t const* cntx);
bool xsens_goToConfig(s_imu_xsens_ctx_t const* cntx);
int xsens_configureMotionTracker(s_imu_xsens_ctx_t * cntx);
bool xsens_getSensorData(s_imu_xsens_ctx_t const* cntx, Imu * const mdata);
bool xsens_goToMeasurmentMode(s_imu_xsens_ctx_t const* cntx);









#endif /* SENSORS_IMU_XSENS_H_ */
