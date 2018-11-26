/* Copyright (C) 2015-2017
 *
 * log.c
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
#include "log.h"

#define LOG_BUFFER_SIZE 20

static Report log_buffer[LOG_BUFFER_SIZE];
static bool entryIsUsed[LOG_BUFFER_SIZE];
static uint8_t write_idx;
static uint8_t read_idx;

bool log_init(bool (*encode) ()) {
  bool b_return = false;
  uint8_t i;
  for (i = 0; i < LOG_BUFFER_SIZE; ++i) {
    entryIsUsed[i] = false;

    log_buffer[i].has_report_type = false;
    log_buffer[i].report_type = (Report_ReportType)0;
    log_buffer[i].report_string.funcs.encode = encode;
  }
  write_idx = 0;
  read_idx = 0;
  b_return = true;
  return b_return;
}

bool log_getError(Report * log_entry) {
  bool b_return = false;
  if (entryIsUsed[read_idx]) {
    entryIsUsed[read_idx] = false;
    *log_entry = log_buffer[read_idx];
    read_idx = (read_idx + 1) % LOG_BUFFER_SIZE;
    b_return = true;
  }
  return b_return;
}

bool log_writeError(Report_ReportType type, char * errString) {
  bool b_return = false;
  if (!entryIsUsed[write_idx]) {
    entryIsUsed[write_idx] = true;
    log_buffer[write_idx].report_type = type;
    log_buffer[write_idx].has_report_type = true;
    log_buffer[write_idx].report_string.arg = (void*) errString;

    write_idx = (write_idx + 1) % LOG_BUFFER_SIZE;
    b_return = true;
  }
  return b_return;
}
