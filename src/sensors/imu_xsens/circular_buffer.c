/* Copyright (C) 2015-2017
 *
 * circular_buffer.c
 *
 * Fabian Girrbach     <fabiangirrbach@gmail.com>
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
#include "cirular_buffer.h"
#include <stdlib.h>
#include <stdint.h>


typedef struct
{
    void** buffer;
    int head;
    int tail;
    int maxLen;
}circularBuffer_t;


void* circular_buffer_init(uint16_t size)
{
	circularBuffer_t* cb = (circularBuffer_t*)malloc(sizeof(circularBuffer_t));
	if (cb != NULL)
	{
		cb->buffer = malloc(sizeof(void*)*size);
		cb->maxLen = size;
		cb->head = 0;
		cb->tail = 0;
	}
	return cb;
}


int circular_buffer_push(void* buf, void* data)
{
	circularBuffer_t* c = (circularBuffer_t*) buf;
	int next = c->head + 1;
    if (next >= c->maxLen)
        next = 0;
 
    // Cicular buffer is full
    if (next == c->tail)
        return -1;  // quit with an error
 
    c->buffer[c->head] = data;
    c->head = next;
    return 0;
}
 
void* circular_buffer_pop(void* buf)
{
	circularBuffer_t* c = (circularBuffer_t*) buf;
	// if the head isn't ahead of the tail, we don't have any characters
    if (c->head == c->tail)
        return NULL;  // quit with an error
 
    void* data = c->buffer[c->tail];
    c->buffer[c->tail] = 0;  // clear the data (optional)
 
    int next = c->tail + 1;
    if(next >= c->maxLen)
        next = 0;
 
    c->tail = next;
 
    return data;
}

void circular_buffer_destroy(void* buf)
{
	circularBuffer_t* cb = (circularBuffer_t*) buf;
	free(cb->buffer);
	free(cb);
}


