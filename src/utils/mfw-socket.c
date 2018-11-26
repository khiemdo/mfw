/* Copyright (C) 2015-2017
 *
 * mfw-socket.c
 *
 * Martin Dold    <martin.dold@gmx.net>
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "mfw-socket.h"

#include "hal_mcu.h"
#include "hal_timer.h"
#include "hal_ethernet.h"

#include "lwiplib.h"

//*****************************************************************************
//
// Start doxygen group
//! \addtogroup utils_mfw_socket
//! @{
//
//*****************************************************************************
/*! @brief Interval (in milliseconds) when ETH interrupt handler is called i.e.
 *         when lwIP is processed. */
#define LWIP_TICK_MS           10U


typedef void (*pfn_callbackRxDataRdy_t)(uint8_t *, uint16_t);

static uint8_t *g_dataBuf;
static uint16_t g_dataBufSize;
static pfn_callbackRxDataRdy_t pfn_cb;
static bool gb_dataAvailable;
static uint16_t g_lastDataLenRvcd;
static uint32_t g_errorDataOverflow;
static ip_addr_t g_destIPAddr;
static uint16_t g_destPort;

static void *pcb;

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static void loc_lwIpTimerCallback(void);
static void
loc_udpRx(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port);

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_lwIpTimerCallback()                                   */
/* ===========================================================================*/
static void loc_lwIpTimerCallback(void)
{
  lwIPTimer(LWIP_TICK_MS);
}

/* ===========================================================================*/
/*                  loc_udpRx()                                               */
/* ===========================================================================*/
static void
loc_udpRx(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port)
{
  if(false == gb_dataAvailable)
  {
    /* Validate data sizes first. */
    if(p->tot_len > g_dataBufSize)
    {
      g_lastDataLenRvcd = g_dataBufSize;
    }
    else
    {
      g_lastDataLenRvcd = p->tot_len;
    }

    /* As this function loc_udpRx() is called from ETH interrupt we copy the data
     * to user defined data buffer and raise the RX event in mfw_socket_udpRun()
     * to make sure that the pb_decode() is not performed inside the interrupt. */
    if( pbuf_copy_partial(p, g_dataBuf, g_lastDataLenRvcd, 0U) == g_lastDataLenRvcd )
    {
      gb_dataAvailable = true;
    }
    else
    {
      g_lastDataLenRvcd = 0U;
    }
  }
  else
  {
    /* New data received while the last data is not yet processed by the user. */
    g_errorDataOverflow++;
  }

  /* The incoming pbuf is no longer needed, so free it. */
  pbuf_free(p);

  return;
}

/* ===========================================================================*/
/*                    API function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  mfw_socket_init()                                         */
/* ===========================================================================*/
bool mfw_socket_init( uint32_t ui32IPAddr,
                      uint32_t ui32NetMask,
                      uint32_t ui32GWAddr
                    )
{
  bool b_return = false;
  const uint8_t *p_macAddr = NULL;
  uint32_t sysClock = 0U;

  /* Proceed with initialization from bottom to top:
   * 1) Init ethernet MAC/PHY. */
  b_return = hal_eth_init();

  if(b_return)
  {
    p_macAddr = hal_eth_getMacAddress();
    sysClock = hal_mcu_getSysClock();

    if(p_macAddr)
    {
      /* 2) Init lwIP stack, i.e. IP and socket (TCP, UDP) layer */
      lwIPInit( sysClock,
                p_macAddr,
                ui32IPAddr,
                ui32NetMask,
                ui32GWAddr,
                IPADDR_USE_STATIC
              );

      /* Start a timer that periodically calls the ETH interrupt handler
       * and thereby processes the lwIP stack. */
      b_return = hal_timer_registerCallback(LWIP_TICK_MS, loc_lwIpTimerCallback);

      /* Init local variables here to ensure proper working if run() is called
       * before udpStart(). */
      g_errorDataOverflow = 0U;
      gb_dataAvailable = false;

      g_dataBuf = NULL;
      g_dataBufSize = 0U;
      pfn_cb = NULL;
      ip_addr_set_zero(&g_destIPAddr);
      g_destPort = 0U;
      pcb = NULL;
    }

  }

  return b_return;
}

/* ===========================================================================*/
/*                  mfw_socket_udpStart()                                     */
/* ===========================================================================*/
bool mfw_socket_udpStart( uint16_t srcPort,
                          uint32_t destIPAddr,
                          uint16_t destPort,
                          uint8_t *pc_rxDataBuf,
                          uint16_t rxDataBufLen,
                          void (*callbackRxDataRdy)(uint8_t *pc_data, uint16_t ui16_len)
                        )
{
  bool b_return = false;
  err_t err;

  /* Setup UDP port to communicate with GS. */
  pcb = udp_new();
  if(pcb)
  {
    udp_recv(pcb, loc_udpRx, NULL);

    /* Convert user param from host to network byte order. */
    g_destIPAddr.addr = htonl(destIPAddr);

    err = udp_bind(pcb, IP_ADDR_ANY, srcPort);

    if(ERR_OK == err)
    {
      /* Store user parameter */
      g_dataBuf = pc_rxDataBuf;
      g_dataBufSize = rxDataBufLen;
      memset(g_dataBuf, 0U, g_dataBufSize);
      pfn_cb = callbackRxDataRdy;

      g_destPort = destPort;

      b_return = true;
    }
    else
    {
      ip_addr_set_zero(&g_destIPAddr);
    }
  }

  return b_return;
}

/* ===========================================================================*/
/*                  mfw_socket_udpRun()                                       */
/* ===========================================================================*/
void mfw_socket_udpRun(void)
{
  //
  // All the lwIP work is done in interrupt handlers. But we check if we
  // received new data (within interrupt) that must be passed to user now.
  //
  if(gb_dataAvailable)
  {
    if(pfn_cb)
    {
      /* Inform user of received data and clear right afterwards. */
      pfn_cb(g_dataBuf, g_lastDataLenRvcd);
      memset(g_dataBuf, 0U, g_dataBufSize);
      gb_dataAvailable = false;
    }
  }

  return;
}

/* ===========================================================================*/
/*                  mfw_socket_udpSend()                                      */
/* ===========================================================================*/
bool mfw_socket_udpSend(uint8_t *data, uint16_t dataLen)
{
  bool b_return = false;
  struct pbuf *p = NULL;
  err_t err = ERR_OK;

  /* Check for pcb to ensure udpStart() was called before. */
  if(data && pcb)
  {
    /* Allocate a new pbuf for sending. */
    p = pbuf_alloc(PBUF_TRANSPORT, dataLen, PBUF_RAM);

    if(p)
    {
      /* Copy the data into the pbuf, i.e. to lwIP stack memory. */
      if( ERR_OK == pbuf_take(p, data, dataLen) )
      {
        /* Send the response. */
        err = udp_sendto(pcb, p, &g_destIPAddr, g_destPort);
        if( ERR_OK == err )
        {
          b_return = true;
        }
      }
    }

    /* Free the pbuf. */
    pbuf_free(p);
  }

  return b_return;
}
