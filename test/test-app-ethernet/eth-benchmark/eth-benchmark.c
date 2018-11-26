/* Copyright (C) 2015-2017
 *
 * eth-benchmark.c
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "eth-benchmark.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#ifndef TI_DEMO_APP_ETH
#include "lwiplib.h"
/* No UART_PRINTF() here. */
#else
#define ETH_BENCHMARK_DEBUG_UART_STDIO
#include "utils/lwiplib.h"
#endif

#ifndef ETH_BENCHMARK_DEBUG_UART_STDIO
#define UART_PRINTF(...)
#else
#include "utils/uartstdio.h"
#define UART_PRINTF(...)    UARTprintf(##__VA_ARGS__)
#endif /* ETH_BENCHMARK_UART_DEBUG */




/* ===========================================================================*/
/*                  Defines/Macros                                            */
/*=========================================================================== */

/*! UDP port used to transmit/receive data for benchmark purpose. */
#define ETH_BENCHM_UDP_PORT_DATA   8080
/*! UDP port used to configure the benchmark mode (e.g. transmit or receive). */
#define ETH_BENCHM_UDP_PORT_CONF   8081
/*! TCP port used to transmit/receive data for benchmark purpose. */
#define ETH_BENCHM_TCP_PORT_DATA   8082

#define ETH_BENCHM_MAX_DATA_SIZE   8096

#define ETH_BENCHM_MAX_CONF_DATA_SIZE   50

#define ETH_BENCHM_CMD_STAT_CLR "ETH_BENCHMARK_COMMAND_STATISTIC_CLEAR\r\n"
#define ETH_BENCHM_CMD_STAT_GET "ETH_BENCHMARK_COMMAND_STATISTIC_GET\r\n"


#define ETH_BENCHM_TCP_POLL_INTERVALL  4U


/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/
uint8_t gac_data[ETH_BENCHM_MAX_DATA_SIZE] = { 0U };
uint8_t gac_confData[ETH_BENCHM_MAX_CONF_DATA_SIZE] = { 0U };
uint32_t gui32_rxDataCount;
uint32_t gui32_txDataCount;
uint32_t gui32_txErrorCount;
uint32_t gui32_tcpPollCount;

const uint8_t gstr_error[] = "Error: Unknown command.\r\n";
const uint8_t gstr_ok[] = "OK.\r\n";

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static void loc_clearStatistics(void);

static void
loc_udpRxConf(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port);
static void
loc_udpRxData(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port);

static err_t loc_tpcAccept(void *arg, struct tcp_pcb *pcb, err_t err);

static err_t loc_tcpRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);

static void loc_tcpError(void *arg, err_t err);

static err_t loc_tcpSent(void *arg, struct tcp_pcb *pcb, u16_t len);

static err_t loc_tcpPoll(void *arg, struct tcp_pcb *pcb);

static err_t
loc_tcpWrite(struct tcp_pcb *pcb, const void* ptr, uint16_t *length, uint8_t apiflags);


/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

/* ===========================================================================*/
/*                  loc_clearStatistics()                                     */
/* ===========================================================================*/
static void loc_clearStatistics(void)
{
  gui32_rxDataCount  = 0U;
  gui32_txDataCount  = 0U;
  gui32_txErrorCount = 0U;
  gui32_tcpPollCount = 0U;
  return;
}/* loc_clearStatistics() */

/* ===========================================================================*/
/*                  loc_udpRxConf()                                           */
/* ===========================================================================*/
static void
loc_udpRxConf(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port)
{
  uint16_t dataLen = 0U;
  int16_t tmp = 0U;

  memset(gac_confData, 0U, sizeof(gac_confData));

  /* Validate data sizes first. */
  if(p->tot_len > ETH_BENCHM_MAX_CONF_DATA_SIZE)
  {
    dataLen = ETH_BENCHM_MAX_CONF_DATA_SIZE;
  }
  else
  {
    dataLen = p->tot_len;
  }

  if( pbuf_copy_partial(p, gac_confData, dataLen, 0U) == dataLen )
  {
    /* Free our receive buf. */
    pbuf_free(p);

    if(strncmp( (const char *)gac_confData, ETH_BENCHM_CMD_STAT_CLR,
                sizeof(ETH_BENCHM_CMD_STAT_CLR) ) == 0)
    {
      loc_clearStatistics();
      memset(gac_confData, 0U, sizeof(gac_confData));

      tmp = snprintf( (char *)gac_confData, sizeof(gac_confData), "%s", gstr_ok);

      if( (tmp > 0U) && (tmp < sizeof(gac_confData)) )
      {
        dataLen = tmp;
      }
    }
    else if(strncmp( (const char *)gac_confData, ETH_BENCHM_CMD_STAT_GET,
                      sizeof(ETH_BENCHM_CMD_STAT_GET) ) == 0)
    {
      memset(gac_confData, 0U, sizeof(gac_confData));
      dataLen = sizeof(gstr_ok);

      tmp = snprintf( (char *)gac_confData, sizeof(gac_confData),
                      "Rx= %d Tx= %d TxErr= %d \r\n",
                      gui32_rxDataCount, gui32_txDataCount, gui32_txErrorCount);

      if( (tmp > 0U) && (tmp < sizeof(gac_confData)) )
      {
        dataLen = tmp;
      }
    }
    else
    {
      /* Error: Unknown command! */
      memset(gac_confData, 0U, sizeof(gac_confData));
      tmp = snprintf( (char *)gac_confData, sizeof(gac_confData), "%s", gstr_error);

      if( (tmp > 0U) && (tmp < sizeof(gac_confData)) )
      {
        dataLen = tmp;
      }
    }

    /* Allocate a new pbuf for sending the response. */
    p = pbuf_alloc(PBUF_TRANSPORT, dataLen, PBUF_RAM);

    if(p)
    {
      if( ERR_OK == pbuf_take(p, gac_confData, dataLen) )
      {
        /* Actually, there is no proper reaction if this fails so skip return. */
        udp_sendto(pcb, p, addr, ETH_BENCHM_UDP_PORT_CONF);

      }

      pbuf_free(p);

    }/* if() */
  }
  else
  {
    /* Error during receive , but free the buf either way. */
    pbuf_free(p);
  }/* if() ... else */

  return;
} /* loc_udpRxConf() */


/* ===========================================================================*/
/*                  loc_udpRxData()                                           */
/* ===========================================================================*/
static void
loc_udpRxData(void *arg, struct udp_pcb *pcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port)
{
  uint16_t dataLen = 0U;

  /* Validate data sizes first. */
  if(p->tot_len > ETH_BENCHM_MAX_DATA_SIZE)
  {
    dataLen = ETH_BENCHM_MAX_DATA_SIZE;
  }
  else
  {
    dataLen = p->tot_len;
  }

  /*
   * For the sake of testing/benchmarking, simply copy the received
   * data to our local buffer. This kind of reflects data processing.
   * As the pbuf is a linked list of buffers and it is not 100%
   * sure that the data memory is fully aligned, let's use the
   * (weird?) API provided by the lwIP stack.
   */
  if( pbuf_copy_partial(p, gac_data, dataLen, 0U) == dataLen )
  {
    /* Count our received bytes */
    gui32_rxDataCount += dataLen;

    /* The incoming pbuf is no longer needed, so free it. */
    pbuf_free(p);

    /* Allocate a new pbuf for sending the response. */
    p = pbuf_alloc(PBUF_TRANSPORT, dataLen, PBUF_RAM);

    if(p)
    {
      /* Copy the response packet data into the pbuf.
       * Again, using the (weird?) API provided by
       * lwIP stack. */
      if( ERR_OK == pbuf_take(p, gac_data, dataLen) )
      {
        /* Send the response. */
        if( ERR_OK == udp_sendto(pcb, p, addr, ETH_BENCHM_UDP_PORT_DATA) )
        {
          /* Free the pbuf. */
          pbuf_free(p);

          /* Update counter and give some visual feedback. */
          gui32_txDataCount += dataLen;

          if( MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) )
          {
            /* LED OFF. */
            MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
          }
          else
          {
            /* LED ON. */
            MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
          }/* if () ... else */
        }
        else
        {
          /* Transmission failed. */
          gui32_txErrorCount++;
        }/* if() ... else */
      }
      else
      {
        /* Transmission failed. */
        gui32_txErrorCount++;
      }/* if() ... else */
    }/* if() */
  }/* if() */

  return;
} /* loc_udpRxData() */


/* ===========================================================================*/
/*                  loc_tpcAccept()                                           */
/* ===========================================================================*/
static err_t loc_tpcAccept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  tcp_setprio(pcb, TCP_PRIO_MAX);

  /* Set up the various callback functions */
  tcp_recv(pcb, loc_tcpRecv);
  tcp_err(pcb, loc_tcpError);
  tcp_poll(pcb, loc_tcpPoll, ETH_BENCHM_TCP_POLL_INTERVALL);
  tcp_sent(pcb, loc_tcpSent);

  return ERR_OK;
}

/* ===========================================================================*/
/*                  loc_tcpRecv()                                             */
/* ===========================================================================*/
static err_t loc_tcpRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  UART_PRINTF("loc_tcpRecv: p->tot_len=%u , p->len=%u , err=%i\r\n", p->tot_len, p->len, err);

  uint16_t dataLen = 0U;

  /* Validate data sizes first. */
  if(p->len > ETH_BENCHM_MAX_DATA_SIZE)
  {
    dataLen = ETH_BENCHM_MAX_DATA_SIZE;
  }
  else
  {
    dataLen = p->len;
  }

  /*
   * For the sake of testing/benchmarking, simply copy the received
   * data to our local buffer. This kind of reflects data processing.
   * As the pbuf is a linked list of buffers and it is not 100%
   * sure that the data memory is fully aligned, let's use the
   * (weird?) API provided by the lwIP stack.
   */
  if( pbuf_copy_partial(p, gac_data, dataLen, 0U) == dataLen )
  {
    /* Count our received bytes */
    gui32_rxDataCount += dataLen;

    /* Inform TCP that we have taken the data. */
    tcp_recved(pcb, p->tot_len);
    /* pbuf not passed to application, free it now */
    pbuf_free(p);

    if(ERR_OK == loc_tcpWrite(pcb, (void *)gac_data, &dataLen, TCP_WRITE_FLAG_COPY) )
    {


      if( MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) )
      {
        /* LED OFF. */
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
      }
      else
      {
        /* LED ON. */
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
      }/* if () ... else */
    }
  }

  return ERR_OK;
}

/* ===========================================================================*/
/*                  loc_tcpError()                                            */
/* ===========================================================================*/
static void loc_tcpError(void *arg, err_t err)
{
  UART_PRINTF("loc_tcpError: err=%i\r\n", err);
  gui32_txErrorCount++;
  return;
}

/* ===========================================================================*/
/*                  loc_tcpSent()                                             */
/* ===========================================================================*/
static err_t loc_tcpSent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  UART_PRINTF("loc_tcpSent: len=%u\r\n", len);
  gui32_txDataCount += len;
  return ERR_OK;
}

/* ===========================================================================*/
/*                  loc_tcpPoll()                                             */
/* ===========================================================================*/
static err_t loc_tcpPoll(void *arg, struct tcp_pcb *pcb)
{
  gui32_tcpPollCount++;
  UART_PRINTF("loc_tcpPoll: tcpPollCount=%u\r\n", gui32_tcpPollCount);
  return ERR_OK;
}

/* ===========================================================================*/
/*                  loc_tcpWrite()                                            */
/* ===========================================================================*/
static err_t
loc_tcpWrite(struct tcp_pcb *pcb, const void* ptr, u16_t *length, u8_t apiflags)
{
  err_t err;
  uint16_t len = *length;
  do {
    err = tcp_write(pcb, ptr, len, apiflags);
    if (err == ERR_MEM) {
      if ((tcp_sndbuf(pcb) == 0) ||
          (tcp_sndqueuelen(pcb) >= TCP_SND_QUEUELEN)) {
        /* no need to try smaller sizes */
        len = 1;
      } else {
        len /= 2;
      }
    }
  } while ((err == ERR_MEM) && (len > 1));

  *length = len;
  return err;
}

/* ===========================================================================*/
/*                  API function implementations                              */
/* ===========================================================================*/

void eth_benchmark_init(void)
{
  void *pcb;
  err_t err;

  memset(gac_confData, 0U, sizeof(gac_confData));
  memset(gac_data, 0U, sizeof(gac_data));
  loc_clearStatistics();

  //
  // Configure Port N1 for as an output for the animation LED.
  //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
  //
  // Initialize LED to OFF (0)
  //
  MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

  /* Setup UDP configuration port. */
  pcb = udp_new();
  udp_recv(pcb, loc_udpRxConf, NULL);
  udp_bind(pcb, IP_ADDR_ANY, ETH_BENCHM_UDP_PORT_CONF);

  /* Setup UDP data port. */
  pcb = udp_new();
  udp_recv(pcb, loc_udpRxData, NULL);
  udp_bind(pcb, IP_ADDR_ANY, ETH_BENCHM_UDP_PORT_DATA);

  /* Setup TCP data port. */
  pcb = tcp_new();
  tcp_setprio(pcb, TCP_PRIO_MAX);
  err = tcp_bind(pcb, IP_ADDR_ANY, ETH_BENCHM_TCP_PORT_DATA);

  if(ERR_OK == err)
  {
    pcb = tcp_listen(pcb);

    if(pcb)
    {
      /* initialize callback arg and accept callback */
      tcp_arg(pcb, pcb);
      tcp_accept(pcb, loc_tpcAccept);
    }
  }

  return;
}
