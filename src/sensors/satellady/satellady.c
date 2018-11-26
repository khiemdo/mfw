/* Copyright (C) 2015-2017
 *
 * satellady.c
 *
 * Elias Rosch         <eliasrosch@gmail.com>
 * Martin Dold         <martin.dold@gmx.net>
 * Thorbjörn Jörger    <thorbjoern.joerger@web.de>
 * Julian Reimer       <schemel9@gmail.com>
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

#include <stdio.h>
#include <string.h>

#include "satellady.h"
#include "hal_uart.h"

//! Defines the max. no. bytes to buffer
#define SAT_BUFFER_LEN 200

/* ===========================================================================*/
/*                    Enums                                                   */
/* ===========================================================================*/
//
typedef enum{
  E_SATELADY_STATE_ERROR, //!< When sth. goes wrong

  E_SATELADY_STATE_FIND_START_OF_LINE, //!< Find start of line determined by a '$'
  E_SATELADY_STATE_GET_DATA,	//!< Read bytewise until end-delimiter '*'
  E_SATELADY_STATE_GET_CRC_BYTE1, //!< Read first CRC
  E_SATELADY_STATE_GET_CRC_BYTE2, //!< Read second CRC and test for correctness
  E_SATELADY_STATE_FIND_END_OF_LINE_BYTE1, //!< Find line break '\r'
  E_SATELADY_STATE_FIND_END_OF_LINE_BYTE2, //!< Find line break '\n'
}E_SATELADY_STATE_T;

/* ===========================================================================*/
/*                    Global variables                                        */
/* ===========================================================================*/

static E_SATELADY_STATE_T g_satelady_state; //!< Contains the current state of module
s_hal_uart_ctx_t *g_gf8701_ctx; //!< Context to Furuno-UART channel
s_hal_uart_ctx_t *g_host_ctx; //!< Context to Host-UART channel (e.g. FC-arm, FC-plane or GS)

static char g_recv_buffer[SAT_BUFFER_LEN]; //!< buffer containing data from start to end of line
uint8_t g_already_read; //! Counter of bytes that are already in the buffer

uint8_t g_recvCrc[3];
uint8_t g_crcByte;
char g_crcCalculated;

uint32_t g_dbgCountLines;
uint32_t g_dbgCountError;
uint32_t g_dbgCountWrongStart;

FurunoGps g_furuno_gps; //!< local protobuf message (content is copied when *get() is called))
FurunoGps g_furuno_gps_default = FurunoGps_init_default;
static bool gps_data_avail; //!< Flag indicating that new data has been parsed.
static bool g_isInitialized; //!< Flag indicating a succesfull configuration of Furuno


static void loc_checkInit(char * strLine, uint16_t len);
static void loc_testParsing (void);
static char * loc_strtok_single (char* str, char const * delims);

static void loc_parseLine   (char *strLine, uint16_t len);
static void loc_strToLong(char * str, uint32_t *deg, float * min);
static void loc_strToLat(char * str, uint32_t *deg, float * min);
static void loc_parseGnrmc  (char *str, uint16_t len);
static void loc_parseGngns  (char *str, uint16_t len);
static void loc_parsePerdcr (char *str, uint16_t len);
static bool loc_cmd_to_uart(char * command);
static char loc_calcCrc(char *data, uint16_t len);



/* ===========================================================================*/
/*                    local functions implementation                          */
/* ===========================================================================*/
//! Check received string for "PERDACK" (if not resend config)
static void loc_checkInit(char * strLine, uint16_t len)
{
  if (!strncmp(g_recv_buffer, "PERDACK", 7))
  {
    g_isInitialized = true;
  }
  else
  {
    loc_cmd_to_uart("$PERDAPI,PPS,VCLK,1,0,10,0,0*36");
  }
}

//! This function is only for developing
static void loc_testParsing(void)
{
  /* This function is called from init() to just test our parsing functions! */

  const char gnrmc[] = "GNRMC,000359.000,V,0123.4560,N,00456.1230,E,0.00,0.00,220899,,,N,V";
  const char gngns[] = "GNGNS,000359.000,0000.0000,N,00000.0000,E,NNN,15,,-180.0,134.0,,,V";
  const char perdcrz[] = "PERDCRZ,TPS4,4,1,01,01,+000000000,+00000,0000,0000000,000000,+000000";

  /* Add all other lines we need to parse here and test your implementation
   * from here! */

  loc_parseLine( (char *)gnrmc, strlen(gnrmc));
  loc_parseLine( (char *)gngns, strlen(gngns));
  loc_parseLine( (char *)perdcrz, strlen(perdcrz));

}

//! Related to c-strtok function.
//! Difference: doesn't skip empty slots (between delimiters)
static char *loc_strtok_single (char * str, char const * delims) {
  static char  *src = NULL;
  char  *p,  *ret = 0;

  if (str != NULL)
    src = str;

  if (src == NULL)
    return NULL;

  if ((p = strpbrk (src, delims)) != NULL) {
    *p  = 0;
    ret = src;
    src = ++p;
  }
  else if (*src)
  {
    ret = src;
    src = NULL;
  }
  return ret;
}

static void loc_parseLine(char *strLine, uint16_t len)
{
  /* This function is the main function to be called when a complete line
   * was received. The parsing starts here, no where else!
   * Firstly, determine the first characters of the line
   * ("GNRMC" vs "PERDCRX" etc.) and when detected, call the dedicated local
   * function that parses the rest of the line, e.g.:
   */
  if (!strncmp(strLine, "GNRMC,", 6))
  {
    loc_parseGnrmc(strLine + 6, len - 6);
    gps_data_avail |= true;
  }
  else if (!strncmp(strLine, "GNGNS,", 6))
  {
    loc_parseGngns(strLine + 6, len - 6);
    gps_data_avail |= true;
  }
  else if (!strncmp(strLine, "PERDCRZ,", 8))
  {
    loc_parsePerdcr(strLine + 8, len - 8);
    gps_data_avail |= true;
  }
}

//!< Converts furuno-latitude representation to [(N/S), degrees, minutes]
static void loc_strToLat(char * str, uint32_t *deg, float * min) {
  (*deg) = 10 * (str[0]-'0') + (str[1]-'0');
  (*min) = strtof(str+2, NULL);
}
//!< Converts furuno-longitude representation to [(E/W), degrees, minutes]
static void loc_strToLong(char * str, uint32_t *deg, float * min) {
  (*deg) = 100 * (str[0]-'0') + 10 * (str[1]-'0') + (str[2] -'0');
  (*min) = strtof(str+3, NULL);
}

//!< Parse information from GNMRC line (further info in: FURUNO GNSSDO-Protocol Specs)
static void loc_parseGnrmc  (char *str, uint16_t len)
{
// FIELD 1
  char * split = loc_strtok_single(str, ",");
// FIELD 2
  split = loc_strtok_single(NULL, ",");
  if (*split == 'A')
  {
    g_furuno_gps.has_position = true;
  }
// FIELD 3: ddmm.mmm
  split = loc_strtok_single(NULL, ",");
  uint32_t deg;
  float min;
  loc_strToLat(split, &deg, &min);
  g_furuno_gps.position.latitude_deg = deg;
  g_furuno_gps.position.latitude_min = min;
  g_furuno_gps.position.has_latitude_deg = true;
  g_furuno_gps.position.has_latitude_min = true;
// FIELD 4
  split = loc_strtok_single(NULL, ",");
  if (*split == 'N')
  {
    g_furuno_gps.position.lat_sign = FurunoGpsPosition_FurunoGpsPositionSign_N;
  }
  else
  {
    g_furuno_gps.position.lat_sign = FurunoGpsPosition_FurunoGpsPositionSign_S;
  }
  g_furuno_gps.position.has_lat_sign = true;
  // FIELD 5: dddmm.mmm
  split = loc_strtok_single(NULL, ",");
  loc_strToLong(split, &deg, &min);
  g_furuno_gps.position.longitude_deg = deg;
  g_furuno_gps.position.longitude_min = min;
  g_furuno_gps.position.has_longitude_deg = true;
  g_furuno_gps.position.has_longitude_min = true;
  // FIELD 6
  split = loc_strtok_single(NULL, ",");
  if (*split == 'E')
  {
    g_furuno_gps.position.long_sign = FurunoGpsPosition_FurunoGpsPositionSign_E;
  }
  else
  {
    g_furuno_gps.position.long_sign = FurunoGpsPosition_FurunoGpsPositionSign_W;
  }
  g_furuno_gps.position.has_long_sign = true;
  g_furuno_gps.has_position = true;
  // FIELD 7
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.velocity.speed_kts = strtof(split, NULL);
  g_furuno_gps.velocity.has_speed_kts = true;
  // FIELD 8
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.velocity.true_course = strtof(split, NULL);
  g_furuno_gps.velocity.has_true_course = true;
  g_furuno_gps.has_velocity = true;
}

//!< Parse information from GNGNS line (further info in: FURUNO GNSSDO-Protocol Specs)
static void loc_parseGngns  (char *str, uint16_t len)
{
  // FIELD 1
  char * split = loc_strtok_single(str, ",");
  // FIELD 2: ddmm.mmm
  split = loc_strtok_single(NULL, ",");
  uint32_t deg;
  float min;
  loc_strToLat(split, &deg, &min);
  g_furuno_gps.position.latitude_deg = deg;
  g_furuno_gps.position.latitude_min = min;
  g_furuno_gps.position.has_latitude_deg = true;
  g_furuno_gps.position.has_latitude_min = true;
  // FIELD 3: north/south
  split = loc_strtok_single(NULL, ",");
  if (*split == 'N')
  {
    g_furuno_gps.position.lat_sign = FurunoGpsPosition_FurunoGpsPositionSign_N;
  }
  else
  {
    g_furuno_gps.position.lat_sign = FurunoGpsPosition_FurunoGpsPositionSign_S;
  }
  g_furuno_gps.position.has_lat_sign = true;
  // FIELD 4: dddmm.mmm
  split = loc_strtok_single(NULL, ",");
  loc_strToLong(split, &deg, &min);
  g_furuno_gps.position.longitude_deg = deg;
  g_furuno_gps.position.longitude_min = min;
  g_furuno_gps.position.has_longitude_deg = true;
  g_furuno_gps.position.has_longitude_min = true;
  // FIELD 5: east/west
  split = loc_strtok_single(NULL, ",");
  if (*split == 'E')
  {
    g_furuno_gps.position.long_sign = FurunoGpsPosition_FurunoGpsPositionSign_E;
  }
  else
  {
    g_furuno_gps.position.long_sign = FurunoGpsPosition_FurunoGpsPositionSign_W;
  }
  g_furuno_gps.position.has_long_sign = true;
  // FIELD 6
  split = loc_strtok_single(NULL, ",");
  // FIELD 7: Number of satelites
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.info.num_of_sat = atoi(split);
  g_furuno_gps.info.has_num_of_sat = true;
  g_furuno_gps.has_info = true;
  // FIELD 8
  split = loc_strtok_single(NULL, ",");
  // FIELD 9: Altitude above sea
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.position.alt_over_sea = strtof(split, NULL);
  g_furuno_gps.position.has_alt_over_sea = true;
  // FIELD 10: Altitude geoidal
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.position.alt_geoidal = strtof(split, NULL);
  g_furuno_gps.position.has_alt_geoidal = true;
  g_furuno_gps.has_position = true;
}

//!< Parse information from PERDCRZ line (further info in: FURUNO GNSSDO-Protocol Specs)
static void loc_parsePerdcr (char *str, uint16_t len)
{
  // FIELD 1
  char * split = loc_strtok_single(str, ",");
  // FIELD 2: freq_mode
  split = loc_strtok_single(NULL, ",");
  uint8_t f_mode = split[0] - '0' + 1;
  switch (f_mode) {
    case 1:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_WARM_UP;
      break;
    case 2:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_PULL_IN;
      break;
    case 3:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_COARSE_LOCK;
      break;
    case 4:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_FINE_LOCK;
      break;
    case 5:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_HOLD_OVER;
      break;
    case 6:
      g_furuno_gps.info.freq_mode = FurunoGpsInfo_FurunoGpsFreqMode_OUT_OF_HOLD_OVER;
      break;
  }
  g_furuno_gps.info.has_freq_mode = true;
  // FIELD 3:
  split = loc_strtok_single(NULL, ",");
  // FIELD 4:
  split = loc_strtok_single(NULL, ",");
  // FIELD 5:
  split = loc_strtok_single(NULL, ",");
  // FIELD 6: pps_error
  split = loc_strtok_single(NULL, ",");
  g_furuno_gps.info.pps_error_nsec = atoi(split+1);
  g_furuno_gps.info.has_pps_error_nsec = true;
  g_furuno_gps.has_info = true;
}

//! Sends a byte-string to Furuno (calculates len and linebreaks)
static bool loc_cmd_to_uart(char * command) {
  bool b_return = false;
  uint8_t len = 0;
  while(command[len] != 0) len++;
  uint8_t i;
  for (i = 0; i < len; i++ ) {
    g_recv_buffer[i] = command[i];
  }
  g_recv_buffer[len+1] = 0x0D;
  g_recv_buffer[len+2] = 0x0A;
  uint16_t i_ret = hal_uart_write(g_gf8701_ctx, (uint8_t*) g_recv_buffer, len+2);

  b_return = true;
  return b_return;
}

//! CRC calculation
static char loc_calcCrc(char *data, uint16_t len)
{
  char ret = 0;
  uint16_t i = 0U;

  for (i = 0; i < len; ++i)
  {
    ret = (ret ^ data[i]);
  }

  return ret;
}

/* ===========================================================================*/
/*                   global functions implementation                          */
/* ===========================================================================*/

bool satellady_init() {
  bool b_return = true;

  g_gf8701_ctx = hal_uart_init(E_HAL_UART_PORT_1, 38400);
  if(g_gf8701_ctx == NULL)
  {
    b_return = false;
  }
  b_return = loc_cmd_to_uart("$PERDAPI,PPS,VCLK,1,0,1,0,0*06");

  g_satelady_state = E_SATELADY_STATE_FIND_START_OF_LINE;

  memset(g_recv_buffer, 0U, sizeof(g_recv_buffer));
  g_already_read = 0U;
  memset(g_recvCrc, 0U, sizeof(g_recvCrc));
  /* Set string NULL termination here for processing of strtol() in run(). */
  g_recvCrc[2] = '\0';
  g_crcByte = 0U;
  g_crcCalculated = 0U;

  g_furuno_gps = g_furuno_gps_default;
  gps_data_avail = false;
  g_isInitialized = false;

  /* Init our debug counters. */
  g_dbgCountLines = 0U;
  g_dbgCountError = 0U;
  g_dbgCountWrongStart = 0U;

  /* Starting testing your parsing functions here! */
  //loc_testParsing();

  return b_return;
}

void satellady_run() {
  char c_tmp = 0U;
  switch (g_satelady_state)
  {
    case E_SATELADY_STATE_FIND_START_OF_LINE:
      /* Read single character wise and proceed if we find the start of line. */
      if( hal_uart_read(g_gf8701_ctx, (uint8_t *) &c_tmp, 1U) )
      {
        if( c_tmp == '$' )
        {
          memset(g_recv_buffer, 0U, sizeof(g_recv_buffer));
          memset(g_recvCrc, 0U, sizeof(g_recvCrc));
          g_already_read = 0U;
          g_satelady_state = E_SATELADY_STATE_GET_DATA;
        }
        else
        {
          g_dbgCountWrongStart++;
        }
      }
      break;

    case E_SATELADY_STATE_GET_DATA:
      /* Loop as long as there is data available and no END_OF_LINE detected. */
      while( hal_uart_read(g_gf8701_ctx, (uint8_t *)(g_recv_buffer + g_already_read), 1U) )
      {
        if( g_recv_buffer[g_already_read] == '*')
        {
          /* END_OF_LINE detected. Overwrite our EOL character by
           * NULL-terminator for the string parsing functions later on.
           * Go to next state to get the CRC.
           */
          g_recv_buffer[g_already_read] = '\0';
          g_satelady_state = E_SATELADY_STATE_GET_CRC_BYTE1;
          g_already_read++;
          break;
        }
        else if(g_already_read > SAT_BUFFER_LEN)
        {
          /* We are running out of buffers! */
          g_satelady_state = E_SATELADY_STATE_ERROR;
          break;
        }
        else
        {
          g_already_read++;
        }
      }
      break;

    case E_SATELADY_STATE_GET_CRC_BYTE1:
      /* Get first byte of two byte CRC. */
      if( hal_uart_read(g_gf8701_ctx, &g_recvCrc[0], 1U) )
      {
        g_satelady_state = E_SATELADY_STATE_GET_CRC_BYTE2;
      }
      break;

    case E_SATELADY_STATE_GET_CRC_BYTE2:
      if( hal_uart_read(g_gf8701_ctx, &g_recvCrc[1], 1U) )
      {
        /* Validate CRC value here! */
        char *tmp;

        g_crcByte = strtol( (char *)&g_recvCrc[0], &tmp, 16);
        g_crcCalculated = loc_calcCrc( g_recv_buffer, g_already_read);

        if(g_crcCalculated == (char)g_crcByte)
        {
          /* CRC valid. Go on and process the line. */
          g_satelady_state = E_SATELADY_STATE_FIND_END_OF_LINE_BYTE1;
        }
        else
        {
          g_satelady_state = E_SATELADY_STATE_ERROR;
        }
      }
      break;

    case E_SATELADY_STATE_FIND_END_OF_LINE_BYTE1:
      if( hal_uart_read(g_gf8701_ctx, (uint8_t *) &c_tmp, 1U) )
      {
        if(c_tmp == '\r')
        {
          /* We found the complete line. Now parse it and start with new line. */
          g_satelady_state = E_SATELADY_STATE_FIND_END_OF_LINE_BYTE2;

        }
        else
        {
          g_satelady_state = E_SATELADY_STATE_ERROR;
        }
      }
      break;

    case E_SATELADY_STATE_FIND_END_OF_LINE_BYTE2:
      if( hal_uart_read(g_gf8701_ctx, (uint8_t *) &c_tmp, 1U) )
      {
        if(c_tmp == '\n')
        {
          /* We found the complete line. Now parse it and start with new line. */
          g_dbgCountLines++;

          if(!g_isInitialized)
          {
            loc_checkInit(g_recv_buffer, g_already_read);
          }
          else
          {
            loc_parseLine(g_recv_buffer, g_already_read);
          }

          g_satelady_state = E_SATELADY_STATE_FIND_START_OF_LINE;
        }
        else
        {
          g_satelady_state = E_SATELADY_STATE_ERROR;
        }
      }
      break;

    case E_SATELADY_STATE_ERROR:
      g_dbgCountError++;
      g_satelady_state = E_SATELADY_STATE_FIND_START_OF_LINE;
      break;
  }
}

bool satellady_get(FurunoGps* const p_gps_msg) {
  bool b_return = false;

  if(p_gps_msg && gps_data_avail)
  {
    gps_data_avail = false;
    *p_gps_msg = g_furuno_gps;
    g_furuno_gps = g_furuno_gps_default;
    b_return = true;
  }
  return b_return;
}

bool satellady_setConf() {
  return false;
}
