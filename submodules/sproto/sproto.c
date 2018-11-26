/*
 * serial_protocol.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "sproto.h"

//*****************************************************************************
//
// Start doxygen group
//! \addtogroup utils_sproto
//! @{
//
//*****************************************************************************

/*! Defines the SYNC bytes transmitted in the header of the serial protocol frame. */
static const uint8_t g_syncField[] = { 0xDE, 0xAD, 0xFC, 0xA5, 0xA5 };

/*! Defines the length of the SYNC_FIELD.
 *  \warning Must be in sync with array definition \ref g_syncField !
 */
#define SPROTO_SYNC_FIELD_LEN 5

/*! Defines the length of the LENGTH_FIELD of a serial protocol frame. */
#define SPROTO_LENGTH_FIELD_LEN 2

/*! Defines the total length of the header of a serial protocol frame. */
#define SPROTO_HEADER_LEN (SPROTO_SYNC_FIELD_LEN + SPROTO_LENGTH_FIELD_LEN)

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

/* ===========================================================================*/
/*                   Local function prototypes                                */
/* ===========================================================================*/
static bool loc_handleSyncFieldStates(s_sproto_ctx_t *ps_ctx, uint8_t syncFieldIndex,
                                      E_SERIAL_PROTOCOL_STATE_RX_t e_nextState);
static bool loc_handleRxStateData(s_sproto_ctx_t *ps_ctx);
static bool loc_txStart(s_sproto_ctx_t *ps_ctx, uint16_t ui16_len);
static bool loc_txWrite(s_sproto_ctx_t *ps_ctx, const uint8_t *pc_data, uint16_t ui16_len);

/* ===========================================================================*/
/*                  Local function implementations                            */
/* ===========================================================================*/

static bool loc_handleSyncFieldStates(s_sproto_ctx_t *ps_ctx, uint8_t syncFieldIndex,
                                      E_SERIAL_PROTOCOL_STATE_RX_t e_nextState)
{
  bool b_return = false;

  if(ps_ctx->pfn_read(ps_ctx->ac_tmpBuf, 1) == 1)
  {
    if(ps_ctx->ac_tmpBuf[0] == g_syncField[syncFieldIndex])
    {
      ps_ctx->e_rxState = e_nextState;
      b_return = true;
    }
    else
    {
      /* Resets the RX state machine to wait for the first SYNC byte. */
      ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1;
      b_return = true; /* Keep the state machine running */
    }
  }

  return b_return;
}

static bool loc_handleRxStateData(s_sproto_ctx_t *ps_ctx)
{
  bool b_return = false;
  uint16_t u16_received = 0;
  do
  {
    u16_received = ps_ctx->pfn_read((ps_ctx->pc_buf + ps_ctx->ui16_bytesReceived),
                                    (ps_ctx->ui16_bytesToReceive - ps_ctx->ui16_bytesReceived));

    ps_ctx->ui16_bytesReceived += u16_received;
  } while(u16_received);

  if(ps_ctx->ui16_bytesReceived >= ps_ctx->ui16_bytesToReceive)
  {
    ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE1;
    b_return = true;
  }

  return b_return;
}

static bool loc_txStart(s_sproto_ctx_t *ps_ctx, uint16_t ui16_len)
{
  bool b_return = false;
  uint8_t ac_serialHeader[SPROTO_SYNC_FIELD_LEN + SPROTO_LENGTH_FIELD_LEN] = { 0 };

  if(ps_ctx)
  {
    if(ps_ctx->e_txState == E_SERIAL_PROTOCOL_STATE_TX_START)
    {
      /* First bytes: SYNC_FIELD */
      memcpy(ac_serialHeader, g_syncField, SPROTO_SYNC_FIELD_LEN);

      /* MSB of the two length bytes. */
      ac_serialHeader[SPROTO_SYNC_FIELD_LEN] = (uint8_t)((ui16_len & 0xFF00) >> 8);
      /* LSB of the two length bytes. */
      ac_serialHeader[SPROTO_SYNC_FIELD_LEN + 1] = (uint8_t)(ui16_len & 0x00FF);

      /* Finaly flush the serial protocol header through the UART port. */
      if(ps_ctx->pfn_write(ac_serialHeader, SPROTO_HEADER_LEN) == SPROTO_HEADER_LEN)
      {
        /* Continue adding the payload data to the serial frame. */
        ps_ctx->e_txState = E_SERIAL_PROTOCOL_STATE_TX_WRITE;
        ps_ctx->ui16_bytesToWrite = ui16_len;
        ps_ctx->ui16_bytesWritten = 0;

        b_return = true;
      }
    } /* if() */
  } /* if() */

  return b_return;
}

static bool loc_txWrite(s_sproto_ctx_t *ps_ctx, const uint8_t *pc_data, uint16_t ui16_len)
{
  bool b_return = false;
  uint8_t ac_serialCRC[2] = { 0 };
  uint16_t crc_result = 0;

  if(ps_ctx)
  {
    if(ps_ctx->e_txState == E_SERIAL_PROTOCOL_STATE_TX_WRITE)
    {
      ps_ctx->ui16_bytesWritten += ps_ctx->pfn_write(pc_data, ui16_len);

      if(ps_ctx->ui16_bytesWritten >= ps_ctx->ui16_bytesToWrite)
      {
        /* We transmitted all payload data through the callback.
         * So finalize the serial frame with the CRC. */
        crc_result = ps_ctx->pfn_crc(pc_data, ui16_len);

        /* MSB of the two CRC bytes. MSB first! */
        ac_serialCRC[0] = ((uint8_t)(crc_result >> 8));
        /* LSB of the two CRC bytes. */
        ac_serialCRC[1] = ((uint8_t)crc_result & 0x00FF);

        ps_ctx->pfn_write(ac_serialCRC, 2);
      }

      /* Reset internal state machine afterwards. */
      ps_ctx->e_txState = E_SERIAL_PROTOCOL_STATE_TX_START;

      b_return = true;
    }
  }

  return b_return;
}

/* ===========================================================================*/
/*                    API function implementations                            */
/* ===========================================================================*/

bool sproto_init(s_sproto_ctx_t *ps_ctx, pfn_write pfn_wr, pfn_read pfn_rd, pfn_rxCallback pfn_cb, pfn_error pfn_err,
                 pfn_crcCalc pfn_crc, uint8_t *pc_buf, uint16_t len)
{
  bool b_return = false;

  if(ps_ctx && pfn_wr && pfn_rd && pfn_cb && pfn_err && pfn_crc)
  {
    memset((void *)ps_ctx, 0, sizeof(s_sproto_ctx_t));

    ps_ctx->e_txState = E_SERIAL_PROTOCOL_STATE_TX_START;
    ps_ctx->ui16_bytesToWrite = 0;
    ps_ctx->ui16_bytesWritten = 0;

    ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1;
    ps_ctx->ui16_bytesToReceive = 0;
    ps_ctx->ui16_bytesReceived = 0;

    memset((void *)pc_buf, 0, len);
    ps_ctx->pc_buf = pc_buf;
    ps_ctx->len = len;

    ps_ctx->pfn_rxCb = pfn_cb;
    ps_ctx->pfn_read = pfn_rd;
    ps_ctx->pfn_write = pfn_wr;
    ps_ctx->pfn_error = pfn_err;
    /* IMPORTANT!
     * In case of HW CRC being used, make sure that the HW (= HAL_CRC for FC)
     * module is initialized prior to call this init function! The serial
     * protocol itself expects the HW CRC to be ready to use and simply calls
     * this callback for CRC calculation. */
    ps_ctx->pfn_crc = pfn_crc;

    /* Initialization is done successfully. */
    b_return = true;
  } /* if() */

  return b_return;
}

bool sproto_sendFrame(s_sproto_ctx_t *ps_ctx, const uint8_t *pc_data, uint16_t ui16_len)
{
  bool b_return = false;

  if(loc_txStart(ps_ctx, ui16_len))
  {
    if(loc_txWrite(ps_ctx, pc_data, ui16_len))
    {
      b_return = true;
    }
  }

  return b_return;
}

bool sproto_run(s_sproto_ctx_t *ps_ctx)
{
  bool b_return = false;
  uint16_t crc_result = 0;
  uint16_t crc_resultRcvd = 0;

  /* Check for RX bytes. */
  switch(ps_ctx->e_rxState)
  {
    case E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1:
      b_return = loc_handleSyncFieldStates(ps_ctx, 0, E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE2);
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE2:
      b_return = loc_handleSyncFieldStates(ps_ctx, 1, E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE3);
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE3:
      b_return = loc_handleSyncFieldStates(ps_ctx, 2, E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE4);
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE4:
      b_return = loc_handleSyncFieldStates(ps_ctx, 3, E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE5);
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE5:
      b_return = loc_handleSyncFieldStates(ps_ctx, 4, E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE1);
      if(b_return)
      {
        /* Reset bytes counter before next state(s). */
        ps_ctx->ui16_bytesReceived = 0;
        ps_ctx->ui16_bytesToReceive = 0;
      }
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE1:
      if(ps_ctx->pfn_read(ps_ctx->ac_tmpBuf, 1) == 1)
      {
        ps_ctx->ui16_bytesToReceive = ps_ctx->ac_tmpBuf[0];
        ps_ctx->ui16_bytesToReceive = (ps_ctx->ui16_bytesToReceive << 8);
        ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE2;
        b_return = true;
      }
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE2:
      if(ps_ctx->pfn_read(ps_ctx->ac_tmpBuf, 1) == 1)
      {
        ps_ctx->ui16_bytesToReceive += ps_ctx->ac_tmpBuf[0];

        if(ps_ctx->ui16_bytesToReceive <= ps_ctx->len)
        {
          ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_DATA;
        }
        else
        {
          /* ERROR: We have a problem! The serial protocol frame is larger than
           * our buffer provided by the user! Give a signal to the user! */
          ps_ctx->pfn_error(E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW);
          ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1;
        }
        b_return = true;
      }
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_DATA:
      b_return = loc_handleRxStateData(ps_ctx);
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE1:
      if(ps_ctx->pfn_read(ps_ctx->ac_tmpBuf, 1) == 1)
      {
        ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE2;
        b_return = true;
      }
      break;

    case E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE2:
      /* When ending up here, we must receive the serial protocol footer (CRC) */
      if(ps_ctx->pfn_read(&ps_ctx->ac_tmpBuf[1], 1) == 1)
      {
        /* ac_tmp now holds the two CRC bytes. Time to calculate the CRC. */
        crc_result = ps_ctx->pfn_crc(ps_ctx->pc_buf, ps_ctx->ui16_bytesReceived);

        /* MSB transmitted first! */
        crc_resultRcvd = (ps_ctx->ac_tmpBuf[0]);
        crc_resultRcvd = (crc_resultRcvd << 8);
        crc_resultRcvd += (ps_ctx->ac_tmpBuf[1]);

        /* Check if received CRC matches to our calculation. */
        if(crc_result == crc_resultRcvd)
        {
          /* Call the callback of the user to signal new RX data. */
          ps_ctx->pfn_rxCb(ps_ctx->pc_buf, ps_ctx->ui16_bytesReceived);
        }
        else
        {
          /* Error: Invalid CRC received! */
          ps_ctx->pfn_error(E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID);
        }

        /* Reset state machine. */
        ps_ctx->e_rxState = E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1;
      }
      break;

    default:
      break;
  }

  return b_return;
}
