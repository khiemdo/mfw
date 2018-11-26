/*
 * serial_protocol.h
 */

#ifndef SERIAL_PROTOCOL_H_
#define SERIAL_PROTOCOL_H_

//*****************************************************************************
//
//! \example test-app-sproto.c
//!
//! \addtogroup utils Utility modules
//! @{
//!
//! \addtogroup utils_sproto Serial Protocol module
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

/*! States of the internal transmission state machine. */
typedef enum
{
  /*! State: Start transmission of a serial protocol frame. Main task is
   *  to send the serial protocol header, that is SYNC byte and the two len field. */
  E_SERIAL_PROTOCOL_STATE_TX_START,

  /*! State: Writes data payload to the payload body of the serial protocol frame. */
  E_SERIAL_PROTOCOL_STATE_TX_WRITE,

  /*! State: Finalizes the serial protocol frame by sending the CRC. */
  E_SERIAL_PROTOCOL_STATE_TX_FINALIZE

} E_SERIAL_PROTOCOL_STATE_TX_t;

/*! States of the internal reception state machine. */
typedef enum
{
  /*! State: Wait for reception of first SYNC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE1,

  /*! State: Wait for reception of 2nd SYNC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE2,

  /*! State: Wait for reception of 3rd SYNC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE3,

  /*! State: Wait for reception of 4th SYNC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE4,

  /*! State: Wait for reception of 5th SYNC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_SYNC_BYTE5,

  /*! State: Wait for reception of first length byte. */
  E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE1,

  /*! State: Wait for reception of second length byte. */
  E_SERIAL_PROTOCOL_STATE_RX_LENGTH_BYTE2,

  /*! State: Wait for reception of data payload. */
  E_SERIAL_PROTOCOL_STATE_RX_DATA,

  /*! State: Wait for reception of first CRC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE1,

  /*! State: Wait for reception of second CRC byte. */
  E_SERIAL_PROTOCOL_STATE_RX_CRC_BYTE2

} E_SERIAL_PROTOCOL_STATE_RX_t;

/*! Different types of errors that might occur. Used by \ref pfn_error. */
typedef enum
{
  /*! Signals that a serial frame was received with invalid CRC. */
  E_SERIAL_PROTOCOL_ERROR_RX_CRC_INVALID,

  /*! Signals that a serial frame was received that exceeds the length of the
   *   provided serial protocol frame buffer, i.e. the caller should provide
   *   a larger buffer when calling \ref sproto_init().
   *   If this error event is raised the internal state machine is reset to
   *   wait for a new serial frame SYNC.
   */
  E_SERIAL_PROTOCOL_ERROR_RX_BUFFER_OVERFLOW

} E_SERIAL_PROTOCOL_ERROR_t;

/*! \brief Typedef of function pointer used as callback to signal an RX event to
 *         the application.
 *
 * \details This typedef is as used parameter of the function \ref sproto_init().
 *
 *          The registered callback gets called in case a complete serial
 *          protocol frame is received, i.e. the CRC check at the end of the
 *          frame is passed. It provides a pointer to received data payload.
 *
 * \warning This callback is called only \b once per received frame. When the
 *          callback returns, the serial protocol module might overwrite the
 *          RX data buffer the next time the function \ref sproto_run() is called.
 *
 * \warning The current version of the serial protocol does not calculate the
 *          CRC yet! Instead, payload "0x5A 0x5A" is transmitted as dummy CRC
 *          bytes. For received frames, the CRC is not validated and any complete
 *          telegram is passed to the application.
 *
 * \param pc_data   Pointer to the start of memory where serial protocol payload
 *                  is stored.
 * \param ui16_len  Length of the memory where payload is stored.
 *
 * \return None.
 */
typedef void (*pfn_rxCallback)(uint8_t *pc_data, uint16_t ui16_len);

/*! \brief Typedef of function pointer used as callback to write the serial
 *         protocol frame to.
 *
 * \details This typedef is as used parameter of the function \ref sproto_init().
 *
 *          The registered callback gets called to pass the serial protocol
 *          frame (as a serial byte stream) to the application.
 *          Feel free to do what ever you want with your data, as long as your
 *          peer serial protocol instance will receive it.
 *          This function is called several times for a single frame.
 *
 * \param pc_data   Pointer to the memory to write.
 * \param ui16_len  Length of the memory to write.
 *
 * \return Number of bytes that are actually written.
 */
typedef uint16_t (*pfn_write)(const uint8_t *pc_data, uint16_t ui16_len);

/*! \brief Typedef of function pointer used as callback to read the serial
 *         protocol frame from.
 *
 * \details This typedef is as used parameter of the function \ref sproto_init().
 *
 *          The registered callback gets called to read the serial protocol
 *          byte stream from the application.
 *          This function is called several times for a single frame.
 *
 * \param pc_data   Pointer to the memory to read/store new data to.
 * \param ui16_len  Length of the memory to read.
 *
 * \return Number of bytes that are actually read.
 */
typedef uint16_t (*pfn_read)(uint8_t *pc_data, uint16_t ui16_len);

/*! \brief Typedef of function pointer used as callback to signal an error.
 *
 * \details This typedef is as used parameter of the function \ref sproto_init().
 *
 *          The registered callback gets called to indicate that an error
 *          occured. The specific type of error is determined by the argument
 *          of type \ref E_SERIAL_PROTOCOL_ERROR_t.
 *
 * \param e_errType  Type of error that occured.
 *
 * \return  None.
 */
typedef void (*pfn_error)(E_SERIAL_PROTOCOL_ERROR_t e_errType);

/*! \brief Typedef of function pointer used as callback to calculate CRC-CCITT.
 *
 * \details This typedef is as used parameter of the function \ref sproto_init().
 *
 *          The registered callback gets called to calculate the CRC of a given
 *          message.
 *
 * \param pc_data   Pointer to the memory to the data to calculate the CRC for.
 * \param ui16_len  Length of the memory.
 *
 * \return  CRC result of the given message.
 */
typedef uint16_t (*pfn_crcCalc)(const uint8_t *pc_data, uint16_t ui16_len);

/*! Context of the serial protocol module that can be seen as applications
 *  "instance" of the module. */
typedef struct S_SERIAL_PROTOCOL_CTX_T
{
  /*! Current TX state of TX state machine. */
  E_SERIAL_PROTOCOL_STATE_TX_t e_txState;
  /*! Remaining bytes to send. */
  uint16_t ui16_bytesToWrite;
  /*! Already transmitted bytes. */
  uint16_t ui16_bytesWritten;

  /*! Current RX state of RX state machine. */
  E_SERIAL_PROTOCOL_STATE_RX_t e_rxState;
  /*! Remaining bytes to receive. */
  uint16_t ui16_bytesToReceive;
  /*! Already received bytes. */
  uint16_t ui16_bytesReceived;

  /*! Pointer to the memory that holds the serial protocol payload body. */
  uint8_t *pc_buf;
  /*! Length of the memory that holds the serial protocol payload body. */
  uint16_t len;
  /*! Temporary buffer. */
  uint8_t ac_tmpBuf[2];

  /*! Pointer to the RX event callback function. */
  pfn_rxCallback pfn_rxCb;
  /*! Pointer to the write callback function. */
  pfn_write pfn_write;
  /*! Pointer to the read callback function. */
  pfn_read pfn_read;
  /*! Pointer to the error callback function. */
  pfn_error pfn_error;
  /*! Pointer to the CRC calculation callback function. */
  pfn_crcCalc pfn_crc;

} s_sproto_ctx_t;

/*! \brief Initializes the instance of the serial protocol module and provides
 *         the required callbacks of the module.
 *
 * \details Call this function prior to any other function of this module.
 *
 * \param ps_ctx  Pointer to the context to be initialized.
 * \param pfn_wr  Pointer to the write callback function.
 * \param pfn_rd  Pointer to the read callback function.
 * \param pfn_cb  Pointer to the RX event callback function.
 * \param pfn_err Pointer to the error event callback function.
 * \param pfn_crc Pointer to the CRC calculation callback function.
 * \param pc_buf  Pointer to the memory buffer that is internally used RAM.
 *                In fact it is used to store/buffer the serial protocol frame.
 * \param len     Length of the provided buffer memory.
 *
 * \return TRUE in case the module is initialized successfully. FALSE otherwise.
 */
bool sproto_init(s_sproto_ctx_t *ps_ctx, pfn_write pfn_wr, pfn_read pfn_rd, pfn_rxCallback pfn_cb, pfn_error pfn_err,
                 pfn_crcCalc pfn_crc, uint8_t *pc_buf, uint16_t len);

/*! \brief Sends a complete serial protocol frame in a single call.
 *
 * \details By a single call to this function a complete serial protocol frame
 *          is transmitted. In other words, the caller must provide the complete
 *          user data to be transmitted in a single call.
 *          A full serial protocol frame is structured as follows:
 *
 * \code
 * +-----------------------------------------------------------------------------+
 * |        |          |          |                        |          |          |
 * |  SYNC  |  LENGTH  |  LENGTH  |       USER DATA        |  CRC-16  |  CRC-16  |
 * |        |          |          |                        |          |          |
 * | 5bytes |  (MSB)   |  (LSB)   |  (1 ... 0xFFFF bytes)  |  (MSB)   |  (LSB)   |
 * |        |          |          |                        |          |          |
 * +-----------------------------------------------------------------------------+
 * \endcode
 *
 * \param ps_ctx    Pointer to the context.
 * \param pc_data   Pointer to the data payload to write to the frame.
 * \param ui16_len  Length of the serial protocol frame body.
 *
 * \return TRUE if frame transmission is started successfully. FALSE otherwise.
 */
bool sproto_sendFrame(s_sproto_ctx_t *ps_ctx, const uint8_t *pc_data, uint16_t ui16_len);

/*! \brief  Function to let the module do its work. Call this function as often
 *          as possible!
 *
 * \details In fact, this function handles the internal RX state machine. Due to
 *          that it is important to call this function in a higher frequency!
 *          This ensures proper RX data handling and avoids RX data loss.
 *
 * \param ps_ctx  Pointer to the context.
 *
 * \return TRUE in case some data of a serial frame was received to indicate
 *         that its worth it to call this function again. This is because when
 *         a first byte of a serial frame is received the next bytes will
 *         follow with high probability. Otherwise returns FALSE.
 */
bool sproto_run(s_sproto_ctx_t *ps_ctx);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* SERIAL_PROTOCOL_H_ */
