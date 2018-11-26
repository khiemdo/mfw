/* Copyright (C) 2015-2017
 *
 * hal_spi.h
 *
 * Martin Dold         <martin.dold@gmx.net>
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

#ifndef HAL_HAL_SPI_H_
#define HAL_HAL_SPI_H_

//*****************************************************************************
//
//! \example test-app-spi.c
//! \addtogroup hal HAL API
//! @{
//!
//! \addtogroup hal_spi_api HAL SPI API
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


/*! Available SPI ports provided and handled by the SPI module. */
typedef enum
{
  /*! \b Port \b 0: MOSI = PA_4, MISO = PA_5, CLK = PA_2.
   *
   *  \attention This port uses DMA channels 10 (RX) and 11 (TX) for
   *             communication. Make sure, this does not conflict with other
   *             DMA configurations.
   */
  E_HAL_SPI_PORT_0,

  /*! \b Port \b 1: MOSI = PE_4, MISO = PE_5, CLK = PB_5.
   *
   *  \attention This port uses DMA channels 24 (RX) and 25 (TX) for
   *             communication. Make sure, this does not conflict with other
   *             DMA configurations.
   */
  E_HAL_SPI_PORT_1,

  /*! \b Port \b 2: MOSI = PD_1, MISO = PD_0, CLK = PD_3.
   *
   *  \attention This port uses DMA channels 12 (RX) and 13 (TX) for
   *             communication. Make sure, this does not conflict with other
   *             DMA configurations.
   */
  E_HAL_SPI_PORT_2,

  /*! \b Port \b 3: MOSI = PF_1, MISO = PF_0, CLK = PF_3.
   *
   *  \attention This port uses DMA channels 14 (RX) and 15 (TX) for
   *             communication. Make sure, this does not conflict with other
   *             DMA configurations.
   */
  E_HAL_SPI_PORT_3

} E_HAL_SPI_PORT_t;

/*! Available SPI modes provided and handled by the SPI module. */
typedef enum
{
  /*! Specifies \b SPI \b master mode. */
  E_HAL_SPI_MODE_MASTER,

  /*! Specifies \b SPI \b slave mode. \attention Currently not supported! */
  E_HAL_SPI_MODE_SLAVE
} E_HAL_SPI_MODE_t ;


/*! Available SPI clock polarities provided and handled by the SPI module. */
typedef enum
{
  /*! SPI clock polarity: \b LOW. */
  E_HAL_SPI_CLK_POL_LOW,

  /*! SPI clock polarity: \b HIGH. */
  E_HAL_SPI_CLK_POL_HIGH
} E_HAL_SPI_CLK_POL_t;


/*! Available SPI clock phases provided and handled by the SPI module. */
typedef enum
{

  /*! SPI clock phase: \b first edge. */
  E_HAL_SPI_CLK_PHA_EDGE_FIRST,

  /*! SPI clock phase: \b second edge. */
  E_HAL_SPI_CLK_PHA_EDGE_SCND
} E_HAL_SPI_CLK_PHA_t;


/*! Typedef of function pointer used as callback by \ref hal_spi_xfer. */
typedef void (*pfn_spi_callback)(void);


/*! Typedef of structure defining the context of a SPI port. */
typedef struct S_HAL_SPI_CTX_T s_hal_spi_ctx_t;


/*!
 * \brief Initializes a SPI port of the microcontroller.
 *
 * \details A call to this function it is required prior to any SPI operations!
 *
 * \attention This function \b does \b not initialize any chip select pins. It is
 *            up to the user to initialize and handle the chip select signals to
 *            the connected SPI slave(s) before starting the SPI transfer.
 *            Furthermore, this SPI driver uses DMA transfers and thereby blocks
 *            the corresponding DMA channels for the SPI ports (see
 *            \ref E_HAL_SPI_PORT_t). Make sure the DMA controller is
 *            initialized before calling this function.
 *
 * \param  e_port         SPI port to be initiliazed.
 * \param  e_mode         SPI mode to be used.
 * \param  ui32_spiClock  SPI clock speed to be used in Hz.
 *
 * \return Returns pointer to context variable if SPI port is initialized
 *         successfully. NULL otherwise.
 */
s_hal_spi_ctx_t *hal_spi_init(E_HAL_SPI_PORT_t e_port, E_HAL_SPI_MODE_t e_mode, uint32_t ui32_spiClock);


/*!
 * \brief Configures a pre-initialized SPI port of the microcontroller.
 *
 * \details Prior to call to this function it is required initialize the SPI port
 *          first! The context pointer returned by \ref hal_spi_init must be
 *          given as parameter here.
 *
 * \param  ps_ctx         Pointer to the SPI port context (previously returned by
 *                        \ref hal_spi_init).
 * \param  e_pol          SPI clock polarity to be used.
 * \param  e_phase        SPI clock phase to be used.
 * \param  ui32_spiClock  SPI clock speed to be used in MHz.
 *
 * \return TRUE if configured successfully. FALSE otherwise.
 */
bool hal_spi_config(s_hal_spi_ctx_t *ps_ctx, E_HAL_SPI_CLK_POL_t e_pol, E_HAL_SPI_CLK_PHA_t e_phase, uint32_t ui32_spiClock);


/*!
 * \brief Starts a SPI transfer in a non-blocking manner.
 *
 * \details Prior to call to this function it is required initialize the SPI port
 *          first! The context pointer returned by \ref hal_spi_init must be
 *          given as parameter here.
 *          This function starts a non-blocking SPI transfer, that is, the
 *          function will return to the caller right after initial processing.
 *          The specified callback function is called when the SPI transfer is
 *          completed.
 *
 * \attention This function \b does \b not handle any chip select pins. It is
 *            up to the user to initialize and handle the chip select signals to
 *            the connected SPI slave(s) before starting the SPI transfer.
 *
 * \param  ps_ctx     Pointer to the SPI port context (previously returned by
 *                    \ref hal_spi_init).
 *
 * \param  pc_txData  \b Optional: Pointer to the data to be transmitted.
 *                    This value might be NULL if no data shall be transmitted
 *                    but only data reception is required. If set to NULL
 *                    the SPI driver will transmit dummy bytes instead to
 *                    allow data reception from the SPI slave.
 *
 * \param  pc_rxData  \b Optional: Pointer to memory to store received data.
 *                    This value might be NULL if no data shall be received
 *                    but only data transmission is required. If set to NULL
 *                    the SPI driver will discard any received bytes.
 *
 * \param  ui16_len   Length of the given data memories. Make sure the number
 *                    matches the size of TX \b and RX data memory if both
 *                    options are used.
 *
 * \param  pf_cb      Function pointer that is called when SPI transfer is completed.
 *
 * \return None.
 */
void hal_spi_xfer(s_hal_spi_ctx_t *ps_ctx, uint8_t *pc_txData, uint8_t *pc_rxData, uint16_t ui16_len, pfn_spi_callback pf_cb);

/*!
 * \brief Starts a SPI transfer in a blocking manner.
 *
 * \details Prior to call to this function it is required initialize the SPI port
 *          first! The context pointer returned by \ref hal_spi_init must be
 *          given as parameter here.
 *          This function starts a blocking SPI transfer, that is, the
 *          function will not return immediately but instead returns when the
 *          SPI transfer is completed.
 *          This function serves as counter part to the non-blocking function
 *          \ref hal_spi_xfer.
 *
 * \attention This function \b does \b not handle any chip select pins. It is
 *            up to the user to initialize and handle the chip select signals to
 *            the connected SPI slave(s) before starting the SPI transfer.
 *
 * \param  ps_ctx     Pointer to the SPI port context (previously returned by
 *                    \ref hal_spi_init).
 *
 * \param  pc_txData  \b Optional: Pointer to the data to be transmitted.
 *                    This value might be NULL if no data shall be transmitted
 *                    but only data reception is required. If set to NULL
 *                    the SPI driver will transmit dummy bytes instead to
 *                    allow data reception from the SPI slave.
 *
 * \param  pc_rxData  \b Optional: Pointer to memory to store received data.
 *                    This value might be NULL if no data shall be received
 *                    but only data transmission is required. If set to NULL
 *                    the SPI driver will discard any received bytes.
 *
 * \param  ui16_len   Length of the given data memories. Make sure the number
 *                    matches the size of TX \b and RX data memory if both
 *                    options are used.
 *
 * \return None.
 */
void hal_spi_xferBlocking(s_hal_spi_ctx_t *ps_ctx, uint8_t *pc_txData, uint8_t *pc_rxData, uint16_t ui16_len);


//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

#endif /* HAL_HAL_SPI_H_ */
