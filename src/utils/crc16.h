/* ********************************************************************
 *
 * Filename:    crc.h
 *
 * Description: A header file describing the various CRC standards.
 *
 * Notes:
 *
 *
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef _crc_16_h
#define _crc_16_h

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \example test-app-sproto.c
//!
//! \addtogroup utils Utility modules
//! @{
//!
//! \addtogroup utils_crc CRC SW module
//! @{
//
//*****************************************************************************


#include <stdint.h>

typedef uint16_t  crc;


/*********************************************************************
 *
 * \fn    crcFast()
 *
 * \brief Compute the CRC of a given message by using lookup table.
 *
 * \return   The CRC of the message.
 *
 *********************************************************************/
crc   crcFast(uint8_t const message[], uint32_t nBytes);

//*****************************************************************************
//
// Close the Doxygen groups.
//! @}
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* _crc_16_h */
