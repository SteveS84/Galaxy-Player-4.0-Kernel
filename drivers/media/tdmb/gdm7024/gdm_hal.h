/* ============================================================================================



                  G D M     H O S T     A D A P T A T I O N     L A Y E R






 This file includes HAL(Host Adatation Layer) function prototypes for S/T-DMB chipset GDM700X.
 
 All the clients who use GCT's host API solution should make HAL functions in accordance with

 the function definition.







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Jan.19.2007       james Shin       delete memory access type in MM function
Jan.15.2007	      Summerj	         mm func moved to hal
Jan.05.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_HAL_H
#define _GDM_HAL_H

/* ============================================================================================
                                I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_types.h"

/* ============================================================================================
                                     D E F I N T I O N S
============================================================================================ */



/* ============================================================================================
                                     F U N C T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------

FUNCTION    
									GDM_HAL_INIT_FUNC



DESCRIPTION

		This function initialize Function Pointer. You should make function appropriately
		to the target system. Keep it in mind that all host API functions are based on this
		function.


PARAMETER



RETURN VALUE

		None

-------------------------------------------------------------------------------------------- */
void gdm_hal_init_func(void);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_GET_ENDIAN



DESCRIPTION
           
           This function gets the information of host system endianness.



PARAMETER

           None.



RETURN VALUE

           The host system endianness.

-------------------------------------------------------------------------------------------- */
hal_endian_e_type gdm_hal_get_endian(void);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_SET_ENDIAN



DESCRIPTION
           
           This function sets host system endianness to big or little endian.



PARAMETER

           [in] endian - the host system endianness.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_set_endian(hal_endian_e_type endian);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_READ_REG



DESCRIPTION
           
           This function reads 16bit data of GDM700X GHOST register through host interface
           such as 6bit address - 16bit data parallel bus, I2C or SPI. The function should be made
           appropriately to the target system. Keep it in mind that all the host APIs
           is based on this function.



PARAMETER

           [in] addr - the 6bit address of GDM700X GHOST register to be read
           [out] read_buf_ptr - the buffer for saving the 16bit data of GDM700X GHOST register




RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_read_reg(G_Uint8 addr, G_Uint16* read_buf_ptr);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_BURST_READ_REG



DESCRIPTION
           
           This function reads array of 16bit data of GDM700X GHOST register through host interface
           such as 6bit address - array of 16bit data parallel bus, I2C or SPI. The function should
           be made appropriately to the target system. 


PARAMETER

           [in] addr - the 6bit address of GDM700X GHOST register to be read
           [out] read_buf_ptr - the buffer for saving the array of 8bit data of GDM700X GHOST register
           [in] size - the size of data to be read in unit of 8bit byte



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_burst_read_reg(G_Uint8 addr, G_Uint8* read_buf_ptr, G_Uint32 size);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_HAL_WRITE_REG



DESCRIPTION
           
           This function writes 16bit data to GDM700X GHOST register through host interface
           such as 6bit address - 16bit data parallel bus, I2C or SPI. The function should be
           made appropriately to the target system. Keep it in mind that all the host APIs
           is based on this function.



PARAMETER

           [in] addr - the 6bit address of GDM700X GHOST register to be written
           [in] data - the 16bit data to be written



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_write_reg(G_Uint8 addr, G_Uint16 data);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_CR_READ



DESCRIPTION
           
           This function reads the register addressed by GHOST_CR_ADDR register.



PARAMETER

           [in] addr - the address of register to be read.
           [out] read_buf_ptr - the buffer for saving the 16bit data of GDM700X CR



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_cr_read(G_Uint16 addr, G_Uint16* read_buf_ptr);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_CR_WRITE



DESCRIPTION
           
           This function writes the 16bit data to register addressed by GHOST_CR_ADDR register.



PARAMETER

           [in] addr - the address of register to be written.
           [in] data - the data to be written.




RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_cr_write
(
  G_Uint16 addr,
  G_Uint16 data
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_MM_READ



DESCRIPTION
           
           This function reads the internal memory addressed by GHOST_MM_ADDR_H 
           and GHOST_MM_ADDR_L register.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [out] read_buf_ptr - the buffer for saving the 32bit data of GDM700X internal memory



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_read
(
  G_Uint32 addr,
  G_Uint32* read_buf_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_HAL_MM_WRITE



DESCRIPTION
           
           This function writes the 32bit data to internal memory addressed by GHOST_MM_ADDR_H 
           and GHOST_MM_ADDR_L register.



PARAMETER

           [in] addr - the address of internal memory to be written.
           [in] data - the 32bit data to be written.
           [in] burst_mode - the flag of internal memory burst access.




RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_write
(
  G_Uint32 addr,
  G_Uint32 data
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                     GDM_HAL_MM_BURST_READ



DESCRIPTION
           
           This function executes the internal memory burst access read scheme.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [out] read_buf_ptr - the buffer for saving the 8bit data
           [in] size - the size of data to be read in unit of 8bit byte.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_burst_read
(
  G_Uint32 addr,
  G_Uint8* read_buf_ptr,
  G_Uint32 size
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_HAL_MM_BURST_WRITE



DESCRIPTION
           
           This function executes the internal memory burst access write scheme.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [in] write_buf_ptr - the buffer of data to write.
           [in] size - the size of data to write in unit of 8bit byte.



RETURN VALUE

           None.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_hal_mm_burst_write
(
  G_Uint32 addr,
  G_Uint8* write_buf_ptr,
  G_Uint32 size
);

void spi_open(void);
void spi_close(void);

#endif /* _GDM_HAL_H */
