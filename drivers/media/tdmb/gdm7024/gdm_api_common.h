/* ============================================================================================



                             G D M     A P I     -     C O M M O N






 This file includes common API(Application Programming Inteface) for customer application.

 The customer may use directly these APIs or change them appropriate to their own application.

 Whatever they choose, this file give them at least how to use GDM host driver APIs.







                          GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                   C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Jan.18.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_API_COMMON_H
#define _GDM_API_COMMON_H

/* ============================================================================================
                                  I N C L U D E     F I L E S
============================================================================================ */
#include "gdm_types.h"


/* ============================================================================================
                                      F U N C T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_PROGRAM_DOWNLOAD



DESCRIPTION
           
           This function downloads the firmware to internal memory in GDM700X.



PARAMETER

           [in] code_addr - the address of binary code to be downloaded to instruction memory(ROM).
           [in] code_size - the size of binary code in unit of byte


RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_program_download
(
  G_Uint8* code_addr,
  G_Uint32 code_size
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_PROGRAM_DOWNLOAD_VERIFY



DESCRIPTION
           
           This function checks a result of download.

PARAMETER

           [in] code_addr - the address of binary code to be downloaded to instruction memory.
           [in] code_size - the size of binary code in unit of byte.
           [in] verify_code_buf - A Buffer pointer to verify code binary.
           [in] verify_code_buf_size - the size of verify_code_buf in unit of byte.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_program_download_verify
(
  G_Uint8* code_addr,
  G_Uint32 code_size,
  G_Uint8* verify_code_buf,
  G_Uint32 verify_code_buf_size
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_BINARY_INFORMATION



DESCRIPTION
           
           This function gets binary information.

PARAMETER

           [in] read_buf  - the buffer pointer for reading binary information.

RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_get_binary_information
(
  G_Uint8* read_buf
);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                         GDM_RUN



DESCRIPTION
           
           This function executes GDM700X.



PARAMETER

           None.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_run(void);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_SET_DM



DESCRIPTION
           
           This function configures the parameters of DM monitor.



PARAMETER

           [in] dm_mode - the kind of diagnostic monitor(DM) mode.
           [in] dm_ms_period - DM report period(ms).



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_set_dm
(
  dm_mode_e_type dm_mode,
  G_Uint16 dm_ms_period
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_DM



DESCRIPTION
           
           This function reads the information of DM monitor.



PARAMETER

           [out] read_buf_ptr - the buffer for saving DM information.
           [in] size - size of DM to be read in unit of 8bit byte



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_dm
(
  G_Uint8* read_buf_ptr,
  G_Uint32 size
);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                     GDM_DM_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DM interrupt.



PARAMETER

           [out] read_buf_ptr - the buffer for saving DM information.
           [in] size - size of DM to be read in unit of 8bit byte
           


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dm_int_handler(G_Uint8 * read_buf_ptr, G_Uint32 size);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_CHIP_ID



DESCRIPTION
           
           This function reads chip id



PARAMETER

           [in] chip_id - chip id.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_chip_id(G_Uint32 * chip_id);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_DEVICE_ID



DESCRIPTION
           
           This function reads device id



PARAMETER

           [out] dev_id - device id.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_device_id(G_Uint16 * dev_id);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_CHIP_VER



DESCRIPTION
           
           This function reads chip version.



PARAMETER

           [out] chip_ver - chip version.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_chip_ver(G_Uint16 *chip_ver);


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_IRAM_ADDRESS



DESCRIPTION
           
           This function reads address of IRAM binary to download.


PARAMETER

           [in] iram_bin - pointer of IRAM binary.
           [out] iram_address - address of binary to download.
           [out] iram_size - size of binary to download.

RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_iram_address
(
  G_Uint8  *iram_bin,
  G_Uint32 *iram_address,
  G_Uint32 *iram_size
);

G_Uint32 gdm_read_binary_size ( G_Uint8 *buf, G_Uint32 buf_size);
#endif /* _GDM_API_COMMON_H */
