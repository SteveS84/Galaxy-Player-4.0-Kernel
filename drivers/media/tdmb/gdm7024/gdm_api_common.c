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

Jan.23.2007       James Shin       Added interrupt handler
Jan.18.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */
#define GDM_API_FILE // definition indicating this file is API file

/* ============================================================================================
                                 I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_headers.h"

/*
#include "gdm_api_dab.h"
#include "gdm_drv_common.h"
#include "gdm_memory_map.h"
#include "gdm_api_common.h"
*/


/* ============================================================================================
                                     D E F I N T I O N S
============================================================================================ */



/* ============================================================================================
                                       V A R I A B L E S
============================================================================================ */


/* ============================================================================================
                                       F U N C T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_PROGRAM_DOWNLOAD



DESCRIPTION
           
           This function downloads the firmware to internal memory in GDM700X.

PARAMETER

           [in] code_addr - the address of binary code to be downloaded to instruction memory.
           [in] code_size - the size of binary code in unit of byte.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_program_download
(
  G_Uint8* code_addr,
  G_Uint32 code_size
)
{
  G_Uint32 iram_addr = NULL,iram_size=0;

  /* download binary code to IROM */
  if(code_addr && code_size)
  {
    if(code_size != GDM7024_IRAM_BIN_SIZE)
	{
		printk("[%s] code_size != GDM7024_IRAM_BIN_SIZE \n", __FUNCTION__);
      return G_ERROR;
	}

    if(gdm_read_iram_address(code_addr,&iram_addr,&iram_size) != G_SUCCESS)
	{
		printk("[%s] %d, %d \n", __FUNCTION__, iram_addr, iram_size);
      return G_ERROR;
	}

    if(iram_addr && iram_size)
    {
      if(gdm_drv_ghost_mm_burst_write(GDM_IROM_BASE, iram_addr, iram_size) != G_SUCCESS)
      {
        return G_ERROR;
      }
    }
  }

  return G_SUCCESS;
}


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
)
{
  G_Uint32 iram_addr = NULL,iram_size=0;
  G_Uint32 i = 0;
  G_Uint8* download_addr = NULL;

  if(code_addr == NULL 
  || code_size == 0 
  || verify_code_buf == NULL
  || verify_code_buf_size == 0)
    return G_ERROR;

  if(gdm_read_iram_address(code_addr,&iram_addr,&iram_size) != G_SUCCESS)
    return G_ERROR;

  if(verify_code_buf_size < iram_size) 
    G_ERROR;

  if(gdm_hal_mm_burst_read(GDM_IROM_BASE,verify_code_buf,iram_size) != G_SUCCESS)
    G_ERROR;

  download_addr = (G_Uint8*)iram_addr;

  for(i=0;i<iram_size;i++)
  {
    if(download_addr[i] != verify_code_buf[i] )
  {
    return G_ERROR;
  }
  }

  return G_SUCCESS;
}

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
)
{
  G_Uint32 addr = GDM7024_IROM_BASE + GDM7024_IROM_SIZE - GDM7024_BIN_INFO_SIZE;
  G_Uint32 size = GDM7024_BIN_INFO_SIZE;

  if(read_buf)
    return gdm_drv_ghost_mm_burst_read(addr,read_buf,size);
  else
    return G_ERROR;
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                         GDM_RUN



DESCRIPTION
           
           This function executes GDM700X.



PARAMETER

           None.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_run(void)
{
  G_Uint32 addr = 0;
  G_Uint32 data = 0;

  /* write "boot complete" command */
  addr = GDM_SYS_STA;
  data = 0x00004000;
  if(gdm_drv_ghost_mm_write(addr, data) != G_SUCCESS)
  {
    return G_ERROR;
  }

  /* write "process run" command */
  addr = GDM_DEBUG_CMD;
  data = 0x00000010;
  return gdm_drv_ghost_mm_write(addr, data);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_SET_DM



DESCRIPTION
           
           This function configures the parameters of DM monitor.



PARAMETER

           [in] dm_mode - the kind of diagnostic monitor(DM) mode.
           [in] dm_ms_period - DM report period(ms).



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_set_dm
(
  dm_mode_e_type dm_mode,
  G_Uint16 dm_ms_period
)
{
  return gdm_drv_cr_set_dm_ctrl(dm_mode, dm_ms_period);
}

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
)
{
  return gdm_drv_ghost_buf_read((G_Uint8)GHOST_DM, read_buf_ptr,size);
}

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
G_Int8 gdm_dm_int_handler
(
  G_Uint8 * read_buf_ptr, 
  G_Uint32 size
)
{
  if(read_buf_ptr == NULL || size == 0)
  {
    return G_ERROR;
  }
  return gdm_read_dm( read_buf_ptr,size);
}


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_CHIP_ID



DESCRIPTION
           
           This function reads chip id



PARAMETER

           [out] chip_id - chip id.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_read_chip_id(G_Uint32 * chip_id)
{
  return gdm_drv_ghost_mm_read(GDM_SYS_GLB_CTRL_CHIP_ID,chip_id);
}

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
G_Int8 gdm_read_device_id(G_Uint16 * dev_id)
{
	return gdm_drv_cr_get_device_id(dev_id);
}

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
G_Int8 gdm_read_chip_ver(G_Uint16 *chip_ver)
{
  char bin_info[40];

  gdm_get_binary_information(bin_info);

  if(bin_info[GDM7024_BIN_INFO_MREV_OFFSET] == '1')
    *chip_ver = GDM_CHIP_VER_R1A;      
  else if(bin_info[GDM7024_BIN_INFO_MREV_VER_OFFSET] == 'C')
    *chip_ver = GDM_CHIP_VER_R0C;
  else 
    *chip_ver = GDM_CHIP_VER_R0;

  return G_SUCCESS;
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_READ_BINARY_SIZE



DESCRIPTION
           
           This function reads size of binary to download.



PARAMETER

           [in] buf - Pointer to binary.
           [in] buf_size - size of binary.


RETURN VALUE

           size of binary to download.

-------------------------------------------------------------------------------------------- */
G_Uint32 gdm_read_binary_size
(
  G_Uint8 *buf, 
  G_Uint32 buf_size
)
{
  G_Uint32 i = 0,multiple = 1;
  G_Uint32 size = 0;

  for( i = 6 ; i > 0 ; i-- )
  {
    size += ( (buf[buf_size - (31 + 6 - i)] - '0')) * multiple;
    multiple *= 10;
  }

  return size;
}

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
)
{

  char bin_info[40];
  unsigned int index =0;
  G_Uint8 *address = 0;

  if(iram_bin == NULL || iram_address == NULL || iram_size == NULL) 
    return G_ERROR;

  gdm_get_binary_information(bin_info);

#if 0
  index = 3;
  address = iram_bin + ((index-2) * 8*1024);
#else
  printk("[%s] revision '%c' \n", __FUNCTION__, bin_info[GDM7024_BIN_INFO_MREV_OFFSET]);
  if(bin_info[GDM7024_BIN_INFO_MREV_OFFSET] == '1')
  {
    address = iram_bin;
	printk("[%s] address = 0x%.8x\n", __FUNCTION__, address);
  }
  else
  {  
    index = bin_info[GDM7024_BIN_INFO_MREV_OFFSET] - '0';
    address = iram_bin + ((index-2) * 8*1024);
  }
#endif

  *iram_address = (G_Uint32)address;
  *iram_size    = gdm_read_binary_size(address,8*1024);

	printk("[%s] address:0x%.8x, iram_size:%d\n", __FUNCTION__, *iram_address, *iram_size);

  return G_SUCCESS;
}

