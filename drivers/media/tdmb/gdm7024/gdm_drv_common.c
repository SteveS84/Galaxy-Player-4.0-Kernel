/* ============================================================================================



                          G D M     D R I V E R     -     C O M M O N






 This file includes common driver functions for S/T-DMB chipset GDM700X. All the GDM700X

 GHOST registers will be only accessed by following functions. The purpose of driver APIs is

 not allow GCT Host API users to access directly to GDM700X GHOST registers. It will be

 very helpful in fixing up clients' problems and prohibiting clients from using GDM700X

 inappropriately.







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                 C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Jan.19.2007       james Shin       delete memory access type in MM function
Jan.15.2007       Summerj          mm func moved to hal
Jan.05.2007       James Shin       created

-------------------------------------------------------------------------------------------- */

#define GDM_API_FILE // definition indicating this file is API file

/* ============================================================================================
                                I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_hal.h"
#include "gdm_drv_common.h"
#include "gdm_memory_map.h"
#include <linux/string.h>
#include <linux/kernel.h>

/* ============================================================================================
                                        E X T E R N S
============================================================================================ */
extern gdm_hal_func_s_type gdm_hal_func;

/* ============================================================================================
                                        F U N C T I O N S
============================================================================================ */


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_DRV_GHOST_GET_SYS_CTRL



DESCRIPTION
           
           This function gets the information of GHOST interface characteristics 
           such as HWAIT_N, host interrupt polarity, SPI clock polarity and SPI clock phase.



PARAMETER

           [out] hwait_down - When set to 1, HWAIT_N goes down 3cycles after HCS_N falls.
           [out] host_int - When set to 1, the interrupt to the host is negative asserted.
           [out] spi_clk_polarity - CPOL of SPI. Clock polarity.
                                    When set to 0, SPI-CLK idle state is LOW.
                                    When set to 1, SPI-CLK idle state is HIGH.
           [out] spi_clk_phase - CPHA of SPI. Clock phase.
                                 When set to 0, valid data available on leading edge of SPI-CLK.
                                 When set to 1, valid data available on trailing edge of SPI-CLK.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_get_sys_ctrl
(
  hwait_down_e_type* hwait_down,
  host_int_e_type* host_int,
  spi_clk_polarity_e_type* spi_clk_polarity,
  spi_clk_phase_e_type* spi_clk_phase
)
{
  G_Uint16 ghost_sys_ctrl = 0;

  if((hwait_down == NULL) || (host_int == NULL) || (spi_clk_polarity == NULL) || (spi_clk_phase == NULL))
  {
    return G_ERROR;
  }

  
  if(gdm_hal_func.read_reg(GHOST_CTRL, &ghost_sys_ctrl) == G_SUCCESS)
  {
    *hwait_down = (hwait_down_e_type)(ghost_sys_ctrl & 0x0001);
    *host_int = (host_int_e_type)((ghost_sys_ctrl & 0x0002) >> 1);
    *spi_clk_polarity = (spi_clk_polarity_e_type)((ghost_sys_ctrl & 0x0004) >> 2);
    *spi_clk_phase = (spi_clk_phase_e_type)((ghost_sys_ctrl & 0x0008) >> 3);

    return G_SUCCESS;
  }
  else
  {
    return G_ERROR;
  }

} /* gdm_drv_ghost_get_sys_ctrl() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_DRV_GHOST_SET_SYS_CTRL



DESCRIPTION
           
           This function configures GHOST interface characteristics such as HWAIT_N, host
           interrupt polarity, SPI clock polarity and SPI clock phase.



PARAMETER

           [in] hwait_down - When set to 1, HWAIT_N goes down 3cycles after HCS_N falls.
           [in] host_int - When set to 1, the interrupt to the host is negative asserted.
           [in] spi_clk_polarity - CPOL of SPI. Clock polarity.
                                   When set to 0, SPI-CLK idle state is LOW.
                                   When set to 1, SPI-CLK idle state is HIGH.
           [in] spi_clk_phase - CPHA of SPI. Clock phase.
                                When set to 0, valid data available on leading edge of SPI-CLK.
                                When set to 1, valid data available on trailing edge of SPI-CLK.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_set_sys_ctrl
(
  hwait_down_e_type hwait_down,
  host_int_e_type host_int,
  spi_clk_polarity_e_type spi_clk_polarity,
  spi_clk_phase_e_type spi_clk_phase
)
{
  G_Uint16 data = 0;

  if((hwait_down >= HWAIT_DOWN_MAX)
      || (host_int >= HOST_INT_MAX)
      || (spi_clk_polarity >= SPI_CLK_POLARITY_MAX)
      || (spi_clk_phase >= SPI_CLK_PHASE_MAX))
  {
    return G_ERROR;
  }

  data = (G_Uint16)(hwait_down | (host_int << 1) | (spi_clk_polarity << 2) | (spi_clk_phase << 3));

  return gdm_hal_func.write_reg(GHOST_CTRL, data);
} /* gdm_drv_ghost_set_sys_ctrl() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_GET_INT_STATUS



DESCRIPTION
           
           This function gets the information of status register for interrupt.



PARAMETER

           [out] int_status_ptr - the information of status register for interrupt.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_get_int_status(G_Uint16* int_status_ptr)
{
  if(int_status_ptr == NULL)
  {
    return G_ERROR;
  }

    return gdm_hal_func.read_reg(GHOST_EXT_IS, int_status_ptr);
} /* gdm_drv_ghost_get_int_status() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_DRV_GHOST_SET_INT_CLEAR



DESCRIPTION
           
           This function set the interrupt bits to be cleared to 1.



PARAMETER

           [in] int_clear_mask - the interrupt bit mask to be cleared.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_set_int_clear(G_Uint16 int_clear_mask)
{
  return gdm_hal_func.write_reg(GHOST_EXT_IC, int_clear_mask);
} /* gdm_drv_ghost_set_int_clear() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                 GDM_DRV_GHOST_GET_INT_MASK



DESCRIPTION
           
           This function gets the information of mask register for interrupt.



PARAMETER

           [out] int_mask_ptr - the information of mask register for interrupt.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_get_int_mask(G_Uint16* int_mask_ptr)
{
  return gdm_hal_func.read_reg(GHOST_EXT_IM, int_mask_ptr);
} /* gdm_drv_ghost_get_int_mask() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                 GDM_DRV_GHOST_SET_INT_MASK



DESCRIPTION
           
           This function set the interrupt bits of mask register to 1.



PARAMETER

           [in] int_mask - the interrupt bit mask to be set.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_set_int_mask(G_Uint16 int_mask)
{
  return gdm_hal_func.write_reg(GHOST_EXT_IM, int_mask);
} /* gdm_drv_ghost_set_int_mask() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_CR_READ



DESCRIPTION
           
           This function reads the register addressed by GHOST_CR_ADDR register.



PARAMETER

           [in] addr - the address of register to be read.
           [out] read_buf_ptr - the buffer within host memory for saving the data of CR




RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_cr_read(G_Uint16 addr, G_Uint16* read_buf_ptr)
{
  G_Int8 ret;

  ret = gdm_hal_func.cr_read(addr, read_buf_ptr);
  return ret;
} /* gdm_drv_ghost_cr_read() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_CR_WRITE



DESCRIPTION
           
           This function writes the 16bit data to register addressed by GHOST_CR_ADDR register.



PARAMETER

           [in] addr - the address of register to be written.
           [in] data - the data to be written.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_cr_write
(
  G_Uint16 addr,
  G_Uint16 data
)
{
  G_Int8 ret;

  ret = gdm_hal_func.cr_write(addr, data);
  return ret;
} /* gdm_drv_ghost_cr_write() */


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DRV_GHOST_BUF_READ



DESCRIPTION
           
           This function reads the data of internal buffer through GHOST configuration registers 
           such as GHOST_DM, GHOST_PILOT, GHOST_TS, GHOST_N_TS, GHOST_DAB0_FIC, GHOST_DAB0_NTSx,
           GHOST_DAB0_TS0, GHOST_DAB0_TS1, GHOST_DAB1_FIC, GHOST_DAB1_NTSx, GHOST_DAB1_TS0
           and GHOST_DAB0_TS1.



PARAMETER

           [in] buf_addr - the address of GHOST buffer
           [out] read_buf_ptr - the buffer to save within host memory




RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_buf_read
(
  G_Uint8 buf_addr, 
  G_Uint8* read_buf_ptr,
  G_Uint32 size
)
{
  if(read_buf_ptr == NULL)
  {
    return G_ERROR;
  }
  
  return gdm_hal_func.burst_read_reg(buf_addr, read_buf_ptr,size);
} /* gdm_drv_ghost_buf_read() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                     GDM_DRV_GHOST_MM_READ



DESCRIPTION
           
           This function reads the internal memory addressed by GHOST_MM_ADDR_H 
           and GHOST_MM_ADDR_L register.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [out] read_buf_ptr - the buffer within host memeory for saving the data of internal
                                memory.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_read
(
  G_Uint32 addr,
  G_Uint32* read_buf_ptr
)
{
  if(read_buf_ptr == NULL)
  {
    return G_ERROR;
  }
  
  return gdm_hal_func.mm_read(addr, read_buf_ptr);
} /* gdm_drv_ghost_mm_read() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DRV_GHOST_MM_WRITE



DESCRIPTION
           
           This function writes the 32bit data to internal memory addressed by GHOST_MM_ADDR_H 
           and GHOST_MM_ADDR_L register.



PARAMETER

           [in] addr - the address of internal memory to be written.
           [in] data - the 32bit data to be written.
           [in] burst_mode - the flag of internal memory burst access.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_write
(
  G_Uint32 addr,
  G_Uint32 data
)
{
  return gdm_hal_func.mm_write(addr, data);
} /* gdm_drv_ghost_mm_write() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_MM_BURST_READ



DESCRIPTION
           
           This function executes the internal memory burst access read scheme.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [out] read_buf_ptr - the buffer within host memory for saving the data of internal
                                memory.
           [in] size - the size of data to be read in unit of 8bit byte.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_burst_read
(
  G_Uint32 addr,
  G_Uint8* read_buf_ptr,
  G_Uint32 size
)
{
  if(read_buf_ptr == NULL)
  {
    return G_ERROR;
  }
  
  return gdm_hal_func.mm_burst_read(addr, read_buf_ptr, size);
} /* gdm_drv_ghost_mm_burst_read() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                GDM_DRV_GHOST_MM_BURST_WRITE



DESCRIPTION
           
           This function executes the internal memory burst access write scheme.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [in] write_buf_ptr - the buffer of data to write.
           [in] size - the size of data to write in unit of 8bit byte



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_burst_write
(
  G_Uint32 addr,
  G_Uint8* write_buf_ptr,
  G_Uint32 size
)
{
  if(write_buf_ptr == NULL)
  {
    return G_ERROR;
  }
  
  return gdm_hal_func.mm_burst_write(addr, write_buf_ptr, size);
} /* gdm_drv_ghost_mm_burst_write() */



/* --------------------------------------------------------------------------------------------

FUNCTION    
                                GDM_DRV_CR_GET_DEVICE_ID



DESCRIPTION
           
           This function gets the information of GDM device identification.



PARAMETER

           [out] device_id_ptr - GDM modem status bit mask.

                                 bit index  description
                                 ---------  -----------------------------------------
                                   15:8     target device.
                                            2:GDM7002, 3:GDM7003, 4:GDM7004, 14:GDM7014

                                    7:0     Chip Revision.




RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed


-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_cr_get_device_id(G_Uint16* device_id_ptr)
{
  G_Uint16 addr = 0;

  if(device_id_ptr == NULL)
  {
    return G_ERROR;
  }

  addr = (G_Uint16)CR_DEV_ID;
  
  return gdm_drv_ghost_cr_read(addr, device_id_ptr);
} /* gdm_drv_cr_get_device_id() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                     GDM_DRV_CR_GET_DM_CTRL



DESCRIPTION
           
           This function gets the information of diagnostic monitor(DM) parameters.



PARAMETER

           [out] dm_mode_ptr - the kind of diagnostic monitor(DM) mode.
           [out] dm_ms_period_ptr - DM report period(ms).



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_cr_get_dm_ctrl
(
  dm_mode_e_type* dm_mode_ptr,
  G_Uint16* dm_ms_period_ptr
)
{
  G_Uint16 addr = 0;
  G_Uint16 dm_ctrl = 0;

  if((dm_mode_ptr == NULL) || (dm_ms_period_ptr == NULL))
  {
    return G_ERROR;
  }

  addr = (G_Uint16)CR_DM_CTRL;

  if(gdm_drv_ghost_cr_read(addr, &dm_ctrl) == G_SUCCESS)
  {
    *dm_mode_ptr = (dm_mode_e_type)(dm_ctrl & 0x0003);
    *dm_ms_period_ptr = (dm_ctrl & 0xFFFC) >> 2;

    return G_SUCCESS;
  }
  else
  {
    return G_ERROR;
  }
} /* gdm_drv_cr_get_dm_ctrl() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                      GDM_DRV_CR_SET_DM_CTRL



DESCRIPTION
           
           This function configures diagnostic monitor(DM) parameters.



PARAMETER

           [in] dm_mode - the kind of diagnostic monitor(DM) mode.
           [in] dm_ms_period - DM report period(ms).



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_cr_set_dm_ctrl
(
  dm_mode_e_type dm_mode,
  G_Uint16 dm_ms_period
)
{
  G_Uint16 addr = 0;
  G_Uint16 data = 0;

  if(dm_mode >= DM_MODE_MAX)
  {
    return G_ERROR;
  }

  addr = (G_Uint16)CR_DM_CTRL;
  data = (G_Uint16)(dm_mode | (dm_ms_period << 2));

  return gdm_drv_ghost_cr_write(addr, data);
} /* gdm_drv_cr_set_dm_ctrl() */


/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_READ



DESCRIPTION
           
           This function reads the register addressed by GHOST_ADDR register.



PARAMETER

           [in] addr - the address of register to be read.
           [out] read_buf_ptr - the buffer within host memory for saving the data of GHOST register




RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_read
(
  G_Uint8 addr, 
  G_Uint16* read_buf_ptr
)
{
  G_Int8 ret;

  ret = gdm_hal_func.read_reg(addr,read_buf_ptr);
  //gdm_hal_func.debug_msg("rgh 0x%x, 0x%x\n",addr,*read_buf_ptr);
  return ret;
} /* gdm_drv_ghost_read() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_GHOST_WRITE



DESCRIPTION
           
           This function writes the 16bit data to register addressed by GHOST register.



PARAMETER

           [in] addr - the address of register to be written.
           [in] data - the data to be written.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_write
(
  G_Uint8 addr,
  G_Uint16 data
)
{
  G_Int8 ret;

  ret = gdm_hal_func.write_reg(addr, data);
  gdm_hal_func.debug_msg("wgh 0x%x, 0x%x\n",addr,data);
  return ret;
} /* gdm_drv_ghost_write() */



// TODO: Add CR_DM_ENDIAN, CR_TEST_ENDIAN for GDM7014
