/* ============================================================================================



                          G D M     D R I V E R     -     C O M M O N






 This file includes common driver functions for S/T-DMB chipset GDM700X. All the GDM700X

 GHOST registers will be only accessed by the following functions. The purpose of driver APIs is

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
Jan.05.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_DRV_COMMON_H
#define _GDM_DRV_COMMON_H

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
);

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
);

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
G_Int8 gdm_drv_ghost_get_int_status(G_Uint16* int_status_ptr);

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
G_Int8 gdm_drv_ghost_set_int_clear(G_Uint16 int_clear_mask);

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
G_Int8 gdm_drv_ghost_get_int_mask(G_Uint16* int_mask_ptr);

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
G_Int8 gdm_drv_ghost_set_int_mask(G_Uint16 int_mask);

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
G_Int8 gdm_drv_ghost_cr_read(G_Uint16 addr, G_Uint16* read_buf_ptr);

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
);

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
           [in] size - the size of data to be read in unit of 16bit word


RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_buf_read
(
  G_Uint8 buf_addr,
  G_Uint8* read_buf_ptr,
  G_Uint32 size
);

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
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DRV_GHOST_MM_WRITE



DESCRIPTION
           
           This function writes the 32bit data to internal memory addressed by GHOST_MM_ADDR_H 
           and GHOST_MM_ADDR_L register.



PARAMETER

           [in] addr - the address of internal memory to be written.
           [in] data - the 32bit data to be written.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_write
(
  G_Uint32 addr,
  G_Uint32 data
);

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
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                GDM_DRV_GHOST_MM_BURST_WRITE



DESCRIPTION
           
           This function executes the internal memory burst access write scheme.



PARAMETER

           [in] addr - the address of internal memory to be read.
           [in] write_buf_ptr - the buffer of data to write.
           [in] size - the size of data to write in unit of 8bit byte.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_ghost_mm_burst_write
(
  G_Uint32 addr,
  G_Uint8* write_buf_ptr,
  G_Uint32 size
);



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
                                            2:GDM7002, 3:GDM7003, 4:GDM7004.

                                    7:0     Chip Revision.




RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_cr_get_device_id(G_Uint16* device_id_ptr);

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
);

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
);


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
);

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
);

#endif /* _GDM_DRV_COMMON_H */
