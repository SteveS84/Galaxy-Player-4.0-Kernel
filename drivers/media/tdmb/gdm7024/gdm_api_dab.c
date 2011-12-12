/* ============================================================================================



                               G D M     A P I     -     D A B






 This file includes DAB API(Application Programming Inteface) for customer application.

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
#include "gdm_hal.h"
#include "gdm_drv_common.h"
#include "gdm_drv_dab.h"
#include "gdm_api_dab.h"
#include "gdm_memory_map.h"
#include <stdlib.h>
*/

/* ============================================================================================
                                      V A R I A B L E S
============================================================================================ */

/* ============================================================================================
                                      F U N C T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_DAB_RUN



DESCRIPTION
           
           This function makes DAB system started.



PARAMETER

           [in] dab_no - DAB path number.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_run(dab_path_e_type dab_no)
{
  return gdm_drv_dab_set_sys_ctrl(dab_no, DAB_SYS_RUN_CMD);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_DAB_SCAN



DESCRIPTION
           
           This function makes DAB system started with fast channel scanning mode.



PARAMETER

           [in] dab_no - DAB path number.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_scan(dab_path_e_type dab_no)
{
  return gdm_drv_dab_set_sys_ctrl(dab_no, DAB_SYS_SCAN_CMD);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_DAB_STOP



DESCRIPTION
           
           This function makes DAB system stopped.
           and, clear all sub-channel configuration.



PARAMETER

           [in] dab_no - DAB path number.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_stop(dab_path_e_type dab_no)
{
  return gdm_drv_dab_set_sys_ctrl(dab_no, DAB_SYS_STOP_CMD);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_DAB_RESET



DESCRIPTION
           
           This function makes DAB system stopped with all configuration ramind.
           On INT_DAB_SYNC_LOSS interrupt, host should use this.



PARAMETER

           [in] dab_no - DAB path number.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_reset(dab_path_e_type dab_no)
{
  return gdm_drv_dab_set_sys_ctrl(dab_no, DAB_SYS_RESET_CMD);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                       GDM_DAB_CHANGE_FREQ



DESCRIPTION
           
           This function changes transmission mode and frequency mode with frequency.



PARAMETER

           [in] dab_no - DAB path number.
           [in] mode - DAB transmission mode
           [in] freq_mode - frequency setting mode
           [in] freq - frequency


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_change_freq
(
  dab_path_e_type dab_no, 
  dab_mode_e_type mode,
  dab_freq_mode_e_type freq_mode,
  G_Uint16 freq
)
{
  if(dab_no >= DAB_PATH_MAX)
  {
    return G_ERROR;
  }

  gdm_drv_dab_set_sys_ctrl(dab_no,DAB_SYS_STOP_CMD); //clear subchannel configuration
	msleep(5);
  gdm_drv_dab_set_mode(dab_no,mode,freq_mode);
   
	if(freq_mode == DAB_FREQ_MODE_PREDEFINED_CH_SELECTOR)
	  gdm_drv_dab_set_predefined_freq(dab_no, freq, DAB_BAND_KOREAN_TDMB);
  else if(freq_mode == DAB_FREQ_MODE_BAND_III_FREQ_CALCULATOR ||
          freq_mode == DAB_FREQ_MODE_L_BAND_FREQ_CALCULATOR)
    gdm_drv_dab_set_band_iii_freq(dab_no, freq);
  else
    return G_ERROR;

  return G_SUCCESS;
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DAB_CONFIG_SUBCH



DESCRIPTION
           
           This function set sub channel id on specfied buffer(TSx, NTSx).



PARAMETER
						[in] dab_no - DAB path number.
						[in] enable_flag - enable/disable flag.
						[in] subch_id - sub channel id

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_config_subch
(
  G_Uint16 cr_addr,
  G_Uint8 enable_flag,
  G_Uint8 subch_id
)
{
    return gdm_drv_dab_set_subch(cr_addr, enable_flag, subch_id);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                 GDM_DAB_GET_DEMOD_STATUS



DESCRIPTION
           
           This function get the information of demodulator status in DAB0/1 system.



PARAMETER

           [in] dab_no - DAB path number.
           [out] demod_status_ptr - DAB0/1 demodulator status.
           [out] sync_status_flag - DAB0/1 sync failure flag.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_get_demod_status
(
  dab_path_e_type dab_no, 
  dab_demod_status_e_type* demod_status_ptr,
  dab_sync_status_e_type * sync_status_ptr
)
{
  return gdm_drv_dab_get_demod_status(dab_no, demod_status_ptr, sync_status_ptr);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DAB_READ_FIC



DESCRIPTION
           
           This function reads the data in FIC buffer.



PARAMETER

           [in] dab_no - DAB path number.
           [in] size - the size of data to read.
           [out] read_buf_ptr - the buffer for saving the FIC data.
           


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_read_fic
(
  dab_path_e_type dab_no,
  G_Uint8* read_buf_ptr,
  G_Uint32 size
)
{
  G_Uint8 addr = 0;
  
  if((dab_no >= DAB_PATH_MAX) || (read_buf_ptr == NULL))
  {
    return G_ERROR;
  }

  addr = (dab_no == DAB_0_PATH) ? GHOST_DAB0_FIC : GHOST_DAB1_FIC;

  if(gdm_drv_ghost_buf_read(addr, read_buf_ptr,size) == G_SUCCESS)
  {
    return G_SUCCESS;
  }
  else
  {
    return G_ERROR;
  }
} /* gdm_dab_read_fic() */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DAB_READ_MSC



DESCRIPTION
           
           This function reads the data in TS0 buffer.



PARAMETER

           [in] dab_no - DAB path number.
           [out] read_buf_ptr - the buffer for saving the TS data.           
           [in] size - the size of data to read in unit of 8bit byte.
           


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_read_msc
(
  dab_path_e_type dab_no,
  G_Uint8* read_buf_ptr,
  G_Uint32 size  
)
{
  G_Uint8 addr = 0;
  
  if((dab_no >= DAB_PATH_MAX) || (read_buf_ptr == NULL))
  {
    return G_ERROR;
  }

  addr = (dab_no == DAB_0_PATH) ? GHOST_DAB0_MSC : GHOST_DAB1_MSC;

  if(gdm_drv_ghost_buf_read(addr, read_buf_ptr,size) == G_SUCCESS)
  {
    return G_SUCCESS;
  }
  else
  {
    return G_ERROR;
  }

  return G_SUCCESS;
} /* gdm_dab_read_msc() */


/* --------------------------------------------------------------------------------------------

FUNCTION    
                            GDM_DAB_FIC_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_FIC interrupt.



PARAMETER

           [in]dab_path - DAB path number.
           [out]buf_ptr - the buffer for saving the FIC data. 
           [in/out]size - the size of data to be read in unit of 8 bits byte. 
                          If there is error in reading FIC, buf_ptr is NULL, or stored size is 
                          greater than size, it will be returned with value of 0.



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_fic_int_handler
(
  dab_path_e_type dab_path,
  G_Uint8 * buf_ptr,
  G_Uint32 * size
)
{
	// TODO: how handle FIC CRC?

  if(buf_ptr == NULL)
  {
    *size = 0;
    return G_ERROR;
  }
  
  return gdm_dab_read_fic(dab_path, buf_ptr, *size);
}


/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DAB_MSC_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_MSC interrupt.



PARAMETER

           [in]dab_path - DAB path number.
           [out]buf_ptr -the buffer for saving the TS0 data. 
           [in]wm_size - the size of data to be read in unit of 8 bits byte that should be equal 
                         to watermark size in unit of byte. 



RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_msc_int_handler
(
  dab_path_e_type dab_path,
  G_Uint8 * buf_ptr,
  G_Uint32 wm_size
)
{
  if(buf_ptr == NULL || wm_size == 0)
  {
    return G_ERROR;
  }
  
  return gdm_dab_read_msc(dab_path, buf_ptr, wm_size);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DAB_MSC_OV_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_MSC_OV interrupt.



PARAMETER

           [in]dab_path - DAB path number.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_msc_ov_int_handler(dab_path_e_type dab_path)
{
	return gdm_drv_dab_set_clear_msc_buf(dab_path);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DAB_SYNC_LOSS_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_SYNC_LOSS interrupt.



PARAMETER

           [in]dab_path - DAB path number.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_sync_loss_int_handler(dab_path_e_type dab_path)
{
	if(gdm_dab_reset(dab_path) == G_ERROR)
		return G_ERROR;

	//wait 2msec here
	//uswait(2000);
	
	return gdm_dab_run(dab_path);
	// TODO: Customer Specific Function for Sync Loss Interrupt
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DAB_FIC_RECONF_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_FIC_RECONF interrupt.



PARAMETER

           [in]dab_path - DAB path number.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_fic_reconf_int_handler(dab_path_e_type dab_path)
{
	// TODO: Host should rescan channel.
	return G_SUCCESS;
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DAB_INVALID_SUBCH_INT_HANDLER



DESCRIPTION
           
           This function handles INT_DABx_INVALID_SUBCH interrupt.



PARAMETER

           [in]dab_path - DAB path number.


RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_dab_invalid_subch_int_handler(dab_path_e_type dab_path)
{
	// TODO: Host should rescan channel.
	return G_SUCCESS;
}

