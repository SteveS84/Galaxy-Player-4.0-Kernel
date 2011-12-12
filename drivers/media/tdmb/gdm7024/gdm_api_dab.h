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

Jan.18.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_API_DAB_H
#define _GDM_API_DAB_H

/* ============================================================================================
                                I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_types.h"

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
G_Int8 gdm_dab_run(dab_path_e_type dab_no);

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
G_Int8 gdm_dab_scan(dab_path_e_type dab_no);

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
G_Int8 gdm_dab_stop(dab_path_e_type dab_no);

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
G_Int8 gdm_dab_reset(dab_path_e_type dab_no);

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
);

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
);

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
);

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
);

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
);

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
);

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
);

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
G_Int8 gdm_dab_msc_ov_int_handler(dab_path_e_type dab_path);

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
G_Int8 gdm_dab_sync_loss_int_handler(dab_path_e_type dab_path);

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
G_Int8 gdm_dab_fic_reconf_int_handler(dab_path_e_type dab_path);

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
G_Int8 gdm_dab_invalid_subch_int_handler(dab_path_e_type dab_path);

#endif /* _GDM_API_DAB_H */