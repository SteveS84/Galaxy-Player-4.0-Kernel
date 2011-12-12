/* ============================================================================================



                           G D M     D R I V E R     -     D A B






 This file includes DAB0/1 driver functions for S/T-DMB chipset GDM700X. All the GDM700X

 CR registers for DAB0/1 conrol will be only accessed by the following functions. 
 
 The purpose of driver APIs is not allow GCT Host API users to access directly to GDM700X CR registers. 

 It will be very helpful in fixing up clients' problems and prohibiting clients from using GDM700X

 inappropriately.







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                 C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Jan.05.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */



#ifndef _GDM_DRV_DAB
#define _GDM_DRV_DAB

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
                                   GDM_DRV_DAB_GET_MODE



DESCRIPTION
           
           This function gets the information of the DAB0/1 mode and frequency.



PARAMETER

           [in] dab_no - DAB path number.
           [out] mode_ptr - DAB mode selection.
           [out] freq_mode_ptr - frequency mode.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_mode
(
  dab_path_e_type dab_no,
  dab_mode_e_type* mode_ptr,
  dab_freq_mode_e_type* freq_mode_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  GDM_DRV_DAB_SET_MODE



DESCRIPTION
           
           This function configures the parameters of the DAB0/1 mode and frequency.



PARAMETER

           [in] dab_no - DAB path number.
           [out] mode - DAB mode selection.
           [out] freq_mode - frequency mode.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_mode
(
  dab_path_e_type dab_no,
  dab_mode_e_type mode,
  dab_freq_mode_e_type freq_mode
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DRV_DAB_GET_FIC_WM



DESCRIPTION
           
           This function gets the watermark levels of FIC buffer.



PARAMETER

					[in] dab_no - DAB path number.
          [out] wm_ptr - the watermark level.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_fic_wm
(
  dab_path_e_type dab_no,
  G_Uint16* wm_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                    GDM_DRV_DAB_GET_MSC_WM



DESCRIPTION
           
           This function gets the watermark levels of MSC buffer.



PARAMETER
						[in] dab_no - DAB path number.
            [out] wm_ptr - the watermark level.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_msc_wm
(
  dab_path_e_type dab_no,
  G_Uint16* wm_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_DAB_SET_MSC_WM



DESCRIPTION
           
           This function set the watermark levels of MSC buffer.



PARAMETER
						[in] dab_no - DAB path number.
            [in] wm_level - the watermark level.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_msc_wm
(
  dab_path_e_type dab_no,
  G_Uint16 wm_level
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_DAB_GET_ERR_TS_DISCARD



DESCRIPTION
           
           This function gets the erred ts discard bit in CR_DAB_ERR_TS_DISCARD



PARAMETER

           [in] dab_no - DAB path number.
           [in] flag - enable/disable flag.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_err_ts_discard
(
  dab_path_e_type dab_no,
	G_Uint16 *flag
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                   GDM_DRV_DAB_SET_ERR_TS_DISCARD



DESCRIPTION
           
           This function gets the erred ts discard bit in CR_DAB_ERR_TS_DISCARD



PARAMETER

           [in] dab_no - DAB path number.
           [out] flag - enable/disable flag.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_err_ts_discard
(
  dab_path_e_type dab_no,
	G_Uint16 flag
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                 GDM_DRV_DAB_SET_SYS_CTRL



DESCRIPTION
           
           This function sends the command about DAB0/1 system operation to GDM700X.



PARAMETER

           [in] dab_no - DAB path number.
           [in] cmd - the command about DAB0/1 system operation.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_sys_ctrl
(
  dab_path_e_type dab_no,
  dab_sys_cmd_e_type cmd
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_DEMOD_STATUS



DESCRIPTION
           
           This function gets the information of DAB0/1 demodulator status.



PARAMETER

           [in] dab_no - DAB path number.
           [out] demod_status_ptr - DAB0/1 demodulator status.

                                    0xf : not ready.
                                    0x0 : standby.
                                    0x1 : Activate.
                                    0x2 : Acquisition.
                                    0x3 : FIC parsing.
                                    0x4 : Decoding.
           [out] sync_status_ptr - DAB0/1 sync failure flag.
                                    0xf : no meaning.
                                    0x0 : no sync failure
                                    0x1 : There was sync failure previously.
                                    Note - sync failure bit will be asserted on sync loss situation also.
                                              Check this on initial ensemble search only.
                                    

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_demod_status
(
  dab_path_e_type dab_no, 
  dab_demod_status_e_type* demod_status_ptr,
  dab_sync_status_e_type * sync_status_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_GET_PREDEFINED_FREQ



DESCRIPTION
           
           This function gets the frequency information by the predefined channel selector.



PARAMETER

           [in] dab_no - DAB path number.
           [out] center_freq_ptr - center frequency.
           [out] band_ptr - band selection.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_predefined_freq
(
  dab_path_e_type dab_no,
  G_Uint16* center_freq_ptr,
  dab_band_e_type* band_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_SET_PREDEFINED_FREQ



DESCRIPTION
           
           This function configures the frequency parameter for predefined channel selector.



PARAMETER

           [in] dab_no - DAB path number.
           [in] center_freq - center frequency.
           [in] band - band selection.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_predefined_freq
(
  dab_path_e_type dab_no,
  G_Uint16 center_freq,
  dab_band_e_type band
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                 GDM_DRV_DAB_GET_BAND_III_FREQ



DESCRIPTION
           
           This function gets the frequency information by the band III frequency calculator.



PARAMETER

           [in] dab_no - DAB path number.
           [out] band_iii_freq_ptr - CR_DAB0_FREQ = Center frequency / 16KHz.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_band_iii_freq(dab_path_e_type dab_no, G_Uint16* band_iii_freq_ptr);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_SET_BAND_III_FREQ



DESCRIPTION
           
           This function set the frequency for the band III frequency calculator mode.



PARAMETER

           [in] dab_no - DAB path number.
           [in] freq - center frequency / 16KHz.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_band_iii_freq
(
  dab_path_e_type dab_no,
  G_Uint16 freq
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_L_BAND_FREQ



DESCRIPTION
           
           This function gets the frequency information by the L band frequency calculator.



PARAMETER

           [in] dab_no - DAB path number.
           [out] l_band_freq_ptr - CR_DAB0_FREQ = (Center frequency - 1,441,792KHz) / 16KHz.



RETURN VALUE
           
           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_l_band_freq(dab_path_e_type dab_no, G_Uint16* l_band_freq_ptr);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                              GDM_DRV_DAB_SET_L_BAND_FREQ



DESCRIPTION
           
           This function set the frequency for the L band frequency calculator mode.



PARAMETER

           [in] dab_no - DAB path number.
           [in] freq - (Center frequency - 1,441,792KHz) / 16KHz.



RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_l_band_freq
(
  dab_path_e_type dab_no,
  G_Uint16 freq
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_STORED_FIB_SIZE



DESCRIPTION
           
           This function gets the size of data stored in FIB buffers.



PARAMETER
						[in] dab_no - DAB path number.
            [out] stored_fib_size_ptr - the size of data store in FIB buffers.
            														unit of 32 bit word. 

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_stored_fib_size
(
  dab_path_e_type dab_no,
  G_Uint16* stored_fib_size_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_FIC_CRC



DESCRIPTION
           
           This function gets the FIC CRC results of consecutive FIBs



PARAMETER
						[in] dab_no - DAB path number.
            [out] fic_crc_ptr - the FIC CRC results

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_fic_crc
(
  dab_path_e_type dab_no,
  G_Uint16* fic_crc_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_STORED_MSC_SIZE



DESCRIPTION
           
           This function gets the size of data stored in MSC buffers.



PARAMETER
						[in] dab_no - DAB path number.
            [out] stored_msc_size_ptr - the size of data store in MSC buffers.
            														unit of TS Packet(188byte). 

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_stored_msc_size
(
  dab_path_e_type dab_no,
  G_Uint16* stored_msc_size_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_SET_CLEAR_MSC_BUF



DESCRIPTION
           
           This function set clear msc buffer bit.
           On INT_DAB_MSC_OV interrupts, host should use this.



PARAMETER
						[in] dab_no - DAB path number.


RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_clear_msc_buf
(
  dab_path_e_type dab_no
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_SET_ENABLE_SERIAL_TS



DESCRIPTION
           
           This function set enable serial ts bits.



PARAMETER
						[in] dab_no - DAB path number.
						[in] flag - enable/disable flag.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_enable_serial_ts
(
  dab_path_e_type dab_no,
	G_Uint16 flag
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_ENCAP_DM_FIC


DESCRIPTION
           
           This function get encapsulating of dm & fic



PARAMETER
						[in] dab_no - DAB path number.
						[in] dm_flag - DM packet encapsulate enable/disable flag.
						[in] fic_flag - FIC packet encapsulate enable/disable flag.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_encap_dm_fic
(
  dab_path_e_type dab_no,
  dab_encap_dm_e_type *dm_flag,
  dab_encap_fic_e_type *fic_flag
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_SET_ENCAP_DM_FIC


DESCRIPTION
           
           This function set encapsulating of dm & fic



PARAMETER
						[in] dab_no - DAB path number.
						[in] dm_flag - DM packet encapsulate enable/disable flag.
						[in] fic_flag - FIC packet encapsulate enable/disable flag.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_encap_dm_fic
(
  dab_path_e_type dab_no,
  dab_encap_dm_e_type dm_flag,
  dab_encap_fic_e_type fic_flag
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_GET_SUBCH



DESCRIPTION
           
           This function gets sub channel id on specfied buffer(TSx, NTSx).



PARAMETER
						[in] dab_no - DAB path number.
						[out] enable_flag - enable/disable flag.
						[out] subch_id - sub channel id

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_subch
(
  G_Uint16 cr_addr,
  G_Uint8* enable_flag,
  G_Uint8* subch_id
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                               GDM_DRV_DAB_SET_SUBCH



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
G_Int8 gdm_drv_dab_set_subch
(
  G_Uint16 cr_addr,
  G_Uint8 enable_flag,
  G_Uint8 subch_id
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_SET_CAL_MODE



DESCRIPTION
           
           This function configures CR_DABn_CAL_MODE register configuration about
           calibration mode



PARAMETER
           [in] dab_no - DAB path number.
           [in] cal_mode - calibration mode.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_cal_mode
(
  dab_path_e_type dab_no,
  cal_mode_e_type cal_mode
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_GET_CAL_MODE



DESCRIPTION
           
           This function gets CR_DABn_CAL_MODE register configuration about
           calibration mode



PARAMETER
           [in] dab_no - DAB path number.
           [out] cal_mode_ptr - calibration mode.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_cal_mode
(
  dab_path_e_type dab_no,
  cal_mode_e_type * cal_mode_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_SET_LNA_GAIN



DESCRIPTION
           
           This function configures CR_DABn_CAL_LNA_GAIN register configuration about
           lna gain.



PARAMETER
           [in] dab_no - DAB path number.
           [in] lna_gain - LNA high gain value for reference.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_cal_lna_gain
(
  dab_path_e_type dab_no,
  G_Uint16 lna_gain
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_GET_LNA_GAIN



DESCRIPTION
           
           This function gets CR_DABn_CAL_LNA_GAIN register configuration about
           lna gain

           

PARAMETER
           [in] dab_no - DAB path number.
           [out] lna_gain - LNA high gain value for reference.

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_cal_lna_gain
(
  dab_path_e_type dab_no,
  G_Uint16 * lna_gain_ptr
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_SET_RSSI_DIFF



DESCRIPTION
           
           This function configures CR_DABn_CAL_RSSI_DIFFx register configuration about
           difference.



PARAMETER
           [in] dab_no - DAB path number.
           [in] cal_mode - calibration mode.
           [in] diff - difference(signed) in dBm scale

RETURN VALUE

           G_SUCCESS - Operation succeeded
           G_ERROR - Operation failed

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_set_cal_rssi_diff
(
  dab_path_e_type dab_no,
  cal_mode_e_type cal_mode,
  G_Int16 diff
);

/* --------------------------------------------------------------------------------------------

FUNCTION    
                             GDM_DRV_DAB_GET_RSSI_DIFF



DESCRIPTION
           
           This function gets CR_DABn_CAL_RSSI_DIFFx register configuration about
           difference.



PARAMETER
           [in] dab_no - DAB path number.
           [in] cal_mode_ptr - calibration mode.           
           [out] diff_ptr - difference(signed) in dBm scale.

RETURN VALUE

           G_SUCCESS - Operation succeeded.
           G_ERROR - Operation failed.

-------------------------------------------------------------------------------------------- */
G_Int8 gdm_drv_dab_get_cal_rssi_diff
(
  dab_path_e_type dab_no,
  cal_mode_e_type cal_mode,
  G_Int16 * diff_ptr
);
#endif /* _GDM_DRV_DAB */
