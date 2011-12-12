/* ============================================================================================



                          G D M     T Y P E     D E F I N I T I O N






 This file describes type definitions used in GDM700X host APIs. If necessary, the clients 

 may change these definitions appropriately to their own systems







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                 C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Jan.16.2007       James Shin       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM_TYPES_H
#define _GDM_TYPES_H

/* ============================================================================================
                                I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_feature.h"


/* ============================================================================================
                                     D E F I N T I O N S
============================================================================================ */
typedef unsigned char             G_Boolean;      // unsigned 8 bit. TRUE or FALSE
typedef unsigned long             G_Uint32;       // unsigned 32 bit type
typedef unsigned short            G_Uint16;       // unsigned 16 bit type
typedef unsigned char             G_Uint8;        // unsigned 8 bit type
typedef signed long               G_Int32;        // signed 32 bit type
typedef signed short              G_Int16;        // signed 16 bit type
typedef signed char               G_Int8;         // signed 8 bit type
typedef char                      G_Char;         // char type

typedef unsigned char             G_Byte;         // unsigned 8 bit type
#ifdef WIN32
typedef __int64					  G_Int64;        // signed 64 bit type
typedef unsigned __int64          G_Uint64;       // unsigned 64bit type
#else
typedef signed long long          G_Int64;        // signed 64 bit type
typedef unsigned long long        G_Uint64;       // unsigned 64bit type
#endif
typedef unsigned short            G_Ucode;        // unsigned 16bit. unicode character
typedef signed long               G_Sint32;       // signed 32bit type
typedef signed short              G_Sint16;       // signed 16bit type
typedef float                     G_Float;          // floating point

#if	1	//insert by hyun
typedef int				BOOL;		 // boolean
typedef void				VOID;		 // void
typedef unsigned short		WORD;		 // unsigned 16bit
typedef unsigned int		DWORD;		 // unsigned 32bit
#endif

#undef NULL
#define NULL                      0

#define TRUE                      1
#define FALSE                     0

//#define inline                    __inline

#define G_SUCCESS                	 0
#define G_ERROR                  	-1
#define G_E_INVALID_DAB          	-2
#define G_ERROR_SSPERROR         	-3
#define G_ERROR_ID               	-101
#define G_ERROR_DEVICEID         	-102
#define G_ERROR_TIMEOUT_SCAN    	-103
#define G_ERROR_MEMORY_INSTART   	-104
#define G_ERROR_FIRMWARE_NOTFOUND   -105
#define G_ERROR_IRQ_ERROR			-106



/* ============================================================================================
                                    C O M M O N S
============================================================================================ */
typedef enum
{
  HWAIT_DOWN_DIRECT = 0,
  HWAIT_DOWN_3_CYCLE_LATER = 1,
  
  HWAIT_DOWN_MAX
} hwait_down_e_type;

typedef enum
{
  HOST_INT_POSITIVE_ASSERT = 0,
  HOST_INT_NEGATIVE_ASSERT = 1,
  
  HOST_INT_MAX
} host_int_e_type;

typedef enum
{
  SPI_CLK_ILDE_LOW = 0,
  SPI_CLK_ILDE_HIGH = 1,
  
  SPI_CLK_POLARITY_MAX
} spi_clk_polarity_e_type;

typedef enum
{
  SPI_CLK_LEADIING_EDGE = 0,
  SPI_CLK_TRAILING_EDGE = 1,
  
  SPI_CLK_PHASE_MAX
} spi_clk_phase_e_type;

typedef enum 
{
  EXT_CLK_65DOT536 = 0,
  EXT_CLK_32DOT768 = 1,
  EXT_CLK_16DOT384 = 2,
  EXT_CLK_8DOT192 = 3,
  EXT_CLK_4DOT096 = 4,
  EXT_CLK_2DOT048 = 5,
  EXT_CLK_1DOT024 = 6,

  EXT_CLK_MAX,
  EXT_CLK_OFF = 0xff,
} ext_clk_e_type ;

typedef enum 
{
  DM_DISABLED_MODE = 0,
  DM_FULL_MODE = 1,
  DM_SHORT_MODE = 2,
  DM_MSG_MODE = 3,
  
  DM_MODE_MAX
} dm_mode_e_type;

typedef enum {
  CAL_MODE_NORMAL = 0x00,
  CAL_MODE_0  = 0x01,
  CAL_MODE_1 = 0x02,
  CAL_MODE_2 = 0x03,

  CAL_MODE_MAX
} cal_mode_e_type;

typedef enum {
  GDM_CHIP_VER_R0,
  GDM_CHIP_VER_R0C,
  GDM_CHIP_VER_R1A,

  GDM_CHIP_VER_MAX
} gdm_chip_ver_e_type;

/* ============================================================================================
                                     T D M B S
============================================================================================ */

/* FIC data size */
#define FIC_DATA_SIZE                   384 /* 384 bytes */

typedef enum
{
  DAB_SYS_RUN_CMD = 0,
	DAB_SYS_SCAN_CMD = 1,
  DAB_SYS_STOP_CMD = 2,
  DAB_SYS_RESET_CMD = 3,

  DAB_SYS_CMD_MAX
} dab_sys_cmd_e_type;

typedef enum
{
  DAB_DEMOD_NOT_READY_S = 0xf,
  DAB_DEMOD_STANDBY_S = 0x0,
  DAB_DEMOD_ACTIVATE_S = 0x1,
  DAB_DEMOD_ACQUISITION_S = 0x2,
  DAB_DEMOD_FIC_PARSING_S = 0x3,
  DAB_DEMOD_DECODING_S = 0x4,
} dab_demod_status_e_type;

typedef enum
{
  DAB_SYNC_NO_MEANING = 0xf,
  DAB_SYNC_NO_FAILURE = 0x0,
  DAB_SYNC_FAILURE = 0x1
} dab_sync_status_e_type;


typedef enum
{
  DAB_0_PATH = 0,
  DAB_1_PATH = 1,

  DAB_PATH_MAX
} dab_path_e_type;

typedef enum 
{
  DAB_BAND_KOREAN_TDMB = 0,
  DAB_BAND_EUROPEAN_DAB = 1,
  DAB_BAND_EUROPEAN_DAB_L_BAND = 2,
  
  DAB_BAND_MAX
} dab_band_e_type;

typedef enum
{
  DAB_MODE_1 = 0,
  DAB_MODE_2 = 1,
  DAB_MODE_3 = 2,
  DAB_MODE_4 = 3,
  
  DAB_MODE_MAX
} dab_mode_e_type;

typedef enum
{
  DAB_FREQ_MODE_PREDEFINED_CH_SELECTOR = 0,
  DAB_FREQ_MODE_BAND_III_FREQ_CALCULATOR = 1,
  DAB_FREQ_MODE_L_BAND_FREQ_CALCULATOR = 2,
  
  DAB_FREQ_MODE_MAX
} dab_freq_mode_e_type;

typedef enum
{
  DAB_ENCAP_DM_DISABLE = 0,
  DAB_ENCAP_DM_ENABLE = 1,
  
  DAB_ENCAP_DM_MAX
} dab_encap_dm_e_type;

typedef enum
{
  DAB_ENCAP_FIC_DISABLE = 0,
  DAB_ENCAP_FIC_ENABLE = 1,
  
  DAB_ENCAP_FIC_MAX
} dab_encap_fic_e_type;

#if 0
typedef enum
{
  DAB_ENCAP_BUF_FIC = 0,
  DAB_ENCAP_BUF_TS,
  DAB_ENCAP_BUF_NTS,
  DAB_ENCAP_BUF_DM,
  
  DAB_ENCAP_BUF_MAX
} dab_encap_buf_e_type;
#endif

/* ============================================================================================
                                     H A L S
============================================================================================ */

typedef struct 
{
  G_Int8 (*read_reg)(G_Uint8 addr, G_Uint16* read_buf_ptr);
  G_Int8 (*burst_read_reg)(G_Uint8 addr, G_Uint8* read_buf_ptr, G_Uint32 size);
  G_Int8 (*write_reg)(G_Uint8 addr, G_Uint16 data);
  G_Int8 (*cr_read)(G_Uint16 addr, G_Uint16* read_buf_ptr);
  G_Int8 (*cr_write)(G_Uint16 addr, G_Uint16 data);
  G_Int8 (*mm_read)(G_Uint32 addr, G_Uint32* read_buf_ptr);
  G_Int8 (*mm_write)(G_Uint32 addr, G_Uint32 data);
  G_Int8 (*mm_burst_read)(G_Uint32 addr, G_Uint8* read_buf_ptr,  G_Uint32 size);
  G_Int8 (*mm_burst_write)(G_Uint32 addr, G_Uint8* write_buf_ptr, G_Uint32 size);
  void (*debug_msg)(const char * fmt,...);
} gdm_hal_func_s_type;

typedef enum 
{
  HAL_BIG_ENDIAN = 0,
  HAL_LITTLE_ENDIAN = 1,
  
  HAL_ENDIAN_MAX
} hal_endian_e_type;


/* ============================================================================================
                                     F I C
============================================================================================ */
#define	MAX_DAB_NUM		2
#define	MAX_SERVICE_NUM		10	//16
#define	MAX_SUBCH_NUM		10	//32
#define	MAX_SC_NUM	1

typedef struct{
	G_Uint32 mjd;			///< Modified Julian Date
	G_Uint8	lsi;
	G_Uint8	conf_ind;
	G_Uint8	utc_flag;
	G_Uint8	hour;
	G_Uint8	minute;
	G_Uint8	sec;
	G_Uint8	millisec;
} gdm_fic_data_time_s_type;

typedef struct {
	G_Uint16	sta_add;	///<Start Address(CU단위)
	G_Uint8	sch_id;		///<SubCh ID[5:0]
	G_Uint8	sl_fg;		///<Short, Long Form
	G_Uint8	table_sw;	///<UEP Table Switch
	G_Uint8	idx;			///<UEP Table Index
	G_Uint16	sch_sz;		///<EEP SubCh Size(CU단위)
	G_Uint16	sch_n;
	G_Uint16	bit_rate;	///<Bit Rate
	G_Uint8	prt_lvl;		///<EEP Protection Level
	G_Uint8	opt;			///<EEP option
	G_Uint8	ts_flag;		///<if TS, set '1' else '0'
	G_Uint8	tmid_scty;	///<tmid_scty
	G_Uint8	tmid_match_flag;
} gdm_fic_sch_s_type;

typedef struct {
	G_Uint8 no;
	G_Uint8 tmid;
	G_Uint16 scid;
	G_Uint8 scty;			///< DSCTy or ASCTy or Upper 6bit of scid
	G_Uint8 ps;
	G_Uint8 ca_flag;
	G_Uint8 sc_status;		///< 3bits is valid. 
	G_Uint8 label[16];
	G_Uint8 xpad_type;
	gdm_fic_sch_s_type	sch;
	G_Uint8 scty_tmp;
} gdm_fic_sc_s_type;

typedef struct {
	G_Uint8 num_of_set;						//Num of selected svc or sch
	G_Uint8 sch_id[MAX_SERVICE_NUM];
	G_Uint32	svc_id[MAX_SUBCH_NUM];
} gdm_fic_selected_sch_s_type;			


typedef struct {
	G_Uint8 pd;
	G_Uint8 svc_status;				///< 2bits is valid. 
	G_Uint32 sid;
	G_Uint8 num_of_svc_comp;
	G_Uint8 label[16];
	gdm_fic_sc_s_type sc[MAX_SC_NUM];
} gdm_fic_svc_s_type;

typedef struct {
	G_Uint8 change_flag;
	G_Uint8 al_flag;
	G_Uint8 esb_status;				///< 2bits is valid.
	G_Uint8 occur_change;
	G_Uint16	eid;
	G_Uint16	cif_cnt;
	G_Uint8 ensemble_label[16];
	G_Uint16	num_of_svc;
	G_Uint16	num_of_sch;
	G_Uint16	num_of_svc_label_end;			///< num of Services which are label found
	G_Uint16	num_of_svc_label_before;		///< num of Services of label founding	
	gdm_fic_svc_s_type svc[MAX_SERVICE_NUM];
} gdm_fic_ensbl_s_type;

typedef struct {
	G_Int32 num_of_ts;
	G_Uint8 sch_id[5];
} gdm_fic_ts_sch_s_type;

typedef struct{
	G_Int32 num_of_packet;
	G_Uint32 sid[5];
} gdm_fic_packet_s_type;

typedef struct{
	G_Uint8 fic_crc_err;			//CRC Err cnt : max 12
	G_Uint8 fic_crc_good;			//CRC Good cnt : max 12
} gdm_fic_crc_result_s_type;

typedef struct{
	G_Uint8 sch_id[MAX_SUBCH_NUM];
	G_Int32 check_cnt;
} gdm_fic_target_sch_id_s_type;

typedef struct{
  G_Int16 agc_gain; /// RSSI signed
  G_Uint16 agc_ref; /// AGC refrence point in dB scale
  G_Int16 afc_fine; /// fine frequency offset in unit of Hz
  G_Int16 afc_int; /// Integral frequency offset unit of sub-carrier space
  G_Uint16 demod_state; /// Demodulator status
  G_Uint16 reserved1[9];
  G_Uint16 unc_bit_err;
  G_Uint16 unc_sym_size;
  G_Uint16 fib_crc_error; /// CRC error of consecutive 12 FIBs 11:0 (each bit)
  G_Uint16 nts_bit_err; /// N of bit error during ber_nts_num
  G_Uint16 ber_nts_num; /// number of bits for NTS bit error monitor. unit:1024
  G_Uint16 nts_test_pattern; /// test pattern for performance estimation of NTS
  G_Uint16 ts_bit_err[2]; /// TS0/1 bit error during ber_ts_num
  G_Uint16 ts_byte_err[2]; /// TS0/1 byte error during ber_ts_num
  G_Uint16 ts_pkt_err[2]; /// TS0/1 packet error during ber_ts_num
  G_Uint16 ber_ts_num[2]; /// number of TS for TS error rate monitoring
  G_Uint16 cqm_ch_s; /// Relative signal power at the input of Viterbi decoder
  G_Uint16 cqm_ch_n; /// Relative noise power at the input of Viterbi decoder
  G_Uint16 antenna; /// report antenna level(0~6)
  G_Uint16 reserved2[1];
  G_Int16 path_idx[12]; 
    /// Path delay information of chahhel profile(unit: FFT sampling time 488ns)
  G_Uint16 path_pwr[12]; /// path power information of channel profile
  G_Uint16 reserved3[7];
  G_Uint16 modem_state; /// Modem status(internal use only)
} __attribute__((packed)) t_tdmb_Dm;

struct t_tdmb_ShortDm {
  G_Int16 agc_gain; /// AGC gain value RSSI=COEF1*this-COEF2
  G_Uint16 nts_bit_err; /// N of bit error during ber_nts_num
  G_Uint16 ts_bit_err[2]; /// TS0/1 bit error during ber_ts_num
  G_Uint16 ts_pkt_err[2]; /// TS0/1 packet error during ber_ts_num
  G_Uint16 cqm_ch_s; /// Relative signal power at the input of Viterbi decoder
  G_Uint16 cqm_ch_n; /// Relative noise power at the input of Viterbi decoder
  G_Uint16 ber_ts_num[2]; /// number of TS for TS error rate monitoring
};

#endif /* _GDM_TYPES_H */
