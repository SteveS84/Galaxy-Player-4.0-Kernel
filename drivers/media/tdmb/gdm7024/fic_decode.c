/* ============================================================================================



                             FIC Decode






 This file execute all about FIC Decoding. 







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                  C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Mar.20.2007      Jake Han     Created
-------------------------------------------------------------------------------------------- */



/* ============================================================================================
                                 I N C L U D E     F I L E S
============================================================================================ */

#include "gdm_headers.h" 
#include "fic_decode.h"

#include <linux/string.h>


#if !(defined(FEATURE_NEXTV_DAL_API))
/* ============================================================================================
                                     D E F I N I T I O N S
============================================================================================ */

//#define FIC_DECODER_LIB

#ifdef LOCAL
#undef LOCAL
#endif
#define LOCAL static

#ifdef FIC_DECODER_LIB
#define PRINTF do{}while(0);
#else
#define PRINTF printf
#endif

#define MAX_FIC_SIZE      (12 * 32)


/*=== External ====*/

void	gdm_fic_check_ensemble(G_Int32 DAB_no);
void gdm_fic_init_db(G_Int32 DAB_no);
G_Int32 gdm_fic_run_decoder(G_Int32 DAB_no, G_Uint8 *fib_buff, G_Int32 fib_buff_size);
G_Uint32 gdm_fib_decode(G_Int32 DAB_no,G_Uint8 *fib_buff,G_Int32 fib_buff_size);//, G_Int32 *sch_tot_num);

typedef struct {
  G_Uint8   buf[MAX_FIC_SIZE];
  G_Uint16  wr_pos;
} gdm_fic_buf_s_type;

typedef struct{
	G_Uint8 cFIGType;
	G_Uint8 cFIGLength;
	G_Uint8 cFIGData[29];
} FIG;

typedef struct {
	G_Uint8	ECC;	
	G_Uint8	sid[16];
	
} t_IntntnlTbl_ExtField_Services;

typedef struct{
	G_Uint8	num_of_svc;
	G_Uint8	LTO;
	t_IntntnlTbl_ExtField_Services IntntnlTbl_ExtFeld_Svcs[16];
} t_IntntnlTbl_ExtField;

typedef struct{
	G_Uint8	ExtFlg;	
	G_Uint8	LTO_Unq;
	G_Uint8	EnsblLTO;
	G_Uint8	EnsblECC;

	G_Uint8	InterTableID;
	t_IntntnlTbl_ExtField IntntnlTbl_ExtField[16];
} t_InternationalTable;



/*=== Local ====*/

LOCAL FIG fig_list;

LOCAL G_Uint32 get_bit_rate(gdm_fic_sch_s_type * sch_ptr);
LOCAL void	fig0_0(G_Int32 DAB_no );
LOCAL void	fig0_1(G_Int32 DAB_no , G_Int32 *sch_tot_num );
LOCAL void	fig0_2(G_Int32 DAB_no , G_Uint8 PDFlag);
LOCAL void	fig0_3(G_Int32 DAB_no );
LOCAL void	fig0_5(G_Int32 DAB_no );
LOCAL void	fig0_8(G_Int32 DAB_no , G_Uint8 PDFlag);
LOCAL void	fig0_9(G_Int32 DAB_no );
LOCAL G_Uint8	fig0_10(G_Int32 DAB_no );
LOCAL void	fig0_13(G_Int32 DAB_no , G_Uint8 PDFlag);
LOCAL void	put_in_label(G_Uint8 *label );
LOCAL void	fig1_0(G_Int32 DAB_no );
LOCAL void	fig1_1(G_Int32 DAB_no );
LOCAL void	fig1_4(G_Int32 DAB_no );
LOCAL void	fig1_5(G_Int32 DAB_no );
LOCAL void	fig1_6(G_Int32 DAB_no );
LOCAL void fig5(G_Int32 DAB_no );
LOCAL void fig6(G_Int32 DAB_no );
LOCAL G_Uint32 fig_decode(G_Int32 DAB_no, G_Int32 *sch_tot_num);
LOCAL G_Uint32   check_duplicate_sch(G_Int32 DAB_no,G_Uint8   sch_id, G_Int32 sch_num);
LOCAL G_Uint8   check_duplicate_svc(G_Int32 DAB_no,G_Uint32   sid, G_Int32 num_of_svc);
LOCAL G_Uint8   check_duplicate_svc_comp(gdm_fic_svc_s_type *svc_ptr, G_Uint8 tmid, G_Uint32 Uniqe_id);
LOCAL G_Uint16 crc16(G_Uint8 *buf);
LOCAL G_Uint32 get_field(G_Uint8 sizebit );
LOCAL void match_svc2sch(G_Int32 DAB_no);
LOCAL void gdm_memcpy( void *dst, const void *src, G_Int32 size );



/* ============================================================================================
                                       V A R I A B L E S 
============================================================================================ */
/*=== External ====*/

gdm_fic_ts_sch_s_type gdm_fic_ts_buf[MAX_DAB_NUM];		///< Save TS sub ch ID
gdm_fic_ensbl_s_type gdm_fic_ensbl[MAX_DAB_NUM];		///< Ensemble Structure for DAB0/DAB1
gdm_fic_sch_s_type gdm_fic_sch[MAX_DAB_NUM][MAX_SUBCH_NUM];	///< Sub Channel Structure
gdm_fic_selected_sch_s_type gdm_fic_selected_sch[MAX_DAB_NUM];
G_Int32	gdm_fic_cnt[MAX_DAB_NUM];
gdm_fic_crc_result_s_type	gdm_fic_crc_result[MAX_DAB_NUM];

gdm_fic_target_sch_id_s_type	TDMB_target[MAX_DAB_NUM];
gdm_fic_data_time_s_type	date_time[MAX_DAB_NUM];




/*=== Local ====*/

LOCAL G_Uint8 gdm_fic_user_status[2] = {0,0};
LOCAL G_Uint8 bit_pos;
LOCAL t_InternationalTable international_table[2];
LOCAL G_Uint8	sch_devide_value[8] = {12,8,6,4,27,21,18,15};
LOCAL G_Uint8	bit_rate_value[64] = 
{
8,            //case  0: BitRate   =  32; break;
8,            //case  1: BitRate   =  32; break;
8,            //case  2: BitRate   =  32; break;
8,            //case  3: BitRate   =  32; break;
8,            //case  4: BitRate   =  32; break;
12,            //case  5: BitRate   =  48; break;
12,            //case  6: BitRate   =  48; break;
12,            //case  7: BitRate   =  48; break;
12,            //case  8: BitRate   =  48; break;
12,            //case  9: BitRate   =  48; break;
14,           //case 10: BitRate   =  56; break;
14,            //case 11: BitRate   =  56; break;
14,            //case 12: BitRate   =  56; break;
14,            //case 13: BitRate   =  56; break;
16,            //case 14: BitRate   =  64; break;
16,           //case 15: BitRate   =  64; break;
16,            //case 16: BitRate   =  64; break;
16,            //case 17: BitRate   =  64; break;
16,            //case 18: BitRate   =  64; break;
20,            //case 19: BitRate   =  80; break;
20,            //case 20: BitRate   =  80; break;
20,            //case 21: BitRate   =  80; break;
20,           //case 22: BitRate   =  80; break;
20,            //case 23: BitRate   =  80; break;
24,            //case 24: BitRate   =  96; break;
24,            //case 25: BitRate   =  96; break;
24,            //case 26: BitRate   =  96; break;
24,            //case 27: BitRate   =  96; break;
24,            //case 28: BitRate   =  96; break;
28,            //case 29: BitRate   = 112; break;
28,            //case 30: BitRate   = 112; break;
28,            //case 31: BitRate   = 112; break;
28,            //case 32: BitRate   = 112; break;
32,            //case 33: BitRate   = 128; break;
32,            //case 34: BitRate   = 128; break;
32,            //case 35: BitRate   = 128; break;
32,            //case 36: BitRate   = 128; break;
32,            //case 37: BitRate   = 128; break;
40,            //case 38: BitRate   = 160; break;
40,            //case 39: BitRate   = 160; break;
40,            //case 40: BitRate   = 160; break;
40,            //case 41: BitRate   = 160; break;
40,            //case 42: BitRate   = 160; break;
48,            //case 43: BitRate   = 192; break;
48,            //case 44: BitRate   = 192; break;
48,            //case 45: BitRate   = 192; break;
48,            //case 46: BitRate   = 192; break;
48,            //case 47: BitRate   = 192; break;
56,            //case 48: BitRate   = 224; break;
56,            //case 49: BitRate   = 224; break;
56,            //case 50: BitRate   = 224; break;
56,            //case 51: BitRate   = 224; break;
56,            //case 52: BitRate   = 224; break;
64,            //case 53: BitRate   = 256; break;
64,            //case 54: BitRate   = 256; break;
64,            //case 55: BitRate   = 256; break;
64,            //case 56: BitRate   = 256; break;
64,            //case 57: BitRate   = 256; break;
80,            //case 58: BitRate   = 320; break;
80,            //case 59: BitRate   = 320; break;
80,            //case 60: BitRate   = 320; break;
96,            //case 61: BitRate   = 384; break;
96,           //case 62: BitRate   = 384; break;
96,            //case 63: BitRate   = 384; break;
};	

LOCAL gdm_fic_buf_s_type gdm_fic_buf[MAX_DAB_NUM];




/* ============================================================================================
                                       F U N C T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  gdm_fic_check_ensemble



DESCRIPTION
           
           This function checks the running condition of FIC Decoding.
           If you want to change the condition of completing status of FIC decoding, 
           you can modify this function.

           There is a fic decoding status check variable which name is esb_status(initial of ensenble status).
           The esb_status has bit allocated values and currenlty 1 bit is valid.

           Basically, when we get service labels as many as services (num_of_svc  ==num_of_svc_label)
           we decide that fic decode is completed and set the 1st bit of esb_status.


PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] num_of_svc(gdm_fic_ensbl)
           [in] num_of_svc_label_end(gdm_fic_ensbl)
           [in/out] esb_status(gdm_fic_ensbl).



RETURN VALUE


-------------------------------------------------------------------------------------------- */
void gdm_fic_check_ensemble(G_Int32 DAB_no)
{
  G_Int32 i = 0, k = 0, j = 0, n = 0;
  gdm_fic_ensbl_s_type  *ensbl_ptr;
  
  static  G_Int32 CheckEsbCnt = 0;
  static  G_Int16 Packet_Temp0,Packet_Temp1,Packet_Temp2;
  ensbl_ptr = &gdm_fic_ensbl[DAB_no];

  if(!(ensbl_ptr->esb_status & 0x1)
     && (ensbl_ptr->num_of_svc > 0)
     && (ensbl_ptr->num_of_svc == ensbl_ptr->num_of_svc_label_end))
  {
    ensbl_ptr->esb_status |= 1; ///<Data Enough !! No more Svc& Subchannel Find
    return ;
  }
  else if((gdm_fic_cnt[DAB_no] > 30)
          && (ensbl_ptr->num_of_svc > 0)
          && (ensbl_ptr->num_of_svc == ensbl_ptr->num_of_sch))
  {
    ensbl_ptr->esb_status = 3; ///<Data Enough !! No more Svc& Subchannel Find. No more label Update
  }
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  gdm_fic_run_decoder



DESCRIPTION
           
			This function sends fic raw data to fib_decode function and checks the result of FIC decoding 
			and call gdm_fic_check_ensemble. 
			There is a variable which name is gdm_fic_user_status. 
			If you want to skip FIC decoding after a certain status of Fic decoding(ex. after FIC decoding complete) 
			you can use this variable.
						

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] fib_buff( fic raw data ).
           [in] fib_buff_size( data size : this should be multiple of 32, in MODE 1 this is usually 384 ).
           [in/out] esb_status(gdm_fic_ensbl).
           [in/out] gdm_fic_user_status.
           [out] gdm_fic_ts_buf.num_of_ts.

RETURN VALUE

			esb_status( gdm_fic_ensbl )
-------------------------------------------------------------------------------------------- */

G_Int32 gdm_fic_run_decoder(G_Int32 DAB_no, G_Uint8 *fib_buff,  G_Int32 fib_buff_size)
{
  gdm_fic_ensbl_s_type	*ensbl_ptr;

  ensbl_ptr = &gdm_fic_ensbl[DAB_no];

  memcpy(&gdm_fic_buf[DAB_no].buf[gdm_fic_buf[DAB_no].wr_pos], fib_buff, fib_buff_size);
  gdm_fic_buf[DAB_no].wr_pos += fib_buff_size;

  if(gdm_fic_buf[DAB_no].wr_pos >= MAX_FIC_SIZE)
  {
    gdm_fic_ts_buf[DAB_no].num_of_ts = 0;

    if(gdm_fib_decode(DAB_no, &gdm_fic_buf[DAB_no].buf[0], gdm_fic_buf[DAB_no].wr_pos))
    {
      if(gdm_fic_user_status[DAB_no] < 1)
      {
        gdm_fic_check_ensemble(DAB_no);
        if(ensbl_ptr->esb_status == 0x03)
        {
	  gdm_fic_user_status[DAB_no] = 1;
        }
      }
    }
    gdm_fic_buf[DAB_no].wr_pos = 0;
  }
	
	return ensbl_ptr->esb_status;
}
/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  gdm_fib_decode



DESCRIPTION
           
           This function is got fic raw data, check CRC by FIB unit and after CRC check, 
           build FIG structure, eventually, execute fig_decode.
           When fig_decode function is running, it fills up the structures of service and sub channel separately.
           So, after fig_decode is done, match services and sub channels by executing match_svc2sch.
           

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] fib_buff( fic raw data ).
           [in] fib_buff_size( data size : this should be multiple of 32, in MODE 1 this is usually 384 ).
           [out] gdm_fic_crc_result.
           [out] FIG structure.


RETURN VALUE
			gdm_fic_crc_result - CRC Error/Good result
-------------------------------------------------------------------------------------------- */
G_Uint32 gdm_fib_decode(G_Int32 DAB_no,G_Uint8 *fib_buff,  G_Int32 fib_buff_size)//, G_Int32 *sch_tot_num)
{
	G_Int32 iFIGListLength;
	G_Int32 i, j;
	//G_Uint32	RegValue;
	G_Int32 sch_tot_num = 0;
	gdm_fic_ensbl_s_type	*ensbl_ptr;
	gdm_fic_ts_sch_s_type		*TsSch;
	gdm_fic_sch_s_type		*sch_ptr;
	G_Uint8	*RawBuff;
	
	ensbl_ptr = &gdm_fic_ensbl[DAB_no];
	iFIGListLength = 0;
	if(fib_buff_size > 384)
		fib_buff_size = 384;

	gdm_fic_crc_result[DAB_no].fic_crc_err = 0;
	gdm_fic_crc_result[DAB_no].fic_crc_good = 0;

	for(i=0;i<fib_buff_size/32;i++)	
	{
		if((((G_Uint16)fib_buff[(i*32)+30]<<8) |
		      (G_Uint16)fib_buff[(i*32)+31]) !=  crc16(fib_buff+(i*32)))
	   {
			gdm_fic_crc_result[DAB_no].fic_crc_err ++;
		}
		else
		{
			for(j=0;;)
			{
				if(j>29)	break;

				RawBuff = (fib_buff + (i*32)+j);
				if(*RawBuff != 0xff){
					fig_list.cFIGType = (*RawBuff & 0xe0)>>5;
					fig_list.cFIGLength = *RawBuff & 0x1f;
					// Length can't be 0,30,31 except End maker's 31
					gdm_memcpy(fig_list.cFIGData, &(fib_buff[(i*32)+j+1]), fig_list.cFIGLength);
					j+= fig_list.cFIGLength+1; // type,size
				
					fig_decode(DAB_no,  &sch_tot_num);
					gdm_fic_crc_result[DAB_no].fic_crc_good ++;
				}
				else
					break;
			}
		}

	}

	j = gdm_fic_crc_result[DAB_no].fic_crc_good;
	//====	After handle	====================
	if(j)
	{	// if there is more than one fig_decode execute.
		if(ensbl_ptr->num_of_sch <= sch_tot_num)				
			ensbl_ptr->num_of_sch = sch_tot_num;					
		if(!(ensbl_ptr->esb_status & 0x01))
		{	
			//==== Find TS =====
			TsSch = &gdm_fic_ts_buf[DAB_no];
			for(i = 0; i < ensbl_ptr->num_of_sch ; i++)
			{
				sch_ptr = &gdm_fic_sch[DAB_no][i];
				for(j = 0; j < TsSch->num_of_ts ; j++)
				{
					if(sch_ptr->sch_id == TsSch->sch_id[j])
						sch_ptr->ts_flag = 1;
				}
			}
			//===============
			match_svc2sch(DAB_no);
		}
	}
	//==================================
	gdm_fic_cnt[DAB_no]++;					 		
	if(gdm_fic_cnt[DAB_no] >= 8000)
	   gdm_fic_cnt[DAB_no] = 10;

	return gdm_fic_crc_result[DAB_no].fic_crc_good;		// return valid fig_decode counter

}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  fig_decode



DESCRIPTION
           
           This function is executing FIG parsing.
           FIGs are parsed to each Types and extensions.

           When fig_decode function is running, it fills up the structures of service and sub channel separately,
			and keeps the total sub channel number(sch_tot_num).           

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] sch_tot_num.
           [in/out] Enseble/Service/sub channel structure.


RETURN VALUE
			Not fixed yet
-------------------------------------------------------------------------------------------- */

G_Uint32 fig_decode(G_Int32 DAB_no, G_Int32 *sch_tot_num)
{
	G_Uint32	TmpInt0;
	G_Uint8	TmpChar0;
	G_Uint8 PDFlag;
	
	gdm_fic_ensbl_s_type	*ensbl_ptr;

	ensbl_ptr = &gdm_fic_ensbl[DAB_no];
	switch (fig_list.cFIGType){		///<====	FIG Type		====
		case 0: ///<	+ FIG Type 0 : MCI and part of the SI
			bit_pos = 0;
			TmpChar0 = get_field(1) & 0x1;		//C/N
			TmpChar0 = get_field(1) & 0x1;		//OE
			if(TmpChar0)						// Other Ensemble
				break;
			PDFlag = get_field(1);
			TmpChar0 = get_field(5) & 0x1F;	//FIG_Ext
				
			switch(TmpChar0)
			{		//	=	FIG Extension		=
				case 0: 			///<==== FIG 0/0 - Ensemble information
				  fig0_0(DAB_no);
				  break;					
				case 1: 			///<==== FIG 0/1 - Basic Sub-Channel Organization
				  fig0_1(DAB_no,sch_tot_num );
				  break;
				case 2:			 ///<==== FIG 0/2 - Basic Service Organization
				  if(ensbl_ptr->esb_status & 0x1) break;
				  fig0_2(DAB_no, PDFlag);
				break;
				case 3:			 ///<==== FIG 0/3 - Additional information about the service component description in packet mode.
				  if(ensbl_ptr->esb_status & 0x1) break;
				  fig0_3(DAB_no);
				  break;
				case 4: 			///<==== FIG 0/4 - Service component with Conditional Access in Stream mode for FIC.
				  break;
				case 5:			///<==== FIG 0/5 - Service Component Language
				  fig0_5(DAB_no);
				  break;
				case 8: 			///<==== FIG 0/8 - Service Component Global Definition
				  if(ensbl_ptr->esb_status & 0x1)	break;
				    fig0_8(DAB_no, PDFlag);
				  break;
				case 9: ///<==== FIG 0/9 - Country, LTO and Internation Table
				  fig0_9(DAB_no);
				  break;							
				case 10: ///<==== FIG 0/10 - Date And Time
				  fig0_10(DAB_no);
				  break;					
				case 13: ///<==== FIG 0/13 - -User Application Information 
				  fig0_13(DAB_no, PDFlag);
				  break;
				default:
				  break;
			}
		break;
		//###     FIG Type 1	############################################################################
		case 1: ///<	+ FIG Type 1 :  Labels, etc. (part of the SI)
			bit_pos=0;
			TmpInt0	= (G_Uint8)(get_field(4) & 0xf);	//Charset.
			if(TmpInt0 == 4 || TmpInt0 == 6)
			{
				break;
			}
				
			TmpChar0= get_field(1);	//OE
			if(TmpChar0)
			{
				break;
			}
			TmpChar0= get_field(3) & 0x1F;		// FIG_Ext
				
			switch(TmpChar0)	
			{	// FIG_Ext
				case 0: ///<==== FIG 1/0 - Ensemble label
					if(ensbl_ptr->esb_status & 0x2) break;
						fig1_0(DAB_no);
					break;
				case 1: ///<==== FIG 1/1 - Programme Service label
					if(ensbl_ptr->esb_status & 0x1) break;
						fig1_1(DAB_no);
					break;
				case 4: ///<==== FIG 1/4 - Service Component label
					if(ensbl_ptr->esb_status & 0x1) break;
					fig1_4(DAB_no);

					break;
				case 5: ///<==== FIG 1/5 - Data Service label
					if(ensbl_ptr->esb_status & 0x1) break;
					fig1_5(DAB_no);
		
					break;
				case 6: ///<==== FIG 0/6 - X-Pad User Application label
					if(ensbl_ptr->esb_status & 0x1) break;
					fig1_6(DAB_no);
					break;
			}
			break;
		//###     FIG Type 2	############################################################################
		case 2: ///<+ FIG Type 2 : Labels, etc. (part of the SI) -We don't Support
		  break;

		case 5: ///<+ FIG Type 5 : FIDC
			bit_pos=0;
			if(ensbl_ptr->esb_status) break;
			fig5(DAB_no);

		  break;
		case 6: ///<+ FIG Type 6 : CA
			bit_pos=0;
		  break;
		
	  default:
		break;
				
	}
	return 0x0;
}

void	fig0_0(G_Int32 DAB_no ){

	gdm_fic_ensbl_s_type	*ensbl_ptr;
	ensbl_ptr = &gdm_fic_ensbl[DAB_no];		
	ensbl_ptr->eid = get_field(16) & 0xffff;	//eid
	ensbl_ptr->change_flag = get_field(2);
			
	if(ensbl_ptr->change_flag == 0x00)
	{
		if(fig_list.cFIGLength < 0x05 || fig_list.cFIGLength > 0x06){
			return;
		}
		ensbl_ptr->al_flag = get_field(1);
		ensbl_ptr->cif_cnt = get_field(13);
		ensbl_ptr->occur_change = 0;
	}
	else
	{
		if(fig_list.cFIGLength != 0x06)
		{
			return;
		}
		ensbl_ptr->al_flag = get_field(1);
		ensbl_ptr->cif_cnt = get_field(13);
		ensbl_ptr->occur_change = get_field(8);
	}
}

void	fig0_1(G_Int32 DAB_no , G_Int32 *sch_tot_num ){
	
	G_Uint8	sch_id, TmpChar0;
	G_Uint32	TmpInt0;
	gdm_fic_sch_s_type	*sch_ptr;
	
	do
	{
		if(*sch_tot_num > MAX_SUBCH_NUM)
			return;	// unusual case.. return

		sch_id = get_field(6);
		TmpInt0= get_field(10);		//StartAddress
		TmpChar0 =  get_field(1) & 0x1;	//LSFlag

		sch_ptr = &gdm_fic_sch[DAB_no][*sch_tot_num];
		sch_ptr->sch_id = sch_id;
		
		if(!check_duplicate_sch(DAB_no, sch_ptr->sch_id,*sch_tot_num))
		{ 
			sch_ptr->sta_add = TmpInt0;
			sch_ptr->sl_fg =  TmpChar0;		//LSFlag

			if(sch_ptr->sl_fg)
			{
				sch_ptr->opt  = get_field(3);
				sch_ptr->prt_lvl  = get_field(2);
				sch_ptr->sch_sz = get_field(10);
				if(sch_ptr->prt_lvl < 4)
				{
					if (sch_ptr->opt)
						sch_ptr->sch_n = sch_ptr->sch_sz/sch_devide_value[(G_Int32)sch_ptr->prt_lvl];
					else 
						sch_ptr->sch_n = sch_ptr->sch_sz/sch_devide_value[(G_Int32)sch_ptr->prt_lvl + 4];
				}
				else
					sch_ptr->sch_n = 0;
				
			}
			else	
			{	//Short Form
				sch_ptr->table_sw = get_field(1);
				sch_ptr->idx = get_field(6);
				sch_ptr->opt = 0;
				sch_ptr->prt_lvl = 0;
				sch_ptr->sch_n = 0;
				sch_ptr->sch_sz = 0;
			}
			sch_ptr->bit_rate = get_bit_rate(sch_ptr);
			(*sch_tot_num)++;

		}
		else
		{
			if(TmpChar0)	
			{				//Long Form
				TmpInt0 = get_field(15);	//pass (option/ProtectionLevel/SubChSize
			}
			else	
			{						//Short Form
				TmpChar0= get_field(1);
				if(TmpChar0)
					break;
				TmpChar0 = get_field(6);
			}
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);
}

void fig0_2(G_Int32 DAB_no , G_Uint8 PDFlag)
{
	G_Uint8	TmpChar0,TmpChar1, j;
	G_Uint8	sch_id = 0, PS_CAFlag = 0, scty = 0;
	G_Uint32 sid = 0;
	G_Uint32   temp_data = 0;

	gdm_fic_ensbl_s_type	*ensbl_ptr;
	gdm_fic_ts_sch_s_type		*TsSch;	

	G_Uint8	k = 0;
	static	 G_Int16	Packet_check_cnt0 = 0,Packet_check_cnt1 = 0;
	static	G_Int32	SID0;
	gdm_fic_sc_s_type  *sc_ptr;
	gdm_fic_svc_s_type    *svc_ptr;
	ensbl_ptr = &gdm_fic_ensbl[DAB_no];

	do
	{
		if(PDFlag == 0)	
		{// 16bit sid
			sid = get_field(16);
		}
		else	
		{	// 32bit sid
			sid = get_field(32);
			//if(((sid >> 24) & 0xF0 ) != 0xF0)
			//	continue;
		}

		TmpChar0 = get_field(1) & 0x1;	// LocalFlag
		TmpChar0 = get_field(3);		//CAId
		TmpChar1 = get_field(4);		//NumSvcComp
		//== Temporary fix sc num is always '1' =============
		if(TmpChar1 > 1){		//when NumSvcComp > 1
			break;
		}
		//==========================================
		for(j=0;j<TmpChar1;j++)
		{
			TmpChar0 = get_field(2);	//tmid

			if(TmpChar0 < 4){	//
				scty = get_field(6);		//ASCTy or DSCTy
				if(TmpChar0 == 0)
				{
					if(scty > 2)
						break;
				}
				else if(TmpChar0 == 1)
				{
					if(scty == 0 || (scty > 5 && scty <24) || (scty > 24 && scty <59) ||scty > 61)
						break;
				}
				sch_id = get_field(6);
				PS_CAFlag = get_field(1)<< 1 | get_field(1);		// PSFlag, CAFlag
				temp_data = scty << 6 | sch_id;
			}
			if((check_duplicate_svc(DAB_no, sid, ensbl_ptr->num_of_svc)) == 0xff)
			{
				ensbl_ptr->svc[ensbl_ptr->num_of_svc].sid = sid;
				svc_ptr = &ensbl_ptr->svc[ensbl_ptr->num_of_svc];
				svc_ptr->num_of_svc_comp = TmpChar1;		//NumSvcComp
				if(check_duplicate_svc_comp(svc_ptr, TmpChar0, temp_data))	// TmpChar0 = tmid
					break;
				sc_ptr = &svc_ptr->sc[j];		

				sc_ptr->no++;
				sc_ptr->tmid = TmpChar0;
				sc_ptr->ca_flag = PS_CAFlag & 0x1;
				if(TmpChar0 < 3)
				{
					sc_ptr->sch.sch_id = sch_id;
					sc_ptr->ps = PS_CAFlag >> 0x1;
					sc_ptr->scty = (temp_data & 0x3f);
					sc_ptr->scid = 0xf000 | sch_id;
					sc_ptr->sc_status |= 0x1;	// sch_id found;
				}
				else
				{
					sc_ptr->scid = temp_data;
					sc_ptr->sc_status &= 0x1;	// sch_id not found yet.
				}
				ensbl_ptr->num_of_svc++;

                sc_ptr->scty_tmp = scty;
			}

			//#######################################
			if((TmpChar0 == 1) && (scty == 0x18))             // If this condition, this channel is TS format.
			{		// tmid, DSCTy
				TsSch = &gdm_fic_ts_buf[DAB_no];
				TsSch->sch_id[TsSch->num_of_ts] = sch_id;
				TsSch->num_of_ts++;
			}
			//#######################################
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);
}

void fig0_3(G_Int32 DAB_no )
{
	G_Uint8	TmpChar0,TmpChar1, j, m, DoCnt = 0;
	G_Uint32	TmpInt0;
	G_Uint8	sch_id = 0, DSCTy = 0;
	G_Uint16  scid = 0;
	gdm_fic_sc_s_type  *sc_ptr;

	do
	{
		DoCnt++;
		//====================
		if(DoCnt > MAX_SERVICE_NUM)
			return;	// unusual case.. return
		//====================
		scid = get_field(12);
		TmpChar0 = get_field(3) & 0x7;	// Rfa
		if(TmpChar0)
			break;
		TmpChar1 = get_field(1);	//CAOrgFlg
		TmpChar0 = get_field(1);	//DGFlag
		TmpChar0  = get_field(1) & 0x1;	// Rfu
		if(TmpChar0)
			break;
		DSCTy  = get_field(6);	//
		sch_id  = get_field(6);	//
		TmpInt0 = get_field(10);	//Packet Addrress

		if(TmpChar1)
			TmpChar1 = get_field(16);	//CAOrg

		//#########################################################################
		for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
		{
			for(m = 0; m < gdm_fic_ensbl[DAB_no].svc[j].num_of_svc_comp; m++)
			{
				sc_ptr = &gdm_fic_ensbl[DAB_no].svc[j].sc[m];
				if(sc_ptr->scid == scid && !(sc_ptr->sc_status & 0x01))
				{
					sc_ptr->sch.sch_id = sch_id;
					sc_ptr->scty = DSCTy;
					sc_ptr->sc_status |= 0x01;
					break;
				}
			}
		}
		//#########################################################################
	}while((bit_pos>>3)<fig_list.cFIGLength);

}

void fig0_5(G_Int32 DAB_no )
{
	G_Uint8	TmpChar0, DoCnt = 0;
	G_Uint32	TmpInt0;
	G_Uint16  scid, Id = 0;

	do
	{
		DoCnt++;
		//====================
		if(DoCnt > MAX_SERVICE_NUM)
			return;	// unusual case.. return
		//====================
		TmpChar0 = get_field(1);			//LSFlag

		if(TmpChar0)	
		{	//Long Form
			TmpInt0 = get_field(3);
			scid = get_field(12);			//scid
			TmpChar0 = get_field(8);		//Language
		}
		else	
		{	//Short Form
			TmpChar0 = get_field(1);
			if(TmpChar0)	
			{
				//FIC & FIDCId
				Id = get_field(6);			//FIDCId
				TmpChar0 = get_field(8);	//Language
			}
			else	
			{
				//MSC & sch_id
				Id = get_field(6);			//sch_id
				TmpChar0 = get_field(8);	//Language
			}
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);
}


void fig0_8(G_Int32 DAB_no , G_Uint8 PDFlag)
{
	G_Uint8	TmpChar0, DoCnt = 0;
	G_Uint32	TmpInt0, TmpInt1, sid;
	G_Uint16  scid, Id = 0;
	do
	{
		DoCnt++;
		//====================
		if(DoCnt > MAX_SERVICE_NUM)
			return;	// unusual case.. return
		//====================
		if(PDFlag == 0)
		{	// 16bit sid
			sid = get_field(16);
		}
		else	
		{		// 32bit sid
			sid = get_field(32);
		}
		TmpChar0 = get_field(1) & 0x1;	//ExtFlag
		TmpInt0 = get_field(3);
		TmpInt1 = get_field(4);			//SCIdS

		TmpChar0= get_field(1);

		if(TmpChar0)
		{		// Long Form
			TmpInt0 = get_field(3);
			scid = get_field(12);
		}
		else	
		{			// Short Form
			TmpChar0 = get_field(1);
			if(TmpChar0)
			{	//FIC & FIDCId
				Id = get_field(6);
			}
			else	
			{		// MSC & sch_id
				Id = get_field(6);
			}
		}
		if(TmpChar0)		// ExtFlag
		{
			// 8bit Rfa field present
			TmpChar0 = get_field(8) & 0xFF;
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);	
}

void fig0_9(G_Int32 DAB_no)
{
	G_Uint8	DoCnt = 0;
	t_InternationalTable *IntntnTbl;
	do
	{							
		DoCnt++;
		//====================
		if(DoCnt > MAX_SERVICE_NUM)
			return;	// unusual case.. return
		//====================
		IntntnTbl = &international_table[DAB_no];
		
		IntntnTbl->ExtFlg = get_field(1);	// 
		IntntnTbl->LTO_Unq=get_field(1);	//
		IntntnTbl->EnsblLTO= get_field(6);	//
		IntntnTbl->EnsblECC= get_field(8);	//
		IntntnTbl->InterTableID= get_field(8);	//
		
		if(IntntnTbl->InterTableID == 1)
		{// PTY from Table 12 & Announcement type from Table 14
			
		}
		else{// RBDS PTY from Table 13 & Announcement type from Table 14

		}
		if(IntntnTbl->ExtFlg){
			
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);
}

G_Uint8 fig0_10(G_Int32 DAB_no)
{
	G_Uint8	TmpChar0;
	gdm_fic_data_time_s_type * Date_Time;

	Date_Time = &date_time[DAB_no];

	TmpChar0 = get_field(1);	// Rfa
	Date_Time->mjd  = get_field(17);	//
	Date_Time->lsi  = get_field(1);	//
	Date_Time->conf_ind  = get_field(1);	//
	Date_Time->utc_flag  = get_field(1);	//

	if(Date_Time->utc_flag)
	{
		TmpChar0 = get_field(5);
		if(TmpChar0 < 24)
			Date_Time->hour  = 	TmpChar0;//
		else
			return (G_Uint8)0xF9;
			
		TmpChar0 = get_field(6);
		if(TmpChar0 < 60) 
			Date_Time->minute  = TmpChar0;
		else
			return (G_Uint8)0xF9;
		Date_Time->sec  = get_field(8);	//
		Date_Time->millisec  = get_field(10);	//
	}
	else
	{
		TmpChar0 = get_field(5);
		if(TmpChar0 < 24)
			Date_Time->hour  = 	TmpChar0;//
		else
			return (G_Uint8)0xF9;
			
		TmpChar0 = get_field(6);
		if(TmpChar0 < 60) 
			Date_Time->minute  = TmpChar0;
		else
			return (G_Uint8)0xF9;
	}
	return 0;
}

void fig0_13(G_Int32 DAB_no , G_Uint8 PDFlag)
{
	G_Uint8	TmpChar0,j,m,TmpChar1, DoCnt= 0;
	G_Uint32	TmpInt0, TmpInt1, sid;

	do{
		DoCnt++;
		//====================
		if(DoCnt > MAX_SERVICE_NUM)
			return;	// unusual case.. return
		//====================
		if(PDFlag == 0)
			sid = get_field(16);
		else
			sid = get_field(32);

		TmpInt1 = get_field(4);		//SCIdS
		TmpChar0 = get_field(4);		//NumUserApp

		for(j=0;j<TmpChar0;j++)
		{
			TmpInt0 = get_field(11);	//UserAppType
			TmpChar1 = get_field(5);	//UserAppLength
			for(m = 0; m < TmpChar1 ; m++)
			{
				TmpChar0 = get_field(8);//
			}
		}
	}while((bit_pos>>3)<fig_list.cFIGLength);
}

void	put_in_label(G_Uint8 *label )
{
	G_Uint8 m;

	for(m = 0; m < 16; m++)
		*label++ = get_field(8);
}

void fig1_0(G_Int32 DAB_no )
{
	G_Uint32	TmpInt0;

	TmpInt0 = get_field(16);		//ID_Field, eid
	if(gdm_fic_ensbl[DAB_no].eid == TmpInt0){
		//====		label input	=============
		put_in_label(&gdm_fic_ensbl[DAB_no].ensemble_label[0]);
		TmpInt0 = get_field(16);	//CharFlagField
		gdm_fic_ensbl[DAB_no].esb_status |= 0x2;
		return;
	}
}

void fig1_1(G_Int32 DAB_no )
{
	G_Uint32	TmpInt0;
	G_Uint8	 j;
	
	TmpInt0 = get_field(16);		//ID_Field, sid
	for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
	{
		if(gdm_fic_ensbl[DAB_no].svc[j].sid == TmpInt0)
		{
			if(!(gdm_fic_ensbl[DAB_no].svc[j].svc_status & 0x2))
			{
				//====		label input	=============
				put_in_label(&gdm_fic_ensbl[DAB_no].svc[j].label[0]);
				gdm_fic_ensbl[DAB_no].svc[j].svc_status |= 0x2;	//
			}
			TmpInt0 = get_field(16);			//CharFlagField
			return;
		}
	}
	//==============================================
}

void fig1_4(G_Int32 DAB_no )
{
	G_Uint32	TmpInt0, scid;
	G_Uint8	TmpChar0, m,j, PDFlag;

	PDFlag = get_field(1);
	TmpChar0= get_field(3) & 0x7;
	if(TmpChar0)
		return;
	TmpInt0 = get_field(4);		//SCIdS
	if(PDFlag)
		scid = get_field(32);
	else
		scid = get_field(16);

	TmpInt0 = get_field(16);		//CharFlagField
	for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
	{
		for(m = 0; m < gdm_fic_ensbl[DAB_no].svc[j].num_of_svc_comp; m++)
		{
			if(gdm_fic_ensbl[DAB_no].svc[j].sc[m].scid == scid)
			{
				//====		label input	=============
				put_in_label(&gdm_fic_ensbl[DAB_no].svc[j].sc[m].label[0]);
				return;
			}
		}
	}
	//==============================================
}

void fig1_5(G_Int32 DAB_no )
{
	G_Uint32	TmpInt0;
	G_Uint8	j;
	TmpInt0= get_field(32);				// ID_Field, sid

	for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
	{
		if(gdm_fic_ensbl[DAB_no].svc[j].sid == TmpInt0)
		{
			if(!(gdm_fic_ensbl[DAB_no].svc[j].svc_status & 0x2))
			{
			//====		label input	=============
				put_in_label(&gdm_fic_ensbl[DAB_no].svc[j].label[0]);
				gdm_fic_ensbl[DAB_no].svc[j].svc_status |= 0x2;
			}
			TmpInt0 = get_field(16);		//CharFlagField
			return;
		}
	}
	//==============================================
}

void fig1_6(G_Int32 DAB_no )
{
	G_Uint32	sid;
	G_Uint8	TmpChar0, m,j, PDFlag, SCIdS;

	PDFlag = get_field(1);
	TmpChar0= get_field(3) & 0x7;
	if(TmpChar0)
		return;
	SCIdS = get_field(4);	//SCIdS
	if(PDFlag)
		sid = get_field(32);
	else
		sid = get_field(32);

	TmpChar0= get_field(2);	//Rfa
	TmpChar0= get_field(1);	//Rfu
	TmpChar0= get_field(5);	//XPadAppTy

	for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
	{
		if(gdm_fic_ensbl[DAB_no].svc[j].sid == sid)
		{
			for(m = 0; m < gdm_fic_ensbl[DAB_no].svc[j].num_of_svc_comp; m++)
			{
				if(gdm_fic_ensbl[DAB_no].svc[j].sc[m].scid == SCIdS)
				{
					gdm_fic_ensbl[DAB_no].svc[j].sc[m].xpad_type = TmpChar0;
					return;
				}
			}
		}
	}
	//==============================================
}

void fig5(G_Int32 DAB_no )
{		// FIDC
}	

void fig6(G_Int32 DAB_no )
{		//CA
	G_Uint32	sid;
	G_Uint8	TmpChar0, PDFlag,i, CASysId,cnt = 0, CAIntChar[24];
	//==============================================
	TmpChar0= get_field(1);	// Rfu	. all '0'
	TmpChar0= get_field(1);	// C/N
	TmpChar0= get_field(1);	// OE	
	PDFlag = get_field(1);		// PD
	TmpChar0= get_field(1);	// LEF
	if(!TmpChar0)
	{
		TmpChar0= get_field(3);	// Short CASysId
		if(!PDFlag)
			sid = get_field(16);	// sid
		else
			sid = get_field(32);	// sid
	}
	else
	{
		TmpChar0= get_field(3);	// Short CASysId
		cnt = 8;
		if(!PDFlag)
		{
			sid = get_field(16);	// sid
			cnt+= 16;
		}
		else
		{
			sid = get_field(32);	// sid
			cnt+= 32;
		}

		CASysId = get_field(16);	// 
		cnt += 16;	
		for(i = 0; i < fig_list.cFIGLength - cnt; i++)
			CAIntChar[i] = get_field(1);
	}
	//==============================================
}	

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  check_duplicate_svc



DESCRIPTION
           
           check if there is same service ID	and return the sequence of it	.

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] sid.
           [in] s_num.
           


RETURN VALUE
			Sqc of found service
			
-------------------------------------------------------------------------------------------- */

G_Uint8   check_duplicate_svc(G_Int32 DAB_no, G_Uint32 sid, G_Int32 num_of_svc){
    G_Uint8 i;

    for(i = 0; i < num_of_svc ; i++){
        if(gdm_fic_ensbl[DAB_no].svc[i].sid == sid)
            return i;
    }

    return 0xff;
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  check_duplicate_svc_comp



DESCRIPTION
           
           check if there is same service component ID and return the sequence of it.

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] Pointer of Service Structure.
           [in] tmid.
           [in] Uniqe_id :	
           	if tmid 0, ASCTy << 6 | sch_id
 				if tmid 1, DSCTy << 6 | sch_id
 				if tmid 2, DSCTy << 6 | FIDCId
 				if tmid 3, scid
           


RETURN VALUE
			Error
			
-------------------------------------------------------------------------------------------- */

G_Uint8   check_duplicate_svc_comp(gdm_fic_svc_s_type *svc_ptr, G_Uint8 tmid, G_Uint32 Uniqe_id){

    G_Uint8 i;
    G_Uint8   scty, sch_id;

    for(i = 0 ; i < svc_ptr->num_of_svc_comp; i++){
        if(tmid < 3){
            scty = Uniqe_id >> 6;
            sch_id = Uniqe_id & 0x3f;
            if((svc_ptr->sc[i].sch.sch_id == sch_id))
                return 0xff;
        }
        else{
            if(svc_ptr->sc[i].scid == Uniqe_id)
                return 0xff;
        }
    }
    return 0;
}
/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  check_duplicate_sch



DESCRIPTION
           
           check if there is same sub ch ID.

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.
           [in] sch_id.
           [in] sch_num - num of total SubCh.


RETURN VALUE
			Error
			
-------------------------------------------------------------------------------------------- */
G_Uint32   check_duplicate_sch(G_Int32 DAB_no,G_Uint8   sch_id, G_Int32 sch_num){
    G_Int32 i;

    for(i = 0; i < sch_num ; i++){
        if(gdm_fic_sch[DAB_no][i].sch_id == sch_id)
            return 0xff;
    }
    return 0;
}
/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  gdm_fic_init_db



DESCRIPTION
           
           Initialize all structures and parameters 
           related FIC Decoding such as Ensemble, 
           service, service component and sub channel etc.

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.


RETURN VALUE
			
-------------------------------------------------------------------------------------------- */

void    gdm_fic_init_db(G_Int32 DAB_no)
{
	G_Int32 i,j;
	gdm_fic_ensbl_s_type	*ensbl_ptr;
	gdm_fic_ts_sch_s_type		*TsSch;	
	gdm_fic_target_sch_id_s_type  *TargetSch;

  memset(&gdm_fic_buf[DAB_no], 0, sizeof(gdm_fic_buf_s_type));
	
	ensbl_ptr = &gdm_fic_ensbl[DAB_no];

    gdm_fic_cnt[DAB_no] = 0;

	gdm_fic_user_status[DAB_no] = 0;	
	memset(ensbl_ptr, 0, sizeof(gdm_fic_ensbl_s_type));
	ensbl_ptr->eid = 0xFFFF;
	memset(ensbl_ptr->ensemble_label, 0x20, sizeof(G_Uint8) * 16);
	for(i = 0 ; i < MAX_SERVICE_NUM; i++)
	{
		ensbl_ptr->svc[i].pd = 0xf;
		ensbl_ptr->svc[i].sid = 0xFFFFFFFF;
		memset(ensbl_ptr->svc[i].label, 0x20, sizeof(G_Uint8) * 16);
		for(j = 0 ; j <  MAX_SC_NUM; j++)
		{
			ensbl_ptr->svc[i].sc[j].tmid = 0xff;
			ensbl_ptr->svc[i].sc[j].scty_tmp = 0xff;
			ensbl_ptr->svc[i].sc[j].scid = 0xFFFF;
			ensbl_ptr->svc[i].sc[j].scty = 0xff;
			ensbl_ptr->svc[i].sc[j].ps = 0xff;
			ensbl_ptr->svc[i].sc[j].ca_flag = 0xff;
			ensbl_ptr->svc[i].sc[j].sch.sch_id = 0xff;
			ensbl_ptr->svc[i].sc[j].xpad_type = 0xff;
			memset(ensbl_ptr->svc[i].sc[j].label, 0x20, sizeof(G_Uint8) * 16);
	   }
	}

	TargetSch = &TDMB_target[DAB_no];
	memset(&TargetSch->sch_id[0], 0xff, sizeof(G_Uint8) * MAX_SUBCH_NUM);
	TargetSch->check_cnt = 0;

	memset(gdm_fic_sch[DAB_no], 0, sizeof(gdm_fic_sch_s_type) * MAX_SUBCH_NUM);
	for(j = 0; j < MAX_SUBCH_NUM; j++){
		gdm_fic_sch[DAB_no][j].sch_id = 0xff;
	}
	TsSch = &gdm_fic_ts_buf[DAB_no];
	TsSch->num_of_ts = 0;      // ts sub channel cnt.

	memset(&TsSch->sch_id[0], 0xff, sizeof(G_Uint8) * 5);
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  match_svc2sch



DESCRIPTION
           
           Right After gdm_fib_decode, find sub channel from gdm_fic_sch which is matched by service
           and copy it to the matched service structure.

PARAMETER

           [in] DAB_no - DAB 0 or DAB 1.


RETURN VALUE

-------------------------------------------------------------------------------------------- */

void    match_svc2sch(G_Int32 DAB_no)
{
  G_Int32 i, j;
  G_Uint8 num_of_svc_label_end = 0,num_of_svc_label_before = 0;

  gdm_fic_sc_s_type  *sc_ptr;

  //==============================================
  for(j = 0; j < gdm_fic_ensbl[DAB_no].num_of_svc ; j++)
  {
    if(gdm_fic_ensbl[DAB_no].svc[j].svc_status == 3)
    {
      num_of_svc_label_end ++;
      continue;
    }
    else if(gdm_fic_ensbl[DAB_no].svc[j].svc_status >=1)
    {
      num_of_svc_label_before ++;
    }

    sc_ptr = &gdm_fic_ensbl[DAB_no].svc[j].sc[0];

    if(sc_ptr->tmid == 0x02) // FIDC service
    {
      if((sc_ptr->sc_status & 0x03) == 0x01)
      {
        sc_ptr->sc_status |= 0x3;

        gdm_fic_ensbl[DAB_no].svc[j].svc_status |= 0x1;
        num_of_svc_label_before ++;

        if(gdm_fic_ensbl[DAB_no].svc[j].svc_status == 3)
        {
          num_of_svc_label_end ++;
        }
        continue;
      }
    }

    for(i = 0; i < gdm_fic_ensbl[DAB_no].num_of_sch ; i++)
    {
      if(sc_ptr->sch.sch_id ==  gdm_fic_sch[DAB_no][i].sch_id)
      {
        if((sc_ptr->sc_status & 0x03) == 0x01)
        {
          gdm_memcpy(&(sc_ptr->sch), &gdm_fic_sch[DAB_no][i], sizeof(gdm_fic_sch_s_type));
          sc_ptr->sc_status |= 0x3;

          gdm_fic_ensbl[DAB_no].svc[j].svc_status |= 0x1;
          num_of_svc_label_before ++;

          if(gdm_fic_ensbl[DAB_no].svc[j].svc_status == 3)
          {
            num_of_svc_label_end ++;
          }
          break;
        }
      }
    }
  }

  //===========================================
  gdm_fic_ensbl[DAB_no].num_of_svc_label_end = num_of_svc_label_end;
  gdm_fic_ensbl[DAB_no].num_of_svc_label_before= num_of_svc_label_before;
  //===========================================
}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  get_field



DESCRIPTION
           
           Selected bit[s] value return.

PARAMETER

           [in] sizebit - size of bit(s).


RETURN VALUE
			
-------------------------------------------------------------------------------------------- */

G_Uint32 get_field(G_Uint8 sizebit )
{
	G_Uint8 startbyte, sub_startbit, cReadTemp[4]={0,0,0,0}, sizebyte, sub_sizebit, NumTemp;
	G_Uint32 ReturnTemp;

	startbyte = bit_pos >>3;
	sub_startbit = bit_pos - (startbyte<<3);

	if(sizebit > 32)
	{
	}
	
	sizebyte = sizebit>>3;
	sub_sizebit = sizebit - (sizebyte<<3);

	NumTemp = 0;
	while(sizebyte != 0)
	{
		cReadTemp[NumTemp] = (*(fig_list.cFIGData+startbyte) << sub_startbit) | (*(fig_list.cFIGData+startbyte+1) >> (8-sub_startbit));
		startbyte++;
		sizebyte--;
		NumTemp++;
	}

	if(sub_sizebit != 0)
	{
		cReadTemp[NumTemp] = (*(fig_list.cFIGData+startbyte)<< sub_startbit) & (0xff<<(8-sub_sizebit));	
	}

	ReturnTemp = (((G_Uint32) cReadTemp[0]) << 24) |
					(((G_Uint32) cReadTemp[1]) << 16) |
					(((G_Uint32) cReadTemp[2]) << 8) |
					((G_Uint32) cReadTemp[3]);

	ReturnTemp = ReturnTemp >> (32-sizebit);
	bit_pos += sizebit;

	return ReturnTemp;
	

}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  get_bit_rate



DESCRIPTION
           
           Calculate Bit Rate and Return it.

PARAMETER

           [in] pointer of SubCh.


RETURN VALUE
			Bit rate
-------------------------------------------------------------------------------------------- */
G_Uint32 get_bit_rate(gdm_fic_sch_s_type * sch_ptr)
{
    if (sch_ptr->sl_fg == 0) 
    {
 		return bit_rate_value[sch_ptr->idx] << 2;
    }
    else
    {
	  if(sch_ptr->opt == 0)
		return (sch_ptr->sch_sz/sch_devide_value[(G_Int32)sch_ptr->prt_lvl]) <<3;
	  else
		return (sch_ptr->sch_sz/sch_devide_value[(G_Int32)sch_ptr->prt_lvl + 4])<< 5;
    }

}

/* --------------------------------------------------------------------------------------------

FUNCTION    
                                  crc16



DESCRIPTION
           
           CRC check.

PARAMETER

           [in] buf.


RETURN VALUE
			CRC value
-------------------------------------------------------------------------------------------- */

G_Uint16 crc16(G_Uint8 *buf)
{
  G_Uint32 b, len;
  G_Uint8 crcl,crcm;

  crcl = 0xff;
  crcm = 0xff;


  for(len=0;len<30;len++)
  {
    b = *(buf+len) ^ crcm;
    b = b ^ (b>>4);
    crcm = crcl ^ (b>>3) ^ (b<<4);
    crcl = b ^ (b<<5);
  }

  crcl = crcl ^ 0xff;
  crcm = crcm ^ 0xff;

  return ((G_Uint16)crcl | (G_Uint16)(crcm)<<8);
}


LOCAL void gdm_memcpy(void *dst, const void *src, G_Int32 size )
{
	while(size--) 
	{
		*(G_Uint8*)dst = *(G_Uint8*)src;
		(G_Uint8*)dst++;
		(G_Uint8*)src++;
	}
}
#endif /* (defined(FEATURE_NEXTV_DAL_API) && defined(FEATURE_NEXTV_DAL_TYPE_2)) */

