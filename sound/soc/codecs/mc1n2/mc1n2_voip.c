/*
 * MC-1N2 VOIP controller 
 *
 *  this is add in file to controll voip in mc1n2 codec driver.
 *  this source add for simulating proximity senesor  
 *
 *  @author park dong yun  
 *  @history first creation 2010.1.3 
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <sound/hwdep.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/file.h>
#include <asm/io.h>
#include <asm/gpio.h> 
#include <plat/gpio-cfg.h> 
#include <plat/map-base.h>
#include <mach/regs-clock.h> 
#include <mach/gpio.h> 
#include <linux/slab.h>
#include "mc1n2.h"
#include "mc1n2_voip.h"
#include "mc1n2_priv.h"
#include "mc1n2_cfg.h"
#include "mcresctrl.h"  // yosef added for test Mic2


    
extern int lastUpdate_by_set_path_mic;
extern int lastUpdate_by_set_path_playback;

// #define ENABLE_CALL_DROP 
//#define LOAD_VOIP_CONFIG 

/* monitor delay 1 sec */
#define VOID_MONITOR_DELAY 400

static void mc1n2_proximity_monitor_init(void);
static void mc1n2_proximity_monitor_pause(void);
static void mc1n2_proximity_monitor_resume(void);
static void mc1n2_proximity_monitor_remove(void);

//#define MC1N2_VOIP_SPEAKER_GAIN_QIK 55
//#define MC1N2_VOIP_SPEAKER_GAIN_SKYPE 63
#define MC1N2_VOIP_SPEAKER_GAIN_SKYPE 55

#define MC1N2_VOIP_RECEIVER_GAIN 63
#define MC1N2_VOIP_HEADPHONE_GAIN 44

// #define MC1N2_VOIP_SPEAKER_MIC_GAIN_QIK 2
//#define MC1N2_VOIP_SPEAKER_MIC_ADC_QIK 177
// #define MC1N2_VOIP_SPEAKER_MIC_GAIN_SKYPE 2
// #define MC1N2_VOIP_SPEAKER_MIC_ADC_SKYPE 203
#define MC1N2_VOIP_SPEAKER_MIC_GAIN_SKYPE 3
#define MC1N2_VOIP_SPEAKER_MIC_ADC_SKYPE 177


#define MC1N2_VOIP_HEADSET_MIC_GAIN 3
#define MC1N2_VOIP_HEADSET_ADC_GAIN 209
#define MC1N2_VOIP_HEADSET_DIT_QIK 192

//#define MC1N2_VOIP_SPEAKER_DAC_QIK 192
#define MC1N2_VOIP_SPEAKER_DAC_SKYPE 192
//#define MC1N2_VOIP_SPEAKER_DAC_MASTER_QIK 192
//#define MC1N2_VOIP_SPEAKER_DAC_MASTER_SKYPE 200
#define MC1N2_VOIP_SPEAKER_DAC_MASTER_SKYPE 192

//#define MC1N2_VOIP_SPEAKER_DIT_QIK 192

//int speakerloopcount=0;
//int speakerfadetab[6]={20,30,40,50,MC1N2_VOIP_SPEAKER_GAIN_SKYPE,MC1N2_VOIP_SPEAKER_GAIN_SKYPE};

#ifdef LOAD_VOIP_CONFIG 
#include <linux/string.h>
#include <linux/kernel.h>
int isLoadVoipConfig=1;
char *token;
char *last;

// playback 
// int gSpeakerGainQik=0;
int gSpeakerGainSkype=0;
int gReceiverGain=0;
int gHeadPhoneGain=0;

// mic 
// int gSpeaker_micgain_qik=0;
// int gSpeaker_micadc_qik=0;
int gSpeaker_micgain_skype=0;
int gSpeaker_micadc_skype=0;
// int gReceiver_micgain=0;
// int gReceiver_micadc=0;
int gHeadPhone_micgain=0;
int gHeadPhone_micadc=0;
int gHeadPhone_micdit_qik=0;

// int gSpeakerGainDacQik=0;
int gSpeakerGainDacSkype=0;
// int gSpeakerDacMasterQik=0;
int gSpeakerDacMasterSkype=0;
int gSpeakerDitVol=0;

int  ReadVoipConfigFile(char *Filename, int StartPos)
{
	struct file 	*filp;  
      char * Buffer;
	mm_segment_t	oldfs;
	int		BytesRead;

	Buffer = kmalloc(256,GFP_KERNEL);
	if (Buffer==NULL) 
		return -1;
	
	filp = filp_open(Filename,00,O_RDONLY);
	if (IS_ERR(filp)||(filp==NULL))
	{
           CODECDBG("ReadSoundconfig open errrrrrrr");
		return -1;  /* Or do something else */
      }
	if (filp->f_op->read==NULL)
	{
           CODECDBG("ReadSoundconfig open read error");
		return-1;  /* File(system) doesn't allow reads */
      }
	/* Now read 4096 bytes from postion "StartPos" */
	filp->f_pos = StartPos;
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	BytesRead = filp->f_op->read(filp,Buffer,256,&filp->f_pos);
	set_fs(oldfs);

      // playback
      last=Buffer;
//      token = strsep(&last, ",");
//      gSpeakerGainQik = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeakerGainSkype = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gReceiverGain = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gHeadPhoneGain = simple_strtoul(token,NULL,10);

       // mic 
//      token = strsep(&last, ",");
//      gSpeaker_micgain_qik = simple_strtoul(token,NULL,10);
//      token = strsep(&last, ",");
//      gSpeaker_micadc_qik = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeaker_micgain_skype = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeaker_micadc_skype = simple_strtoul(token,NULL,10);
/*
      token = strsep(&last, ",");
      gReceiver_micgain = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gReceiver_micadc = simple_strtoul(token,NULL,10);
 */
      token = strsep(&last, ",");
      gHeadPhone_micgain = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gHeadPhone_micadc = simple_strtoul(token,NULL,10);
//       token = strsep(&last, ",");
//      gHeadPhone_micdit_qik = simple_strtoul(token,NULL,10);
      
//      token = strsep(&last, ",");
//      gSpeakerGainDacQik = simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeakerGainDacSkype= simple_strtoul(token,NULL,10);
//      token = strsep(&last, ",");
//      gSpeakerDacMasterQik= simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeakerDacMasterSkype= simple_strtoul(token,NULL,10);
      token = strsep(&last, ",");
      gSpeakerDitVol= simple_strtoul(token,NULL,10);

#if 1
     	 CODECDBG("--output gain ---------- \n gSpeakerGainSkype=%d\n gReceiverGain=%d\n gHeadPhoneGain=%d\n -------------output gain --------------------- ",
	                                                               gSpeakerGainSkype,           gReceiverGain,          gHeadPhoneGain);
       CODECDBG("--mic gain speaker ------ \n gSpeaker_micgain_skype=%d\n gSpeaker_micadc_skype=%d\n --mic gain speaker ------- ",
                                                                  gSpeaker_micgain_skype,          gSpeaker_micadc_skype);
//       CODECDBG("--mic gain rcv and hp----- \n gReceiver_micgain=%d\n gReceiver_micadc=%d\n gHeadPhone_micgain=%d\n gHeadPhone_micadc=%d\n --mic gain rcv and hp----- ",
   //                                                        ver_micadc,          gHeadPhone_micgain,          gHeadPhone_micadc);

       CODECDBG("--digital gain ----- \n  gSpeakerGainDacSkype=%d\n gSpeakerDacMaster=%d\n gSpeakerDitVol=%d\n --mic gain rcv and hp----- ",
                                                         gSpeakerGainDacSkype,          gSpeakerDacMasterSkype  ,    gSpeakerDitVol);
#endif 

//     speakerfadetab[4]=gSpeakerGainSkype;
//        speakerfadetab[5]=gSpeakerGainSkype;
                                                             
        /* Close the file */
        fput(filp);
        /* release allocate memeory */
        kfree(Buffer);      
        return 0;
        
}
#endif 

/* monitor for proximity sensor */
static void mc1n2_proximity_monitor_body(struct work_struct* p_work);
DECLARE_DELAYED_WORK(mc1n2_proximity_monitor_wq, mc1n2_proximity_monitor_body);

// for blocking lcd 
typedef enum {
	BACKLIGHT_LEVEL_OFF		= 0,
	BACKLIGHT_LEVEL_DIMMING	= 1,
	BACKLIGHT_LEVEL_NORMAL	= 6
} backlight_level_t;

/* back light off function define fron LCD driver */
extern void backlight_onoff(backlight_level_t f_onNoff );

/* global information */
mc1n2_voip_info g_voip_info={0};


void mc1n2_voip_init()
{
    g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;
    g_voip_info.device=MC1N2_VOIP_DEVICE_NOTHING;
    g_voip_info.status=MC1N2_VOIP_STATUS_DISCONNECT;
    g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_OFF;
    g_voip_info.camera_status=MC1N2_VOIP_CAMERA_OFF;
    g_voip_info.mic_gain=MC1N2_VOIP_MICGAIN_NOTHING;
    g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_NOTHING;
    g_voip_info.voip_type=MC1N2_VOIP_TYPE_SKYPE;
    g_voip_info.sensor_flag=MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_OFF;

}

void mc1n2_start_voip(void)
{   
    if (MC1N2_VOIP_STATUS_CONNECT!=g_voip_info.status)
    {
        mc1n2_proximity_monitor_init();          
        g_voip_info.status=MC1N2_VOIP_STATUS_CONNECT;
     
        CODECDBG("current voip touch =%d, device=%d, status=%d, monitor=%d, camera=%d",
          g_voip_info.touch_status, 
          g_voip_info.device, 
          g_voip_info.status,
          g_voip_info.monitor_status,
          g_voip_info.camera_status );  
      
#ifdef LOAD_VOIP_CONFIG  // ROSSI PROJECT  UPDATE BY LEE SEUNG WOOK FOR SOUND TUNNING, 2010.08.23
	  if(0== ReadVoipConfigFile("/sdcard/soundcfg/voip.txt",0))
        {
            isLoadVoipConfig=0;
        }
#endif
//        speakerloopcount=0;
     }    
}
void mc1n2_end_voip(void)
{
    struct snd_soc_codec *codec=mc1n2_get_codec_data();
    MCDRV_PATH_INFO path;
    memset(&path, 0, sizeof(path));
    mc1n2_proximity_monitor_remove();
     
    CODECDBG("current voip touch =%d, device=%d, status=%d, monitor=%d, camera=%d",
        g_voip_info.touch_status, 
        g_voip_info.device, 
        g_voip_info.status,
        g_voip_info.monitor_status,
        g_voip_info.camera_status );  

#ifdef ENABLE_CALL_DROP
 //   if( MC1N2_VOIP_TOUCH_PREVENT==g_voip_info.touch_status)
//    {
        CODECDBG("release blocking touch input");            
        backlight_onoff(1);
        g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;
//  }
#endif 

    mc1n2_write_reg(codec,MC1N2_DAC_VOL_L,192);
    mc1n2_write_reg(codec,MC1N2_DAC_MASTER,192); 
    mc1n2_write_reg(codec,MC1N2_DAC_DAT_VAL,192); 
     CODECDBG("resotre eq filter");
    _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Rcv, 0x1FF); // voip volup recovery.

//    path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
//    path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON | MCDRV_SRC5_DAC_R_ON;

       /* turn off mic path */
       path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
       path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
       path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC3_OFF;  // y
       mc1n2_set_path(codec, &path);

     /* remove whole value for proximity monitor */
    g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;
    g_voip_info.device=MC1N2_VOIP_DEVICE_NOTHING;
    g_voip_info.status=MC1N2_VOIP_STATUS_DISCONNECT;
    g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_OFF;
    g_voip_info.camera_status=MC1N2_VOIP_CAMERA_OFF;
    g_voip_info.mic_gain=MC1N2_VOIP_MICGAIN_NOTHING;
    g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_NOTHING;
    g_voip_info.voip_type=MC1N2_VOIP_TYPE_SKYPE;
    return;       
}

mc1n2_voip_connet_status mc1n2_voip_get_status()
{
    return g_voip_info.status;
}

mc1n2_voip_type mc1n2_voip_get_voip_type()
{
    return g_voip_info.voip_type;
}

void mc1n2_voip_set_playback_parameters(int path)
{
    struct snd_soc_codec *codec=NULL; 
    if(lastUpdate_by_set_path_playback==0) return;

    /* check codec status for duplication setting */
    codec=mc1n2_get_codec_data();

     /*
     if (g_voip_info.voip_type==MC1N2_VOIP_TYPE_QIK)
     {
         CODECDBG("apply qik eq filter\n");
   	    _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_qik, 0x1FF); // qik filter apply	
     }

     else 
     {
         CODECDBG("apply skype eq filter\n");
        _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_skype, 0x1FF); // skype  filter apply
     }
     */
    
    if (SPK==path)
    {

#if 0  
        MCDRV_PATH_INFO path;
        memset(&path, 0, sizeof(path));        
      // to turn off SPEAKER R channel.
        path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
        path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON | MCDRV_SRC5_DAC_R_OFF;
         mc1n2_set_path(codec, &path);
        CODECDBG( "turn off  right speaker in voip\n");
#else 
        mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R,0);        
#endif 
            
#if 0
        // DISABLE GAIN SEPERATION IN VOIP update by park dong yun 
         if (g_voip_info.voip_type==MC1N2_VOIP_TYPE_QIK)
        {
#ifdef LOAD_VOIP_CONFIG
            mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, gSpeakerGainQik);
            mc1n2_write_reg(codec,MC1N2_DAC_VOL_L,gSpeakerGainDacQik);
            mc1n2_write_reg(codec,MC1N2_DAC_MASTER,gSpeakerDacMasterQik);
            CODECDBG("MC1N2_SPEAKER_VOL_L=%d",MC1N2_VOIP_SPEAKER_GAIN_QIK);                
#else 
            mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, MC1N2_VOIP_SPEAKER_GAIN_QIK);
            mc1n2_write_reg(codec,MC1N2_DAC_VOL_L,MC1N2_VOIP_SPEAKER_DAC_QIK);
            mc1n2_write_reg(codec,MC1N2_DAC_MASTER,MC1N2_VOIP_SPEAKER_DAC_MASTER_QIK); 
#endif 
            g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_SPK;         
            CODECDBG("MC1N2_SPEAKER_VOL_L=%d",MC1N2_VOIP_SPEAKER_GAIN_QIK);                
        }else 
        {
#endif 

#ifdef LOAD_VOIP_CONFIG                         
            mc1n2_write_reg(codec,MC1N2_SPEAKER_VOL_L, gSpeakerGainSkype);
            mc1n2_write_reg(codec,MC1N2_DAC_VOL_L,gSpeakerGainDacSkype);
            mc1n2_write_reg(codec,MC1N2_DAC_MASTER,gSpeakerDacMasterSkype);
            CODECDBG("MC1N2_SPEAKER_VOL_L=%d", gSpeakerGainSkype);                
#else 
            mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, MC1N2_VOIP_SPEAKER_GAIN_SKYPE);
            mc1n2_write_reg(codec,MC1N2_DAC_VOL_L,MC1N2_VOIP_SPEAKER_DAC_SKYPE);
            mc1n2_write_reg(codec,MC1N2_DAC_MASTER,MC1N2_VOIP_SPEAKER_DAC_MASTER_SKYPE); 
            CODECDBG("MC1N2_SPEAKER_VOL_L=%d",MC1N2_VOIP_SPEAKER_GAIN_SKYPE);                
#endif 
            g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_SPK;    
 //       }
 

    }else if(RCV==path)
    {
  //      if (g_voip_info.voip_type!=MC1N2_VOIP_TYPE_QIK)
//        {

 #ifdef LOAD_VOIP_CONFIG 
        mc1n2_write_reg(codec, MC1N2_RCV_VOL_L, gReceiverGain);
        mc1n2_write_reg(codec, MC1N2_RCV_VOL_R, gReceiverGain);
#else 
        mc1n2_write_reg(codec, MC1N2_RCV_VOL_L, MC1N2_VOIP_RECEIVER_GAIN);
        mc1n2_write_reg(codec, MC1N2_RCV_VOL_R, MC1N2_VOIP_RECEIVER_GAIN);
#endif         
        g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_RCV;      

  //       }
         lastUpdate_by_set_path_mic=1;
        mc1n2_voip_set_mic_parameters(MIC_MAIN);
        CODECDBG("MC1N2_RCV_VOL_L=%d",MC1N2_VOIP_RECEIVER_GAIN);
    }else if (HP==path)
    {
#ifdef LOAD_VOIP_CONFIG 
        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gHeadPhoneGain);
        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gHeadPhoneGain);
#else 
        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, MC1N2_VOIP_HEADPHONE_GAIN);
        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, MC1N2_VOIP_HEADPHONE_GAIN);
#endif         
        g_voip_info.output_gain=MC1N2_VOIP_OUTPUTGAIN_HP;
         CODECDBG("MC1N2_HEADPHONE_VOL_L=%d",MC1N2_VOIP_HEADPHONE_GAIN);
    }        
    
    if( lastUpdate_by_set_path_playback==1) lastUpdate_by_set_path_playback=0;
        
}


void mc1n2_voip_set_mic_parameters(int micpath )
{
   struct snd_soc_codec *codec = NULL;
   	MCDRV_PATH_INFO path;
    if(lastUpdate_by_set_path_mic==0) return;
    if(lastUpdate_by_set_path_mic==1) lastUpdate_by_set_path_mic=0;  

    /* check codec status for duplication setting */    
    codec=mc1n2_get_codec_data();
    memset(&path, 0, sizeof(path));
    if (MIC_MAIN==micpath)
    {

#if 0        
            if (g_voip_info.voip_type==MC1N2_VOIP_TYPE_QIK)
            {
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, gSpeaker_micgain_qik);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gSpeaker_micadc_qik);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gSpeaker_micadc_qik);   
                mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, gSpeakerDitVol);   
#else 
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, MC1N2_VOIP_SPEAKER_MIC_GAIN_QIK);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_VOIP_SPEAKER_MIC_ADC_QIK);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_VOIP_SPEAKER_MIC_ADC_QIK);	      
                mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, MC1N2_VOIP_SPEAKER_DIT_QIK);	      
#endif             
                g_voip_info.mic_gain=MC1N2_VOIP_MICGAIN_SPEAKER_QIK;
                CODECDBG("set mic gain for qik speaker =%d,%d",MC1N2_VOIP_SPEAKER_MIC_GAIN_QIK,MC1N2_VOIP_SPEAKER_MIC_ADC_QIK);       
            }else 
            {
#endif 

#ifdef LOAD_VOIP_CONFIG            
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, gSpeaker_micgain_skype);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gSpeaker_micadc_skype);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gSpeaker_micadc_skype);    
#else 
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, MC1N2_VOIP_SPEAKER_MIC_GAIN_SKYPE);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_VOIP_SPEAKER_MIC_ADC_SKYPE);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_VOIP_SPEAKER_MIC_ADC_SKYPE);                
#endif 
                g_voip_info.mic_gain=MC1N2_VOIP_MICGAIN_SPEAKER_SKYPE;
                CODECDBG("set mic gain for skype speaker=%d,%d",MC1N2_VOIP_SPEAKER_MIC_GAIN_SKYPE,MC1N2_VOIP_SPEAKER_MIC_ADC_SKYPE);       
//            }        

            msleep (50);
            path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON; 
            path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF; 
            path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
            path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
            path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC3_OFF;  // yosef added test for mic 
		 CODECDBG( "set voip main mic path");   //yosef add 
	
    }else if (MIC_SUB==micpath)
    {   
#ifdef LOAD_VOIP_CONFIG
            mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, gHeadPhone_micgain);
            mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gHeadPhone_micadc); 
            mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gHeadPhone_micadc); 
            mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gHeadPhone_micdit_qik); 
#else 
            mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, MC1N2_VOIP_HEADSET_MIC_GAIN);
            mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_VOIP_HEADSET_ADC_GAIN); 
            mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_VOIP_HEADSET_ADC_GAIN);        
            mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, MC1N2_VOIP_HEADSET_DIT_QIK);	      
#endif  

        msleep(50);
              
        path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON; 
        path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF; 
        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF; 
        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF; 
        path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON;  // yosef added test for mic 
        CODECDBG("set sub mic path for voip");   //yosef add 

        g_voip_info.mic_gain=MC1N2_VOIP_MICGAIN_HP;
        CODECDBG("set mic gain HEADPHONE =%d,%d",MC1N2_VOIP_HEADSET_MIC_GAIN,MC1N2_VOIP_HEADSET_ADC_GAIN);
    }        

    	mc1n2_set_path(codec, &path);
  
        
}


void mc1n2_voip_set_camera_status(int status)
{
#if 0
    // DISABLE CAMERA CHECK LOGIC 
    CODECDBG("camera status =%d connection_status =%d",status, g_voip_info.status);

    /* when user turn on the camear change skepaker gain for qik */
    if (g_voip_info.camera_status==0&&status==1)
    {
        g_voip_info.voip_type=MC1N2_VOIP_TYPE_QIK;
        CODECDBG("change camear status filter=%d",status);       
        lastUpdate_by_set_path_playback=1;
        lastUpdate_by_set_path_mic=1;
    }

    if (g_voip_info.status==MC1N2_VOIP_STATUS_DISCONNECT) 
    {
        struct snd_soc_codec *codec=mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;
        
        if(status==1)
        {
            CODECDBG( "set camera volume max");            
            mc1n2_set_analog_volume_spk(30);         
        }else 
        {
            CODECDBG( "restore camera volume");            
            mc1n2_set_analog_volume_spk(mc1n2->analog_vol);

        }
    }
    g_voip_info.camera_status=status;  
#endif     
}
mc1n2_voip_camera_status mcn1n2_voip_get_camera_status()
{
    return g_voip_info.camera_status;       
}
EXPORT_SYMBOL(mc1n2_voip_set_camera_status);


static void mc1n2_proximity_monitor_body(struct work_struct* p_work)
{
     struct snd_soc_codec *codec=mc1n2_get_codec_data();
     struct mc1n2_data *mc1n2 = codec->drvdata;    
 //  CODECDBG("monitor call path= %d, playback path =%d \n",mc1n2->call_path,mc1n2->playback_path);             

     if(g_voip_info.monitor_status ==MC1N2_VOIP_MONITOR_STATUS_CANCEL_BY_USER)
     {
         backlight_onoff(1);
        g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;        
        g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_OFF;
        CODECDBG("monitor stop by user");       
        return;
     }    

     /* check current audio path */
     /* if call path is PLAYBACK_OFF see playback_path */
        if (PLAYBACK_OFF==mc1n2->call_path)
        {
            if(RCV== mc1n2->playback_path)
            {
                g_voip_info.device=MC1N2_VOIP_DEVICE_RECEIVER;    
            }else if (SPK==mc1n2->playback_path)        
            {
                g_voip_info.device=MC1N2_VOIP_DEVICE_SPEAKER;
            }else if (HP==mc1n2->playback_path)
            {
                g_voip_info.device=MC1N2_VOIP_DEVICE_HP;                
            }
        }else if (RCV==mc1n2->call_path) 
        {
            g_voip_info.device=MC1N2_VOIP_DEVICE_RECEIVER; 
        }
        else if (SPK==mc1n2->call_path)
        {
            g_voip_info.device=MC1N2_VOIP_DEVICE_SPEAKER;            
        }else if (HP==mc1n2->call_path)
        {
            g_voip_info.device=MC1N2_VOIP_DEVICE_HP;            
        }

        /* prevent input touch for call drop */
        if(MC1N2_VOIP_DEVICE_RECEIVER==g_voip_info.device)
        {
            if(MC1N2_VOIP_CAMERA_OFF==g_voip_info.camera_status)
            {
#ifdef ENABLE_CALL_DROP
                if(MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_ON==g_voip_info.sensor_flag)
                {
                    if(g_voip_info.touch_status==MC1N2_VOIP_TOUCH_BYPASS)
                    {
                        /* prevent touch input and turn off LCD*/    
                        g_voip_info.touch_status=MC1N2_VOIP_TOUCH_PREVENT;
                        /* set mic gain for receiver */           
                        backlight_onoff(0);         
                    }
                }
#endif 
            }
#if 0            
            if (g_voip_info.voip_type==MC1N2_VOIP_TYPE_QIK)
            {
                 mc1n2_voip_set_playback_parameters(HP);           
            }else 
            {
#endif                 
                mc1n2_voip_set_playback_parameters(RCV);        
//           }

        // case for speaker         
        }else if (MC1N2_VOIP_DEVICE_SPEAKER==g_voip_info.device)
        {
            if(g_voip_info.touch_status==MC1N2_VOIP_TOUCH_PREVENT) backlight_onoff(1);

            /* set mic gain for speaker  */       
            mc1n2_voip_set_playback_parameters(SPK);
            g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;        
            
        // case for headphone       
         }else if (MC1N2_VOIP_DEVICE_HP==g_voip_info.device)
         {
             mc1n2_voip_set_playback_parameters(HP);                     
         }

        /* SET MIC PATH */
       	switch(mc1n2->mic_path) {
		case MIC_MAIN:
                 mc1n2_voip_set_mic_parameters(MIC_MAIN);        
			break;
		case MIC_SUB:
                 mc1n2_voip_set_mic_parameters(MIC_SUB);        
			break;
		default:
			break;
	    }        
    
 /*    
    CODECDBG("current voip touch =%d, device=%d, status=%d, monitor=%d, camera=%d \n",
        __func__,
        g_voip_info.touch_status, 
        g_voip_info.device, 
        g_voip_info.status,
        g_voip_info.monitor_status,
        g_voip_info.camera_status );  
*/

    /* scheduled next delayed work */
   if ( MC1N2_VOIP_MONITOR_STATUS_ON==g_voip_info.monitor_status)
       schedule_delayed_work(&mc1n2_proximity_monitor_wq, msecs_to_jiffies(VOID_MONITOR_DELAY));
   
}


static void mc1n2_proximity_monitor_init(void)
{
    CODECDBG("init");  
    schedule_delayed_work(&mc1n2_proximity_monitor_wq, msecs_to_jiffies(VOID_MONITOR_DELAY));
    g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_ON;
}

static void mc1n2_proximity_monitor_pause(void)
{
    if(MC1N2_VOIP_MONITOR_STATUS_ON==g_voip_info.monitor_status)
    {
        CODECDBG("paused!");
        cancel_delayed_work_sync(&mc1n2_proximity_monitor_wq);
        g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_OFF;
    }
}

static void mc1n2_proximity_monitor_resume(void)
{
    if(MC1N2_VOIP_MONITOR_STATUS_OFF==g_voip_info.monitor_status)
    {
        CODECDBG("resume!");
        schedule_delayed_work(&mc1n2_proximity_monitor_wq, msecs_to_jiffies(VOID_MONITOR_DELAY));
        g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_ON;
    }
}

static void mc1n2_proximity_monitor_remove()
{
 //   if(MC1N2_VOIP_MONITOR_STATUS_ON==g_voip_info.monitor_status)
  //  {
        CODECDBG("remove!");
        cancel_delayed_work_sync(&mc1n2_proximity_monitor_wq);             
        g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_OFF;
  //  }
}
int  mc1n2_wakeup_check_for_proximity_monitor(void)
{
    if (MC1N2_VOIP_TOUCH_BYPASS== g_voip_info.touch_status) return 0;

//  if(g_voip_info.touch_status==MC1N2_VOIP_TOUCH_PREVENT)
//  {
//     g_voip_info.touch_status=MC1N2_VOIP_TOUCH_BYPASS;              
        g_voip_info.monitor_status=MC1N2_VOIP_MONITOR_STATUS_CANCEL_BY_USER;
//  }
    return 1;
}
EXPORT_SYMBOL(mc1n2_wakeup_check_for_proximity_monitor);

int mc1n2_get_voip_touch_status()
{
//    CODECDBG("touch status %d\n",g_voip_info.touch_status);
    return g_voip_info.touch_status;
}
EXPORT_SYMBOL(mc1n2_get_voip_touch_status);

int mc1n2_set_voip_fake_proximity_sensor(bool onOff)
{
    CODECDBG("onOff=%d",onOff);
    if(onOff)
    {
        g_voip_info.sensor_flag=MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_ON;
    }else
    {
        g_voip_info.sensor_flag=MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_OFF;
    }    
}

