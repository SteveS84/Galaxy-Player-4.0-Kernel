/*
 * MC-1N2 VOIP controller 
 *
 *  this is add in file to controll voip in mc1n2 codec driver.
 *  this source add for simulating proximity senesor  
 *
 *  @author park dong yun  
 *  @history first creation 2010.1.3 
 */

#ifndef MC1N2_VOIP_H
#define MC1N2_VOIP_H


typedef enum{
    MC1N2_VOIP_MONITOR_STATUS_OFF=0,
    MC1N2_VOIP_MONITOR_STATUS_ON,
    MC1N2_VOIP_MONITOR_STATUS_CANCEL_BY_USER
}mc1n2_voip_monitor_status;

typedef enum{
    MC1N2_VOIP_STATUS_DISCONNECT=0,
    MC1N2_VOIP_STATUS_CONNECT,
}mc1n2_voip_connet_status;    

typedef enum{
    MC1N2_VOIP_DEVICE_NOTHING=0,
    MC1N2_VOIP_DEVICE_RECEIVER=1,
    MC1N2_VOIP_DEVICE_SPEAKER=2,
    MC1N2_VOIP_DEVICE_HP=4,
}mc1n2_voip_device;    

typedef enum{
    MC1N2_VOIP_TOUCH_BYPASS=0,
    MC1N2_VOIP_TOUCH_PREVENT=1,
}mc1n2_voip_touch_status;    

typedef enum{
    MC1N2_VOIP_CAMERA_OFF=0,
    MC1N2_VOIP_CAMERA_ON=1
}mc1n2_voip_camera_status;    

typedef enum{
   MC1N2_VOIP_MICGAIN_NOTHING=0,
   MC1N2_VOIP_MICGAIN_SPEAKER_SKYPE=1,
   MC1N2_VOIP_MICGAIN_SPEAKER_QIK=2,
   MC1N2_VOIP_MICGAIN_RCV=3,
   MC1N2_VOIP_MICGAIN_HP=4,
}mc1n2_voip_mic_gain;

typedef enum{
    MC1N2_VOIP_OUTPUTGAIN_NOTHING=0,
    MC1N2_VOIP_OUTPUTGAIN_SPK=1,
    MC1N2_VOIP_OUTPUTGAIN_RCV=2,
    MC1N2_VOIP_OUTPUTGAIN_HP=3
}mc1n2_voip_output_gain;


typedef enum{
   MC1N2_VOIP_TYPE_NONE=0,
   MC1N2_VOIP_TYPE_QIK=1,
   MC1N2_VOIP_TYPE_SKYPE=2   
}mc1n2_voip_type;

typedef enum{
    MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_OFF=0,
    MC1N2_VOIP_SET_FAKE_PROXIMITY_SENSOR_ON=1
}mc1n2_voip_fake_proximity_sensor_flag;

   

typedef struct mc1n2_voip_st {
    mc1n2_voip_connet_status  status;
    mc1n2_voip_device device;
    mc1n2_voip_touch_status touch_status;
    mc1n2_voip_monitor_status monitor_status;
    mc1n2_voip_camera_status camera_status;
    mc1n2_voip_mic_gain mic_gain;
    mc1n2_voip_output_gain output_gain;
    mc1n2_voip_type voip_type;
    mc1n2_voip_fake_proximity_sensor_flag sensor_flag;
}mc1n2_voip_info;

void mc1n2_voip_init(void);
void mc1n2_start_voip(void);
void mc1n2_end_voip(void);

mc1n2_voip_connet_status mc1n2_voip_get_status(void);
void mc1n2_voip_set_playback_parameters(int micpath );
void mc1n2_voip_set_mic_parameters(int path);
int mc1n2_set_voip_fake_proximity_sensor(bool onOff);
int  mc1n2_wakeup_check_for_proximity_monitor(void);
mc1n2_voip_camera_status mcn1n2_voip_get_camera_status();
mc1n2_voip_type mc1n2_voip_get_voip_type();


#endif 


