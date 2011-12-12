/*
 * linux/drivers/power/adc_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME "sec-battery"

#define BATT_CAL				2022	/* 3.60V */

#define BATT_MAXIMUM			780		/* 4.176V */
#define BATT_FULL				682		/* 4.10V  */
#define BATT_SAFE_RECHARGE		682		/* 4.10V */
#define BATT_ALMOST_FULL		354		/* 3.8641V */   //322    [691]  /* 4.066V */
#define BATT_HIGH				209		/* 3.7554V */       //221  483 /* 3.919V */
#define BATT_MED				117		/* 3.6907V */        //146 330  /* 3.811V */
#define BATT_LOW				55		/* 3.6566V */        //112  268 /* 3.763V */
#define BATT_CRITICAL			(1)		/* 3.6037V */     //(74) 137 /* 3.707V */
#define BATT_MINIMUM			(-85)	/* 3.554V */  //(38) 86 /* 3.655V */
#define BATT_OFF				(-293)	/* 3.4029V */    //(-103)    -231/* 3.45V  */

#if 0
const int battery_table[][3] = 
{
	/* %,    V,  adc*/
	{100, 4130, 2710},
	{ 95, 4061, 2618},
	{ 90, 4010, 2548},
	{ 85, 3963, 2483},
	{ 80, 3918, 2430},
	{ 75, 3879, 2375},
	{ 70, 3841, 2323},
	{ 65, 3806, 2278},
	{ 60, 3770, 2219},
	{ 55, 3738, 2178},
	{ 50, 3717, 2151},
	{ 45, 3704, 2125},
	{ 40, 3697, 2119},
	{ 35, 3690, 2112},
	{ 30, 3679, 2099},
	{ 25, 3657, 2071},
	{ 20, 3616, 2019},
	{ 15, 3557, 1940},
	{ 10, 3502, 1855},
	{  5, 3450, 1782},
	{  3, 3435, 1750},
	{  1, 3410, 1720},
	{  0, 3400, 1718},
};
#endif

/* Venturi new table. 4.13, 3.98, 3.82, 3.78, 3.74, 3.68, 3.50, 3.40 */
#if 0
const int battery_table[][3] = 
{
	/* %,    V,  adc*/
	{100, 4130, 2796},
	{ 95, 4100, 2741},
	{ 90, 4060, 2686},
	{ 85, 4020, 2631},
	{ 80, 3980, 2575},
	{ 75, 3930, 2505},
	{ 70, 3870, 2435},
	{ 65, 3820, 2364},
	{ 60, 3800, 2337},
	{ 55, 3790, 2322},
	{ 50, 3780, 2305},
	{ 45, 3760, 2282},
	{ 40, 3750, 2262},
	{ 35, 3740, 2252},
	{ 30, 3720, 2222},
	{ 25, 3700, 2197},
	{ 20, 3680, 2168},
	{ 15, 3610, 2084},
	{ 10, 3550, 2000},
	{  5, 3500, 1914},
	{  3, 3440, 1847},
	{  1, 3410, 1782},
	{  0, 3400, 1779},
};
#endif
const int battery_table[][3] = 
{
	/* %,    V,  adc*/
	{100, 4130, 2779},
	{ 95, 4100, 2724},
	{ 90, 4060, 2669},
	{ 85, 4020, 2614},
	{ 80, 3980, 2558},
	{ 75, 3930, 2488},
	{ 70, 3870, 2418},
	{ 65, 3820, 2347},
	{ 60, 3800, 2320},
	{ 55, 3790, 2305},
	{ 50, 3780, 2288},
	{ 45, 3760, 2265},
	{ 40, 3750, 2245},
	{ 35, 3740, 2235},
	{ 30, 3720, 2205},
	{ 25, 3700, 2180},
	{ 20, 3680, 2151},
	{ 15, 3610, 2067},
	{ 10, 3550, 1983},
	{  5, 3500, 1897},
	{  3, 3440, 1830},
	{  1, 3410, 1765},
	{  0, 3400, 1762},
};

const int NTH5G10P33B103E08TH_resistance_table[] = 
{
	/* -15C ~ 85C */

	/*0[-15] ~  14[-1]*/
	58037,55174,52473,49924,47516,
	45241,43092,41059,39137,37318,
	35596,33966,32421,30958,29570,
	/*15[0] ~ 34[19]*/
	28255,27006,25822,24697,23629,
	22614,21650,20733,19861,19032,
	18243,17491,16776,16095,15445,
	14826,14236,13674,13137,12625,
	/*35[20] ~ 74[59]*/    
	12136,11669,11223,10797,10389,
	10000, 9628, 9272, 8931, 8605,
	 8293, 7994, 7708, 7434, 7171,
	 6919, 6678, 6446, 6224, 6011,
	 5807, 5610, 5422, 5241, 5067,
	 4900, 4739, 4585, 4437, 4294,
	 4157, 4025, 3897, 3775, 3657,
	 3544, 3434, 3329, 3228, 3130,
	/*75[60] ~ 100[85]*/
	 3036, 2945, 2857, 2773, 2691,
	 2612, 2537, 2463, 2392, 2324,
	 2258, 2194, 2133, 2073, 2016,
	 1960, 1906, 1854, 1804, 1755,
	 1708, 1663, 1618, 1576, 1534,
	 1494
};


/*
 * AriesQ Rev00 board Temperature Table
 */
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
	{ 829,     -70 },
	{ 732,     -30 },
	{ 659,        0 },
	{ 283,      250 },
	{ 174,      400 },
	{ 144,      450 },
	{ 128,      500 },
//	{ 103,      550 },
	{  85,      540 },
	{  63,      650 },
	{  39,      750 },
};


#define TEMP_IDX_ZERO_CELSIUS   2


//jmin : Charging off at 65' and recover at 55' except USA CDMA product

//#define SUPPORT_EVENT_TEMP_BLOCK	
// [ VENTURI SPEC. 
// 	 NORMAL => HIGH_BLOCK: 45, HIGH_RECOV: 40, LOW_BLOCK: 0, LOW_BLOCK: 3
//	 EVENT  => HIGH_BLOCK: 65, HIGH_RECOV: 55
//#define TEMP_HIGH_BLOCK			144 //temper_table[TEMP_IDX_ZERO_CELSIUS+45][0]
//#define TEMP_HIGH_RECOVER		174 //temper_table[TEMP_IDX_ZERO_CELSIUS+43][0]

// [ VENTURI NEW SPEC. by jmin
// 	 NORMAL => HIGH_BLOCK: 65, HIGH_RECOV: 55, LOW_BLOCK: 0, LOW_BLOCK: 3
//   EVENT => not used..
#define TEMP_HIGH_BLOCK			63 //temper_table[TEMP_IDX_ZERO_CELSIUS+45][0]
#define TEMP_HIGH_RECOVER		85 //temper_table[TEMP_IDX_ZERO_CELSIUS+43][0]
#define TEMP_LOW_BLOCK			659 //temper_table[TEMP_IDX_ZERO_CELSIUS+0][0]
#define TEMP_LOW_RECOVER		585 //temper_table[TEMP_IDX_ZERO_CELSIUS+3][0]

#define TEMP_EVENT_HIGH_BLOCK	63 //(65')    224//(63') // It is not in table. temper_table[TEMP_IDX_ZERO_CELSIUS+53+20][0]
#define TEMP_EVENT_HIGH_RECOVER	85 // using video playback   //temper_table[TEMP_IDX_ZERO_CELSIUS+50][0]
// ]



/*
 * Venturi board ADC channel
 */
typedef enum s3c_adc_channel {
	S3C_ADC_BATT_MON = 1,
	S3C_ADC_CHG_CURRENT = 2,
	S3C_ADC_EAR = 3,
	S3C_ADC_TEMPERATURE = 4,
	ENDOFADC
} adc_channel_type;


//#define IRQ_TA_CONNECTED_N    IRQ_EINT(19)
//#define IRQ_TA_CHG_N          IRQ_EINT(25)

//#define IRQ_FUEL_INT_N        IRQ_EINT(8) /* It doesn't work */
#ifdef __FUEL_GAUGES_IC__
#define IRQ_FUEL_INT_N      IRQ_EINT8
#endif

/******************************************************************************
 * Battery driver features
 * ***************************************************************************/
/* #define __TEMP_ADC_VALUE__ */
/* #define __USE_EGPIO__ */
#define __CHECK_BATTERY_V_F__
#define __BATTERY_COMPENSATION__ /* eur-feature */ /* venturi-feature*/
/* #define __CHECK_BOARD_REV__ */
/* #define __BOARD_REV_ADC__ */
/* #define __TEST_DEVICE_DRIVER__ */ /* eur-feature */
/* #define __ALWAYS_AWAKE_DEVICE__  */
/* #define __TEST_MODE_INTERFACE__ */ /* eur-feature */ 
/* #define __TEMP_BLOCK_ECXEPT__ */
#define __CHECK_CHG_CURRENT__
#ifdef __FUEL_GAUGES_IC__
#define __CHECK_BATT_VOLTAGE__
#endif
#define __POPUP_DISABLE_MODE__
#define __SET_TEST_VALUE__
//#define __PSEUDO_BOOT_COMPLETED__
/*****************************************************************************/

#define TOTAL_CHARGING_TIME   (6*60*60*1000)  /* 6 hours */
//#define TOTAL_CHARGING_TIME		(5*60*60*1000)  /* 5 hours */
#define TOTAL_RECHARGING_TIME	(1*60*60*1000+30*60*1000)   /* 1.5 hours */
//#define TOTAL_RECHARGING_TIME   (2*60*60*1000)  /* 2 hours */

#ifdef __TEMP_BLOCK_ECXEPT__
#define CALL_TEMP_EXCEPT_BIT        0
#define DMB_TEMP_EXCEPT_BIT         1
#define MUSIC_TEMP_EXCEPT_BIT       2
#define VIDEO_TEMP_EXCEPT_BIT       3
#define CAMERA_TEMP_EXCEPT_BIT      4
#define INTERNEL_TEMP_EXCEPT_BIT    5
#endif /* __TEMP_BLOCK_ECXEPT__ */

#ifdef __BATTERY_COMPENSATION__
#define COMPENSATE_VIBRATOR     19
#define COMPENSATE_CAMERA           25
#define COMPENSATE_MP3              25 //17
#define COMPENSATE_VIDEO            28
#define COMPENSATE_DMB            30//28
#define COMPENSATE_INTERNET            20
#define COMPENSATE_VOICE_CALL_2G    13
#define COMPENSATE_VOICE_CALL_3G    14
#define COMPENSATE_DATA_CALL        25
#define COMPENSATE_LCD              0
#define COMPENSATE_TA               -230	// jmin : -30 added 
#define COMPENSATE_USB              -130	// jmin : -30 added
#define COMPENSATE_CAM_FALSH        0
#define COMPENSATE_BOOTING      	300
#endif /* __BATTERY_COMPENSATION__ */

//#define SOC_LB_FOR_POWER_OFF      27



/* Venturi taejin */

#define RECHARGE_COND_TIME      (20*1000)   /* 20 seconds */
#define FULL_CHARGE_COND_VOLTAGE    4160
#define RECHARGE_COND_VOLTAGE       4150

#if 0
//#define LPM_RECHARGE_COND_VOLTAGE 4110
#define CURRENT_OF_FULL_CHG         316     /* 170mA => (code*1.5)mV , refer to code table. */ 
//#define LPM_CURRENT_OF_FULL_CHG   316     /* 170mA => (code*1.5)mV , refer to code table. */
#endif 

#define SUPPORT_DOUBLE_CHECK_FULL_CHG
#ifdef SUPPORT_DOUBLE_CHECK_FULL_CHG
#define CURRENT_OF_FULL_CHG_1ST			415 // 0.1C => 0.1 * 2500 = 250mA(+-20mA)  465adc    /* 1.5mV/mA, VICHG_Vol = 3.3V * VICHG_adc / 2^12 (12 bit ADC, MAX 3.3V) */ 
#define CURRENT_OF_FULL_CHG_2ND			70
#else
#define CURRENT_OF_FULL_CHG         316     /* 170mA => (code*1.5)mV , refer to code table. */ 
//#define LPM_CURRENT_OF_FULL_CHG   316     /* 170mA => (code*1.5)mV , refer to code table. */

#define CURRENT_OF_FULL_CHG 			373
#define LPM_CURRENT_OF_FULL_CHG 	335

#define LPM_CURRENT_OF_FULL_CHG_TA 		190
#define LPM_CURRENT_OF_FULL_CHG_USB 	130

#define CURRENT_OF_FULL_CHG_TA 		190
#define CURRENT_OF_FULL_CHG_USB 	130

#endif

#define ADC_12BIT_RESOLUTION        8056    /* 3300mV/4096 = 0.805664063, * scale factor */
#define ADC_12BIT_SCALE             10000   /* scale factor */
#define ADC_CURRENT_FACTOR          15 /* 1mA = 1.5mV */


/* Venturi taejin */


#define BAT_DETECTED        1
#define BAT_NOT_DETECTED    0


//#define BATTERY_DEBUG
#ifdef BATTERY_DEBUG
#define battery_debug(fmt,arg...) printk(KERN_ERR "--------" fmt "\n",## arg)
#else
#define battery_debug(fmt,arg...)
#endif


#ifdef LPM_MODE
void charging_mode_set(unsigned int val);
unsigned int charging_mode_get(void);
#endif
unsigned int get_battery_level(void);
unsigned int is_charging_enabled(void);
#ifdef __TEMP_BLOCK_ECXEPT__
static void batt_set_temper_exception(int bit);
static void batt_clear_temper_exception(int bit);
#endif /* __TEMP_BLOCK_ECXEPT__ */
static int s3c_bat_get_property(struct power_supply *bat_ps, enum power_supply_property psp, union power_supply_propval *val);
static int s3c_power_get_property(struct power_supply *bat_ps,  enum power_supply_property psp, union power_supply_propval *val);
static ssize_t s3c_bat_show_property(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef __BATTERY_COMPENSATION__
static void s3c_bat_set_compesation(int mode, int offset, int compensate_value);
#endif /* __BATTERY_COMPENSATION__ */
//static void s3c_bat_set_vol_cal(int batt_cal);
static int s3c_bat_get_charging_status(void);
static ssize_t s3c_bat_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#ifdef __BATTERY_COMPENSATION__
void s3c_bat_set_compensation_for_drv(int mode, int offset);
#endif /* __BATTERY_COMPENSATION__ */
//void low_battery_power_off(void); /* eur-feature */
static int get_usb_power_state(void);
static int s3c_bat_get_adc_data(adc_channel_type adc_ch);
//static unsigned long calculate_average_adc(adc_channel_type channel, int adc);
static unsigned long s3c_read_temp(void);
static int is_over_abs_time(void);
#ifdef __CHECK_CHG_CURRENT__
//static void check_chg_current(struct power_supply *bat_ps);
static void check_chg_current(void);
static void s3c_check_chg_current(void);
#endif /* __CHECK_CHG_CURRENT__ */
static u32 s3c_get_bat_health(void);
static void s3c_set_bat_health(u32 batt_health);
static void s3c_set_time_for_charging(int mode);
static void s3c_set_chg_en(int enable);
static void s3c_temp_control(int mode);
static void s3c_cable_check_status(void);
void s3c_cable_changed(bool connected);
//void s3c_cable_charging(void);
static int s3c_cable_status_update(void);
static int s3c_get_bat_temp(void);
#ifdef __FUEL_GAUGES_IC__
#ifdef __CHECK_BATT_VOLTAGE__
static void check_batt_voltage(void);
#endif /* __CHECK_BATT_VOLTAGE__ */
#endif
static unsigned long s3c_read_bat(void);
static int s3c_get_bat_vol(void);
static int s3c_get_bat_level(void);
static int s3c_bat_need_recharging(void);
static int s3c_bat_is_full_charged(void);
static void s3c_bat_charging_control(void);
static int batt_read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static void s3c_bat_status_update(void);
static int s3c_read_vol_adc_for_cal(void);
static int s3c_read_vol_for_cal(void);
#ifdef __CHECK_BATTERY_V_F__
static unsigned int s3c_bat_check_v_f(void);
//static void s3c_bat_check_v_f(void);
#endif /* __CHECK_BATTERY_V_F__ */
static void polling_timer_func(unsigned long unused);
static void s3c_store_bat_old_data(void);
static void s3c_bat_work(struct work_struct *work);
static int s3c_bat_create_attrs(struct device * dev);
static void battery_early_suspend(struct early_suspend *h);
static void battery_late_resume(struct early_suspend *h);
#ifdef MAX17043
static irqreturn_t low_battery_isr( int irq, void *_di );
int _low_battery_alarm_(void);
static void fuelgauge_work_handler( struct work_struct *work );
#endif /* MAX17043 */
#ifdef __PSEUDO_BOOT_COMPLETED__
static void boot_complete_work_handler( struct work_struct *work )
#endif /* __PSEUDO_BOOT_COMPLETED__ */
static int __devinit s3c_bat_probe(struct platform_device *pdev);
#ifdef CONFIG_PM
static int s3c_bat_suspend(struct platform_device *pdev, pm_message_t state);
static int s3c_bat_resume(struct platform_device *pdev);
#endif /* CONFIG_PM */

static int __devexit s3c_bat_remove(struct platform_device *pdev);
static int __init s3c_bat_init(void);
static void __exit s3c_bat_exit(void);

