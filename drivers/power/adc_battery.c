/*
 * linux/drivers/power/adc_battery.c
 *
 * Battery measurement code for S3C6410 platform.
 *
 * based on palmtx_battery.c
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <linux/mfd/max8998.h>
#include <plat/gpio-cfg.h>
#include <linux/earlysuspend.h>
#include <linux/io.h>
#include <mach/regs-clock.h>
//#include <mach/regs-power.h>
#include <mach/map.h>
#include <mach/sec_battery.h>
#include <mach/max8998_function.h>

#include "adc_battery.h"
#include <linux/proc_fs.h>

static int batt_quick_off_flag = 0;	/* jmin : if this flag is 1, battery level will be 0 */

static struct wake_lock update_wake_lock;
static struct wake_lock vbus_wake_lock;
static int vbus_wake_lock_status = 0;

#include <linux/i2c.h>

#ifdef __FUEL_GAUGES_IC__
#include "fuel_gauge.c"
#endif /* __FUEL_GAUGES_IC__ */

/* Prototypes */
extern int s3c_adc_get_adc_data(int channel);
extern void MAX8998_IRQ_init(void);
extern void maxim_ta_charging_mode(int mode);
extern void maxim_charging_control(unsigned int dev_type, unsigned int cmd);
//extern void maxim_topoff_change(void); /* eur-feature */
//extern unsigned char maxim_charging_enable_status(void); /* eur-feature */
extern unsigned char maxim_vf_status(void);
//extern u8 FSA9480_Get_JIG_Status(void); /* eur-feature MP3-feature */
//extern void set_low_bat_interrupt(int on); /* eur-feature */

/* prototypes : kor-feature start */
//extern u8 FSA9480_Get_JIG_Status(void);
//extern int FSA9480_Get_I2C_JIG_Status2(void);
/* prototypes : kor-feature end */

#ifdef __TEST_DEVICE_DRIVER__
extern int amp_enable(int);
extern int audio_power(int);

static ssize_t s3c_test_show_property(struct device *dev,
				      struct device_attribute *attr, char *buf);
static ssize_t s3c_test_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count);
static int bat_temper_state = 0;
static struct wake_lock wake_lock_for_dev;
#endif /* __TEST_DEVICE_DRIVER__ */

#define LPM_MODE

#define TRUE        1
#define FALSE   0

#define ADC_DATA_ARR_SIZE   6
#define ADC_TOTAL_COUNT     50
//#define POLLING_INTERVAL  5000

#define POLLING_INTERVAL_TEST   1000


#ifdef __BATTERY_COMPENSATION__
/* Offset Bit Value */
#define OFFSET_VIBRATOR_ON		(0x1 << 0)
#define OFFSET_CAMERA_ON		(0x1 << 1)
#define OFFSET_MP3_PLAY			(0x1 << 2)
#define OFFSET_VIDEO_PLAY		(0x1 << 3)
#define OFFSET_DMB_PLAY			(0x1 << 4)
#define OFFSET_INTERNET_USE		(0x1 << 5)
#define OFFSET_VOICE_CALL_2G		(0x1 << 6)
#define OFFSET_VOICE_CALL_3G		(0x1 << 7)
#define OFFSET_DATA_CALL		(0x1 << 8)
#define OFFSET_LCD_ON			(0x1 << 9)
#define OFFSET_TA_ATTACHED		(0x1 << 10)
#define OFFSET_USB_ATTACHED		(0x1 << 11)
#define OFFSET_CAM_FLASH		(0x1 << 12)
#define OFFSET_BOOTING			(0x1 << 13)
#endif /* __BATTERY_COMPENSATION__ */

#ifndef __FUEL_GAUGES_IC__
#ifdef __9BITS_RESOLUTION__
#define INVALID_VOL_ADC		20
#else /* __9BITS_RESOLUTION__ */
#define INVALID_VOL_ADC		250		/* jmin : Charger ADC compensation is -230 (before 160)*/
#endif /* __9BITS_RESOLUTION__ */
#endif

/* defines : kor-feature start */
#define APPLY_5SEC_POLLING
//#define APPLY_2SEC_POLLING
#if defined(APPLY_5SEC_POLLING)
#define POLLING_INTERVAL    (30*1000)
//#define CHG_RESLEEP_COUNT 1 //for 3 times check of batt info.
//#define FULL_CHG_COND_COUNT 6
#define FULL_CHG_COND_COUNT 4
#elif defined(APPLY_2SEC_POLLING)
#define POLLING_INTERVAL    2000
#define CHG_RESLEEP_COUNT   3 //for 5 times check of batt info.
#define FULL_CHG_COND_COUNT 10
#else
#error "unknown poll_interval!"
#endif
#define PROC_FILENAME   ("batt_info_proc")
/* defines : kor-feature end */

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

/*
static char *status_text[] = {
    [POWER_SUPPLY_STATUS_UNKNOWN]       =   "Unknown",
    [POWER_SUPPLY_STATUS_CHARGING]      =   "Charging",
    [POWER_SUPPLY_STATUS_DISCHARGING]       =   "Discharging",
    [POWER_SUPPLY_STATUS_NOT_CHARGING]  =   "Not Charging",
    [POWER_SUPPLY_STATUS_FULL]              =   "Full",
};
*/

struct battery_info {
	u32 batt_id;		/* Battery ID from ADC */
	s32 batt_vol;		/* Battery voltage from ADC */
	s32 batt_vol_adc;	/* Battery ADC value */
	s32 batt_vol_adc_cal;	/* Battery ADC value (calibrated) */
	s32 batt_temp;		/* Battery Temperature (C) from ADC */
	s32 batt_temp_adc;	/* Battery Temperature ADC value */
	s32 batt_temp_adc_cal;	/* Battery Temperature ADC value (calibrated) */
	s32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 batt_is_full;	/* 0 : Not full 1: Full */
	u32 batt_is_recharging;	/* 0 : Not recharging 1: Recharging */
	s32 batt_vol_adc_aver;	/* batt vol adc average */
#ifdef __TEST_MODE_INTERFACE__
	u32 batt_test_mode;	/* test mode */
	s32 batt_vol_aver;	/* batt vol average */
	s32 batt_temp_aver;	/* batt temp average */
	s32 batt_temp_adc_aver;	/* batt temp adc average */
#endif				/* __TEST_MODE_INTERFACE__ */

	s32 batt_v_f_adc;	/* batt V_F adc */
	s32 batt_current_adc;	/* batt current adc */
	s32 batt_current_adc_aver;	/* batt current adc average */
};

struct s3c_battery_info {
	struct sec_bat_platform_data *pdata;
	int present;
	int polling;
	unsigned int polling_interval;
	enum cable_type_t cable_status;
#ifdef __BATTERY_COMPENSATION__
	unsigned int device_state;
#endif /* __BATTERY_COMPENSATION__ */

	struct battery_info bat_info;
#ifdef LPM_MODE
	unsigned int charging_mode_booting;
#endif
	struct max8998_charger_callbacks callbacks;
};

static enum power_supply_property s3c_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property s3c_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply s3c_power_supplies[] = {
	{
	 .name = "battery",
	 .type = POWER_SUPPLY_TYPE_BATTERY,
	 .properties = s3c_battery_properties,
	 .num_properties = ARRAY_SIZE(s3c_battery_properties),
	 .get_property = s3c_bat_get_property,
	 },
	{
	 .name = "usb",
	 .type = POWER_SUPPLY_TYPE_USB,
	 .supplied_to = supply_list,
	 .num_supplicants = ARRAY_SIZE(supply_list),
	 .properties = s3c_power_properties,
	 .num_properties = ARRAY_SIZE(s3c_power_properties),
	 .get_property = s3c_power_get_property,
	 },
	{
	 .name = "ac",
	 .type = POWER_SUPPLY_TYPE_MAINS,
	 .supplied_to = supply_list,
	 .num_supplicants = ARRAY_SIZE(supply_list),
	 .properties = s3c_power_properties,
	 .num_properties = ARRAY_SIZE(s3c_power_properties),
	 .get_property = s3c_power_get_property,
	 },
};

#define SEC_BATTERY_ATTR(_name)                             \
{                                           \
		.attr = { .name = #_name, .mode = 0664, .owner = THIS_MODULE },    \
		.show = s3c_bat_show_property,                          \
		.store = s3c_bat_store,                             \
}

static struct device_attribute s3c_battery_attrs[] = {
	SEC_BATTERY_ATTR(batt_vol),
	SEC_BATTERY_ATTR(batt_vol_adc),
	SEC_BATTERY_ATTR(batt_vol_adc_cal),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(batt_temp_adc),
	SEC_BATTERY_ATTR(batt_temp_adc_cal),
	SEC_BATTERY_ATTR(batt_vol_adc_aver),
#ifdef __TEST_MODE_INTERFACE__
	/* test mode */
	SEC_BATTERY_ATTR(batt_test_mode),
	/* average */
	SEC_BATTERY_ATTR(batt_vol_aver),
	SEC_BATTERY_ATTR(batt_temp_aver),
	SEC_BATTERY_ATTR(batt_temp_adc_aver),
#endif /* __TEST_MODE_INTERFACE__ */
	SEC_BATTERY_ATTR(batt_v_f_adc),
#ifdef __CHECK_CHG_CURRENT__
	SEC_BATTERY_ATTR(batt_chg_current),
	SEC_BATTERY_ATTR(batt_chg_current_adc),
#endif /* __CHECK_CHG_CURRENT__ */
	SEC_BATTERY_ATTR(charging_source),

// [ These are used for BATT __BATTERY_COMPENSATION__ and TEMP_BLOCK_ECXEPT together.
	SEC_BATTERY_ATTR(talk_gsm),
	SEC_BATTERY_ATTR(talk_wcdma),
	SEC_BATTERY_ATTR(is_booting),
// ]
#ifdef __BATTERY_COMPENSATION__
	SEC_BATTERY_ATTR(vibrator),
	SEC_BATTERY_ATTR(camera),
	SEC_BATTERY_ATTR(mp3),
	SEC_BATTERY_ATTR(video),
	SEC_BATTERY_ATTR(dmb),
	SEC_BATTERY_ATTR(internet),
	SEC_BATTERY_ATTR(data_call),
	SEC_BATTERY_ATTR(device_state),
	SEC_BATTERY_ATTR(batt_compensation),
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __TEMP_BLOCK_ECXEPT__
	SEC_BATTERY_ATTR(dmb_play),
	SEC_BATTERY_ATTR(music_play),
	SEC_BATTERY_ATTR(video_play),
	SEC_BATTERY_ATTR(camera_use),
	SEC_BATTERY_ATTR(internet_use),
#endif /* __TEMP_BLOCK_ECXEPT__ */

	SEC_BATTERY_ATTR(fg_soc),
#ifdef __FUEL_GAUGES_IC__
	SEC_BATTERY_ATTR(fg_vcell),
	SEC_BATTERY_ATTR(fg_pure_soc),
	SEC_BATTERY_ATTR(reset_soc),
#endif
#ifdef LPM_MODE
	SEC_BATTERY_ATTR(charging_mode_booting),
	SEC_BATTERY_ATTR(batt_temp_check),
	SEC_BATTERY_ATTR(batt_full_check),
#endif
#ifdef __SET_TEST_VALUE__
	SEC_BATTERY_ATTR(batt_test_value),
	SEC_BATTERY_ATTR(batt_rtctic_test),
#endif /* __SET_TEST_VALUE__ */
	SEC_BATTERY_ATTR(hw_revision),
	SEC_BATTERY_ATTR(batt_quick_off),
	SEC_BATTERY_ATTR(charging_ta_mode),
	SEC_BATTERY_ATTR(batt_vol_adc_for_cal),
	SEC_BATTERY_ATTR(batt_vol_for_cal),
};

enum {
	BATT_VOL = 0,
	BATT_VOL_ADC,
	BATT_VOL_ADC_CAL,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_ADC_CAL,
	BATT_VOL_ADC_AVER,
#ifdef __TEST_MODE_INTERFACE__
	BATT_TEST_MODE,
	BATT_VOL_AVER,
	BATT_TEMP_AVER,
	BATT_TEMP_ADC_AVER,
#endif /* __TEST_MODE_INTERFACE__ */
	BATT_V_F_ADC,
#ifdef __CHECK_CHG_CURRENT__
	BATT_CHG_CURRENT,
	BATT_CHG_CURRENT_ADC,
#endif /* __CHECK_CHG_CURRENT__ */
	BATT_CHARGING_SOURCE,

// [ These are used for BATT __BATTERY_COMPENSATION__ and TEMP_BLOCK_ECXEPT together.
	BATT_VOICE_CALL_2G,
	BATT_VOICE_CALL_3G,
	BATT_BOOTING,
// ]
#ifdef __BATTERY_COMPENSATION__
	BATT_VIBRATOR,
	BATT_CAMERA,
	BATT_MP3,
	BATT_VIDEO,
	BATT_DMB,
	BATT_INTERNET,
	BATT_DATA_CALL,
	BATT_DEV_STATE,
	BATT_COMPENSATION,
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __TEMP_BLOCK_ECXEPT__
	BATT_DMB_PLAY,
	BATT_MUSIC_PLAY,
	BATT_VIDEO_PLAY,
	BATT_CAMERA_USE,
	BATT_INTERNET_USE,
#endif /* __TEMP_BLOCK_ECXEPT__ */

// [ Even if the fuelgauge isn't used in some projcet,
//   we must use BATT_FG_SOC to provide battery level to lpm demon. 
	BATT_FG_SOC,
// ]
#ifdef __FUEL_GAUGES_IC__
	BATT_FG_VCELL,
	BATT_FG_PURE_SOC,
	BATT_RESET_SOC,
#endif

#ifdef LPM_MODE
	CHARGING_MODE_BOOTING,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
#endif
#ifdef __SET_TEST_VALUE__
	BATT_TEST_VALUE,
	BATT_RTCTIC_TEST,
#endif /* __SET_TEST_VALUE__ */
	BATT_HW_REVISION,
	BATT_QUICK_OFF,
	CHARGING_TA_MODE,
	BATT_VOL_ADC_FOR_CAL,
	BATT_VOL_FOR_CAL,
};

#ifndef __FUEL_GAUGES_IC__
struct adc_sample_info {
	unsigned int cnt;
	int total_adc;
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};

static struct adc_sample_info adc_sample[ENDOFADC];
#endif

struct battery_driver {
	struct early_suspend early_suspend;
};
struct battery_driver *battery = NULL;

//static int batt_chg_full_1st=0; /* eur-feature */

/* lock to protect the battery info */
static DEFINE_MUTEX(work_lock);

static struct work_struct bat_work;
static struct device *dev;
static struct timer_list polling_timer;
static int s3c_battery_initial;
static int force_update, force_log;
static int old_level, old_temp, old_is_full, old_is_recharging, old_health, new_temp_level;
static charger_type_t cable_status = CHARGER_BATTERY;

#ifndef __FUEL_GAUGES_IC__
static int batt_max;
static int batt_full;
static int batt_safe_rech;
static int batt_almost;
static int batt_high;
static int batt_medium;
static int batt_low;
static int batt_critical;
static int batt_min;
static int batt_off;
#endif
#ifdef __BATTERY_COMPENSATION__
static int batt_compensation;
#endif /* __BATTERY_COMPENSATION__ */

static unsigned int start_time_msec;
static unsigned int total_time_msec;
static unsigned int end_time_msec;

static struct s3c_battery_info s3c_bat_info;

static int full_charge_flag;

/* Variables : kor-feature start */
#ifdef __FUEL_GAUGES_IC__
static struct delayed_work fuelgauge_work;
#endif
#ifdef __PSEUDO_BOOT_COMPLETED__
static struct delayed_work boot_complete_work;
#endif /* __PSEUDO_BOOT_COMPLETED__ */
extern unsigned int HWREV;
extern unsigned int FGPureSOC;
extern unsigned int is_cal_ftm_sleep;
//extern unsigned int full_soc;
static struct proc_dir_entry *entry;
//int top_off_interrupted;
static int iboot_completed;
//static int full_charge_count;
static int rechg_count;
static int full_check_count;
#ifdef __SET_TEST_VALUE__
static int ibatt_test_value;
static int en_current_test;
#endif /* __SET_TEST_VALUE__ */
#ifdef __TEMP_BLOCK_ECXEPT__
static int except_temp_block;
//static int except_temp_count;
#endif /* __TEMP_BLOCK_ECXEPT__ */
int isVoiceCall = 0;
EXPORT_SYMBOL(isVoiceCall);
/* Variables : kor-feature end */

#ifdef __TEST_DEVICE_DRIVER__
#define SEC_TEST_ATTR(_name)                                \
{                                           \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO, .owner = THIS_MODULE },    \
	.show = s3c_test_show_property,                         \
	.store = s3c_test_store,                            \
}

static struct device_attribute s3c_test_attrs[] = {
	SEC_TEST_ATTR(pm),
	SEC_TEST_ATTR(usb),
	SEC_TEST_ATTR(bt_wl),
	SEC_TEST_ATTR(tflash),
	SEC_TEST_ATTR(audio),
	SEC_TEST_ATTR(lcd),
	SEC_TEST_ATTR(suspend_lock),
	SEC_TEST_ATTR(control_tmp),
};

enum {
	TEST_PM = 0,
	USB_OFF,
	BT_WL_OFF,
	TFLASH_OFF,
	AUDIO_OFF,
	LCD_CHECK,
	SUSPEND_LOCK,
	CTRL_TMP,
};

static int s3c_test_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(s3c_test_attrs); i++) {
		rc = device_create_file(dev, &s3c_test_attrs[i]);
		if (rc)
			goto s3c_attrs_failed;
	}
	goto succeed;

s3c_attrs_failed:
	while (i--)
		device_remove_file(dev, &s3c_test_attrs[i]);
succeed:
	return rc;
}

static void s3c_lcd_check(void)
{
/*
	unsigned char reg_buff = 0;
	if (Get_MAX8698_PM_REG(ELDO6, &reg_buff)) {
		pr_info("%s: VLCD 1.8V (%d)\n", __func__, reg_buff);
	}
	if ((Get_MAX8698_PM_REG(ELDO7, &reg_buff))) {
		pr_info("%s: VLCD 2.8V (%d)\n", __func__, reg_buff);
	}
*/
}

static void s3c_usb_off(void)
{
/*
	unsigned char reg_buff = 0;
	if (Get_MAX8698_PM_REG(ELDO3, &reg_buff)) {
		pr_info("%s: OTGI 1.2V off(%d)\n", __func__, reg_buff);
		if (reg_buff)
			Set_MAX8698_PM_REG(ELDO3, 0);
	}
	if ((Get_MAX8698_PM_REG(ELDO8, &reg_buff))) {
		pr_info("%s: OTG 3.3V off(%d)\n", __func__, reg_buff);
		if (reg_buff)
			Set_MAX8698_PM_REG(ELDO8, 0);
	}
*/
}

static void s3c_bt_wl_off(void)
{
/*
	unsigned char reg_buff = 0;
	if (Get_MAX8698_PM_REG(ELDO4, &reg_buff)) {
		pr_info("%s: BT_WL 2.6V off(%d)\n", __func__, reg_buff);
		if (reg_buff)
			Set_MAX8698_PM_REG(ELDO4, 0);
	}
*/
}

static void s3c_tflash_off(void)
{
/*
	unsigned char reg_buff = 0;
	if (Get_MAX8698_PM_REG(ELDO5, &reg_buff)) {
		pr_info("%s: TF 3.0V off(%d)\n", __func__, reg_buff);
		if (reg_buff)
			Set_MAX8698_PM_REG(ELDO5, 0);
	}
*/
}

static void s3c_audio_off(void)
{
/*
	pr_info("%s: Turn off audio power, amp\n", __func__);
	amp_enable(0);
	audio_power(0);
*/
}

static void s3c_test_pm(void)
{
#if 0
	/* PMIC */
	s3c_usb_off();
	s3c_bt_wl_off();
	s3c_tflash_off();
	s3c_lcd_check();

	/* AUDIO */
	s3c_audio_off();

	/* GPIO */
#endif
}

static ssize_t s3c_test_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - s3c_test_attrs;

	switch (off) {
	case TEST_PM:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		s3c_test_pm();
		break;
	case USB_OFF:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 1);
		s3c_usb_off();
		break;
	case BT_WL_OFF:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 2);
		s3c_bt_wl_off();
		break;
	case TFLASH_OFF:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 3);
		s3c_tflash_off();
		break;
	case AUDIO_OFF:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 4);
		s3c_audio_off();
		break;
	case LCD_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 5);
		s3c_lcd_check();
		break;
	default:
		i = -EINVAL;
	}

	return i;
}

static ssize_t s3c_test_store(struct device *dev, 
                 struct device_attribute *attr,
                 const char *buf, size_t count)
{
	int mode = 0;
	int ret = 0;
	const ptrdiff_t off = attr - s3c_test_attrs;

	switch (off) {
	case SUSPEND_LOCK:
		if (sscanf(buf, "%d\n", &mode) == 1) {
			//dev_dbg(dev, "%s: suspend_lock(%d)\n", __func__, mode);
			if (mode)
				wake_lock(&wake_lock_for_dev);
			else
				wake_lock_timeout(&wake_lock_for_dev, HZ / 2);
			ret = count;
		}
		break;
	case CTRL_TMP:
		if (sscanf(buf, "%d\n", &mode) == 1) {
			//dev_info(dev, "%s: control tmp(%d)\n", __func__, mode);
			bat_temper_state = mode;
			ret = count;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}
#endif /* __TEST_DEVICE_DRIVER__ */

#ifdef LPM_MODE
void charging_mode_set(unsigned int val)
{
	s3c_bat_info.charging_mode_booting = val;
}

unsigned int charging_mode_get(void)
{
	return s3c_bat_info.charging_mode_booting;
}
#endif

unsigned int get_battery_level(void)
{
	return s3c_bat_info.bat_info.level;
}

unsigned int is_charging_enabled(void)
{
	return s3c_bat_info.bat_info.charging_enabled;
}

#ifdef __TEMP_BLOCK_ECXEPT__
static void batt_set_temper_exception(int bit)
{
	/* for temper. blocking exception */
	except_temp_block |= (0x1 << bit);

	/* if it is already abnormal health, don't change the health to good - DF23 */
	/*
	if((s3c_get_bat_health() == POWER_SUPPLY_HEALTH_OVERHEAT) || (s3c_get_bat_health() == POWER_SUPPLY_HEALTH_COLD))
	{
		s3c_set_bat_health(POWER_SUPPLY_HEALTH_GOOD);
		schedule_work(&bat_work); //force update
	}
	*/
}   

static void batt_clear_temper_exception(int bit)
{
	except_temp_block &= ~(0x1 << bit);
}
#endif /* __TEMP_BLOCK_ECXEPT__ */

static int s3c_bat_get_property(struct power_supply *bat_ps,
		     enum power_supply_property psp,
		     union power_supply_propval *val)
{
	//pr_info("[BAT]:%s\n", __func__);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s3c_bat_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s3c_get_bat_health();
		//pr_info("[BAT]:%s:HEALTH=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s3c_bat_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = s3c_bat_info.bat_info.level;
		//pr_info("[BAT]:%s:LEVEL=%d\n", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = s3c_bat_info.bat_info.batt_temp;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int s3c_power_get_property(struct power_supply *bat_ps,
		       enum power_supply_property psp,
		       union power_supply_propval *val)
{
	//pr_info("[BAT]:%s\n", __func__);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (s3c_bat_info.cable_status== CABLE_TYPE_AC ? 1 : 0);
		} else if (bat_ps->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (s3c_bat_info.cable_status== CABLE_TYPE_USB ? 1 : 0);
		} else {
			val->intval = 0;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t s3c_bat_show_property(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - s3c_battery_attrs;

	//pr_info("[BAT]:%s\n", __func__);

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_vol);
		break;
	case BATT_VOL_ADC:
		//s3c_bat_info.bat_info.batt_vol_adc = 0;
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_vol_adc);
		break;
	case BATT_VOL_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_vol_adc_cal);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_temp);
		break;
	case BATT_TEMP_ADC:
		//s3c_bat_info.bat_info.batt_temp_adc = s3c_bat_get_adc_data(S3C_ADC_TEMPERATURE);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_temp_adc);
		break;
#ifdef __TEST_MODE_INTERFACE__
	case BATT_TEST_MODE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_test_mode);
		break;
#endif /* __TEST_MODE_INTERFACE__ */
	case BATT_TEMP_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_temp_adc_cal);
		break;
	case BATT_VOL_ADC_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_vol_adc_aver);
		break;
#ifdef __TEST_MODE_INTERFACE__
	case BATT_VOL_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_vol_aver);
		break;
	case BATT_TEMP_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_temp_aver);
		break;
	case BATT_TEMP_ADC_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_temp_adc_aver);
		break;
#endif /* __TEST_MODE_INTERFACE__ */
	case BATT_V_F_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_v_f_adc);
		break;
#ifdef __CHECK_CHG_CURRENT__
	case BATT_CHG_CURRENT:
		//s3c_bat_info.bat_info.batt_current = 
		//  s3c_bat_get_adc_data(ADC_CHG_CURRENT); //this is adc value, so do not use it.
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_current);
		break;
	case BATT_CHG_CURRENT_ADC:
		//s3c_bat_info.bat_info.batt_current = 
		//  s3c_bat_get_adc_data(ADC_CHG_CURRENT); //this is adc value, so do not use it.
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_current_adc);
		break;
#endif /* __CHECK_CHG_CURRENT__ */
	case BATT_CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.charging_source);
		break;
#ifdef __BATTERY_COMPENSATION__
	case BATT_DEV_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%08x\n",
			       s3c_bat_info.device_state);
		break;
	case BATT_COMPENSATION:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       batt_compensation);
		break;
#endif /* __BATTERY_COMPENSATION__ */

	case BATT_FG_SOC:
		mutex_lock(&work_lock);
#ifdef __FUEL_GAUGES_IC__
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", fg_read_soc());
#else
		s3c_get_bat_vol();
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_get_bat_level());
#endif
		mutex_unlock(&work_lock);
		break;
#ifdef __FUEL_GAUGES_IC__
	case BATT_FG_VCELL:
		mutex_lock(&work_lock);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", fg_read_vcell());
		mutex_unlock(&work_lock);
		break;
	case BATT_FG_PURE_SOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", FGPureSOC);
		break;
#endif

#ifdef LPM_MODE
	case CHARGING_MODE_BOOTING:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       charging_mode_get());
		break;
	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_health);
		break;
	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       s3c_bat_info.bat_info.batt_is_full);
		break;
#endif
#ifdef __SET_TEST_VALUE__
	case BATT_TEST_VALUE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       ibatt_test_value);
		break;
	case BATT_RTCTIC_TEST:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", en_current_test);
		break;
#endif /* __SET_TEST_VALUE__ */
	case BATT_HW_REVISION:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", HWREV);
		break;
	case BATT_VOL_ADC_FOR_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", s3c_read_vol_adc_for_cal());
		break;
	case BATT_VOL_FOR_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", s3c_read_vol_for_cal());
		break;
	default:
		i = -EINVAL;
	}

	return i;
}

#ifdef __BATTERY_COMPENSATION__
static void s3c_bat_set_compesation(int mode, int offset, int compensate_value)
{

	//pr_info("[BAT]:%s\n", __func__);

	if (mode) {
		if (!(s3c_bat_info.device_state & offset)) {
			s3c_bat_info.device_state |= offset;
			batt_compensation += compensate_value;
		}
	} else {
		if (s3c_bat_info.device_state & offset) {
			s3c_bat_info.device_state &= ~offset;
			batt_compensation -= compensate_value;
		}
	}
	pr_info("[BAT]:%s: device_state=0x%x, compensation=%d\n", __func__, s3c_bat_info.device_state, batt_compensation);

}
#endif /* __BATTERY_COMPENSATION__ */

#if 0
static void s3c_bat_set_vol_cal(int batt_cal)
{
	int max_cal = 4096;

	//pr_info("[BAT]:%s\n", __func__);

	if (!batt_cal) {
		return;
	}

	if (batt_cal >= max_cal) {
		//dev_err(dev, "%s: invalid battery_cal(%d)\n", __func__, batt_cal);
		return;
	}

	batt_max = batt_cal + BATT_MAXIMUM;
	batt_full = batt_cal + BATT_FULL;
	batt_safe_rech = batt_cal + BATT_SAFE_RECHARGE;
	batt_almost = batt_cal + BATT_ALMOST_FULL;
	batt_high = batt_cal + BATT_HIGH;
	batt_medium = batt_cal + BATT_MED;
	batt_low = batt_cal + BATT_LOW;
	batt_critical = batt_cal + BATT_CRITICAL;
	batt_min = batt_cal + BATT_MINIMUM;
	batt_off = batt_cal + BATT_OFF;
}
#endif

static int s3c_bat_get_charging_status(void)
{
	charger_type_t charger = CHARGER_BATTERY;
	int ret = 0;

	//pr_info("[BAT]:%s\n", __func__);

	charger = s3c_bat_info.bat_info.charging_source;

	/* use for factory test, disable low battery popup */
	if (ibatt_test_value == 777) {
		pr_info("%s: status unknown, ibatt_test_value = %d\n", __func__,
			ibatt_test_value);
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		return ret;
	}

/*
  * Discharging and Not-charging is switched in GB from FR
  */
	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		if (s3c_bat_info.bat_info.batt_is_full) {
			ret = POWER_SUPPLY_STATUS_FULL;
#ifdef __POPUP_DISABLE_MODE__
			if (ibatt_test_value == 999) {
				ret = POWER_SUPPLY_STATUS_CHARGING;
			}
#endif /* __POPUP_DISABLE_MODE__ */
		} else {
			ret = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case CHARGER_DISCHARGE:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return ret;
}

static ssize_t s3c_bat_store(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - s3c_battery_attrs;

	//pr_info("[BAT]:%s\n", __func__);

	switch (off) {
	case BATT_VOL_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_vol_adc_cal = x;
			//s3c_bat_set_vol_cal(x);
			ret = count;
		}
		//pr_info("[BAT]:%s: batt_vol_adc_cal = %d\n", __func__, x);
		break;
	case BATT_TEMP_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_temp_adc_cal = x;
			ret = count;
		}
		//pr_info("[BAT]:%s: batt_temp_adc_cal = %d\n", __func__, x);
		break;
#ifdef __TEST_MODE_INTERFACE__
	case BATT_TEST_MODE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_test_mode = x;
			ret = count;
		}
		if (s3c_bat_info.bat_info.batt_test_mode) {
			s3c_bat_info.polling_interval = POLLING_INTERVAL_TEST;
			/*              if (s3c_bat_info.polling)
			   {
			   del_timer_sync(&polling_timer);
			   }
			   mod_timer(&polling_timer, jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
			   s3c_bat_status_update();
			 */
		} else {
			s3c_bat_info.polling_interval = POLLING_INTERVAL;
			/*              if (s3c_bat_info.polling)
			   {
			   del_timer_sync(&polling_timer);
			   }
			   mod_timer(&polling_timer,jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
			   s3c_bat_status_update();
			 */
		}
		//pr_info("[BAT]:%s: batt_test_mode = %d\n", __func__, x);
		break;
#endif /* __TEST_MODE_INTERFACE__ */

// [ These are used for BATT __BATTERY_COMPENSATION__ and TEMP_BLOCK_ECXEPT together.
	case BATT_VOICE_CALL_2G:
		if (sscanf(buf, "%d\n", &x) == 1) {
#ifdef __BATTERY_COMPENSATION__
			s3c_bat_set_compesation(x, OFFSET_VOICE_CALL_2G,
						COMPENSATE_VOICE_CALL_2G);
#endif

#ifdef __TEMP_BLOCK_ECXEPT__
			if (x == 1) {
				isVoiceCall = 1;
				batt_set_temper_exception(CALL_TEMP_EXCEPT_BIT);
			} else {
				isVoiceCall = 0;
				batt_clear_temper_exception
				    (CALL_TEMP_EXCEPT_BIT);
			}
#endif
			ret = count;
		}
		//pr_info("[BAT]:%s: voice call 2G = %d\n", __func__, x);
		break;
	case BATT_VOICE_CALL_3G:
		if (sscanf(buf, "%d\n", &x) == 1) {
#ifdef __BATTERY_COMPENSATION__
			s3c_bat_set_compesation(x, OFFSET_VOICE_CALL_3G,
						COMPENSATE_VOICE_CALL_3G);
#endif

#ifdef __TEMP_BLOCK_ECXEPT__
			if (x == 1) {
				isVoiceCall = 1;
				batt_set_temper_exception(CALL_TEMP_EXCEPT_BIT);
			} else {
				isVoiceCall = 0;
				batt_clear_temper_exception
				    (CALL_TEMP_EXCEPT_BIT);
			}
#endif
			ret = count;
		}
		//pr_info("[BAT]:%s: voice call 3G = %d\n", __func__, x);
		break;

	case BATT_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
#if defined(__BATTERY_COMPENSATION__) && defined(COMPENSATE_BOOTING)
			s3c_bat_set_compesation(x, OFFSET_BOOTING,
						COMPENSATE_BOOTING);
#endif

#ifdef __TEMP_BLOCK_ECXEPT__
			if (x == 1) {
				iboot_completed = 1;
				printk
				    ("[battery] boot completed! - iboot_completed = %d\n",
				     iboot_completed);
			}
#endif
			ret = count;
		}
		//pr_info("[BAT]:%s: boot complete = %d\n", __func__, x);
		break;
// ]

#ifdef __BATTERY_COMPENSATION__
	case BATT_VIBRATOR:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_VIBRATOR_ON,
						COMPENSATE_VIBRATOR);
			ret = count;
		}
		//pr_info("[BAT]:%s: vibrator = %d\n", __func__, x);
		break;
	case BATT_CAMERA:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_CAMERA_ON,
						COMPENSATE_CAMERA);
			ret = count;
		}
		//pr_info("[BAT]:%s: camera = %d\n", __func__, x);
		break;
	case BATT_MP3:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_MP3_PLAY,
						COMPENSATE_MP3);
			ret = count;
		}
		//pr_info("[BAT]:%s: mp3 = %d\n", __func__, x);
		break;
	case BATT_VIDEO:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_VIDEO_PLAY,
						COMPENSATE_VIDEO);
			ret = count;
		}
		break;
	case BATT_DMB:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_DMB_PLAY,
						COMPENSATE_DMB);
			ret = count;
		}
		break;
	case BATT_INTERNET:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_INTERNET_USE,
						COMPENSATE_INTERNET);
			ret = count;
		}
		//pr_info("[BAT]:%s: video = %d\n", __func__, x);
		break;
	case BATT_DATA_CALL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_DATA_CALL,
						COMPENSATE_DATA_CALL);
			ret = count;
		}
		//pr_info("[BAT]:%s: data call = %d\n", __func__, x);
		break;
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __TEMP_BLOCK_ECXEPT__
	case BATT_DMB_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(DMB_TEMP_EXCEPT_BIT);
			} else {
					batt_clear_temper_exception(DMB_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		//dev_info(dev, "%s: BATT_DMB_PLAY = %d\n", __func__, x);
		break;
	case BATT_MUSIC_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(MUSIC_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(MUSIC_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		//dev_info(dev, "%s: BATT_MUSIC_PLAY = %d\n", __func__, x);
		break;
	case BATT_VIDEO_PLAY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(VIDEO_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(VIDEO_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		//dev_info(dev, "%s: BATT_VIDEO_PLAY = %d\n", __func__, x);
		break;
	case BATT_CAMERA_USE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(CAMERA_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(CAMERA_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		//dev_info(dev, "%s: BATT_CAMERA_USE = %d\n", __func__, x);
		break;
	case BATT_INTERNET_USE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				batt_set_temper_exception(INTERNEL_TEMP_EXCEPT_BIT);
			} else {
				batt_clear_temper_exception(INTERNEL_TEMP_EXCEPT_BIT);
			}
			ret = count;
		}
		//dev_info(dev, "%s: BATT_INTERNET_USE = %d\n", __func__, x);
		break;
#endif /* __TEMP_BLOCK_ECXEPT__ */

#ifdef __FUEL_GAUGES_IC__
	case BATT_RESET_SOC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1) {
				/* use for factory test, disable low battery popup */
				ibatt_test_value = 777;
				/* change ftm sleep timeout value for SMD Function Test */
				if (is_cal_ftm_sleep != 1) {
					is_cal_ftm_sleep = 1;
				}

				mutex_lock(&work_lock);
				fg_reset_soc();
				mutex_unlock(&work_lock);
			} else if (x == 2) {
				mutex_lock(&work_lock);
				fg_reset_soc();
				mutex_unlock(&work_lock);
			}
			ret = count;
		}
		dev_info(dev, "%s: Reset SOC:%d\n", __func__, x);
		break;
#endif
#ifdef LPM_MODE
	case CHARGING_MODE_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			charging_mode_set(x);
			ret = count;
		}
		//pr_info("[BAT]:%s: CHARGING_MODE_BOOTING:%d\n", __func__, x);
		break;
#endif
#ifdef __SET_TEST_VALUE__
	case BATT_TEST_VALUE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0) {
				ibatt_test_value = 0;
			} else if (x == 1) {
				ibatt_test_value = 1;	// for temp warning event
			} else if (x == 2) {
				ibatt_test_value = 2;	// for full event
			}
#if 1
			else if (x == 3) {
				ibatt_test_value = 3;	// for abs time event
			}
#endif
			else if (x == 999) {
				ibatt_test_value = 999;	// for pop-up disable
				/* make the charging possible in case of temper. blocking state */
				if ((s3c_get_bat_health() == POWER_SUPPLY_HEALTH_OVERHEAT)
				    || (s3c_get_bat_health() == POWER_SUPPLY_HEALTH_COLD)) {
					s3c_set_bat_health(POWER_SUPPLY_HEALTH_GOOD);
					schedule_work(&bat_work);	//force update
				}
			} else {
				ibatt_test_value = 0;
			}
		}
		dev_info(dev, "%s: BATT_TEST_VALUE:%d, ibatt_test_value: %d\n", __func__, x, ibatt_test_value);
		break;
	case BATT_RTCTIC_TEST:	/* early_suspend current test */
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0) {
				en_current_test = 0;
				wake_lock_timeout(&vbus_wake_lock, 5 * HZ);
			} else {
				en_current_test = 1;
				wake_lock(&vbus_wake_lock);
			}
		}
		dev_info(dev, "%s: BATT_TEST_VALUE:%d, early_suspend current test : %d\n", __func__, x, en_current_test);
		break;
#endif /* __SET_TEST_VALUE__ */

	case BATT_QUICK_OFF:
		batt_quick_off_flag = 1;	/* if this is called, battery capacity will be always 0 */
		break;

	case CHARGING_TA_MODE:
		if (s3c_bat_info.cable_status== CABLE_TYPE_AC) {
			if (sscanf(buf, "%d\n", &x) == 1) {
				printk("[BAT]:%s: TA_MODE = %d  (1: 700mA, 2: 550mA)\n", __func__, x);
				maxim_ta_charging_mode(x);
				ret = count;
			}
		}
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

#ifdef __BATTERY_COMPENSATION__
void s3c_bat_set_compensation_for_drv(int mode, int offset)
{

	//pr_info("[BAT]:%s\n", __func__);

	switch (offset) {
	case OFFSET_VIBRATOR_ON:
		//pr_info("[BAT]:%s: vibrator = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_VIBRATOR);
		break;
	case OFFSET_LCD_ON:
		//pr_info("[BAT]:%s: LCD On = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_LCD);
		break;
	case OFFSET_CAM_FLASH:
		//pr_info("[BAT]:%s: flash = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_CAM_FALSH);
		break;
	default:
		break;
	}

}

EXPORT_SYMBOL(s3c_bat_set_compensation_for_drv);
#endif /* __BATTERY_COMPENSATION__ */


#if 0 /* eur-feature */
void low_battery_power_off(void)
{
	s3c_bat_info.bat_info.level = 0;

	schedule_work(&bat_work);
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}
#endif /* eur-feature */

static int get_usb_power_state(void)
{

	//pr_info("[BAT]:%s\n", __func__);

	if (s3c_bat_info.cable_status == CABLE_TYPE_USB)
		return 1;
	else
		return 0;
}

static int s3c_bat_get_adc_data(adc_channel_type adc_ch)
{
	int adc_arr[ADC_DATA_ARR_SIZE];
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;

	//pr_info("[BAT]:%s\n", __func__);

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) {
		adc_arr[i] = s3c_adc_get_adc_data(adc_ch);
		//      pr_info("[BAT]:%s: adc_arr = %d\n", __func__, adc_arr[i]);
		if (i != 0) {
			if (adc_arr[i] > adc_max) {
				adc_max = adc_arr[i];
			} else if (adc_arr[i] < adc_min) {
				adc_min = adc_arr[i];
			}
		} else {
			adc_max = adc_arr[0];
			adc_min = adc_arr[0];
		}
		adc_total += adc_arr[i];
	}

	//  pr_info("[BAT]:%s: adc_max = %d, adc_min = %d\n",   __func__, adc_max, adc_min);
	//      printk("adc_max = %d, adc_min = %d, adc_total = %d", adc_max, adc_min, adc_total);
	return (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
}

#ifndef __FUEL_GAUGES_IC__
static unsigned long calculate_average_adc(adc_channel_type channel, int adc)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = adc_sample[channel].cnt;
	total_adc = adc_sample[channel].total_adc;

	if (adc < 0 || adc == 0) {
		//dev_err(dev, "%s: invalid adc : %d\n", __func__, adc);
		adc = adc_sample[channel].average_adc;
	}

	if (cnt < ADC_TOTAL_COUNT) {
		adc_sample[channel].adc_arr[cnt] = adc;
		adc_sample[channel].index = cnt;
		adc_sample[channel].cnt = ++cnt;

		total_adc += adc;
		average_adc = total_adc / cnt;
	} else {
		index = adc_sample[channel].index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = (total_adc - adc_sample[channel].adc_arr[index]) + adc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		adc_sample[channel].adc_arr[index] = adc;
		adc_sample[channel].index = index;
	}

	adc_sample[channel].total_adc = total_adc;
	adc_sample[channel].average_adc = average_adc;

	//pr_info("[BAT]:%s: ch:%d adc=%d, avg_adc=%d\n", __func__, channel, adc, average_adc);

	return average_adc;
}
#endif

static unsigned long s3c_read_temp(void)
{
	int adc = 0;

	//pr_info("[BAT]:%s\n", __func__);

	adc = s3c_bat_get_adc_data(S3C_ADC_TEMPERATURE);

	s3c_bat_info.bat_info.batt_temp_adc = adc;

	return adc;
}

static int is_over_abs_time(void)
{
	unsigned int total_time = 0;

	//pr_info("[BAT]:%s\n", __func__);

	if (!start_time_msec) {
		return 0;
	}

	if (s3c_bat_info.bat_info.batt_is_recharging) {
		total_time = TOTAL_RECHARGING_TIME;
	} else {
		total_time = TOTAL_CHARGING_TIME;
	}

	if (jiffies_to_msecs(jiffies) >= start_time_msec) {
		total_time_msec = jiffies_to_msecs(jiffies) - start_time_msec;
	} else {
		total_time_msec = jiffies_to_msecs(0xFFFFFFFF) - start_time_msec + jiffies_to_msecs(jiffies);
	}

	if (total_time_msec > total_time && start_time_msec) {
		//pr_info("[BAT]:%s:abs time is over.:start_time=%u, total_time_msec=%u\n", __func__, start_time_msec, total_time_msec);
		return 1;
	} else {
		return 0;
	}
}

#ifdef __CHECK_CHG_CURRENT__
static void check_chg_current(void)
{
	unsigned long chg_current_adc = 0;
	unsigned long chg_current_temp = 0;
	unsigned long chg_current_volt = 0;
	unsigned long chg_current = 0;

#ifdef __SET_TEST_VALUE__
#ifndef SUPPORT_DOUBLE_CHECK_FULL_CHG
	unsigned long top_off_current = 0;

	if (s3c_bat_info.charging_mode_booting == 1) {
		if (s3c_bat_info.bat_info.charging_source == CHARGER_USB)
			top_off_current = LPM_CURRENT_OF_FULL_CHG_USB;
		else if (s3c_bat_info.bat_info.charging_source == CHARGER_AC)
			top_off_current = LPM_CURRENT_OF_FULL_CHG_TA;
		else
			top_off_current = LPM_CURRENT_OF_FULL_CHG;
	} else {
		if (s3c_bat_info.bat_info.charging_source == CHARGER_USB)
			top_off_current = CURRENT_OF_FULL_CHG_USB;
		else if (s3c_bat_info.bat_info.charging_source == CHARGER_AC)
			top_off_current = CURRENT_OF_FULL_CHG_TA;
		else
			top_off_current = CURRENT_OF_FULL_CHG;
	}

#endif
#endif

	chg_current_adc = s3c_bat_get_adc_data(S3C_ADC_CHG_CURRENT);	/* get adc code value */
	s3c_bat_info.bat_info.batt_current_adc = chg_current_adc;

	chg_current_temp = chg_current_adc * ADC_12BIT_RESOLUTION;
	chg_current_volt = chg_current_temp / ADC_12BIT_SCALE;
	if ((chg_current_temp % ADC_12BIT_SCALE) >= (ADC_12BIT_SCALE / 2)) {
		chg_current_volt += 1;
	}
	chg_current_temp = 0;
	chg_current_temp = (chg_current_volt * 100) / ADC_CURRENT_FACTOR;	/* scaled 10 */
	chg_current = chg_current_temp / 10;
	if ((chg_current_temp % 10) >= 5) {
		chg_current += 1;
	}

	s3c_bat_info.bat_info.batt_current = chg_current;

#if 0 /* for debug */
	pr_info("%s : chg_current = %d adc, %dmV, %dmA\n", __func__,
		chg_current_adc, chg_current_volt, chg_current);
#endif

#ifdef __SET_TEST_VALUE__
#ifndef SUPPORT_DOUBLE_CHECK_FULL_CHG

#if 0 //venturi taejin
	if (ibatt_test_value == 2) {
		chg_current_adc = CURRENT_OF_FULL_CHG - 1;
		full_check_count = 10;
	}
#if 1
	else if (ibatt_test_value == 3) {
		chg_current_adc = CURRENT_OF_FULL_CHG + 1;
		full_check_count = 0;
	}
#endif

#else //venturi using this

	//mutex_lock(&work_lock);
	if (ibatt_test_value == 2) {
		chg_current_adc = top_off_current - 1;
		full_check_count = 10;
	}
	else if (ibatt_test_value == 3) {
		chg_current_adc = top_off_current + 1;
		full_check_count = 0;
	}
	//mutex_unlock(&work_lock);

#endif //venturi taejin

#endif /* SUPPORT_DOUBLE_CHECK_FULL_CHG */
#endif /* __SET_TEST_VALUE__ */


#ifdef SUPPORT_DOUBLE_CHECK_FULL_CHG
	if (s3c_bat_info.bat_info.batt_vol >= FULL_CHARGE_COND_VOLTAGE) {

		if (chg_current_adc <= CURRENT_OF_FULL_CHG_2ND) {
			full_check_count++;
			if (full_check_count >= FULL_CHG_COND_COUNT) {
				s3c_bat_info.bat_info.batt_is_full = 1;
				full_charge_flag = 0xFF;
				full_check_count = 0;
			}
		}
		else if (!full_charge_flag && chg_current_adc <= CURRENT_OF_FULL_CHG_1ST) {
			full_check_count++;
			if (full_check_count >= FULL_CHG_COND_COUNT) {
				//dev_info(dev, "%s: battery full\n", __func__);
				//s3c_set_chg_en(0);
				s3c_bat_info.bat_info.batt_is_full = 1;
				full_charge_flag = 0xF;
				//full_charge_count = 0;
				//if (!s3c_bat_info.bat_info.batt_is_recharging) {
				//  force_update = 1;
				//}
				full_check_count = 0;
			}
		}
  		else {
			full_check_count = 0;
		}
	} else {
		full_check_count = 0;
	}
	battery_debug("[BAT]:%s :  current_adc: %lu, count: %d\n", __func__, chg_current_adc, full_check_count);
#else
	if (s3c_bat_info.bat_info.batt_vol >= FULL_CHARGE_COND_VOLTAGE) {
		if (chg_current_adc <= CURRENT_OF_FULL_CHG) {
			full_check_count++;
			if (full_check_count >= FULL_CHG_COND_COUNT) {
				//dev_info(dev, "%s: battery full\n", __func__);
				//s3c_set_chg_en(0);
				s3c_bat_info.bat_info.batt_is_full = 1;
				full_charge_flag = 1;
				//full_charge_count = 0;
				//if (!s3c_bat_info.bat_info.batt_is_recharging) {
				//  force_update = 1;
				//}
				full_check_count = 0;
			}
		}
	} else {
		full_check_count = 0;
	}
#endif
}

static void s3c_check_chg_current(void)
{
	if (s3c_bat_info.bat_info.charging_enabled) {
		check_chg_current();
	} else {
		full_check_count = 0;
		s3c_bat_info.bat_info.batt_current = 0;
		s3c_bat_info.bat_info.batt_current_adc = 0;
		s3c_bat_info.bat_info.batt_current_adc_aver = 0;
	}
}
#endif /* __CHECK_CHG_CURRENT__ */

static u32 s3c_get_bat_health(void)
{
	//pr_info("[BAT]:%s\n", __func__);

	return s3c_bat_info.bat_info.batt_health;
}

static void s3c_set_bat_health(u32 batt_health)
{
	//pr_info("[BAT]:%s\n", __func__);

	s3c_bat_info.bat_info.batt_health = batt_health;
}

static void s3c_set_time_for_charging(int mode)
{

	//pr_info("[BAT]:%s\n", __func__);

	if (mode) {
		/* record start time for abs timer */
		start_time_msec = jiffies_to_msecs(jiffies);
		end_time_msec = 0;
		//pr_info("[BAT]:%s: start_time(%u)\n", __func__, start_time_msec);
	} else {
		/* initialize start time for abs timer */
		start_time_msec = 0;
		total_time_msec = 0;
		end_time_msec = jiffies_to_msecs(jiffies);
		//pr_info("[BAT]:%s: start_time_msec(%u)\n", __func__, start_time_msec);
	}
}

static void s3c_set_chg_en(int enable)
{
	static int charging_state = -1;
	int chg_en_val;

	//pr_info("[BAT]:%s\n", __func__);

	if (charging_state != enable) {
		chg_en_val = s3c_bat_info.cable_status;

		if (enable && chg_en_val) {

			if (s3c_bat_info.cable_status == CABLE_TYPE_AC) {
				maxim_charging_control(CABLE_TYPE_AC, TRUE);
				s3c_set_time_for_charging(1);
				charging_state = 1;
#ifdef __BATTERY_COMPENSATION__
				s3c_bat_set_compesation(1, OFFSET_TA_ATTACHED, COMPENSATE_TA);
#endif /* __BATTERY_COMPENSATION__ */
			} else if (s3c_bat_info.cable_status == CABLE_TYPE_USB) {
				maxim_charging_control(CABLE_TYPE_USB, TRUE);
				s3c_set_time_for_charging(1);
				charging_state = 1;
#ifdef __BATTERY_COMPENSATION__
				s3c_bat_set_compesation(1, OFFSET_USB_ATTACHED, COMPENSATE_USB);
#endif /* __BATTERY_COMPENSATION__ */
			} else {
				maxim_charging_control(CABLE_TYPE_NONE, FALSE);
				s3c_set_time_for_charging(0);
				//s3c_bat_info.bat_info.batt_is_recharging = 0; /* s1-eur feature : it always makes re-charging full popup...*/
				charging_state = 0;
				//pr_info("[BAT]:%s:unknown charger!!\n", __func__);
#ifdef __BATTERY_COMPENSATION__
				if (s3c_bat_info.device_state & OFFSET_TA_ATTACHED)
					s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED, COMPENSATE_TA);
				else if (s3c_bat_info.device_state & OFFSET_USB_ATTACHED)
					s3c_bat_set_compesation(0, OFFSET_USB_ATTACHED, COMPENSATE_USB);
#endif /* __BATTERY_COMPENSATION__ */
			}
		} else {
			maxim_charging_control(CABLE_TYPE_NONE, FALSE);
			s3c_set_time_for_charging(0);
			//s3c_bat_info.bat_info.batt_is_recharging = 0; /* s1-eur feature : it always makes re-charging full popup...*/
			charging_state = 0;
#ifdef __BATTERY_COMPENSATION__
				if (s3c_bat_info.device_state & OFFSET_TA_ATTACHED)
					s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED, COMPENSATE_TA);
				else if (s3c_bat_info.device_state & OFFSET_USB_ATTACHED)
					s3c_bat_set_compesation(0, OFFSET_USB_ATTACHED, COMPENSATE_USB);
#endif /* __BATTERY_COMPENSATION__ */
		}
		s3c_bat_info.bat_info.charging_enabled = charging_state;

		//pr_info("[BAT]:%s: charging_state = %d\n", __func__, charging_state);

	}

}

static void s3c_temp_control(int mode)
{
	//pr_info("[BAT]:%s\n", __func__);

#ifdef __POPUP_DISABLE_MODE__
	if (ibatt_test_value == 999) {
		pr_debug("%s : ibatt_test_value = %d \n", __func__, ibatt_test_value);
		//mode = POWER_SUPPLY_HEALTH_GOOD;
		if (mode != POWER_SUPPLY_HEALTH_GOOD) {
			return;
		}
	}
#endif /* __POPUP_DISABLE_MODE__ */

#ifdef __TEMP_BLOCK_ECXEPT__
	//if((except_temp_count > 0) || (except_temp_block != 0))
	if ((iboot_completed == 0) || (except_temp_block != 0))
	//if(except_temp_block != 0)
	{
		//pr_info("%s : current except_temp_block = %d (%d)\n", __func__, except_temp_block, except_temp_count);
		//pr_info("%s : current except_temp_block = %d \n", __func__, except_temp_block);
		//mode = POWER_SUPPLY_HEALTH_UNKNOWN; /* for skip below routine... */
		if (mode != POWER_SUPPLY_HEALTH_GOOD) {
			return;
		}
	}
#endif /* __TEMP_BLOCK_ECXEPT__ */

	switch (mode) {
	case POWER_SUPPLY_HEALTH_GOOD:
		//pr_info("[BAT]:%s: GOOD\n", __func__);
		s3c_set_bat_health(mode);
		break;
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		//pr_info("[BAT]:%s: OVERHEAT\n", __func__);
		s3c_set_bat_health(mode);
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		//pr_info("[BAT]:%s: COLD\n", __func__);
		s3c_set_bat_health(mode);
		break;
	default:
		break;
	}

	//s3c_cable_check_status(); /* do not check here */

}

static void s3c_cable_check_status(void)
{

	battery_debug("[BAT]:%s\n", __func__);
	if (s3c_bat_info.cable_status) {
		if (s3c_get_bat_health() != POWER_SUPPLY_HEALTH_GOOD) {
			battery_debug("[BAT]:%s:Unhealth battery state!\n", __func__);
			cable_status = CHARGER_DISCHARGE;
			s3c_bat_info.bat_info.batt_is_recharging = 0;	/* s1-kor feature */
		} else if (get_usb_power_state()) {
			cable_status = CHARGER_USB;
			battery_debug("[BAT]:%s: status : USB\n", __func__);
		} else {
			cable_status = CHARGER_AC;
			battery_debug("[BAT]:%s: status : AC\n", __func__);
		}

	} else {
		if(charging_mode_get())
		{
			printk("\nPower off cause charger removed in LPM mode\n");
			if (pm_power_off)
				pm_power_off();
		}	

		cable_status = CHARGER_BATTERY;
		s3c_bat_info.bat_info.batt_is_recharging = 0;	/* s1-kor feature */
		if (s3c_get_bat_health() != POWER_SUPPLY_HEALTH_GOOD) {	/* s1-kor feature */
			s3c_set_bat_health(POWER_SUPPLY_HEALTH_GOOD);
		}
		battery_debug("[BAT]:%s: status : BATTERY\n", __func__);
	}

}

#if 1 /* eur-feature , MP3-feature */
static unsigned int s3c_bat_check_v_f(void)
{
	unsigned int ret = 0;

	if (maxim_vf_status() ||
		((s3c_bat_info.pdata->jig_cb)?s3c_bat_info.pdata->jig_cb():0))	// bat detected
	{
		ret = 1;
	} else {
		//dev_err(dev, "%s: VF error!\n", __func__);
		s3c_set_bat_health(POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		ret = 0;
	}

	battery_debug("[BAT]:%s ret: %d\n", __func__, ret);

	return ret;
}
#endif /* eur-feature */

extern int set_tsp_for_ta_detect(int state);
void s3c_cable_changed(bool connected)
{
	pr_info("[BAT]:%s(%d,%d,%d)\n", __func__, connected
		,s3c_bat_info.cable_status, s3c_bat_info.pdata->cable_cb());
	
	if (connected) {
		if(s3c_bat_info.pdata->cable_cb() == CABLE_TYPE_AC)
			if(!charging_mode_get())
				set_tsp_for_ta_detect(1);
	} else {
		if(charging_mode_get())
		{
			printk("\nPower off cause charger removed in LPM mode\n");
			if (pm_power_off)
				pm_power_off();
		}	

		if(s3c_bat_info.cable_status == CABLE_TYPE_AC)
			if(!charging_mode_get())
				set_tsp_for_ta_detect(0);
	}

	s3c_bat_info.cable_status = s3c_bat_info.pdata->cable_cb();
	
	if (s3c_bat_info.cable_status == CABLE_TYPE_MISC) {
		if (connected)
			s3c_bat_info.cable_status = CABLE_TYPE_USB;
		else
			s3c_bat_info.cable_status = CABLE_TYPE_NONE;
	}

	if (!s3c_battery_initial) {
		return;
	}
#if 1 /* eur-feature, MP3-feature */
	if (s3c_bat_check_v_f() == 0) {
		if (pm_power_off)
			pm_power_off();
	}
#endif /* eur-feature, MP3-feature */

	s3c_bat_info.bat_info.batt_is_full = 0;
	full_charge_flag = 0;
	//batt_chg_full_1st=0;
	force_log = 1;
	s3c_cable_check_status();

	schedule_work(&bat_work);

	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}

extern byte Get_MAX8998_PM_REG(max8998_pm_section_type reg_num);

void s3c_set_cable_cb(struct max8998_charger_callbacks *ptr, enum cable_type_t status)
{
	printk("s3c_set_cable_cb(%d, %d)\n", status, Get_MAX8998_PM_REG(VDCINOK_status));
	if((Get_MAX8998_PM_REG(VDCINOK_status) && (status != CABLE_TYPE_NONE)) ||
		(!Get_MAX8998_PM_REG(VDCINOK_status) && (status == CABLE_TYPE_NONE)))
		s3c_cable_changed(status != CABLE_TYPE_NONE);
}


#if 0 /* eur-feature */
void s3c_cable_charging(void)
{

	//pr_info("[BAT]:%s\n", __func__);

	if (!s3c_battery_initial) {
		return;
	}

	if (s3c_bat_info.bat_info.charging_enabled
	    && s3c_get_bat_health() == POWER_SUPPLY_HEALTH_GOOD) {
		if (batt_chg_full_1st == 1) {
			//pr_info("[BAT]:%s:2nd fully charged interrupt.\n",__func__);
			full_charge_flag = 1;
			force_update = 1;
		} else {
			//pr_info("[BAT]:%s:1st fully charged interrupt.\n",__func__);
			s3c_bat_info.bat_info.batt_is_full = 1;
			batt_chg_full_1st = 1;
			force_update = 1;
			maxim_topoff_change();
		}
	}

	schedule_work(&bat_work);
	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}
#endif /* eur-feature */

static int s3c_cable_status_update(void)
{
	static charger_type_t old_cable_status = CHARGER_BATTERY;

	battery_debug("[BAT]:%s\n", __func__);

	if (!s3c_battery_initial) {
		return -EPERM;
	}

	if (old_cable_status != cable_status) {
		old_cable_status = cable_status;
		s3c_bat_info.bat_info.charging_source = cable_status;

		/* if the power source changes, all power supplies may change state */
		power_supply_changed(&s3c_power_supplies[CHARGER_BATTERY]);

		//pr_info("[BAT]:%s: call power_supply_changed\n", __func__);
	}
	if ((s3c_bat_info.cable_status == CABLE_TYPE_NONE && vbus_wake_lock_status == 1)
	    || (s3c_bat_info.cable_status != CABLE_TYPE_NONE && vbus_wake_lock_status == 0)) {
		if (s3c_bat_info.cable_status != CABLE_TYPE_NONE) {
			vbus_wake_lock_status = 1;
			wake_lock(&vbus_wake_lock);
			//pr_info("[BAT]:%s:suspend wake lock.\n", __func__);
		} else {
			/* give userspace some time to see the uevent and update
			 * LED state or whatnot...
			 */
			if (!s3c_bat_info.cable_status) {
				vbus_wake_lock_status = 0;
				wake_lock_timeout(&vbus_wake_lock, 5 * HZ);
				//pr_info("[BAT]:%s:suspend wake unlock.\n", __func__);
			}
		}
	}

	return 0;

}

static int s3c_get_bat_temp(void)
{
#if (CONFIG_VENTURI_BOARD_REV < CONFIG_VENTURI_REV03)
	s3c_bat_info.bat_info.batt_temp = 270;
	return 270;
#endif

	int temp = 0;
	//int temp_adc_aver=0;
	int temp_adc = s3c_read_temp();
	int health = s3c_get_bat_health();
#ifdef __BATTERY_COMPENSATION__
	//unsigned int ex_case = 0;
#endif /* __BATTERY_COMPENSATION__ */
	unsigned int event_case = 0;
	int temp_high_block, temp_high_recover;
	int mvolt, r;

#ifdef __SET_TEST_VALUE__
	static int test_on_off = 0;
#endif /* __SET_TEST_VALUE__ */

	battery_debug("[BAT]:%s\n", __func__);
//	temp_adc_aver = calculate_average_adc(S3C_ADC_TEMPERATURE, temp_adc); 

#ifdef __TEST_DEVICE_DRIVER__
	/* switch (bat_temper_state) {
	case 0:
		break;
	case 1:
		temp_adc = TEMP_HIGH_BLOCK;
		break;
	case 2:
		temp_adc = TEMP_LOW_BLOCK;
		break;
	default:
		break;
	}*/
#endif /* __TEST_DEVICE_DRIVER__ */

	//s3c_bat_info.bat_info.batt_temp_adc_aver=temp_adc_aver; /* eur-feature */

#ifdef __SET_TEST_VALUE__
	if (ibatt_test_value == 1) {
		if (test_on_off == 0) {
			temp_adc = TEMP_HIGH_BLOCK - 20;
			//temp_adc = TEMP_LOW_BLOCK + 20;
			test_on_off = 1;
		} else {
			temp_adc = TEMP_HIGH_BLOCK - 40;
			//temp_adc = TEMP_LOW_BLOCK + 40;
			test_on_off = 0;
		}

		if (s3c_bat_info.bat_info.charging_source == CHARGER_BATTERY) {
			temp_adc = TEMP_HIGH_RECOVER - 40;
		}
	}
#endif /* __SET_TEST_VALUE__ */

#ifdef __BATTERY_COMPENSATION__
	/*ex_case = OFFSET_MP3_PLAY 	| OFFSET_VOICE_CALL_2G 
		| OFFSET_VOICE_CALL_3G	| OFFSET_DATA_CALL 
		| OFFSET_VIDEO_PLAY;
	if (s3c_bat_info.device_state & ex_case) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT
		    || health == POWER_SUPPLY_HEALTH_COLD) {
			s3c_temp_control(POWER_SUPPLY_HEALTH_GOOD);
			//pr_info("[BAT]:%s : temp exception case. : device_state=%d \n", __func__, s3c_bat_info.device_state);
		}
		goto __map_temperature__;
	}*/
#endif /* __BATTERY_COMPENSATION__ */

#ifdef SUPPORT_EVENT_TEMP_BLOCK
	// Event CHG Block
	event_case = OFFSET_MP3_PLAY | OFFSET_VOICE_CALL_2G | OFFSET_VOICE_CALL_3G	| OFFSET_VIDEO_PLAY;

	if (s3c_bat_info.device_state & event_case)
	{
		temp_high_block   = TEMP_EVENT_HIGH_BLOCK;
		temp_high_recover = TEMP_EVENT_HIGH_RECOVER;
	}
	else
#endif
	{
		temp_high_block   = TEMP_HIGH_BLOCK;
		temp_high_recover = TEMP_HIGH_RECOVER;
	}

	if (temp_adc <= temp_high_block)
	{
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT
		    && health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) 
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_OVERHEAT);
		}
	} 
	else if (temp_adc >= temp_high_recover && temp_adc <= TEMP_LOW_RECOVER) 
	{
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT || health == POWER_SUPPLY_HEALTH_COLD)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_GOOD);
		}
	} 
	else if (temp_adc >= TEMP_LOW_BLOCK) 
	{
		if (health != POWER_SUPPLY_HEALTH_COLD && health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) 
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_COLD);
		}
	}

#ifdef __BATTERY_COMPENSATION__
	//__map_temperature__:
#endif

#if 0

	int i = 0;
	int array_size = 0;
	array_size = ARRAY_SIZE(temper_table);

	for (int i = 0; i < (array_size - 1); i++) {
		if (i == 0) {
			if (temp_adc >= temper_table[0][0]) {
				temp = temper_table[0][1];
				break;
			} else if (temp_adc <= temper_table[array_size - 1][0]) {
				temp = temper_table[array_size - 1][1];
				break;
			}
		}

		if (temper_table[i][0] > temp_adc
		    && temper_table[i + 1][0] <= temp_adc) {
			temp = temper_table[i + 1][1];
		}
	}
#else

	// 0~40mV, 3.26~3.3V => ADC 0
	// 0.04V< x <3.26V, x = adc value
	// 4096/(3.26-0.04) = 1272.25 adc/1V, 1.272 adc/1mV
	mvolt = (1000*temp_adc) / 1272 + 80;

	//  - VDD: 3300mV
	// Rt = ( 100K * Vadc  ) / ( VDD - 2 * Vadc )
	if( mvolt >= 1650 )	{
		printk("[BAT]:%s: TEMP. Vadc is wrong value (%d)!!\n", __func__, mvolt);
		return s3c_bat_info.bat_info.batt_temp;
	}

	r = ( 100000 * mvolt ) / ( 3300 - 2 * mvolt );

	/*calculating temperature*/
	for( temp = 100; temp >= 0; temp-- ) {
		if( ( NTH5G10P33B103E08TH_resistance_table[temp] - r ) >= 0 )
			break;
	}
	temp -= 15;
	temp *= 10;
#endif

	battery_debug("[BAT]:%s: temp = %d, temp_adc_aver = %d\n", __func__, temp, temp_adc);
	//pr_info("[BAT]:%s: temp = %d, temp_adc = %d, temp_adc_aver = %d, batt_health=%d\n",   __func__, temp, temp_adc, temp_adc_aver, s3c_bat_info.bat_info.batt_health);

	s3c_bat_info.bat_info.batt_temp = temp;

	return temp;
}

#ifdef __FUEL_GAUGES_IC__
#ifdef __CHECK_BATT_VOLTAGE__
static void check_batt_voltage(void)
{
	unsigned long batt_voltage_mon = 0;

	batt_voltage_mon = s3c_bat_get_adc_data(S3C_ADC_BATT_MON);
	s3c_bat_info.bat_info.batt_vol_adc = batt_voltage_mon;
#if 0 /* for debug */
	pr_info("%s : batt_vol_adc = %dmV (%d)\n", __func__, batt_voltage_mon, s3c_bat_info.bat_info.batt_vol_adc);
#endif  
}
#endif /* __CHECK_BATT_VOLTAGE__ */

static int s3c_get_bat_vol(void)
{
	int fg_vcell = -1;
	static unsigned int ulow_vmon = 2460;	/* BATT_MON meas. range is 1V to 2.46V */

	//pr_info("[BAT]:%s\n", __func__);

#ifdef __CHECK_BATT_VOLTAGE__
	/* before using, turn on the Battery Monitor in MAX8998_function.c */
	check_batt_voltage();
#endif

	if ((fg_vcell = fg_read_vcell()) < 0) {
		//dev_err(dev, "%s: Can't read vcell!!!\n", __func__);
		pr_err("%s: Can't read vcell!!!\n", __func__);
		if (s3c_bat_info.bat_info.batt_vol_adc < ulow_vmon) {
			ulow_vmon = s3c_bat_info.bat_info.batt_vol_adc;
		}
		pr_err("%s: vbatt_monitor (1644-3.3V, 1375-3.1V)  = %d, %d\n",
		       __func__, ulow_vmon, s3c_bat_info.bat_info.batt_vol_adc);
		fg_vcell = s3c_bat_info.bat_info.batt_vol;
	} else {
		s3c_bat_info.bat_info.batt_vol = fg_vcell;
	}

	//  pr_info("[BAT]:%s: fg_vcell = %d\n", __func__, fg_vcell);

	return fg_vcell;
}

static int s3c_get_bat_level(void)
{
	int fg_soc = -1;
	static int err_cnt = 0;

	//pr_info("[BAT]:%s\n", __func__);

	if ((fg_soc = fg_read_soc()) < 0) {
		//dev_err(dev, "%s: Can't read soc!!!\n", __func__);
		pr_err("%s: Can't read soc!!!\n", __func__);
		if (fg_i2c_client == NULL) {
			pr_err("%s: fg_i2c_client is null !!\n", __func__);
			err_cnt++;
			if (err_cnt > 10) {
				pr_err("%s: fg_i2c_client is null, set batt level to zero, it will cause power off as low battery!!!\n", __func__);
				s3c_bat_info.bat_info.level = 0;
				// for power off 
				//s3c_set_bat_health(POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
				err_cnt = 0;
			}
		}
		fg_soc = s3c_bat_info.bat_info.level;
	} else {
		err_cnt = 0;
	}

	new_temp_level = fg_soc;

	//  pr_info("[BAT]:%s: fg_soc = %d \n", __func__, fg_soc);

	return fg_soc;
}
#else // __FUEL_GAUGES_IC__

static int s3c_read_vol_adc_for_cal(void)
{
	int adc = 0;

	adc = s3c_bat_get_adc_data(S3C_ADC_BATT_MON);
	adc += s3c_bat_info.bat_info.batt_vol_adc_cal;

	return adc;
}

static int s3c_read_vol_for_cal(void)
{
	int adc = 0;
	int bat_mon_vol = 0;
	int bat_vol = 0;

	adc = s3c_read_vol_adc_for_cal();

	// (2^12-1)adc/3.3v = 1240.9adc/v = 1240900adc/mv
	// bat_vol is from MAX8998 data sheet
	bat_mon_vol = (1000*adc)/1241;
	bat_vol = (1000 * (bat_mon_vol + 896) / 1083) + 1250;

	if (bat_vol >= 4200)
	{
		bat_vol = 4200;
	}

	return bat_vol;
}

static unsigned long s3c_read_bat()
{
	int adc = 0;
	static int cnt = 0;
	static int boot_cnt = 60000 / POLLING_INTERVAL;	/* for 60s  adc+ by 0.03v */

	dev_dbg(dev, "%s\n", __func__);

	adc = s3c_bat_get_adc_data(S3C_ADC_BATT_MON);
	dev_dbg(dev, "%s: adc = %d\n", __func__, adc);

	adc += s3c_bat_info.bat_info.batt_vol_adc_cal;

	/* +0.03v while booting */
	if (boot_cnt > 0)
	{
		adc += 45;	
		boot_cnt--;
	}

#ifdef __BATTERY_COMPENSATION__
	adc += batt_compensation;
	if (1 == charging_mode_get())	// jmin : for booting mode charging
	{
		adc += (-30);	// approx. -0.02v for both usb and ta charger
	}
#endif /* __BATTERY_COMPENSATION__ */
	if (adc < s3c_bat_info.bat_info.batt_vol_adc_aver - INVALID_VOL_ADC
	    && cnt < 10) {
		dev_err(dev, "%s: invaild adc = %d\n", __func__, adc);
		adc = s3c_bat_info.bat_info.batt_vol_adc_aver;
		cnt++;
	} else {
		cnt = 0;
	}
	s3c_bat_info.bat_info.batt_vol_adc = adc;
	return calculate_average_adc(S3C_ADC_BATT_MON, adc);
}

static int s3c_get_bat_vol(void)
{
	int bat_vol = 0;
	int bat_mon_vol = 0;
	int bat_adc = s3c_read_bat();

	if (bat_adc < 0) {
		printk("%s: Read battery ADC failed!!\n", __func__);
		return -1;
	}
	//s3c_bat_info.bat_info.batt_vol_adc = adc;

#if 0
	// 12 bit ADC, MAX 3.3V 
	bat_mon_vol = (3300 * (bat_adc)) / 4095;
	if (((3300 * bat_adc) % 4095) >= 2048)
		bat_mon_vol++;

	// MAX8987 PMIC
	// VBATTMONITOR(V) = 1.083 * ( VBATT - 1.25V ) - 0.896
	// VBATT(V)  = ( VBATTMONITOR(V) + 0.896 ) / 1.083 + 1.25V
	// VBATT(mV) = ( VBATTMONITOR(mV) + 896 ) / 1.083 + 1250mV
	// VBATT(mV) = ( 1000 * ( VBATTMONITOR(mV) + 896 ) / 1083 ) + 1250
	bat_vol = (1000 * (bat_mon_vol + 896) / 1083) + 1250;
	if (bat_vol >= 4200)
		bat_vol = 4200;
#else
	// 0~40mV, 3.26~3.3V => ADC 0
	// 0.04V< x <3.26V, x = adc value
	// 4096/(3.26-0.04) = 1272.25 adc/1V, 1.272 adc/1mV
	bat_mon_vol = (1000*bat_adc)/1272 + 80;
	
	// MAX8987 PMIC
	// VBATTMONITOR(V) = 1.083 * ( VBATT - 1.25V ) - 0.896
	// VBATT(V)  = ( VBATTMONITOR(V) + 0.896 ) / 1.083 + 1.25V
	// VBATT(mV) = ( VBATTMONITOR(mV) + 896 ) / 1.083 + 1250mV
	// VBATT(mV) = ( 1000 * ( VBATTMONITOR(mV) + 896 ) / 1083 ) + 1250
	bat_vol = (1000 * (bat_mon_vol + 896) / 1083) + 1250;
	if (bat_vol >= 4200)
		bat_vol = 4200;
#endif


#ifdef DEBUG_BATTERY_VOLTAGE
	if (p_count++ % 5 == 0)
		printk("%s : adc = %d, batt_mon_vol = %d, batt_vol = %d\n",
		       __func__, bat_adc, bat_mon_vol, bat_vol);

	if (p_count > 5)
		p_count = 1;
#endif

	s3c_bat_info.bat_info.batt_vol_adc_aver = bat_adc;
	s3c_bat_info.bat_info.batt_vol = bat_vol;
	dev_dbg(dev, "%s: adc = %d, bat_vol = %d\n", __func__, bat_adc,
		bat_vol);
	return bat_vol;
}

#if 1
static int s3c_get_bat_level()
{
	int bat_level = 0;
	int bat_vol_adc = s3c_bat_info.bat_info.batt_vol_adc_aver;

	int i = 0;
	int array_size = 0;

	/* jmin : this is for GUMI line test for immediate power-off */
	if (batt_quick_off_flag == 1)
	{
		new_temp_level = bat_level;
		return 0;
	}

#ifdef __BATTERY_COMPENSATION__
	if (s3c_bat_info.bat_info.charging_enabled) {
		if (bat_vol_adc > batt_almost - COMPENSATE_TA) {
			s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED,
						COMPENSATE_TA);
		}
	}
#endif /* __BATTERY_COMPENSATION__ */


#if 0
	if (bat_vol_adc > batt_full) {
		int temp = (batt_max - batt_full);
		if (bat_vol_adc > (batt_full + temp)
		    || s3c_bat_info.bat_info.batt_is_full)
			bat_level = 100;
		else
			bat_level = 90;

		dev_dbg(dev, "%s: (full)level = %d\n", __func__, bat_level);
	} else if (batt_full >= bat_vol_adc && bat_vol_adc > batt_almost) {
		int temp = (batt_full - batt_almost) / 2;
// FIX ME               if (bat_vol_adc > (batt_almost + 86))
		if (bat_vol_adc > (batt_almost + temp))
			bat_level = 80;
		else
			bat_level = 70;

		dev_dbg(dev, "%s: (almost)level = %d\n", __func__, bat_level);
	} else if (batt_almost >= bat_vol_adc && bat_vol_adc > batt_high) {
		int temp = (batt_almost - batt_high) / 2;
// FIX ME               if (bat_vol_adc > (batt_high + 62))
		if (bat_vol_adc > (batt_high + temp))
			bat_level = 60;
		else
			bat_level = 50;
		dev_dbg(dev, "%s: (high)level = %d\n", __func__, bat_level);
	} else if (batt_high >= bat_vol_adc && bat_vol_adc > batt_medium) {
		int temp = (batt_high - batt_medium) / 2;
// FIX ME               if (bat_vol_adc > (batt_medium + 26))
		if (bat_vol_adc > (batt_medium + temp))
			bat_level = 40;
		else
			bat_level = 30;
		dev_dbg(dev, "%s: (med)level = %d\n", __func__, bat_level);
	} else if (batt_medium >= bat_vol_adc && bat_vol_adc > batt_low) {
		bat_level = 15;
		dev_dbg(dev, "%s: (low)level = %d\n", __func__, bat_level);
	} else if (batt_low >= bat_vol_adc && bat_vol_adc > batt_critical) {
		bat_level = 5;
		dev_dbg(dev, "%s: (cri)level = %d, vol = %d\n", __func__,
			bat_level, bat_vol_adc);
	} else if (batt_critical >= bat_vol_adc && bat_vol_adc > batt_min) {
		bat_level = 3;
		dev_info(dev, "%s: (min)level = %d, vol = %d\n", __func__,
			 bat_level, bat_vol_adc);
	} else if (batt_min >= bat_vol_adc && bat_vol_adc > batt_off) {
		bat_level = 1;
		dev_info(dev, "%s: (off)level = %d, vol = %d\n", __func__,
			 bat_level, bat_vol_adc);
	} else if (batt_off >= bat_vol_adc) {
		bat_level = 0;
		dev_info(dev, "%s: (off)level = %d, vol = %d", __func__,
			 bat_level, bat_vol_adc);
	}
#if 0
	if ((++p_count % 150) == 0) {	// Print debug message every 5 minutes.
		p_count = 1;
		printk
		    ("[BATT] level(%d), is_full(%d), is_recharging(%d), charging_enabled(%d), batt_vol(%d)\n",
		     bat_level, s3c_bat_info.bat_info.batt_is_full,
		     s3c_bat_info.bat_info.batt_is_recharging,
		     s3c_bat_info.bat_info.charging_enabled, bat_vol_adc);
	}
#endif
#else
	array_size= ARRAY_SIZE(battery_table);
	
	for (i = 0; i < array_size; i++) {
		if (bat_vol_adc >= battery_table[i][2]) {
			bat_level = battery_table[i][0];
			break;
		}
	}
#endif
	dev_dbg(dev, "%s: bat_vol_adc = %d, level = %d, is_full = %d\n",
		__func__, bat_vol_adc, bat_level,
		s3c_bat_info.bat_info.batt_is_full);

	battery_debug("[BAT]:%s: bat_vol_adc = %d, level = %d, is_full = %d\n",
		__func__, bat_vol_adc, bat_level,
		s3c_bat_info.bat_info.batt_is_full);

#ifdef __TEMP_ADC_VALUE__
	return 80;
#else

	new_temp_level = bat_level;

	return bat_level;
#endif /* __TEMP_ADC_VALUE__ */
}

#else

static int s3c_get_bat_level()
{
	int bat_level = 0;
	int bat_vol = s3c_bat_info.bat_info.batt_vol_adc_aver;

#if 0				// like FG
	if (is_over_abs_time()) {
		bat_level = 100;
		dev_info(dev, "%s: charging time is over\n", __func__);
		s3c_set_chg_en(0);
		s3c_bat_info.bat_info.batt_is_full = 1;
		goto __end__;
	}
#endif

#if 0				// FIX ME
	if (!get_jig_cable_state() && low_batt_power_off)	// Low batt interrupt occured
	{
		bat_level = 0;	// Now, phone will be shutdown
		dev_info(dev, "%s: power off by low battery\n", __func__);
		goto __end__;
	}
#endif

#if 0				// FIX ME def __BATTERY_COMPENSATION__
	if (s3c_bat_info.bat_info.charging_enabled) {
		if (bat_vol > batt_almost - COMPENSATE_TA) {
			s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED,
						COMPENSATE_TA);
		}
	}
#endif /* __BATTERY_COMPENSATION__ */

	if (bat_vol > batt_full) {
		int temp = (batt_max - batt_full);
		if (bat_vol > (batt_full + temp)
		    || s3c_bat_info.bat_info.batt_is_full)
			bat_level = 100;
		else
			bat_level = 90;

#if 0				// like FG
#ifdef __CHECK_CHG_CURRENT__
		if (s3c_bat_info.bat_info.charging_enabled) {
			check_chg_current();
			if (!s3c_bat_info.bat_info.batt_is_full)
				bat_level = 90;

		}
#endif /* __CHECK_CHG_CURRENT__ */
#endif

		dev_dbg(dev, "%s: (full)level = %d\n", __func__, bat_level);
	} else if (batt_full >= bat_vol && bat_vol > batt_almost) {
		int temp = (batt_full - batt_almost) / 2;
// FIX ME               if (bat_vol > (batt_almost + 86))
		if (bat_vol > (batt_almost + temp))
			bat_level = 80;
		else
			bat_level = 70;

		dev_dbg(dev, "%s: (almost)level = %d\n", __func__, bat_level);
	} else if (batt_almost >= bat_vol && bat_vol > batt_high) {
		int temp = (batt_almost - batt_high) / 2;
// FIX ME               if (bat_vol > (batt_high + 62))
		if (bat_vol > (batt_high + temp))
			bat_level = 60;
		else
			bat_level = 50;
		dev_dbg(dev, "%s: (high)level = %d\n", __func__, bat_level);
	} else if (batt_high >= bat_vol && bat_vol > batt_medium) {
		int temp = (batt_high - batt_medium) / 2;
// FIX ME               if (bat_vol > (batt_medium + 26))
		if (bat_vol > (batt_medium + temp))
			bat_level = 40;
		else
			bat_level = 30;
		dev_dbg(dev, "%s: (med)level = %d\n", __func__, bat_level);
	} else if (batt_medium >= bat_vol && bat_vol > batt_low) {
		bat_level = 15;
		dev_dbg(dev, "%s: (low)level = %d\n", __func__, bat_level);
	} else if (batt_low >= bat_vol && bat_vol > batt_critical) {
		bat_level = 5;
		dev_dbg(dev, "%s: (cri)level = %d, vol = %d\n", __func__,
			bat_level, bat_vol);
	} else if (batt_critical >= bat_vol && bat_vol > batt_min) {
		bat_level = 3;
		dev_info(dev, "%s: (min)level = %d, vol = %d\n", __func__,
			 bat_level, bat_vol);
	} else if (batt_min >= bat_vol && bat_vol > batt_off) {
		bat_level = 1;
		dev_info(dev, "%s: (off)level = %d, vol = %d\n", __func__,
			 bat_level, bat_vol);
	} else if (batt_off >= bat_vol) {
		bat_level = 0;
		dev_info(dev, "%s: (off)level = %d, vol = %d", __func__,
			 bat_level, bat_vol);
	}
#if 0				// like FG
	// If current status is full or recharging, then it should be 100% regardless of current real battery level.
	if (s3c_bat_info.bat_info.batt_is_full
	    || s3c_bat_info.bat_info.batt_is_recharging)
		bat_level = 100;
#endif

#if 0
	if ((++p_count % 150) == 0) {	// Print debug message every 5 minutes.
		p_count = 1;
		printk
		    ("[BATT] level(%d), is_full(%d), is_recharging(%d), charging_enabled(%d), batt_vol(%d)\n",
		     bat_level, s3c_bat_info.bat_info.batt_is_full,
		     s3c_bat_info.bat_info.batt_is_recharging,
		     s3c_bat_info.bat_info.charging_enabled, bat_vol);
	}
#endif

#if 0				// like FG
	// If current status is full because of absolute timer, then it should be recharging.
	if (s3c_bat_info.bat_info.batt_is_full && !s3c_bat_info.bat_info.charging_enabled && bat_vol < (batt_max + 45)) {	// under 4.15V
		dev_info(dev, "%s: recharging(under full)\n", __func__);
		s3c_bat_info.bat_info.batt_is_recharging = 1;
		s3c_set_chg_en(1);
		bat_level = 100;
	}
#endif

	dev_dbg(dev, "%s: level = %d\n", __func__, bat_level);

__end__:
	dev_dbg(dev, "%s: bat_vol = %d, level = %d, is_full = %d\n",
		__func__, bat_vol, bat_level,
		s3c_bat_info.bat_info.batt_is_full);
#ifdef __TEMP_ADC_VALUE__
	return 80;
#else

	new_temp_level = bat_level;

	return bat_level;
#endif /* __TEMP_ADC_VALUE__ */
}

#endif

#endif //__FUEL_GAUGES_IC__ // venturi taejin

static int s3c_bat_need_recharging(void)
{
	unsigned int charging_end_total_time = 0;
	//static int cnt = 0;

	//pr_info("[BAT]:%s\n", __func__);

	if (end_time_msec) {
		if (jiffies_to_msecs(jiffies) >= end_time_msec) {
			charging_end_total_time = jiffies_to_msecs(jiffies) - end_time_msec;
		} else {
			charging_end_total_time = jiffies_to_msecs(0xFFFFFFFF) - end_time_msec + jiffies_to_msecs(jiffies);
		}
	}
	//  pr_info("[BAT]:%s:end_time_msec=%d, total_time=%d \n", __func__, end_time_msec, charging_end_total_time);

	//if(s3c_bat_info.bat_info.batt_is_full != 1 || s3c_bat_info.bat_info.batt_is_recharging == 1 || s3c_bat_info.bat_info.batt_vol > RECHARGE_COND_VOLTAGE
	if (s3c_bat_info.bat_info.batt_is_full != 1
#ifdef SUPPORT_DOUBLE_CHECK_FULL_CHG
		|| full_charge_flag != 0xFF
#endif
		|| s3c_bat_info.bat_info.batt_is_recharging == 1 
		|| s3c_bat_info.bat_info.batt_vol > RECHARGE_COND_VOLTAGE
		|| charging_end_total_time < RECHARGE_COND_TIME) {
		//pr_info("[BAT]:%s : not need recharge. \n", __func__);
		//cnt = 0;
		rechg_count = 0;
		return 0;
	} else {
		//pr_info("[BAT]:%s : need recharge. \n", __func__);
		//cnt++;
		rechg_count++;
		//if(cnt > 4)
		if (rechg_count > 4) {
			//cnt = 0;
			rechg_count = 0;
			return 1;
		} else if (s3c_bat_info.bat_info.batt_vol <= FULL_CHARGE_COND_VOLTAGE) {
			//cnt = 0;
			rechg_count = 0;
			return 1;
		} else {
			return 0;
		}
	}

}

static int s3c_bat_is_full_charged(void)
{
	//pr_info("[BAT]:%s\n", __func__);

	if(s3c_bat_info.bat_info.batt_is_full == 1 && 
#ifdef SUPPORT_DOUBLE_CHECK_FULL_CHG
		full_charge_flag == 0xFF)
#else
		full_charge_flag == 1)
#endif
	{
		//pr_info("[BAT]:%s : battery is fully charged\n", __func__);
		return 1;
	} else {
		//pr_info("[BAT]:%s : battery is not fully charged\n", __func__);
		return 0;
	}

}

static void s3c_bat_charging_control(void)
{
	//pr_info("[BAT]:%s\n", __func__);

	if (cable_status == CHARGER_BATTERY || cable_status == CHARGER_DISCHARGE) {
		s3c_set_chg_en(0);
		battery_debug("[BAT]:%s:no charging.\n", __func__);
	} else if (s3c_bat_need_recharging()) {
		battery_debug("[BAT]:%s:need recharging.\n", __func__);
		s3c_set_chg_en(1);
		s3c_bat_info.bat_info.batt_is_recharging = 1;
		full_charge_flag = 0;
	} else if (s3c_bat_is_full_charged()) {
		battery_debug("[BAT]:%s:battery full!!.\n", __func__);
		s3c_set_chg_en(0);
		s3c_bat_info.bat_info.batt_is_recharging = 0;
	} else if (is_over_abs_time()) {
		battery_debug("[BAT]:%s:battery full by timer!!.\n", __func__);
		s3c_set_chg_en(0);
		s3c_bat_info.bat_info.batt_is_full = 1;
		s3c_bat_info.bat_info.batt_is_recharging = 0;
#ifdef SUPPORT_DOUBLE_CHECK_FULL_CHG
		full_charge_flag = 0xFF;
#else
		full_charge_flag = 1;
#endif
	} else {
		battery_debug("[BAT]:%s:charging...\n", __func__);
		s3c_set_chg_en(1);
	}

}

static int batt_read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int len = 0;

//len = sprintf(buf, "%d, %u, %u, %d, %d, %d, %d, %u, %d, %u, %u, %d, %d, %d, %u, %u, %d, %d, %u\n",
	len = sprintf(buf, "%lu, %u, %d, %d, %d, %d, %u, %d, %u, %u, %d, %d, %d, %u, %u, %d, %d, %d\n",
		get_seconds(),
//            FGPureSOC,
		s3c_bat_info.bat_info.level,
		s3c_bat_info.bat_info.batt_vol,
		s3c_bat_info.bat_info.batt_current,
		s3c_bat_info.bat_info.batt_current_adc,
		//s3c_bat_info.bat_info.batt_current_adc_aver,
		//full_charge_count,
		rechg_count,
		s3c_bat_info.bat_info.batt_is_full,
		//top_off_interrupted,
		full_charge_flag,
		s3c_bat_info.bat_info.charging_enabled,
		s3c_bat_info.bat_info.batt_is_recharging,
		//s3c_bat_info.bat_info.batt_temp,
		full_check_count,
		s3c_bat_info.bat_info.batt_temp_adc,
		//s3c_bat_info.bat_info.batt_temp_adc_aver,
		s3c_bat_info.bat_info.batt_vol_adc,
		s3c_bat_info.bat_info.batt_health,
		s3c_bat_info.bat_info.charging_source,
		s3c_bat_info.bat_info.batt_v_f_adc,
		s3c_bat_info.cable_status,
		total_time_msec);

	return len;
}

static void s3c_bat_status_update(void)
{

	battery_debug("[BAT]:%s\n", __func__);

	if (s3c_bat_info.bat_info.batt_is_full || s3c_bat_info.bat_info.batt_is_recharging) {
		s3c_bat_info.bat_info.level = 100;
	} else if (!s3c_bat_info.bat_info.charging_enabled
		   && !s3c_bat_info.bat_info.batt_is_full && new_temp_level > old_level) {
		s3c_bat_info.bat_info.level = old_level;
	} else {
		s3c_bat_info.bat_info.level = new_temp_level;
	}

	if (old_level != s3c_bat_info.bat_info.level
	    || old_temp != s3c_bat_info.bat_info.batt_temp
	    || old_is_full != s3c_bat_info.bat_info.batt_is_full 
	    || force_update
	    || old_health != s3c_bat_info.bat_info.batt_health) {
		//pr_info("[BAT]:update\n");
		force_update = 0;
		power_supply_changed(&s3c_power_supplies[CHARGER_BATTERY]);

		if (old_level != s3c_bat_info.bat_info.level) {

#ifdef __FUEL_GAUGES_IC__
			//printk("[FG] P:%d, S1:%d, S2:%d, V:%d, T:%d\n", FGPureSOC, s3c_bat_info.bat_info.level, old_level, s3c_bat_info.bat_info.batt_vol, s3c_bat_info.bat_info.batt_temp);
#else
			//printk("[FG] S1:%d, S2:%d, V:%d, T:%d\n", s3c_bat_info.bat_info.level, old_level, s3c_bat_info.bat_info.batt_vol, s3c_bat_info.bat_info.batt_temp);
#endif
		}

	}

	if (old_level != s3c_bat_info.bat_info.level
	    || old_is_full != s3c_bat_info.bat_info.batt_is_full
	    || old_is_recharging != s3c_bat_info.bat_info.batt_is_recharging
	    || old_health != s3c_bat_info.bat_info.batt_health
	    || force_log == 1) {
		//      pr_info("[BAT]:Vol=%d, Temp=%d, SOC=%d, Lv=%d, ST=%u, TT=%u, CS=%d, CE=%d, RC=%d, FC=%d, Hlth=%d\n", 
		//          s3c_bat_info.bat_info.batt_vol, s3c_bat_info.bat_info.batt_temp, new_temp_level, s3c_bat_info.bat_info.level, start_time_msec, total_time_msec, 
		//          cable_status, s3c_bat_info.bat_info.charging_enabled, s3c_bat_info.bat_info.batt_is_recharging, s3c_bat_info.bat_info.batt_is_full, s3c_bat_info.bat_info.batt_health);
		force_log = 0;
	}
#if 0 /* for debug */
	pr_info("time=%d, psoc=%d%%, soc=%d%%, vcell=%dmV, cur=%d, curadc=%d, rechgcnt=%d, isfull=%d, fullflag=%d, ischgen=%d, isrechg=%d, fullcnt=%d, tempadc=%d, voladc=%d, health=%d, chgsrc=%d, cf=%d, devtype=%d, cable=%d, ttime=%d\n",
		get_seconds(),
		FGPureSOC,
		s3c_bat_info.bat_info.level,
		s3c_bat_info.bat_info.batt_vol,
		s3c_bat_info.bat_info.batt_current,
		s3c_bat_info.bat_info.batt_current_adc,
		//s3c_bat_info.bat_info.batt_current_adc_aver,
		//full_charge_count,
		rechg_count,
		s3c_bat_info.bat_info.batt_is_full,
		//top_off_interrupted,
		full_charge_flag,
		s3c_bat_info.bat_info.charging_enabled,
		s3c_bat_info.bat_info.batt_is_recharging,
		//s3c_bat_info.bat_info.batt_temp,
		full_check_count,
		s3c_bat_info.bat_info.batt_temp_adc,
		//s3c_bat_info.bat_info.batt_temp_adc_aver,
		s3c_bat_info.bat_info.batt_vol_adc,
		s3c_bat_info.bat_info.batt_health,
		s3c_bat_info.bat_info.charging_source,
		s3c_bat_info.bat_info.batt_v_f_adc,
		s3c_bat_info.cable_status,
		s3c_bat_info.cable_status,
		total_time_msec);
#endif
	entry->read_proc = batt_read_proc;

}


#if 0 // S1-Kor def __CHECK_BATTERY_V_F__
static void s3c_bat_check_v_f(void)
{
	static int cnt = 0;
	unsigned char v_f_state = BAT_DETECTED;
	u32 batt_health = POWER_SUPPLY_HEALTH_GOOD;
	//static u32 prev_batt_health = POWER_SUPPLY_HEALTH_GOOD;

	if (maxim_vf_status() == BAT_NOT_DETECTED) {
		cnt++;
		//pr_info("%s: vf status count = %d\n", __func__, cnt);
		if (cnt > 3) {
			v_f_state = BAT_NOT_DETECTED;
			//pr_info("%s: Unauthorized battery!... (%d)\n", __func__, cnt);
		}
	} else {
		cnt = 0;
		v_f_state = BAT_DETECTED;
	}

	if (s3c_bat_info.cable_status == CABLE_TYPE_NONE) {
		cnt = 0;
		v_f_state = BAT_DETECTED;
	}

	if (v_f_state == BAT_NOT_DETECTED) {
		//pr_info("%s: Unauthorized battery!\n", __func__);

		if (FSA9480_Get_I2C_JIG_Status2())
		//if (FSA9480_Get_JIG_Status())
		{
			batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		} else {
			batt_health = POWER_SUPPLY_HEALTH_DEAD;
		}

#ifdef __POPUP_DISABLE_MODE__
		if (ibatt_test_value == 999) {
			batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		}
#endif /* __POPUP_DISABLE_MODE__ */

		s3c_set_bat_health(batt_health);
		s3c_bat_info.bat_info.batt_v_f_adc = 0;
		s3c_bat_info.present = 0;
	} else {
		s3c_bat_info.bat_info.batt_v_f_adc = 1;
		s3c_bat_info.present = 1;
	}
}
#endif /* __CHECK_BATTERY_V_F__ */

static void polling_timer_func(unsigned long unused)
{
	//pr_info("[BAT]:%s\n", __func__);

	schedule_work(&bat_work);

	mod_timer(&polling_timer, jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}

static void s3c_store_bat_old_data(void)
{
	battery_debug("[BAT]:%s\n", __func__);

	old_temp = s3c_bat_info.bat_info.batt_temp;
	old_level = s3c_bat_info.bat_info.level;
	old_is_full = s3c_bat_info.bat_info.batt_is_full;
	old_is_recharging = s3c_bat_info.bat_info.batt_is_recharging;
	old_health = s3c_bat_info.bat_info.batt_health;

	battery_debug("[BAT]:%s : old_temp=%d, old_level=%d, old_is_full=%d\n", __func__, old_temp, old_level, old_is_full);
}

static void s3c_bat_work(struct work_struct *work)
{
	battery_debug("[BAT]:%s\n", __func__);

	if (!s3c_battery_initial) {
		return;
	}

	wake_lock(&update_wake_lock);
	mutex_lock(&work_lock);

	s3c_store_bat_old_data();
	s3c_get_bat_temp();
	s3c_bat_check_v_f();
	s3c_get_bat_vol();
	s3c_get_bat_level();
	s3c_cable_check_status();	/* it is helth related function, We have to update once atfer checking temperature, c_f and fg_i2c_read_error. */
#ifdef __CHECK_CHG_CURRENT__
	s3c_check_chg_current();
#endif /* __CHECK_CHG_CURRENT__ */
	s3c_bat_charging_control();
	s3c_cable_status_update();
	s3c_bat_status_update();

	mutex_unlock(&work_lock);
	wake_unlock(&update_wake_lock);
}

static int s3c_bat_create_attrs(struct device *dev)
{
	int i, rc;

	//pr_info("[BAT]:%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(s3c_battery_attrs); i++) {
		rc = device_create_file(dev, &s3c_battery_attrs[i]);
		if (rc) {
			goto s3c_attrs_failed;
		}
	}
	goto succeed;

s3c_attrs_failed:
	while (i--)
		device_remove_file(dev, &s3c_battery_attrs[i]);
succeed:
	return rc;
}

static void battery_early_suspend(struct early_suspend *h)
{
#if 0  // fix me
	u32 con;

	/*g3d clock disable */
	con = readl(S5P_CLKGATE_IP0);
	con &= ~S5P_CLKGATE_IP0_G3D;
	writel(con, S5P_CLKGATE_IP0);

	/*power gating */
	s5p_power_gating(S5PC110_POWER_DOMAIN_G3D, DOMAIN_LP_MODE);
	s5p_power_gating(S5PC110_POWER_DOMAIN_MFC, DOMAIN_LP_MODE);
	s5p_power_gating(S5PC110_POWER_DOMAIN_TV, DOMAIN_LP_MODE);
	s5p_power_gating(S5PC110_POWER_DOMAIN_CAM, DOMAIN_LP_MODE);
	s5p_power_gating(S5PC110_POWER_DOMAIN_AUDIO, DOMAIN_LP_MODE);
	/*
	   con = readl(S5P_NORMAL_CFG);
	   con &= ~(S5PC110_POWER_DOMAIN_G3D|S5PC110_POWER_DOMAIN_MFC|S5PC110_POWER_DOMAIN_TV \
	   |S5PC110_POWER_DOMAIN_CAM|S5PC110_POWER_DOMAIN_AUDIO);
	   writel(con , S5P_NORMAL_CFG);
	 */

	/*usb clock disable */
	//  s3c_usb_cable(0);
#endif  // fix me

	return;
}

static void battery_late_resume(struct early_suspend *h)
{

	return;
}

#ifdef __FUEL_GAUGES_IC__
#ifdef MAX17043
static irqreturn_t low_battery_isr(int irq, void *_di)
{
	pr_info("%s: low battery isr\n", __func__);
	cancel_delayed_work(&fuelgauge_work);
	schedule_delayed_work(&fuelgauge_work, 0);

	return IRQ_HANDLED;
}

int _low_battery_alarm_(void)
{
	int fg_soc_temp = -1;

	/* refresh current pure soc */
	mutex_lock(&work_lock);
	fg_soc_temp = fg_read_soc();
	mutex_unlock(&work_lock);

	/* defense code from invalid low interrrupt */
	/* you have to check with pure soc level, not adjusted. */
	printk("[FG-INT] P:%d, S:%d, V:%d, T:%d\n", FGPureSOC,
	       s3c_bat_info.bat_info.level, s3c_bat_info.bat_info.batt_vol,
	       s3c_bat_info.bat_info.batt_temp);

	if (FGPureSOC < 200) {	/* 200(==2%), means 1%(setting) + margin 1%, apply in case of 0x1f */
		s3c_bat_info.bat_info.level = 0;
	} else {
		pr_info("%s: unknown case, invalid low interrupt!\n", __func__);
	}

	wake_lock_timeout(&vbus_wake_lock, 5 * HZ);
	power_supply_changed(&s3c_power_supplies[CHARGER_BATTERY]);

	return 0;
}

static void fuelgauge_work_handler(struct work_struct *work)
{
	pr_info("%s: low battery alert!\n", __func__);
	_low_battery_alarm_();
}
#else
#error "check the fg model name!"
#endif /* MAX17043 */
#endif /* __FUEL_GAUGES_IC__ */

#ifdef __PSEUDO_BOOT_COMPLETED__
static void boot_complete_work_handler(struct work_struct *work)
{
	pr_info("%s: boot complete check!\n", __func__);

	if (iboot_completed == 0) {
		iboot_completed = 1;
	}
}
#endif /* __PSEUDO_BOOT_COMPLETED__ */

#if 0 /* eur-feature */
static ssize_t set_fuel_gauge_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int fg_soc = -1;

	if ((fg_soc = fg_read_soc()) < 0) {
		fg_soc = s3c_bat_info.bat_info.level;
	} else {
		s3c_bat_info.bat_info.level = fg_soc;
	}
	//printk("%s: soc is  %d!!!\n", __func__, fg_soc);
	return sprintf(buf, "%d\n", fg_soc);

}

static ssize_t set_fuel_gauge_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int ret_value = 0;
	int fg_soc = -1;

	ret_value = fg_reset_soc();

	if ((fg_soc = fg_read_soc()) < 0) {
		fg_soc = s3c_bat_info.bat_info.level;
	} else {
		s3c_bat_info.bat_info.level = fg_soc;
	}

	force_update = 1;
	force_log = 1;

	//printk("Enter set_fuel_gauge_reset_show by AT command return vlaue is %d \n", ret_value);
	ret_value = 0;

	return sprintf(buf, "%d\n", ret_value);

}

extern struct class *sec_class;
struct device *fg_atcom_test;
static DEVICE_ATTR(set_fuel_gauage_read, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_fuel_gauge_read_show, NULL);
static DEVICE_ATTR(set_fuel_gauage_reset, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_fuel_gauge_reset_show, NULL);
#endif /* eur-feature */

static int __devinit s3c_bat_probe(struct platform_device *pdev)
{
	struct sec_bat_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;
	int ret = 0;

	dev = &pdev->dev;
	//pr_info("[BAT]:%s\n", __func__);

	s3c_bat_info.pdata = pdata;
	if (!s3c_bat_info.pdata) {
		pr_err("%s : No platform data\n", __func__);
		return -EINVAL;
	}

	if (!s3c_bat_info.pdata->cable_cb) {
		pr_err("%s : Can't check cable status\n", __func__);
		return -EINVAL;
	}		

	s3c_bat_info.present = 1;
	s3c_bat_info.polling = 1;
	s3c_bat_info.polling_interval = POLLING_INTERVAL;
	s3c_bat_info.cable_status = s3c_bat_info.pdata->cable_cb();
#ifdef __BATTERY_COMPENSATION__
	s3c_bat_info.device_state = 0;
#endif
#ifdef __TEST_MODE_INTERFACE__
	s3c_bat_info.bat_info.batt_test_mode = 0;
	s3c_bat_info.bat_info.batt_vol_aver = 0;
	s3c_bat_info.bat_info.batt_temp_aver = 0;
	s3c_bat_info.bat_info.batt_temp_adc_aver = 0;
	s3c_power_supplies_test = s3c_power_supplies;
#endif /* __TEST_MODE_INTERFACE__ */
	s3c_bat_info.bat_info.batt_id = 0;
	s3c_bat_info.bat_info.batt_vol = 0;
	s3c_bat_info.bat_info.batt_vol_adc = 2460;	//for vbatt_mon, initial value is 2460
	s3c_bat_info.bat_info.batt_vol_adc_cal = 0;
	s3c_bat_info.bat_info.batt_temp = 0;
	s3c_bat_info.bat_info.batt_temp_adc = 0;
	s3c_bat_info.bat_info.batt_temp_adc_cal = 0;
	s3c_bat_info.bat_info.batt_current = 0;
	s3c_bat_info.bat_info.level = 100;	/*do not set zero, it will cause power off as low battery in case of discharging */
	s3c_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	s3c_bat_info.bat_info.charging_enabled = 0;
	s3c_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	s3c_bat_info.bat_info.batt_is_full = 0;
	s3c_bat_info.bat_info.batt_is_recharging = 0;
	s3c_bat_info.bat_info.batt_vol_adc_aver = 0;
	s3c_bat_info.bat_info.batt_v_f_adc = 0;
	s3c_bat_info.bat_info.batt_current_adc_aver = 0;

#ifdef LPM_MODE
	s3c_bat_info.charging_mode_booting = 0;
#endif

#ifndef __FUEL_GAUGES_IC__
	memset(adc_sample, 0x00, sizeof adc_sample);

	batt_max = BATT_CAL + BATT_MAXIMUM;
	batt_full = BATT_CAL + BATT_FULL;
	batt_safe_rech = BATT_CAL + BATT_SAFE_RECHARGE;
	batt_almost = BATT_CAL + BATT_ALMOST_FULL;
	batt_high = BATT_CAL + BATT_HIGH;
	batt_medium = BATT_CAL + BATT_MED;
	batt_low = BATT_CAL + BATT_LOW;
	batt_critical = BATT_CAL + BATT_CRITICAL;
	batt_min = BATT_CAL + BATT_MINIMUM;
	batt_off = BATT_CAL + BATT_OFF;
#endif

	INIT_WORK(&bat_work, s3c_bat_work);

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(s3c_power_supplies); i++) {
		ret = power_supply_register(&pdev->dev, &s3c_power_supplies[i]);
		if (ret) {
			//dev_err(dev, "Failed to register power supply %d,%d\n", i, ret);
			goto __end__;
		}
	}

	/* create sec detail attributes */
	s3c_bat_create_attrs(s3c_power_supplies[CHARGER_BATTERY].dev);

#ifdef __TEST_DEVICE_DRIVER__
	s3c_test_create_attrs(s3c_power_supplies[CHARGER_AC].dev);
	wake_lock_init(&wake_lock_for_dev, WAKE_LOCK_SUSPEND, "wake_lock_dev");
#endif /* __TEST_DEVICE_DRIVER__ */

	if (s3c_bat_info.polling) {
		setup_timer(&polling_timer, polling_timer_func, 0);
		mod_timer(&polling_timer, jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
	}

	s3c_battery_initial = 1;
	force_update = 0;
	force_log = 0;
	full_charge_flag = 0;
	//full_charge_count = 0;
	full_check_count = 0;
	rechg_count = 0;
#ifdef __BATTERY_COMPENSATION__
	batt_compensation = 0;
#endif /* __BATTERY_COMPENSATION__ */
	//top_off_interrupted = 0;
	ibatt_test_value = 0;
#ifdef __TEMP_BLOCK_ECXEPT__
	except_temp_block = 0;
#endif
	isVoiceCall = 0;
	/* Request IRQ */
	// must request before s3c_bat_status_update, so change location.
	MAX8998_IRQ_init();

#ifdef __FUEL_GAUGES_IC__
	// Set RCOMP value in the bootloader
	// Check the defense code in the low batt interrupt service routine.
	// Here, check again.
	mutex_lock(&work_lock);
	if ((fg_read_rcomp() & 0xff1f) != 0xc01f) {
		fuel_gauge_rcomp();
		fg_read_rcomp();
	}
	mutex_unlock(&work_lock);

#ifdef MAX17043
	dev_info(dev, "%s: low battery interrupt setting!\n", __func__);
	set_irq_type(IRQ_FUEL_INT_N, IRQ_TYPE_EDGE_FALLING);
	if (request_irq(IRQ_FUEL_INT_N, low_battery_isr, IRQF_DISABLED, "fg alert irq", NULL)) {
		pr_err("%s: Can NOT request irq %d, status %d\n", __func__, IRQ_FUEL_INT_N, ret);
	}

	INIT_DELAYED_WORK(&fuelgauge_work, fuelgauge_work_handler);
#ifdef __PSEUDO_BOOT_COMPLETED__
	INIT_DELAYED_WORK(&boot_complete_work, boot_complete_work_handler);
#endif /* __PSEUDO_BOOT_COMPLETED__ */
#else
#error "check the fg model name!"
#endif /* MAX17043 */
#endif /* __FUEL_GAUGES_IC__ */

#ifdef __CHECK_BATTERY_V_F__
	s3c_bat_check_v_f();
#endif /* __CHECK_BATTERY_V_F__ */

	s3c_cable_check_status();

	s3c_bat_work(&bat_work);

	/* Request IRQ */
	// must request before s3c_bat_status_update, so change location.
	//MAX8998_IRQ_init();

	if (charging_mode_get()) {
		battery = kzalloc(sizeof(struct battery_driver), GFP_KERNEL);
		battery->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
		battery->early_suspend.suspend = battery_early_suspend;
		battery->early_suspend.resume = battery_late_resume;
		register_early_suspend(&battery->early_suspend);
	}
#if 0 /* eur-feature */
	fg_atcom_test = device_create(sec_class, NULL, 0, NULL, "fg_atcom_test");

	if (IS_ERR(fg_atcom_test)) {
		//printk("Failed to create device(fg_atcom_test)!\n");
	}

	if (device_create_file(fg_atcom_test, &dev_attr_set_fuel_gauage_read) < 0) {
		//printk("Failed to create device file(%s)!\n", dev_attr_set_fuel_gauage_read.attr.name);
	}
	if (device_create_file(fg_atcom_test, &dev_attr_set_fuel_gauage_reset) < 0) {
		//printk("Failed to create device file(%s)!\n", dev_attr_set_fuel_gauage_reset.attr.name);
	}
#endif /* eur-feature */

	/* check bootcomplete in case of normal booting. */
	if (charging_mode_get()) {
		//ibatt_test_value = 1; /* for temperature block test in case of lpm mode */
		iboot_completed = 1;
		printk("[battery-lpm_mode] boot completed! - iboot_completed = %d\n", iboot_completed);
	} else {
		iboot_completed = 0;
#ifdef __PSEUDO_BOOT_COMPLETED__
		schedule_delayed_work(&boot_complete_work, 50000);
#endif /* __PSEUDO_BOOT_COMPLETED__ */
	}

	s3c_bat_info.callbacks.set_cable = s3c_set_cable_cb;
	s3c_bat_info.callbacks.set_esafe = NULL;
	s3c_bat_info.callbacks.get_vdcin = NULL;
	if (s3c_bat_info.pdata->register_callbacks)
		s3c_bat_info.pdata->register_callbacks(&s3c_bat_info.callbacks);

__end__:
	return ret;
}

#ifdef CONFIG_PM
static int s3c_bat_suspend(struct platform_device *pdev, pm_message_t state)
{
	//pr_info("[BAT]:%s\n", __func__);

	//set_low_bat_interrupt(1); /* eur-feature */

	if (s3c_bat_info.polling) {
		del_timer_sync(&polling_timer);
	}

	flush_scheduled_work();
	return 0;
}

static int s3c_bat_resume(struct platform_device *pdev)
{
	//pr_info("[BAT]:%s\n", __func__);
	//wake_lock(&vbus_wake_lock);
	//set_low_bat_interrupt(0); /* eur-feature */

	schedule_work(&bat_work);

	if (s3c_bat_info.polling) {
		mod_timer(&polling_timer, jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
	}
	return 0;
}
#endif /* CONFIG_PM */

static int __devexit s3c_bat_remove(struct platform_device *pdev)
{
	int i;
	//pr_info("[BAT]:%s\n", __func__);

	if (s3c_bat_info.polling) {
		del_timer_sync(&polling_timer);
	}

	for (i = 0; i < ARRAY_SIZE(s3c_power_supplies); i++) {
		power_supply_unregister(&s3c_power_supplies[i]);
	}

	return 0;
}

static struct platform_driver s3c_bat_driver = {
	.driver.name = DRIVER_NAME,
	.driver.owner = THIS_MODULE,
	.probe = s3c_bat_probe,
	.remove = __devexit_p(s3c_bat_remove),
	.suspend = s3c_bat_suspend,
	.resume = s3c_bat_resume,
};

static int __init s3c_bat_init(void)
{
	//pr_info("[BAT]:%s\n", __func__);

	wake_lock_init(&update_wake_lock, WAKE_LOCK_SUSPEND, "update_status");
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");

	entry = create_proc_entry(PROC_FILENAME, S_IRUGO, NULL);
	if (entry == NULL) {
		pr_err("%s: Can't create_proc_entry\n", __func__);
	}
#ifdef __FUEL_GAUGES_IC__
	if (i2c_add_driver(&fg_i2c_driver)) {
		//pr_err("%s: Can't add fg i2c drv\n", __func__);
	}
#endif /* __FUEL_GAUGES_IC__ */
	return platform_driver_register(&s3c_bat_driver);
}

static void __exit s3c_bat_exit(void)
{
	//pr_info("[BAT]:%s\n", __func__);

	remove_proc_entry(PROC_FILENAME, NULL);

#ifdef __FUEL_GAUGES_IC__
	i2c_del_driver(&fg_i2c_driver);
#endif /* __FUEL_GAUGES_IC__ */
	platform_driver_unregister(&s3c_bat_driver);
}

late_initcall(s3c_bat_init);
module_exit(s3c_bat_exit);

MODULE_AUTHOR("Minsung Kim <ms925.kim@samsung.com>");
MODULE_DESCRIPTION("S3C6410 battery driver");
MODULE_LICENSE("GPL");
