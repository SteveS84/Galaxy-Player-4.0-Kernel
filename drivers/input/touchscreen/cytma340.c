/* drivers/input/touchscreen/qt602240.c
 *
 * Quantum TSP driver.
 *
 * Copyright (C) 2009 Samsung Electronics Co. Ltd.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/timer.h>    // add timer
#include <asm/io.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/jiffies.h>
#include <plat/iic.h> //#include "cytma340.h"
#include <linux/workqueue.h>
#include <plat/regs-watchdog.h>

/*
 *	Operation Features
 */

#define CYTSP_TIMER_ENABLE
// #define	CYTSP_HWRESET_LONGKEY // Rossi Workaround Code
//#define CYTSP_WDOG_ENABLE
#define CYTSP_FWUPG_ENABLE

#define TOUCH_DVFS_CONTROL 1

#ifdef CYTSP_FWUPG_ENABLE
#ifdef CONFIG_VENTURI_USA
#include "cytma340_fw_usa.h"
#else
#include "cytma340_fw.h"
#endif
#endif

#ifdef CONFIG_CPU_FREQ
#include <mach/cpu-freq-v210.h>
#endif

#include <mach/gpio.h>


static	void	__iomem		*gpio_pend_mask_mem;

#define INT_PEND_BASE	0xE0200A44
#define IRQ_TOUCH_INT (IRQ_EINT_GROUP18_BASE+5) /* J0_5 */

enum driver_setup_t {DRIVER_SETUP_OK, DRIVER_SETUP_INCOMPLETE};



struct i2c_driver 			cytouch_i2c_driver;
struct workqueue_struct 	*cytouch_wq = NULL;
static struct input_dev*	gp_cytouch_input;
static struct i2c_client*	gp_cytouch_client;
static struct early_suspend	g_cytouch_early_suspend;
struct work_struct 			g_cytouch_work;
static enum driver_setup_t 	driver_setup = DRIVER_SETUP_INCOMPLETE;
struct delayed_work			g_cytouch_dwork;
static bool resume_dvfs_lock;

extern void led_power_control(int onoff);

extern int checkTSPKEYdebuglevel;
#define KERNEL_SEC_DEBUG_LEVEL_LOW	(0x574F4C44)
#define KERNEL_SEC_DEBUG_LEVEL_MID	(0x44494D44)
#define KERNEL_SEC_DEBUG_LEVEL_HIGH	(0x47494844)

/* Early Suspend */
#define USE_TSP_EARLY_SUSPEND
static int cytouch_early_suspend(struct early_suspend *h);
static int cytouch_late_resume(struct early_suspend *h);

static void cytouch_register_irq(void);



/* Device ID */
static int 			g_vendor_id;
static int 			g_module_id;
static int 			g_fw_ver;
static uint8_t		tsp_version;

/* TouchKey */
#define TOUCHKEY_ENABLE
#ifdef TOUCHKEY_ENABLE
#define TOUCHKEY_MENU			KEY_MENU
#define TOUCHKEY_BACK			KEY_BACK
#endif

#define TOUCHKEY_LED_ENABLE

#ifdef TOUCHKEY_LED_ENABLE
static int touchkey_control(int data);
#endif

void init_hw_setting(void);



/* Debug Function */
#define CYTSPDBG_ENABLE
#ifdef CYTSPDBG_ENABLE
#define CYTSPDBG(fmt, args...)	printk(fmt, ##args)
#define DEBUG printk("[TSP] %s/%d\n",__func__,__LINE__)
#define DEBUG_MSG(p, x...)			printk("[TSP]:[%s] ", __func__); printk(p, ## x);
#define ENTER_FUNC	{ printk("[TSP] +%s\n", __func__); }
#define LEAVE_FUNC	{ printk("[TSP] -%s\n", __func__); }
#else
#define CYTSPDBG(fmt, args...)	do {} while(0)
#define DEBUG
#define DEBUG_MSG(p, x...)
#define ENTER_FUNC
#define LEAVE_FUNC
#endif


/* Variable */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static DEFINE_MUTEX(cytouch_i2c_lock);

#ifdef CYTSP_FWUPG_ENABLE
static int cytouch_upgrade_fw(void);
static int cytouch_upgrade_fw_force(void);
int tma340_frimware_update(void);
#define CYPRESS_DISABLE_WATCHDOG_TIMER_RESET()   __raw_writel(0, S3C2410_WTCON); /* disable watchdog, to download touch firmware */  
/* enable watchdog, after  downloading  touch firmware  */
#define CYPRESS_ROLLBACK_WATCHDOG_TIMER_RESET()    unsigned int val;              \
           static unsigned watchdog_reset = (30 * 2048);      \
           val = S3C2410_WTCON_DIV128;        \
             val |= S3C2410_WTCON_PRESCALE(255);                  \
               writel(val, S3C2410_WTCON);                          \
        writel(watchdog_reset, S3C2410_WTCNT);               \
               writel(watchdog_reset, S3C2410_WTDAT);              \
               val |= S3C2410_WTCON_RSTEN | S3C2410_WTCON_ENABLE;   \
        writel(val,S3C2410_WTCON);


#endif


unsigned int touch_state_val=0;
EXPORT_SYMBOL(touch_state_val);

int set_tsp_for_ta_detect(int state)
{
	return 1;
}
EXPORT_SYMBOL(set_tsp_for_ta_detect);


void TSP_forced_release(void)
{
}
EXPORT_SYMBOL(TSP_forced_release);

void TSP_forced_release_forOKkey(void)
{
}

EXPORT_SYMBOL(TSP_forced_release_forOKkey);


static int cytouch_i2c_read(u8 reg, u8* data, int len);


/********************** ++Cypress ***********************************/

#define CYTOUCH_REG_HST_MODE 		0x00
#define CYTOUCH_REG_TT_MODE 		0x01
#define CYTOUCH_REG_TT_STAT 		0x02	// 0
#define CYTOUCH_REG_TOUCH1_XH 		0x03	// 1
#define CYTOUCH_REG_TOUCH1_XL 		0x04	// 2
#define CYTOUCH_REG_TOUCH1_YH 		0x05	// 3
#define CYTOUCH_REG_TOUCH1_YL 		0x06	// 4
#define CYTOUCH_REG_TOUCH1_Z 		0x07	// 5
#define CYTOUCH_REG_TOUCH12_ID 		0x08	// 6
#define CYTOUCH_REG_TOUCH2_XH 		0x09	// 7
#define CYTOUCH_REG_TOUCH2_XL 		0x0A	// 8
#define CYTOUCH_REG_TOUCH2_YH 		0x0B	// 9
#define CYTOUCH_REG_TOUCH2_YL 		0x0C	// 10
#define CYTOUCH_REG_TOUCH2_Z 		0x0D	// 11
#define CYTOUCH_REG_GEST_CNT 		0x0E
#define CYTOUCH_REG_GEST_ID 		0x0F
#define CYTOUCH_REG_TOUCH3_XH 		0x10
#define CYTOUCH_REG_TOUCH3_XL 		0x11
#define CYTOUCH_REG_TOUCH3_YH 		0x12
#define CYTOUCH_REG_TOUCH3_YL 		0x13
#define CYTOUCH_REG_TOUCH3_Z 		0x14
#define CYTOUCH_REG_TOUCH34_ID 		0x15
#define CYTOUCH_REG_TOUCH4_XH 		0x16
#define CYTOUCH_REG_TOUCH4_XL 		0x17
#define CYTOUCH_REG_TOUCH4_YH 		0x18
#define CYTOUCH_REG_TOUCH4_YL 		0x19
#define CYTOUCH_REG_TOUCH4_Z 		0x1A
#define CYTOUCH_REG_VENDOR_ID 		0x1B
#define CYTOUCH_REG_MODULE_ID 		0x1C
#define CYTOUCH_REG_FW_VER 			0x1D
#define CYTOUCH_REG_GEST_SET 		0x1E
#define CYTOUCH_REG_WDOG 			0x1F
#define CYTOUCH_REG_CALIBRATE		0x00
#define CYTOUCH_REG_READ_START		CYTOUCH_REG_TT_STAT
#define CYTOUCH_REG_READ_SIZE		(CYTOUCH_REG_TOUCH2_Z-CYTOUCH_REG_TT_STAT+1)
#define CYTOUCH_REG_READ_POS(x)		(x-CYTOUCH_REG_READ_START)
#define CYTOUCH_MAX_ID			(15)


typedef enum
{
	CYTOUCH_PWROFF = 0,
	CYTOUCH_PWRON = 1,
}CYTOUCH_PWRSTAT;

static int cytouch_hw_set_pwr(CYTOUCH_PWRSTAT onoff);

typedef struct
{
	int x;
	int y;
	int z;
	int stat;
	int id;
}CYTOUCH_POINT;

enum
{
	CYTOUCH_ID_STAT_RELEASED = 0,
	CYTOUCH_ID_STAT_PRESSED = 1,
	CYTOUCH_ID_STAT_MOVED = 2,
};

enum
{
	CYTOUCH_ID_STAT_DIRTY = 0,
	CYTOUCH_ID_STAT_NEW = 1,
};

typedef struct
{
	int status;
	int dirty;
	int x,y,z;
	int b_report;
}CYTOUCH_ID_STAT;

CYTOUCH_ID_STAT g_cytouch_id_stat[CYTOUCH_MAX_ID+1]={{0,0,0,0,0,0},};// {0,};
bool g_cytouch_log[CYTOUCH_MAX_ID+1] = {0,};

typedef struct
{
	u8 tt_stat;		// 0
	u8 touch1_xh;	// 1
	u8 touch1_xl;	// 2
	u8 touch1_yh;	// 3
	u8 touch1_yl;	// 4
	u8 touch1_z;	// 5
	u8 touch12_id;	// 6
	u8 touch2_xh;	// 7
	u8 touch2_xl;	// 8
	u8 touch2_yh;	// 9
	u8 touch2_yl;	//10
	u8 touch2_z;	//11
	u8 gest_cnt;	//12
	u8 gest_id;		//13
	u8 touch3_xh;	//14
	u8 touch3_xl;	//15
	u8 touch3_yh;	//16
	u8 touch3_yl;	//17
	u8 touch3_z;	//18
	u8 touch34_id;	//19
	u8 touch4_xh;	//20
	u8 touch4_xl;	//21
	u8 touch4_yh;	//22
	u8 touch4_yl;	//23
	u8 touch4_z;	//24    size = 25
#ifdef CONFIG_VENTURI_USA
	u8 vendor_id;	//25    size = 26
#endif
}__attribute__((packed))CYTOUCH_RAW_DATA;

/********************** --Cypress ***********************************/

#define TSP_PRESSED 				1
#define TSP_RELEASE 				0
#define TSP_INITIAL 				-1
#define TSP_FORCED_RELEASE			-2
#ifdef CONFIG_VENTURI_USA
#define TSP_MENUKEY_PRESS			0x01
#define TSP_HOMEKEY_PRESS			0x02
#define TSP_BACKKEY_PRESS			0x04
#else
#define TSP_MENUKEY_PRESS			0x40
#define TSP_BACKKEY_PRESS			0x80
#endif
#define TOUCHKEY_KEYCODE_MENU		158
#define TOUCHKEY_KEYCODE_BACK		28

static u8		prev_menu =	0;
static u8		prev_back =	0;
static u8		prev_num_of_touch = 0;

#ifdef CYTSP_TIMER_ENABLE
static struct timer_list g_cytouch_backkey_timer;
static struct timer_list g_cytouch_menukey_timer;
static struct timer_list g_cytouch_touch_timer;
#endif

static int prev_wdog_val = -1;

static u64 g_backkey_start_time = 0;
static u64 g_menukey_start_time = 0;

static int 	g_suspend_state = FALSE;

#ifdef CONFIG_VENTURI_USA
#ifdef TOUCHKEY_ENABLE
#define TOUCHKEY_HOME			KEY_HOME
#endif

#define TOUCHKEY_KEYCODE_HOME		139

static u8		prev_home =	0;
#ifdef CYTSP_TIMER_ENABLE
static struct timer_list g_cytouch_homekey_timer;
#endif
static u64 g_homekey_start_time = 0;

#endif

#ifdef CYTSP_TIMER_ENABLE
void cytouch_backkey_timer_body(struct work_struct* p_work)
{
	u8 buf = 0x0;
	int ret = 0;

	if(	g_suspend_state == TRUE)
	{
		return;
	}

	/*	read I2C
	*/
    mutex_lock(&cytouch_i2c_lock);
	ret = cytouch_i2c_read(CYTOUCH_REG_TT_STAT, (u8 *)&buf, 1);
	mutex_unlock(&cytouch_i2c_lock);
	if (ret != 0)
	{
		printk("%s : fail!\n", __func__);
		return;
	}

	if (buf == 0x0)
	{
		if(prev_back == TSP_PRESSED)
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : lost BACK release!\n", __func__);
			input_report_key(gp_cytouch_input, TOUCHKEY_BACK, 0);
			input_sync(gp_cytouch_input);
			prev_back = TSP_RELEASE;
			g_backkey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
		}
	}
}

void cytouch_menukey_timer_body(struct work_struct* p_work)
{
	u8 buf = 0x0;
	int ret = 0;

	if( g_suspend_state == TRUE )
	{
		return;
	}

	/*	read I2C
	*/
    mutex_lock(&cytouch_i2c_lock);
	ret = cytouch_i2c_read(CYTOUCH_REG_TT_STAT, (u8 *)&buf, 1);
	mutex_unlock(&cytouch_i2c_lock);
	if (ret < 0)
	{
		printk("%s : fail!\n", __func__);
		return;
	}

	if (buf == 0x0)
	{
		if (TSP_PRESSED== prev_menu)
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : lost MENU release!\n", __func__);
			input_report_key(gp_cytouch_input, TOUCHKEY_MENU, 0);
			input_sync(gp_cytouch_input);
			prev_menu = TSP_RELEASE;
			g_menukey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
		}
	}
}

DECLARE_WORK(cytouch_backkey_timer_wq, cytouch_backkey_timer_body);
DECLARE_WORK(cytouch_menukey_timer_wq, cytouch_menukey_timer_body);

/* this is called when last touch key IRQ isn't handled */
/* since i2c read should be done outside of atomic scope, */
/* schedule_work() is used.. -.- */
static void cytouch_backkey_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_backkey_timer_wq);
}

static void cytouch_menukey_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_menukey_timer_wq);
}

#ifdef CONFIG_VENTURI_USA
void cytouch_homekey_timer_body(struct work_struct* p_work)
{
	u8 buf = 0x0;
	int ret = 0;

	if(	g_suspend_state == TRUE)
	{
		return;
	}

	/*	read I2C
	*/
    mutex_lock(&cytouch_i2c_lock);
	ret = cytouch_i2c_read(CYTOUCH_REG_TT_STAT, (u8 *)&buf, 1);	
	mutex_unlock(&cytouch_i2c_lock);	
	if (ret != 0) 
	{
		printk("%s : fail!\n", __func__);
		return;
	}

	if (buf == 0x0)
	{
		if(prev_home == TSP_PRESSED)
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : lost HOME release!\n", __func__);
			input_report_key(gp_cytouch_input, TOUCHKEY_HOME, 0);
			input_sync(gp_cytouch_input);
			prev_home = TSP_RELEASE;
			g_homekey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
		}
	}
}

DECLARE_WORK(cytouch_homekey_timer_wq, cytouch_homekey_timer_body);

static void cytouch_homekey_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_homekey_timer_wq);
}
#endif /* CONFIG_VENTURI_USA */

void cytouch_touch_timer_body(struct work_struct* p_work)
{
	u8 				buf = 0x0;
	int 			i 	= 0;
	int 			ret = 0;


	if( g_suspend_state == TRUE )
	{
		return;
	}

	/*	read I2C
	*/
        mutex_lock(&cytouch_i2c_lock);
	ret = cytouch_i2c_read(CYTOUCH_REG_TT_STAT, (u8 *)&buf, 1);
	mutex_unlock(&cytouch_i2c_lock);
	if (ret < 0)
	{
		printk("%s : fail!\n", __func__);
		return;
	}

	if (0 == (buf & 0xf))	/* lower 4bits are number of touch inputs */
	{
		/* check previous touch input and return RELEASE */
		for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
		{
			if (g_cytouch_id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
			{
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_X, g_cytouch_id_stat[i].x);
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_Y, g_cytouch_id_stat[i].y);
				input_report_abs(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, ((i<<8)| g_cytouch_id_stat[i].z));
				input_report_abs(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 0);
				g_cytouch_id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
				if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
					CYTSPDBG("lostUP(%d,%d,%d,%d)\n", g_cytouch_id_stat[i].x, g_cytouch_id_stat[i].y, g_cytouch_id_stat[i].z, i);
			}

			g_cytouch_id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;

			input_mt_sync(gp_cytouch_input);
#if TOUCH_DVFS_CONTROL
			if(i == 1)
			{
				// first touch released!
				if(touch_state_val == 1)
				{
					s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
					resume_dvfs_lock = false;
					touch_state_val = 0;
				}
			}
#endif
		}
		input_sync(gp_cytouch_input);
	}
}

DECLARE_WORK(cytouch_touch_timer_wq, cytouch_touch_timer_body);

static void cytouch_touch_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_touch_timer_wq);
}


static void cytouch_init_rel_timer(void)
{
	init_timer(&g_cytouch_backkey_timer);
	init_timer(&g_cytouch_menukey_timer);
	init_timer(&g_cytouch_touch_timer);

	g_cytouch_backkey_timer.function = cytouch_backkey_timer_handler;
	g_cytouch_menukey_timer.function = cytouch_menukey_timer_handler;
	g_cytouch_touch_timer.function = cytouch_touch_timer_handler;
#ifdef CONFIG_VENTURI_USA
	init_timer(&g_cytouch_homekey_timer);
	g_cytouch_homekey_timer.function = cytouch_homekey_timer_handler;
#endif
}
#endif // --CYTSP_TIMER_ENABLE


static void cytouch_wdog_wq_body(struct work_struct* p_work);

DECLARE_DELAYED_WORK(cytouch_wdog_wq, cytouch_wdog_wq_body);

void cytouch_release_all(void)
{
	int i = 0;

#ifdef CYTSP_TIMER_ENABLE
	del_timer(&g_cytouch_backkey_timer);
	del_timer(&g_cytouch_menukey_timer);
	del_timer(&g_cytouch_touch_timer);

	cancel_work_sync(&cytouch_backkey_timer_wq);
	cancel_work_sync(&cytouch_menukey_timer_wq);
	cancel_work_sync(&cytouch_touch_timer_wq);
#endif

	input_report_key(gp_cytouch_input, TOUCHKEY_MENU, 0);	// Menu Key release
	input_report_key(gp_cytouch_input, TOUCHKEY_BACK, 0);	// Back Key release

	prev_menu = TSP_RELEASE;
	prev_back = TSP_RELEASE;

	g_backkey_start_time = 0;
	g_menukey_start_time = 0;
#ifdef CONFIG_VENTURI_USA
#ifdef CYTSP_TIMER_ENABLE
	del_timer(&g_cytouch_homekey_timer);
	cancel_work_sync(&cytouch_homekey_timer_wq);
#endif

	input_report_key(gp_cytouch_input, TOUCHKEY_HOME, 0);	// Home Key release 

	prev_home = TSP_RELEASE;

	g_homekey_start_time = 0;
#endif
	/* check previous touch input and return RELEASE */
	for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
	{
		if (g_cytouch_id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
		{
			input_report_abs(gp_cytouch_input, ABS_MT_POSITION_X, g_cytouch_id_stat[i].x);
			input_report_abs(gp_cytouch_input, ABS_MT_POSITION_Y, g_cytouch_id_stat[i].y);
			input_report_abs(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, ((i<<8)| g_cytouch_id_stat[i].z));
			input_report_abs(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 0);
			g_cytouch_id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
			input_mt_sync(gp_cytouch_input);
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("FAKE_UP[%d](%d,%d,%d,%d)\n", i, g_cytouch_id_stat[i].x, g_cytouch_id_stat[i].y, g_cytouch_id_stat[i].z, i);
			g_cytouch_log[i] = 0;
		}

		g_cytouch_id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;
	}
	input_sync(gp_cytouch_input);	/* Rossi jmin : power on button backlight */
#if TOUCH_DVFS_CONTROL
	if(touch_state_val == 1)
	{
		s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
		resume_dvfs_lock = false;
		touch_state_val = 0;
	}
#endif
}

static void cytouch_wdog_wq_body(struct work_struct* p_work)
{
	u8 wdog_val = 0x0;
	int fail = 0;
	int i;
	int ret;

	if(	g_suspend_state == TRUE )
	{
		return;
	}

	mutex_lock(&cytouch_i2c_lock);
	if (1 == gpio_get_value(GPIO_TSP_SDA_28V) &&  1 == gpio_get_value(GPIO_TSP_SCL_28V))
	{
		for (i = 0; i < 5; i++)
		{
			ret = cytouch_i2c_read(CYTOUCH_REG_WDOG, &wdog_val, 1);
			if (0 == ret)
			{
				break;
			}
			mdelay(5);
		}
		if (i == 5)
		{
			CYTSPDBG("%s : fail!\n", __func__);
			fail = 1;
		}
	}
	else
	{
		CYTSPDBG("%s : I2C line setup fail!\n", __func__);
		fail = 1;
	}
	mutex_unlock(&cytouch_i2c_lock);

	/* wdog value isn't changed OR i2c fails */
	if (wdog_val == (u8)prev_wdog_val || fail == 1)
	{
		printk("%s : Touch WDOG fail! (%d->%d)\n", __func__, prev_wdog_val, wdog_val);
		disable_irq(IRQ_TOUCH_INT);
		cytouch_hw_set_pwr(CYTOUCH_PWROFF);
		mdelay(20);
		cytouch_hw_set_pwr(CYTOUCH_PWRON);
		msleep(400);
		cytouch_release_all();
		if(readl(gpio_pend_mask_mem)&(0x1<<5))
			writel(readl(gpio_pend_mask_mem)|(0x1<<5), gpio_pend_mask_mem);
		enable_irq(IRQ_TOUCH_INT);
		prev_wdog_val = -1;
	}
	else
	{
		// CYTSPDBG("%s : Touch WDOG OK! (%d->%d)\n", __func__, prev_wdog_val, wdog_val);
		prev_wdog_val = wdog_val;
	}

	schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(1200));
}

static void cytouch_init_wdog(void)
{
	schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(1200));
}

static void cytouch_pause_wdog(void)
{
	CYTSPDBG("%s : Touch WDOG paused!\n", __func__);
	cancel_delayed_work_sync(&cytouch_wdog_wq);
}

static void cytouch_resume_wdog(void)
{
	CYTSPDBG("%s : Touch WDOG resumed!\n", __func__);
	prev_wdog_val = -1;
	schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(1500));
}

void  get_message(struct work_struct * p)
{
	int 			ret = 0;
	int 			i = 0;
	u8 				num_of_touch = 0, key = 0;
#ifdef CYTSP_TIMER_ENABLE
	int 			b_add_backkey_timer = FALSE;
	int 			b_add_menukey_timer = FALSE;
#ifdef CONFIG_VENTURI_USA
	int 			b_add_homekey_timer = FALSE;
#endif
	int 			b_add_touch_timer 	= FALSE;
#endif
#ifdef CYTSP_HWRESET_LONGKEY // workaround code for Rossi
	int 			b_menukey_reset = FALSE;
	int 			b_backkey_reset = FALSE;
#endif
	int				dx = 0, dy = 0;
	CYTOUCH_POINT 	point[4] = {{0,0,0,0,0}, {0,0,0,0,0}, {0,0,0,0,0}, {0,0,0,0,0}};
	CYTOUCH_RAW_DATA buf = {0,};

	//printk("[TSP] get_message  \n");

	if (driver_setup != DRIVER_SETUP_OK)
	{
		goto work_func_out;
	}

	if(	g_suspend_state == TRUE )
	{
		goto work_func_out;
	}

#ifdef CYTSP_TIMER_ENABLE
	/*	delete timer
	*/
	del_timer(&g_cytouch_touch_timer);
#endif

	/*	read Data
	 */
    mutex_lock(&cytouch_i2c_lock);
	ret = cytouch_i2c_read(CYTOUCH_REG_READ_START, (u8 *)&buf, sizeof(buf));
	mutex_unlock(&cytouch_i2c_lock);
	if (ret < 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		mutex_lock(&cytouch_i2c_lock);
		ret = cytouch_i2c_read(CYTOUCH_REG_READ_START, (u8 *)&buf, sizeof(buf));
		mutex_unlock(&cytouch_i2c_lock);
		if (ret < 0) {
			printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
#ifdef CYTSP_WDOG_ENABLE
			if(g_fw_ver >= 0x5)
				cytouch_pause_wdog();
#endif
			cytouch_release_all();

			mdelay(20);

			//s3c_i2c2_force_stop();

			cytouch_hw_set_pwr(CYTOUCH_PWROFF);
			msleep(100);
			
			if( g_suspend_state == FALSE ) {
				cytouch_hw_set_pwr(CYTOUCH_PWRON);
				msleep(400);
			}
			goto work_func_out;
		}
	}

	/*	parse Data
	*/
	num_of_touch 	= buf.tt_stat & 0x0F;	 // pressed finger count
#ifdef CONFIG_VENTURI_USA
	key 			= buf.vendor_id;	 // pressed key
#else
	key 			= buf.tt_stat & 0xC0;	 // pressed key
#endif
	//if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
		//CYTSPDBG("num=%d,key=%d\n", num_of_touch, key);

	/* check touch & key press
	*/
	if(key)
	{
		if(num_of_touch)
		{
			// if touch & key pressed at the same time
			// should ignore key
			key = 0;
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("touch&key pressed. ignore key\n");
		}
	}

#ifdef CYTSP_TIMER_ENABLE
	/*	delete timer
	*/
	if(key == TSP_MENUKEY_PRESS)
		del_timer(&g_cytouch_menukey_timer);
	else if(key == TSP_BACKKEY_PRESS)
		del_timer(&g_cytouch_backkey_timer);
#ifdef CONFIG_VENTURI_USA
	else if (key == TSP_HOMEKEY_PRESS)
		del_timer(&g_cytouch_homekey_timer);
#endif
#endif

	/*	parse Data
	*/
	point[0].x = (int)buf.touch1_xh << 8 | (int)buf.touch1_xl;
	point[0].y = (int)buf.touch1_yh << 8 | (int)buf.touch1_yl;
	point[0].z = (int)buf.touch1_z;
	point[0].id = (int)buf.touch12_id >> 4;

	point[1].x = (int)buf.touch2_xh << 8 | (int)buf.touch2_xl;
	point[1].y = (int)buf.touch2_yh << 8 | (int)buf.touch2_yl;
	point[1].z = (int)buf.touch2_z;
	point[1].id = (int)buf.touch12_id & 0xf;

	point[2].x = (int)buf.touch3_xh << 8 | (int)buf.touch3_xl;
	point[2].y = (int)buf.touch3_yh << 8 | (int)buf.touch3_yl;
	point[2].z = (int)buf.touch3_z;
	point[2].id = (int)buf.touch34_id >> 4;

	point[3].x = (int)buf.touch4_xh << 8 | (int)buf.touch4_xl;
	point[3].y = (int)buf.touch4_yh << 8 | (int)buf.touch4_yl;
	point[3].z = (int)buf.touch4_z;
	point[3].id = (int)buf.touch34_id & 0xf;

	/*	check Touch Count
	*/
	if(num_of_touch > 4)
	{
		/* invalid Touch Input */
		goto work_func_out;
	}

	// wrokaround
	for(i=0; i < num_of_touch; i++)
	{
		if((point[i].x > 480) || (point[i].y > 800))
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
			    CYTSPDBG("err(%d,%d,%d,%d)\n", point[i].x, point[i].y, point[i].z, point[i].id);
			goto work_func_out;
		}
		//if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
			//CYTSPDBG("(%d,%d,%d,%d)\n", point[i].x, point[i].y, point[i].z, point[i].id);
	}
#if TOUCH_DVFS_CONTROL
#ifdef CONFIG_VENTURI_USA
	if (num_of_touch > 0) {
		if(touch_state_val == 0)
		{
			s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L1); // cpu high speed setting.
			resume_dvfs_lock = false;
			touch_state_val = 1;
		}
	}
#else
	if(point[0].id == 1)
	{
		// first touch pressed!
		if(touch_state_val == 0)
		{
			s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L1); // cpu high speed setting.
			resume_dvfs_lock = false;
			touch_state_val = 1;
		}
	}
#endif
#endif
	//CYTSPDBG("num_of_touch=%d, key=0x%x, TOUCH12_ID=0x%x\n", num_of_touch, key, buf.touch12_id);


	/***********************************************************/
	/*		Touch Key	Processing							     */
	/***********************************************************/

	if(key == TSP_MENUKEY_PRESS)
	{
		// MenuKey pressed
#ifdef CONFIG_VENTURI_USA
		if((prev_back != TSP_PRESSED) && (prev_menu != TSP_PRESSED) && (prev_home != TSP_PRESSED))
#else
		if((prev_back != TSP_PRESSED) && (prev_menu != TSP_PRESSED))
#endif
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : MENU DOWN\n", __func__);

			input_report_key(gp_cytouch_input, TOUCHKEY_MENU,1);
			input_sync(gp_cytouch_input);
			prev_menu = TSP_PRESSED;
#ifdef CYTSP_TIMER_ENABLE
			b_add_menukey_timer = TRUE;
#endif
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 0)
			{
				s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L1); // cpu high speed setting.
				resume_dvfs_lock = false;
				touch_state_val = 1;
			}
#endif
		}

#ifdef CYTSP_HWRESET_LONGKEY // workaround code for Rossi
		if (g_menukey_start_time == 0)
		{
			g_menukey_start_time = get_jiffies_64();
		}
		else
		{
			//CYTSPDBG("menu press\n"); // test
			if (time_after64(get_jiffies_64(), g_menukey_start_time + msecs_to_jiffies(3000)))
			{
				b_menukey_reset = TRUE;
			}
		}
#endif
	}
	else if(key == TSP_BACKKEY_PRESS)
	{
		// BackKey pressed
#ifdef CONFIG_VENTURI_USA
		if((prev_back != TSP_PRESSED) && (prev_menu != TSP_PRESSED) && (prev_home != TSP_PRESSED))
#else
		if((prev_back != TSP_PRESSED) && (prev_menu != TSP_PRESSED))
#endif
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : BACK DOWN\n", __func__);

			input_report_key(gp_cytouch_input, TOUCHKEY_BACK,1);
			input_sync(gp_cytouch_input);
			prev_back = TSP_PRESSED;
#ifdef CYTSP_TIMER_ENABLE
			b_add_backkey_timer = TRUE;
#endif
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 0)
			{
				s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L1); // cpu high speed setting.
				resume_dvfs_lock = false;
				touch_state_val = 1;
			}
#endif
		}
#ifdef CYTSP_HWRESET_LONGKEY // workaround code for Rossi
		if (g_backkey_start_time == 0)
		{
			g_backkey_start_time = get_jiffies_64();
		}
		else
		{
			if (time_after64(get_jiffies_64(), g_backkey_start_time + msecs_to_jiffies(3000)))
			{
				b_backkey_reset = TRUE;
			}
		}
#endif
	}
#ifdef CONFIG_VENTURI_USA
	else if(key == TSP_HOMEKEY_PRESS)
	{
		// HomeKey pressed
		if((prev_back != TSP_PRESSED) && (prev_menu != TSP_PRESSED) && (prev_home != TSP_PRESSED))
		{
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : HOME DOWN\n", __func__);

			input_report_key(gp_cytouch_input, TOUCHKEY_HOME,1);
			input_sync(gp_cytouch_input);
			prev_home = TSP_PRESSED;
#ifdef CYTSP_TIMER_ENABLE
			b_add_homekey_timer = TRUE;
#endif
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 0)
			{
				s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L1); // cpu high speed setting.
				resume_dvfs_lock = false;
				touch_state_val = 1;
			}
#endif
		}
	}
#endif
#ifdef CONFIG_VENTURI_USA
	else if(key == (TSP_MENUKEY_PRESS | TSP_BACKKEY_PRESS | TSP_HOMEKEY_PRESS))
#else
	else if(key == (TSP_MENUKEY_PRESS | TSP_BACKKEY_PRESS))
#endif
	{
		// both Key pressed, Not Supported
		;
	}
	else
	{
		// Key released

		if(prev_menu == TSP_PRESSED)
		{
			input_report_key(gp_cytouch_input, TOUCHKEY_MENU, 0);
			input_sync(gp_cytouch_input);
			prev_menu = TSP_RELEASE;
			g_menukey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : MENU UP\n", __func__);
		}
		else if(prev_back == TSP_PRESSED)
		{
			input_report_key(gp_cytouch_input, TOUCHKEY_BACK, 0);
			input_sync(gp_cytouch_input);
			prev_back = TSP_RELEASE;
			g_backkey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : BACK UP\n", __func__);
		}
#ifdef CONFIG_VENTURI_USA
		else if(prev_home == TSP_PRESSED)
		{
			input_report_key(gp_cytouch_input, TOUCHKEY_HOME, 0);
			input_sync(gp_cytouch_input);
			prev_home = TSP_RELEASE;
			g_backkey_start_time = 0;
#if TOUCH_DVFS_CONTROL
			if(touch_state_val == 1)
			{
				s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
				resume_dvfs_lock = false;
				touch_state_val = 0;
			}
#endif
			if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
				CYTSPDBG("%s : HOME UP\n", __func__);
		}
#endif
	}
#ifdef CYTSP_HWRESET_LONGKEY
	if (b_menukey_reset == TRUE || b_backkey_reset == TRUE)
	{
#ifdef CYTSP_WDOG_ENABLE
		if(g_fw_ver >= 0x5)
			cytouch_pause_wdog();
#endif
		disable_irq(IRQ_TOUCH_INT); // free_irq(IRQ_TOUCH_INT, 0);
		cytouch_release_all();
		cytouch_hw_set_pwr(CYTOUCH_PWROFF);
		msleep(10);
		cytouch_hw_set_pwr(CYTOUCH_PWRON);
		msleep(400);
		enable_irq(IRQ_TOUCH_INT); // cytouch_register_irq();
#ifdef CYTSP_WDOG_ENABLE
		if(g_fw_ver >= 0x5)
			cytouch_resume_wdog();
#endif
		printk("%s : backkey pressed too long.. hw reset now!\n", __func__);
		goto work_func_out;
	}
#endif // -- CYTSP_HWRESET_LONGKEY

#ifdef CYTSP_TIMER_ENABLE
	/* activate key timer to prevent unprocessed UP event */
	if (TRUE == b_add_backkey_timer)
	{
		g_cytouch_backkey_timer.expires = get_jiffies_64() + msecs_to_jiffies(40);
		add_timer(&g_cytouch_backkey_timer);
	}

	if (TRUE == b_add_menukey_timer)
	{
		g_cytouch_menukey_timer.expires = get_jiffies_64() + msecs_to_jiffies(40);
		add_timer(&g_cytouch_menukey_timer);
	}
#ifdef CONFIG_VENTURI_USA
	if (TRUE == b_add_homekey_timer)
	{
		g_cytouch_homekey_timer.expires = get_jiffies_64() + msecs_to_jiffies(40);
		add_timer(&g_cytouch_homekey_timer);
	}
#endif
#endif

	/***********************************************************/
	/*		Touch Processing							    		     */
	/***********************************************************/

	/* update the status of current touch inputs */
	for (i = 0; i < num_of_touch; i++)
	{
		if(g_cytouch_id_stat[point[i].id].status == CYTOUCH_ID_STAT_PRESSED)
		{
			dx = abs(g_cytouch_id_stat[point[i].id].x - point[i].x);
			dy = abs(g_cytouch_id_stat[point[i].id].y - point[i].y);

			if(dx <= 1 && dy <= 1)
			{
				if(num_of_touch < 2)
				{
					/*
					  *	(CASE1) -  1 FINGER LONG PRESS
					  */
					if (prev_num_of_touch > 1) {
						CYTSPDBG("switching two-finger touch to one-finger touch\n");
					} else {
					// discard new point which is near(<+-10) from previous point. (but accept all input for multi(2) touch input)
						g_cytouch_id_stat[point[i].id].b_report = FALSE;	/* suppress long touch input */
						g_cytouch_id_stat[point[i].id].status 	= CYTOUCH_ID_STAT_PRESSED;
						g_cytouch_id_stat[point[i].id].dirty 	= CYTOUCH_ID_STAT_NEW;
						continue;
					}
				}
			}
			/* Remark : if delta-value is getting smaller, fast-flick doesn't recognize.
			 */
			else if( (dx > 200) || (dy > 200) )
			{
				/*
				  *	(CASE2) - lost UP-Event from previous finger
				  */
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_X, g_cytouch_id_stat[point[i].id].x);
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_Y, g_cytouch_id_stat[point[i].id].y);
				input_report_abs(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, ((point[i].id<<8)| g_cytouch_id_stat[point[i].id].z));
				input_report_abs(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 0);
				g_cytouch_id_stat[point[i].id].status = CYTOUCH_ID_STAT_RELEASED;
				g_cytouch_id_stat[point[i].id].dirty = CYTOUCH_ID_STAT_DIRTY;
				input_mt_sync(gp_cytouch_input);
				input_sync(gp_cytouch_input);
				if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
					CYTSPDBG("lostUP[%d](%d,%d,%d,%d)\n", i, g_cytouch_id_stat[point[i].id].x, g_cytouch_id_stat[point[i].id].y, g_cytouch_id_stat[point[i].id].z, point[i].id);
				g_cytouch_log[i] = 0;
			}
		}

		// update Touch
		g_cytouch_id_stat[point[i].id].x 		= point[i].x;
		g_cytouch_id_stat[point[i].id].y 		= point[i].y;
		g_cytouch_id_stat[point[i].id].z 		= point[i].z;
		g_cytouch_id_stat[point[i].id].b_report = TRUE;
		g_cytouch_id_stat[point[i].id].status 	= CYTOUCH_ID_STAT_PRESSED;
		g_cytouch_id_stat[point[i].id].dirty 	= CYTOUCH_ID_STAT_NEW;
	}

	prev_num_of_touch = num_of_touch;

	/* check previous touch input and return RELEASE if new input is not */
	/* generated for that previous input */
	for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
	{
		if (g_cytouch_id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
		{
			if (g_cytouch_id_stat[i].dirty == CYTOUCH_ID_STAT_NEW)
			{
				if (g_cytouch_id_stat[i].b_report == TRUE)
				{
					if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW) {
						if (g_cytouch_log[i] == 0) {
							CYTSPDBG("DN[%d](%d,%d,%d,%d)\n", i, g_cytouch_id_stat[i].x, g_cytouch_id_stat[i].y, g_cytouch_id_stat[i].z, i);
							g_cytouch_log[i] = 1;
						}
					}
					input_report_abs(gp_cytouch_input, ABS_MT_POSITION_X, g_cytouch_id_stat[i].x);
					input_report_abs(gp_cytouch_input, ABS_MT_POSITION_Y, g_cytouch_id_stat[i].y);
					input_report_abs(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, ((i<<8)| g_cytouch_id_stat[i].z));
					input_report_abs(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 10);
					input_mt_sync(gp_cytouch_input);
				}
#ifdef CYTSP_TIMER_ENABLE
				b_add_touch_timer = TRUE;	/* to prevent unprocessed UP event */
#endif
			}
			else	/* previously pressed... so this is release... */
			{
				if (checkTSPKEYdebuglevel != KERNEL_SEC_DEBUG_LEVEL_LOW)
					CYTSPDBG("UP[%d](%d,%d,%d,%d)\n", i, g_cytouch_id_stat[i].x, g_cytouch_id_stat[i].y, g_cytouch_id_stat[i].z, i);
				g_cytouch_log[i] = 0;
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_X, g_cytouch_id_stat[i].x);
				input_report_abs(gp_cytouch_input, ABS_MT_POSITION_Y, g_cytouch_id_stat[i].y);
				input_report_abs(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, ((i<<8)| g_cytouch_id_stat[i].z) );
				input_report_abs(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 0);
				g_cytouch_id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
				input_mt_sync(gp_cytouch_input);
#if TOUCH_DVFS_CONTROL
#ifndef CONFIG_VENTURI_USA
				if(i == 1)
				{
					// first touch released!
					if(touch_state_val == 1)
					{
						s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
						resume_dvfs_lock = false;
						touch_state_val = 0;
					}
				}
#endif
#endif
			}
		}

		g_cytouch_id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;
	}

	input_sync(gp_cytouch_input);

#if TOUCH_DVFS_CONTROL
#ifdef CONFIG_VENTURI_USA
	if (num_of_touch == 0) {
		if(touch_state_val == 1) {
			s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
			resume_dvfs_lock = false;
			touch_state_val = 0;
		}
	}
#endif
#endif

#ifdef CYTSP_TIMER_ENABLE
	/*	start Timer
	*/
	if (TRUE == b_add_touch_timer)
	{
		g_cytouch_touch_timer.expires = get_jiffies_64() + msecs_to_jiffies(40);
		add_timer(&g_cytouch_touch_timer);
	}
#endif

work_func_out:
	if(readl(gpio_pend_mask_mem)&(0x1<<5))
		writel(readl(gpio_pend_mask_mem)|(0x1<<5), gpio_pend_mask_mem);

	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(0xf));
	enable_irq(IRQ_TOUCH_INT);

	return ;
}


#if 0 //defined(CONFIG_MACH_S5PC110_VENTURI)
 #define MDNIE_TUNING
extern int mDNIe_txtbuf_to_parsing2(void);
#endif

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef MDNIE_TUNING
	return sprintf(buf, "mdnie : %d\n",mDNIe_txtbuf_to_parsing2());
#else
	return sprintf(buf,"not support\n");
#endif
}

static ssize_t key_threshold_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int i;
	if(sscanf(buf,"%d",&i)==1)
	{
		CYTSPDBG("not support %d\n",i);
	}
	else
		CYTSPDBG("not support\n");

	return 0;
}

static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);


#ifdef CYTSP_FWUPG_ENABLE

struct device 		*cytouch_atcom_test;
struct work_struct 	qt_touch_update_work;
unsigned int 		qt_firm_status_data=0;

void set_qt_update_exe(struct work_struct * p)
{
	disable_irq(IRQ_TOUCH_INT);

#ifdef CYTSP_WDOG_ENABLE
		if(g_fw_ver >= 0x5)
			cytouch_pause_wdog();
#endif

    printk("Enter to Firmware download by AT command \n");

    if(!cytouch_upgrade_fw())
    {
		qt_firm_status_data=2;		// firmware update success
    	printk("Reprogram done : Firmware update Success~~~~~~~~~~\n");
    }
	else
	{
		qt_firm_status_data=3;		// firmware update Fail
		printk("[QT]Reprogram done : Firmware update Fail~~~~~~~~~~\n");
	}

	enable_irq(IRQ_TOUCH_INT);

#ifdef CYTSP_WDOG_ENABLE
	if(g_fw_ver >= 0x5)
		cytouch_resume_wdog();
#endif

}

static ssize_t set_qt_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	CYTSPDBG("touch firmware update \n");
	qt_firm_status_data=1;	//start firmware updating
	INIT_WORK(&qt_touch_update_work, set_qt_update_exe);
	queue_work(cytouch_wq, &qt_touch_update_work);

	if(qt_firm_status_data == 3)
	{
		count = sprintf(buf,"FAIL\n");
	}
	else
		count = sprintf(buf,"OK\n");
	return count;
}

static ssize_t set_qt_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", cytma340_new_fw_ver);

}

static ssize_t set_qt_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", tsp_version);

}

static ssize_t set_qt_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int count;

	printk("Enter set_qt_firm_status_show by AT command \n");

	if(qt_firm_status_data == 1)
	{
		count = sprintf(buf,"Downloading\n");
	}
	else if(qt_firm_status_data == 2)
	{
		count = sprintf(buf,"PASS\n");
	}
	else if(qt_firm_status_data == 3)
	{
		count = sprintf(buf,"FAIL\n");
	}
	else
		count = sprintf(buf,"PASS\n");

	return count;

}

static DEVICE_ATTR(set_qt_update, S_IRUGO | S_IWUSR | S_IWGRP, set_qt_update_show, NULL);
static DEVICE_ATTR(set_qt_firm_version, S_IRUGO | S_IWUSR | S_IWGRP, set_qt_firm_version_show, NULL);
static DEVICE_ATTR(set_qt_firm_status, S_IRUGO | S_IWUSR | S_IWGRP, set_qt_firm_status_show, NULL);
static DEVICE_ATTR(set_qt_firm_version_read, S_IRUGO | S_IWUSR | S_IWGRP, set_qt_firm_version_read_show, NULL);
#endif /* CYTSP_FWUPG_ENABLE */


/*------------------------------ I2C Driver block -----------------------------------*/

static int cytouch_i2c_read(u8 reg, u8* data, int len)
{
	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = gp_cytouch_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	if (1 != i2c_transfer(gp_cytouch_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = gp_cytouch_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(gp_cytouch_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	//printk("%s read success : reg(0x%x) data(0x%x)\n", __func__, reg, buf[0]);

	return 0;
}

irqreturn_t cytouch_irq_handler(int irq, void *dev_id)
{
//	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_INPUT);

	disable_irq_nosync(IRQ_TOUCH_INT);

//	queue_work(cytouch_wq, &g_cytouch_work);
	if (!work_pending(&g_cytouch_work))
		schedule_work(&g_cytouch_work);

	return IRQ_HANDLED;
}

static int cytouch_print_ver(void)
{
	u8 buf[4] = {0,};

	CYTSPDBG("[%s] \n", __func__);

	if(	g_suspend_state == TRUE )
	{
		return FALSE;
	}

	if (cytouch_i2c_read(CYTOUCH_REG_VENDOR_ID, buf, 4) >= 0)
	{
		g_vendor_id = buf[0];
		g_module_id = buf[1];
		g_fw_ver = buf[2];
		tsp_version = buf[2];

		printk("%s :Vendor ID : 0x%x, Module ID : 0x%x, FW Ver : 0x%x\n", __func__, buf[0], buf[1], buf[2]);
		return TRUE;
	}
	else
	{
		g_vendor_id = g_module_id = g_fw_ver = 0;
		tsp_version = 0;
		printk("%s : Can't find vendor id, module id, fw ver!\n", __func__);
		printk("%s :Vendor ID : 0x%x, Module ID : 0x%x, FW Ver : 0x%x\n", __func__, buf[1], buf[2], buf[3]);
		return FALSE;
	}
}

static void cytouch_register_irq(void)
{
	int ret;
	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_TOUCH_INT, S3C_GPIO_PULL_UP);

	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW);

	ret = request_irq(IRQ_TOUCH_INT/*IRQ_TOUCH_INT*/, cytouch_irq_handler, IRQF_DISABLED, "qt602240 irq", 0);
	if (ret == 0) {
		CYTSPDBG("[TSP] cytouch_probe: Start touchscreen %s\n", gp_cytouch_input->name);
	}
	else {
		printk("[TSP] request_irq failed\n");
	}
}

void cytouch_set_input_dev(void)
{
	gp_cytouch_input->name = "cytma340_input"; // "qt602240_ts_input";

	set_bit(EV_SYN, gp_cytouch_input->evbit);
	set_bit(EV_KEY, gp_cytouch_input->evbit);
	set_bit(BTN_TOUCH, gp_cytouch_input->keybit);
	set_bit(EV_ABS, gp_cytouch_input->evbit);

	set_bit(TOUCHKEY_MENU, gp_cytouch_input->keybit);
	set_bit(TOUCHKEY_BACK, gp_cytouch_input->keybit);
#ifdef CONFIG_VENTURI_USA
	set_bit(TOUCHKEY_HOME, gp_cytouch_input->keybit);
#endif

	input_set_abs_params(gp_cytouch_input, ABS_X, 0, 479, 0, 0);
	input_set_abs_params(gp_cytouch_input, ABS_Y, 0, 799, 0, 0);
//#ifdef _SUPPORT_MULTITOUCH_
	input_set_abs_params(gp_cytouch_input, ABS_MT_POSITION_X, 0, 479, 0, 0);
	input_set_abs_params(gp_cytouch_input, ABS_MT_POSITION_Y, 0, 799, 0, 0);
//#endif

	input_set_abs_params(gp_cytouch_input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(gp_cytouch_input, ABS_TOOL_WIDTH, 0, 15, 0, 0);
//#ifdef _SUPPORT_MULTITOUCH_
	input_set_abs_params(gp_cytouch_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(gp_cytouch_input, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
//#endif
}

static void cytouch_dwork(struct work_struct *work)
{
#if TOUCH_DVFS_CONTROL
	if (resume_dvfs_lock) {
		s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
		resume_dvfs_lock = false;
	}
#endif
}

int cytouch_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	int i;

	gp_cytouch_client = client;
	gp_cytouch_client->irq = IRQ_TOUCH_INT;



	INIT_WORK(&g_cytouch_work, get_message );

	// allocate Input Device
	gp_cytouch_input = input_allocate_device();
	if (gp_cytouch_input == NULL) {
		ret = -ENOMEM;
		printk(KERN_DEBUG "cytouch_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	cytouch_set_input_dev();

	// register Input Device
	ret = input_register_device(gp_cytouch_input);
	if (ret) {
		printk(KERN_DEBUG "cytouch_probe: Unable to register %s input device\n", gp_cytouch_input->name);
		goto err_input_register_device_failed;
	}

	cytouch_register_irq();

	if (cytouch_print_ver() == FALSE) {
		printk(KERN_DEBUG "\n[TSP][ERROR] Touch device NOT found ...1\n");
		cytouch_upgrade_fw_force();

		if(cytouch_print_ver() == FALSE) {
			printk(KERN_DEBUG "\n[TSP][ERROR] Touch device NOT found ...2\n");
			//return -1;
		}
	} else {
		cytouch_upgrade_fw();
	}

#ifdef CYTSP_TIMER_ENABLE
	cytouch_init_rel_timer();
#endif

#ifdef CYTSP_WDOG_ENABLE
	if(g_fw_ver >= 0x5)
	{
		CYTSPDBG("[TSP] Init wdog\n");
		cytouch_init_wdog();
	}
#endif

	driver_setup = DRIVER_SETUP_OK;

#ifdef USE_TSP_EARLY_SUSPEND
	g_cytouch_early_suspend.level 	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_cytouch_early_suspend.suspend = cytouch_early_suspend;
	g_cytouch_early_suspend.resume 	= cytouch_late_resume;
	register_early_suspend(&g_cytouch_early_suspend);
#endif

	led_power_control(1);

	INIT_DELAYED_WORK(&g_cytouch_dwork, cytouch_dwork);

	return 0;

err_input_register_device_failed:
	input_free_device(gp_cytouch_input);

err_input_dev_alloc_failed:

	return ret;
}


static int cytouch_remove(struct i2c_client *client)
{
#ifdef USE_TSP_EARLY_SUSPEND
	unregister_early_suspend(&g_cytouch_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	free_irq(IRQ_TOUCH_INT, 0);
	input_unregister_device(gp_cytouch_input);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifndef USE_TSP_EARLY_SUSPEND
static int cytouch_suspend(struct platform_device * dev,pm_message_t mesg)
{
	return 0;
}

static int cytouch_resume(struct platform_device *dev)
{
	return 0;
}
#endif

#ifdef USE_TSP_EARLY_SUSPEND
static int cytouch_early_suspend(struct early_suspend *h)
{
	g_suspend_state = TRUE;

	CYTSPDBG("\n[TSP][%s] \n",__func__);

#if TOUCH_DVFS_CONTROL
	cancel_delayed_work_sync(&g_cytouch_dwork);
	s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L0);
	resume_dvfs_lock = true;
#endif

	disable_irq(IRQ_TOUCH_INT);

#ifdef CYTSP_WDOG_ENABLE
	if(g_fw_ver >= 0x5)
		cytouch_pause_wdog();
#endif

	cytouch_release_all(); // release previous pressed key and touch

	/*	We already disabled irq, wdog, touch-timer, menukey-timer, backkey-timer
	 *    But, We should wait until previous i2c operations complete
	 */
	mdelay(20);

	s3c_i2c2_force_stop();	// 2010.12.13, eungchan.kim, fix i2c busy issue when sleep and wakeup, it's testcode

	touch_state_val=0;

	cytouch_hw_set_pwr(CYTOUCH_PWROFF);

#if 0
	/* GPIO Pin Configuration
	*/
	s3c_gpio_cfgpin(GPIO_TOUCH_INT, 		S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TOUCH_INT, 		S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, 		S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V,		S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, 		S3C_GPIO_PULL_DOWN);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V, 		S3C_GPIO_PULL_DOWN);
#endif
#ifdef TOUCHKEY_LED_ENABLE
	touchkey_control(2); // LED OFF
	led_power_control(0);
#endif

#if TOUCH_DVFS_CONTROL
	s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
	resume_dvfs_lock = false;
#endif

	return 0;
}


int cytouch_hw_set_pwr(CYTOUCH_PWRSTAT onoff)
{
	if (onoff != CYTOUCH_PWRON && onoff != CYTOUCH_PWROFF)
	{
		CYTSPDBG("%s : Error! Unknown parameter => %d\n", __func__, onoff);
		return FALSE;
	}

	// Touch On/Off
	s3c_gpio_cfgpin(GPIO_TOUCH_EN, 	S3C_GPIO_OUTPUT/*GPIO_TOUCH_EN_AF*/);
	s3c_gpio_setpull(GPIO_TOUCH_EN, S3C_GPIO_PULL_NONE);
//	s3c_gpio_setpin(GPIO_TOUCH_EN, onoff);
	gpio_set_value(GPIO_TOUCH_EN, onoff);

	return TRUE;
}

static int cytouch_late_resume(struct early_suspend *h)
{
	CYTSPDBG("\n[TSP][%s] \n",__func__);

#if 0 /* TOUCH_DVFS_CONTROL */
	s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L0);
	resume_dvfs_lock = true;
	if (!delayed_work_pending(&g_cytouch_dwork))
		schedule_delayed_work(&g_cytouch_dwork, msecs_to_jiffies(5000));
#endif

#ifdef TOUCHKEY_LED_ENABLE
	led_power_control(1);
#endif
#if 0
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V,		GPIO_TSP_SDA_28V_AF);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V,		GPIO_TSP_SCL_28V_AF);
	s3c_gpio_setpull(GPIO_TSP_SDA_28V,		S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V,		S3C_GPIO_PULL_NONE);
#endif
	init_hw_setting();
	g_suspend_state = FALSE;

	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(0xf));
	enable_irq(IRQ_TOUCH_INT);

#ifdef CYTSP_WDOG_ENABLE
	if(g_fw_ver >= 0x5)
		cytouch_resume_wdog();
#endif
	return 0;
}

#endif	// End of USE_TSP_EARLY_SUSPEND



#ifdef CYTSP_FWUPG_ENABLE
static int cytouch_upgrade_fw(void)
{
	if( (g_vendor_id | g_module_id | g_fw_ver) != 0 )
	{
		if(g_vendor_id != cytma340_new_vendor_id)
		{
			CYTSPDBG("vendor is invalid (%d, %d)\n", g_vendor_id, cytma340_new_vendor_id);
			return -1;
		}

		if(g_module_id != cytma340_new_module_id)
		{
			CYTSPDBG("module_id is invalid (%d, %d)\n", g_module_id, cytma340_new_module_id);
			return -1;
		}

		if(g_fw_ver >= cytma340_new_fw_ver)
		{
			CYTSPDBG("fw_ver is the latest version	(%d, %d)\n", g_fw_ver, cytma340_new_fw_ver);
			return 0;
		}
	}

	CYTSPDBG("%s : Firmware Upgrade (%d -> %d)\n", __func__, g_fw_ver, cytma340_new_fw_ver);

	// clear i2c gpio setting
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_INPUT);

	// disable i2c gpio pull-up
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V, S3C_GPIO_PULL_NONE);

	if (driver_setup == DRIVER_SETUP_OK)
		tma340_frimware_update();
	else {
		CYPRESS_DISABLE_WATCHDOG_TIMER_RESET();
		tma340_frimware_update();
		CYPRESS_ROLLBACK_WATCHDOG_TIMER_RESET();
	}

	/* update version info.. */
	g_vendor_id = cytma340_new_vendor_id;
	g_module_id = cytma340_new_module_id;
	g_fw_ver = cytma340_new_fw_ver;
	tsp_version = g_fw_ver;

	/* reset */
	cytouch_hw_set_pwr(CYTOUCH_PWROFF);
	mdelay(100);

	// set i2c gpio setting
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V, S3C_GPIO_PULL_NONE);

	cytouch_hw_set_pwr(CYTOUCH_PWRON);
	msleep(1000); // mdelay(400);
	msleep(1000);

	// minhyodebug
#if 0
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, 		S3C_GPIO_OUTPUT);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V_REV03,	S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V_REV03, S3C_GPIO_PULL_UP);
#endif

	return 0;
}

static int cytouch_upgrade_fw_force(void)
{
	CYTSPDBG("\n================== cytouch_upgrade_fw_force\n");


	// clear i2c gpio setting
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_INPUT);

	// disable i2c gpio pull-up
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V, S3C_GPIO_PULL_NONE);

	if (driver_setup == DRIVER_SETUP_OK)
		tma340_frimware_update();
	else {
		CYPRESS_DISABLE_WATCHDOG_TIMER_RESET();
		tma340_frimware_update();
		CYPRESS_ROLLBACK_WATCHDOG_TIMER_RESET();
	}

	/* update version info.. */
	g_vendor_id = cytma340_new_vendor_id;
	g_module_id = cytma340_new_module_id;
	g_fw_ver = cytma340_new_fw_ver;
	tsp_version = g_fw_ver;

	/* reset */
	cytouch_hw_set_pwr(CYTOUCH_PWROFF);
	mdelay(100);

	// set i2c gpio setting
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V, S3C_GPIO_PULL_NONE);

	cytouch_hw_set_pwr(CYTOUCH_PWRON);
	msleep(1000); // mdelay(400);
	msleep(1000);

	// minhyodebug
#if 0
	s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, 		S3C_GPIO_OUTPUT);
	s3c_gpio_cfgpin(GPIO_TSP_SCL_28V_REV03,	S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_SDA_28V, S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(GPIO_TSP_SCL_28V_REV03, S3C_GPIO_PULL_UP);
#endif

	return 0;
}



#endif


#ifdef TOUCHKEY_LED_ENABLE
static int touchkey_control(int data)
{
	if(data == 1)
	{
		// ON
		CYTSPDBG("[%s] LED Enable\n", __func__);
	}
	else if(data == 2)
	{
		// OFF
		data = 0;
		CYTSPDBG("[%s] LED Disable\n", __func__);

	}
	else
	{
		// Error
		printk("[%s] ERROR : parameter value = %d\n", __func__, data);
		return -1;
	}

	if (gpio_is_valid(GPIO_LED1_EN))
	{
			if (gpio_request(GPIO_LED1_EN, "GPH2"))
					printk("Failed to request GPIO_LED1_EN!\n");

			s3c_gpio_cfgpin(GPIO_LED1_EN, S3C_GPIO_OUTPUT);
			gpio_direction_output(GPIO_LED1_EN, (int)data);
	}
	s3c_gpio_setpull(GPIO_LED1_EN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_LED1_EN);

	if (gpio_is_valid(GPIO_LED2_EN))
	{
			if (gpio_request(GPIO_LED2_EN, "GPH2"))
					printk("Failed to request GPIO_LED1_EN!\n");

			s3c_gpio_cfgpin(GPIO_LED2_EN, S3C_GPIO_OUTPUT);
			gpio_direction_output(GPIO_LED2_EN, (int)data);
	}
	s3c_gpio_setpull(GPIO_LED2_EN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_LED2_EN);

	return 1;

}
static ssize_t touchkey_led_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int data;

	sscanf(buf, "%d", &data);

	touchkey_control(data);

	return size;
}

static DEVICE_ATTR(touchkey_led, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touchkey_led_store);
#endif

static ssize_t firmware1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	CYTSPDBG("[TSP] Cypress Firmware Ver. %x:%x:%x\n", g_vendor_id, g_module_id, g_fw_ver);

	return sprintf(buf, "%x:%x:%x\n", g_vendor_id, g_module_id, g_fw_ver);
}

static ssize_t firmware1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	CYTSPDBG("[TSP] %s - operate nothing\n", __func__);

	return size;
}

static DEVICE_ATTR(firmware1, S_IRUGO | S_IWUSR | S_IWGRP, firmware1_show, firmware1_store);


static struct i2c_device_id cytouch_idtable[] = {
	{ "cytma340", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cytouch_idtable);

struct i2c_driver cytouch_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= "cytma340",
	},
	.id_table	= cytouch_idtable,
	.probe		= cytouch_probe,
	.remove		= __devexit_p(cytouch_remove),
#ifndef USE_TSP_EARLY_SUSPEND
	.suspend	= cytouch_suspend,
	.resume		= cytouch_resume,
#endif //USE_TSP_EARLY_SUSPEND
};

void init_hw_setting(void)
{
	s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TOUCH_INT, S3C_GPIO_PULL_UP);

	if (gpio_is_valid(GPIO_TOUCH_EN)) {
		if (gpio_request(GPIO_TOUCH_EN, "GPG3"))
			printk(KERN_DEBUG "Failed to request GPIO_TOUCH_EN!\n");
		gpio_direction_output(GPIO_TOUCH_EN, 1);
	}

	s3c_gpio_setpull(GPIO_TOUCH_EN, S3C_GPIO_PULL_NONE);
	gpio_free(GPIO_TOUCH_EN);

	CYTSPDBG("cytouch GPIO Status\n");
	CYTSPDBG("TOUCH_EN  : %s\n", gpio_get_value(GPIO_TOUCH_EN)? "High":"Low");
	CYTSPDBG("TOUCH_INT : %s\n", gpio_get_value(GPIO_TOUCH_INT)? "High":"Low");

	msleep(400);  // need 400ms delay after TSP IC Power On.
}

extern struct class *sec_class;
struct device *ts_dev;

/*****************************************************************************
*
*  FUNCTION
*  PURPOSE
*  INPUT
*  OUTPUT
*
* ***************************************************************************/


int __init cytouch_init(void)
{
	int ret;

	DEBUG;

	cytouch_wq = create_singlethread_workqueue("cytouch_wq");
	if (!cytouch_wq)
		return -ENOMEM;

	init_hw_setting();

	gpio_pend_mask_mem = ioremap(INT_PEND_BASE, 0x10);

	ret = i2c_add_driver(&cytouch_i2c_driver);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	CYTSPDBG("ret : %d, gp_cytouch_client name : %s\n",ret,gp_cytouch_client->name);

	if(!(gp_cytouch_client)){
		printk("[%s],slave address changed try to firmware reprogram \n",__func__);
		i2c_del_driver(&cytouch_i2c_driver);

		ret = i2c_add_driver(&cytouch_i2c_driver);
		if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);
		printk("ret : %d, gp_cytouch_client name : %s\n",ret,gp_cytouch_client->name);

		if(gp_cytouch_client){
			i2c_del_driver(&cytouch_i2c_driver);

			ret = i2c_add_driver(&cytouch_i2c_driver);
			if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);
			printk("ret : %d, gp_cytouch_client name : %s\n",ret,gp_cytouch_client->name);
		}
	}

	if(!(gp_cytouch_client)){
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&cytouch_i2c_driver);
		return 0;
	}

	if (sec_class == NULL)
		sec_class = class_create(THIS_MODULE, "sec");

        if (IS_ERR(sec_class))
                pr_err("Failed to create class(sec)!\n");

	ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(ts)!\n");

	if (device_create_file(ts_dev, &dev_attr_key_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_threshold.attr.name);

#ifdef TOUCHKEY_LED_ENABLE
	if (device_create_file(ts_dev, &dev_attr_touchkey_led) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_led.attr.name);
#endif

	if (device_create_file(ts_dev, &dev_attr_firmware1) < 0)
	{
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware1.attr.name);
	}

	/*------------------------------	 AT COMMAND TEST 		---------------------*/
#ifdef CYTSP_FWUPG_ENABLE
	cytouch_atcom_test = device_create(sec_class, NULL, 0, NULL, "qt602240_atcom_test");
	if (IS_ERR(cytouch_atcom_test))
		printk("Failed to create device(qt602240_atcom_test)!\n");

	if (device_create_file(cytouch_atcom_test, &dev_attr_set_qt_update)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_update.attr.name);
	if (device_create_file(cytouch_atcom_test, &dev_attr_set_qt_firm_version)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version.attr.name);
	if (device_create_file(cytouch_atcom_test, &dev_attr_set_qt_firm_status)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_status.attr.name);
	if (device_create_file(cytouch_atcom_test, &dev_attr_set_qt_firm_version_read)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version_read.attr.name);
#endif
	/*------------------------------	 AT COMMAND TEST 		---------------------*/

	return 0;
}

void __exit cytouch_exit(void)
{
	i2c_del_driver(&cytouch_i2c_driver);
	if (cytouch_wq)
		destroy_workqueue(cytouch_wq);
}
late_initcall(cytouch_init);
module_exit(cytouch_exit);

MODULE_DESCRIPTION("Quantum Touchscreen Driver");
MODULE_LICENSE("GPL");

#ifdef CYTSP_FWUPG_ENABLE

#define TX_ON

#ifndef UInt32
#define UInt32 u32
#endif

#ifndef UInt16
#define UInt16 u16
#endif

#ifndef UInt8
#define UInt8 u8
#endif

//////////////////////////////////////////// VENTURI WORKING //////////////////////////////////////////////////////


#define I2C_GPIO_SDA	GPIO_TSP_SDA_28V
#define I2C_GPIO_SCL	GPIO_TSP_SCL_28V 		// venturi rev03 GPIO_TSP_SCL_28V

#define I2C_GPIO_IRQ	GPIO_TOUCH_INT

#define I2C_SET_SCL_GPIO() 		do { s3c_gpio_cfgpin(I2C_GPIO_SCL, S3C_GPIO_INPUT); } while(0)		// gpio input
#define I2C_CLR_SCL_GPIO() 		do { s3c_gpio_cfgpin(I2C_GPIO_SCL, S3C_GPIO_OUTPUT); } while(0)	// gpio output
#define I2C_SET_SCL_GPIO_LOW() 	do { gpio_set_value(I2C_GPIO_SCL, GPIO_LEVEL_LOW); } while(0)		// gpio output low
#define I2C_SET_SCL_GPIO_HIGH() do { gpio_set_value(I2C_GPIO_SCL, GPIO_LEVEL_HIGH); } while(0)		// gpio output high

#define I2C_SET_SDA_GPIO() 		do { s3c_gpio_cfgpin(I2C_GPIO_SDA, S3C_GPIO_INPUT); } while(0)		// gpio input
#define I2C_CLR_SDA_GPIO() 		do { s3c_gpio_cfgpin(I2C_GPIO_SDA, S3C_GPIO_OUTPUT); } while(0)	// gpio output
#define I2C_SET_SDA_GPIO_LOW() 	do { gpio_set_value(I2C_GPIO_SDA, GPIO_LEVEL_LOW); } while(0)		// gpio output low
#define I2C_SET_SDA_GPIO_HIGH() do { gpio_set_value(I2C_GPIO_SDA, GPIO_LEVEL_HIGH); } while(0)		// gpio output high

#define I2C_READ_SDA_GPIO() 	gpio_get_value(I2C_GPIO_SDA)
#define I2C_READ_SCL_GPIO() 	gpio_get_value(I2C_GPIO_SCL)


#define TARGET_DATABUFF_LEN		128

#define NUM_BANKS				1
#define BLOCKS_PER_BANK			256
#define SECURITY_BYTES_PER_BANK	64

// The following are defines for error messages from the ISSP program.
#define PASS           0
// PASS is used to indicate that a function completed successfully.
#define ERROR         -1
// ERROR is a generic failure used within lower level functions before the
// error is reported.  This should not be seen as an error that is reported
// from main.
#define INIT_ERROR     1
// INIT_ERROR means a step in chip initialization failed.
#define SiID_ERROR     2
// SiID_ERROR means that the Silicon ID check failed. This happens if the
// target part does not match the device type that the ISSP program is
// configured for.
#define ERASE_ERROR    3
// ERASE_ERROR means that the bulk erase step failed.
#define BLOCK_ERROR    4
// BLOCK_ERROR means that a step in programming a Flash block or the verify
// of the block failed.
#define VERIFY_ERROR   5
// VERIFY_ERROR means that the checksum verification failed.
#define SECURITY_ERROR 6
// SECURITY_ERROR means that the write of the security information failed.
#define STATUS_ERROR 7

#define CHECKSUM_ERROR 8


#define DELAY_M    1
#define DELAY_B    3
#define TRANSITION_TIMEOUT     (65535*10)
#define XRES_CLK_DELAY    ((63 - DELAY_B) / DELAY_M)
#define POWER_CYCLE_DELAY ((150 - DELAY_B) / DELAY_M)
#define DELAY100us        ((100 - DELAY_B) / DELAY_M)

unsigned char target_id_v[] = {0x05, 0x96};     //ID for CY8CTMA340_36LQXI

const unsigned int num_bits_checksum = 418;
const unsigned char checksum_v[] =
{
	0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF4, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x00, 0x7D, 0xE0,
    0x0F, 0xF7, 0xC0, 0x07, 0xDF, 0x28, 0x1F, 0x7D, 0x18, 0x7D,
    0xFE, 0x25, 0xC0
};

const unsigned char read_status[] =
{
	0xBF, 0x00, 0x80
};


const unsigned int num_bits_id_setup_1 = 616;
const unsigned char id_setup_1[] =
{
	0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0xEE, 0x21, 0xF7,
    0xF0, 0x27, 0xDC, 0x40,	0x9F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xE7, 0xC1,	0xD7, 0x9F, 0x20, 0x7E, 0x3F, 0x9D, 0x78, 0xF6,
	0x21, 0xF7, 0xB8, 0x87, 0xDF, 0xC0, 0x1F, 0x71,	0x00, 0x7D,
	0xC0, 0x07, 0xF7, 0xB8, 0x07, 0xDE,	0x80, 0x7F, 0x7A, 0x80,
	0x7D, 0xEC, 0x01, 0xF7,	0x80, 0x4F, 0xDF, 0x00, 0x1F, 0x7C,
	0xA0, 0x7D,	0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_id_setup_2 = 418;
const unsigned char id_setup_2[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF4, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x00, 0x7D, 0xE0,
    0x0D, 0xF7, 0xC0, 0x07, 0xDF, 0x28, 0x1F, 0x7D, 0x18, 0x7D,
    0xFE, 0x25, 0xC0
};

const unsigned int num_bits_tsync_enable = 110;
const unsigned char tsync_enable[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0xDE, 0xE0, 0x1C
};
const unsigned int num_bits_tsync_disable = 110;
const unsigned char tsync_disable[] =
{
    0xDE, 0xE2, 0x1F, 0x71, 0x00, 0x7D, 0xFC, 0x01, 0xF7, 0x00,
    0x1F, 0xDE, 0xE0, 0x1C
};


#if 0
const unsigned int num_bits_set_block_num = 33;
const unsigned char set_block_num[] =
{
    0xDE, 0xE0, 0x1E, 0x7D, 0x00, 0x70
};
#else
const unsigned int num_bits_set_block_num = 11;
const unsigned char set_block_num[] =
{
    0x9F, 0x40
};
#endif

const unsigned int num_bits_set_block_num_end = 3;		//PTJ: this selects the first three bits of set_block_num_end
const unsigned char set_block_num_end = 0xE0;

const unsigned int num_bits_read_write_setup = 66;		//PTJ:
const unsigned char read_write_setup[] =
{
    0xDE, 0xF0, 0x1F, 0x78, 0x00, 0x7D, 0xA0, 0x03, 0xC0
};

const unsigned int num_bits_my_verify_setup = 440;
const unsigned char verify_setup[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA8, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x0F, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_erase = 396;		//PTJ: erase with TSYNC Enable and Disable
const unsigned char erase[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x85, 0xFD, 0xFC, 0x01, 0xF7,
    0x10, 0x07, 0xDC, 0x00, 0x7F, 0x7B, 0x80, 0x7D, 0xE0, 0x0B,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x04, 0x7D, 0xF0,
    0x01, 0xF7, 0xC9, 0x87, 0xDF, 0x48, 0x1F, 0x7F, 0x89, 0x70
};

const unsigned int num_bits_secure = 440;		//PTJ: secure with TSYNC Enable and Disable
const unsigned char secure[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA0, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x27, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_program_and_verify = 440;		//PTJ: length of program_block[], not including zero padding at end
const unsigned char program_and_verify[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA0, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x57, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned char read_id_v[] =
{
    0xBF, 0x00, 0xDF, 0x90, 0x00, 0xFE, 0x60, 0xFF, 0x00
};

const unsigned char    write_byte_start = 0x90;			//PTJ: this is set to SRAM 0x80
const unsigned char    write_byte_end = 0xE0;

const unsigned char    num_bits_wait_and_poll_end = 40;
const unsigned char    wait_and_poll_end[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char read_checksum_v[] =
{
    0xBF, 0x20, 0xDF, 0x80, 0x80
};


const unsigned char read_byte_v[] =
{
    0xB0, 0x80
};


#ifdef TX_ON

void UART_PutChar(char ch)
{
	printk("%c", ch);
}

void UART_PutString(char* str)
{
	printk("%s\n", str);
}

#define UART_PutCRLF(x)	do { ; } while(0)

void UART_PutHexHalf(char ch)
{
    if(ch >=10)
        UART_PutChar(ch + 'A'-10);
    else
        UART_PutChar(ch + '0');
}

void UART_PutHexByte(unsigned char ch)
{
    UART_PutHexHalf(ch >> 4);
    UART_PutHexHalf(ch & 0x0f);
}

void UART_PutHexWord(unsigned int ch)
{
    UART_PutHexByte(ch>>8);
    UART_PutHexByte(ch&0xff);
}

#endif


/***********************
*
* Touchpad Tuning APIs
*
************************/

void TchDrv_DownloadVddSetHigh(void)
{
   	s3c_gpio_cfgpin(GPIO_TOUCH_EN, 	S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TOUCH_EN, S3C_GPIO_PULL_NONE);
//	s3c_gpio_setpin(GPIO_TOUCH_EN, 1);
	gpio_set_value(GPIO_TOUCH_EN, 1);
}

void TchDrv_DownloadVddSetLow(void)
{
   	s3c_gpio_cfgpin(GPIO_TOUCH_EN, 	S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TOUCH_EN, S3C_GPIO_PULL_NONE);
//	s3c_gpio_setpin(GPIO_TOUCH_EN, 0);
	gpio_set_value(GPIO_TOUCH_EN, 0);
}

void TchDrv_DownloadIntSetHigh(void)
{
	gpio_set_value(I2C_GPIO_IRQ, GPIO_LEVEL_HIGH);
}

void TchDrv_DownloadIntSetLow(void)
{
	gpio_set_value(I2C_GPIO_IRQ, GPIO_LEVEL_LOW);
}

void TchDrv_DownloadIntSetOutput(void)
{
	s3c_gpio_cfgpin(I2C_GPIO_IRQ, S3C_GPIO_OUTPUT);
}

void TchDrv_DownloadIntSetInput(void)
{
	s3c_gpio_cfgpin(I2C_GPIO_IRQ, S3C_GPIO_INPUT);
}

void TchDrv_DownloadDisableIRQ(void)
{
	disable_irq(I2C_GPIO_IRQ);
}

void TchDrv_DownloadEnableIRQ(void)
{
	enable_irq(I2C_GPIO_IRQ);
}

void TchDrv_DownloadDisableWD(void)
{
	/* null */
}

void TchDrv_DownloadEnableWD(void)
{
	/* null */
}

#if 0
// provides delays in uS
static void Delay1us(void)
{
	udelay(1);
}
#endif

static void Delay10us(UInt32 uSdelay)
{
	udelay(uSdelay);
}

void OSTASK_Sleep(int delay)
{
	mdelay(delay);
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_driver_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
// WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Cypress reserves the right to make changes without further notice to the
// materials described herein. Cypress does not assume any liability arising
// out of the application or use of any product or circuit described herein.
// Cypress does not authorize its products for use as critical components in
// life-support systems where a malfunction or failure may reasonably be
// expected to result in significant injury to the user. The inclusion of
// Cypress?product in a life-support systems application implies that the
// manufacturer assumes all risk of such use and in doing so indemnifies
// Cypress against all charges.
//
// Use may be limited by and subject to the applicable Cypress software
// license agreement.
//
//--------------------------------------------------------------------------

#define SECURITY_DATA	0xFF

unsigned char    bTargetDataPtr;
unsigned char    abTargetDataOUT[TARGET_DATABUFF_LEN];
//unsigned char    firmData[512][64];


// ****************************** PORT BIT MASKS ******************************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
#define SDATA_PIN   0x80        // P1.7
#define SCLK_PIN    0x40        // P1.6
#define XRES_PIN    0x40        // P2.6
#define TARGET_VDD  0x08        // P2.3









// ((((((((((((((((((((((( DEMO ISSP SUBROUTINE SECTION )))))))))))))))))))))))
// ((((( Demo Routines can be deleted in final ISSP project if not used   )))))
// ((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))))

// ============================================================================
// InitTargetTestData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Loads a 64-Byte array to use as test data to program target. Ultimately,
// this data should be fed to the Host by some other means, ie: I2C, RS232,
// etc. Data should be derived from hex file.
//  Global variables affected:
//    bTargetDataPtr
//    abTargetDataOUT
// ============================================================================
void InitTargetTestData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // create unique data for each block
    for (bTargetDataPtr = 0; bTargetDataPtr < TARGET_DATABUFF_LEN; bTargetDataPtr++)
    {
        abTargetDataOUT[bTargetDataPtr] = 0x55;
    }
}


// ============================================================================
// LoadArrayWithSecurityData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Most likely this data will be fed to the Host by some other means, ie: I2C,
// RS232, etc., or will be fixed in the host. The security data should come
// from the hex file.
//   bStart  - the starting byte in the array for loading data
//   bLength - the number of byte to write into the array
//   bType   - the security data to write over the range defined by bStart and
//             bLength
// ============================================================================
void LoadArrayWithSecurityData(unsigned char bStart, unsigned char bLength, unsigned char bType)
{
    // Now, write the desired security-bytes for the range specified
    for (bTargetDataPtr = bStart; bTargetDataPtr < bLength; bTargetDataPtr++) {
        abTargetDataOUT[bTargetDataPtr] = bType;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// Delay()
// This delay uses a simple "nop" loop. With the CPU running at 24MHz, each
// pass of the loop is about 1 usec plus an overhead of about 3 usec.
//      total delay = (n + 3) * 1 usec
// To adjust delays and to adapt delays when porting this application, see the
// ISSP_Delays.h file.
// ****************************************************************************
void Delay(unsigned int n)
{
    while(n)
    {
        //asm("nop");
        //_nop_();

        n -= 1;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// LoadProgramData()
// The final application should load program data from HEX file generated by
// PSoC Designer into a 64 byte host ram buffer.
//    1. Read data from next line in hex file into ram buffer. One record
//      (line) is 64 bytes of data.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread data from file or abort
//    4. Exit this Function and Program block or verify the block.
// This demo program will, instead, load predetermined data into each block.
// The demo does it this way because there is no comm link to get data.
// ****************************************************************************
void LoadProgramData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function InitTargetTestData fills buffer for demo
    InitTargetTestData(bBlockNum, bBankNum);

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns VOID.
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fLoadSecurityData()
// Load security data from hex file into 64 byte host ram buffer. In a fully
// functional program (not a demo) this routine should do the following:
//    1. Read data from security record in hex file into ram buffer.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread security data from file or abort
//    4. Exit this Function and Program block
// In this demo routine, all of the security data is set to unprotected (0x00)
// and it returns.
// This function always returns PASS. The flag return is reserving
// functionality for non-demo versions.
// ****************************************************************************
signed char fLoadSecurityData(unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function LoadArrayWithSecurityData fills buffer for demo
//    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0x00);
    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, SECURITY_DATA);		//PTJ: 0x1B (00 01 10 11) is more interesting security data than 0x00 for testing purposes

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns PASS.
    return(PASS);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fSDATACheck()
// Check SDATA pin for high or low logic level and return value to calling
// routine.
// Returns:
//     0 if the pin was low.
//     1 if the pin was high.
// ****************************************************************************
unsigned char fSDATACheck(void)
{
#if 0
    //if(PRT1DR & SDATA_PIN)
    if (SDATA_Read())
        return(1);
    else
        return(0);
#endif
	//I2C_SET_SDA_GPIO();	//gpio input

	if (I2C_READ_SDA_GPIO())
        return(1);
    else
        return(0);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKHigh()
// Set the SCLK pin High
// ****************************************************************************
void SCLKHigh(void)
{
#if 0
    //PRT1DR |= SCLK_PIN;
    SCLK_Write(1);
#endif
	I2C_CLR_SCL_GPIO();		//gpio output
	I2C_SET_SCL_GPIO_HIGH();//gpio output high
	Delay10us(1);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKLow()
// Make Clock pin Low
// ****************************************************************************
void SCLKLow(void)
{
#if 0
    //PRT1DR &= ~SCLK_PIN;
    SCLK_Write(0);
#endif
	I2C_CLR_SCL_GPIO(); 	//gpio output
	I2C_SET_SCL_GPIO_LOW();	//gpio output low
	Delay10us(1);
}

#ifndef RESET_MODE  // Only needed for power cycle mode
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKHiZ()
// Set SCLK pin to HighZ drive mode.
// ****************************************************************************
void SetSCLKHiZ(void)
{
#if 0
    //PRT1DM0 &= ~SCLK_PIN;
    //PRT1DM1 |=  SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    SCLK_SetDriveMode(SCLK_DM_DIG_HIZ);
#endif
	I2C_SET_SCL_GPIO();	//gpio input
}
#endif

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKStrong()
// Set SCLK to an output (Strong drive mode)
// ****************************************************************************
void SetSCLKStrong(void)
{
#if 0
    //PRT1DM0 |=  SCLK_PIN;
    //PRT1DM1 &= ~SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    SCLK_SetDriveMode(SCLK_DM_STRONG);
#endif
	I2C_CLR_SCL_GPIO(); 	//gpio output
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHigh()
// Make SDATA pin High
// ****************************************************************************
void SetSDATAHigh(void)
{
#if 0
    //PRT1DR |= SDATA_PIN;
    SDATA_Write(1);
#endif
	I2C_CLR_SDA_GPIO(); 	//gpio output
	I2C_SET_SDA_GPIO_HIGH();//gpio output high
	Delay10us(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATALow()
// Make SDATA pin Low
// ****************************************************************************
void SetSDATALow(void)
{
#if 0
    //PRT1DR &= ~SDATA_PIN;
    SDATA_Write(0);
#endif
	I2C_CLR_SDA_GPIO(); 	//gpio output
	I2C_SET_SDA_GPIO_LOW();	//gpio output low
	Delay10us(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHiZ()
// Set SDATA pin to an input (HighZ drive mode).
// ****************************************************************************
void SetSDATAHiZ(void)
{
#if 0
    //PRT1DM0 &= ~SDATA_PIN;
    //PRT1DM1 |=  SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    SDATA_SetDriveMode(SDATA_DM_DIG_HIZ);
#endif
	I2C_SET_SDA_GPIO(); //gpio input
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAStrong()
// Set SDATA for transmission (Strong drive mode) -- as opposed to being set to
// High Z for receiving data.
// ****************************************************************************
void SetSDATAStrong(void)
{
#if 0
    //PRT1DM0 |=  SDATA_PIN;
    //PRT1DM1 &= ~SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    SDATA_SetDriveMode(SDATA_DM_STRONG);
#endif
	I2C_CLR_SDA_GPIO(); 	//gpio output
}

#ifdef RESET_MODE
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetXRESStrong()
// Set external reset (XRES) to an output (Strong drive mode).
// ****************************************************************************
void SetXRESStrong(void)
{
    PRT2DM0 |=  XRES_PIN;
    PRT2DM1 &= ~XRES_PIN;
    PRT2DM2 &= ~XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// AssertXRES()
// Set XRES pin High
// ****************************************************************************
void AssertXRES(void)
{
    PRT2DR |= XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// DeassertXRES()
// Set XRES pin low.
// ****************************************************************************
void DeassertXRES(void)
{
    PRT2DR &= ~XRES_PIN;
}
#else

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetTargetVDDStrong()
// Set VDD pin (PWR) to an output (Strong drive mode).
// ****************************************************************************
void SetTargetVDDStrong(void)
{
#if 0
    //PRT2DM0 |=  TARGET_VDD;
    //PRT2DM1 &= ~TARGET_VDD;
    //PRT2DM2 &= ~TARGET_VDD;
#endif
	TchDrv_DownloadVddSetLow();
//	OsSleep(200);
	OSTASK_Sleep(200);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// ApplyTargetVDD()
// Provide power to the target PSoC's Vdd pin through a GPIO.
// ****************************************************************************
void ApplyTargetVDD(void)
{
#if 0
    //PRT2DR |= TARGET_VDD;
    Vdd_Write(1);
#endif
	TchDrv_DownloadVddSetHigh();
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// RemoveTargetVDD()
// Remove power from the target PSoC's Vdd pin.
// ****************************************************************************
void RemoveTargetVDD(void)
{
#if 0
    //PRT2DR &= ~TARGET_VDD;
    Vdd_Write(0);
#endif
}
#endif
//end of file ISSP_Drive_Routines.c



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
// WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Cypress reserves the right to make changes without further notice to the
// materials described herein. Cypress does not assume any liability arising
// out of the application or use of any product or circuit described herein.
// Cypress does not authorize its products for use as critical components in
// life-support systems where a malfunction or failure may reasonably be
// expected to result in significant injury to the user. The inclusion of
// Cypress?product in a life-support systems application implies that the
// manufacturer assumes all risk of such use and in doing so indemnifies
// Cypress against all charges.
//
// Use may be limited by and subject to the applicable Cypress software
// license agreement.
//
//--------------------------------------------------------------------------

//#include <m8c.h>        // part specific constants and macros
//#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
//#include "ISSP_Defs.h"
//#include "ISSP_Vectors.h"
//#include "ISSP_Extern.h"
//#include "ISSP_Errors.h"
//#include "ISSP_Directives.h"
//#include "ISSP_Delays.h"
//#include "Device.h"

#define PROGRAM_DATA	0x11

unsigned char  bTargetDataIN;
unsigned char  abTargetDataOUT_secure[TARGET_DATABUFF_LEN] ={0x00,};

unsigned char  bTargetAddress;
unsigned char  bTargetDataPtr = 0;
unsigned char  bTargetID[10];
unsigned char  bTargetStatus; // bTargetStatus[10];			//PTJ: created to support READ-STATUS in fReadStatus()

unsigned char  fIsError = 0;

/* ((((((((((((((((((((( LOW-LEVEL ISSP SUBROUTINE SECTION ))))))))))))))))))))
   (( The subroutines in this section use functions from the C file          ))
   (( ISSP_Drive_Routines.c. The functions in that file interface to the     ))
   (( processor specific hardware. So, these functions should work as is, if ))
   (( the routines in ISSP_Drive_Routines.c are correctly converted.         ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

// ============================================================================
// RunClock()
// Description:
// Run Clock without sending/receiving bits. Use this when transitioning from
// write to read and read to write "num_cycles" is number of SCLK cycles, not
// number of counter cycles.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// ============================================================================
void RunClock(unsigned int iNumCycles)
{
    int i;

    for(i=0; i < iNumCycles; i++)
    {
        SCLKLow();
        SCLKHigh();
    }
}

// ============================================================================
// bReceiveBit()
// Clocks the SCLK pin (high-low-high) and reads the status of the SDATA pin
// after the rising edge.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if SDATA was low
//     1 if SDATA was high
// ============================================================================
unsigned char bReceiveBit(void)
{
    SCLKLow();
    SCLKHigh();
    if (fSDATACheck()) {
        return(1);
    }
    else {
        return(0);
    }
}

// ============================================================================
// bReceiveByte()
// Calls ReceiveBit 8 times to receive one byte.
// Returns:
//     The 8-bit values recieved.
// ============================================================================
unsigned char bReceiveByte(void)
{
    unsigned char b;
    unsigned char bCurrByte = 0x00;

    for (b=0; b<8; b++) {
        bCurrByte = (bCurrByte<<1) + bReceiveBit();
    }
    return(bCurrByte);
}


// ============================================================================
// SendByte()
// This routine sends up to one byte of a vector, one bit at a time.
//    bCurrByte   the byte that contains the bits to be sent.
//    bSize       the number of bits to be sent. Valid values are 1 to 8.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// There is no returned value.
// ============================================================================
void SendByte(unsigned char bCurrByte, unsigned char bSize)
{
    unsigned char b = 0;

    for(b=0; b<bSize; b++)
    {
        if (bCurrByte & 0x80)
        {
            // Send a '1'
            SetSDATAHigh();
            SCLKHigh();
            SCLKLow();
        }
        else
        {
            // Send a '0'
            SetSDATALow();
            SCLKHigh();
            SCLKLow();
        }
        bCurrByte = bCurrByte << 1;
    }
}

// ============================================================================
// SendVector()
// This routine sends the vector specifed. All vectors constant strings found
// in ISSP_Vectors.h.  The data line is returned to HiZ after the vector is
// sent.
//    bVect      a pointer to the vector to be sent.
//    nNumBits   the number of bits to be sent.
//    bCurrByte  scratch var to keep the byte to be sent.
//
// There is no returned value.
// ============================================================================
void SendVector(const unsigned char* bVect, unsigned int iNumBits)
{
    SetSDATAStrong();
    while(iNumBits > 0)
    {
        if (iNumBits >= 8) {
            SendByte(*(bVect), 8);
            iNumBits -= 8;
            bVect++;
        }
        else {
            SendByte(*(bVect), iNumBits);
            iNumBits = 0;
        }
    }
    SetSDATAHiZ();
}


// ============================================================================
// fDetectHiLoTransition()
// Waits for transition from SDATA = 1 to SDATA = 0.  Has a 100 msec timeout.
// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for
// a high-to-low transition. This is used in the polling loop of
// fDetectHiLoTransition(). The timing of the while(1) loops can be calculated
// and the number of loops is counted, using iTimer, to determine when 100
// msec has passed.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if successful
//    -1 if timed out.
// ============================================================================
signed char fDetectHiLoTransition(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = TRANSITION_TIMEOUT;
    while(1)
    {
        SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    return (PASS);
}

signed char fDetectHiLoTransition_2(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = TRANSITION_TIMEOUT;
    while(1)
    {
        //SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        //SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        //SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        //SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    return (PASS);
}


/* ((((((((((((((((((((( HIGH-LEVEL ISSP ROUTINE SECTION ))))))))))))))))))))))
   (( These functions are mostly made of calls to the low level routines     ))
   (( above.  This should isolate the processor-specific changes so that     ))
   (( these routines do not need to be modified.                             ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

#ifdef RESET_MODE
// ============================================================================
// fXRESInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fXRESInitializeTargetForISSP(void)
{
    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();
    SetXRESStrong();

    // Cycle reset and put the device in programming mode when it exits reset
    AssertXRES();
    Delay(XRES_CLK_DELAY);
    DeassertXRES();

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    //PTJ: Send id_setup_1 instead of init1_v
    //PTJ: both send CA Test Key and do a Calibrate1 SROM function
    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    // NOTE: DO NOT not wait for HiLo on SDATA after vector Init-3
    //       it does not occur (per spec).
    return(PASS);
}

#else  //else = the part is power cycle programmed

// ============================================================================
// fPowerCycleInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// The first time fDetectHiLoTransition is called the Clk pin is highZ because
// the clock is not needed during acquire.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fPowerCycleInitializeTargetForISSP(void)
{
//    unsigned char n;

    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();

    // Turn on power to the target device before other signals
    SetTargetVDDStrong();
    ApplyTargetVDD();
    // wait 1msec for the power to stabilize

//    for (n=0; n<10; n++) {
//        Delay(DELAY100us);
//    }

	//OsSleep(1);
	OSTASK_Sleep(1);

    // Set SCLK to high Z so there is no clock and wait for a high to low
    // transition on SDAT. SCLK is not needed this time.
    SetSCLKHiZ();
//    if (fIsError = fDetectHiLoTransition_2()) {
    if ( (fIsError = fDetectHiLoTransition()) ) {
        return(INIT_ERROR);
    }

    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();					//PTJ: DO NOT SET A BREAKPOINT HERE AND EXPECT SILICON ID TO PASS!

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    SendVector(id_setup_1, num_bits_id_setup_1);
    if ( (fIsError = fDetectHiLoTransition()) ) {
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    // NOTE: DO NOT not wait for HiLo on SDATA after vector Init-3
    //       it does not occur (per spec).
    return(PASS);
}
#endif


// ============================================================================
// fVerifySiliconID()
// Returns:
//     0 if successful
//     Si_ID_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySiliconID(void)
{
    SendVector(id_setup_2, num_bits_id_setup_2);
    if ( (fIsError = fDetectHiLoTransition()) )
    {
        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("fDetectHiLoTransition Error");
        #endif
        return(SiID_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    SendVector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetID[0] = bReceiveByte();
    RunClock(1);
    SendVector(read_id_v+2, 12);    // 1+11 bits starting from the 3rd byte

    RunClock(2);                    // Read-LSB Command
    bTargetID[1] = bReceiveByte();

    RunClock(1);
    SendVector(read_id_v+4, 1);     // 1 bit starting from the 5th byte

    //read Revision ID from Accumulator A and Accumulator X
    //SendVector(read_id_v+5, 11);	//11 bits starting from the 6th byte
    //RunClock(2);
    //bTargetID[2] = bReceiveByte();	//Read from Acc.X
    //RunClock(1);
    //SendVector(read_id_v+7, 12);    //1+11 bits starting from the 8th byte
    //
    //RunClock(2);
    //bTargetID[3] = bReceiveByte();	//Read from Acc.A
    //
    //RunClock(1);
    //SendVector(read_id_v+4, 1);     //1 bit starting from the 5th byte,

    SendVector(tsync_disable, num_bits_tsync_disable);


    #ifdef TX_ON
        // Print READ-ID
        UART_PutCRLF(0);
        UART_PutString("Silicon-ID : ");
        UART_PutChar(' ');
        UART_PutHexByte(bTargetID[0]);
        UART_PutChar(' ');
        UART_PutHexByte(bTargetID[1]);
        UART_PutChar(' ');
    #endif

    #ifdef LCD_ON
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("ID : ");
        LCD_Char_PrintInt8(bTargetID[0]);
        LCD_Char_PutChar(' ');
        LCD_Char_PrintInt8(bTargetID[1]);
        LCD_Char_PutChar(' ');
    #endif

#if 1 // VENTURI
	if (bTargetID[0] != target_id_v[0] || bTargetID[1] <= 0x80|| bTargetID[1] >= 0x9F)
#else
    if (bTargetID[0] != target_id_v[0] /*|| bTargetID[1] != target_id_v[1]*/)
#endif
    {
        return(SiID_ERROR);
    }
    else
    {
        return(PASS);
    }
}

// PTJ: =======================================================================
// fReadStatus()
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadStatus(void)
{
    SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    SendVector(read_status, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetStatus = bReceiveByte();
    RunClock(1);
    SendVector(read_status+2, 1);    // 12 bits starting from the 3rd character

    SendVector(tsync_disable, num_bits_tsync_disable);

    if (bTargetStatus == 0x00)  // if bTargetStatus is 0x00, result is pass.
    {
        return PASS;
    }
    else
    {
        return BLOCK_ERROR;
    }

}

// PTJ: =======================================================================
// fReadWriteSetup()
// PTJ: The READ-WRITE-SETUP vector will enable TSYNC and switches the device
//		to SRAM bank1 for PROGRAM-AND-VERIFY, SECURE and VERIFY-SETUP.
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadWriteSetup(void)
{
	SendVector(read_write_setup, num_bits_read_write_setup);
	return(PASS);					//PTJ: is there anything else that should be done?
}

// ============================================================================
// fEraseTarget()
// Perform a bulk erase of the target device.
// Returns:
//     0 if successful
//     ERASE_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fEraseTarget(void)
{
    SendVector(erase, num_bits_erase);
    if ( (fIsError = fDetectHiLoTransition()) ) {
        return(ERASE_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}


// ============================================================================
// LoadTarget()
// Transfers data from array in Host to RAM buffer in the target.
// Returns the checksum of the data.
// ============================================================================
unsigned int iLoadTarget(void)
{
	unsigned char bTemp;
	unsigned int  iChecksumData = 0;

    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();

    // Transfer the temporary RAM array into the target.
    // In this section, a 128-Byte array was specified by #define, so the entire
    // 128-Bytes are written in this loop.
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    while(bTargetDataPtr < TARGET_DATABUFF_LEN) {
        bTemp = abTargetDataOUT[bTargetDataPtr];
        iChecksumData += bTemp;

        SendByte(write_byte_start,4);    //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTargetAddress, 7);	 //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);

        // !!!NOTE:
        // SendByte() uses MSbits, so inc by '2' to put the 0..128 address into
        // the seven MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 7-Bits long. The SendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..128 as the address using
        // SendByte(), we have to left justify the address by 1-Bit.
        //   This can be done easily by incrementing the address each time by
        // '2' rather than by '1'.

        bTargetAddress += 2;			//PTJ: inc by 2 in order to support a 128 byte address space
        bTargetDataPtr++;
    }

    return(iChecksumData);
}


// ============================================================================
// fProgramTargetBlock()
// Program one block with data that has been loaded into a RAM buffer in the
// target device.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fProgramTargetBlock(unsigned char bBankNumber, unsigned char bBlockNumber)
{

    SendVector(tsync_enable, num_bits_tsync_enable);

    SendVector(set_block_num, num_bits_set_block_num);

	// Set the drive here because SendByte() does not.
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    SendByte(set_block_num_end, 3);

    SendVector(tsync_disable, num_bits_tsync_disable);	//PTJ:

    // Send the program-block vector.
    SendVector(program_and_verify, num_bits_program_and_verify);		//PTJ: PROGRAM-AND-VERIFY
    // wait for acknowledge from target.
    if ( (fIsError = fDetectHiLoTransition()) )
    {
        return(BLOCK_ERROR);
    }
    // Send the Wait-For-Poll-End vector
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);

    //PTJ: Don't do READ-STATUS here because that will
    //PTJ: require that we return multiple error values, if error occurs
}


// ============================================================================
// fAddTargetBankChecksum()
// Reads and adds the target bank checksum to the referenced accumulator.
// Returns:
//     0 if successful
//     VERIFY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fAccTargetBankChecksum(unsigned int* pAcc)
{
    unsigned int wCheckSumData=0;

    SendVector(checksum_v, num_bits_checksum);

    if ( (fIsError = fDetectHiLoTransition()) )
    {
        return(CHECKSUM_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    //SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read Checksum vector and get Target Checksum
    SendVector(read_checksum_v, 11);     // first 11-bits is ReadCKSum-MSB
    RunClock(2);                         // Two SCLKs between write & read
    bTargetDataIN = bReceiveByte();
    wCheckSumData = ((unsigned int)(bTargetDataIN))<<8;

    RunClock(1);                         // See Fig. 6
    SendVector(read_checksum_v + 2, 12); // 12 bits starting from 3rd character
    RunClock(2);                         // Read-LSB Command
    bTargetDataIN = bReceiveByte();
    wCheckSumData |= (unsigned int) bTargetDataIN;
    RunClock(1);
    SendVector(read_checksum_v + 4, 1);  // Send the final bit of the command

    //SendVector(tsync_disable, num_bits_tsync_disable);

    *pAcc = wCheckSumData;

    return(PASS);
}


// ============================================================================
// ReStartTarget()
// After programming, the target PSoC must be reset to take it out of
// programming mode. This routine performs a reset.
// ============================================================================
void ReStartTarget(void)
{
#ifdef RESET_MODE
    // Assert XRES, then release, then disable XRES-Enable
    AssertXRES();
    Delay(XRES_CLK_DELAY);
    DeassertXRES();
#else
    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();
    // Cycle power on the target to cause a reset
    RemoveTargetVDD();
    Delay(POWER_CYCLE_DELAY);
    ApplyTargetVDD();
#endif
}

// ============================================================================
// fVerifySetup()
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySetup(unsigned char bBankNumber, unsigned char bBlockNumber)
{
    SendVector(tsync_enable, num_bits_tsync_enable);

    SendVector(set_block_num, num_bits_set_block_num);

	//Set the drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    SendByte(set_block_num_end, 3);

    SendVector(tsync_disable, num_bits_tsync_disable);

    SendVector(verify_setup, num_bits_my_verify_setup);
    if ( (fIsError = fDetectHiLoTransition()) )
    {
        return(VERIFY_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    return(PASS);
}

// ============================================================================
// fReadByteLoop()
// Reads the data back from Target SRAM and compares it to expected data in
// Host SRAM
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================

signed char fReadByteLoop(void)
{
	bTargetAddress = 0;
    bTargetDataPtr = 0;

    while(bTargetDataPtr < TARGET_DATABUFF_LEN)
    {
        //Send Read Byte vector and then get a byte from Target
        SendVector(read_byte_v, 4);
        // Set the drive here because SendByte() does not
        SetSDATAStrong();
        SendByte(bTargetAddress,7);

        RunClock(2);       // Run two SCLK cycles between writing and reading
        SetSDATAHiZ();     // Set to HiZ so Target can drive SDATA
        bTargetDataIN = bReceiveByte();

        RunClock(1);
        SendVector(read_byte_v + 1, 1);     // Send the ReadByte Vector End

        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if (bTargetDataIN != abTargetDataOUT[bTargetDataPtr])
        {
            #ifdef TX_ON
                UART_PutCRLF(0);
                UART_PutString("bTargetDataIN : ");
                UART_PutHexByte(bTargetDataIN);
                UART_PutString(" abTargetDataOUT : ");
                UART_PutHexByte(abTargetDataOUT[bTargetDataPtr]);
            #endif
            return(BLOCK_ERROR);
        }

        bTargetDataPtr++;
        // Increment the address by 2 to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        bTargetAddress += 2;

    }

    return(PASS);
}

// ============================================================================
// fSecureTargetFlash()
// Before calling, load the array, abTargetDataOUT, with the desired security
// settings using LoadArrayWithSecurityData(StartAddress,Length,SecurityType).
// The can be called multiple times with different SecurityTypes as needed for
// particular Flash Blocks. Or set them all the same using the call below:
// LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0);
// Returns:
//     0 if successful
//     SECURITY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fSecureTargetFlash(void)
{
    unsigned char bTemp;

    // Transfer the temporary RAM array into the target
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    SetSDATAStrong();
    while(bTargetDataPtr < SECURITY_BYTES_PER_BANK)
    {
        bTemp = abTargetDataOUT_secure[bTargetDataPtr];
        SendByte(write_byte_start,4);
        SendByte(bTargetAddress, 7);
        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);


        // SendBytes() uses MSBits, so increment the address by '2' to put
        // the 0..n address into the seven MSBit locations
        bTargetAddress += 2;				//PTJ: inc by 2 in order to support a 128 byte address space
        bTargetDataPtr++;
    }

    SendVector(secure, num_bits_secure);	//PTJ:
    if ( (fIsError = fDetectHiLoTransition()) )
    {
        return(SECURITY_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Main.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/* Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
// WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Cypress reserves the right to make changes without further notice to the
// materials described herein. Cypress does not assume any liability arising
// out of the application or use of any product or circuit described herein.
// Cypress does not authorize its products for use as critical components in
// life-support systems where a malfunction or failure may reasonably be
// expected to result in significant injury to the user. The inclusion of
// Cypress?product in a life-support systems application implies that the
// manufacturer assumes all risk of such use and in doing so indemnifies
// Cypress against all charges.
//
// Use may be limited by and subject to the applicable Cypress software
// license agreement.
//
//---------------------------------------------------------------------------*/

/* ############################################################################
   ###################  CRITICAL PROJECT CONSTRAINTS   ########################
   ############################################################################

   ISSP programming can only occur within a temperature range of 5C to 50C.
   - This project is written without temperature compensation and using
     programming pulse-widths that match those used by programmers such as the
     Mini-Prog and the ISSP Programmer.
     This means that the die temperature of the PSoC device cannot be outside
     of the above temperature range.
     If a wider temperature range is required, contact your Cypress Semi-
     conductor FAE or sales person for assistance.

   The project can be configured to program devices at 5V or at 3.3V.
   - Initialization of the device is different for different voltages. The
     initialization is hardcoded and can only be set for one voltage range.
     The supported voltages ranges are 3.3V (3.0V to 3.6V) and 5V (4.75V to
     5.25V). See the device datasheet for more details. If varying voltage
     ranges must be supported, contact your Cypress Semiconductor FAE or sales
     person for assistance.
   - ISSP programming for the 2.7V range (2.7V to 3.0V) is not supported.

   This program does not support programming all PSoC Devices
   - It does not support obsoleted PSoC devices. A list of devices that are
     not supported is shown here:
         CY8C22x13 - not supported
         CY8C24x23 - not supported (CY8C24x23A is supported)
         CY8C25x43 - not supported
         CY8C26x43 - not supported
   - It does not suport devices that have not been released for sale at the
     time this version was created. If you need to ISSP program a newly released
     device, please contact Cypress Semiconductor Applications, your FAE or
     sales person for assistance.
     The CY8C20x23 devices are not supported at the time of this release.

   ############################################################################
   ##########################################################################*/


/* (((((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))
 PSoC In-System Serial Programming (ISSP) Template
 This PSoC Project is designed to be used as a template for designs that
 require PSoC ISSP Functions.

 This project is based on the AN2026 series of Application Notes. That app
 note should be referenced before any modifications to this project are made.

 The subroutines and files were created in such a way as to allow easy cut &
 paste as needed. There are no customer-specific functions in this project.
 This demo of the code utilizes a PSoC as the Host.

 Some of the subroutines could be merged, or otherwise reduced, but they have
 been written as independently as possible so that the specific steps involved
 within each function can easily be seen. By merging things, some code-space
 savings could be realized.

 As is, and with all features enabled, the project consumes approximately 3500
 bytes of code space, and 19-Bytes of RAM (not including stack usage). The
 Block-Verify requires a 64-Byte buffer for read-back verification. This same
 buffer could be used to hold the (actual) incoming program data.

 Please refer to the compiler-directives file "directives.h" to see the various
 features.

 The pin used in this project are assigned as shown below. The HOST pins are
 arbitrary and any 3 pins could be used (the masks used to control the pins
 must be changed). The TARGET pins cannot be changed, these are fixed function
 pins on the PSoC.
 The PWR pin is used to provide power to the target device if power cycle
 programming mode is used. The compiler directive RESET_MODE in ISSP_directives.h
 is used to select the programming mode. This pin could control the enable on
 a voltage regulator, or could control the gate of a FET that is used to turn
 the power to the PSoC on.
 The TP pin is a Test Point pin that can be used signal from the host processor
 that the program has completed certain tasks. Predefined test points are
 included that can be used to observe the timing for bulk erasing, block
 programming and security programming.

      SIGNAL  HOST  TARGET
      ---------------------
      SDATA   P1.0   P1.0
      SCLK    P1.1   P1.1
      XRES    P2.0   XRES
      PWR     P2.1   Vdd
      TP      P0.7   n/a

 For test & demonstration, this project generates the program data internally.
 It does not take-in the data from an external source such as I2C, UART, SPI,
 etc. However, the program was written in such a way to be portable into such
 designs. The spirit of this project was to keep it stripped to the minimum
 functions required to do the ISSP functions only, thereby making a portable
 framework for integration with other projects.

 The high-level functions have been written in C in order to be portable to
 other processors. The low-level functions that are processor dependent, such
 as toggling pins and implementing specific delays, are all found in the file
 ISSP_Drive_Routines.c. These functions must be converted to equivalent
 functions for the HOST processor.  Care must be taken to meet the timing
 requirements when converting to a new processor. ISSP timing information can
 be found in Application Note AN2026.  All of the sections of this program
 that need to be modified for the host processor have "PROCESSOR_SPECIFIC" in
 the comments. By performing a "Find in files" using "PROCESSOR_SPECIFIC" these
 sections can easily be identified.

 The variables in this project use Hungarian notation. Hungarian prepends a
 lower case letter to each variable that identifies the variable type. The
 prefixes used in this program are defined below:
  b = byte length variable, signed char and unsigned char
  i = 2-byte length variable, signed int and unsigned int
  f = byte length variable used as a flag (TRUE = 0, FALSE != 0)
  ab = an array of byte length variables


 After this program has been ported to the desired host processor the timing
 of the signals must be confirmed.  The maximum SCLK frequency must be checked
 as well as the timing of the bulk erase, block write and security write
 pulses.

 The maximum SCLK frequency for the target device can be found in the device
 datasheet under AC Programming Specifications with a Symbol of "Fsclk".
 An oscilloscope should be used to make sure that no half-cycles (the high
 time or the low time) are shorter than the half-period of the maximum
 freqency. In other words, if the maximum SCLK frequency is 8MHz, there can be
 no high or low pulses shorter than 1/(2*8MHz), or 62.5 nsec.

 The test point (TP) functions, enabled by the define USE_TP, provide an output
 from the host processor that brackets the timing of the internal bulk erase,
 block write and security write programming pulses. An oscilloscope, along with
 break points in the PSoC ICE Debugger should be used to verify the timing of
 the programming.  The Application Note, "Host-Sourced Serial Programming"
 explains how to do these measurements and should be consulted for the expected
 timing of the erase and program pulses.

 ############################################################################
 ############################################################################

(((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))) */



/*----------------------------------------------------------------------------
//                               C main line
//----------------------------------------------------------------------------
*/

//#include <m8c.h>        // part specific constants and macros
//#include "PSoCAPI.h"    // PSoC API definitions for all User Modules


// ------ Declarations Associated with ISSP Files & Routines -------
//     Add these to your project as needed.
//#include "ISSP_extern.h"
//#include "ISSP_directives.h"
//#include "ISSP_defs.h"
//#include "ISSP_errors.h"
//#include "Device.h"
/* ------------------------------------------------------------------------- */

unsigned char bBankCounter;
unsigned int  iBlockCounter;
unsigned int  iChecksumData;
unsigned int  iChecksumTarget;




/* ========================================================================= */
// ErrorTrap()
// Return is not valid from main for PSOC, so this ErrorTrap routine is used.
// For some systems returning an error code will work best. For those, the
// calls to ErrorTrap() should be replaced with a return(bErrorNumber). For
// other systems another method of reporting an error could be added to this
// function -- such as reporting over a communcations port.
/* ========================================================================= */
void ErrorTrap(unsigned char bErrorNumber)
{
    #ifndef RESET_MODE
        // Set all pins to highZ to avoid back powering the PSoC through the GPIO
        // protection diodes.
        SetSCLKHiZ();
        SetSDATAHiZ();
        // If Power Cycle programming, turn off the target
        RemoveTargetVDD();
    #endif


    #ifdef TX_ON
        UART_PutCRLF(0);
        UART_PutString("ErrorTrap");
        UART_PutHexByte(bErrorNumber);
    #endif

    #ifdef LCD_ON
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("                ");
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("ErrorTrap");
        LCD_Char_PrintInt8(bErrorNumber);
    #endif

	/* Enable watchdog and interrupt */
//	TchDrv_DownloadEnableWD();
//	TchDrv_DownloadEnableIRQ();

    //while (1);
    // return(bErrorNumbers);
}

/* ========================================================================= */
/* MAIN LOOP                                                                 */
/* Based on the diagram in the AN2026                                        */
/* ========================================================================= */
unsigned char make2ChTo1(unsigned char hi, unsigned char lo)
{
    unsigned char ch;

    if(hi == 'A' || hi == 'a')
        hi = 0xa;
    else if(hi == 'B' || hi == 'b')
        hi = 0xb;
    else if(hi == 'C' || hi == 'c')
        hi = 0xc;
    else if(hi == 'D' || hi == 'd')
        hi = 0xd;
    else if(hi == 'E' || hi == 'e')
        hi = 0xe;
    else if(hi == 'F' || hi == 'f')
        hi = 0xf;
    else
        hi = hi;

    if(lo == 'A' || lo == 'a')
        lo = 0xa;
    else if(lo == 'B' || lo == 'b')
        lo = 0xb;
    else if(lo == 'C' || lo == 'c')
        lo = 0xc;
    else if(lo == 'D' || lo == 'd')
        lo = 0xd;
    else if(lo == 'E' || lo == 'e')
        lo = 0xe;
    else if(lo == 'F' || lo == 'f')
        lo = 0xf;
    else
        lo = lo;

    ch = ((hi&0x0f) << 4) | (lo & 0x0f);

    return ch;
}


UInt16 load_tma340_frimware_data(void)
{
#if defined(CONFIG_MACH_VENTURI)

	;

#else
	UInt8 temp_onelinedata[128];
	UInt16 i, firmwareline, onelinelength;

	for(firmwareline=0; firmwareline<512; firmwareline++)
	{
		i = 0;
		strncpy(temp_onelinedata, cytma340_fw + 141*firmwareline + 9, 128);

		for(onelinelength=0; onelinelength<64; onelinelength++)
		{
			firmData[firmwareline][onelinelength] = make2ChTo1(temp_onelinedata[i], temp_onelinedata[i+1]);
			i += 2;
		}
	}
#endif
	return PASS;
}


int tma340_frimware_update(void)
{
    // -- This example section of commands show the high-level calls to -------
    // -- perform Target Initialization, SilcionID Test, Bulk-Erase, Target ---
    // -- RAM Load, FLASH-Block Program, and Target Checksum Verification. ----
	UInt16 i;
	UInt16 aIndex;

    #ifdef TX_ON
        UART_PutString("Start HSSP - TMA3x0");
        UART_PutCRLF(0);
    #endif

	if ( (fIsError = load_tma340_frimware_data()) )
    {
        ErrorTrap(fIsError);
		return fIsError;
    }

    // >>>> ISSP Programming Starts Here <<<<

    // Acquire the device through reset or power cycle
    #ifdef RESET_MODE
        // Initialize the Host & Target for ISSP operations
        if (fIsError = fXRESInitializeTargetForISSP())
        {
            ErrorTrap(fIsError);
			return fIsError;
        }
    #else
        // Initialize the Host & Target for ISSP operations
        if ( (fIsError = fPowerCycleInitializeTargetForISSP()) )
        {
            ErrorTrap(fIsError);
			return fIsError;
        }
    #endif


    // Run the SiliconID Verification, and proceed according to result.
    if ( (fIsError = fVerifySiliconID()) )
    {
        ErrorTrap(fIsError);
		return fIsError;
    }
    #ifdef TX_ON
        UART_PutCRLF(0);
        UART_PutString("End VerifySiliconID");
    #endif

	/* Disable watchdog and interrupt */
	TchDrv_DownloadDisableIRQ();	// Disable Baseband touch interrupt ISR.
	TchDrv_DownloadDisableWD();		// Disable Baseband watchdog timer

    #if 1
        // Bulk-Erase the Device.
        if ( (fIsError = fEraseTarget()) )
        {
            ErrorTrap(fIsError);
			//return fIsError;
			goto MCSDL_DOWNLOAD_FINISH;
        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("End EraseTarget");
            UART_PutCRLF(0);
            UART_PutString("Program Flash Blocks Start");
            UART_PutCRLF(0);
        #endif

    #endif

    #if 1   // program flash block
        //LCD_Char_Position(1, 0);
        //LCD_Char_PrintString("Program Flash Blocks Start");

        //==============================================================//
        // Program Flash blocks with predetermined data. In the final application
        // this data should come from the HEX output of PSoC Designer.

        iChecksumData = 0;     // Calculte the device checksum as you go
        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
            if ( (fIsError = fReadWriteSetup()) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

			aIndex = iBlockCounter*2;

			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}

            //LoadProgramData(bBankCounter, (unsigned char)iBlockCounter);
            iChecksumData += iLoadTarget();

            if ( (fIsError = fProgramTargetBlock(bBankCounter,(unsigned char)iBlockCounter)) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            if ( (fIsError = fReadStatus()) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            #ifdef TX_ON
                UART_PutChar('#');
            #endif

        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Program Flash Blocks End");
        #endif

    #endif


    #if 1  // verify
        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Verify Start");
            UART_PutCRLF(0);
        #endif

        //=======================================================//
        //PTJ: Doing Verify
        //PTJ: this code isnt needed in the program flow because we use PROGRAM-AND-VERIFY (ProgramAndVerify SROM Func)
        //PTJ: which has Verify built into it.
        // Verify included for completeness in case host desires to do a stand-alone verify at a later date.

        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
        	//LoadProgramData(bBankCounter, (unsigned char) iBlockCounter);
			aIndex = iBlockCounter*2;

			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}

            if ( (fIsError = fReadWriteSetup()) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            if ( (fIsError = fVerifySetup(bBankCounter,(unsigned char)iBlockCounter)) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if ( (fIsError = fReadStatus()) ) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if ( (fIsError = fReadWriteSetup()) ) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if ( (fIsError = fReadByteLoop()) ) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            #ifdef TX_ON
                UART_PutChar('.');
            #endif

        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Verify End");
        #endif

    #endif // end verify


    #if 1

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Security Start");
        #endif


        //=======================================================//
        // Program security data into target PSoC. In the final application this
        // data should come from the HEX output of PSoC Designer.
        for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
        {
            //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1

            if ( (fIsError = fReadWriteSetup()) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }
            // Secure one bank of the target flash
            if ( (fIsError = fSecureTargetFlash()) )
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }
        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("End Security data");
        #endif

    #endif

MCSDL_DOWNLOAD_FINISH :

	Delay10us(50*1000);
	Delay10us(50*1000);

	/* Enable watchdog and interrupt */
	TchDrv_DownloadEnableWD();
//	TchDrv_DownloadEnableIRQ();

	Delay10us(50*1000);
	Delay10us(50*1000);
	Delay10us(50*1000);
	Delay10us(50*1000);

	return fIsError;

}


#endif
