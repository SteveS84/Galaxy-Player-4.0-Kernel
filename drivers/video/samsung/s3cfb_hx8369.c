/*
 * hx8369 TFT-LCD Panel Driver for the Samsung Universal board
 *
 * Derived from drivers/video/samsung/s3cfb_hx8369.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#include <linux/backlight.h>

#include <plat/gpio-cfg.h>

#include "s3cfb.h"
#include "hx8369.h"
#if defined(CONFIG_MACH_VENTURI)
#include "s3cfb_mdnie.h"
#endif


// Brightness Level 
#define DIM_BL					20
#define MIN_BL					30
#define MAX_BL					255

#define MAX_GAMMA_VALUE	24 // Venturi_?


#define CRITICAL_BATTERY_LEVEL 	5

// Venturi_MustDelete
#define GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform

#if defined(CONFIG_MACH_VENTURI)
#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 30

#define MAX_BACKLIGHT_VALUE_SONY 183 //180 // 190
#define LOW_BACKLIGHT_VALUE_SONY 13 //8 //39 // 50
#define DIM_BACKLIGHT_VALUE_SONY 13 //8 //23 // 30
#endif

// Backlight Level
typedef enum {
	BACKLIGHT_LEVEL_OFF		= 0,
	BACKLIGHT_LEVEL_DIMMING	= 1,
	BACKLIGHT_LEVEL_NORMAL	= 6
} backlight_level_t;


/*********** for debug **********************************************************/
#if 0
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* CABC ************************************************************************/

typedef enum
{
	CABC_OFF,
	CABC_UI,
	CABC_IMAGE,
	CABC_VIDEO,
}LCD_CABC;

static LCD_CABC	cur_cabc = CABC_OFF;
/*******************************************************************************/

static int ldi_enable = 0;



backlight_level_t backlight_level = BACKLIGHT_LEVEL_OFF;
static int bd_brightness = 0;

static int on_19gamma = 0;

static DEFINE_MUTEX(spi_use);

#if defined(CONFIG_MACH_VENTURI)
#define PWM_REG_OFFSET		1
static unsigned short brightness_setting_table[] = {
	0x051, 0x17f,
	ENDDEF, 0x0000                                
};
#endif


struct s5p_lcd {
	struct spi_device 			*g_spi;
	struct lcd_device 			*lcd_dev;
	struct backlight_device 	*bl_dev;
	struct early_suspend    early_suspend;
};

#ifdef GAMMASET_CONTROL
struct class *gammaset_class;
struct device *switch_gammaset_dev;
#endif

#ifdef CONFIG_FB_S3C_HX8369_ACL
static int acl_enable = 0;

struct class *acl_class;
struct device *switch_aclset_dev;
#endif

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

static struct s5p_lcd lcd;

/* 
 *	Venturi LCD Spec
 *	DOTCLK = FrameRate x (HSW+HBP+XRES+HFP) x (VSW+VBP+800+VFP)
 *	             = 60 x (8+24+480+24) x (8+24+800+24) = 27,528,960 = 27.52MHz
 */
static struct s3cfb_lcd hx8369 = {
	.width 		= 480,
	.height 	= 800,
	.p_width 	= 52,	// Venturi_?   height of lcd in mm
	.p_height 	= 86,	// Venturi_?  width of lcd in mm
	.bpp 		= 24,
	.freq 		= 60,	// Venturi_?
	.timing = {		
		.h_fp 	= 32, 
		.h_bp 	= 32,
		.h_sw 	= 14, 
		.v_fp 	= 12, 
		.v_fpe 	= 1,
		.v_bp 	= 12, 
		.v_bpe 	= 1,
		.v_sw 	= 8,
	},
	.polarity = {		
		.rise_vclk 	= 0, // video data fetch at DOTCLK falling edge 
		.inv_hsync 	= 1,	// low active
		.inv_vsync 	= 1,	// low active
		.inv_vden 	= 0,	// data is vaild when DEpin is high
	},
};

static void wait_ldi_enable(void);

static int hx8369_spi_write_driver(int reg)
{
	u16 buf[1];
	int ret;
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= buf,
	};

	buf[0] = reg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);


	ret = spi_sync(lcd.g_spi, &msg);

	if (ret < 0)
		pr_err("%s::%d -> spi_sync failed Err=%d\n",__func__,__LINE__,ret);
	return ret ;

}

static void hx8369_spi_write(unsigned short reg)
{
  	hx8369_spi_write_driver(reg);	
}

static void hx8369_panel_send_sequence(const unsigned short *wbuf)
{
	int i = 0;

	mutex_lock(&spi_use);

	gprintk("#################SPI start##########################\n");
	
	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC){
			hx8369_spi_write(wbuf[i]);
			i+=1;}
		else{
			msleep(wbuf[i+1]);
			i+=2;}
	}
	
	gprintk("#################SPI end##########################\n");

	mutex_unlock(&spi_use);
}

int IsLDIEnabled(void)
{
	return ldi_enable;
}
EXPORT_SYMBOL(IsLDIEnabled);

static void SetLDIEnabledFlag(int OnOff)
{
	ldi_enable = OnOff;
}


#if defined(CONFIG_MACH_VENTURI)
extern Lcd_mDNIe_UI current_mDNIe_UI;

void on_cabc(void)
{
	if(acl_enable == 0)
		return;
		
	// ACL ON
	switch(current_mDNIe_UI)
	{
		case mDNIe_UI_MODE:
#if 0	//2010.12.19 disabled CABC_UI Mode, requested by younghyup.kim
			if(cur_cabc != CABC_UI)
			{
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_UI);
				cur_cabc = CABC_UI;
				gprintk("set hx8369_SEQ_CABC_UI\n");
			}
#else
			if(cur_cabc != CABC_OFF)
			{
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_OFF);
				cur_cabc = CABC_OFF;
				gprintk("set hx8369_SEQ_CABC_OFF\n");
			}
#endif
			break;
		case mDNIe_VIDEO_MODE:
		case mDNIe_VIDEO_WARM_MODE:
		case mDNIe_VIDEO_COLD_MODE:
		case mDNIe_CAMERA_MODE:
		case mDNIe_DMB_MODE:
		case mDNIe_DMB_WARM_MODE:
		case mDNIe_DMB_COLD_MODE:
        	if(cur_cabc != CABC_VIDEO)
        	{
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_VIDEO);
			 	cur_cabc = CABC_VIDEO;
				gprintk("set hx8369_SEQ_CABC_VIDEO\n");
			}
			 break;
		case mDNIe_NAVI:
			if(cur_cabc != CABC_IMAGE)
	        	{
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_IMAGE);
			 	cur_cabc = CABC_IMAGE;
				gprintk("set hx8369_SEQ_CABC_IMAGE\n");
			}
			break;
		default:
#if 0	//2010.12.19 disabled CABC_UI Mode, requested by younghyup.kim
			if(cur_cabc != CABC_UI)
			{		
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_UI);
				cur_cabc = CABC_UI;
				gprintk("set hx8369_SEQ_CABC_UI-\n");
			}
#else
			if(cur_cabc != CABC_OFF)
			{		
				hx8369_panel_send_sequence(hx8369_SEQ_CABC_OFF);
				cur_cabc = CABC_OFF;
				gprintk("set hx8369_SEQ_CABC_OFF-\n");
			}
#endif
			break;
	}
}

void off_cabc(void)
{
	// check LDI Status
	wait_ldi_enable();
	if (!IsLDIEnabled())
	{
		printk("<<<<<<<<<<< [off_cabc] failt to CABC Mode Setting >>>>>>>>>>>>>>>>\n");
		return;
	}

	// ACL OFF
	gprintk("set hx8369_SEQ_CABC_OFF\n");
	cur_cabc = CABC_OFF;
	hx8369_panel_send_sequence(hx8369_SEQ_CABC_OFF);
}

//#define CABC_TEST
#ifdef CABC_TEST

static int cabc_mode = 0;

static ssize_t test_cabc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	gprintk("[%s] cabc_mode = %d\n",__func__, cabc_mode);

	return sprintf(buf,"%u\n", cabc_mode);
}
static ssize_t test_cabc_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
    sscanf(buf, "%d", &value);

	switch(value)
	{
		case 1:
			hx8369_panel_send_sequence(hx8369_SEQ_CABC_UI);	cabc_mode = 1;	gprintk("[%s] set CABC_UI\n", __func__);
			break;
		case 2:
			hx8369_panel_send_sequence(hx8369_SEQ_CABC_IMAGE); cabc_mode = 2;	gprintk("[%s] set CABC_IMAGE\n", __func__);
			break;
		case 3:
			hx8369_panel_send_sequence(hx8369_SEQ_CABC_VIDEO); cabc_mode = 3;	gprintk("[%s] set CABC_VIDEO\n", __func__);
		default:
			hx8369_panel_send_sequence(hx8369_SEQ_CABC_OFF); cabc_mode = 0;	gprintk("[%s] set CABC_OFF\n", __func__);
			break;
	}

	return size;
}

static DEVICE_ATTR(test_cabc,0664, test_cabc_show, test_cabc_store);
#endif
#endif


void hx8369_ldi_init(void)
{
	gprintk("[%s]\n", __func__); 
//	msleep(120); // add by SMD
	hx8369_panel_send_sequence(hx8369_SEQ_SETTING);
	hx8369_panel_send_sequence(hx8369_BACKLIGHT_SETTING);
	
	SetLDIEnabledFlag(1);

	if(acl_enable)
		on_cabc();
}


void hx8369_ldi_enable(void)
{
}

void hx8369_ldi_disable(void)
{
 	hx8369_panel_send_sequence(hx8369_SEQ_DISPLAY_OFF); 	

	SetLDIEnabledFlag(0);
	printk(KERN_DEBUG "LDI disable ok\n");
	//pr_info("%s::%d -> ldi disabled\n",__func__,__LINE__);	
}


void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	hx8369.init_ldi = NULL;
	ctrl->lcd = &hx8369;
}

// Venturi
#if defined (CONFIG_MACH_VENTURI)
static int get_pwm_value_from_bl(int level)
{
	int tune_value;

	// SMD LCD
	if(level > MAX_BRIGHTNESS_LEVEL)
		level = MAX_BRIGHTNESS_LEVEL;

	if(level >= LOW_BRIGHTNESS_LEVEL)
		tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_SONY-LOW_BACKLIGHT_VALUE_SONY) / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_SONY;
	else if(level > 0)
		tune_value = DIM_BACKLIGHT_VALUE_SONY;
	else
		tune_value = level;
	
	if(tune_value > MAX_BACKLIGHT_VALUE_SONY)
		tune_value = MAX_BACKLIGHT_VALUE_SONY;			// led_val must be less than or equal to MAX_BACKLIGHT_VALUE

	if(level && !tune_value)
		tune_value = 1;

	return tune_value;
}


static int update_brightness(int level)
{
	unsigned int led_val;

	// check LDI Status
	wait_ldi_enable();
	if (!IsLDIEnabled())
	{
		printk("<<<<<<<<<<< [update_brightness] brightness setting error >>>>>>>>>>>>>>>>\n");
		return 0;
	}

	led_val = get_pwm_value_from_bl(level);

	brightness_setting_table[PWM_REG_OFFSET] = 0x100 | (led_val & 0xff);

	gprintk("[bl]%d(%d)\n", level, brightness_setting_table[PWM_REG_OFFSET]&0xff);
	hx8369_panel_send_sequence(brightness_setting_table);

	return 0;
}


void backlight_onoff(backlight_level_t f_onoff)
{
	gprintk("[%s]=%d\n", __func__, f_onoff); 
	
	if(f_onoff) 
	{
		// on
		if (gpio_is_valid(GPIO_BACKLIGHT_EN)) 
		{
				if (gpio_request(GPIO_BACKLIGHT_EN, "GPD0"))
						printk("Failed to request GPIO_BACKLIGHT_EN!\n");
				
				s3c_gpio_cfgpin(GPIO_BACKLIGHT_EN, S3C_GPIO_OUTPUT);
				gpio_direction_output(GPIO_BACKLIGHT_EN, (int)1);
		}
		s3c_gpio_setpull(GPIO_BACKLIGHT_EN, S3C_GPIO_PULL_NONE);
		gpio_free(GPIO_BACKLIGHT_EN);


		backlight_level = BACKLIGHT_LEVEL_NORMAL;
		//printk("[VIBETONZ] ENABLE\n");
	}
	else  
	{
		// off
		if (gpio_is_valid(GPIO_BACKLIGHT_EN)) 
		{
				if (gpio_request(GPIO_BACKLIGHT_EN, "GPD0"))
						printk("Failed to request GPIO_BACKLIGHT_EN!\n");

				s3c_gpio_cfgpin(GPIO_BACKLIGHT_EN, S3C_GPIO_OUTPUT);
				gpio_direction_output(GPIO_BACKLIGHT_EN, (int)0);
		}
		s3c_gpio_setpull(GPIO_BACKLIGHT_EN, S3C_GPIO_PULL_NONE);
		gpio_free(GPIO_BACKLIGHT_EN);	
		
		backlight_level = BACKLIGHT_LEVEL_OFF;
	}	
}
EXPORT_SYMBOL(backlight_onoff);

void hx8369_backlight_init(void)
{
	backlight_onoff(BACKLIGHT_LEVEL_NORMAL);
}

void hx8369_backlight_resume(void)
{
		hx8369_ldi_init();
		
		backlight_onoff(BACKLIGHT_LEVEL_NORMAL);

		printk(KERN_DEBUG "LDI enable ok\n");
		pr_info("%s::%d -> ldi initialized\n",__func__,__LINE__);	
}

void hx8369_backlight_suspend(void)
{

		backlight_onoff(BACKLIGHT_LEVEL_OFF);

		hx8369_ldi_disable();


		printk(KERN_DEBUG "LDI disable ok\n");
		pr_info("%s::%d -> ldi disabled\n",__func__,__LINE__);	
}

#endif

int s5p_lcd_set_powerOnOff(int power)
{
	printk("[%s][minhyodebug]=================\n", __func__); 
	
	if (power)	
	{
		printk("[%s] = on\n", __func__);
//		msleep(120); // add by SMD
		hx8369_panel_send_sequence(hx8369_SEQ_SETTING);
		hx8369_panel_send_sequence(hx8369_BACKLIGHT_SETTING);
		backlight_onoff(BACKLIGHT_LEVEL_NORMAL);

		SetLDIEnabledFlag(1);
		printk(KERN_DEBUG "LDI enable ok\n");
		pr_info("%s::%d -> ldi initialized\n",__func__,__LINE__);	
	}
	else	
	{
		hx8369_panel_send_sequence(hx8369_SEQ_DISPLAY_OFF);
		backlight_onoff(BACKLIGHT_LEVEL_OFF);

		SetLDIEnabledFlag(0);
		printk(KERN_DEBUG "LDI disable ok\n");
		pr_info("%s::%d -> ldi disabled\n",__func__,__LINE__);	
	}

	return 0;
}

// LCD ON/OFF function
static int s5p_lcd_set_power(struct lcd_device *ld, int power)
{
	printk("s5p_lcd_set_power is called:============================== %d", power);

	s5p_lcd_set_powerOnOff(power); 

	return 0;
}

static int s5p_lcd_check_fb(struct lcd_device *lcddev, struct fb_info *fi)
{
	return 0;
}

static struct lcd_ops s5p_lcd_ops = {
	.set_power = s5p_lcd_set_power,
   .check_fb = s5p_lcd_check_fb,      
};

#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
static ssize_t gammaset_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	gprintk("called %s \n",__func__);

	return sprintf(buf,"%u\n", bd_brightness);
}
static ssize_t gammaset_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
    sscanf(buf, "%d", &value);
	
	// not support for TFT-LCD

	return size;
}

static DEVICE_ATTR(gammaset_file_cmd,0664, gammaset_file_cmd_show, gammaset_file_cmd_store);
#endif

#ifdef CONFIG_FB_S3C_HX8369_ACL 
static ssize_t aclset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", acl_enable);
}
static ssize_t aclset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	gprintk("CABC = %d\n", value );

	if (!IsLDIEnabled())	{
		printk(KERN_DEBUG "[acl set] return because LDI is disabled, input value = %d \n", value);
		return size;
	}

	if ((value != 0) && (value != 1))	{
		gprintk(KERN_DEBUG "\naclset_file_cmd_store value is same : value(%d)\n", value);
		return size;
	}

	if (acl_enable != value)	
	{
		acl_enable = value;

		if (acl_enable == 1)	
			on_cabc();
		else	
			off_cabc();
	}

	return size;
}

static DEVICE_ATTR(aclset_file_cmd,0664, aclset_file_cmd_show, aclset_file_cmd_store);
#endif


#ifdef CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT
extern void mDNIe_Mode_set_for_backlight(u16 *buf);
extern u16 *pmDNIe_Gamma_set[];
extern int pre_val;
extern int autobrightness_mode;
#endif

static void wait_ldi_enable(void)
{
	int i = 0;

	for (i = 0; i < 100; i++)	{
		gprintk("ldi_enable : %d \n", ldi_enable);

		if(IsLDIEnabled())
			break;
		
		msleep(10);
	};
}

#if 0
static void off_display(void)
{
	msleep(20);
	hx8369_panel_send_sequence(hx8369_SEQ_DISPLAY_OFF); 
	bd_brightness = 0;
	backlight_level = BACKLIGHT_LEVEL_OFF;

	SetLDIEnabledFlag(0);
	printk(KERN_DEBUG "LDI disable ok\n");
	pr_info("%s::%d -> ldi disabled\n",__func__,__LINE__);	
}
#endif

static int s5p_bl_update_status(struct backlight_device* bd)
{
	int 				br = bd->props.brightness;		// Brightness Level
	backlight_level_t 	level = BACKLIGHT_LEVEL_OFF;	// Brightness Mode
	int 				bl = 0;							// Backlight Level
	

	// check brightness level
	if(br < 0)						
		return 0;

	// check LDI Status
	wait_ldi_enable();
	if (!IsLDIEnabled())
		return 0;

	// decide Brightness Mode & Backlight Level
	if(br == 0)				{ level = BACKLIGHT_LEVEL_OFF;		}
	else if(br < MIN_BL)	{ level = BACKLIGHT_LEVEL_DIMMING;	}	
	else					{ level = BACKLIGHT_LEVEL_NORMAL;	}

	update_brightness(br);
	
	// update Backlight & Brightness & Screen Mode
	bd_brightness 	= br;		// Brightness Level
	backlight_level = level;	// Backlight  Level
	
	gprintk("[%s] br = %d, bl = %d\n", __func__, br, bl);

	return 0;
}

static int s5p_bl_get_brightness(struct backlight_device* bd)
{
	gprintk("[%s] Brightness Read = %d\n", __func__, bd_brightness);
	return bd_brightness;
}

static struct backlight_ops s5p_bl_ops = {
	.update_status = s5p_bl_update_status,
	.get_brightness = s5p_bl_get_brightness,
};

void hx8396_early_suspend(struct early_suspend *h)
{
	hx8369_backlight_suspend();

	return ;
}
void hx8396_late_resume(struct early_suspend *h)
{
	  hx8369_backlight_resume();

	return ;
}

static int __init hx8396_probe(struct spi_device *spi)
{
	int ret;

	printk("hx8396_probe  INIT ..........\n");

	gprintk("[%s] \n", __func__);

	spi->bits_per_word = 9;
	ret = spi_setup(spi);
	lcd.g_spi = spi;
	lcd.lcd_dev = lcd_device_register("s5p_lcd",&spi->dev,&lcd,&s5p_lcd_ops);
	lcd.bl_dev = backlight_device_register("s5p_bl",&spi->dev,&lcd,&s5p_bl_ops,NULL);
	lcd.bl_dev->props.max_brightness = 255;
	dev_set_drvdata(&spi->dev,&lcd);

	SetLDIEnabledFlag(1);

#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
	gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(gammaset_class))
		pr_err("Failed to create class(gammaset_class)!\n");

	switch_gammaset_dev = device_create(gammaset_class, NULL, 0, NULL, "switch_gammaset");
	if (IS_ERR(switch_gammaset_dev))
		pr_err("Failed to create device(switch_gammaset_dev)!\n");

	if (device_create_file(switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);
#endif	

#ifdef CONFIG_FB_S3C_HX8369_ACL //ACL On,Off
	acl_class = class_create(THIS_MODULE, "aclset");
	if (IS_ERR(acl_class))
		pr_err("Failed to create class(acl_class)!\n");

	switch_aclset_dev = device_create(acl_class, NULL, 0, NULL, "switch_aclset");
	if (IS_ERR(switch_aclset_dev))
		pr_err("Failed to create device(switch_aclset_dev)!\n");

	if (device_create_file(switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);
#endif
	hx8369_backlight_init();

#ifdef CONFIG_FB_S3C_MDNIE
	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd.early_suspend.suspend = hx8396_early_suspend;
	lcd.early_suspend.resume = hx8396_late_resume;
	lcd.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&lcd.early_suspend);
#endif

	if (ret < 0)	{
		pr_err("%s::%d-> hx8369 probe failed Err=%d\n",__func__,__LINE__,ret);
		return 0;
	}
	pr_info("%s::%d->hx8369 probed successfuly\n",__func__,__LINE__);
	return ret;
}

#if 0
#ifdef CONFIG_PM // add by ksoo (2009.09.07)
int hx8369_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s::%d->hx8369 suspend called\n",__func__,__LINE__);
	hx8369_ldi_disable();
	return 0;
}

int hx8369_resume(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s::%d -> hx8369 resume called\n",__func__,__LINE__);
	hx8369_ldi_init();
	hx8369_ldi_enable();

	return 0;
}
#endif	/* CONFIG_PM */
#endif

static struct spi_driver hx8369_driver = {
	.driver = {
		.name	= "hx8369",
		.owner	= THIS_MODULE,
	},
	.probe		= hx8396_probe,
	.remove		= __exit_p(hx8369_remove),
};

static int __init hx8369_init(void)
{
	printk("hx8369_init  INIT ..........\n");

	return spi_register_driver(&hx8369_driver);
}

static void __exit hx8369_exit(void)
{
	spi_unregister_driver(&hx8369_driver);
}


module_init(hx8369_init);
module_exit(hx8369_exit);


MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("hx8369 LDI driver");
MODULE_LICENSE("GPL");

