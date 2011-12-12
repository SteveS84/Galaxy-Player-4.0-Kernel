/*
 * nt35580 TFT Panel Driver for the Samsung Universal board
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
#include <linux/nt35580.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-fb.h>
#include <linux/earlysuspend.h>
#if defined(CONFIG_FB_S3C_MDNIE)
#include "s3cfb_mdnie.h"
#endif

#if defined(CONFIG_FB_S3C_MDNIE)
extern void init_mdnie_class(struct s5p_lcd *lcd);
#endif

#define NT35580_POWERON_DELAY	150

#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 31//30
#define MAX_BACKLIGHT_VALUE 213//240	
#define LOW_BACKLIGHT_VALUE_SONY 31//7//35
#define DIM_BACKLIGHT_VALUE_SONY 8//15	hw requeset cause by backlight ic issue on lowest level
/*For hitachi lcd*/
#define MAX_BACKLIGHT_VALUE_HITACHI 181//216
#define DEF_BACKLIGHT_VALUE_HITACHI  76//91
#define LOW_BACKLIGHT_VALUE_HITACHI  31//29
#define DIM_BACKLIGHT_VALUE_HITACHI   1//17

static int nt35580_spi_write_driver(struct s5p_lcd *lcd, u16 reg)
{
	u16 buf;
	int ret;
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= &buf,
	};

	buf = reg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(lcd->g_spi, &msg);

	if (ret < 0)
		pr_err("%s: error: spi_sync (%d)", __func__, ret);

	return ret;
}

static void nt35580_panel_send_sequence(struct s5p_lcd *lcd,
	const u16 *wbuf)
{
	int i = 0;

	while ((wbuf[i] & DEFMASK) != ENDDEF)
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			nt35580_spi_write_driver(lcd, wbuf[i]);
			i += 1;
		} else {
			msleep(wbuf[i+1]);
			i += 2;
		}
}

#if defined(CONFIG_FB_S3C_MDNIE)
extern Lcd_mDNIe_UI current_mDNIe_UI;

void on_cabc(struct s5p_lcd *lcd)
{
	struct s5p_tft_panel_data *pdata = lcd->data;
	printk(KERN_INFO "on_cabc ...acl = [%d]....ldi...[%d]\n", lcd->acl_enable,lcd->ldi_enable); 
	
	if (lcd->acl_enable == 1 && lcd->ldi_enable == 1) {
		printk(KERN_INFO "mDNIe_MODE = %d\n", current_mDNIe_UI); 

		switch (current_mDNIe_UI) {
		case mDNIe_UI_MODE:		
			if(lcd->cur_cabc != CABC_UI)
			{
				nt35580_panel_send_sequence(lcd, pdata->seq_cabc_ui);
				lcd->cur_cabc = CABC_UI;
				printk(KERN_INFO "set CABC_UI \n");
			}
			 break;
		case mDNIe_VIDEO_MODE:
		case mDNIe_VIDEO_WARM_MODE:
		case mDNIe_VIDEO_COLD_MODE:
		case mDNIe_CAMERA_MODE:
        	case mDNIe_DMB_MODE:
        	case mDNIe_DMB_WARM_MODE:
        	case mDNIe_DMB_COLD_MODE:
        		if(lcd->cur_cabc != CABC_VIDEO)
        		{
				nt35580_panel_send_sequence(lcd, pdata->seq_cabc_video);
			 	lcd->cur_cabc= CABC_VIDEO;
			 	printk(KERN_INFO "set CABC_VIDEO \n");
			}
			 break;
		case mDNIe_NAVI:
			if(lcd->cur_cabc != CABC_IMAGE)
        		{
			 	nt35580_panel_send_sequence(lcd, pdata->seq_cabc_image);
			 	lcd->cur_cabc = CABC_IMAGE;
			 	printk(KERN_INFO "set CABC_IMAGE \n");
			}
			 break;
		default:
			if(lcd->cur_cabc != CABC_UI)
			{
				nt35580_panel_send_sequence(lcd, pdata->seq_cabc_ui);
				lcd->cur_cabc = CABC_UI;
				printk(KERN_INFO "set CABC_UI \n");
			}
			 break;
		}
	}
}

void off_cabc(struct s5p_lcd *lcd)
{
   struct s5p_tft_panel_data *pdata = lcd->data;

   // ACL OFF
   nt35580_panel_send_sequence(lcd, pdata->seq_cabc_off);
   lcd->cur_cabc = CABC_OFF;
   printk("set CABC_OFF \n");
}
#endif

static int get_pwm_value_from_bl(int level,int cur_vendor)
{
   int tune_value;
   
   if(cur_vendor == LCD_HITACHI) {
//      printk("LCD HITACH >>>> Backlight update \n" );
      if(level > MAX_BRIGHTNESS_LEVEL)
         level = MAX_BRIGHTNESS_LEVEL;

      if(level >= LOW_BACKLIGHT_VALUE_HITACHI)
         tune_value = (level - LOW_BACKLIGHT_VALUE_HITACHI) * (MAX_BACKLIGHT_VALUE_HITACHI-LOW_BACKLIGHT_VALUE_HITACHI) 
                      / (MAX_BRIGHTNESS_LEVEL-LOW_BACKLIGHT_VALUE_HITACHI) + LOW_BACKLIGHT_VALUE_HITACHI;
      else if(level > 0)
         tune_value = DIM_BACKLIGHT_VALUE_HITACHI;
      else
         tune_value = level;

      if(tune_value > MAX_BACKLIGHT_VALUE_HITACHI)
         tune_value = MAX_BACKLIGHT_VALUE_HITACHI;
      
   } else {
      if(level >= LOW_BRIGHTNESS_LEVEL)
      	tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE_SONY) 
                     / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_SONY;
      else if(level > 0)
      	tune_value = DIM_BACKLIGHT_VALUE_SONY;
      else
      	tune_value = level;

      if(tune_value > MAX_BACKLIGHT_VALUE)
      	tune_value = MAX_BACKLIGHT_VALUE;
   }
      if(level && !tune_value)
   	   tune_value = 1;
      
   return tune_value;
}


static void update_brightness(struct s5p_lcd *lcd, int level)
{
	struct s5p_tft_panel_data *pdata = lcd->data;

	pdata->brightness_set[pdata->pwm_reg_offset] = 0x100 | (level & 0xff);

	nt35580_panel_send_sequence(lcd, pdata->brightness_set);
}

static void nt35580_ldi_enable(struct s5p_lcd *lcd)
{
	struct s5p_tft_panel_data *pdata = lcd->data;

	mutex_lock(&lcd->lock);

	msleep(NT35580_POWERON_DELAY);

	nt35580_panel_send_sequence(lcd, pdata->seq_set);
	update_brightness(lcd, lcd->bl);
	nt35580_panel_send_sequence(lcd, pdata->display_on);

	if (pdata->backlight_on)
		pdata->backlight_on(1);
#if defined(CONFIG_FB_S3C_MDNIE)
	on_cabc(lcd);
#endif
	lcd->ldi_enable = 1;

	mutex_unlock(&lcd->lock);
//    printk("nt35580_ldi_enable");
    
}

static void nt35580_ldi_disable(struct s5p_lcd *lcd)
{
	struct s5p_tft_panel_data *pdata = lcd->data;

	mutex_lock(&lcd->lock);

	lcd->ldi_enable = 0;
   if(pdata->cur_vendor == LCD_HITACHI) {
//        printk("susend HITACHI Panel \n");
      nt35580_panel_send_sequence(lcd, pdata->sleep_in);
      nt35580_panel_send_sequence(lcd, pdata->display_off);
   } else {
    	nt35580_panel_send_sequence(lcd, pdata->display_off);
    	nt35580_panel_send_sequence(lcd, pdata->sleep_in);
   }
	if (pdata->backlight_on)
		pdata->backlight_on(0);

	mutex_unlock(&lcd->lock);
//    printk("nt35580_ldi_disable \n");
}

static int s5p_bl_update_status(struct backlight_device *bd)
{
	struct s5p_lcd *lcd = bl_get_data(bd);
	int bl = bd->props.brightness;
	int gamma_value;

   pr_debug("\nupdate status brightness %d \n",
				bd->props.brightness);

	if (bl < 0 || bl > 255)
		return -EINVAL;

	mutex_lock(&lcd->lock);

	lcd->bl = bl;

	gamma_value = get_pwm_value_from_bl(lcd->bl,lcd->data->cur_vendor);

	if (lcd->ldi_enable && gamma_value != lcd->current_gamma) {
   pr_debug("\n gamma_value :%d \n", gamma_value);
		update_brightness(lcd, gamma_value);
		lcd->current_gamma = gamma_value;
	}

	mutex_unlock(&lcd->lock);

	return 0;
}

const struct backlight_ops s5p_bl_tft_ops = {
	.update_status = s5p_bl_update_status,
};

void nt35580_early_suspend(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd,
								early_suspend);

	nt35580_ldi_disable(lcd);

	return;
}
void nt35580_late_resume(struct early_suspend *h)
{
	struct s5p_lcd *lcd = container_of(h, struct s5p_lcd,
								early_suspend);

	nt35580_ldi_enable(lcd);

	return;
}

static int s5p_lcd_set_power(struct lcd_device *ld, int power)
{
	struct s5p_lcd *lcd = lcd_get_data(ld);
	struct s5p_tft_panel_data *pdata = lcd->data;

	printk(KERN_DEBUG "s5p_lcd_set_power is called: %d", power);

	if (power)
		nt35580_panel_send_sequence(lcd, pdata->display_on);
	else
		nt35580_panel_send_sequence(lcd, pdata->display_off);

	return 0;
}

static int s5p_lcd_check_fb(struct lcd_device *lcddev, struct fb_info *fi)
{
	return 0;
}

struct lcd_ops s5p_lcd_ops = {
	.set_power = s5p_lcd_set_power,
	.check_fb = s5p_lcd_check_fb,
};

static ssize_t aclset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	
	printk("called %s\n", __func__);

	return sprintf(buf, "%u\n", lcd->acl_enable);
}
static ssize_t aclset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct s5p_lcd *lcd = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);
	
#if defined(CONFIG_FB_S3C_MDNIE)
	if ((lcd->ldi_enable) && ((value == 0) || (value == 1))) {
		printk(KERN_INFO "[acl set] in aclset_file_cmd_store, input value = %d\n", value);
		if (lcd->acl_enable != value) {

			if (value) {				
				lcd->acl_enable = value;
				on_cabc(lcd);
			}
			else {
				lcd->acl_enable = value;
				off_cabc(lcd);
			}
//			lcd->acl_enable = value;
			printk(KERN_INFO "aclset_file_cmd_store ... acl_enable =[%d]",lcd->acl_enable);
			
		}
	}
#endif
	return size;
}

static DEVICE_ATTR(aclset_file_cmd, 0664, aclset_file_cmd_show, aclset_file_cmd_store);

static int __devinit nt35580_probe(struct spi_device *spi)
{
	struct s5p_lcd *lcd;
	int ret;

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	mutex_init(&lcd->lock);

	spi->bits_per_word = 9;
	if (spi_setup(spi)) {
		pr_err("failed to setup spi\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->g_spi = spi;
	lcd->dev = &spi->dev;
	lcd->bl = 255;

	if (!spi->dev.platform_data) {
		dev_err(lcd->dev, "failed to get platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}
	lcd->data = (struct s5p_tft_panel_data *)spi->dev.platform_data;

	if (!lcd->data->seq_set || !lcd->data->display_on ||
		!lcd->data->display_off || !lcd->data->sleep_in ||
		!lcd->data->brightness_set || !lcd->data->pwm_reg_offset) {
		dev_err(lcd->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->bl_dev = backlight_device_register("s5p_bl",
			&spi->dev, lcd, &s5p_bl_tft_ops, NULL);
	if (!lcd->bl_dev) {
		dev_err(lcd->dev, "failed to register backlight\n");
		ret = -EINVAL;
		goto err_setup;
	}

	lcd->lcd_dev = lcd_device_register("s5p_lcd",
			&spi->dev, lcd, &s5p_lcd_ops);
	if (!lcd->lcd_dev) {
		dev_err(lcd->dev, "failed to register lcd\n");
		ret = -EINVAL;
		goto err_setup_lcd;
	}

	lcd->acl_class = class_create(THIS_MODULE, "aclset");
	if (IS_ERR(lcd->acl_class))
		pr_err("Failed to create class(acl_class)!\n");

	lcd->switch_aclset_dev = device_create(lcd->acl_class, &spi->dev, 0, lcd, "switch_aclset");
	if (IS_ERR(lcd->switch_aclset_dev))
		pr_err("Failed to create device(switch_aclset_dev)!\n");

	if (device_create_file(lcd->switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);

	lcd->bl_dev->props.max_brightness = 255;
	spi_set_drvdata(spi, lcd);

	lcd->ldi_enable = 1;
	lcd->acl_enable = 1;
	printk("nt35580_probe ... acl_enable =[%d]",lcd->acl_enable);
	lcd->cur_cabc = CABC_OFF;
	
#if defined(CONFIG_FB_S3C_MDNIE)
	init_mdnie_class(lcd);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->early_suspend.suspend = nt35580_early_suspend;
	lcd->early_suspend.resume = nt35580_late_resume;
	lcd->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&lcd->early_suspend);
#endif
	pr_info("%s successfully probed\n", __func__);

	return 0;

err_setup_lcd:
	backlight_device_unregister(lcd->bl_dev);

err_setup:
	mutex_destroy(&lcd->lock);
	kfree(lcd);

err_alloc:
	return ret;
}

static int __devexit nt35580_remove(struct spi_device *spi)
{
	struct s5p_lcd *lcd = spi_get_drvdata(spi);

	unregister_early_suspend(&lcd->early_suspend);

	backlight_device_unregister(lcd->bl_dev);

	nt35580_ldi_disable(lcd);

	kfree(lcd);

	return 0;
}

static struct spi_driver nt35580_driver = {
	.driver = {
		.name	= "nt35580",
		.owner	= THIS_MODULE,
	},
	.probe		= nt35580_probe,
	.remove		= __devexit_p(nt35580_remove),
};

static int __init nt35580_init(void)
{
	return spi_register_driver(&nt35580_driver);
}
static void __exit nt35580_exit(void)
{
	spi_unregister_driver(&nt35580_driver);
}

module_init(nt35580_init);
module_exit(nt35580_exit);
