/*include/linux/nt35580.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * Header file for Sony LCD Panel(TFT) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/earlysuspend.h>

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define DEFMASK		0xFF00

typedef enum
{
    LCD_SONY,
    LCD_HYDIS,
    LCD_HITACHI,
}LCD_Vendor;

struct s5p_tft_panel_data {
	const u16 *seq_set;
	const u16 *sleep_in;
	const u16 *display_on;
	const u16 *display_off;
   int        cur_vendor;
	u16 *brightness_set;
	int pwm_reg_offset;
	const u16* seq_cabc_ui;
	const u16* seq_cabc_video;
	const u16* seq_cabc_image;
	const u16* seq_cabc_off;
	void (*backlight_on)(int enable);
};

typedef enum
{
	CABC_OFF,
	CABC_UI,
	CABC_IMAGE,
	CABC_VIDEO,
}LCD_CABC;


struct s5p_lcd {
	int ldi_enable;
	int acl_enable;
	int bl;
	int current_gamma;
	LCD_CABC cur_cabc;
	struct mutex lock;
	struct device *dev;
	struct spi_device *g_spi;
	struct s5p_tft_panel_data *data;
	struct backlight_device *bl_dev;
	struct lcd_device *lcd_dev;
	struct class *acl_class;
	struct device *switch_aclset_dev;
	struct early_suspend early_suspend;
};

