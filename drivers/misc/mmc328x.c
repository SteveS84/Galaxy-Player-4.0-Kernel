/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 * 	Dale Hou
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>

#include <linux/i2c/mmc328x.h>
#include <linux/regulator/consumer.h>

#define DEBUG			0
#define MAX_FAILURE_COUNT	3
#define READMD			0

#define MMC328X_DELAY_TM	10	/* ms */
#define MMC328X_DELAY_RM	10	/* ms */
#define MMC328X_DELAY_STDN	1	/* ms */
#define MMC328X_DELAY_RRM       1       /* ms */

#define MMC328X_RETRY_COUNT	3
#define MMC328X_RRM_INTV	100

#define MMC328X_DEV_NAME	"mmc328x"

static u32 read_idx = 0;
struct class *mag_class;

static struct i2c_client *this_client;

static int mmc328x_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
		printk("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mmc328x_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
		printk("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}


/*
 *	MMC328X Sysfs						
 */
static ssize_t mmc328x_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[16] = {0};
	int vec[3] = {0};
	int count;
	int res = 0;

	// Write 0x00000010 to 0x00000000

	data[0] =MMC328X_REG_CTRL;      
	data[1] =MMC328X_CTRL_RM |MMC328X_CTRL_NOBOOST;

	if (mmc328x_i2c_tx_data(data, 2) < 0) {
		return 0;
	}
	/* wait external capacitor charging done for next SET/RESET */
	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */	
	msleep(MMC328X_DELAY_RM);	

	/* [HSS] Charge pump reset before TM command (MMC3280) */
	data[0] =MMC328X_REG_CTRL;        
	data[1] =MMC328X_CTRL_CHARGERST |MMC328X_CTRL_NOBOOST;
	mmc328x_i2c_tx_data(data, 2);

	// Write 0x00000001 to 0x00000000
	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */
      	data[0] =MMC328X_REG_CTRL;      
       	data[1] =MMC328X_CTRL_TM |MMC328X_CTRL_NOBOOST;
	
	res = mmc328x_i2c_tx_data(data, 2);

	/* wait TM done for coming data read */

	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */
	msleep(MMC328X_DELAY_TM);	   
       	data[0] =MMC328X_REG_DATA;      

	if (mmc328x_i2c_rx_data(data, 6) < 0) {
		return 0;
	}
	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */	
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];

#if DEBUG
	printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
		vec[0], vec[1], vec[2]);
#endif

    	printk("x: %d,y: %d,z: %d\n", vec[0], vec[1], vec[2]);
	count = sprintf(buf,"%d,%d,%d\n", vec[0], vec[1], vec[2]);

	return count;
}

static ssize_t mmc328x_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
	printk("%s : input data --> %s\n",__func__, buf);

	return size;
}

static ssize_t mmc328x_power_on(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[16] = {0};	
	int count;
	int res = 0;

	/* [HSS] Charge pump reset before TM command (MMC3280) */
	data[0] =MMC328X_REG_CTRL;        
	data[1] =MMC328X_CTRL_CHARGERST |MMC328X_CTRL_NOBOOST;
	mmc328x_i2c_tx_data(data, 2);

	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */
      	data[0] =MMC328X_REG_CTRL;      
	data[1] =MMC328X_CTRL_TM |MMC328X_CTRL_NOBOOST;

	res = mmc328x_i2c_tx_data(data, 2);

	/* wait TM done for coming data read */
	/* [HSS][Cooper] Magnetic sensor change : MMC3140 -> MMC3280 (since REV0.1) */	
	msleep(MMC328X_DELAY_TM);

#if DEBUG
	printk("[%s] result of i2c writing: %d\n", __func__, !(res < 0));
#endif
	count = sprintf(buf,"%d\n", !(res < 0));

	return count;
}
static DEVICE_ATTR(read_mag, S_IRUGO | S_IWUSR | S_IWGRP, mmc328x_fs_read, mmc328x_fs_write);
static DEVICE_ATTR(power_on, S_IRUGO, mmc328x_power_on, NULL);


static int mmc328x_open(struct inode *inode, struct file *file)
{
	printk("%s\n",__func__);
	return nonseekable_open(inode, file);
}

static int mmc328x_release(struct inode *inode, struct file *file)
{
	printk("%s\n",__func__);
	return 0;
}

static int mmc328x_ioctl(struct inode *inode, struct file *file, 
	unsigned int cmd, unsigned long arg)
{
	void __user *pa = (void __user *)arg;
	unsigned char data[16] = {0};
	int vec[3] = {0};
	//printk("%s\n",__func__);

	switch (cmd) {
	case MMC328X_IOC_TM:
		//printk("%s : %d\n",__func__, 138);
		data[0] = MMC328X_REG_CTRL;
		data[1] = MMC328X_CTRL_TM;
		if (mmc328x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait TM done for coming data read */
		msleep(MMC328X_DELAY_TM);
		break;
	case MMC328X_IOC_RM:
		//printk("%s : %d\n",__func__, 148);
		data[0] = MMC328X_REG_CTRL;
		data[1] = MMC328X_CTRL_RM;
		if (mmc328x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait external capacitor charging done for next SET*/
		msleep(MMC328X_DELAY_RM);
		break;
	case MMC328X_IOC_RRM:
		//printk("%s : %d\n",__func__, 158);
		data[0] = MMC328X_REG_CTRL;
		data[1] = MMC328X_CTRL_RRM;
		if (mmc328x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait external capacitor charging done for next RRM */
		msleep(MMC328X_DELAY_RM);
		break;
	case MMC328X_IOC_READ:
		//printk("%s : %d\n",__func__, 168);
		data[0] = MMC328X_REG_DATA;
		if (mmc328x_i2c_rx_data(data, 6) < 0) {
			return -EFAULT;
		}
		vec[0] = data[1] << 8 | data[0];
		vec[1] = data[3] << 8 | data[2];
		vec[2] = data[5] << 8 | data[4];
	#if DEBUG
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}
		break;
	case MMC328X_IOC_READXYZ:
		//printk("%s : %d\n",__func__, 185);
		/* do RM every MMC328X_RRM_INTV times read */
		if (!(read_idx % MMC328X_RRM_INTV)) {
#ifdef CONFIG_SENSORS_MMC328xMA_MAG 
	            data[0] = MMC328X_REG_CTRL;
	            data[1] = MMC328X_CTRL_RRM;
	            mmc328x_i2c_tx_data(data, 2);
	            msleep(MMC328X_DELAY_RRM);
#endif
			/* RM */
			data[0] = MMC328X_REG_CTRL;
			data[1] = MMC328X_CTRL_RM;
			/* not check return value here, assume it always OK */
			mmc328x_i2c_tx_data(data, 2);
			/* wait external capacitor charging done for next RM */
			msleep(MMC328X_DELAY_RM);
		}
		/* send TM cmd before read */
		data[0] = MMC328X_REG_CTRL;
		data[1] = MMC328X_CTRL_TM;
		/* not check return value here, assume it always OK */
		mmc328x_i2c_tx_data(data, 2);
		/* wait TM done for coming data read */
		msleep(MMC328X_DELAY_TM);
#if READMD
		/* Read MD */
		data[0] = MMC328X_REG_DS;
		if (mmc328x_i2c_rx_data(data, 1) < 0) {
			return -EFAULT;
		}
		while (!(data[0] & 0x01)) {
			msleep(1);
			/* Read MD again*/
			data[0] = MMC328X_REG_DS;
			if (mmc328x_i2c_rx_data(data, 1) < 0) {
				return -EFAULT;
                        }
			
			if (data[0] & 0x01) break;
			MD_times++;
			if (MD_times > 2) {
		#if DEBUG
				printk("TM not work!!");
		#endif
				return -EFAULT;
			}
		}
#endif		
		/* read xyz raw data */
		read_idx++;
		data[0] = MMC328X_REG_DATA;
		if (mmc328x_i2c_rx_data(data, 6) < 0) {
			return -EFAULT;
		}
		vec[0] = data[1] << 8 | data[0];
		vec[1] = data[3] << 8 | data[2];
		vec[2] = data[5] << 8 | data[4];
	#if DEBUG
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}

		break;
	default:
		break;
	}

	return 0;
}

static ssize_t mmc328x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
printk("%s : %d\n",__func__, 261);
	sprintf(buf, "MMC328X");
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(mmc328x, S_IRUGO, mmc328x_show, NULL);

static struct file_operations mmc328x_fops = {
	.owner		= THIS_MODULE,
	.open		= mmc328x_open,
	.release	= mmc328x_release,
	.ioctl		= mmc328x_ioctl,
};

static struct miscdevice mmc328x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MMC328X_DEV_NAME,
	.fops = &mmc328x_fops,
};

static int mmc328x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char data[16] = {0};
	int res = 0;
	struct regulator *reg;

	printk("-----------------------------\n");
	printk("-- mmc328x_probe ----\n");
	printk("-----------------------------\n");
	

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
	}
	
	reg = regulator_get(NULL, "mag_vcc");
	
	if(reg)
		regulator_enable( reg);
	else
		printk("%s : regulator fail\n", __func__);
	
	this_client = client;

	res = misc_register(&mmc328x_device);
	if (res) {
		printk("%s: mmc328x_device register failed\n", __FUNCTION__);
		goto out;
	}
	res = device_create_file(&client->dev, &dev_attr_mmc328x);
	if (res) {
		printk("%s: device_create_file failed\n", __FUNCTION__);
		goto out_deregister;
	}

	/* send SET/RESET cmd to mag sensor first of all */
#ifdef CONFIG_SENSORS_MMC328xMA_MAG 
	data[0] = MMC328X_REG_CTRL;
	data[1] = MMC328X_CTRL_RRM;
	if (mmc328x_i2c_tx_data(data, 2) < 0) {
	}
	msleep(MMC328X_DELAY_RRM);
	data[0] = MMC328X_REG_CTRL;
	data[1] = MMC328X_CTRL_TM;
	if (mmc328x_i2c_tx_data(data, 2) < 0) {
	}
	msleep(5*MMC328X_DELAY_TM);
#endif

	data[0] = MMC328X_REG_CTRL;
	data[1] = MMC328X_CTRL_RM;
	if (mmc328x_i2c_tx_data(data, 2) < 0) {
		printk("%s: Rx Error\n",__func__);
		/* assume RM always success */
	}
#ifndef CONFIG_SENSORS_MMC328xMA_MAG 
	/* wait external capacitor charging done for next RM */
	msleep(MMC328X_DELAY_RM);
#else
	msleep(10*MMC328X_DELAY_RM);
	data[0] = MMC328X_REG_CTRL;
	data[1] = MMC328X_CTRL_TM;
	if (mmc328x_i2c_tx_data(data, 2) < 0) {
		
	}
#endif

	return 0;

out_deregister:
	misc_deregister(&mmc328x_device);
out:
	return res;
}

static int mmc328x_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_mmc328x);
	misc_deregister(&mmc328x_device);

	return 0;
}

static const struct i2c_device_id mmc328x_id[] = {
	{ MMC328X_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver mmc328x_driver = {
	.probe 		= mmc328x_probe,
	.remove 	= mmc328x_remove,
	.id_table	= mmc328x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MMC328X_I2C_NAME,
	},
};


static int __init mmc328x_init(void)
{
	struct device *dev_t;
	pr_info("mmc328x driver: init\n");
	mag_class = class_create(THIS_MODULE, "magnetic");

	if (IS_ERR(mag_class)) 
		return PTR_ERR( mag_class );

	dev_t = device_create( mag_class, NULL, 0, "%s", "magnetic");

	if (device_create_file(dev_t, &dev_attr_read_mag) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_read_mag.attr.name);

	if (device_create_file(dev_t, &dev_attr_power_on) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_power_on.attr.name);

	if (IS_ERR(dev_t)) 
	{
		return PTR_ERR(dev_t);
	}	
	return i2c_add_driver(&mmc328x_driver);
}

static void __exit mmc328x_exit(void)
{
	pr_info("mmc328x driver: exit\n");
	i2c_add_driver(&mmc328x_driver);
		
}

module_init(mmc328x_init);
module_exit(mmc328x_exit);

MODULE_AUTHOR("Dale Hou<byhou@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MMC328X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");

