/*
 * BMA222 accelerometer driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define BMA222_NAME "bma222"

/* Default parameters */
#define BMA222_DEFAULT_DELAY            100
#define BMA222_MAX_DELAY                2000
#define BMA222_COMPENSATION_NUM		10

/* Registers */
#define BMA222_CHIP_ID_REG              0x00
#define BMA222_CHIP_ID                  0x03

#define BMA222_ACC_REG                  0x02

#define BMA222_SOFT_RESET_REG           0x14
#define BMA222_SOFT_RESET_MASK          0xFF
#define BMA222_SOFT_RESET_SHIFT         0

#define BMA222_SUSPEND_REG		0x0A
#define BMA222_SUSPEND_MASK		0x80
#define BMA222_SUSPEND_SHIFT		7

#define BMA222_RANGE_REG                0x0F
#define BMA222_RANGE_MASK               0x18
#define BMA222_RANGE_SHIFT		0
#define BMA222_RANGE_2G			0x11

#define BMA222_BANDWIDTH_REG            0x10
#define BMA222_BANDWIDTH_MASK           0x1F
#define BMA222_BANDWIDTH_SHIFT          0
#define BMA222_BANDWIDTH_0781HZ		0x08
#define BMA222_BANDWIDTH_1563HZ		0x09
#define BMA222_BANDWIDTH_3125HZ		0x0A
#define BMA222_BANDWIDTH_6250HZ		0x0B
#define BMA222_BANDWIDTH_12500HZ	0x0C
#define BMA222_BANDWIDTH_25000HZ	0x0D
#define BMA222_BANDWIDTH_50000HZ	0x0F
#define BMA222_BANDWIDTH_100000HZ	0x10

#define BMA222_CAL_TRIGGER		0x36
#define BMA222_OFFSET_TARGET		0x37
#define BMA222_OFFSET_FILT_X		0x38
#define BMA222_OFFSET_FILT_Y		0x39
#define BMA222_OFFSET_FILT_Z		0x3A

#define BMA222_EEPROM			0x33

/* filter setting */
#define BMA222_FILTER_LEN		8
#define BMA222_STABLE_TH		13

/* ioctl commnad */
#define DCM_IOC_MAGIC			's'
#define BMA222_CALIBRATION		_IOWR(DCM_IOC_MAGIC, 48, short)


/* Acceleration measurement */
struct acceleration {
	short x;
	short y;
	short z;
};

/* Output data rate  */
struct bma222_odr {
	unsigned long delay;	/* min delay (msec) in the range of ODR */
	u8 odr;			/* bandwidth register value */
};

static const struct bma222_odr bma222_odr_table[] = {
	{   1, BMA222_BANDWIDTH_100000HZ },
	{   2, BMA222_BANDWIDTH_50000HZ },
	{   4, BMA222_BANDWIDTH_25000HZ },
	{   8, BMA222_BANDWIDTH_12500HZ },
	{  16, BMA222_BANDWIDTH_6250HZ },
	{  30, BMA222_BANDWIDTH_3125HZ },
	{  60, BMA222_BANDWIDTH_1563HZ },
	{ 120, BMA222_BANDWIDTH_0781HZ },

};

struct bma222_fir_filter {
	int num;
	int filter_len;
	int index;
	int32_t sequence[BMA222_FILTER_LEN];
};

/* driver private data */
struct bma222_data {
	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct miscdevice bma222_device;
	struct bma222_fir_filter filter[3];
};

#define delay_to_jiffies(d) ((d) ? msecs_to_jiffies(d) : 1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/* register access functions */
#define bma222_read_bits(p, r) \
	((i2c_smbus_read_byte_data((p)->client, r##_REG) \
	& r##_MASK) >> r##_SHIFT)

#define bma222_update_bits(p, r, v) \
	i2c_smbus_write_byte_data((p)->client, r##_REG, \
	((i2c_smbus_read_byte_data((p)->client, r##_REG) \
	& ~r##_MASK) | ((v) << r##_SHIFT)))

static void fir_filter_init(struct bma222_fir_filter *filter, int len)
{
	int i;

	filter->num = 0;
	filter->index = 0;
	filter->filter_len = len;

	for (i = 0; i < filter->filter_len; ++i)
		filter->sequence[i] = 0;
}

static s16 fir_filter_filter(struct bma222_fir_filter *filter, s16 in)
{
	int out = 0;
	int i;

	if (filter->filter_len == 0)
		return in;
	if (filter->num < filter->filter_len) {
		filter->sequence[filter->index++] = in;
		filter->num++;
		return in;
	} else {
		if (filter->filter_len <= filter->index)
			filter->index = 0;
		filter->sequence[filter->index++] = in;

		for (i = 0; i < filter->filter_len; i++)
			out += filter->sequence[i];
		return out / filter->filter_len;
	}
}

static void filter_init(struct bma222_data *bma222)
{
	int i;

	for (i = 0; i < 3; i++)
		fir_filter_init(&bma222->filter[i], BMA222_FILTER_LEN);
}

static void filter_filter(struct bma222_data *bma222, s16 *orig, s16 *filtered)
{
	int i;

	for (i = 0; i < 3; i++)
		filtered[i] = fir_filter_filter(&bma222->filter[i], orig[i]);
}

static void filter_stabilizer(struct bma222_data *bma222,
					s16 *orig, s16 *stabled)
{
	int i;
	static s16 buffer[3] = { 0, };

	for (i = 0; i < 3; i++) {
		if ((buffer[i] - orig[i] >= BMA222_STABLE_TH)
			|| (buffer[i] - orig[i] <= -BMA222_STABLE_TH)) {
			stabled[i] = orig[i];
			buffer[i] = stabled[i];
		} else
			stabled[i] = buffer[i];
	}
}

/* Device dependant operations */
static int bma222_power_up(struct bma222_data *bma222)
{
	bma222_update_bits(bma222, BMA222_SUSPEND, 0);
	/* wait 1ms for wake-up time from sleep to operational mode */
	msleep(1);
	return 0;
}

static int bma222_power_down(struct bma222_data *bma222)
{
	bma222_update_bits(bma222, BMA222_SUSPEND, 1);
	return 0;
}

static int bma222_hw_init(struct bma222_data *bma222)
{
	/* reset hardware */
	bma222_power_up(bma222);
	i2c_smbus_write_byte_data(bma222->client,
			  BMA222_SOFT_RESET_REG, 0xB6);

	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);

	bma222_update_bits(bma222, BMA222_RANGE, BMA222_RANGE_2G);
	bma222_power_down(bma222);

	return 0;
}

static int bma222_get_enable(struct device *dev)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);
	return atomic_read(&bma222->enable);
}

static void bma222_set_enable(struct device *dev, int enable)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);
	int delay = atomic_read(&bma222->delay);

	mutex_lock(&bma222->enable_mutex);
	if (enable) { /* enable if state will be changed */
		if (!atomic_cmpxchg(&bma222->enable, 0, 1)) {
			bma222_power_up(bma222);
			schedule_delayed_work(&bma222->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else { /* disable if state will be changed */
		if (atomic_cmpxchg(&bma222->enable, 1, 0)) {
			cancel_delayed_work_sync(&bma222->work);
			bma222_power_down(bma222);
		}
	}
	atomic_set(&bma222->enable, enable);
	mutex_unlock(&bma222->enable_mutex);
}

static int bma222_get_delay(struct device *dev)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);
	return atomic_read(&bma222->delay);
}

static void bma222_set_delay(struct device *dev, int delay)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);
	int i;
	u8 odr;

	/* determine optimum ODR */
	for (i = 1; (i < ARRAY_SIZE(bma222_odr_table)) &&
		     (actual_delay(delay) >= bma222_odr_table[i].delay); i++)
		;
	odr = bma222_odr_table[i-1].odr;
	atomic_set(&bma222->delay, delay);

	mutex_lock(&bma222->enable_mutex);
	if (bma222_get_enable(dev)) {
		cancel_delayed_work_sync(&bma222->work);
		bma222_update_bits(bma222, BMA222_BANDWIDTH, odr);
		schedule_delayed_work(&bma222->work,
				      delay_to_jiffies(delay) + 1);
	} else {
		bma222_power_up(bma222);
		bma222_update_bits(bma222, BMA222_BANDWIDTH, odr);
		bma222_power_down(bma222);
	}
	mutex_unlock(&bma222->enable_mutex);
}

static int bma222_measure(struct bma222_data *bma222,
				struct acceleration *accel)
{
	struct i2c_client *client = bma222->client;
	int err;
	int i;
	s16 raw_data[3];
	/* s16 filtered_data[3]; */
	s16 stabled_data[3];
	u8 buf[6];

	/* read acceleration raw data */
	err = i2c_smbus_read_i2c_block_data(client,
			BMA222_ACC_REG, sizeof(buf), buf) != sizeof(buf);
	if (err < 0) {
		pr_err("%s: i2c read fail addr=0x%02x, len=%d\n",
			__func__, BMA222_ACC_REG, sizeof(buf));
		for (i = 0; i < 3; i++)
			raw_data[i] = 0;
	} else
		for (i = 0; i < 3; i++)
			raw_data[i] = (*(s16 *)&buf[i*2]) >> 6;

	/* filter out sizzling values */
	/* filter_filter(bma222, raw_data, filtered_data); */
	filter_stabilizer(bma222, raw_data, stabled_data);

	accel->x = stabled_data[1];
	accel->y = -stabled_data[0];
	accel->z = stabled_data[2];

	return err;
}

static void bma222_work_func(struct work_struct *work)
{
	struct bma222_data *bma222 = container_of((struct delayed_work *)work,
						  struct bma222_data, work);
	struct acceleration accel;
	unsigned long delay = delay_to_jiffies(atomic_read(&bma222->delay));

	mutex_lock(&bma222->data_mutex);
	bma222_measure(bma222, &accel);
	mutex_unlock(&bma222->data_mutex);

	input_report_rel(bma222->input, REL_X, accel.x);
	input_report_rel(bma222->input, REL_Y, accel.y);
	input_report_rel(bma222->input, REL_Z, accel.z);
	input_sync(bma222->input);

	schedule_delayed_work(&bma222->work, delay);
}

/* Input device interface */
static int bma222_input_init(struct bma222_data *bma222)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = "accelerometer_sensor";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_RY);
	input_set_capability(dev, EV_REL, REL_X);
	input_set_capability(dev, EV_REL, REL_Y);
	input_set_capability(dev, EV_REL, REL_Z);
	input_set_drvdata(dev, bma222);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	bma222->input = dev;

	return 0;
}

static void bma222_input_fini(struct bma222_data *bma222)
{
	struct input_dev *dev = bma222->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/* sysfs device attributes */
static ssize_t bma222_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma222_get_enable(dev));
}

static ssize_t bma222_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	err = strict_strtoul(buf, 10, &enable);

	if ((enable == 0) || (enable == 1))
		bma222_set_enable(dev, enable);

	return count;
}

static ssize_t bma222_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bma222_get_delay(dev));
}

static ssize_t bma222_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err;
	long delay;

	err = strict_strtoul(buf, 10, &delay);
	if (err < 0)
		return count;

	if (delay > BMA222_MAX_DELAY)
		delay = BMA222_MAX_DELAY;

	bma222_set_delay(dev, delay);

	return count;
}

static ssize_t bma222_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_rel(input, REL_RY, atomic_inc_return(&serial));

	return count;
}

static ssize_t bma222_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma222_data *bma222 = input_get_drvdata(input);
	struct acceleration accel;
	int on;

	mutex_lock(&bma222->data_mutex);
	on = bma222_get_enable(dev);
	if (!on)
		bma222_set_enable(dev, 1);
	bma222_measure(bma222, &accel);
	if (!on)
		bma222_set_enable(dev, 0);
	mutex_unlock(&bma222->data_mutex);

	return sprintf(buf, "%d,%d,%d\n", accel.x, accel.y, accel.z);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		   bma222_enable_show, bma222_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		   bma222_delay_show, bma222_delay_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		   NULL, bma222_wake_store);
static DEVICE_ATTR(data, S_IRUGO,
		   bma222_data_show, NULL);

static struct attribute *bma222_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	NULL
};

static struct attribute_group bma222_attribute_group = {
	.attrs = bma222_attributes
};

static int bma222_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int id;

	id = i2c_smbus_read_byte_data(client, BMA222_CHIP_ID_REG);
	if (id != BMA222_CHIP_ID) {
		pr_err("%s: wrong chip id\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int bma222_calibrate(struct bma222_data *bma222)
{
	struct acceleration accel;
	int res = 0;
	int on, backup_odr, i;

	/* turn off bma222 */
	on = bma222_get_enable(&bma222->client->dev);
	if (on)
		bma222_set_enable(&bma222->client->dev, 0);

	mutex_lock(&bma222->data_mutex);
	bma222_power_up(bma222);

	/* save the lastest bandwidth */
	backup_odr = bma222_read_bits(bma222, BMA222_BANDWIDTH);
	if (backup_odr > BMA222_BANDWIDTH_MASK)
		pr_err("%s: backing-up the lastest bandwidth failed(%d)\n",
			__func__, backup_odr);

	/* set the fastest bandwidth */
	res = bma222_update_bits(bma222, BMA222_BANDWIDTH,
				BMA222_BANDWIDTH_100000HZ);
	if (res < 0) {
		mutex_unlock(&bma222->data_mutex);
		pr_err("%s: setting the fastest bandwidth failed\n", __func__);
		return res;
	}

	/* x = 0G, y = 0G, z = 1G */
	res = i2c_smbus_write_byte_data(bma222->client,
					BMA222_OFFSET_TARGET, 0x21);
	if (res < 0) {
		pr_err("%s: target offset setting failed\n", __func__);
		goto done;
	}

	/* repeat for better result */
	for (i = 0; i < BMA222_COMPENSATION_NUM; i++) {
		/* start calibration */
		res = i2c_smbus_write_byte_data(bma222->client,
						BMA222_CAL_TRIGGER, 0x07);
		if (res < 0)
			pr_err("%s: %dth enabling slow compensation failed\n",
				__func__, i);

		/* wait for 16 samplings */
		msleep(100);

		/* stop calbration */
		res = i2c_smbus_write_byte_data(bma222->client,
						BMA222_CAL_TRIGGER, 0x00);
		if (res < 0)
			pr_err("%s: %dth disabling slow compensation failed\n",
				__func__, i);

		/* write compensated offsets to EEPROM */
		res = i2c_smbus_write_byte_data(bma222->client,
					BMA222_EEPROM, 0x03);
		if (res < 0)
			pr_err("%s: %dth EEPROM write unlock and trigger failed\n",
				__func__, i);
	}

	/* blocking EEPROM writing */
	res = i2c_smbus_write_byte_data(bma222->client,
					BMA222_EEPROM, 0x00);
	if (res < 0)
		pr_err("%s: EEPROM write lock failed\n", __func__);

	/* free target offset */
	res = i2c_smbus_write_byte_data(bma222->client,
					BMA222_OFFSET_TARGET, 0x00);
	if (res < 0)
		pr_err("%s: target offset freeing failed\n", __func__);

done:
	/* wait for clean data */
	msleep(300);

	/* dummy read to get rid of wrong data */
	for (i = 0; i < 3; i++) {
		bma222_measure(bma222, &accel);
		input_report_rel(bma222->input, REL_X, accel.x - i);
		input_report_rel(bma222->input, REL_Y, accel.y - i);
		input_report_rel(bma222->input, REL_Z, accel.z - i);
		input_sync(bma222->input);
		msleep(10);
	}

	/* restore back-up bandwidth */
	res = bma222_update_bits(bma222, BMA222_BANDWIDTH, backup_odr);
	if (res < 0)
		pr_err("%s: restoring bandwidth failed\n", __func__);

	bma222_power_down(bma222);
	mutex_unlock(&bma222->data_mutex);

	/* turn on bma222 forcely if it was on */
	if (on)
		bma222_set_enable(&bma222->client->dev, 1);

	return res;
}

static int bma222_open(struct inode *inode, struct file *file)
{
	struct bma222_data *bma222 = container_of(file->private_data,
						struct bma222_data,
						bma222_device);
	file->private_data = bma222;
	return 0;
}

static int bma222_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int bma222_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct bma222_data *bma222 = file->private_data;
	int err = 0;

	switch (cmd) {
	case BMA222_CALIBRATION:
		err = bma222_calibrate(bma222);
		if (err < 0) {
			pr_err("%s: bma222_calibrate failed\n", __func__);
			return err;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct file_operations bma222_fops = {
	.owner = THIS_MODULE,
	.open = bma222_open,
	.release = bma222_close,
	.ioctl = bma222_ioctl,
};

static int bma222_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bma222_data *bma222;
	int err;

	/* setup private data */
	bma222 = kzalloc(sizeof(struct bma222_data), GFP_KERNEL);
	if (!bma222)
		return -ENOMEM;
	mutex_init(&bma222->enable_mutex);
	mutex_init(&bma222->data_mutex);

	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		pr_err("%s: i2c check functionality failed\n", __func__);
		goto err_i2c_fail;
	}
	i2c_set_clientdata(client, bma222);
	bma222->client = client;

	/* detect and init hardware */
	err = bma222_detect(client, NULL);
	if (err) {
		pr_err("%s: bma222 detection failed\n", __func__);
		goto err_id_read;
	}

	bma222_hw_init(bma222);
	bma222_set_delay(&client->dev, BMA222_DEFAULT_DELAY);

	/* setup driver interfaces */
	INIT_DELAYED_WORK(&bma222->work, bma222_work_func);

	err = bma222_input_init(bma222);
	if (err < 0)
		goto err_input_allocate;

	err = sysfs_create_group(&bma222->input->dev.kobj,
					&bma222_attribute_group);
	if (err < 0) {
		pr_err("%s: create sysfs gruop failed\n", __func__);
		goto err_sys_create;
	}

	bma222->bma222_device.minor = MISC_DYNAMIC_MINOR;
	bma222->bma222_device.name = "accelerometer";
	bma222->bma222_device.fops = &bma222_fops;

	err = misc_register(&bma222->bma222_device);
	if (err) {
		pr_err("%s: misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* filter init */
	filter_init(bma222);

	return 0;

err_misc_register:
	sysfs_remove_group(&bma222->input->dev.kobj,
				&bma222_attribute_group);
err_sys_create:
	bma222_input_fini(bma222);
err_input_allocate:
err_id_read:
err_i2c_fail:
	kfree(bma222);
	return err;
}

static int bma222_remove(struct i2c_client *client)
{
	struct bma222_data *bma222 = i2c_get_clientdata(client);

	bma222_set_enable(&client->dev, 0);

	sysfs_remove_group(&bma222->input->dev.kobj, &bma222_attribute_group);
	bma222_input_fini(bma222);
	kfree(bma222);

	return 0;
}

static int bma222_suspend(struct device *dev)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);

	mutex_lock(&bma222->enable_mutex);
	if (bma222_get_enable(dev)) {
		cancel_delayed_work_sync(&bma222->work);
		bma222_power_down(bma222);
	}
	mutex_unlock(&bma222->enable_mutex);

	return 0;
}

static int bma222_resume(struct device *dev)
{
	struct bma222_data *bma222 = dev_get_drvdata(dev);
	int delay = atomic_read(&bma222->delay);

	bma222_hw_init(bma222);
	bma222_set_delay(dev, delay);

	mutex_lock(&bma222->enable_mutex);
	if (bma222_get_enable(dev)) {
		bma222_power_up(bma222);
		schedule_delayed_work(&bma222->work,
				      delay_to_jiffies(delay) + 1);
	}
	mutex_unlock(&bma222->enable_mutex);

	return 0;
}

static const struct dev_pm_ops bma222_pm_ops = {
	.suspend = bma222_suspend,
	.resume = bma222_resume,
};

static const struct i2c_device_id bma222_id[] = {
	{BMA222_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bma222_id);

struct i2c_driver bma222_driver = {
	.driver = {
		.name = "bma222",
		.owner = THIS_MODULE,
		.pm = &bma222_pm_ops,
	},
	.probe = bma222_probe,
	.remove = bma222_remove,
	.id_table = bma222_id,
};

static int __init bma222_init(void)
{
	return i2c_add_driver(&bma222_driver);
}
module_init(bma222_init);

static void __exit bma222_exit(void)
{
	i2c_del_driver(&bma222_driver);
}
module_exit(bma222_exit);

MODULE_DESCRIPTION("BMA222 accelerometer driver");
MODULE_AUTHOR("tim.sk.lee@samsung.com");
MODULE_LICENSE("GPL");
MODULE_VERSION(1.0.0);
