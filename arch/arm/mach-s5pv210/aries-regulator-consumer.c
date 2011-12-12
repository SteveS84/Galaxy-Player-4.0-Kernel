/* aries-regulator-consumer.c
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>

static int regulator_control(bool enable)
{
	struct regulator *reg = NULL;

	reg = regulator_get(NULL, "vmem");

	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get %s\n", __func__, "vmem");
		goto out0;
	}

	if (enable)
		regulator_enable(reg);
	else
		regulator_force_disable(reg);

	regulator_put(reg);

out0:
	return 0;
}


static int regulator_consumer_probe(struct platform_device *pdev)
{
	struct regulator *reg = NULL;

	pr_info("%s: loading regulator-consumer\n", __func__);

	reg = regulator_get(NULL, "vmem");
	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get %s\n", __func__, "vmem");
	} else {
		regulator_enable(reg);
		regulator_put(reg);
	}

	return 0;
}

#ifdef CONFIG_PM
static int regulator_consumer_suspend(struct device *dev)
{
	return regulator_control(false);
}

static int regulator_consumer_resume(struct device *dev)
{
	return regulator_control(true);
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops regulator_consumer_pm_ops = {
#ifdef CONFIG_PM
	.suspend = regulator_consumer_suspend,
	.resume = regulator_consumer_resume,
#endif /* CONFIG_PM */
};

static struct platform_driver regulator_consumer_driver = {
	.probe = regulator_consumer_probe,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "regulator-consumer",
		   .pm = &regulator_consumer_pm_ops,
	},
};

static int __init regulator_consumer_init(void)
{
	return platform_driver_register(&regulator_consumer_driver);
}
module_init(regulator_consumer_init);

MODULE_AUTHOR("ms925.kim@samsung.com")
MODULE_DESCRIPTION("regulator consumer driver");
MODULE_LICENSE("GPL");
