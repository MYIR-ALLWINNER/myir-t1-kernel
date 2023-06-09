/*
 * drivers/pwm/pwm-sunxi-dev.c
 *
 * Allwinnertech pulse-width-modulation controller driver
 *
 * Copyright (C) 2019 AllWinner
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <asm/io.h>
#include <linux/pwm.h>

#define PWM_ERR(fmt, arg...) pr_err("%s()%d - "fmt, __func__, __LINE__, ##arg)

#define PWM_IOCTL_BASE 'P'
#define GROUP_PWM_CONFIG	_IOW(PWM_IOCTL_BASE, 4, struct pwm_config_group)
#define GROUP_PWM_DISABLE   _IOW(PWM_IOCTL_BASE, 5, struct pwm_config_group)

struct pwm_config_group {
	int group_channel;
	int group_run_count;
	int pwm_polarity;
	int pwm_period;
};

struct sunxi_pwm_dev {
	struct device *dev;
	struct cdev cdev;
	dev_t chrdev;
};

static struct sunxi_pwm_dev    *sunxi_pwm_dev;
static struct class            *sunxi_pwm_class;

static int sunxi_pwm_open(struct inode *inode, struct file *filp)
{
	filp->private_data = sunxi_pwm_dev;
	return 0;
}

static int sunxi_pwm_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long  sunxi_pwm_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	unsigned int size;
	struct pwm_config_group *code_group;
	unsigned int ret, i, group;
	unsigned char name[30];

	static struct pwm_device *pwm[8] = {NULL};
	switch (cmd) {
	case GROUP_PWM_CONFIG:
		size = _IOC_SIZE(cmd);

		code_group = (struct pwm_config_group *)kzalloc(size, GFP_KERNEL);
		if (IS_ERR_OR_NULL(code_group)) {
			PWM_ERR("not enough memory\n");
			return -ENOMEM;
		}

		if (copy_from_user(code_group, (void __user *)arg, size)) {
			PWM_ERR("copy buffer err\n");
			return -ENOMEM;
		}

		group = code_group->group_channel;

		if (group < 1) {
			return -EINVAL;
		}

		for (i = 4*(group-1); i < 4*group; i++) {
			sprintf(name, "sunxi_pwm%d", i);
			if (pwm[i] == NULL) {
				pwm[i] = pwm_request(i, name);

				if (IS_ERR_OR_NULL(pwm[i])) {
					PWM_ERR("pwm err\n");
					return -ENODEV;
				}
			}

			pwm[i]->chip_data = code_group;

//			pwm_disable(pwm[i]);  /* first disabled then enable */

			ret = pwm_config(pwm[i], 0, 1); /* the argument can’t be same as the first */
			if (ret < 0) {
				PWM_ERR("pwm ioctl err0\n");
				return -EINVAL;
			}
			ret = pwm_config(pwm[i], 0x2ee, 0x7cf);
			if (ret < 0) {
				PWM_ERR("pwm ioctl err\n");
				return -EINVAL;
			}

			pwm_enable(pwm[i]);
			/*pwm_free(pwm);*/
		}

		kfree(code_group);
		break;
	case GROUP_PWM_DISABLE:
		size = _IOC_SIZE(cmd);

		code_group = (struct pwm_config_group *)kzalloc(size, GFP_KERNEL);
		if (IS_ERR_OR_NULL(code_group)) {
			PWM_ERR("not enough memory\n");
			return -ENOMEM;
		}

		if (copy_from_user(code_group, (void __user *)arg, size)) {
			PWM_ERR("copy buffer err\n");
			return -ENOMEM;
		}

		group = code_group->group_channel;

		if (group < 1) {
			PWM_ERR("group para err\n");
			return -EINVAL;
		}

		for (i = 4*(group-1); i < 4*group; i++) {
			pwm[i]->chip_data = code_group;

			if (pwm[i]) {
				pwm_disable(pwm[i]);
				pwm_free(pwm[i]);
				pwm[i] = NULL;
			}
		}


		kfree(code_group);
		break;
	default:
		PWM_ERR("a err cmd");
		return -ENOTTY;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long  sunxi_pwm_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	unsigned long translated_arg = (unsigned long)compat_ptr(arg);

	return sunxi_pwm_unlocked_ioctl(filp, cmd, translated_arg);
}
#endif

static const struct file_operations sunxi_pwm_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= sunxi_pwm_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = sunxi_pwm_compat_ioctl,
#endif
	.open		= sunxi_pwm_open,
	.release	= sunxi_pwm_release,
};

static int __init sunxi_pwm_init(void)
{
	int err = 0;
	struct device *dev;

	sunxi_pwm_dev = kzalloc(sizeof(struct sunxi_pwm_dev), GFP_KERNEL);
	if (sunxi_pwm_dev == NULL) {
		PWM_ERR("kzalloc failed!\n");
		return -ENOMEM;
	}

	err = alloc_chrdev_region(&sunxi_pwm_dev->chrdev, 0, 1, "sunxi-pwm-dev");

	if (err) {
		PWM_ERR("alloc_chrdev_region failed!\n");
		goto alloc_chrdev_err;
	}

	cdev_init(&(sunxi_pwm_dev->cdev), &sunxi_pwm_fops);
	sunxi_pwm_dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&(sunxi_pwm_dev->cdev), sunxi_pwm_dev->chrdev, 1);
	if (err) {
		PWM_ERR("cdev_add failed!\n");
		goto cdev_add_err;
	}

	sunxi_pwm_class = class_create(THIS_MODULE, "sunxi_pwm_char_class");
	if (IS_ERR(sunxi_pwm_class)) {
		err = PTR_ERR(sunxi_pwm_class);
		PWM_ERR("class_create failed!\n");
		goto class_err;
	}

	dev = device_create(sunxi_pwm_class, NULL, sunxi_pwm_dev->chrdev, NULL,
			"sunxi_pwm%d", 0);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		PWM_ERR("device_create failed!\n");
		goto device_err;
	}

	return 0;

device_err:
	device_destroy(sunxi_pwm_class, sunxi_pwm_dev->chrdev);
class_err:
	cdev_del(&(sunxi_pwm_dev->cdev));
cdev_add_err:
	unregister_chrdev_region(sunxi_pwm_dev->chrdev, 1);
alloc_chrdev_err:
	kfree(sunxi_pwm_dev);

	return err;
}

static void __exit sunxi_pwm_exit(void)
{
	cdev_del(&(sunxi_pwm_dev->cdev));
	unregister_chrdev_region(sunxi_pwm_dev->chrdev, 1);
	device_destroy(sunxi_pwm_class, sunxi_pwm_dev->chrdev);
	class_destroy(sunxi_pwm_class);
	kfree(sunxi_pwm_dev);
}

module_init(sunxi_pwm_init);
module_exit(sunxi_pwm_exit);
MODULE_AUTHOR("Li huaxing");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SUNXI_PWM char");
