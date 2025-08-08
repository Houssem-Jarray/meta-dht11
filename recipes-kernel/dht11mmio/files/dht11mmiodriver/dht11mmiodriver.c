// SPDX-License-Identifier: GPL-2.0
/*
 * dht11-mmio.c
 * DHT11 bit-banged driver using ioremap of Raspberry Pi GPIO block
 *
 * Exposes a character device /dev/dht11 which returns a text line:
 *   "Humidity: x.y %  Temp: a.b C\n"
 *
 * Device tree should provide "reg" for the GPIO block and "gpio-pin" (optional).
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/version.h>

#define DEVICE_NAME "dht11"
#define CLASS_NAME "dht"
#define DEFAULT_GPIO_PIN 4   /* fallback if gpio-pin property missing */
#define GPIO_SIZE 0x1000     /* map at least this much */

#define GPFSEL_OFFSET 0x00
#define GPSET_OFFSET  0x1C
#define GPCLR_OFFSET  0x28
#define GPLEV_OFFSET  0x34

struct dht11_dev {
	struct device *dev;
	void __iomem *gpio_base;
	u32 gpio_pin;
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct mutex lock;
};

static inline u32 gpio_fsel_reg(u32 pin)
{
	return GPFSEL_OFFSET + ((pin / 10) * 4);
}

static inline u32 gpio_set_reg(u32 pin)
{
	return GPSET_OFFSET + ((pin / 32) * 4);
}

static inline u32 gpio_clr_reg(u32 pin)
{
	return GPCLR_OFFSET + ((pin / 32) * 4);
}

static inline u32 gpio_lev_reg(u32 pin)
{
	return GPLEV_OFFSET + ((pin / 32) * 4);
}

static void gpio_set_input(struct dht11_dev *d)
{
	u32 reg = gpio_fsel_reg(d->gpio_pin);
	u32 val = readl(d->gpio_base + reg);

	val &= ~(0x7 << ((d->gpio_pin % 10) * 3));
	writel(val, d->gpio_base + reg);
	/*
	 * A brief delay so the pin mode has time to settle.
	 * The hardware will usually switch immediately, but a short delay
	 * reduces timing surprises.
	 */
	udelay(2);
}

static void gpio_set_output(struct dht11_dev *d)
{
	u32 reg = gpio_fsel_reg(d->gpio_pin);
	u32 val = readl(d->gpio_base + reg);

	val &= ~(0x7 << ((d->gpio_pin % 10) * 3));
	val |= (0x1 << ((d->gpio_pin % 10) * 3)); /* output = 001 */
	writel(val, d->gpio_base + reg);
	udelay(2);
}

static void gpio_write(struct dht11_dev *d, int v)
{
	if (v)
		writel(1u << (d->gpio_pin % 32), d->gpio_base + gpio_set_reg(d->gpio_pin));
	else
		writel(1u << (d->gpio_pin % 32), d->gpio_base + gpio_clr_reg(d->gpio_pin));
}

static int gpio_read(struct dht11_dev *d)
{
	u32 v = readl(d->gpio_base + gpio_lev_reg(d->gpio_pin));
	return !!(v & (1u << (d->gpio_pin % 32)));
}

/* wait for pin to reach value; timeout_us microseconds */
static int wait_for_pin(struct dht11_dev *d, int value, int timeout_us)
{
	while (gpio_read(d) != value) {
		if (--timeout_us <= 0)
			return -ETIMEDOUT;
		udelay(1);
	}
	return 0;
}

/* Read 5 bytes from DHT11 into data[5]; returns 0 on success, negative on error */
static int dht11_read_bytes(struct dht11_dev *d, u8 data[5])
{
	int i, j, err;
	u8 byte;

	memset(data, 0, 5);

	/* Start signal: host pulls line low for >=18 ms */
	gpio_set_output(d);
	gpio_write(d, 0);
	mdelay(20); /* >= 18ms */
	gpio_write(d, 1);
	udelay(30); /* 20-40us */
	gpio_set_input(d);

	/* Sensor response: low ~80us, high ~80us */
	err = wait_for_pin(d, 0, 100); /* wait for initial low from sensor */
	if (err) return err;
	err = wait_for_pin(d, 1, 100); /* then high */
	if (err) return err;
	err = wait_for_pin(d, 0, 100); /* then low - start of data */
	if (err) return err;

	/* Read 40 bits (5 bytes) */
	for (i = 0; i < 5; ++i) {
		byte = 0;
		for (j = 0; j < 8; ++j) {
			/* each bit: ~50us low, then high: short=26-28us => bit=0, long~70us => bit=1 */
			err = wait_for_pin(d, 1, 100); /* wait for start of high */
			if (err) return err;

			udelay(35); /* sample at ~35us into high to distinguish 0/1 */

			byte <<= 1;
			if (gpio_read(d))
				byte |= 1;

			err = wait_for_pin(d, 0, 100); /* wait for next low */
			if (err) return err;
		}
		data[i] = byte;
	}

	/* checksum */
	if ((u8)(data[0] + data[1] + data[2] + data[3]) != data[4])
		return -EBADMSG;

	return 0;
}

/* character device read: perform a single sensor read and return text */
static ssize_t dht11_chr_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct dht11_dev *d = file->private_data;
	u8 data[5];
	char out[64];
	int ret;

	if (!mutex_trylock(&d->lock))
		return -EBUSY;

	/* try up to 3 attempts */
	ret = -EIO;
	if (dht11_read_bytes(d, data) == 0) {
		snprintf(out, sizeof(out), "Humidity: %d.%d %%  Temp: %d.%d C\n",
		         data[0], data[1], data[2], data[3]);
		ret = simple_read_from_buffer(buf, count, ppos, out, strlen(out));
	} else {
		/* allow two retries */
		msleep(100);
		if (dht11_read_bytes(d, data) == 0) {
			snprintf(out, sizeof(out), "Humidity: %d.%d %%  Temp: %d.%d C\n",
			         data[0], data[1], data[2], data[3]);
			ret = simple_read_from_buffer(buf, count, ppos, out, strlen(out));
		} else {
			msleep(100);
			if (dht11_read_bytes(d, data) == 0) {
				snprintf(out, sizeof(out), "Humidity: %d.%d %%  Temp: %d.%d C\n",
				         data[0], data[1], data[2], data[3]);
				ret = simple_read_from_buffer(buf, count, ppos, out, strlen(out));
			} else {
				dev_warn(d->dev, "DHT11: reading failed after retries\n");
				ret = -EIO;
			}
		}
	}

	mutex_unlock(&d->lock);
	return ret;
}

static int dht11_open(struct inode *inode, struct file *file)
{
	struct dht11_dev *d = container_of(inode->i_cdev, struct dht11_dev, cdev);
	file->private_data = d;
	return 0;
}

static const struct file_operations dht11_fops = {
	.owner = THIS_MODULE,
	.open  = dht11_open,
	.read  = dht11_chr_read,
};

static int dht11_probe(struct platform_device *pdev)
{
	struct dht11_dev *d;
	struct resource *res;
	int ret;
	u32 gpio_pin = DEFAULT_GPIO_PIN;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->dev = &pdev->dev;
	mutex_init(&d->lock);

	/* get gpio pin (optional) */
	if (of_property_read_u32(pdev->dev.of_node, "gpio-pin", &gpio_pin))
		gpio_pin = DEFAULT_GPIO_PIN;
	d->gpio_pin = gpio_pin;

	/* map registers from device-tree reg property */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(d->dev, "no memory resource\n");
		return -ENODEV;
	}

	d->gpio_base = devm_ioremap_resource(d->dev, res);
	if (IS_ERR(d->gpio_base)) {
		dev_err(d->dev, "ioremap failed\n");
		return PTR_ERR(d->gpio_base);
	}

	/* allocate char device number */
	ret = alloc_chrdev_region(&d->devt, 0, 1, DEVICE_NAME);
	if (ret) {
		dev_err(d->dev, "alloc_chrdev_region failed\n");
		return ret;
	}

	cdev_init(&d->cdev, &dht11_fops);
	d->cdev.owner = THIS_MODULE;

	ret = cdev_add(&d->cdev, d->devt, 1);
	if (ret) {
		dev_err(d->dev, "cdev_add failed\n");
		goto err_unregister;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	d->class = class_create(CLASS_NAME);
#else
	d->class = class_create(THIS_MODULE, CLASS_NAME);
#endif
	if (IS_ERR(d->class)) {
		ret = PTR_ERR(d->class);
		dev_err(d->dev, "class_create failed\n");
		goto err_cdev_del;
	}

	if (!device_create(d->class, NULL, d->devt, NULL, DEVICE_NAME)) {
		dev_err(d->dev, "device_create failed\n");
		ret = -ENODEV;
		goto err_class;
	}

	platform_set_drvdata(pdev, d);
	dev_info(d->dev, "DHT11 MMIO driver ready on GPIO %u\n", d->gpio_pin);
	return 0;

err_class:
	class_destroy(d->class);
err_cdev_del:
	cdev_del(&d->cdev);
err_unregister:
	unregister_chrdev_region(d->devt, 1);
	return ret;
}

static void dht11_remove(struct platform_device *pdev)
{
	struct dht11_dev *d = platform_get_drvdata(pdev);

	device_destroy(d->class, d->devt);
	class_destroy(d->class);
	cdev_del(&d->cdev);
	unregister_chrdev_region(d->devt, 1);

	// return 0;
}

static const struct of_device_id dht11_of_match[] = {
	{ .compatible = "myvendor,dht11-mmio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dht11_of_match);

static struct platform_driver dht11_platform_driver = {
	.probe = dht11_probe,
	.remove = dht11_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = dht11_of_match,
	},
};
module_platform_driver(dht11_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Houssem");
MODULE_DESCRIPTION("DHT11 driver (MMIO bit-bang) for Raspberry Pi (platform driver)");
MODULE_VERSION("1.0");
