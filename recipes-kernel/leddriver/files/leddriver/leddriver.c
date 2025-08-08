#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#define DEVICE_NAME "leddriver"
#define CLASS_NAME "ledclass"

static int gpio_led = -1;
static dev_t dev_num;
static struct cdev led_cdev;
static struct class *led_class;
static struct device *led_device;

static ssize_t leddriver_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char value;

    if (copy_from_user(&value, buf, 1))
        return -EFAULT;

    if (value == '1') {
        gpio_set_value(gpio_led, 1);
        pr_info("LED: ON\n");
    } else if (value == '0') {
        gpio_set_value(gpio_led, 0);
        pr_info("LED: OFF\n");
    } else {
        pr_info("LED: Invalid input\n");
    }

    return count;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .write = leddriver_write,
};

static int leddriver_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    gpio_led = of_get_named_gpio(dev->of_node, "gpios", 0);
    if (!gpio_is_valid(gpio_led)) {
        dev_err(dev, "Invalid GPIO\n");
        return -EINVAL;
    }

    if (gpio_request(gpio_led, "led_gpio19")) {
        dev_err(dev, "GPIO request failed\n");
        return -EBUSY;
    }

    gpio_direction_output(gpio_led, 0);

    alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    cdev_init(&led_cdev, &fops);
    cdev_add(&led_cdev, dev_num, 1);

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    led_class = class_create(CLASS_NAME);
    #else
        led_class = class_create(THIS_MODULE, CLASS_NAME);
        #endif
    led_device = device_create(led_class, NULL, dev_num, NULL, DEVICE_NAME);

    dev_info(dev, "LED driver probed, /dev/leddriver ready\n");
    return 0;
}

static void leddriver_remove(struct platform_device *pdev)
{
    gpio_set_value(gpio_led, 0);
    gpio_free(gpio_led);
    device_destroy(led_class, dev_num);
    class_destroy(led_class);
    cdev_del(&led_cdev);
    unregister_chrdev_region(dev_num, 1);

    dev_info(&pdev->dev, "LED driver removed\n");
}

static const struct of_device_id leddriver_of_match[] = {
    { .compatible = "houssem,leddriver" },
    {},
};
MODULE_DEVICE_TABLE(of, leddriver_of_match);

static struct platform_driver leddriver_platform_driver = {
    .probe = leddriver_probe,
    .remove = leddriver_remove,
    .driver = {
        .name = "leddriver",
        .of_match_table = leddriver_of_match,
    },
};

module_platform_driver(leddriver_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Houssem");
MODULE_DESCRIPTION("Platform LED driver using device tree");
MODULE_VERSION("1.0");
