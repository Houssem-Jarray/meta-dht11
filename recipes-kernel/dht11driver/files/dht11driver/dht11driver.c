// /*
//  * dht11.c - a DHT11 platform device driver
//  */

// #include <linux/module.h>
// #include <linux/fs.h>
// #include <linux/cdev.h>
// #include <linux/device.h>
// #include <linux/uaccess.h>
// #include <linux/delay.h>
// #include <linux/io.h>
// #include <linux/version.h>
// #include <linux/platform_device.h>
// #include <linux/of.h>
// #include <linux/of_gpio.h>
// #include <linux/gpio.h>

// #define DEVICE_NAME "dht11"
// #define CLASS_NAME "dht"

// #define GPIO_BASE_PHYS 0xFE200000 // Physical base address for GPIO (Raspberry Pi 4)
// #define GPIO_SIZE 0x100

// #define GPFSEL_OFFSET 0x00
// #define GPSET_OFFSET 0x1C
// #define GPCLR_OFFSET 0x28
// #define GPLEV_OFFSET 0x34

// struct dht11_data {
//     dev_t dev_num;
//     struct cdev cdev;
//     struct class *class;
//     struct device *device;
//     void __iomem *gpio_base;
//     int gpio_pin;
// };

// static struct dht11_data *dht11_dev;

// static void gpio_set_input(struct dht11_data *data)
// {
//     u32 val = readl(data->gpio_base + GPFSEL_OFFSET + (data->gpio_pin / 10) * 4);
//     val &= ~(0x7 << ((data->gpio_pin % 10) * 3));
//     writel(val, data->gpio_base + GPFSEL_OFFSET + (data->gpio_pin / 10) * 4);
// }

// static void gpio_set_output(struct dht11_data *data)
// {
//     u32 val = readl(data->gpio_base + GPFSEL_OFFSET + (data->gpio_pin / 10) * 4);
//     val &= ~(0x7 << ((data->gpio_pin % 10) * 3));
//     val |= (0x1 << ((data->gpio_pin % 10) * 3));
//     writel(val, data->gpio_base + GPFSEL_OFFSET + (data->gpio_pin / 10) * 4);
// }

// static void gpio_write(struct dht11_data *data, int val)
// {
//     if (val)
//         writel(1 << data->gpio_pin, data->gpio_base + GPSET_OFFSET);
//     else
//         writel(1 << data->gpio_pin, data->gpio_base + GPCLR_OFFSET);
// }

// static int gpio_read(struct dht11_data *data)
// {
//     return !!(readl(data->gpio_base + GPLEV_OFFSET) & (1 << data->gpio_pin));
// }

// static int wait_for_pin(struct dht11_data *data, int value, int timeout_us)
// {
//     while (gpio_read(data) != value && timeout_us-- > 0)
//         udelay(1);
//     if (timeout_us <= 0) {
//         pr_err("DHT11: wait_for_pin() timeout waiting for value %d\n", value);
//         return -1;
//     }
//     return 0;
// }

// static int dht11_read_sensor(struct dht11_data *data, u8 sensor_data[5])
// {
//     int i, j;
//     u8 byte = 0;

//     pr_info("DHT11: Starting read sequence on GPIO %d...\n", data->gpio_pin);

//     memset(sensor_data, 0, 5);

//     // Send start signal
//     pr_info("DHT11: Sending start signal...\n");
//     gpio_set_output(data);
//     gpio_write(data, 0);
//     mdelay(20); // At least 18ms
//     gpio_write(data, 1);
//     udelay(40);
//     gpio_set_input(data);

//     pr_info("DHT11: Waiting for sensor response...\n");

//     // Wait for response from sensor (low, then high, then low)
//     if (wait_for_pin(data, 0, 5000) < 0) {
//         pr_err("DHT11: Timeout waiting for LOW response from sensor\n");
//         return -1;
//     }
//     if (wait_for_pin(data, 1, 5000) < 0) {
//         pr_err("DHT11: Timeout waiting for HIGH response from sensor\n");
//         return -1;
//     }
//     if (wait_for_pin(data, 0, 5000) < 0) {
//         pr_err("DHT11: Timeout waiting for LOW signal before data transmission\n");
//         return -1;
//     }

//     pr_info("DHT11: Reading data bits...\n");

//     // Read 5 bytes
//     for (i = 0; i < 5; i++) {
//         byte = 0;
//         for (j = 0; j < 8; j++) {
//             if (wait_for_pin(data, 1, 1000) < 0) {
//                 pr_err("DHT11: Timeout waiting for bit %d.%d HIGH\n", i, j);
//                 return -1;
//             }

//             udelay(40);  // Increased from 30 to 40us for better stability

//             byte <<= 1;
//             if (gpio_read(data)) {
//                 byte |= 1;
//                 pr_debug("DHT11: Bit %d.%d = 1\n", i, j);
//             } else {
//                 pr_debug("DHT11: Bit %d.%d = 0\n", i, j);
//             }

//             if (wait_for_pin(data, 0, 1000) < 0) {
//                 pr_err("DHT11: Timeout waiting for bit %d.%d LOW\n", i, j);
//                 return -1;
//             }
//         }
//         sensor_data[i] = byte;
//         pr_info("DHT11: Byte %d = 0x%02x\n", i, byte);
//     }

//     // Verify checksum
//     u8 sum = sensor_data[0] + sensor_data[1] + sensor_data[2] + sensor_data[3];
//     if (sum != sensor_data[4]) {
//         pr_err("DHT11: Checksum failed. Sum = 0x%02x, Expected = 0x%02x\n", sum, sensor_data[4]);
//         return -2;
//     }

//     pr_info("DHT11: Data read successfully. H: %d.%d%% T: %d.%d°C\n",
//             sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]);

//     return 0;
// }

// // static int dht11_read_sensor(struct dht11_data *data, u8 sensor_data[5])
// // {
// //     int i, j;
// //     u8 byte = 0;

// //     pr_info("DHT11: Starting read sequence on GPIO %d...\n", data->gpio_pin);

// //     memset(sensor_data, 0, 5);

// //     // Send start signal
// //     pr_info("DHT11: Sending start signal...\n");
// //     gpio_set_output(data);
// //     gpio_write(data, 0);
// //     mdelay(20); // At least 18ms
// //     gpio_write(data, 1);
// //     udelay(40);
// //     gpio_set_input(data);

// //     pr_info("DHT11: Waiting for sensor response...\n");

// //     // Wait for response from sensor (low, then high)
// //     if (wait_for_pin(data, 0, 1000) < 0) {
// //         pr_err("DHT11: Timeout waiting for LOW response from sensor\n");
// //         return -1;
// //     }
// //     if (wait_for_pin(data, 1, 1000) < 0) {
// //         pr_err("DHT11: Timeout waiting for HIGH response from sensor\n");
// //         return -1;
// //     }
// //     if (wait_for_pin(data, 0, 1000) < 0) {
// //         pr_err("DHT11: Timeout waiting for LOW signal before data transmission\n");
// //         return -1;
// //     }

// //     pr_info("DHT11: Reading data bits...\n");

// //     // Read 5 bytes
// //     for (i = 0; i < 5; i++) {
// //         byte = 0;
// //         for (j = 0; j < 8; j++) {
// //             if (wait_for_pin(data, 1, 100) < 0) {
// //                 pr_err("DHT11: Timeout waiting for bit %d.%d HIGH\n", i, j);
// //                 return -1;
// //             }
// //             udelay(30);  // Might need tuning (40us for better stability)
// //             byte <<= 1;
// //             if (gpio_read(data)) {
// //                 byte |= 1;
// //                 pr_info("DHT11: Bit %d.%d = 1\n", i, j);
// //             } else {
// //                 pr_info("DHT11: Bit %d.%d = 0\n", i, j);
// //             }
// //             if (wait_for_pin(data, 0, 100) < 0) {
// //                 pr_err("DHT11: Timeout waiting for bit %d.%d LOW\n", i, j);
// //                 return -1;
// //             }
// //         }
// //         sensor_data[i] = byte;
// //         pr_info("DHT11: Byte %d = 0x%02x\n", i, byte);
// //     }

// //     // Verify checksum
// //     u8 sum = sensor_data[0] + sensor_data[1] + sensor_data[2] + sensor_data[3];
// //     if (sum != sensor_data[4]) {
// //         pr_err("DHT11: Checksum failed. Sum = 0x%02x, Expected = 0x%02x\n", sum, sensor_data[4]);
// //         return -2;
// //     }

// //     pr_info("DHT11: Data read successfully. H: %d.%d%% T: %d.%d°C\n",
// //             sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]);

// //     return 0;
// // }


// // static int dht11_read_sensor(struct dht11_data *data, u8 sensor_data[5])
// // {
// //     int i, j;
// //     u8 byte = 0;

// //     memset(sensor_data, 0, 5);

// //     // Send start signal
// //     gpio_set_output(data);
// //     gpio_write(data, 0);
// //     mdelay(20); // At least 18ms
// //     gpio_write(data, 1);
// //     udelay(40);
// //     gpio_set_input(data);

// //     // Wait for response from sensor (low, then high)
// //     if (wait_for_pin(data, 0, 100) < 0)
// //         return -1;
// //     if (wait_for_pin(data, 1, 100) < 0)
// //         return -1;
// //     if (wait_for_pin(data, 0, 100) < 0)
// //         return -1;

// //     // Read 5 bytes
// //     for (i = 0; i < 5; i++) {
// //         byte = 0;
// //         for (j = 0; j < 8; j++) {
// //             if (wait_for_pin(data, 1, 100) < 0) // wait for high
// //                 return -1;
// //             udelay(30);
// //             byte <<= 1;
// //             if (gpio_read(data))
// //                 byte |= 1;
// //             if (wait_for_pin(data, 0, 100) < 0) // wait for low
// //                 return -1;
// //         }
// //         sensor_data[i] = byte;
// //     }

// //     // Verify checksum
// //     if ((u8)(sensor_data[0] + sensor_data[1] + sensor_data[2] + sensor_data[3]) != sensor_data[4])
// //         return -2;

// //     return 0;
// // }

// static ssize_t dht11_read_file(struct file *file, char __user *buf, size_t count, loff_t *ppos)
// {
//     u8 data[5];
//     char result[64];
//     int ret;

//     if (!dht11_dev)
//         return -ENODEV;

//     ret = dht11_read_sensor(dht11_dev, data);
//     if (ret)
//         return -EIO;

//     snprintf(result, sizeof(result), "Humidity: %d.%d %%  Temp: %d.%d C\n",
//              data[0], data[1], data[2], data[3]);

//     return simple_read_from_buffer(buf, count, ppos, result, strlen(result));
// }

// static struct file_operations fops = {
//     .owner = THIS_MODULE,
//     .read = dht11_read_file,
// };

// static int dht11_probe(struct platform_device *pdev)
// {
//     struct device *dev = &pdev->dev;
//     struct device_node *np = dev->of_node;
//     int ret;
//     int gpio_pin;

//     dev_info(dev, "DHT11 device probed\n");

//     // Get GPIO pin from device tree
//     gpio_pin = of_get_named_gpio(np, "gpios", 0);
//     if (gpio_pin < 0) {
//         dev_err(dev, "Failed to get GPIO from device tree\n");
//         return gpio_pin;
//     }

//     // Allocate device data
//     dht11_dev = devm_kzalloc(dev, sizeof(*dht11_dev), GFP_KERNEL);
//     if (!dht11_dev)
//         return -ENOMEM;

//     dht11_dev->gpio_pin = gpio_pin;

//     // Map GPIO memory
//     dht11_dev->gpio_base = ioremap(GPIO_BASE_PHYS, GPIO_SIZE);
//     if (!dht11_dev->gpio_base) {
//         dev_err(dev, "Failed to ioremap GPIO\n");
//         return -ENOMEM;
//     }

//     // Allocate character device
//     ret = alloc_chrdev_region(&dht11_dev->dev_num, 0, 1, DEVICE_NAME);
//     if (ret) {
//         dev_err(dev, "Failed to allocate chrdev\n");
//         goto err_iounmap;
//     }

//     cdev_init(&dht11_dev->cdev, &fops);
//     dht11_dev->cdev.owner = THIS_MODULE;

//     ret = cdev_add(&dht11_dev->cdev, dht11_dev->dev_num, 1);
//     if (ret) {
//         dev_err(dev, "Failed to add cdev\n");
//         goto err_chrdev;
//     }

// #if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
//     dht11_dev->class = class_create(CLASS_NAME);
// #else
//     dht11_dev->class = class_create(THIS_MODULE, CLASS_NAME);
// #endif
//     if (IS_ERR(dht11_dev->class)) {
//         dev_err(dev, "Failed to create class\n");
//         ret = PTR_ERR(dht11_dev->class);
//         goto err_cdev;
//     }

//     dht11_dev->device = device_create(dht11_dev->class, dev, dht11_dev->dev_num, NULL, DEVICE_NAME);
//     if (IS_ERR(dht11_dev->device)) {
//         dev_err(dev, "Failed to create device\n");
//         ret = PTR_ERR(dht11_dev->device);
//         goto err_class;
//     }

//     platform_set_drvdata(pdev, dht11_dev);

//     dev_info(dev, "DHT11 driver loaded successfully on GPIO %d. Device: /dev/%s\n", 
//              gpio_pin, DEVICE_NAME);
//     return 0;

// err_class:
//     class_destroy(dht11_dev->class);
// err_cdev:
//     cdev_del(&dht11_dev->cdev);
// err_chrdev:
//     unregister_chrdev_region(dht11_dev->dev_num, 1);
// err_iounmap:
//     iounmap(dht11_dev->gpio_base);
//     return ret;
// }

// static void dht11_remove(struct platform_device *pdev)
// {
//     struct dht11_data *data = platform_get_drvdata(pdev);

//     if (data) {
//         device_destroy(data->class, data->dev_num);
//         class_destroy(data->class);
//         cdev_del(&data->cdev);
//         unregister_chrdev_region(data->dev_num, 1);
//         iounmap(data->gpio_base);
//     }

//     dev_info(&pdev->dev, "DHT11 driver removed\n");
//     // return 0;
// }

// static const struct of_device_id dht11_of_match[] = {
//     { .compatible = "myvendor,dht11", },
//     {},
// };

// MODULE_DEVICE_TABLE(of, dht11_of_match);

// static struct platform_driver dht11_driver = {
//     .probe = dht11_probe,
//     .remove = dht11_remove,
//     .driver = {
//         .name = "dht11",
//         .of_match_table = dht11_of_match,
//     },
// };

// static int __init dht11_init(void)
// {
//     return platform_driver_register(&dht11_driver);
// }

// static void __exit dht11_exit(void)
// {
//     platform_driver_unregister(&dht11_driver);
// }

// module_init(dht11_init);
// module_exit(dht11_exit);

// MODULE_LICENSE("GPL");
// MODULE_AUTHOR("HOUSSEM JARRAY");
// MODULE_DESCRIPTION("DHT11 Platform Driver for Raspberry Pi 4");
// MODULE_VERSION("1.0");
// MODULE_ALIAS("platform:dht11");

// // SPDX-License-Identifier: GPL-2.0
// #include <linux/module.h>
// #include <linux/platform_device.h>
// #include <linux/gpio/consumer.h>
// #include <linux/interrupt.h>
// #include <linux/completion.h>
// #include <linux/fs.h>
// #include <linux/cdev.h>
// #include <linux/uaccess.h>
// #include <linux/ktime.h>
// #include <linux/of.h>
// #include <linux/of_device.h>
// #include <linux/version.h>
// #include <linux/delay.h>

// #define DEVICE_NAME "dht11"
// #define CLASS_NAME "dht"
// #define DHT11_EDGES 83
// #define DHT11_THRESHOLD_NS 49000

// struct dht11_data {
//     struct device *dev;
//     struct gpio_desc *gpiod;
//     int irq;

//     struct completion completion;
//     int num_edges;
//     struct { s64 ts; int value; } edges[DHT11_EDGES];

//     dev_t devt;
//     struct cdev cdev;
//     struct class *class;
// };

// static irqreturn_t dht11_irq_handler(int irq, void *dev_id)
// {
//     struct dht11_data *data = dev_id;

//     if (data->num_edges < DHT11_EDGES && data->num_edges >= 0) {
//         data->edges[data->num_edges].ts = ktime_get_boottime_ns();
//         data->edges[data->num_edges++].value = gpiod_get_value(data->gpiod);

//         if (data->num_edges >= DHT11_EDGES)
//             complete(&data->completion);
//     }

//     return IRQ_HANDLED;
// }

// static int dht11_read_raw(struct dht11_data *data, u8 buf[5])
// {
//     int i, j, t, ret;
//     char bits[40];

//     reinit_completion(&data->completion);
//     data->num_edges = 0;

//     gpiod_direction_output(data->gpiod, 0);
//     usleep_range(18000, 20000);
//     gpiod_direction_input(data->gpiod);

//     ret = request_irq(data->irq, dht11_irq_handler,
//                 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
//                 DEVICE_NAME, data);
//     if (ret)
//         return ret;

//     if (!wait_for_completion_timeout(&data->completion, HZ)) {
//         free_irq(data->irq, data);
//         return -ETIMEDOUT;
//     }

//     free_irq(data->irq, data);

//     for (i = 0; i < 40; ++i) {
//         t = data->edges[2 * i + 2].ts - data->edges[2 * i + 1].ts;
//         if (!data->edges[2 * i + 1].value)
//             return -EIO;
//         bits[i] = t > DHT11_THRESHOLD_NS;
//     }

//     for (i = 0; i < 5; ++i) {
//         buf[i] = 0;
//         for (j = 0; j < 8; ++j)
//             buf[i] = (buf[i] << 1) | bits[i * 8 + j];
//     }

//     if ((buf[0] + buf[1] + buf[2] + buf[3]) != buf[4])
//         return -EIO;

//     return 0;
// }

// static ssize_t dht11_read_file(struct file *file, char __user *buf, size_t count, loff_t *ppos)
// {
//     struct dht11_data *data = file->private_data;
//     u8 raw[5];
//     char result[64];
//     int ret;

//     ret = dht11_read_raw(data, raw);
//     if (ret)
//         return -EIO;

//     snprintf(result, sizeof(result), "Humidity: %d.%d %%  Temp: %d.%d C\n",
//              raw[0], raw[1], raw[2], raw[3]);

//     return simple_read_from_buffer(buf, count, ppos, result, strlen(result));
// }

// static int dht11_open(struct inode *inode, struct file *file)
// {
//     file->private_data = container_of(inode->i_cdev, struct dht11_data, cdev);
//     return 0;
// }

// static const struct file_operations dht11_fops = {
//     .owner = THIS_MODULE,
//     .open = dht11_open,
//     .read = dht11_read_file,
// };

// static int dht11_probe(struct platform_device *pdev)
// {
//     struct dht11_data *data;
//     int ret;

//     data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
//     if (!data)
//         return -ENOMEM;

//     data->dev = &pdev->dev;
//     platform_set_drvdata(pdev, data);

//     data->gpiod = devm_gpiod_get(&pdev->dev, NULL, GPIOD_IN);
//     if (IS_ERR(data->gpiod))
//         return PTR_ERR(data->gpiod);

//     data->irq = gpiod_to_irq(data->gpiod);
//     if (data->irq < 0)
//         return data->irq;

//     init_completion(&data->completion);

//     ret = alloc_chrdev_region(&data->devt, 0, 1, DEVICE_NAME);
//     if (ret)
//         return ret;

//     cdev_init(&data->cdev, &dht11_fops);
//     data->cdev.owner = THIS_MODULE;

//     ret = cdev_add(&data->cdev, data->devt, 1);
//     if (ret)
//         goto unregister_chrdev;
// #if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
//     // dht11_dev->class = class_create(CLASS_NAME);
//         data->class = class_create(CLASS_NAME);
// #else
//     // dht11_dev->class = class_create(THIS_MODULE, CLASS_NAME);
//         data->class = class_create(THIS_MODULE, CLASS_NAME);
// #endif

//     // data->class = class_create(THIS_MODULE, CLASS_NAME);
//     if (IS_ERR(data->class)) {
//         ret = PTR_ERR(data->class);
//         goto del_cdev;
//     }

//     device_create(data->class, NULL, data->devt, NULL, DEVICE_NAME);
//     dev_info(&pdev->dev, "DHT11 driver loaded on GPIO\n");
//     return 0;

// del_cdev:
//     cdev_del(&data->cdev);
// unregister_chrdev:
//     unregister_chrdev_region(data->devt, 1);
//     return ret;
// }

// static void dht11_remove(struct platform_device *pdev)
// {
//     struct dht11_data *data = platform_get_drvdata(pdev);

//     device_destroy(data->class, data->devt);
//     class_destroy(data->class);
//     cdev_del(&data->cdev);
//     unregister_chrdev_region(data->devt, 1);
//     // return 0;
// }

// static const struct of_device_id dht11_of_match[] = {
//     { .compatible = "myvendor,dht11" },
//     {},
// };
// MODULE_DEVICE_TABLE(of, dht11_of_match);

// static struct platform_driver dht11_driver = {
//     .driver = {
//         .name = DEVICE_NAME,
//         .of_match_table = dht11_of_match,
//     },
//     .probe = dht11_probe,
//     .remove = dht11_remove,
// };
// module_platform_driver(dht11_driver);

// MODULE_LICENSE("GPL");
// MODULE_AUTHOR("Houssem + Copilot");
// MODULE_DESCRIPTION("Interrupt-driven DHT11 driver with character device");



// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define DEVICE_NAME "dht11"
#define CLASS_NAME "dht"
#define DHT11_EDGES 85
#define DHT11_THRESHOLD_NS 49000

struct dht11_data {
    struct device *dev;
    struct gpio_desc *gpiod;
    int irq;

    struct completion completion;
    int num_edges;
    struct { s64 ts; int value; } edges[DHT11_EDGES];

    dev_t devt;
    struct cdev cdev;
    struct class *class;

    struct mutex lock;
};

static irqreturn_t dht11_irq_handler(int irq, void *dev_id)
{
    struct dht11_data *data = dev_id;

    if (data->num_edges < DHT11_EDGES && data->num_edges >= 0) {
        data->edges[data->num_edges].ts = ktime_get_boottime_ns();
        data->edges[data->num_edges++].value = gpiod_get_value(data->gpiod);

        if (data->num_edges >= DHT11_EDGES)
            complete(&data->completion);
    }

    return IRQ_HANDLED;
}

static int dht11_read_raw(struct dht11_data *data, u8 buf[5])
{
    int i, j, ret, attempt;
    char bits[40];
    const int offset = 3;  // Skip handshake edges
    const int threshold_ns = 45000;

    mutex_lock(&data->lock);

    for (attempt = 0; attempt < 3; attempt++) {
        reinit_completion(&data->completion);
        data->num_edges = 0;

        gpiod_direction_output(data->gpiod, 0);
        usleep_range(18000, 20000);
        gpiod_direction_input(data->gpiod);
        usleep_range(10, 20);  // Let line stabilize

        ret = request_irq(data->irq, dht11_irq_handler,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          DEVICE_NAME, data);
        if (ret)
            break;

        if (!wait_for_completion_timeout(&data->completion, HZ)) {
            free_irq(data->irq, data);
            dev_warn(data->dev, "DHT11 read timed out\n");
            continue;
        }

        free_irq(data->irq, data);

        if (data->num_edges < DHT11_EDGES) {
            dev_warn(data->dev, "Insufficient edges captured: %d\n", data->num_edges);
            continue;
        }

        for (i = 0; i < 40; ++i) {
            int idx = 2 * i + offset;
            if (idx + 2 >= DHT11_EDGES) {
                dev_warn(data->dev, "Edge index out of bounds at bit %d\n", i);
                ret = -EIO;
                goto unlock;
            }

            s64 low_ts = data->edges[idx + 1].ts;
            s64 high_ts = data->edges[idx + 2].ts;
            int value = data->edges[idx + 1].value;

            if (!value) {
                dev_warn(data->dev, "Invalid edge value at bit %d\n", i);
                ret = -EIO;
                goto unlock;
            }

            bits[i] = (high_ts - low_ts) > threshold_ns;

            dev_info(data->dev, "Bit %d: Δt = %lld ns\n", i, high_ts - low_ts);
        }

        for (i = 0; i < 5; ++i) {
            buf[i] = 0;
            for (j = 0; j < 8; ++j)
                buf[i] = (buf[i] << 1) | bits[i * 8 + j];
        }

        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            ret = 0;
            goto unlock;
        }

        dev_warn(data->dev, "Checksum mismatch: %02x + %02x + %02x + %02x != %02x\n",
                 buf[0], buf[1], buf[2], buf[3], buf[4]);
        msleep(100);  // Retry delay
    }

    ret = -EIO;

unlock:
    mutex_unlock(&data->lock);
    return ret;
}


// static int dht11_read_raw(struct dht11_data *data, u8 buf[5])
// {
//     int i, j, ret;
//     char bits[40];

//     mutex_lock(&data->lock);

//     reinit_completion(&data->completion);
//     data->num_edges = 0;

//     gpiod_direction_output(data->gpiod, 0);
//     usleep_range(18000, 20000);
//     gpiod_direction_input(data->gpiod);

//     ret = request_irq(data->irq, dht11_irq_handler,
//                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
//                       DEVICE_NAME, data);
//     if (ret) {
//         mutex_unlock(&data->lock);
//         return ret;
//     }

//     if (!wait_for_completion_timeout(&data->completion, HZ)) {
//         free_irq(data->irq, data);
//         dev_warn(data->dev, "DHT11 read timed out\n");
//         mutex_unlock(&data->lock);
//         return -ETIMEDOUT;
//     }

//     free_irq(data->irq, data);

//     if (data->num_edges < DHT11_EDGES) {
//         dev_warn(data->dev, "Insufficient edges captured: %d\n", data->num_edges);
//         mutex_unlock(&data->lock);
//         return -EIO;
//     }

//     for (i = 0; i < 40; ++i) {
//         s64 low_ts = data->edges[2 * i + 1].ts;
//         s64 high_ts = data->edges[2 * i + 2].ts;
//         int value = data->edges[2 * i + 1].value;

//         if (!value) {
//             dev_warn(data->dev, "Invalid edge value at bit %d\n", i);
//             mutex_unlock(&data->lock);
//             return -EIO;
//         }

//         bits[i] = (high_ts - low_ts) > DHT11_THRESHOLD_NS;
//     }

//     for (i = 0; i < 5; ++i) {
//         buf[i] = 0;
//         for (j = 0; j < 8; ++j)
//             buf[i] = (buf[i] << 1) | bits[i * 8 + j];
//     }

//     if ((buf[0] + buf[1] + buf[2] + buf[3]) != buf[4]) {
//         dev_warn(data->dev, "Checksum mismatch: %02x + %02x + %02x + %02x != %02x\n",
//                  buf[0], buf[1], buf[2], buf[3], buf[4]);
//         mutex_unlock(&data->lock);
//         return -EIO;
//     }

//     mutex_unlock(&data->lock);
//     return 0;
// }

static ssize_t dht11_read_file(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct dht11_data *data = file->private_data;
    u8 raw[5];
    char result[64];
    int ret;

    ret = dht11_read_raw(data, raw);
    if (ret)
        return ret;

    snprintf(result, sizeof(result), "Humidity: %d.%d %%  Temp: %d.%d C\n",
             raw[0], raw[1], raw[2], raw[3]);

    return simple_read_from_buffer(buf, count, ppos, result, strlen(result));
}

static int dht11_open(struct inode *inode, struct file *file)
{
    file->private_data = container_of(inode->i_cdev, struct dht11_data, cdev);
    return 0;
}

static const struct file_operations dht11_fops = {
    .owner = THIS_MODULE,
    .open = dht11_open,
    .read = dht11_read_file,
};

static int dht11_probe(struct platform_device *pdev)
{
    struct dht11_data *data;
    int ret;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->dev = &pdev->dev;
    platform_set_drvdata(pdev, data);

    data->gpiod = devm_gpiod_get(&pdev->dev, NULL, GPIOD_IN);
    if (IS_ERR(data->gpiod))
        return PTR_ERR(data->gpiod);

    data->irq = gpiod_to_irq(data->gpiod);
    if (data->irq < 0)
        return data->irq;

    init_completion(&data->completion);
    mutex_init(&data->lock);

    ret = alloc_chrdev_region(&data->devt, 0, 1, DEVICE_NAME);
    if (ret)
        return ret;

    cdev_init(&data->cdev, &dht11_fops);
    data->cdev.owner = THIS_MODULE;

    ret = cdev_add(&data->cdev, data->devt, 1);
    if (ret)
        goto unregister_chrdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    data->class = class_create(CLASS_NAME);
#else
    data->class = class_create(THIS_MODULE, CLASS_NAME);
#endif

    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        goto del_cdev;
    }

    device_create(data->class, NULL, data->devt, NULL, DEVICE_NAME);
    dev_info(&pdev->dev, "DHT11 driver loaded on GPIO\n");
    return 0;

del_cdev:
    cdev_del(&data->cdev);
unregister_chrdev:
    unregister_chrdev_region(data->devt, 1);
    return ret;
}

static void dht11_remove(struct platform_device *pdev)
{
    struct dht11_data *data = platform_get_drvdata(pdev);

    device_destroy(data->class, data->devt);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->devt, 1);
}

static const struct of_device_id dht11_of_match[] = {
    { .compatible = "myvendor,dht11" },
    {},
};
MODULE_DEVICE_TABLE(of, dht11_of_match);

static struct platform_driver dht11_driver = {
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = dht11_of_match,
    },
    .probe = dht11_probe,
    .remove = dht11_remove,
};
module_platform_driver(dht11_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Houssem + Copilot");
MODULE_DESCRIPTION("Robust interrupt-driven DHT11 driver with character device");
