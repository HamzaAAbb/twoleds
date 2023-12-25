#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/err.h>

#define EN_DEBOUNCE
#ifdef EN_DEBOUNCE
#include <linux/jiffies.h>
extern unsigned long volatile jiffies;
unsigned long old_jiffies = 0;
#endif

#define GPIO_21_OUT (21)
#define GPIO_22_OUT (22)
#define GPIO_25_IN (25)
#define GPIO_26_IN (26)

unsigned int led1_toggle = 0;
unsigned int led2_toggle = 0;
unsigned int GPIO_irqNumber1, GPIO_irqNumber2;

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    static unsigned long flags = 0;

#ifdef EN_DEBOUNCE
    unsigned long diff = jiffies - old_jiffies;
    if (diff < 200)
    {
        return IRQ_HANDLED;
    }
    old_jiffies = jiffies;
#endif

    if (irq == GPIO_irqNumber1)
    {
        led1_toggle = (0x01 ^ led1_toggle);
        gpio_set_value(GPIO_21_OUT, led1_toggle);
        pr_info("Interruption survenue : GPIO_21_OUT : %d\n", gpio_get_value(GPIO_21_OUT));
    }
    else if (irq == GPIO_irqNumber2)
    {
        led2_toggle = (0x01 ^ led2_toggle);
        gpio_set_value(GPIO_22_OUT, led2_toggle);
        pr_info("Interruption survenue : GPIO_22_OUT : %d\n", gpio_get_value(GPIO_22_OUT));
    }

    local_irq_restore(flags);
    return IRQ_HANDLED;
}

dev_t dev = 0;
static struct class *dev_class;
static struct cdev sud_cdev;

static int __init sud_driver_init(void);
static void __exit sud_driver_exit(void);

static int sud_open(struct inode *inode, struct file *file);
static int sud_release(struct inode *inode, struct file *file);
static ssize_t sud_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t sud_write(struct file *filp, const char *buf, size_t len, loff_t *off);

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = sud_read,
    .write = sud_write,
    .open = sud_open,
    .release = sud_release,
};

static int sud_open(struct inode *inode, struct file *file)
{
    pr_info("Fichier périphérique ouvert...!!!\n");
    return 0;
}

static int sud_release(struct inode *inode, struct file *file)
{
    pr_info("Fichier périphérique fermé...!!!\n");
    return 0;
}

static ssize_t sud_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    uint8_t gpio_state = 0;
    gpio_state = gpio_get_value(GPIO_21_OUT);
    len = 1;
    if (copy_to_user(buf, &gpio_state, len) > 0)
    {
        pr_err("ERREUR: Certains octets n'ont pas pu être copiés vers l'espace utilisateur\n");
    }
    pr_info("sud_read: GPIO_21 = %d\n", gpio_state);
    return 0;
}

static ssize_t sud_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    uint8_t rec_buf[10] = {0};
    if (copy_from_user(rec_buf, buf, len) > 0)
    {
        pr_err("ERREUR: Certains octets n'ont pas pu être copiés depuis l'espace utilisateur\n");
    }
    pr_info("sud_write : GPIO_21 Set = %c\n", rec_buf[0]);
    if (rec_buf[0] == '1')
    {
        gpio_set_value(GPIO_21_OUT, 1);
    }
    else if (rec_buf[0] == '0')
    {
        gpio_set_value(GPIO_21_OUT, 0);
    }
    else
    {
        pr_err("Commande inconnue : merci de donner soit 1 soit 0\n");
    }
    return len;
}

static int __init sud_driver_init(void)
{
    if (alloc_chrdev_region(&dev, 0, 1, "sud_Dev") < 0)
    {
        pr_err("Impossible d'allouer le nombre Majeur\n");
        goto r_unreg;
    }
    pr_info("Majeur = %d Mineur = %d\n", MAJOR(dev), MINOR(dev));

    cdev_init(&sud_cdev, &fops);

    if (cdev_add(&sud_cdev, dev, 1) < 0)
    {
        pr_err("Impossible d'ajouter le périphérique au système\n");
        goto r_del;
    }

    if (IS_ERR(dev_class = class_create(THIS_MODULE, "sud_class")))
    {
        pr_err("Impossible de créer la structure class\n");
        goto r_class;
    }

    if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "sud_device")))
    {
        pr_err("Impossible de créer le périphérique\n");
        goto r_device;
    }

    if (gpio_is_valid(GPIO_21_OUT) == false)
    {
        pr_err("GPIO %d n'est pas valide\n", GPIO_21_OUT);
        goto r_device;
    }
    if (gpio_request(GPIO_21_OUT, "GPIO_21_OUT") < 0)
    {
        pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_21_OUT);
        goto r_gpio_out;
    }
    gpio_direction_output(GPIO_21_OUT, 0);

    if (gpio_is_valid(GPIO_22_OUT) == false)
    {
        pr_err("GPIO %d n'est pas valide\n", GPIO_22_OUT);
        goto r_gpio_out;
    }
    if (gpio_request(GPIO_22_OUT, "GPIO_22_OUT") < 0)
    {
        pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_22_OUT);
        goto r_gpio_out;
    }
    gpio_direction_output(GPIO_22_OUT, 0);

    if (gpio_is_valid(GPIO_25_IN) == false)
    {
        pr_err("GPIO %d n'est pas valide\n", GPIO_25_IN);
        goto r_gpio_out;
    }
    if (gpio_request(GPIO_25_IN, "GPIO_25_IN") < 0)
    {
        pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_25_IN);
        goto r_gpio_out;
    }
    gpio_direction_input(GPIO_25_IN);

    if (gpio_is_valid(GPIO_26_IN) == false)
    {
        pr_err("GPIO %d n'est pas valide\n", GPIO_26_IN);
        goto r_gpio_in;
    }
    if (gpio_request(GPIO_26_IN, "GPIO_26_IN") < 0)
    {
        pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_26_IN);
        goto r_gpio_in;
    }
    gpio_direction_input(GPIO_26_IN);

    #ifndef EN_DEBOUNCE
    if (gpio_set_debounce(GPIO_25_IN, 200) < 0)
    {
        pr_err("ERREUR: gpio_set_debounce − %d\n", GPIO_25_IN);
    }

    if (gpio_set_debounce(GPIO_26_IN, 200) < 0)
    {
        pr_err("ERREUR: gpio_set_debounce − %d\n", GPIO_26_IN);
    }
    #endif

    GPIO_irqNumber1 = gpio_to_irq(GPIO_25_IN);
    pr_info("GPIO_irqNumber1 = %d\n", GPIO_irqNumber1);

    GPIO_irqNumber2 = gpio_to_irq(GPIO_26_IN);
    pr_info("GPIO_irqNumber2 = %d\n", GPIO_irqNumber2);

    if (request_irq(GPIO_irqNumber1, (void *)gpio_irq_handler, IRQF_TRIGGER_RISING, "sud_device_button1", NULL))
    {
        pr_err("Impossible d’enregistrer l’IRQ pour le bouton-poussoir 1\n");
        goto r_gpio_in;
    }

    if (request_irq(GPIO_irqNumber2, (void *)gpio_irq_handler, IRQF_TRIGGER_RISING, "sud_device_button2", NULL))
    {
        pr_err("Impossible d’enregistrer l’IRQ pour le bouton-poussoir 2\n");
        goto r_gpio_in;
    }

    pr_info("Insertion du pilote de périphérique...OK!!!\n");
    return 0;

r_gpio_in:
    gpio_free(GPIO_26_IN);
r_gpio_out:
    gpio_free(GPIO_22_OUT);
r_gpio_out1:
    gpio_free(GPIO_21_OUT);
r_device:
    device_destroy(dev_class, dev);
r_class:
    class_destroy(dev_class);
r_del:
    cdev_del(&sud_cdev);
r_unreg:
    unregister_chrdev_region(dev, 1);
    return -1;
}

static void __exit sud_driver_exit(void)
{
    free_irq(GPIO_irqNumber1, NULL);
    free_irq(GPIO_irqNumber2, NULL);
    gpio_free(GPIO_26_IN);
    gpio_free(GPIO_25_IN);
    gpio_free(GPIO_22_OUT);
    gpio_free(GPIO_21_OUT);
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&sud_cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Suppression du pilote de périphérique...OK!!!\n");
}

module_init(sud_driver_init);
module_exit(sud_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SUD");
MODULE_DESCRIPTION("Driver avec gestion des interruptions (avec des GPIOs)");
MODULE_VERSION("1.0");
