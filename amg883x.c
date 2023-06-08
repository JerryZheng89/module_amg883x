#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/fs.h>
#include <linux/regmap.h>

#include "amg883x.h"

#define AMG883X_DEFAULT_FPS 10
#define AMG883X_CLASS "amg883x-class"

#define AMG883x_OPERA_MODE	0x00
#define AMG883x_RESET_REG	0x01
#define AMG883x_FRAME_RATE	0x02
#define AMG883x_INT_CTRL	0x03
#define AMG883x_STATUS		0x04
#define AMG883x_STATUS_CLEAR	0x05
#define AMG883x_INTHL		0x08
#define AMG883x_INTHH		0x09
#define AMG883x_INTLL		0x0A
#define AMG883x_INTLH		0x0B
#define AMG883x_IHYSL		0x0C
#define AMG883x_IHYSH		0x0D
#define AMG883x_TEMPERATURE	0x0E
#define AMG883x_INT_TABLE	0x10
#define AMG883x_INT_TABLE_CNT	0x08
#define AMG883x_PIXEL_VALUE	0x80
#define AMG883x_PIXEL_CNT	64


static struct class *amg883x_class;
static dev_t dev_num;

struct amg883x {
	struct i2c_client *client;
	struct gpio_desc *int_gpio; /* for interrupt */
	struct cdev cdev;
	int fps;
};

static int amg883x_irq_process(struct amg883x *amg883x)
{
	struct device *dev = &amg883x->client->dev;
	struct i2c_client *client = amg883x->client;
	struct amg883x_write_data *wr_buf;
	s32 value;
	int ret;
	int i;

	wr_buf = kmalloc(sizeof(struct amg883x_write_data), 
			      GFP_KERNEL);

	value = i2c_smbus_read_byte_data(client, AMG883x_STATUS);
	dev_info(dev, "irq flag 0x%02x\n", value);
	value = i2c_smbus_read_byte_data(client, AMG883x_INT_CTRL);

	if ((value&0x02) == 0x00) {
		ret = i2c_smbus_write_byte_data(client, AMG883x_INT_CTRL, 0x03);
	}

	dev_info(dev, "irq control 0x%02x\n", value);
	value = i2c_smbus_read_byte_data(client, AMG883x_INTHL);
	dev_info(dev, "INT_LVL_H_L 0x%02x\n", value);

	if (value < 0x80) {
		ret = i2c_smbus_write_byte_data(client, AMG883x_INTHL, 0x80);
		ret = i2c_smbus_write_byte_data(client, AMG883x_INTHH, 0x00);
		ret = i2c_smbus_write_byte_data(client, AMG883x_IHYSL, 0x00);
		ret = i2c_smbus_write_byte_data(client, AMG883x_IHYSH, 0x00);
	}

	value = i2c_smbus_read_byte_data(client, AMG883x_INTLL);
	dev_info(dev, "INT_LVL_L_L 0x%02x\n", value);

	for (i = 0; i < AMG883x_INT_TABLE_CNT; i++) {
		value = i2c_smbus_read_byte_data(client, AMG883x_INT_TABLE + i);
		dev_info(dev, "INT TABLE%d %2x\n", i, value);
	}

	if (value < 0) {
		ret = value;
		dev_err(dev, "failed to read data");
		goto un_alloc;
	}

	wr_buf->clear_reg = 0;
	wr_buf->clear_reg_bit.int_clear = 1;

	/* clear interrupt flag */
	ret = i2c_smbus_write_byte_data(client, AMG883x_STATUS_CLEAR, 
					wr_buf->clear_reg);

un_alloc:
	kfree(wr_buf);
	return ret;
}

static irqreturn_t amg883x_irq_handler(int irq, void *devid)
{
	struct amg883x *amg883x = devid;
	int ret; 

	ret = amg883x_irq_process(amg883x);

	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

static int amg883x_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct amg883x *ir_array = NULL;
	struct device *dev = NULL;

	ir_array = container_of(inode->i_cdev, struct amg883x, cdev);
	dev = &ir_array->client->dev;

	if (ir_array->fps != 10 && ir_array->fps != 1) {
		dev_err(dev, "get amg883x data failed, fps:%d", ir_array->fps);
		return -EIO;
	}

	filp->private_data = ir_array;

	ret = i2c_smbus_write_byte_data(ir_array->client, AMG883x_OPERA_MODE, 
					AMG883x_MODE_NORMAL);
	if (ret < 0) {
		dev_err(dev, "wrtie operation mode failed!, %d\n", ret);
		return ret;
	}

	return 0;
}

static int amg883x_release(struct inode *inode, struct file *filp)
{
	int ret;
	struct amg883x *ir_array = NULL;
	struct device *dev = NULL;

	filp->private_data = NULL;
	ir_array = container_of(inode->i_cdev, struct amg883x, cdev);
	dev = &ir_array->client->dev;
	
	ret = i2c_smbus_write_byte_data(ir_array->client, AMG883x_OPERA_MODE, 
					AMG883x_MODE_NORMAL);
	if (ret < 0) {
		dev_err(dev, "wrtie operation mode failed!, %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "%s success\n", __func__);

	return 0;
}

static ssize_t amg883x_read(struct  file *filp,  char  *buf, size_t len, 
			     loff_t *offset) 
{
	struct amg883x *ir_array = filp->private_data;
	struct i2c_client *client = ir_array->client;
	struct device *dev = &client->dev;
	struct amg883x_read_data *rd_buf;
	s32 value;
	int i;

	rd_buf = kmalloc(sizeof(struct amg883x_read_data), 
			      GFP_KERNEL);

	value = i2c_smbus_read_byte_data(client, AMG883x_OPERA_MODE);
	rd_buf->mode = (u8)value;
	value = i2c_smbus_read_byte_data(client, AMG883x_FRAME_RATE);
	rd_buf->fps = (u8)value;
	value = i2c_smbus_read_byte_data(client, AMG883x_INT_CTRL);
	rd_buf->int_control = (u8)value;
	value = i2c_smbus_read_byte_data(client, AMG883x_STATUS);
	rd_buf->status = (u8)value;
	value = i2c_smbus_read_word_data(client, AMG883x_INTHL);
	rd_buf->int_high_level = (u16)value;
	value = i2c_smbus_read_word_data(client, AMG883x_INTHH);
	rd_buf->int_low_level = (u16)value;
	value = i2c_smbus_read_word_data(client, AMG883x_IHYSL);
	rd_buf->int_hysteresis_level = (u16)value;
	value = i2c_smbus_read_word_data(client, AMG883x_TEMPERATURE);
	rd_buf->thermistor = (u16)value;

	for (i = 0; i < AMG883x_INT_TABLE_CNT; i++) {
		dev_dbg(dev, "read interrupt table");
		value = i2c_smbus_read_byte_data(client, AMG883x_INT_TABLE + i);
		rd_buf->int_pixel_table[i] = (u8)value;
	}

	for (i = 0; i < AMG883x_PIXEL_CNT; i++) {
		dev_dbg(dev, "read pixel table");
		value = i2c_smbus_read_word_data(client, AMG883x_PIXEL_VALUE + 
						 i*2);
		rd_buf->pixel_value_table[i] = (u16)value;
	}

	if (copy_to_user(buf, rd_buf, sizeof(struct amg883x_read_data)))
		return -EIO;

	kfree(rd_buf);
	return  sizeof(struct amg883x_read_data); /* temperature: two bytes */
}
 
static ssize_t amg883x_write( struct file *filp,  const char *buf,
			       size_t len, loff_t *offset) 
{
	int ret;
	struct amg883x *ir_array = filp->private_data;
	struct i2c_client *client = ir_array->client;
	struct device *dev = &client->dev;
	struct amg883x_write_data * wr_buf;

	if (len != sizeof(struct amg883x_write_data)) {
		dev_err(dev, "write size is wrong\n");
		return -EIO;
	}

	wr_buf = kmalloc(sizeof(struct amg883x_write_data), 
			      GFP_KERNEL);
	if (wr_buf == NULL) {
		dev_err(dev, "memory alloc failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(wr_buf, buf, len)) {
		dev_err(dev, "memory alloc failed\n");
		return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_RESET) {
		dev_dbg(dev, "write command reset register %02X",
			wr_buf->reset_reg);
		ret = i2c_smbus_write_byte_data(client, AMG883x_RESET_REG, 
						wr_buf->reset_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_CLEAR) {
		dev_dbg(dev, "write command clear register %02X\n", 
				wr_buf->clear_reg);
		ret = i2c_smbus_write_byte_data(client, AMG883x_STATUS_CLEAR, 
						wr_buf->clear_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_FPS) {
		dev_dbg(dev, "write command fps register %02X\n", wr_buf->fps);
		ret = i2c_smbus_write_byte_data(client, AMG883x_FRAME_RATE, 
						wr_buf->fps);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_INTC) {
		dev_dbg(dev, "write command int control register %02X\n", 
			wr_buf->int_ctrl_reg);
		ret = i2c_smbus_write_byte_data(client, AMG883x_INT_CTRL, 
						wr_buf->int_ctrl_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	kfree(wr_buf);
     	return  len;
}

static ssize_t amg883x_ioctl( struct file *filp, unsigned int cmd, unsigned long arg ) 
{
	struct amg883x *ir_array = filp->private_data;
	struct i2c_client *client = ir_array->client;
	struct device *dev = &client->dev;
	int ret = 0;
	s32 value;

	switch (cmd) {
	case AMG_CMD_PW_ON:
		dev_info(dev, "get ioctl command power on\n");
		break;
	case AMG_CMD_PW_OFF:
		dev_info(dev, "get ioctl command power off\n");
		break;
	case AMG_CMD_RD_TEMP:
		dev_info(dev, "get ioctl command read device temperature\n");

        	value = i2c_smbus_read_word_data(client, AMG883x_TEMPERATURE);

		if (value < 0) {
			dev_err(dev, "smbus read word failed to get temperature");
			ret = -EIO;
		} else {
			if (copy_to_user((int *)arg, &value, sizeof(int)))
				ret =  -EFAULT;
		}
		break;
	default:
		break;
	}

	return ret;
}

static struct file_operations amg883x_fops = {
	.read 		= amg883x_read,
	.write 		= amg883x_write,
	.open 		= amg883x_open,
	.release 	= amg883x_release,
	.unlocked_ioctl = amg883x_ioctl,
};

static int amg883x_probe(struct i2c_client *client, 
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct amg883x *amg883x;

	int ret;
	dev_info(dev, "%s insert!\n", dev->of_node->name);

	if (!i2c_check_functionality(client->adapter, 
				     I2C_FUNC_SMBUS_READ_BYTE_DATA | 
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA | 
				     I2C_FUNC_SMBUS_WRITE_WORD_DATA)) {
		dev_err(dev, "smbus read byte, write byte not supported!\n");
		return -EIO;
	}

	amg883x = devm_kzalloc(dev, sizeof(amg883x), GFP_KERNEL);
	if (!amg883x)
		return -ENOMEM;

	amg883x->client = client;

	amg883x->int_gpio = devm_gpiod_get_optional(dev, "int", GPIOD_IN);
	if (IS_ERR(amg883x->int_gpio))
		return PTR_ERR(amg883x->int_gpio);

	ret = of_property_read_u32(dev->of_node, "fps", &amg883x->fps);

	if (ret) {
		amg883x->fps = AMG883X_DEFAULT_FPS;
		dev_err(dev, "not set fps, set to default 10fps\n");
	} else {
		dev_info(dev, "fps set to %d\n", amg883x->fps);
		/* fps 10 or 3??*/
		if (amg883x->fps != AMG883X_DEFAULT_FPS || amg883x->fps != 3) {
			amg883x->fps = 10;
		}
	}

	if (client->irq) {
		dev_info(dev, "get irq of amg883x %d\n", client->irq);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						amg883x_irq_handler,
						IRQF_TRIGGER_LOW|IRQF_ONESHOT, dev_name(dev),
						amg883x);
		if (ret)
			dev_err(dev, "irq request failed!\n");
	}

	ret = alloc_chrdev_region(&dev_num, 0, 1, "amg883x");
	if  (ret < 0) {
        	dev_err(dev, "Can't static register chrdev region!\n" );   
		goto unregister_chrdev;
	}
	dev->devt = dev_num;

	cdev_init(&amg883x->cdev, &amg883x_fops);
	/* amg883x->cdev.owner = THIS_MODULE; */
	ret = cdev_add(&amg883x->cdev, dev_num, 1);
	if (ret) {
        	dev_err(dev, "Can't static register chrdev region!\n" );   
		goto cdev_remove;
	}
	/* dev_info(dev, "device name:%s", dev->kobj.name); */

	dev_info(dev, "major:%u", MAJOR(dev_num));
	amg883x_class = class_create(THIS_MODULE, AMG883X_CLASS);
	device_create(amg883x_class, NULL, dev_num, NULL, "amg883x");

	i2c_set_clientdata(client, amg883x);
	return 0;

cdev_remove:
	unregister_chrdev_region(client->dev.devt, 1);
unregister_chrdev:
	kzfree(amg883x);

	return ret;
}

static int amg883x_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct amg883x *amg883x = i2c_get_clientdata(client);

	dev_info(dev, "amg8833 removed!\n");
	device_destroy(amg883x_class, client->dev.devt);
	class_destroy(amg883x_class);
	cdev_del(&amg883x->cdev);

	unregister_chrdev_region(client->dev.devt, 1);

	return 0;
}

static const struct of_device_id amg883x_dt_ids[] = {
	{ .compatible = "panasonic,amg8833"},
	{ .compatible = "panasonic,amg8831"},
	{ },
};
MODULE_DEVICE_TABLE(of, amg883x_dt_ids);

static const struct i2c_device_id amg883x_ids[] = {
	{ "amg883x", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, amg883x_ids);

static struct i2c_driver amg883x_driver = {
	.probe  = amg883x_probe,
	.remove = amg883x_remove,
	.driver = {
		.name = "amg883x",
		.of_match_table = amg883x_dt_ids,
	},
	.id_table = amg883x_ids,
};
module_i2c_driver(amg883x_driver);

MODULE_AUTHOR("Jerry Zheng <JerryZheng89@outlook.com>");
MODULE_DESCRIPTION("Panasonic ir array sensor");
MODULE_LICENSE("GPL");
