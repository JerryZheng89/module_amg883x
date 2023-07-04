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
#include <linux/miscdevice.h>
#include <linux/poll.h>
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
#define AMG883x_RESERVED_REG 	0x06

static DECLARE_WAIT_QUEUE_HEAD(amg883x_rq);
static int new_data_ready = false;

struct amg883x {
	struct i2c_client *client;
	struct gpio_desc *int_gpio; /* for interrupt */
	struct miscdevice miscdev;
	struct device *dev;
	int fps;
	struct regmap *regmap;
};

static const struct regmap_range amg883x_rd_reg_range[] = {
	{
		.range_min = 0x00,
		.range_max = 0x00,
	},{
		.range_min = 0x02,
		.range_max = 0x04,
	},{
		.range_min = 0x07,
		.range_max = 0x17,
	},{
		.range_min = 0x80,
		.range_max = 0xFF,
	},
};

static const struct regmap_range amg883x_wr_reg_range[] = {
	{
		.range_min = 0x00,
		.range_max = 0x03,
	},{
		.range_min = 0x05,
		.range_max = 0x05,
	},{
		.range_min = 0x08,
		.range_max = 0x0D,
	},
};

static const struct regmap_access_table amg883x_rd_reg_table = {
	.yes_ranges = amg883x_rd_reg_range,
	.n_yes_ranges = ARRAY_SIZE(amg883x_rd_reg_range),
};

static const struct regmap_access_table amg883x_wr_reg_table = {
	.yes_ranges = amg883x_wr_reg_range,
	.n_yes_ranges = ARRAY_SIZE(amg883x_wr_reg_range),
};

static const struct regmap_config amg883x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.rd_table = &amg883x_rd_reg_table,
	.wr_table = &amg883x_wr_reg_table,
};

static inline int amg883x_read_reg(struct amg883x *amg883x, u8 addr, u8 *value)
{
	unsigned int val;
	int ret;

	ret = regmap_read(amg883x->regmap, addr , &val);
	if (ret) {
		dev_err(amg883x->dev, "i2c read failed at addr: %x\n", addr);
		return ret;
	}

	*value = val & 0xff;

	return 0;
}

static inline int amg883x_write_reg(struct amg883x *amg883x, u8 addr, u8 value)
{
	int ret;

	ret = regmap_write(amg883x->regmap, addr , value);
	if (ret) {
		dev_err(amg883x->dev, "i2c write failed at addr: %x\n", addr);
		return ret;
	}

	return 0;
}

static int amg883x_irq_process(struct amg883x *amg883x)
{
	struct device *dev = &amg883x->client->dev;
	struct amg883x_write_data *wr_buf;
	u8 value;
	int ret;
	int i;

	wr_buf = kmalloc(sizeof(struct amg883x_write_data), 
			      GFP_KERNEL);

	ret = amg883x_read_reg(amg883x, AMG883x_STATUS, &value);
	dev_info(dev, "irq flag 0x%02x\n", value);

	ret = amg883x_read_reg(amg883x, AMG883x_INT_CTRL, &value);
	if ((value&0x02) == 0x00) {
		ret = amg883x_write_reg(amg883x, AMG883x_INT_CTRL, 0x03);
	}

	dev_info(dev, "irq control 0x%02x\n", value);
	ret = amg883x_read_reg(amg883x, AMG883x_INTHL, &value);
	dev_info(dev, "INT_LVL_H_L 0x%02x\n", value);

	if (value < 0x80) {
		ret = amg883x_write_reg(amg883x, AMG883x_INTHL, 0x80);
		ret = amg883x_write_reg(amg883x, AMG883x_INTHH, 0x00);
		ret = amg883x_write_reg(amg883x, AMG883x_IHYSL, 0x00);
		ret = amg883x_write_reg(amg883x, AMG883x_IHYSH, 0x00);
	}

	ret = amg883x_read_reg(amg883x, AMG883x_INTLL, &value);
	dev_info(dev, "INT_LVL_L_L 0x%02x\n", value);

	for (i = 0; i < AMG883x_INT_TABLE_CNT; i++) {
		ret = amg883x_read_reg(amg883x, AMG883x_INT_TABLE + i, &value);
		dev_info(dev, "INT TABLE%d %2x\n", i, value);
	}

	if (ret) {
		ret = value;
		dev_err(dev, "failed to read data");
		goto un_alloc;
	}

	wr_buf->clear_reg = 0;
	wr_buf->clear_reg_bit.int_clear = 1;

	/* clear interrupt flag */
	ret = amg883x_write_reg(amg883x, AMG883x_STATUS_CLEAR, 
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
	if (ret == 0) {
		new_data_ready = true;
		wake_up_interruptible(&amg883x_rq);
	}

	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

static int amg883x_open(struct inode *inode, struct file *filp)
{
	int ret;
	u8 value;
	struct miscdevice *misc = filp->private_data;
	struct amg883x *ir_array = container_of(misc, struct amg883x, miscdev);
	struct amg883x *amg883x = container_of(misc, struct amg883x, miscdev);
	struct device *dev = NULL;

	dev = &ir_array->client->dev;
	new_data_ready = false;

	if (ir_array->fps != 10 && ir_array->fps != 1) {
		dev_err(dev, "get amg883x data failed, fps:%d", ir_array->fps);
		return -EIO;
	}

	filp->private_data = ir_array;

	ret = amg883x_write_reg(amg883x, AMG883x_OPERA_MODE, 
				AMG883x_MODE_NORMAL);
	ret = amg883x_write_reg(amg883x, AMG883x_INT_CTRL, 0x03);
	ret = amg883x_write_reg(amg883x, AMG883x_INTHL, 0x80);
	ret = amg883x_write_reg(amg883x, AMG883x_INTHH, 0x00);
	if (ret < 0) {
		dev_err(dev, "wrtie operation mode failed!, %d\n", ret);
		return ret;
	}

	/**
	 * test case for the reg range of the regmap framework.
	 */
	ret = amg883x_read_reg(amg883x, AMG883x_RESERVED_REG, &value);
	dev_err(dev, "read reg out of range!, %d\n", ret);
	ret = amg883x_write_reg(amg883x, AMG883x_RESERVED_REG, 0x00);
	dev_err(dev, "write reg out of range!, %d\n", ret);
	ret = amg883x_read_reg(amg883x, AMG883x_STATUS_CLEAR, &value);
	dev_err(dev, "read reg out of range!, %d\n", ret);
	ret = amg883x_write_reg(amg883x, AMG883x_STATUS, 0x00);
	dev_err(dev, "write reg out of range!, %d\n", ret);


	return 0;
}

static int amg883x_release(struct inode *inode, struct file *filp)
{
	int ret;
	struct device *dev = NULL;
	struct amg883x *amg883x = filp->private_data;

	filp->private_data = NULL;
	dev = &amg883x->client->dev;
	
	ret = amg883x_write_reg(amg883x, AMG883x_OPERA_MODE, 
					AMG883x_MODE_SLEEP);
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
	struct amg883x *amg883x = filp->private_data;
	struct device *dev = &amg883x->client->dev;
	struct amg883x_read_data *rd_buf;
	s32 value;
	int i;
	int ret;

	rd_buf = kmalloc(sizeof(struct amg883x_read_data), 
			      GFP_KERNEL);

	ret = amg883x_read_reg(amg883x, AMG883x_OPERA_MODE, (u8 *)&value);
	rd_buf->mode = (u8)value;
	ret = amg883x_read_reg(amg883x, AMG883x_FRAME_RATE, (u8 *)&value);
	rd_buf->fps = (u8)value;
	ret = amg883x_read_reg(amg883x, AMG883x_INT_CTRL, (u8 *)&value);
	rd_buf->int_control = (u8)value;
	ret = amg883x_read_reg(amg883x, AMG883x_STATUS, (u8 *)&value);
	rd_buf->status = (u8)value;
	ret = amg883x_read_reg(amg883x, AMG883x_INTHL, (u8 *)&value);
	ret = regmap_bulk_read(amg883x->regmap, AMG883x_INTHL, &value, 2);
	rd_buf->int_high_level = (u16)value;
	ret = regmap_bulk_read(amg883x->regmap, AMG883x_INTHH, &value, 2);
	rd_buf->int_low_level = (u16)value;
	ret = regmap_bulk_read(amg883x->regmap, AMG883x_IHYSL, &value, 2);
	rd_buf->int_hysteresis_level = (u16)value;
	ret = regmap_bulk_read(amg883x->regmap, AMG883x_TEMPERATURE, &value, 2);
	rd_buf->thermistor = (u16)value;

	for (i = 0; i < AMG883x_INT_TABLE_CNT; i++) {
		dev_dbg(dev, "read interrupt table");
		ret = amg883x_read_reg(amg883x, AMG883x_INT_TABLE + i, (u8 *)&value);
		rd_buf->int_pixel_table[i] = (u8)value;
	}

	for (i = 0; i < AMG883x_PIXEL_CNT; i++) {
		dev_dbg(dev, "read pixel table");
		ret = regmap_bulk_read(amg883x->regmap, AMG883x_PIXEL_VALUE + 
						 i*2, &value, 2);
		if (ret) {
			dev_err(dev, "i2c read pixel failed to get temperature");
			return -EIO;
		}
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
	struct amg883x *amg883x = filp->private_data;
	struct device *dev = &amg883x->client->dev;
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
		ret = amg883x_write_reg(amg883x, AMG883x_RESET_REG, 
						wr_buf->reset_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_CLEAR) {
		dev_dbg(dev, "write command clear register %02X\n", 
				wr_buf->clear_reg);
		ret = amg883x_write_reg(amg883x, AMG883x_STATUS_CLEAR, 
						wr_buf->clear_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_FPS) {
		dev_dbg(dev, "write command fps register %02X\n", wr_buf->fps);
		ret = amg883x_write_reg(amg883x, AMG883x_FRAME_RATE, 
						wr_buf->fps);
		if (ret < 0) 
			return -EFAULT;
	}

	if (wr_buf->wr_flag & AMG883X_WR_FLAG_INTC) {
		dev_dbg(dev, "write command int control register %02X\n", 
			wr_buf->int_ctrl_reg);
		ret = amg883x_write_reg(amg883x, AMG883x_INT_CTRL, 
						wr_buf->int_ctrl_reg);
		if (ret < 0) 
			return -EFAULT;
	}

	kfree(wr_buf);
     	return  len;
}

static ssize_t amg883x_ioctl( struct file *filp, unsigned int cmd, unsigned long arg ) 
{
	struct amg883x *amg883x = filp->private_data;
	struct device *dev = &amg883x->client->dev;
	int ret = 0;
	int value = 0;

	switch (cmd) {
	case AMG_CMD_PW_ON:
		dev_info(dev, "get ioctl command power on\n");
		break;
	case AMG_CMD_PW_OFF:
		dev_info(dev, "get ioctl command power off\n");
		break;
	case AMG_CMD_RD_TEMP:
		dev_info(dev, "get ioctl command read device temperature\n");

        	ret = regmap_bulk_read(amg883x->regmap, AMG883x_TEMPERATURE, &value, 2);

		if (ret) {
			dev_err(dev, "i2c read word failed to get temperature");
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

static __poll_t amg883x_poll(struct file *filp, poll_table *wait)
{
	__poll_t mask = 0;

	poll_wait(filp, &amg883x_rq, wait);
	if (new_data_ready == true) {
		new_data_ready = false;
		mask = POLLIN | POLLRDNORM;
	}

	return mask;
}

static struct file_operations amg883x_fops = {
	.owner		= THIS_MODULE,
	.read 		= amg883x_read,
	.write 		= amg883x_write,
	.open 		= amg883x_open,
	.release 	= amg883x_release,
	.unlocked_ioctl = amg883x_ioctl,
	.poll 		= amg883x_poll,
};

static int amg883x_probe(struct i2c_client *client, 
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct amg883x *amg883x;

	int ret;
	dev_info(dev, "%s insert!\n", dev->of_node->name);

	amg883x = devm_kzalloc(dev, sizeof(amg883x), GFP_KERNEL);
	if (!amg883x)
		return -ENOMEM;

	amg883x->client = client;

	amg883x->regmap = devm_regmap_init_i2c(client, &amg883x_regmap_config);
	if (IS_ERR(amg883x->regmap)) {
		dev_err(dev, "Failed to initialize I2C\n");
		return -ENODEV;
	}

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

	amg883x->dev = get_device(dev);
	dev_set_drvdata(dev, amg883x);
	amg883x->miscdev.parent = amg883x->dev;
	amg883x->miscdev.fops = &amg883x_fops;
	amg883x->miscdev.name = "amg883x";
	amg883x->miscdev.minor = MISC_DYNAMIC_MINOR;

	ret = misc_register(&amg883x->miscdev);
	if (ret != 0) {
		amg883x->miscdev.name = NULL;
		dev_err(dev, "misc register failed!\n");
		goto put_device;
	}
	i2c_set_clientdata(client, amg883x);
	
	return ret;
put_device:
	put_device(amg883x->dev);
	kfree(amg883x);
	return -1;
}

static int amg883x_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct amg883x *amg883x = i2c_get_clientdata(client);

	dev_info(dev, "amg8833 removed!\n");
	misc_deregister(&amg883x->miscdev);

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
