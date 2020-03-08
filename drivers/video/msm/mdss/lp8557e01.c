#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/HWVersion.h>

static struct i2c_client *lp8557e01_client;

static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
	};
	u8 rx_data[1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
		value & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

static int is_support = -1;
int lp8557e01_support(void)
{
	if(is_support != -1)
		return is_support;

	return is_support;
}
EXPORT_SYMBOL(lp8557e01_support);

void lp8557e01_suspend(void)
{
	u8 value0=0;
	int ret0;

	i2c_reg_write(lp8557e01_client, 0x00, 0x00);
	ret0 = i2c_reg_read(lp8557e01_client, 0x00, &value0);
	printk("[DISPLAY] %s: 00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0);
	return;
}
EXPORT_SYMBOL(lp8557e01_suspend);

void lp8557e01_set_brightness(int level)
{
	i2c_reg_write(lp8557e01_client, 0x04, level);

	pr_debug("[DISPLAY] %s: level=%d\n", __func__, level);
	return;
}
EXPORT_SYMBOL(lp8557e01_set_brightness);

static bool first_boot=true;
void lp8557e01_resume(void)
{
	u8 value0=0;
	int ret0=0;

	if(first_boot){
		printk(KERN_INFO "%s: bypass lp8557e01 i2c settings.\n", __func__);
		first_boot=false;
		return;
	}

	usleep_range(5000, 5000);

	i2c_reg_write(lp8557e01_client, 0x00, 0x00);
	i2c_reg_write(lp8557e01_client, 0x10, 0x85);
	i2c_reg_write(lp8557e01_client, 0x11, 0x05);
	i2c_reg_write(lp8557e01_client, 0x12, 0x2C);
	i2c_reg_write(lp8557e01_client, 0x13, 0x03);
	i2c_reg_write(lp8557e01_client, 0x15, 0xC1);
	i2c_reg_write(lp8557e01_client, 0x7F, 0x21);
	i2c_reg_write(lp8557e01_client, 0x7A, 0x00);
	i2c_reg_write(lp8557e01_client, 0x16, 0x60);
	i2c_reg_write(lp8557e01_client, 0x14, 0xBF);

	i2c_reg_write(lp8557e01_client, 0x00, 0x01);
	ret0 = i2c_reg_read(lp8557e01_client, 0x00, &value0);

	printk("[DISPLAY] %s: 00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0);
	return;
}
EXPORT_SYMBOL(lp8557e01_resume);

static int lp8557e01_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int addr;

	printk("[DISPLAY] %s: Enter\n",__func__);

	lp8557e01_client = client;
	addr = client->addr;

	printk("[DISPLAY] %s: slave address=0x%x\n", __func__, addr);

	return 0;
}

static struct of_device_id lp8557e01_i2c_table[] = {
	{ .compatible = "lp,8557e01"}, //Compatible node must match dts
	{ },
};

static const struct i2c_device_id lp8557e01_id[] = {
	{ "lp8557e01", 0 },
	{ },
};

static struct i2c_driver lp8557e01_driver = {
	.driver = {
		.name = "lp8557e01",
		.owner = THIS_MODULE,
		.of_match_table = lp8557e01_i2c_table,
	},
	.probe = lp8557e01_probe,
	.id_table = lp8557e01_id,
};


static int __init lp8557e01_I2C_init(void)
{
	int ret = 0;
	printk("[DISPLAY] %s: Enter\n",__func__);
	ret = i2c_add_driver(&lp8557e01_driver);

	return ret;
}

static void __exit lp8557e01_I2C_exit(void)
{
	return;
}

module_init(lp8557e01_I2C_init);
module_exit(lp8557e01_I2C_exit);

MODULE_DESCRIPTION("lp8557e01");
MODULE_LICENSE("GPL v2");
