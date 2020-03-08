/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/regmap.h>

#include "type-c-ti.h"

#include <linux/HWVersion.h>

extern const char* Read_HW_ID_STR(void);

#define T_MS_I2C_EN 100		/* Time from EN_N to low and VDD active
				 * to I2C access available */
#define T_MS_SW_RESET 95	/* Software reset duration */

/* Register ADDR */
#define CC_STATE_REG1			0x8
#define CC_CURRENT_MODE_DETECT_BIT	4
#define CC_CURRENT_MODE_DETECT_MASK	(3 << CC_CURRENT_MODE_DETECT_BIT)
enum {
	CC_CURRENT_MODE_DETECT_DEFAULT = 0,
	CC_CURRENT_MODE_DETECT_MDEIUM,
	CC_CURRENT_MODE_DETECT_ACC,
	CC_CURRENT_MODE_DETECT_HIGH,
};
#define current_mode_detect(v) ((v & CC_CURRENT_MODE_DETECT_MASK) >> CC_CURRENT_MODE_DETECT_BIT)

#define CC_STATE_REG2		0x9
#define ATTACHED_BIT		6
#define CC_ATTACHED_STATE_MASK	(3 << ATTACHED_BIT)
enum {
	NOT_ATTACHED = 0,
	ATTACHED_DFP,
	ATTACHED_UFP,
	ATTACHED_ACC,
};
#define attached_state(v) ((v & CC_ATTACHED_STATE_MASK) >> ATTACHED_BIT)

#define CABLE_DIR_MASK (1 << 5)
#define CABLE_DIR_CC1 0
#define CABLE_DIR_CC2 1
#define cable_dir(v) ((v >> 5) & 0x01)

#define INT_STATUS_MASK (1 << 4)
#define is_interrupted(v) ((v & INT_STATUS_MASK) > 0)

#define CC_CONTROL_REG		0xA
#define CC_I2C_SOFT_RESET	(1 << 3)
#define CC_MODEL_SELECT_MATAIN	0
#define CC_MODEL_SELECT_UFP	(1 << 4)
#define CC_MODEL_SELECT_DFP	(2 << 4)
#define CC_MODEL_SELECT_DRP	(3 << 4)
#define CC_MODEL_SELECT_MASK	(3 << 4)

struct ti_usb_type_c {
	struct i2c_client	*client;
	struct regmap		*map;
	struct power_supply	*usb_psy;
	struct work_struct	 cc_wq;
	int			 cc_state;
	int			 cc_valid;
	u8			 cc_sm_delay;
	struct timer_list	 delay_timer;

	// status
	char			 id[9];
	int			 max_current;
	int			 attach_state;
	// platform data
	int			 cc_int_n;
	int			 enb_gpio;
	int			 enb_gpio_polarity;
};

enum {
	CC_INIT,
	CC_RST,
	CC_SYNC_STATE,
	CC_RUNNING,
};
static struct ti_usb_type_c *ti_usb;

const char* get_attached_mode_str(int mode)
{
	static const char* str[] = {
		[NOT_ATTACHED] = "NOT_ATTACHED",
		[ATTACHED_DFP] = "ATTACHED_DFP",
		[ATTACHED_UFP] = "ATTACHED_UFP",
		[ATTACHED_ACC] = "ATTACHED_ACC"
	};
	BUG_ON(mode > ATTACHED_ACC);
	return str[mode];
}
const char* get_ufp_max_current_str(int mode)
{
	static const char* str[]		 = {
		[CC_CURRENT_MODE_DETECT_DEFAULT] = "default",
		[CC_CURRENT_MODE_DETECT_MDEIUM]	 = "medium",
		[CC_CURRENT_MODE_DETECT_ACC]	 = "charging through accessary",
		[CC_CURRENT_MODE_DETECT_HIGH]	 = "high"
	};
	BUG_ON(mode > CC_CURRENT_MODE_DETECT_HIGH);
	return str[mode];
}
static ssize_t show_i2c_status(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	/* Since we have check I2C accessibility in CC_RST,
	 * simply report 'good' if the state machine is in CC_RUNNING state. */
	return snprintf(buf, PAGE_SIZE, "%d\n", ti_usb->cc_state >= CC_RUNNING);
}
static DEVICE_ATTR(i2c_status, S_IRUGO, show_i2c_status, NULL);

static ssize_t show_cc_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ti_usb->cc_valid);
}

static DEVICE_ATTR(cc_status, S_IRUGO, show_cc_status, NULL);

static struct attribute *dev_attrs[] = {
    &dev_attr_i2c_status.attr,
    &dev_attr_cc_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};

/**
 * read status registers and update meaningful internal data.
 * @param none
 * @return true if state changed. Currently, only attach state tack in place.
 */
static inline bool cc_sync_state(void)
{
	int state_reg[2];
	int pre_attach_state = ti_usb->attach_state;

	regmap_read(ti_usb->map, CC_STATE_REG1, state_reg);
	regmap_read(ti_usb->map, CC_STATE_REG2, state_reg + 1);
	DBG_RL(DBG_SM, "CC_STATE_REG1=%02x, CC_STATE_REG2=%02x\n",
	       state_reg[0], state_reg[1]);

	ti_usb->max_current = CC_CURRENT_MODE_DETECT_DEFAULT;
	ti_usb->attach_state = attached_state(state_reg[1]);
	if (ti_usb->attach_state == ATTACHED_UFP) {
		ti_usb->max_current = current_mode_detect(state_reg[0]);
	}
	if (ti_usb->attach_state != NOT_ATTACHED) {
		ti_usb->cc_valid = cable_dir(state_reg[1])? 2 : 1;
		DBG(DBG_SM, "CC%c attached\n", cable_dir(state_reg[1]) ? '2' : '1');
	}

	if (is_interrupted(state_reg[1])) {
		/* clear INT */
		state_reg[1] |= INT_STATUS_MASK;
		regmap_write(ti_usb->map, CC_STATE_REG2, state_reg[1]);
		//DBG(DBG_SM, "clr INT\n");
	}
	return (pre_attach_state != ti_usb->attach_state);
}

static irqreturn_t cc_i2c_irqt(int irq, void *dev_id) {
	flush_work(&ti_usb->cc_wq);
	if (cc_sync_state())
		schedule_work(&ti_usb->cc_wq);
	return IRQ_HANDLED;
}

static inline void start_cc_delay_timer(u8 delay)
{
	if (delay == 0)
		return;

	ti_usb->cc_sm_delay = delay;
	mod_timer(&ti_usb->delay_timer,
		  (jiffies + msecs_to_jiffies(delay)));
}

static void delay_timer_func(unsigned long data)
{
	bool work = 0;

	switch (ti_usb->cc_sm_delay) {
	case T_MS_I2C_EN:
		ti_usb->cc_state = CC_RST;
		work = 1;
		break;
	case T_MS_SW_RESET:
		ti_usb->cc_state = CC_SYNC_STATE;
		work = 1;
		break;
	default:
		DBG(DBG_ERR, "incorrect delay value: %d\n", ti_usb->cc_sm_delay);
	}
	if (work) {
		schedule_work(&ti_usb->cc_wq);
	}
}

/**
 * state machine work.
 */
static void cc_sm_work(struct work_struct *w) {
	bool work = 0;
	int cstate = ti_usb->cc_state;
	struct i2c_adapter *adapter = to_i2c_adapter(ti_usb->client->dev.parent);
	int val, i;
	char *ptr;

	switch (cstate) {
	case CC_INIT:
		/* Chip is uninitialize. enable chip.
		 * will need to have a long delay - T_MS_I2C_EN */
		gpio_set_value(ti_usb->enb_gpio, !ti_usb->enb_gpio_polarity);
		start_cc_delay_timer(T_MS_I2C_EN);
		break;
	case CC_RST:
		WARN_ON(!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA));
		/* From here we can access chip through I2C.
		 * We do the things as below.
		 */

		/* 1. Read IDs */
		ptr = ti_usb->id;
		for (i = 7; i >= 0; i--) {
			regmap_read(ti_usb->map, (unsigned int)i, &val);
			if (val == 0)
				continue;
			*ptr++ = (char)val;
		}
		*ptr = 0;
		DBG(DBG_INIT, "ID: %s\n", ti_usb->id);

		/* 2. Set CC in DRP unattach mode */
		regmap_read(ti_usb->map, CC_CONTROL_REG, &val);
		val &= ~CC_MODEL_SELECT_MASK;
		val |= CC_MODEL_SELECT_DRP;
		regmap_write(ti_usb->map, CC_CONTROL_REG, val);
		mdelay(1);

		/* 3. Do S/W reset. It will need to have a long delay - T_MS_SW_RESET */
		val |= CC_I2C_SOFT_RESET;
		regmap_write(ti_usb->map, CC_CONTROL_REG, val);

		start_cc_delay_timer(T_MS_SW_RESET);
		break;
	case CC_SYNC_STATE:
		cc_sync_state();
		enable_irq(ti_usb->client->irq);
		ti_usb->cc_state = CC_RUNNING;
		work = 1;
		break;
	case CC_RUNNING:
		DBG(DBG_SM, "attach mode: %s, UFP max current: %s\n",
		    get_attached_mode_str(ti_usb->attach_state),
		    get_ufp_max_current_str(ti_usb->max_current));
		/* TODO. publish status here. */

		break;
	default:
		DBG(DBG_ERR, "unknown state: %d\n", cstate);
		/* TODO. inactive chip, then re-run stat machine from CC_INIT */
		return;
	}

	DBG(DBG_SM, "%s, state chage %d -> %d\n", __func__, cstate, ti_usb->cc_state);
	if (work) {
		schedule_work(&ti_usb->cc_wq);
	}
}

static struct regmap_config cc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static int tiusb_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
	int ret;
	int gpio_idx;
	struct power_supply *usb_psy;
	struct device_node *np = i2c->dev.of_node;
	enum of_gpio_flags flags;
	u8 num_enb_pins;
	const char *hardware_phase;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		DBG(DBG_ERR, "USB power_suppply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	ti_usb = devm_kzalloc(&i2c->dev, sizeof(struct ti_usb_type_c),
				GFP_KERNEL);

	if (!ti_usb)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ti_usb);
	ti_usb->client = i2c;
	ti_usb->usb_psy = usb_psy;
	ti_usb->map = devm_regmap_init_i2c(i2c, &cc_regmap_config);
	if (IS_ERR(ti_usb->map)) {
		DBG(DBG_ERR, "Failed to initialize regmap\n");
		ret = -EINVAL;
		goto out;
	}
	INIT_WORK(&ti_usb->cc_wq, cc_sm_work);

	if(!of_property_read_u8(np, "ti,enb-pins", &num_enb_pins)) {
		hardware_phase = Read_HW_ID_STR();
        DBG(DBG_INIT, "hardware phase: %s\n", hardware_phase);
		gpio_idx = of_property_match_string(np, "ti,enb-names", hardware_phase);
        if( gpio_idx < 0)
			gpio_idx = 0;
	};

	ti_usb->enb_gpio = of_get_named_gpio_flags(np, "ti,enb-gpio", gpio_idx,
						   &flags);
	if(!gpio_is_valid(ti_usb->enb_gpio)) {
		DBG(DBG_ERR, "enb gpio_get fail:%d\n", ti_usb->enb_gpio);
		goto out;
	}
	ti_usb->enb_gpio_polarity = (flags & OF_GPIO_ACTIVE_LOW);
	DBG(DBG_INIT, "EN_N gpio=%d, active %s\n", ti_usb->enb_gpio,
	    ti_usb->enb_gpio_polarity ? "low" : "high");
	/* enable chip.  */
	ret = devm_gpio_request_one(&ti_usb->client->dev, ti_usb->enb_gpio,
				    ti_usb->enb_gpio_polarity ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
				    PIN_NAME("EN_N"));
	if (ret) {
		goto out;
	}
	/* setup irq */
	ret = of_get_named_gpio(np, "ti,usb-ccirq", 0);
	if (ti_usb->cc_int_n < 0) {
		DBG(DBG_ERR, "INT_N gpio is not available, err=%d\n", ret);
		goto out;
	}
	ti_usb->cc_int_n = ret;
	DBG(DBG_INIT, "INT_N gpio=%d\n", ti_usb->cc_int_n);

	if (!gpio_is_valid(ti_usb->cc_int_n)) {
		DBG(DBG_ERR, "INT_N gpio is not available\n");
		goto out;
	}
	ret = gpio_request_one(ti_usb->cc_int_n, GPIOF_IN, PIN_NAME("CC_INT"));
	if (ret) {
		DBG(DBG_ERR, "unable to setup INT_N, err=%d\n", ret);
	}

	if (i2c->irq < 0) {
		/* irq is not specified in dts. get it according to gpio */
		ret = gpio_to_irq(ti_usb->cc_int_n);
		if (ret < 0) {
			/* still error */
			DBG(DBG_ERR, "INT_N irq not defined, err=%d\n", ret);
			goto out;
		}
		i2c->irq = ret;
	}
	DBG(DBG_INIT, "INT_N irq=%d\n", i2c->irq);

	/* setup irq handler */
	ret = devm_request_threaded_irq(&i2c->dev, i2c->irq, NULL, cc_i2c_irqt,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					PIN_NAME("INT_N"), ti_usb);
	if (ret < 0) {
		DBG(DBG_ERR, "unable to request threaded_irq. err=%d\n", ret);
		goto out;
	}
	disable_irq_nosync(i2c->irq);

	setup_timer(&ti_usb->delay_timer, delay_timer_func,
		    (unsigned long) ti_usb);

	ti_usb->cc_state = CC_INIT;
	schedule_work(&ti_usb->cc_wq);

	if(sysfs_create_group(&i2c->dev.kobj, &dev_attr_grp)){
			DBG(DBG_ERR, "sysfs_create_group create dev_attr_grp fail");
	}
	return 0;
out:
	devm_kfree(&i2c->dev, ti_usb);
	return ret;
}

static int tiusb_remove(struct i2c_client *i2c)
{
	struct ti_usb_type_c *ti_usb = i2c_get_clientdata(i2c);
	sysfs_remove_group(&i2c->dev.kobj, &dev_attr_grp);
	devm_kfree(&i2c->dev, ti_usb);
	return 0;
}

static const struct i2c_device_id tiusb_id[] ={
	{ DRV_NAME, 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id tiusb_of_match[] = {
	{ .compatible = "ti,usb-type-c", },
	{},

};
MODULE_DEVICE_TABLE(of, tiusb_of_match);
#endif

static struct i2c_driver tiusb_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(tiusb_of_match),
	},
	.probe		= tiusb_probe,
	.remove		= tiusb_remove,
	.id_table	= tiusb_id,
};

static int __init tiusb_i2c_init(void)
{
	DBG(DBG_INIT, "%s\n", __func__);
	return i2c_add_driver(&tiusb_driver);
}

static void __exit tiusb_i2c_exit(void)
{
	return;
}

module_init(tiusb_i2c_init);
module_exit(tiusb_i2c_exit);
//module_i2c_driver(tiusb_driver);

MODULE_AUTHOR("Nick,O.Y nick_ouyang@asus.com");
MODULE_DESCRIPTION("ASUS TI TUSB320 Type C Detection driver");
MODULE_LICENSE("GPL v2");
