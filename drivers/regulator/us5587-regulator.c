/*
 * tps51632-regulator.c -- TI us5587
 *
 * Regulator driver for us5587 3-2-1 Phase D-Cap Step Down Driverless
 * Controller with serial VID control and DVFS.
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

/**********************************************************
 *
 *      [I2C Slave Setting]
 *
 *********************************************************/
#define us5587_SLAVE_ADDR_READ        0x73
static struct i2c_client *new_client;

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
static unsigned int ADC_SW_EN;
u32 ADCPWREN_PMI_GP1;  //on going
struct qpnp_pin_cfg param ={
        .mode = 1,
        .output_type = 1,
        .invert = 0,
        .pull = 4,
        .vin_sel = 1,
        .out_strength = 1,
        .master_en = 1
};
static DEFINE_MUTEX(us5587_i2c_access);
/**********************************************************
 *
 *      [I2C Function For Read/Write us5587]
 *
 *********************************************************/
int us5587_read_byte(unsigned char cmd, unsigned char *returnData)
{
        char     readData = 0;
        int      ret = 0;
        struct i2c_msg msg[2];
        struct i2c_adapter *adap = new_client->adapter;

        mutex_lock(&us5587_i2c_access);
        msg[0].addr = new_client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &cmd;

        msg[1].addr = new_client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = &readData;

        ret = i2c_transfer(adap, msg, 2);
        if (ret < 0) {
                mutex_unlock(&us5587_i2c_access);
                return 0;
        }
        *returnData = readData;

        mutex_unlock(&us5587_i2c_access);
        return 1;
}

extern int ADCPWR_enable(bool bEnable);
extern int DPM_SW_enable(bool bEnable);
/**********************************************************
 *
 *	[platform_driver API]
 *
 *********************************************************/
static ssize_t show_us5587_dump (struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char *p = buf;
	unsigned char RegNum;
	unsigned char us5587_reg = 0;
	
	p += sprintf(p,"[us5587_dump]\n");
	RegNum = 0x03;
	ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0x04;
	ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0x05;
	ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0xB2;
	ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

	ret = p - buf;		
	return ret;
}
static ssize_t store_us5587_access(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_ERR "[store_us5587_access]\n ");
	return size;
}
static DEVICE_ATTR(us5587_dump, 0664, show_us5587_dump, store_us5587_access);

static ssize_t show_gpio_pmic (struct device *dev, struct device_attribute *attr, char *buf)
{
	char *p = buf;
        int ret = 0;
//        unsigned char RegNum;
//       unsigned char us5587_reg = 0;

	p += sprintf(p,"1ADCPWREN_PMI_GP1 = %d pull=%d\n",ADCPWREN_PMI_GP1,gpio_get_value(ADCPWREN_PMI_GP1)); 

	ret = p- buf;
	return ret;
}
static ssize_t store_gpio_pmic(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int value = 0;

	pvalue = (char *)buf;
	ret = kstrtou32(pvalue, 16, (unsigned int *)&value);
	gpio_direction_output(ADCPWREN_PMI_GP1, value);
	printk(KERN_ERR "value=%d/n", value);
	printk(KERN_ERR "1ADCPWREN_PMI_GP1 = %d pull=%d\n",ADCPWREN_PMI_GP1,gpio_get_value(ADCPWREN_PMI_GP1));
	
	return size;

}
static DEVICE_ATTR(gpio_pmic, 0664, show_gpio_pmic, store_gpio_pmic);

static ssize_t show_gpiot (struct device *dev, struct device_attribute *attr, char *buf)
{
        char *p = buf;
        int ret = 0;
        unsigned char RegNum;
        unsigned char us5587_reg = 0;

        p += sprintf(p,"ADC_SW_EN= %d pull=%d\n",ADC_SW_EN,gpio_get_value(ADC_SW_EN));
        p += sprintf(p,"[us5587_dump]\n");
        RegNum = 0x03;
        ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

        RegNum = 0x04;
        ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

        RegNum = 0x05;
        ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

        RegNum = 0xB2;
        ret = us5587_read_byte(RegNum, &us5587_reg);
    p += sprintf(p, "Reg[%x]=0x%x\n",RegNum , us5587_reg);

        ret = p- buf;
        return ret;
}


static ssize_t store_gpiot(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int ret = 0;
        char *pvalue = NULL;
        unsigned int value = 0;

        pvalue = (char *)buf;
        ret = kstrtou32(pvalue, 16, (unsigned int *)&value);
        gpio_direction_output(ADC_SW_EN, value);
        printk(KERN_ERR "value=%d/n", value);
        printk(KERN_ERR "ADC_SW_EN = %d pull=%d\n",ADC_SW_EN,gpio_get_value(ADC_SW_EN));

        return size;

}
static DEVICE_ATTR(gpiot, 0664, show_gpiot, store_gpiot); 
 /**********************************************************
  *
  *   [External Function]
  *
  *********************************************************/
int us5587_set_gpio(int gpio, int pull)
{
	int err =0;
	printk(KERN_ERR "gpio = %d, pull %s\n",gpio, pull==1?"hi":"lo");
	err = gpio_direction_output(gpio, pull);
	if (err<0)
		printk(KERN_ERR "pull up/down fail, err = %d\n",err);
	else
		printk(KERN_ERR "pull up/down success\n");

	return err;
}
EXPORT_SYMBOL(us5587_set_gpio);
// Reg = 0x3, 0x4, 0x5, 0xB2
unsigned char us5587_reg_val(int Reg){
	int err = 0;
	unsigned char val = 0;
	err = us5587_read_byte(Reg, &val);	
	printk(KERN_ERR "us5587 Reg[%x]=0x%x\n",Reg ,val);
	return val;
}
EXPORT_SYMBOL(us5587_reg_val);

int us5587_enable(bool bEnable)
{
	int rc = 0;

	pr_info("%s: bEnable=%d\n", __func__, bEnable);
	if(bEnable)
	{
		// Enable us5587
		gpio_direction_output(ADCPWREN_PMI_GP1, 1);
		gpio_direction_output(ADC_SW_EN, 1);
	}
	else
	{
		// Disable us5587
		gpio_direction_output(ADCPWREN_PMI_GP1, 0);
		gpio_direction_output(ADC_SW_EN, 0);
	}
	
	return rc;
}
EXPORT_SYMBOL(us5587_enable);

int ADCPWR_enable(bool bEnable)
{
	int rc = 0;
	
	if(bEnable)
	{
		// Enable us5587 ADC , default actie high(Without DCP IN)
		gpio_direction_output(ADCPWREN_PMI_GP1, 1);
	}
	else
	{
		// Disable us5587 ADC
		gpio_direction_output(ADCPWREN_PMI_GP1, 0);
	}
		
	return rc;
}
EXPORT_SYMBOL(ADCPWR_enable);

int DPM_SW_enable(bool bEnable)
{
	int rc = 0;
	
	pr_info("%s: bEnable=%d\n", __func__, bEnable);
	if(bEnable)
	{
		// Switch D+/D- to ADC channel		
		gpio_direction_output(ADC_SW_EN, 1);
	}
	else
	{
		// Switch D+/D- to USB port		
		gpio_direction_output(ADC_SW_EN, 0);
	}
		
	return rc;
}
EXPORT_SYMBOL(DPM_SW_enable);

 /**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/

void us5587_get_gpio_info(void)
{
	static struct device_node *node=NULL;
	int err = 0;

	node = of_find_compatible_node(NULL, NULL, "upi,us5587");
	ADC_SW_EN = of_get_named_gpio(node, "adc_sw_en", 0);
	printk(KERN_ERR "ADC_SW_EN = %d\n",ADC_SW_EN);
	err = gpio_request(ADC_SW_EN,"ADC_SW_EN");
	if (err < 0)
		printk(KERN_ERR "%s: gpio_request failed for gpio %d\n", __func__, ADC_SW_EN);
	else
		printk(KERN_ERR "%s: gpio_request success for gpio %d\n", __func__, ADC_SW_EN);

	
	ADCPWREN_PMI_GP1 = of_get_named_gpio(node, "qcom,adcpwren_pmi_gp1",0);
	printk(KERN_ERR "ADCPWREN_PMI_GP1 = %d\n", ADCPWREN_PMI_GP1);

	err = gpio_request(ADCPWREN_PMI_GP1,"ADCPWREN_PMI_GP1");
        if (err < 0)
                printk(KERN_ERR "%s: gpio_request failed for gpio %d, err = %d\n", __func__, ADCPWREN_PMI_GP1,err);
        else
                printk(KERN_ERR "%s: gpio_request success for gpio %d\n", __func__, ADCPWREN_PMI_GP1);

}  
  
static int us5587_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	printk(KERN_ERR "[us5587_probe] \n");
	new_client = client;

	return 0;
}

static int us5587_remove(struct i2c_client *client)
{
	return 0;
}
struct platform_device us5587_user_space_device = {
                .name   = "us5587",
                .id     = -1,
};

static int us5587_user_space_probe(struct platform_device *dev)
{
    int ret_device_file = 0;
	printk(KERN_ERR "******** us5587_user_space_probe!! ********\n");
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_us5587_dump);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_gpiot);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_gpio_pmic);
	
	return 0;
}

static struct platform_driver us5587_user_space_driver = {
                .probe      = us5587_user_space_probe,
                .driver     = {
                                .name = "us5587",
                },
};


static const struct i2c_device_id us5587_i2c_id[] = { {"us5587", 0}, {} };

static struct i2c_driver us5587_i2c_driver = {
	.driver = {
		.name = "us5587",
		.owner = THIS_MODULE,
	},
	.probe = us5587_probe,
	.remove = us5587_remove,
	.id_table = us5587_i2c_id,
};

static int __init us5587_init(void)
{
	unsigned char us5587_reg = 0;
	int ret = 0;
	unsigned char RegNum;

	if(i2c_add_driver(&us5587_i2c_driver) != 0)
		printk(KERN_ERR "[us5587_init] failed to register us5587 i2c driver\n");
	else
		printk(KERN_ERR "[us5587_init] Success to register us5587 i2c driver\n");
	
	ret = platform_device_register(&us5587_user_space_device);
	if (ret){
		printk(KERN_ERR "[us5587_init] Unable to device register\n");
	}

	ret = platform_driver_register(&us5587_user_space_driver);
	if (ret){
		printk(KERN_ERR "[us5587_init] Unable to register driver\n");
	}

	us5587_get_gpio_info();

	RegNum = 0x03;
	ret = us5587_read_byte(RegNum, &us5587_reg);
	printk(KERN_ERR  "[us5587_init] Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0x04;
	ret = us5587_read_byte(RegNum, &us5587_reg);
	printk(KERN_ERR  "[us5587_init] Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0x05;
	ret = us5587_read_byte(RegNum, &us5587_reg);
	printk(KERN_ERR  "[us5587_init] Reg[%x]=0x%x\n",RegNum , us5587_reg);

	RegNum = 0xB2;
	ret = us5587_read_byte(RegNum, &us5587_reg);
	printk(KERN_ERR  "[us5587_init] Reg[%x]=0x%x\n",RegNum , us5587_reg);

	
	return 0;
	
}
subsys_initcall(us5587_init);

static void __exit us5587_cleanup(void)
{
	i2c_del_driver(&us5587_i2c_driver);
}
module_exit(us5587_cleanup);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("us5587 voltage regulator driver");
MODULE_LICENSE("GPL v2");
