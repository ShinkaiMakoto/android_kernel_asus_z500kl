/*
 * tps51632-regulator.c -- TI TPS6128x
 *
 * Regulator driver for TPS6128x 3-2-1 Phase D-Cap Step Down Driverless
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
//#include <linux/regulator/tps6128x-regulator.h>
#include <linux/slab.h>

#include <linux/regulator/driver.h>
/*CON3*/
#define CON3_REG_VOUTROOFSET    0x03
#define CON3_VOUTROOFSET_MASK   0x1F
#define CON3_VOUTROOFSET_SHIFT  0
/*CONFF*/
#define CONFF_REG_E2PROMCTRL    0xFF
#define CONFF_E2PROMCTRL_MASK   0x7
#define CONFF_E2PROMCTRL_SHIFT  5
/**********************************************************
 *
 *      [I2C Slave Setting]
 *
 *********************************************************/
#define tps6128x_SLAVE_ADDR_WRITE       0xEA
#define tps6128x_SLAVE_ADDR_READ        0xEB
static struct i2c_client *new_client;

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
static DEFINE_MUTEX(tps6128x_i2c_access);
/**********************************************************
 *
 *      [I2C Function For Read/Write tps6128x]
 *
 *********************************************************/
int tps6128x_read_byte(unsigned char cmd, unsigned char *returnData)
{
        char     readData = 0;
        int      ret = 0;
        struct i2c_msg msg[2];
        struct i2c_adapter *adap = new_client->adapter;

        mutex_lock(&tps6128x_i2c_access);
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
                mutex_unlock(&tps6128x_i2c_access);
                return 0;
        }
        *returnData = readData;

        mutex_unlock(&tps6128x_i2c_access);
        return 1;
}

int tps6128x_write_byte(unsigned char cmd, unsigned char writeData)
{
        char write_data[2] = { 0 };
        int ret = 0;
        struct i2c_msg msg;
        struct i2c_adapter *adap = new_client->adapter;

        mutex_lock(&tps6128x_i2c_access);
        write_data[0] = cmd;
        write_data[1] = writeData;
        msg.addr = new_client->addr;
        msg.flags = 0;
        msg.len = 2;
        msg.buf = (char *)write_data;

        ret = i2c_transfer(adap, &msg, 1);
        if (ret < 0) {
                mutex_unlock(&tps6128x_i2c_access);
                return 0;
        }

        mutex_unlock(&tps6128x_i2c_access);
        return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int tps6128x_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
                                  unsigned char SHIFT)
{
        unsigned char tps6128x_reg = 0;
        int ret = 0;

        printk(KERN_ERR "--------------------------------------------------\n");

        ret = tps6128x_read_byte(RegNum, &tps6128x_reg);

        printk(KERN_ERR "[tps6128x_read_interface] Reg[%x]=0x%x\n", RegNum, tps6128x_reg);

        tps6128x_reg &= (MASK << SHIFT);
        *val = (tps6128x_reg >> SHIFT);

        printk(KERN_ERR  "[tps6128x_read_interface] val=0x%x\n", *val);

        return ret;
}

unsigned int tps6128x_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
                                    unsigned char SHIFT)
{
        unsigned char tps6128x_reg = 0;
        int ret = 0;

        printk(KERN_ERR  "--------------------------------------------------\n");

        ret = tps6128x_read_byte(RegNum, &tps6128x_reg);
        printk(KERN_ERR  "[tps6128x_config_interface] Reg[%x]=0x%x\n", RegNum, tps6128x_reg);

        tps6128x_reg &= ~(MASK << SHIFT);
        tps6128x_reg |= (val << SHIFT);

        ret = tps6128x_write_byte(RegNum, tps6128x_reg);
        printk(KERN_ERR "[tps6128x_config_interface] write Reg[%x]=0x%x\n", RegNum, tps6128x_reg);

        /* Check */
        tps6128x_read_byte(RegNum, &tps6128x_reg); 
        printk(KERN_ERR  "[tps6128x_config_interface] Check Reg[%x]=0x%x\n", RegNum, tps6128x_reg); 

        return ret;
}


/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON3--03------------------------------------------------ */
void tps6128x_set_voutroofset(unsigned int val)
{
        unsigned int ret = 0;
        ret = tps6128x_config_interface((unsigned char) (CON3_REG_VOUTROOFSET),
                                       (unsigned char) (val),
                                       (unsigned char) (CON3_VOUTROOFSET_MASK),
                                       (unsigned char) (CON3_VOUTROOFSET_SHIFT)
            );
}

/* CONFF--FF------------------------------------------------ */
void tps6128x_set_e2promctrl(unsigned int val)
{
        unsigned int ret = 0;
        ret = tps6128x_config_interface((unsigned char) (CONFF_REG_E2PROMCTRL),
                                       (unsigned char) (val),
                                       (unsigned char) (CONFF_E2PROMCTRL_MASK),
                                       (unsigned char) (CONFF_E2PROMCTRL_SHIFT)
            );
}

/**********************************************************
  *
  *   [External Function]
  *
  *********************************************************/

void tps6128x_initial(void){
	unsigned char ret;

        tps6128x_set_e2promctrl(0x4);
	tps6128x_read_byte(0xFF, &ret);
	printk(KERN_ERR "%s: Reg[FF] = 0x%x \n", __func__, ret);

        tps6128x_set_voutroofset(0xF);
	tps6128x_read_byte(0x03, &ret);
        printk(KERN_ERR "%s: Reg[03] = 0x%x \n", __func__, ret);
}
EXPORT_SYMBOL(tps6128x_initial);

/* --------------------------------------------------------- */

static int tps6128x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	printk(KERN_ERR "[tps6128x_probe] \n");
	new_client = client;

	return 0;
}

static int tps6128x_remove(struct i2c_client *client)
{
	return 0;
}
struct platform_device tps6128x_user_space_device = {
                .name   = "tps6128x",
                .id     = -1,
};

static int tps6128x_user_space_probe(struct platform_device *dev)
{

	printk(KERN_ERR "******** tps6128x_user_space_probe!! ********\n");

        return 0;
}

static struct platform_driver tps6128x_user_space_driver = {
                .probe      = tps6128x_user_space_probe,
                .driver     = {
                                .name = "tps6128x",
                },
};


static const struct i2c_device_id tps6128x_i2c_id[] = { {"tps6128x", 0}, {} };

static struct i2c_driver tps6128x_i2c_driver = {
	.driver = {
		.name = "tps6128x",
		.owner = THIS_MODULE,
	},
	.probe = tps6128x_probe,
	.remove = tps6128x_remove,
	.id_table = tps6128x_i2c_id,
};

static int __init tps6128x_init(void)
{
//	unsigned char tps6128x_reg = 0;
	int ret = 0;
//	unsigned char RegNum;

	if(i2c_add_driver(&tps6128x_i2c_driver) != 0)
		printk(KERN_ERR "[tps6128x_init] failed to register tps6128x i2c driver\n");
	else
		printk(KERN_ERR "[tps6128x_init] Success to register tps6128x i2c driver\n");
	
	ret = platform_device_register(&tps6128x_user_space_device);
	if (ret){
		printk(KERN_ERR "[tps6128x_init] Unable to device register\n");
	}

	ret = platform_driver_register(&tps6128x_user_space_driver);
	if (ret){
		printk(KERN_ERR "[tps6128x_init] Unable to register driver\n");
	}

        tps6128x_set_e2promctrl(0x4);
        tps6128x_set_voutroofset(0xF);
//        tps6128x_set_e2promctrl(0x3);

	return 0;
	
}
subsys_initcall(tps6128x_init);

static void __exit tps6128x_cleanup(void)
{
	i2c_del_driver(&tps6128x_i2c_driver);
}
module_exit(tps6128x_cleanup);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("TPS6128x voltage regulator driver");
MODULE_LICENSE("GPL v2");
