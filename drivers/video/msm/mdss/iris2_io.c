#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-qpnp-wled.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/pm_qos.h>
#include <iris2_io.h>

struct clk *s_clk;
/* performance hack */
struct clk *bimc_clk;
struct clk *snoc_clk;
int iris_1v1_1v8_en_gpio, iris_rst_gpio;
extern int Read_HW_ID(void);
int pw_iris2_chip_version = IRIS2_28;

int pw_iris2_id = 0x1;
MODULE_PARM_DESC(pw_iris2_id, "Pixelworks Iris2 ID");
module_param_named(pw_iris2_id, pw_iris2_id, int, 0644);

int pw_iris2_status = -1;
MODULE_PARM_DESC(pw_iris2_status, "Pixelworks Iris2 Status");
module_param_named(pw_iris2_status, pw_iris2_status, int, 0644);

extern void ft5726_touch_power_enable(void); // add by leo ++

void iris2_io_bbclk_enable(int on){
	int ret=0;

	if(on){
		ret = clk_prepare_enable(s_clk);
		printk("[DISPLAY] %s: clk_prepare_enable ret = %d \n", __func__, ret);
	}else{
		clk_disable_unprepare(s_clk);
		printk("[DISPLAY] %s: clk_disable_unprepare\n", __func__);
	}

	return;
}
EXPORT_SYMBOL(iris2_io_bbclk_enable);

void iris2_io_bimc_enable(int on) {
	int ret = 0;
	if (bimc_clk == NULL) {
		printk("[DISPLAY] %s: bimc_clk is NULL\n", __func__);
		return;
	}
	if (on) {
		ret = clk_prepare_enable(bimc_clk);
		printk("[DISPLAY] %s: clk_prepare_enable ret = %d \n", __func__, ret);
	} else {
		clk_disable_unprepare(bimc_clk);
		printk("[DISPLAY] %s: clk_disable_unprepare\n", __func__);
	}

	return;
}
EXPORT_SYMBOL(iris2_io_bimc_enable);

void iris2_io_snoc_enable(int on) {
	int ret = 0;
	if (snoc_clk == NULL) {
		printk("[DISPLAY] %s: snoc_clk is NULL\n", __func__);
		return;
	}
	if (on) {
		ret = clk_prepare_enable(snoc_clk);
		printk("[DISPLAY] %s: clk_prepare_enable ret = %d \n", __func__, ret);
	} else {
		clk_disable_unprepare(snoc_clk);
		printk("[DISPLAY] %s: clk_disable_unprepare\n", __func__);
	}

	return;
}
EXPORT_SYMBOL(iris2_io_snoc_enable);

void iris2_io_suspend(void)
{
	if (gpio_is_valid(iris_rst_gpio)) {
		gpio_set_value((iris_rst_gpio), 0);
	}
	if (gpio_is_valid(iris_1v1_1v8_en_gpio)) {
		gpio_set_value((iris_1v1_1v8_en_gpio), 0);
	}

	if(iris2_get_chip_version()==IRIS2_26 || iris2_get_chip_version()>=IRIS2P_50){
		//do nothing
	}else{
		iris2_io_bbclk_enable(0);
	}
#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2P_FULL_SUPPORT)
	usleep_range(400, 600);
#else
	usleep_range(500, 500);
#endif
}
EXPORT_SYMBOL(iris2_io_suspend);

void iris2_io_resume(void)
{
	int rc = 0;

#ifdef CONFIG_TOUCHSCREEN_FT5726
        ft5726_touch_power_enable(); // add by leo ++
#endif

	if(iris2_get_chip_version()==IRIS2_26 || iris2_get_chip_version()>=IRIS2P_50){
		//do nothing
	}else{
		iris2_io_bbclk_enable(1);
	}

#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2P_FULL_SUPPORT)
	usleep_range(2000, 4000);
#else
	usleep_range(3000, 3000);
#endif

	if (gpio_is_valid(iris_1v1_1v8_en_gpio)) {
		rc = gpio_direction_output(iris_1v1_1v8_en_gpio, 1);
	    if (rc)
		 pr_err("%s: unable to set dir for iris_1v1_1v8_en_gpio gpio\n",
			  __func__);
	}
#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2P_FULL_SUPPORT)
	usleep_range(2000, 4000);
#else
	usleep_range(3000, 3000);
#endif
	if (gpio_is_valid(iris_rst_gpio)) {
		rc = gpio_direction_output(iris_rst_gpio, 1);

		if(iris2_get_chip_version()==IRIS2_26 || iris2_get_chip_version()>=IRIS2P_50){
			//do nothing
		}else{
#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2P_FULL_SUPPORT)
			usleep_range(10000, 15000);
#else
			usleep_range(15000, 15000);
#endif
			rc |= gpio_direction_output(iris_rst_gpio, 0);
			udelay(5);
		}
		rc |= gpio_direction_output(iris_rst_gpio, 1);
#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2P_FULL_SUPPORT)
		//usleep_range(60000, 60000);
#else
		usleep_range(60000, 60000);
#endif

		if (rc)
			pr_err("%s: unable to set dir for iris_rst_gpio gpio\n",  __func__);
	}
}
EXPORT_SYMBOL(iris2_io_resume);

void iris2_io_reset(void)
{
	if (gpio_is_valid(iris_rst_gpio)) {
		gpio_set_value(iris_rst_gpio, 0);
		usleep_range(10000, 15000);
		gpio_set_value(iris_rst_gpio, 1);
		usleep_range(60000, 60000);
	}
}
EXPORT_SYMBOL(iris2_io_reset);

void iris2_io_set_reset(int use_direction, int value)
{
	if (gpio_is_valid(iris_rst_gpio)) {
		if (use_direction) {
			gpio_direction_output(iris_rst_gpio, value);
		} else {
			gpio_set_value(iris_rst_gpio, value);
		}
	}
}
EXPORT_SYMBOL(iris2_io_set_reset);

static int iris2_io_probe(struct platform_device *pdev)
{
	int rc;
	int hw_id;

	pr_info("[DISPLAY] %s: Enter\n", __func__);

	hw_id = Read_HW_ID();

#if defined(CONFIG_IRIS2_FULL_SUPPORT) || defined(CONFIG_IRIS2_LIGHTUP_ONLY)
#if defined(TARGET_PROJECT_ZT581KL_DISPLAY)
	if((hw_id == 0)||(hw_id == 1))
	{
		pw_iris2_chip_version = IRIS2_28;
	}
	else
#endif
	{
		pw_iris2_chip_version = IRIS2_26; // PX8459WM-01(external crystal 19.2MHz)
	}
#elif defined(CONFIG_IRIS2P_FULL_SUPPORT) || defined(CONFIG_IRIS2P_LIGHTUP_ONLY)
	pw_iris2_chip_version = IRIS2P_50; // PX8468WM
#endif

	printk(KERN_CRIT "[DISPLAY] hw_id = %d, pw_iris2_chip_version = %d\n", hw_id, pw_iris2_chip_version);


	//gpio

	iris_1v1_1v8_en_gpio = of_get_named_gpio(pdev->dev.of_node, "qcom,iris-1v1-1v8-en-gpio", 0);
	if (!gpio_is_valid(iris_1v1_1v8_en_gpio))
		pr_info("%s: iris_1v1_1v8_en_gpio gpio not specified\n", __func__);
	else {
		rc = gpio_request(iris_1v1_1v8_en_gpio, "iris_1v1_1v8_en_gpio");
		if (rc)
			pr_err("request iris_1v1_1v8_en_gpio gpio failed, rc=%d\n", rc);

	}

	iris_rst_gpio = of_get_named_gpio(pdev->dev.of_node, "qcom,iris-rst-gpio", 0);
	if (!gpio_is_valid(iris_rst_gpio))
		pr_info("%s: iris_rst_gpio gpio not specified\n", __func__);
	else {
		rc = gpio_request(iris_rst_gpio, "iris_rst_gpio");
		if (rc) {
			pr_err("request iris_rst_gpio gpio failed, rc=%d\n", rc);
		}
	}

	bimc_clk = clk_get(&pdev->dev, "bimc_clk");
	if (bimc_clk == NULL)
		printk("[DISPLAY] bimc_clk == NULL\n");
	else
		printk("[DISPLAY] bimc_clk != NULL\n");

	snoc_clk = clk_get(&pdev->dev, "snoc_clk");
	if (snoc_clk == NULL)
		printk("[DISPLAY] snoc_clk == NULL\n");
	else
		printk("[DISPLAY] snoc_clk != NULL\n");

	//clock
	s_clk  = clk_get(&pdev->dev, "iris2_clk");
	if (s_clk == NULL)
		printk("[DISPLAY] s_clk == NULL\n");
	else
		printk("[DISPLAY] s_clk != NULL\n");

	iris2_io_bbclk_enable(1);

	if(iris2_get_chip_version()==IRIS2_26 || iris2_get_chip_version()>=IRIS2P_50){
		iris2_io_bbclk_enable(0);
	}

	return 0;
}

static struct of_device_id iris2_io_dt_match[] = {
	{ .compatible = "pw,iris2_io"}, //Compatible node must match dts
	{ },
};

MODULE_DEVICE_TABLE(of, iris2_io_dt_match);

static struct platform_driver iris2_io_driver = {
	.probe = iris2_io_probe,
	.driver = {
		.name = "iris2_io",
		.of_match_table = iris2_io_dt_match,
	},
};

static int __init iris2_io_init(void)
{
	int ret = 0;

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	ret = platform_driver_register(&iris2_io_driver);

	return ret;
}

static void __exit iris2_io_exit(void)
{
	platform_driver_unregister(&iris2_io_driver);
}

int iris2_get_chip_version(void)
{
	return pw_iris2_chip_version;
}

fs_initcall(iris2_io_init);
module_exit(iris2_io_exit);

MODULE_DESCRIPTION("iris2_io");
MODULE_LICENSE("GPL v2");
