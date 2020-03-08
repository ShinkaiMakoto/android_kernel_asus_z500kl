#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <linux/power/notify.h>
#include <linux/regulator/consumer.h>

/* The following constants (followed by BATT_XXX) come from charger porting
 * guild. */
#define BATT_SHUTDOWN_TH 1
#define BATT_SHUTDOWN_TEMPERATURE_TH 450 /* 45 degree */


#define POWER_CAPACITY_INIT_VALUE -1
#define POWER_TEMPERATURE_INIT_VALUE -999*10

struct typec_core_state {
	u32 vbus:1,
		vcon:1,
		data:2,
		pd_has_contract:1,
		giveup_get_vbus_reg:1,
		giveup_get_usb_phy:1;
};

typedef enum {
	DATA_ROLE_NONE = 0,
	DATA_ROLE_DOWN = 1,
	DATA_ROLE_UP = 2,
} typec_core_data_role;

extern bool wait_usb_phy_ready(void);
extern int usb_peripheral_mode(bool);
static void typc_core_handle_battery_capacity_change(int previous_batt_capacity,
					      int batt_cap);
static void typc_core_handle_battery_temperature_change(int previous_temperature,
						 int current_temperature);
static int battery_capacity_notify(struct notifier_block *self,
				unsigned long action, void *priv);

struct rt_typec_pd_info {
	struct device *dev;
	struct notifier_block nb;
	struct tcpc_device *tcpc;
	struct mutex state_lock;
	struct tcp_ny_typec_state typec_state;
	struct tcp_ny_pd_state pd_state;
	struct tcp_ny_vbus_state vbus_state;
	int mux_sel_gpio;
	struct notifier_block power_nb;
	atomic_t latest_batt_capacity;
	atomic_t latest_batt_temperature;
	struct typec_device_ops *ops;
	struct typec_core_state state;
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
	struct regulator *vbus_reg;
	struct power_supply *usb_psy;
#endif
};
static struct rt_typec_pd_info rt_info;

#define rt_info_set_field(field, v)			\
	do {						\
		mutex_lock(&rt_info.state_lock);	\
		rt_info.field = (v);			\
		mutex_unlock(&rt_info.state_lock);	\
							\
	} while(0)

#define rt_info_get_field(field, v)			\
	do {						\
		mutex_lock(&rt_info.state_lock);	\
		v = rt_info.field;			\
		mutex_unlock(&rt_info.state_lock);	\
							\
	} while(0)

struct rt1711_chip *chip;

/**
 * Invoked by typec-core/typec_current_mode_detect for charger to detect
 * the termination type when we are SINK_ATTACHED.
 * @return The detected current mode
 *   TYPEC_CURRENT_MODE_UNSPPORTED: When the attached mode is not sink attached
 *   TYPEC_CURRENT_MODE_DEFAULT: 500mA
 *   TYPEC_CURRENT_MODE_MID: 1.5A
 *   TYPEC_CURRENT_MODE_HIGH: 3A
 */
static enum typec_current_mode rt1711_current_mode_detect(void)
{
	enum typec_current_mode cmode;
	struct tcp_ny_typec_state typec_state;

	rt_info_get_field(typec_state, typec_state);
	pr_info("RT1711 %s: rp_level=%d, polarity=%d, old_state=%d, new_state=%d\n",
		 __func__, typec_state.rp_level, typec_state.polarity,
		 typec_state.old_state, typec_state.new_state);

	if (typec_state.new_state != TYPEC_ATTACHED_SNK) {
		return TYPEC_CURRENT_MODE_UNSPPORTED;
	}
	switch (typec_state.rp_level) {
	case TYPEC_CC_VOLT_SNK_1_5:
		cmode = TYPEC_CURRENT_MODE_MID;
		break;
	case TYPEC_CC_VOLT_SNK_3_0:
		cmode = TYPEC_CURRENT_MODE_HIGH;
		break;
	case TYPEC_CC_VOLT_SNK_DFT:
	default:
		cmode = TYPEC_CURRENT_MODE_DEFAULT;

	}
	return cmode;
}

static enum typec_attached_state rt1711_attatched_state_detect(void)
{
	enum typec_attached_state attach_state;
	struct tcp_ny_typec_state typec_state;

	rt_info_get_field(typec_state, typec_state);
	pr_info("RT1711 %s: rp_level=%d, polarity=%d, old_state=%d, new_state=%d\n",
		 __func__, typec_state.rp_level, typec_state.polarity,
		 typec_state.old_state, typec_state.new_state);

	/* map 'enum typec_attach_type' (RT specific) to 'enum typec_attached_state' */
	switch (typec_state.new_state) {
	case TYPEC_ATTACHED_SNK:
		attach_state = TYPEC_ATTACHED_AS_UFP;
		break;
	case TYPEC_ATTACHED_SRC:
		attach_state = TYPEC_ATTACHED_AS_DFP;
		break;
	case TYPEC_ATTACHED_AUDIO:
	case TYPEC_ATTACHED_DEBUG:
		attach_state = TYPEC_ATTACHED_TO_ACCESSORY;
		break;
	default:
		attach_state = TYPEC_NOT_ATTACHED;
	}
	return attach_state;
}

static enum typec_current_mode rt1711_current_advertise_get(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_CURRENT_MODE_DEFAULT;
}

static int rt1711_current_advertise_set(enum typec_current_mode current_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat /sys/class/typec/typec_device/port_mode_ctrl'
 * Besides BSP/ATD tools, no one will access it. */
static enum typec_port_mode rt1711_port_mode_get(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_MODE_ACCORDING_TO_PROT;
}
/* call from 'echo 0|1|2|3 > /sys/class/typec/typec_device/port_mode_ctrl'
 *  0: TYPEC_MODE_ACCORDING_TO_PROT
 *  1: TYPEC_UFP_MODE
 *  2: TYPEC_DFP_MODE
 *  3: TYPEC_DRP_MODE
 * Besides BSP/ATD tools, no one will access it.
 * */
static int rt1711_port_mode_set(enum typec_port_mode port_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat /sys/class/typec/typec_device/dump_regs'
 * Besides BSP/ATD tools, no one will access it.
 * */
static ssize_t rt1711_dump_regs(char *buf)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat /sys/class/typec/typec_device/i2c_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static bool rt1711_i2c_status(void)
{
	struct i2c_client *client = chip->client;
	u16 vid, pid;
	int ret;

	ret = rt1711_read_device(client, TCPC_V10_REG_VID, 2, &vid);
	if (ret < 0) {
		dev_err(&client->dev, "read chip ID fail\n");
		return 0;
	}

	if (vid != RICHTEK_1711_VID) {
		pr_info("%s failed, VID=0x%04x\n", __func__, vid);
		return 0;
	}

	ret = rt1711_read_device(client, TCPC_V10_REG_PID, 2, &pid);
	if (ret < 0) {
		dev_err(&client->dev, "read product ID fail\n");
		return 0;
	}

	if (pid != RICHTEK_1711_PID) {
		pr_info("%s failed, PID=0x%04x\n", __func__, pid);
		return 0;
	}

	return 1;
}

static bool rt1711_pd_wait_for_source_caps(void)
{
	//return fusb_pd_wait_for_source_caps();
	return 0;
}

/* call from 'cat /sys/class/typec/typec_device/cc_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static int rt1711_cc_status(void)
{
	int ret, cc, cc1, cc2;

	ret = rt_info.tcpc->ops->get_cc(rt_info.tcpc, &cc1, &cc2);
	if (ret < 0)
		return ret;

	cc1 = !!cc1;
	cc2 = !!cc2;

	if (cc1 ^ cc2)
		cc = cc1 ? 1 : 2;
	else
		cc = 0;

	return cc;
}

static enum power_supply_type rt1711_pd_power_supply_type(void)
{
	return POWER_SUPPLY_TYPE_USB_PD_DRP;
}

static void rt1711_pd_reevaluate_contract(bool force_vsafe_5v)
{
	/* ZT582KL's sink cap is 5V only. No need to re-evaluate contract */
}

static int rt1711_get_battery_capacity(void)
{
	return atomic_read(&rt_info.latest_batt_capacity);
}

static void rt1711_set_battery_capacity(int cap)
{
	int previous_value = atomic_read(&rt_info.latest_batt_capacity);
	atomic_set(&rt_info.latest_batt_capacity, cap);
	typc_core_handle_battery_capacity_change(previous_value, cap);
}

static int rt1711_get_battery_temperature(void)
{
	return atomic_read(&rt_info.latest_batt_temperature);
}

static void rt1711_set_battery_temperature(int temp)
{
	int previous_value = atomic_read(&rt_info.latest_batt_temperature);
	atomic_set(&rt_info.latest_batt_temperature, temp);
	typc_core_handle_battery_temperature_change(previous_value, temp);
}

struct typec_device_ops rt1711_typec_ops = {
	.current_detect = rt1711_current_mode_detect,
	.attached_state_detect = rt1711_attatched_state_detect,
	.current_advertise_get = rt1711_current_advertise_get,
	.current_advertise_set = rt1711_current_advertise_set,
	.port_mode_get = rt1711_port_mode_get,
	.port_mode_set = rt1711_port_mode_set,
	.dump_regs = rt1711_dump_regs,
	.i2c_status = rt1711_i2c_status,
	.cc_status = rt1711_cc_status,
	.pd_wait_for_source_caps = rt1711_pd_wait_for_source_caps,
	.pd_power_supply_type = rt1711_pd_power_supply_type,
	.pd_reevaluate_contract = rt1711_pd_reevaluate_contract,
	.get_battery_capacity = rt1711_get_battery_capacity,
	.set_battery_capacity = rt1711_set_battery_capacity,
	.get_battery_temperature = rt1711_get_battery_temperature,
	.set_battery_temperature = rt1711_set_battery_temperature,
};


static inline const char* notify_evt_string(unsigned long evt)
{
	const char *evt_str[] = {
		[TCP_NOTIFY_DIS_VBUS_CTRL]    = "TCP_NOTIFY_DIS_VBUS_CTRL",
		[TCP_NOTIFY_SOURCE_VCONN]     = "TCP_NOTIFY_SOURCE_VCONN",
		[TCP_NOTIFY_SOURCE_VBUS]      = "TCP_NOTIFY_SOURCE_VBUS",
		[TCP_NOTIFY_SINK_VBUS]	      = "TCP_NOTIFY_SINK_VBUS",
		[TCP_NOTIFY_PR_SWAP]	      = "TCP_NOTIFY_PR_SWAP",
		[TCP_NOTIFY_DR_SWAP]	      = "TCP_NOTIFY_DR_SWAP",
		[TCP_NOTIFY_VCONN_SWAP]	      = "TCP_NOTIFY_VCONN_SWAP",
		[TCP_NOTIFY_ENTER_MODE]	      = "TCP_NOTIFY_ENTER_MODE",
		[TCP_NOTIFY_EXIT_MODE]	      = "TCP_NOTIFY_EXIT_MODE",
		[TCP_NOTIFY_AMA_DP_STATE]     = "TCP_NOTIFY_AMA_DP_STATE",
		[TCP_NOTIFY_AMA_DP_ATTENTION] = "TCP_NOTIFY_AMA_DP_ATTENTION",
		[TCP_NOTIFY_AMA_DP_HPD_STATE] = "TCP_NOTIFY_AMA_DP_HPD_STATE",
		[TCP_NOTIFY_TYPEC_STATE]      = "TCP_NOTIFY_TYPEC_STATE",
		[TCP_NOTIFY_PD_STATE]	      = "TCP_NOTIFY_PD_STATE",
#ifdef CONFIG_USB_PD_UVDM
		[TCP_NOTIFY_UVDM]	      = "TCP_NOTIFY_UVDM",
#endif /* CONFIG_USB_PD_UVDM */

#ifdef CONFIG_USB_PD_ALT_MODE_RTDC
		[TCP_NOTIFY_DC_EN_UNLOCK] = "TCP_NOTIFY_DC_EN_UNLOCK",
#endif	/* CONFIG_USB_PD_ALT_MODE_RTDC */

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK
		[TCP_NOTIFY_ATTACHWAIT_SNK] = "TCP_NOTIFY_ATTACHWAIT_SNK",
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC
		[TCP_NOTIFY_ATTACHWAIT_SRC] = "TCP_NOTIFY_ATTACHWAIT_SRC",
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC */

		[TCP_NOTIFY_EXT_DISCHARGE] = "TCP_NOTIFY_EXT_DISCHARGE",
	};
	return evt < ARRAY_SIZE(evt_str) ? evt_str[evt] : "unknown";
}

static void typec_core_update_vbus_role(bool source)
{
	bool giveup = rt_info.state.giveup_get_vbus_reg != 0;
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
	int ret;
	bool vbus_state = rt_info.state.vbus != 0;
	struct regulator *reg;
	unsigned long timeout;
	struct timespec bootime;

	if (source == vbus_state)
		return;		/* sync. nothing to do. */

	if (!rt_info.vbus_reg && !giveup) {
		get_monotonic_boottime(&bootime);
		if (bootime.tv_sec >= 60) {
			timeout = jiffies + (1*HZ);
		} else {
			timeout = jiffies + (60 - bootime.tv_sec)*HZ;
		}
		do {
			reg = devm_regulator_get(rt_info.dev, "vbus_otg");
			if (!IS_ERR_OR_NULL(reg)) {
				rt_info.vbus_reg = reg;
				break;
			}
			/* regulator still not ready, retry again later */
			dev_err(rt_info.dev, "get vbus_otg fail %ld\n", PTR_ERR(reg));
			msleep(100);
		} while(time_before(jiffies, timeout));

		if (time_after(jiffies, timeout)) {
			dev_err(rt_info.dev, "give-up getting vbus-otg regulator\n");
			giveup = true;
		}
	}
        if (rt_info.vbus_reg) {
		wait_usb_phy_ready();
		/* We have the reference of regulator AND VBUS state is not sync. */
		pr_info("%s: %s vbus_otg\n", __func__, source ? "enable" : "disable");
		ret = source ? regulator_enable(rt_info.vbus_reg):
			regulator_disable(rt_info.vbus_reg);
		if (ret)
			dev_err(rt_info.dev, "unable to enable VBUS regulator.\n");
	}
#endif
	mutex_lock(&rt_info.state_lock);
	rt_info.state.vbus = source ? 1 : 0;
	if (giveup)
		rt_info.state.giveup_get_vbus_reg = 1;
	mutex_unlock(&rt_info.state_lock);
}

void typec_core_update_vconn_role(bool source)
{
	mutex_lock(&rt_info.state_lock);
	rt_info.state.vcon = source ? 1 : 0;
	mutex_unlock(&rt_info.state_lock);
}

void typec_core_update_data_role(typec_core_data_role role)
{
	bool giveup = rt_info.state.giveup_get_usb_phy != 0;
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
	struct power_supply *usb_psy;
	unsigned long timeout;
	struct timespec bootime;

	if (role == rt_info.state.data)
		return;

	if (!rt_info.usb_psy && !giveup) {
		get_monotonic_boottime(&bootime);
		if (bootime.tv_sec >= 60) {
			timeout = jiffies + (1*HZ);
		} else {
			timeout = jiffies + (60 - bootime.tv_sec)*HZ;
		}
		do {
			usb_psy = power_supply_get_by_name("usb");
			if (!IS_ERR_OR_NULL(usb_psy)) {
				rt_info.usb_psy = usb_psy;
				break;
			}
			/* USB psy is still not ready, retry again later */
			dev_err(rt_info.dev, "get usb_psy fail %ld\n", PTR_ERR(usb_psy));
			msleep(100);
		} while(time_before(jiffies, timeout));

		if (time_after(jiffies, timeout)) {
			dev_err(rt_info.dev, "give-up getting usb_psy\n");
			giveup = true;
		}
	}
	if (rt_info.usb_psy) {
		wait_usb_phy_ready();
		pr_info("%s: %d\n", __func__, role);
		switch (role) {
		case DATA_ROLE_DOWN:
			usb_peripheral_mode(true);
			break;
		case DATA_ROLE_UP:
			power_supply_set_usb_otg(rt_info.usb_psy, 1);
			break;
		case DATA_ROLE_NONE:
		default:
			power_supply_set_usb_otg(rt_info.usb_psy, 0);
			usb_peripheral_mode(false);
		}
	}
#endif
	mutex_lock(&rt_info.state_lock);
	rt_info.state.data = role;
	if (giveup)
		rt_info.state.giveup_get_usb_phy = 1;
	mutex_unlock(&rt_info.state_lock);
}

/* Please handle the notification in notifier call function,
 * User should control the Power here when you got SOURCE_VBUS notification
 * and SINK_VBUS notification
 */
static int chg_tcp_notifer_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *tcp_noti = data;
	enum typec_attached_state attach_state;

	pr_debug("RT1711 %s with data: %p\n", notify_evt_string(event), data);

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
	case TCP_NOTIFY_DR_SWAP:
	case TCP_NOTIFY_VCONN_SWAP:
		/* Do what you want to do here */
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		/* Implement source vbus behavior here */
		pr_info("RT1711 NOTIFY_SOURCE_VBUS: %d mV, %d mA\n",
			tcp_noti->vbus_state.mv, tcp_noti->vbus_state.ma);
		typec_core_update_vbus_role(tcp_noti->vbus_state.mv > 0);
		rt_info_set_field(vbus_state, tcp_noti->vbus_state);
		break;
	case TCP_NOTIFY_SINK_VBUS_PREREQUISITE:
		typec_pd_on_sink_ready_prerequisite(tcp_noti->pdo_tobe_request.pdo, 0);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		/* Implement sink vubs behavior here */
		pr_info("RT1711 NOTIFY_SINK_VBUS: %d mV, %d mA, type=%d\n",
			tcp_noti->vbus_state.mv, tcp_noti->vbus_state.ma,
			tcp_noti->vbus_state.type);
		typec_core_update_vbus_role(false);
		rt_info_set_field(vbus_state, tcp_noti->vbus_state);
		if (tcp_noti->vbus_state.type == TCP_VBUS_CTRL_REMOVE ||
		    tcp_noti->vbus_state.mv == 0) {
			typec_pd_on_sink_ready(0);
		} else if (tcp_noti->vbus_state.type == TCP_VBUS_CTRL_PD_REQUEST) {
			typec_pd_on_sink_ready_plain_code(tcp_noti->vbus_state.mv,
				tcp_noti->vbus_state.ma, 0);
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		rt_info_set_field(pd_state, tcp_noti->pd_state);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		rt_info_set_field(typec_state, tcp_noti->typec_state);
		if (gpio_is_valid(rt_info.mux_sel_gpio)) {
			/* set MUX select according to CC state
			 * TODO. need to ensure we will be notified when cable unplugged. */
			gpio_set_value(rt_info.mux_sel_gpio,
				       (tcp_noti->typec_state.polarity) ? 0 : 1);
		}
		if (rt_info.ops->attached_state_detect) {
			attach_state = (rt_info.ops->attached_state_detect)();
			switch (attach_state) {
			case TYPEC_ATTACHED_AS_DFP:
				typec_core_update_data_role(DATA_ROLE_UP);
				break;
			case TYPEC_ATTACHED_AS_UFP:
				typec_core_update_data_role(DATA_ROLE_DOWN);
				break;
			default:
				typec_core_update_data_role(DATA_ROLE_NONE);
			}
		}
		break;
	default:;
	};

	return NOTIFY_OK;
}

static void platform_extra_init(struct device *dev)
{
	int ret;
	struct rt_typec_pd_info *info = &rt_info;
	struct device_node *np = dev->of_node;
	int gpio = of_get_named_gpio(np, "asus,mux_sel_pin", 0);

	pr_err("%s: got gpio of asus,mux_sel_pin: %d\n", __func__, gpio);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW, "MUX_SEL");
		if (ret < 0)
			pr_err("%s: MUX_SEL request error, ret=%d\n", __func__, ret);
	}
	info->mux_sel_gpio = gpio;
}

int rt1711_typec_ops_init(struct rt1711_chip *rt_chip)
{
	int ret;
	struct rt_typec_pd_info *info = &rt_info;

	chip = rt_chip;
	/* Get tcpc device by tcpc_device'name */
	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->tcpc) {
		pr_err("%s: get rt1711-tcpc fail\n", __func__);
		return -ENODEV;
	}

	info->ops = &rt1711_typec_ops;
	rt_info.dev = rt_chip->dev;
	platform_extra_init(rt_info.dev);
#if defined(CONFIG_USB_DWC3_PD_DUAL_ROLE_SWAP)
	rt_info.vbus_reg = devm_regulator_get(rt_info.dev, "vbus_otg");
	if (IS_ERR(rt_info.vbus_reg) &&
	    PTR_ERR(rt_info.vbus_reg) == -EPROBE_DEFER) {
		/* regulators may not be ready, so retry again later */
		rt_info.vbus_reg = NULL;
	}
	rt_info.usb_psy = power_supply_get_by_name("usb");
	if (!rt_info.usb_psy) {
		dev_err(rt_info.dev, "%s : USB supply not found\n", __func__);
	}
#endif
	/* register notifier for battery capacity and temperature*/
	rt_info.power_nb.notifier_call = battery_capacity_notify;
	atomic_set(&rt_info.latest_batt_capacity, POWER_CAPACITY_INIT_VALUE);
	atomic_set(&rt_info.latest_batt_temperature, POWER_TEMPERATURE_INIT_VALUE);
	power_register_notify(&rt_info.power_nb);

	mutex_init(&rt_info.state_lock);
	/* register tcpc notifier */
	info->nb.notifier_call = chg_tcp_notifer_call;
	ret = register_tcp_dev_notifier(info->tcpc, &info->nb);
	if (ret < 0) {
		mutex_destroy(&rt_info.state_lock);
		pr_err("%s: register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}
	return 0;
}

void rt1711_typec_ops_remove(struct rt1711_chip *rt_chip)
{
	typec_core_update_vbus_role(false);
	mutex_destroy(&rt_info.state_lock);
}

#ifdef CONFIG_PM
int rt1711_typec_ops_suspend(struct device *dev)
{
	if (gpio_is_valid(rt_info.mux_sel_gpio)) {
		/* Always set MUX select to low in S3 */
		gpio_set_value(rt_info.mux_sel_gpio, 0);
	}
	return 0;
}

int rt1711_typec_ops_resume(struct device *dev)
{
	/* NOP */
	return 0;
}
#endif


static void typc_core_handle_battery_capacity_change(int previous_batt_capacity,
						     int batt_cap)
{
	if (previous_batt_capacity == POWER_CAPACITY_INIT_VALUE) {
		/* This is the first time to update battery capacity.
		 * Only consider the current capacity. */
		if ((batt_cap < BATT_SHUTDOWN_TH) &&
		    (rt_info.ops->pd_reevaluate_contract))
			(rt_info.ops->pd_reevaluate_contract)(true);
		return;
	}
	if (batt_cap > previous_batt_capacity) {
		/* going up. */
		if ((batt_cap >= BATT_SHUTDOWN_TH) &&
		    (previous_batt_capacity < BATT_SHUTDOWN_TH)) {
			if (rt_info.ops->pd_reevaluate_contract)
				(rt_info.ops->pd_reevaluate_contract)(false);
		}
	} else if (batt_cap < previous_batt_capacity) {
		/* going down */
		if ((batt_cap < BATT_SHUTDOWN_TH) &&
		    (previous_batt_capacity >= BATT_SHUTDOWN_TH)) {
			if (rt_info.ops->pd_reevaluate_contract)
				(rt_info.ops->pd_reevaluate_contract)(true);
		}
	}
}

static void typc_core_handle_battery_temperature_change(int previous_temperature,
							int current_temperature)
{
	if (previous_temperature == POWER_TEMPERATURE_INIT_VALUE) {
		/* This is the first time to update battery temperature.
		 * Only consider the current temperature. */
		if ((current_temperature >= BATT_SHUTDOWN_TEMPERATURE_TH) &&
		    (rt_info.ops->pd_reevaluate_contract))
			(rt_info.ops->pd_reevaluate_contract)(true);
		return;
	}
	if (current_temperature > previous_temperature) {
		/* going up. */
		if ((current_temperature >= BATT_SHUTDOWN_TEMPERATURE_TH) &&
		    (previous_temperature < BATT_SHUTDOWN_TEMPERATURE_TH)) {
			if (rt_info.ops->pd_reevaluate_contract)
				(rt_info.ops->pd_reevaluate_contract)(true);
		}
	} else if (current_temperature < previous_temperature) {
		/* going down */
		if ((current_temperature < BATT_SHUTDOWN_TEMPERATURE_TH) &&
		    (previous_temperature >= BATT_SHUTDOWN_TEMPERATURE_TH)) {
			if (rt_info.ops->pd_reevaluate_contract)
				(rt_info.ops->pd_reevaluate_contract)(false);
		}
	}
}

static int battery_capacity_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	uintptr_t temp;
	int previous_value;

	temp = (uintptr_t)priv;
	if (action == POWER_CAPACITY_CHANGE) {
		if (temp == -EOVERFLOW) {
			temp = 0;
			pr_err("%s: invalid battery capacity\n", __func__);
		}
		previous_value = atomic_read(&rt_info.latest_batt_capacity);
		atomic_set(&rt_info.latest_batt_capacity, (int)temp);
		typc_core_handle_battery_capacity_change(previous_value,
							 (int)temp);
	} else if (action == POWER_TEMPERATURE_CHANGE) {
		previous_value = atomic_read(&rt_info.latest_batt_temperature);
		atomic_set(&rt_info.latest_batt_temperature, (int)temp);
		typc_core_handle_battery_temperature_change(previous_value,
							    (int)temp);
	}
	return 0;
}
