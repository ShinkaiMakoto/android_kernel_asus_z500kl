/*
 * typec-core.c - Type-C connector Framework
 *
 * Copyright (C) 2015 HUAWEI, Inc.
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Author: HUAWEI, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/usb/typec.h>
#include <linux/power_supply.h>

typedef union {
    u32 object;
    struct {
        unsigned :30;
        unsigned SupplyType:2;
    } PDO;          // General purpose Power Data Object
    struct {
        unsigned MaxCurrent:10;         // Max current in 10mA units
        unsigned Voltage:10;            // Voltage in 50mV units
        unsigned PeakCurrent:2;         // Peak current (divergent from Ioc ratings)
        unsigned Reserved:3;                    // Reserved
        unsigned DataRoleSwap:1;        // Data role swap (supports DR_Swap message)
        unsigned USBCommCapable:1;      // USB communications capable (is communication available)
        unsigned ExternallyPowered:1;   // Externally powered (set when AC supply is supporting 100% of power)
        unsigned USBSuspendSupport:1;   // USB Suspend Supported
        unsigned DualRolePower:1;       // Dual-Role power (supports PR_Swap message)
        unsigned SupplyType:2;          // (Fixed Supply)
    } FPDOSupply;   // Fixed Power Data Object for Supplies
    struct {
        unsigned OperationalCurrent:10; // Operational current in 10mA units
        unsigned Voltage:10;            // Voltage in 50mV units
        unsigned Reserved:5;                    // Reserved
        unsigned DataRoleSwap:1;        // Data role swap (supports DR_Swap message)
        unsigned USBCommCapable:1;      // USB communications capable (is communication available)
        unsigned ExternallyPowered:1;   // Externally powered (set when AC supply is supporting 100% of power)
        unsigned HigherCapability:1;    // Whether the sink needs more than vSafe5V to provide full functionality
        unsigned DualRolePower:1;       // Dual-Role power (supports PR_Swap message)
        unsigned SupplyType:2;          // (Fixed Supply)
    } FPDOSink;     // Fixed Power Data Object for Sinks
    struct {
        unsigned MaxCurrent:10;         // Max current in 10mA units
        unsigned MinVoltage:10;         // Minimum Voltage in 50mV units
        unsigned MaxVoltage:10;         // Maximum Voltage in 50mV units
        unsigned SupplyType:2;          // (Variable Supply)
    } VPDO;         // Variable Power Data Object
    struct {
        unsigned MaxPower:10;           // Max power in 250mW units
        unsigned MinVoltage:10;         // Minimum Voltage in 50mV units
        unsigned MaxVoltage:10;         // Maximum Voltage in 50mV units
        unsigned SupplyType:2;          // (Battery Supply)
    } BPDO;         // Battery Power Data Object
    struct {
        unsigned MinMaxCurrent:10;      // Min/Max current in 10mA units
        unsigned OpCurrent:10;          // Operating current in 10mA units
        unsigned Reserved0:4;                    // Reserved - set to zero
        unsigned NoUSBSuspend:1;        // Set when the sink wants to continue the contract during USB suspend (i.e. charging battery)
        unsigned USBCommCapable:1;      // USB communications capable (is communication available)
        unsigned CapabilityMismatch:1;  // Set if the sink cannot satisfy its power requirements from capabilities offered
        unsigned GiveBack:1;            // Whether the sink will respond to the GotoMin message
        unsigned ObjectPosition:3;      // Which object in the source capabilities is being requested
        unsigned Reserved1:1;                    // Reserved - set to zero
    } FVRDO;        // Fixed/Variable Request Data Object
} pdo_object_t;

struct power_delievery_status
{
	enum pd_mode mode;
	u32 pdo;		/* Active PDO for sink or source, 0 indicates an invalid value */
	enum pd_pdo_type pdo_type;
	u32 max_voltage;	/* in uV */
	u32 min_voltage;	/* in uV */
	u32 max_current;	/* in uA */
	bool usb_comm_capable;
	struct completion charger_prerequisite_completion;
};
#define valid_pdo(d) ((d) > 0)

static struct class *typec_class;
static struct device *typec_dev;
static struct power_supply *usb_psy;
static struct power_supply typec_psy;
static struct power_supply pd_psy;
static struct power_delievery_status pd_status;

/* to get the Type-C Current mode */
static ssize_t current_detect_show(struct device *pdev,
				   struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	enum typec_current_mode current_mode = typec_ops->current_detect();
	return snprintf(buf, PAGE_SIZE, "%d\n", current_mode);
}

/* to get the attached state and determine what was attached */
static ssize_t attached_state_show(struct device *pdev,
				   struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	enum typec_attached_state attached_state =
	    typec_ops->attached_state_detect();
	return snprintf(buf, PAGE_SIZE, "%d\n", attached_state);
}

/* to get the current advertisement in DFP or DRP modes */
static ssize_t current_advertise_show(struct device *pdev,
				      struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	enum typec_current_mode current_mode =
	    typec_ops->current_advertise_get();
	return snprintf(buf, PAGE_SIZE, "%d\n", current_mode);
	return 0;
}

/* to set the current advertisement in DFP or DRP modes */
static ssize_t current_advertise_store(struct device *pdev,
				       struct device_attribute *attr,
				       const char *buff, size_t size)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int current_mode;

	if (sscanf(buff, "%d", &current_mode) != 1)
		return -EINVAL;

	if (current_mode >= TYPEC_CURRENT_MODE_UNSPPORTED)
		return -EINVAL;

	if (typec_ops->current_advertise_set((enum typec_current_mode)
					     current_mode))
		return -1;

	return size;
}

/* to get the port mode (UFP, DFP or DRP) */
static ssize_t port_mode_ctrl_show(struct device *pdev,
				   struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	enum typec_port_mode port_mode = typec_ops->port_mode_get();
	return snprintf(buf, PAGE_SIZE, "%d\n", port_mode);
	return 0;
}

/* to set the port mode (UFP, DFP or DRP), the chip will operate according the mode */
static ssize_t port_mode_ctrl_store(struct device *pdev,
				    struct device_attribute *attr,
				    const char *buff, size_t size)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int port_mode;

	if (sscanf(buff, "%d", &port_mode) != 1)
		return -EINVAL;

	if (port_mode > TYPEC_DRP_MODE)
		return -EINVAL;

	if (typec_ops->port_mode_set((enum typec_port_mode)port_mode))
		return -1;

	return size;
}

/* to get all the register value */
static ssize_t dump_regs_show(struct device *pdev,
			      struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	return typec_ops->dump_regs(buf);
}

static ssize_t show_i2c_status(struct device *pdev,
                struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n",
		       (typec_ops->i2c_status && typec_ops->i2c_status()));
}

static ssize_t show_cc_status(struct device *pdev,
				struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int cc = -1;

	if (typec_ops->cc_status)
		cc = typec_ops->cc_status();
	else
		pr_err("%s callback not found\n", __func__);
	return sprintf(buf, "%d\n", cc);
}

#ifdef PD_DEBUG
static ssize_t batt_capacity_show(struct device *pdev,
					  struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int cap = -1;

	if (typec_ops->get_battery_capacity)
		cap = typec_ops->get_battery_capacity();

	return (cap >= 0 && cap <= 100) ? sprintf(buf, "%d\n", cap) : 0;
}

static ssize_t batt_capacity_store(struct device *pdev,
				   struct device_attribute *attr,
				   const char *buff, size_t size)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int cap;

	if (sscanf(buff, "%d", &cap) != 1)
		return -EINVAL;

	if (cap < 0 || cap > 100)
		return -EINVAL;

	if (typec_ops->set_battery_capacity) {
		typec_ops->set_battery_capacity(cap);
		return size;
	}
	return -EPERM;
}

static ssize_t batt_temperature_show(struct device *pdev,
				     struct device_attribute *attr, char *buf)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int cap;

	if (typec_ops->get_battery_temperature) {
		cap = typec_ops->get_battery_temperature();
		return sprintf(buf, "%d\n", cap);
	}
	return -EPERM;
}

static ssize_t batt_temperature_store(struct device *pdev,
				      struct device_attribute *attr,
				      const char *buff, size_t size)
{
	struct typec_device_ops *typec_ops = dev_get_drvdata(pdev);
	int cap;

	if (sscanf(buff, "%d", &cap) != 1)
		return -EINVAL;

	if (typec_ops->set_battery_temperature) {
		typec_ops->set_battery_temperature(cap);
		return size;
	}
	return -EPERM;
}
#endif

static DEVICE_ATTR(current_detect, S_IRUGO, current_detect_show, NULL);
static DEVICE_ATTR(attached_state, S_IRUGO, attached_state_show, NULL);
static DEVICE_ATTR(current_advertise, S_IRUGO | S_IWUSR, current_advertise_show,
		   current_advertise_store);
static DEVICE_ATTR(port_mode_ctrl, S_IRUGO | S_IWUSR, port_mode_ctrl_show,
		   port_mode_ctrl_store);
static DEVICE_ATTR(dump_regs, S_IRUGO, dump_regs_show, NULL);
static DEVICE_ATTR(i2c_status, S_IRUGO, show_i2c_status, NULL);
static DEVICE_ATTR(cc_status, S_IRUGO, show_cc_status, NULL);
#ifdef PD_DEBUG
/* Debug interface for PD policy */
static DEVICE_ATTR(batt_capacity, S_IRUGO | S_IWUSR, batt_capacity_show, batt_capacity_store);
static DEVICE_ATTR(batt_temperature, S_IRUGO | S_IWUSR, batt_temperature_show, batt_temperature_store);
#endif
static int pd_power_get_property_usb(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (!valid_pdo(pd_status.pdo)) {
			val->intval = POWER_SUPPLY_TYPEC_MODE_UNKNOWN;
			break;
		}

		if (pd_status.mode == PD_MODE_SOURCE)
			val->intval = POWER_SUPPLY_TYPEC_MODE_SOURCE;
		else if (pd_status.mode == PD_MODE_SINK)
			val->intval = POWER_SUPPLY_TYPEC_MODE_SINK;
		else
			/* In case: PD_MODE_INACTIVE */
			val->intval = POWER_SUPPLY_TYPEC_MODE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (!valid_pdo(pd_status.pdo)) {
			val->intval = 0;
			break;
		}
		val->intval = pd_status.max_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if (!valid_pdo(pd_status.pdo)) {
			val->intval = 0;
			break;
		}
		val->intval = pd_status.min_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (!valid_pdo(pd_status.pdo)) {
			val->intval = 0;
			break;
		}
		val->intval = pd_status.max_current;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (valid_pdo(pd_status.pdo) &&
		    pd_status.mode != PD_MODE_INACTIVE) {
			val->intval = 1;
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PD_RECEIPT:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pd_power_set_property_usb(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_PD_RECEIPT) {
		complete(&pd_status.charger_prerequisite_completion);
		return 0;
	}
	return -EINVAL;
}

static struct device_attribute *typec_attributes[] = {
	&dev_attr_current_detect,
	&dev_attr_attached_state,
	&dev_attr_current_advertise,
	&dev_attr_port_mode_ctrl,
	&dev_attr_dump_regs,
	&dev_attr_i2c_status,
	&dev_attr_cc_status,
#ifdef PD_DEBUG
	&dev_attr_batt_capacity,
	&dev_attr_batt_temperature,
#endif
	NULL
};

static char *pd_pm_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property pd_pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_ONLINE, /* sink, source: 1, inactive: 0 */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPEC_MODE, /* sink, source or none */
	POWER_SUPPLY_PROP_PD_RECEIPT,
};
static int pd_power_property_is_writeable_usb(struct power_supply *psy,
					      enum power_supply_property psp)
{
	/* Besides POWER_SUPPLY_PROP_PD_RECEIPT, nothing is writable */
	if (psp == POWER_SUPPLY_PROP_PD_RECEIPT)
		return 1;
	else
		return 0;
}

static inline int decode_pdo_sink(struct power_delievery_status *pds, u32 pdo)
{
	return -EINVAL;
}

static inline int decode_pdo_source(struct power_delievery_status *pds, u32 pdo)
{
	int err = 0;
	pdo_object_t t_pdo;

	t_pdo.object = pdo;
	switch (t_pdo.PDO.SupplyType) {
	case 0b00:		/* Fixed type */
		pds->pdo_type = PD_PDO_TYPE_FIXED;
		/* 50mV to uV */
		pds->min_voltage = pds->max_voltage =
			t_pdo.FPDOSupply.Voltage * 50 * 1000;
		/* 10mA to uA */
		pds->max_current = t_pdo.FPDOSupply.MaxCurrent * 10 * 1000;
		pds->usb_comm_capable = t_pdo.FPDOSupply.USBCommCapable;
		break;
	case 0b10:		/* Variable type */
		pds->pdo_type = PD_PDO_TYPE_VARIABLE;
		pds->max_voltage = t_pdo.VPDO.MaxVoltage * 50 * 1000;
		pds->min_voltage = t_pdo.VPDO.MinVoltage * 50 * 1000;
		pds->max_current = t_pdo.VPDO.MaxCurrent * 10 * 1000;
		pds->usb_comm_capable = false;
		break;
	case 0b01:		/* Battery type */
		/*
		pds->pdo_type = PD_PDO_TYPE_BATTERY;
		pds->max_voltage = t_pdo.BPDO.MaxVoltage * 50 * 1000;
		pds->min_voltage = t_pdo.BPDO.MinVoltage * 50 * 1000;
		pds->max_current = ((t_pdo.BPDO.MaxPower*250)/((float)(pds->max_voltage)/1000*1000)) * 1000;
		break;
		*/
		WARN(1, "Battery type is not support\n");
	default:
		err = -EINVAL;
	}
	return err;
}

static int decode_pdo(struct power_delievery_status *pds,
		      u32 pdo, enum pd_mode mode)
{
	int err;

	if (pdo == 0 || mode == PD_MODE_INACTIVE) {
		pds->mode = PD_MODE_INACTIVE;
		pds->pdo = 0;
		return 0;
	}

	if (mode == PD_MODE_SINK)
		err = decode_pdo_source(pds, pdo);
	else if (mode == PD_MODE_SOURCE)
		err = decode_pdo_sink(pds, pdo);
	else
		err = -EINVAL;

	if (!err) {
		pds->pdo = pdo;
		pds->mode = mode;
	} else {
		memset(pds, 0, sizeof(struct power_delievery_status));
		pr_err("%s: unable to decode PDO\n", __func__);
	}

	return err;
}

int typec_pd_on_sink_ready(u32 pdo)
{
	int err;

	if (pd_status.pdo == pdo && pd_status.mode != PD_MODE_SOURCE)
		return 0;	/* pd_status.mode is inactive or in sink mode AND
				 * the new PDO equals to the old one.  */
	err = decode_pdo(&pd_status, pdo, PD_MODE_SINK);
	if (!err)
		power_supply_changed(&pd_psy);
	return err;
}

int typec_pd_on_sink_ready_plain_code(int mv, int ma, bool usb_comm)
{
	pdo_object_t t_pdo;

	t_pdo.object = 0;
	t_pdo.FPDOSupply.MaxCurrent = ma / 10;
	t_pdo.FPDOSupply.Voltage = mv / 50;
	t_pdo.FPDOSupply.USBCommCapable = usb_comm;
	t_pdo.FPDOSupply.SupplyType = PD_PDO_TYPE_FIXED;

	typec_pd_on_sink_ready(t_pdo.object);
	return 0;
}

int typec_pd_on_sink_ready_prerequisite(u32 pdo, u32 timeout)
{
	pdo_object_t t_pdo;

	t_pdo.object = pdo;
	switch (t_pdo.PDO.SupplyType) {
	case 0b00:		/* Fixed type */
		t_pdo.FPDOSupply.MaxCurrent = 0;
		break;
	case 0b10:		/* Variable type */
		t_pdo.VPDO.MaxCurrent = 0;
		break;
	case 0b01:		/* Battery type */
		t_pdo.BPDO.MaxPower = 0;
		break;
	default:
		return 0;
	}
	if (timeout > 0)
		reinit_completion(&pd_status.charger_prerequisite_completion);
	typec_pd_on_sink_ready(t_pdo.object);
	if (timeout > 0 &&
	    !wait_for_completion_timeout(&pd_status.charger_prerequisite_completion,
					 msecs_to_jiffies(timeout))) {
		pr_warn("%s, timeout\n", __func__);
	}
	return 0;
}

static enum power_supply_property typec_properties[] = {
	POWER_SUPPLY_PROP_ONLINE, /* attached as UFP: 1, detatched: 0 */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPEC_MODE, /* DFP, UFP or none */
};

static int typec_get_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *val)
{
	struct typec_device_ops *typec_ops;
	enum typec_attached_state attached_state;
	enum typec_current_mode current_mode;

	if (!typec_dev) {
		pr_err("%s: no typec device registered\n", __func__);
		return -EINVAL;
	}
	typec_ops = dev_get_drvdata(typec_dev);
	if (typec_ops == NULL) {
		pr_err("%s: no typec operations registered\n", __func__);
		return -EINVAL;
	}
	/* TYPEC_NOT_ATTACHED
	 * TYPEC_ATTACHED_AS_UFP
	 * TYPEC_ATTACHED_AS_DFP
	 * TYPEC_ATTACHED_TO_ACCESSORY
	 */
	attached_state = typec_ops->attached_state_detect ?
		typec_ops->attached_state_detect() : TYPEC_NOT_ATTACHED;

	/* TYPEC_CURRENT_MODE_DEFAULT
	 * TYPEC_CURRENT_MODE_MID
	 * TYPEC_CURRENT_MODE_HIGH
	 * TYPEC_CURRENT_MODE_UNSPPORTED
	 */
	current_mode = typec_ops->current_detect ?
		typec_ops->current_detect() : TYPEC_CURRENT_MODE_UNSPPORTED;
	switch (prop) {
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (attached_state == TYPEC_ATTACHED_AS_UFP)
			val->intval = POWER_SUPPLY_TYPEC_MODE_UFP;
		else if (attached_state == TYPEC_ATTACHED_AS_DFP)
			val->intval = POWER_SUPPLY_TYPEC_MODE_DFP;
		else
			val->intval =  POWER_SUPPLY_TYPEC_MODE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if (attached_state != TYPEC_ATTACHED_AS_UFP) {
			val->intval = 0;
			break;
		}
		val->intval = 5000*1000; /* 5V */
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (attached_state != TYPEC_ATTACHED_AS_UFP) {
			val->intval = 0;
			break;
		}
		if (current_mode == TYPEC_CURRENT_MODE_DEFAULT)
			val->intval = 500*1000; /* 500mA */
		else if (current_mode == TYPEC_CURRENT_MODE_MID)
			val->intval = 1500*1000; /* 1500mA */
		else if (current_mode == TYPEC_CURRENT_MODE_HIGH)
			val->intval = 3000*1000; /* 3000mA */
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (attached_state == TYPEC_ATTACHED_AS_UFP) {
			val->intval = 1;
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int add_typec_device(struct device *parent, struct typec_device_ops *typec_ops)
{
	struct device *dev;
	struct device_attribute **attrs = typec_attributes;
	struct device_attribute *attr;
	enum power_supply_type pd_psy_type;
	int err;

	if (!typec_ops || !typec_ops->current_detect
	    || !typec_ops->attached_state_detect
	    || !typec_ops->current_advertise_get
	    || !typec_ops->current_advertise_set || !typec_ops->port_mode_get
	    || !typec_ops->port_mode_set || !typec_ops->dump_regs) {
		pr_err("%s: ops is NULL\n", __func__);
		return -1;
	}

	dev = device_create(typec_class, NULL, MKDEV(0, 0), typec_ops,
			    "typec_device");
	if (IS_ERR(dev)) {
		pr_err("%s: device_create fail\n", __func__);
		return -1;
	}

	while ((attr = *attrs++)) {
		err = device_create_file(dev, attr);
		if (err) {
			pr_err("%s: device_create_file fail\n", __func__);
			device_destroy(typec_class, dev->devt);
			return -1;
		}
	}

	typec_dev = dev;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("%s USB supply not found\n", __func__);
	}

	/* Always has TypeC power supply */
	typec_psy.name		  = "typec";
	typec_psy.type		  = POWER_SUPPLY_TYPE_USB_TYPE_C;
	typec_psy.get_property	  = typec_get_property;
	typec_psy.properties	  = typec_properties;
	typec_psy.num_properties  = ARRAY_SIZE(typec_properties);
	typec_psy.supplied_to	  = pd_pm_power_supplied_to;
	typec_psy.num_supplicants = ARRAY_SIZE(pd_pm_power_supplied_to);

	err = power_supply_register(dev, &typec_psy);
	if (err < 0) {
		pr_err("Unable to register typec_psy err=%d\n", err);
		return err;
	}
	/* PD power supply is optional, need to ask controller side. */
	pd_psy_type = typec_ops->pd_power_supply_type ?
		typec_ops->pd_power_supply_type() : POWER_SUPPLY_TYPE_UNKNOWN;

	if (pd_psy_type == POWER_SUPPLY_TYPE_USB_PD ||
	    pd_psy_type == POWER_SUPPLY_TYPE_USB_PD_DRP) {
		pd_psy.name = "power_delivery";
		pd_psy.type = pd_psy_type;
		pd_psy.supplied_to = pd_pm_power_supplied_to;
		pd_psy.num_supplicants = ARRAY_SIZE(pd_pm_power_supplied_to);
		pd_psy.properties = pd_pm_power_props_usb;
		pd_psy.num_properties = ARRAY_SIZE(pd_pm_power_props_usb);
		pd_psy.get_property = pd_power_get_property_usb;
		pd_psy.set_property = pd_power_set_property_usb;
		pd_psy.property_is_writeable
			= pd_power_property_is_writeable_usb;

		err = power_supply_register(dev, &pd_psy);
		if (err < 0) {
			dev_err(dev, "%s:power_supply_register pd failed\n",
				__func__);
			return err;
		}
		pr_info("%s: PD DRP %ssupport\n", __func__,
			pd_psy_type == POWER_SUPPLY_TYPE_USB_PD ? "not " : "");
	}
	return 0;
}

int typec_sink_detected_handler(enum typec_event typec_event)
{
#if defined(CONFIG_TYPEC_ID_HW_CTRL)
	return 0;
#else
	if (!usb_psy) {
		pr_err("%s USB supply not found\n", __func__);
		return -1;
	}

	power_supply_set_usb_otg(usb_psy,
				 (typec_event == TYPEC_SINK_DETECTED) ? 1 : 0);
	return 0;
#endif
}

/**
 * @param
 *  'usb_comm_capable' will be set to 'true' if peer is PD cabable and
 *  'USBCommCapable' is specified.
 * @return
 *  'true' if peer is PD capable device.
 */
bool typec_pd_online(bool *usb_comm_capable)
{
	bool ret = (valid_pdo(pd_status.pdo) &&
		    pd_status.mode != PD_MODE_INACTIVE);

	if (usb_comm_capable)
		*usb_comm_capable = ret && pd_status.usb_comm_capable;

	return ret;
}
/**
 * This is blocking call and might sleep. Don't invoke in interrupt context.
 *
 * @return 'true' if all of the following conditions are true.
 * 1. TypeC is playing as UFP attached. After this cindition examed, 'false'
 *    is returned immediately if the peer is impossible a PD capable device.
 * 2. PD is active and play as sink role.
 * 3. Source capability has been received.
 */
bool typec_wait_pd_sink_completion(void)
{
	struct typec_device_ops *typec_ops;
	enum typec_attached_state cstate;

	if (!typec_dev) {
		pr_err("%s: no typec device registered\n", __func__);
		return false;
	}

	typec_ops = dev_get_drvdata(typec_dev);
	if (!typec_ops->attached_state_detect) {
		WARN(1, "attached_state_detect is un-implement\n");
		return false;
	}
	cstate = typec_ops->attached_state_detect();
	if (cstate != TYPEC_ATTACHED_AS_UFP)
		return false;

	return typec_ops->pd_wait_for_source_caps ?
		typec_ops->pd_wait_for_source_caps() : false;
}

static int __init typec_init(void)
{
	typec_class = class_create(THIS_MODULE, "typec");
	if (IS_ERR(typec_class)) {
		pr_err("failed to create typec class --> %ld\n",
		       PTR_ERR(typec_class));
		return PTR_ERR(typec_class);
	}
	init_completion(&pd_status.charger_prerequisite_completion);
	return 0;
}

subsys_initcall(typec_init);

static void __exit typec_exit(void)
{
	class_destroy(typec_class);
}

module_exit(typec_exit);

MODULE_AUTHOR("HUAWEI");
MODULE_DESCRIPTION("Type-C connector Framework");
MODULE_LICENSE("GPL v2");
