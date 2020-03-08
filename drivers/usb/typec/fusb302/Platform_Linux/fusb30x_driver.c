/* 
 * File:   fusb30x_driver.c
 * Author: Tim Bremm <tim.bremm@fairchildsemi.com>
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */
/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality
#include <linux/usb/typec.h>
#include <linux/usb/class-dual-role.h>
#include <linux/power/notify.h>
#include <linux/atomic.h>
#include <linux/delay.h>

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc

#ifdef FSC_DEBUG
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"
static enum typec_attached_state fusb302_attatched_state_detect(void);

const char* bus_clkname[FUSB_NUM_BUS_CLOCKS] = {"bimc_clk", "snoc_clk",
						"pcnoc_clk"};
/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
    pr_debug("FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
    pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

 /* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
static int fusb302_dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
					    enum dual_role_property prop,
					    unsigned int *val)
{
	struct fusb30x_chip* chip = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state;

	if (!chip)
		return -EINVAL;

	attached_state = fusb302_attatched_state_detect();

	/* TODO.
	 * Should we need to have modification if we are full PD capability? */
	if (attached_state == TYPEC_ATTACHED_AS_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

/* 1. Check to see if current attached_state is same as requested state
 * if yes, then, return.
 * 2. Disonect current session
 * 3. Set approrpriate mode (dfp or ufp)
 * 4. wait for 1.5 secs to see if we get into the corresponding target state
 * if yes, return
 * 5. if not, fallback to Try.SNK
 * 6. wait for 1.5 secs to see if we get into one of the attached states
 * 7. return -EIO
 * Also we have to fallback to Try.SNK state machine on cable disconnect
 *
 * Callback for "echo <value> > /sys/class/dual_role_usb/otg_default/<property>"
 *    'prop' may be one of 'mode', 'data_role', 'power_role'
 *          DUAL_ROLE_PROP_MODE: 'val' could be
 *               0: DUAL_ROLE_PROP_MODE_UFP
 *               1: DUAL_ROLE_PROP_MODE_DFP
 *               2: DUAL_ROLE_PROP_MODE_NONE
 *               3: DUAL_ROLE_PROP_MODE_TOTAL (invalid)
 *
 *          DUAL_ROLE_PROP_PR: 'val' could be
 *               0: DUAL_ROLE_PROP_PR_SRC
 *               1: DUAL_ROLE_PROP_PR_SNK
 *               2: DUAL_ROLE_PROP_PR_NONE
 *               3: DUAL_ROLE_PROP_PR_TOTAL (invalid)
 *
 *          DUAL_ROLE_PROP_DR: 'val' could be
 *               0: DUAL_ROLE_PROP_DR_HOST
 *               1: DUAL_ROLE_PROP_DR_DEVICE
 *               2: DUAL_ROLE_PROP_DR_NONE
 *               3: DUAL_ROLE_PROP_DR_TOTAL (invalid)
 */
#define DUAL_ROLE_SET_MODE_WAIT_MS 3000
static int fusb302_dual_role_set_mode_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct fusb30x_chip* chip = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state = TYPEC_NOT_ATTACHED;
	int timeout = 0;
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (prop != DUAL_ROLE_PROP_MODE) {
		pr_err("unsupport prop setter - prop:%d with value:%d\n", prop, *val);
		return -EINVAL;
	}
	if (*val != DUAL_ROLE_PROP_MODE_DFP && *val != DUAL_ROLE_PROP_MODE_UFP)
		return -EINVAL;

	attached_state = fusb302_attatched_state_detect();

	if (attached_state != TYPEC_ATTACHED_AS_DFP
	    && attached_state != TYPEC_ATTACHED_AS_UFP)
		return 0;	/* NOP if attached state is unattached or accessory */

	if (attached_state == TYPEC_ATTACHED_AS_DFP
	    && *val == DUAL_ROLE_PROP_MODE_DFP)
		return 0;	/* ignore due to current attached_state is same as requested state */

	if (attached_state == TYPEC_ATTACHED_AS_UFP
	    && *val == DUAL_ROLE_PROP_MODE_UFP)
		return 0;	/* ignore due to current attached_state is same as requested state */

	pr_info("%s: start\n", __func__);

	/* AS DFP now, try reversing, form Source to Sink */
	if (attached_state == TYPEC_ATTACHED_AS_DFP) {

		pr_err("%s: try reversing, form Source to Sink\n", __func__);
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		fusb_vbus_off(true);
		fusb_disabled_state_enter();
		chip->reverse_state = REVERSE_ATTEMPT;
		fusb_enable(true, FUSB_EN_F_ALL, TYPEC_UFP_MODE);
	}
	/* AS UFP now, try reversing, form Sink to Source */
	else if (attached_state == TYPEC_ATTACHED_AS_UFP) {

		pr_err("%s: try reversing, form Sink to Source\n", __func__);
		fusb_notify_pd_sink_reset();
		fusb_vbus_off(true);
		fusb_vbus_off(false);
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		fusb_disabled_state_enter();
		chip->reverse_state = REVERSE_ATTEMPT;
		fusb_enable(true, FUSB_EN_F_ALL, TYPEC_DFP_MODE);
	} else {
		pr_err("%s: attached state is not ether ATTACHED_AS_DFP or ATTACHED_AS_UFP, but got %d\n",
		       __func__, attached_state);
	}

	INIT_COMPLETION(chip->reverse_completion);
	timeout =
	    wait_for_completion_timeout(&chip->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));
	if (!timeout) {
		/* If falling back to here, disable everything and reinitialie them within DRP */
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		pr_err("%s: reverse failed, set mode to DRP\n", __func__);
		chip->reverse_state = 0;
		fusb_reinitialize(TYPEC_DRP_MODE); /* to DRP mode */
		INIT_COMPLETION(chip->reverse_completion);
		wait_for_completion_timeout(&chip->reverse_completion,
					    msecs_to_jiffies
					    (DUAL_ROLE_SET_MODE_WAIT_MS));

		ret = -EIO;
	} else {
		/* reverse complete, set DRP mode */
		if (chip->reverse_state == REVERSE_DEBOUNCE) {
			msleep(1000);
			chip->reverse_state = REVERSE_COMPLETE;
		}
		fusb_revert_to_drp_mode();
	}

	pr_err("%s: end ret = %d\n", __func__, ret);

	return ret;
}

/* Callback for "echo <value> >
 *                      /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switch to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to Try.SNK state machine.
 */
static int fusb302_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
				      enum dual_role_property prop,
				      const unsigned int *val)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return fusb302_dual_role_set_mode_prop(dual_role, prop, val);
	else
		/* TODO.
		 * Implement PR_swap and DR_swap here. */
		return -EINVAL;
}

static enum dual_role_property fusb302_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

/* Decides whether userspace can change a specific property */
static int fusb302_dual_role_is_writeable(struct dual_role_phy_instance *drp,
					  enum dual_role_property prop)
{
	/* TODO.
	 * MODE switch only. Because DP is under developing */
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

/**
 * Invoked by typec-core/typec_current_mode_detect for charger to detect
 * the termination type when we are SINK_ATTACHED.
 * @return The detected current mode
 *   TYPEC_CURRENT_MODE_UNSPPORTED: When the attached mode is not sink attached
 *   TYPEC_CURRENT_MODE_DEFAULT: 500mA
 *   TYPEC_CURRENT_MODE_MID: 1.5A
 *   TYPEC_CURRENT_MODE_HIGH: 3A
 */
static enum typec_current_mode fusb302_current_mode_detect(void)
{
	enum typec_current_mode cmode;
	struct connecting_state_descriptor desc;

	fusb_get_connecting_state(&desc);
	pr_debug("%s: conn_state=%d, cc1=%d, cc2=%d, sink term type=%d\n",
		 __func__, desc.conn_state, desc.cc1, desc.cc2,
		 desc.termination_type);

	if (desc.conn_state != AttachedSink) {
		return TYPEC_CURRENT_MODE_UNSPPORTED;
	}
	switch (desc.termination_type) {
	case CCTypeRd1p5:
		cmode = TYPEC_CURRENT_MODE_MID;
		break;
	case CCTypeRd3p0:
		cmode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		cmode = TYPEC_CURRENT_MODE_DEFAULT;

	}
	return cmode;
}

static enum typec_attached_state fusb302_attatched_state_detect(void)
{
	return fusb_get_connecting_state(NULL);
}

static enum typec_current_mode fusb302_current_advertise_get(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_CURRENT_MODE_DEFAULT;
}

static int fusb302_current_advertise_set(enum typec_current_mode current_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat /sys/class/typec/typec_device/port_mode_ctrl'
 * Besides BSP/ATD tools, no one will access it. */
static enum typec_port_mode fusb302_port_mode_get(void)
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
static int fusb302_port_mode_set(enum typec_port_mode port_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat > /sys/class/typec/typec_device/dump_regs'
 * Besides BSP/ATD tools, no one will access it.
 * */
static ssize_t fusb302_dump_regs(char *buf)
{
	u8 regs[] = {
		regDeviceID,
		regSwitches0,
		regSwitches1,
		regMeasure,
		regSlice,
		regControl0,
		regControl1,
		regControl2,
		regControl3,
		regMask,
		regPower,
		regReset,
		regOCPreg,
		regMaska,
		regMaskb,
		regControl4,
		regStatus0a,
		regStatus1a,
		regInterrupt,
		regInterrupt,
		regStatus0,
		regStatus1,
		regInterrupt,
		regFIFO,
	};
	int i;
	u8 tmp;
	bool retv;
	ssize_t numChars = 0;

	numChars += sprintf(buf, ": =========== REGISTER DUMP (FUSB302) ===========\n");
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		retv = fusb_I2C_ReadData(regs[i], &tmp);
		if (retv) {
			numChars += sprintf(buf + numChars, ": Sys addr: 0x%02x = 0x%02x\n", regs[i], tmp);
		} else {
			numChars += sprintf(buf + numChars, ": Sys addr: 0x%02x = n/a\n", regs[i]);
		}
	}
	return numChars;
}
/* call from 'cat > /sys/class/typec/typec_device/i2c_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static bool fusb302_i2c_status(void)
{
	u8 tmp;
	return fusb_I2C_ReadData(regDeviceID, &tmp);
}

static bool fusb302_pd_wait_for_source_caps(void)
{
	return fusb_pd_wait_for_source_caps();
}

/* call from 'cat > /sys/class/typec/typec_device/cc_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
static int fusb302_cc_status(void)
{
	enum typec_attached_state attached_state = TYPEC_NOT_ATTACHED;
	struct connecting_state_descriptor desc;
	int cc = -1;

	attached_state = fusb_get_connecting_state(&desc);

	pr_debug("%s: attached_state=%d, cc1=%d, cc2=%d, sink term type=%d\n",
		 __func__, attached_state, desc.cc1, desc.cc2,
		 desc.termination_type);
	if (attached_state == TYPEC_ATTACHED_AS_DFP ||
	    attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (desc.cc1 ^ desc.cc2)
			cc = desc.cc1 ? 1 : 2;
		else
			cc = 0;
	}

	return cc;
}

static enum power_supply_type fusb302_pd_power_supply_type(void)
{
	return fusb_pd_is_pr_swapable() ? POWER_SUPPLY_TYPE_USB_PD_DRP :
		POWER_SUPPLY_TYPE_USB_PD;
}

static struct power_supply *usb_psy;
static enum typec_vbus_state fusb302_get_vbus_state(void)
{
	union power_supply_propval val = {0, };

	if (fusb_GPIO_Get_VBus5v()) {
		return TYPEC_VBUS_SRC_5V;
	}

	if (!usb_psy) {
		usb_psy = power_supply_get_by_name("usb");
		if (IS_ERR_OR_NULL(usb_psy))
			usb_psy = NULL;
	}
	if (usb_psy) {
		usb_psy->get_property(usb_psy,
				      POWER_SUPPLY_PROP_PRESENT, &val);
		if (val.intval)
			return TYPEC_VBUS_SINK_5V;
	}

	return TYPEC_VBUS_NONE;
}
struct typec_device_ops fusb302_typec_ops = {
	.current_detect = fusb302_current_mode_detect,
	.attached_state_detect = fusb302_attatched_state_detect,
	.current_advertise_get = fusb302_current_advertise_get,
	.current_advertise_set = fusb302_current_advertise_set,
	.port_mode_get = fusb302_port_mode_get,
	.port_mode_set = fusb302_port_mode_set,
	.dump_regs = fusb302_dump_regs,
	.i2c_status = fusb302_i2c_status,
	.cc_status = fusb302_cc_status,
	.pd_wait_for_source_caps = fusb302_pd_wait_for_source_caps,
	.pd_power_supply_type = fusb302_pd_power_supply_type,
	.get_vbus_state = fusb302_get_vbus_state,
};

void register_typec_device(struct fusb30x_chip* chip)
{
	int ret = 0;
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;

	chip->dev = &chip->client->dev;

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc = devm_kzalloc(chip->dev, sizeof(struct dual_role_phy_desc),
				    GFP_KERNEL);
		if (!desc) {
			pr_err("unable to allocate dual role descriptor\n");
			return;
		}

		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = fusb302_dual_role_get_local_prop;
		desc->set_property = fusb302_dual_role_set_prop;
		desc->properties = fusb302_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb302_drp_properties);
		desc->property_is_writeable = fusb302_dual_role_is_writeable;
		dual_role =
		    devm_dual_role_instance_register(chip->dev, desc);
		dual_role->drv_data = chip;
		chip->dual_role = dual_role;
		chip->desc = desc;
	}

	ret = add_typec_device(chip->dev, &fusb302_typec_ops);
	if (ret < 0) {
		pr_err("%s: add_typec_device fail\n", __func__);
	}
}

static int fusb30x_power_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct fusb30x_chip* chip = container_of(self, struct fusb30x_chip, power_nb);
	uintptr_t capacity;
	external_event evt;

	if (action == POWER_CAPACITY_CHANGE) {
		capacity = (uintptr_t)priv;
		if (capacity == -EOVERFLOW) {
			capacity = 0;
			pr_err("%s: invalid battery capacity\n", __func__);
		}
		atomic_set(&chip->latest_batt_capacity, (int)capacity);
		evt = EXTEVT_BATT_CAPACITY_CHANGE;
	}
	fusb_handle_external_event(evt);
	return 0;
}
static void fusb30x_do_pd_handle_batt_capacity_change_work(struct work_struct *w)
{
	struct fusb30x_chip* chip = container_of(w, struct fusb30x_chip,
						 pd_batt_cap_change_work.work);
	int timeout = 0;

	pr_debug("%s: entered\n", __func__);
	fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
		    TYPEC_MODE_ACCORDING_TO_PROT);
	fusb_vbus_off(true);
	fusb_disabled_state_enter();
	chip->reverse_state = REVERSE_ATTEMPT;
	fusb_enable(true, FUSB_EN_F_ALL, TYPEC_DRP_MODE);

	INIT_COMPLETION(chip->reverse_completion);
	timeout =
	    wait_for_completion_timeout(&chip->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));
	if (!timeout) {
		pr_err("%s: unable to reconnect peer\n", __func__);
	}
}

static int fusb30x_bus_freq_init(struct fusb30x_chip* chip, struct device *dev)
{
	int i;
	unsigned long clk_rate;

	for (i = 0; i < FUSB_NUM_BUS_CLOCKS; i++) {
		chip->bus_clks[i] = clk_get(dev, bus_clkname[i]);
		if (chip->bus_clks[i] == NULL) {
			pr_err("%s get failed\n", bus_clkname[i]);
			goto put_clks;
		}
		/* Do we need to wait until clk rate has been set rate by USB? */
		clk_rate = clk_get_rate(chip->bus_clks[i]);
		if (clk_rate == 0) {
			pr_err("%s unable to get clk rate\n", bus_clkname[i]);
			i++;
			goto put_clks;
		}
		pr_debug("%s get at %lu Hz\n", bus_clkname[i], clk_rate);
	}
	chip->bus_clks_enabled = false;
	return 0;
put_clks:
	if (i > 0) {
		while(--i >= 0) {
			clk_put(chip->bus_clks[i]);
		}
	}
	return -1;
}

static int fusb30x_probe (struct i2c_client* client,
                          const struct i2c_device_id* id)
{
    int ret = 0;
    struct fusb30x_chip* chip; 
    struct i2c_adapter* adapter;

    if (!client)
    {
        pr_err("FUSB  %s - Error: Client structure is NULL!\n", __func__);
        return -EINVAL;
    }
    dev_info(&client->dev, "%s\n", __func__);

    /* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb30x_dt_match, &client->dev))
	{
		dev_err(&client->dev, "FUSB  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}
    pr_debug("FUSB  %s - Device tree matched!\n", __func__);

    /* Allocate space for our chip structure (devm_* is managed by the device) */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to allocate memory for g_chip!\n", __func__);
		return -ENOMEM;
	}
    if (fusb30x_bus_freq_init(chip, &client->dev)) {
	    devm_kfree(&client->dev, chip);
	    return -EPROBE_DEFER;
    }
    chip->client = client;                                                      // Assign our client handle to our chip
    fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory
    pr_debug("FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

    /* Initialize the chip lock */
    mutex_init(&chip->lock);

    /* Initialize the chip's data members */
    fusb_InitChipData();
    pr_debug("FUSB  %s - Chip struct data initialized!\n", __func__);

    /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
    adapter = to_i2c_adapter(client->dev.parent);
    if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
    {
        chip->use_i2c_blocks = true;
    }
    else
    {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
        dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
        if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
        {
            dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
            dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
            return -EIO;
        }
    }
    pr_debug("FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

    /* Assign our struct as the client's driverdata */
    i2c_set_clientdata(client, chip);
    pr_debug("FUSB  %s - I2C client data set!\n", __func__);

    init_completion(&chip->reverse_completion);

    /* Verify that our device exists and that it's what we expect */
    if (!fusb_IsDeviceValid())
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
        return -EIO;
    }
    pr_debug("FUSB  %s - Device check passed!\n", __func__);

    /* Initialize the platform's GPIO pins and IRQ */
    ret = fusb_InitializeGPIO();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
        return ret;
    }
    pr_debug("FUSB  %s - GPIO initialized!\n", __func__);

    /* Initialize our timer */
    fusb_InitializeTimer();
    pr_debug("FUSB  %s - Timers initialized!\n", __func__);

#ifdef FSC_DEBUG
    /* Initialize debug sysfs file accessors */
    fusb_Sysfs_Init();
    pr_debug("FUSB  %s - Sysfs device file created!\n", __func__);
#endif // FSC_DEBUG
    INIT_DELAYED_WORK(&chip->pd_batt_cap_change_work, fusb30x_do_pd_handle_batt_capacity_change_work);
    chip->power_nb.notifier_call = fusb30x_power_notify;
    atomic_set(&chip->latest_batt_capacity, BATT_SHUTDOWN_TH);
    power_register_notify(&chip->power_nb);

#ifdef FSC_INTERRUPT_TRIGGERED
    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
	power_unregister_notify(&chip->power_nb);
        return -EIO;
    }
    /* Disable IRQ because of we want to exam it by delayed work */
    fusb_enable(false, FUSB_EN_F_HOST_IRQ, TYPEC_MODE_ACCORDING_TO_PROT);

    chip->pm_qos_latency = 2;	/* hardcoded, comes from msm8956.dtsi/qcom,pm-qos-latency#0 */
    chip->pm_qos_req_dma.type = PM_QOS_REQ_AFFINE_IRQ;
    chip->pm_qos_req_dma.irq = chip->gpio_IntN_irq;
    pm_qos_add_request(&chip->pm_qos_req_dma,
		       PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    pr_debug("FUSB  %s - Core is initialized!\n", __func__);

#else
    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now) */
    fusb_InitializeCore();
    pr_debug("FUSB  %s - Core is initialized!\n", __func__);

    /* Init our workers, but don't start them yet */
    fusb_InitializeWorkers();
    /* Start worker threads after successful initialization */
    fusb_ScheduleWork();
    pr_debug("FUSB  %s - Workers initialized and scheduled!\n", __func__);
#endif  // ifdef FSC_POLLING elif FSC_INTERRUPT_TRIGGERED
    register_typec_device(chip);
    dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);

#ifdef FSC_INTERRUPT_TRIGGERED
    if (platform_get_device_irq_state()) {
	    fusb_InitializeWorkers();
	    fusb_ScheduleWork();
	    pr_info("FUSB  %s - INT is active, workers initialized and scheduled!\n", __func__);
    } else {
	    /* Nothing to do, enable IRQ  */
	    fusb_enable(true, FUSB_EN_F_HOST_IRQ, TYPEC_MODE_ACCORDING_TO_PROT);
    }
#endif

	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip) {
		power_unregister_notify(&chip->power_nb);
	}
	pm_qos_remove_request(&chip->pm_qos_req_dma);
    pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);
#ifndef FSC_INTERRUPT_TRIGGERED // Polling mode by default
    fusb_StopThreads();
#endif  // !FSC_INTERRUPT_TRIGGERED

    fusb_StopTimers();
    fusb_GPIO_Cleanup();
    pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
    return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
	FSC_U8 reset_value = 0x03;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip) {
		power_unregister_notify(&chip->power_nb);
	}
	pm_qos_remove_request(&chip->pm_qos_req_dma);
	fusb_I2C_WriteData(regReset, 1, &reset_value);
}

/*******************************************************************************
 * Driver macros
 ******************************************************************************/
module_init(fusb30x_init);                                                      // Defines the module's entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Tim Bremm<tim.bremm@fairchildsemi.com>");                        // Exposed on call to modinfo
