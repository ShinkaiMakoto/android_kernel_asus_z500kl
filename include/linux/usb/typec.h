/*
 * typec.h - Type-C connector Framework
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

#ifndef _TYPEC_H_
#define _TYPEC_H_

#include <linux/types.h>
#include <linux/device.h>
#include <linux/power_supply.h>

enum pd_mode {
	PD_MODE_INACTIVE,
	PD_MODE_SOURCE,
	PD_MODE_SINK,
};

enum pd_pdo_type {
	PD_PDO_TYPE_FIXED,
	PD_PDO_TYPE_VARIABLE,
	PD_PDO_TYPE_BATTERY,
};

enum typec_current_mode {
	TYPEC_CURRENT_MODE_DEFAULT = 0,
	TYPEC_CURRENT_MODE_MID,
	TYPEC_CURRENT_MODE_HIGH,
	TYPEC_CURRENT_MODE_UNSPPORTED
};

enum typec_attached_state {
	TYPEC_NOT_ATTACHED = 0,
	TYPEC_ATTACHED_AS_UFP,
	TYPEC_ATTACHED_AS_DFP,
	TYPEC_ATTACHED_TO_ACCESSORY
};

enum typec_port_mode {
	TYPEC_MODE_ACCORDING_TO_PROT = 0,
	TYPEC_UFP_MODE,
	TYPEC_DFP_MODE,
	TYPEC_DRP_MODE
};

enum typec_event {
	TYPEC_SINK_DETECTED = 0,
	TYPEC_SINK_REMOVED
};

enum typec_vbus_state {
       TYPEC_VBUS_NONE = 0,
       TYPEC_VBUS_SRC_0V,      /* We are sourcing VBUS, but there is no voltage
                                * detected on VBUS. */
       TYPEC_VBUS_SRC_5V,      /* We are sourcing VBUS, 5V is detected at VBUS. */
       TYPEC_VBUS_SINK_0V,     /* We are sinking VBUS, but no valid voltage on VBUS. */
       TYPEC_VBUS_SINK_5V,     /* We are sinking VBUS, 5V is detected on VBUS. */
};

struct typec_device_ops {
	/* to get the Type-C Current mode */
	enum typec_current_mode (*current_detect) (void);

	/* to get the attached state and determine what was attached */
	enum typec_attached_state (*attached_state_detect) (void);

	/* to get the current advertisement in DFP or DRP modes */
	enum typec_current_mode (*current_advertise_get) (void);
	/* to set the current advertisement in DFP or DRP modes */
	int (*current_advertise_set) (enum typec_current_mode current_mode);

	/* to get the port mode (UFP, DFP or DRP) */
	enum typec_port_mode (*port_mode_get) (void);
	/* to set the port mode (UFP, DFP or DRP), the chip will operate according the mode */
	int (*port_mode_set) (enum typec_port_mode port_mode);

	/* to get all the register value */
	ssize_t(*dump_regs) (char *buf);

	/**
	 * to get whether I2C-slave is accessable
	 * @return
	 *   true: I2C communication works well
	 *   false: I2C is dead
	 */
	bool (*i2c_status) (void);

	/**
	 * to get CC orientation,
	 * @return
	 *   1: indicates CC1 attached, no mater what was the attached type.
	 *   2: CC2 attached
	 *   others: invalid value, indicates nothing attached
	 */	
	int (*cc_status) (void);
	bool (*pd_wait_for_source_caps) (void);

	/**
	 * To get the type of USB power delivery
	 * @return
	 *   POWER_SUPPLY_TYPE_USB_PD: If platform support USB power delivery
	 *   POWER_SUPPLY_TYPE_USB_PD_DRP: If platform support USB power delivery and able to swap power and data roles
	 *   POWER_SUPPLY_TYPE_UNKNOWN: If platform is typeC only.
	 */
	enum power_supply_type (*pd_power_supply_type) (void);
	void (*pd_reevaluate_contract)(bool vsafe_5v);
	int (*get_battery_capacity)(void);
	void (*set_battery_capacity)(int);
	int (*get_battery_temperature)(void);
	void (*set_battery_temperature)(int);
	enum typec_vbus_state (*get_vbus_state)(void);
};

extern int add_typec_device(struct device *parent,
			    struct typec_device_ops *typec_ops);
extern int typec_sink_detected_handler(enum typec_event typec_event);
extern int typec_pd_on_sink_ready(u32 pdo_object);
extern int typec_pd_on_sink_ready_plain_code(int mv, int ma, bool usb_comm);
extern bool typec_pd_online(bool *usb_comm_capable);
extern bool typec_wait_pd_sink_completion(void);
extern int typec_pd_on_sink_ready_prerequisite(u32 pdo, u32 timeout);
#endif /* _TYPEC_H_ */
