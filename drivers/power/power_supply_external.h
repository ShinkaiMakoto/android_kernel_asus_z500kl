/*
 * Copyright (c) 2015, ASUSTek, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */
 
struct device;
struct device_type;
struct power_supply;

enum CHG_TYPE {
	CHG_TYPE_UNDEFINED = 0,
	CHG_TYPE_USB,
	CHG_TYPE_CDP,
	CHG_TYPE_DCP,
	CHG_TYPE_DCP_QC_2,
	CHG_TYPE_DCP_QC_3,
	CHG_TYPE_DCP_NOT_QC,
	CHG_TYPE_DCP_TYPC_3,
	CHG_TYPE_OTHER,
};

#define STOP_CHARGING_ENABLE	1
#define	STOP_CHARGING_DISABLE	0

#if defined(CONFIG_ZT500KL)
#define SUPPORT_9V_HVDCP
#define SUPPORT_PARALLEL_CHG
#define SUPPORT_PD_ADAPTER
extern int smb1351_read_register(int reg, u8 val);
extern int smb1351_write_register(int reg, u8 val);
extern int smb1351_masked_write_register(int reg, u8 mask, u8 val);
#endif

#ifdef ENG_BUILD
#define SUPPORT_FORCE_STOP_CHARGING		1
#define CHARGING_LIMIT_THRESHOLD        60
#define CHARGING_LIMIT_THRESHOLD_FILE   "/factory/charging_limit_threshold"
extern int eng_charging_limit(bool b_stop_charging);
#else // NOT ENG_BUILD
#define SUPPORT_DEMO_MODE_CHG_CTL 1
#if SUPPORT_DEMO_MODE_CHG_CTL
#define SUPPORT_FORCE_STOP_CHARGING	1
#define DEMO_MODE_CTL_MAX_CAPACITY	35
#define DEMO_MODE_CTL_MIN_CAPACITY	30
#define DEMO_MODE_STATUS_FILE_PTAH	"/data/data/vzw_demo_mode"
#define DEMO_MODE_ENABLE			1
#define DEMO_MODE_DISABLE			0
extern int vzw_demo_mode_control(bool b_stop_charging);
#else
#define SUPPORT_FORCE_STOP_CHARGING	0
#endif
extern int jeita_control(int i_bat_capacity, int i_bat_temp, int i_bat_volt);
#endif // NOT ENG_BUILD

extern int fg_read_mem_data(int index);
