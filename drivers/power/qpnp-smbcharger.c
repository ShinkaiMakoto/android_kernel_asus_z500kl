/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
#include <linux/proc_fs.h>
#include <linux/HWVersion.h>
#include "pmic-voter.h"
#include "power_supply_external.h"
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/usb/typec.h>
#include <linux/wakelock.h>
/* Mask/Bit helpers */
#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))
/* Config registers */
struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct smbchg_version_tables {
	const int			*dc_ilim_ma_table;
	int				dc_ilim_ma_len;
	const int			*usb_ilim_ma_table;
	int				usb_ilim_ma_len;
	const int			*iterm_ma_table;
	int				iterm_ma_len;
	const int			*fcc_comp_table;
	int				fcc_comp_len;
	const int			*aicl_rerun_period_table;
	int				aicl_rerun_period_len;
	int				rchg_thr_mv;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	int				schg_version;

	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				dc_target_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	int				sw_esr_pulse_current_ma;
	int				aicl_rerun_period_s;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				cfg_chg_led_support;
	bool				cfg_chg_led_sw_ctrl;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				hvdcp3_supported;
	bool				allow_hvdcp3_detection;
	bool				restricted_charging;
	bool				cfg_override_usb_current;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
#if 0 //remove qualcomm parallel charge
	struct delayed_work		parallel_en_work;
#endif
	struct dentry			*debug_root;
	struct smbchg_version_tables	tables;

	/* wipower params */
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	/* vfloat adjustment */
	int				max_vbat_sample;
	int				n_vbat_samples;

	/* status variables */
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	enum power_supply_type		usb_supply_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	bool				chg_otg_enabled;
	bool				flash_triggered;
	bool				icl_disabled;
	u32				wa_flags;
	int				usb_icl_delta;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				wdog_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		*usb_psy;
#ifdef SUPPORT_PD_ADAPTER
	struct power_supply		*pd_psy;
#endif
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		adapter_detect_work;
	struct delayed_work		asus_sdp_delayed_work;
	struct workqueue_struct		*smbchg_work_queue;
	struct workqueue_struct		*smbchg_src_detect_queue;
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	struct delayed_work		pule_down_5v_work;
#endif
	struct delayed_work		cable_poweron_work;
	struct delayed_work		usb_insertion_work;
#if defined(ENG_BUILD)
	struct delayed_work		charger_limit_work;
#endif
#ifdef SUPPORT_9V_HVDCP
	struct delayed_work		hvdcp_det_work;
#endif
	spinlock_t			sec_access_lock;
	struct mutex			therm_lvl_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;
	bool				skip_usb_notification;

	/* voters */
	struct votable			*fcc_votable;
	struct votable			*usb_icl_votable;
	struct votable			*dc_icl_votable;
	struct votable			*usb_suspend_votable;
	struct votable			*dc_suspend_votable;
	struct votable			*battchg_suspend_votable;

	u32				vchg_adc_channel;
	struct qpnp_vadc_chip		*vchg_vadc_dev;
	
	/* oem customization */
	bool stop_chg_via_temp_cold;
	bool stop_chg_via_temp_cool;
	bool stop_chg_via_temp_warm;
	bool stop_chg_via_temp_hot;
	bool dedicated_adapter;
	bool dedicated_power_bank;
	bool cable_power_on;
	int hw_id;
	int proj_id;
	int fg_capacity;
	int fg_voltage;
	enum CHG_TYPE charging_type;

	/* charger LED */
	unsigned int	INDCAT_EN;
	struct completion		sdp_retry_complete_flag;
};

/* AC charging alarm */
atomic_t AC_charging_alarm;
EXPORT_SYMBOL(AC_charging_alarm);
bool boost_up_voltage_done = false;
EXPORT_SYMBOL(boost_up_voltage_done);
bool g_dcp_insert = false;
bool b_full_capacity = false;
bool is_cc_logic = false;
bool b_last_charging=true;
bool g_cable_poweron = false;
struct wake_lock alarm_cable_lock;
static int g_sdp_retry_flag = 0; //WA for slow plug casues wrong charging type detection issue
#ifdef SUPPORT_PARALLEL_CHG
static int dual_chg_enable(int usb_in_current, int fast_chg_current);
#ifndef ENG_BUILD
static int dual_chg_disable(void);
#endif
#endif
static struct smbchg_chip *chip_fg;
static struct workqueue_struct *adapter_wq;

extern int entry_mode;
void demochg_judge(int en);
EXPORT_SYMBOL(demochg_judge);
bool g_demochg_res = 0;
bool g_demochg_flag = 0;
EXPORT_SYMBOL(g_demochg_flag);
#if defined(VZW)
extern int get_capacity_remapping_in(void);
extern int get_capacity_remapping_out(void);
#endif
extern enum typec_current_mode typec_current_mode_detect(void);
extern bool typec_wait_pd_sink_completion(void);
#if defined(CONFIG_ZT582KL)
extern int gt6108_cable_status_handler(int state);
#endif
/***************** charging flags *******************/
enum dual_chg_flags{
	Undefined,
	Single,
	ASUS_2A,
	ASUS_10W,
	ASUS_18W,
	TYPC_3A,
};
static char *dual_chg_flag_str[] = {
	"Undefined",
	"Single",
	"ASUS_2A",
	"ASUS_10W",
	"ASUS_18W",
	"TYPC_3A",
}; 
int dual_chg_flag = Undefined;

enum hvdcp_flags{
	NO_HVDCP,
	HVDCP_2,
	HVDCP_3,
};
static char *hvdcp_flag_str[] ={
	"NO_HVDCP",
	"HVDCP_2",
	"HVDCP_3",
};
int hvdcp_flag = NO_HVDCP;

enum chg_type_flags{
	UNDEFINED,
	DCP_ASUS_750K_2A,
	HVDCP_OTHERS_1A,
	HVDCP_ASUS_200K_2A,
	DCP_PB_2A,
	HVDCP_PB_1A,
	CDP_1P5A,
	OTHERS_1A,
	SDP_0P5A,
	SDP,
	FLOATING,
	OTHERS,
	CDP,
};
static char *chg_type_flag_str[] ={
	"UNDEFINED",
	"DCP_ASUS_750K_2A",
	"HVDCP_OTHERS_1A",
	"HVDCP_ASUS_200K_2A",
	"DCP_PB_2A",
	"HVDCP_PB_1A",
	"CDP_1P5A",
	"OTHERS_1A",
	"SDP_0P5A",
	"SDP",
	"FLOATING",
	"OTHERS",
	"CDP",
};
int chg_type_flag = UNDEFINED;

enum thermal_policy_flags{
	TP_LEVEL0,
	TP_LEVEL1,
	TP_LEVEL2,
	TP_LEVEL3,
};
static char *thermal_policy_flag_str[] ={
	"TP_LEVEL0",
	"TP_LEVEL1",
	"TP_LEVEL2",
	"TP_LEVEL3",
};
int thermal_policy_flag = TP_LEVEL0;

enum dfp_type_flags{
	Others,
	TYPEC_1P5A,
	TYPEC_3A,
	TYPEC_PD,
};
static char *dfp_type_flag_str[] ={
	"Others",
	"TYPEC_1P5A",
	"TYPEC_3A",
	"TYPEC_PD",
};
int dfp_type_flag = Others;

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
	PMI8996		= 19,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
	SMBCHG_USB100_WA = BIT(2),
	SMBCHG_BATT_OV_WA = BIT(3),
	SMBCHG_CC_ESR_WA = BIT(4),
	SMBCHG_FLASH_ICL_DISABLE_WA = BIT(5),
};

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
	PM_OEM_DCP_TYPE_DETECT = BIT(4),
};

enum fcc_voters {
	ESR_PULSE_FCC_VOTER,
	BATT_TYPE_FCC_VOTER,
	RESTRICTED_CHG_FCC_VOTER,
	NUM_FCC_VOTER,
};

enum icl_voters {
	PSY_ICL_VOTER,
	THERMAL_ICL_VOTER,
	HVDCP_ICL_VOTER,
	USER_ICL_VOTER,
	WEAK_CHARGER_ICL_VOTER,
	SW_AICL_ICL_VOTER,
	NUM_ICL_VOTER,
};

enum enable_voters {
	/* userspace has suspended charging altogether */
	USER_EN_VOTER,
	/*
	 * this specific path has been suspended through the power supply
	 * framework
	 */
	POWER_SUPPLY_EN_VOTER,
	/*
	 * the usb driver has suspended this path by setting a current limit
	 * of < 2MA
	 */
	USB_EN_VOTER,
	/*
	 * when a wireless charger comes online,
	 * the dc path is suspended for a second
	 */
	WIRELESS_EN_VOTER,
	/*
	 * the thermal daemon can suspend a charge path when the system
	 * temperature levels rise
	 */
	THERMAL_EN_VOTER,
	/*
	 * an external OTG supply is being used, suspend charge path so the
	 * charger does not accidentally try to charge from the external supply.
	 */
	OTG_EN_VOTER,
	/*
	 * the charger is very weak, do not draw any current from it
	 */
	WEAK_CHARGER_EN_VOTER,
	NUM_EN_VOTERS,
};

enum battchg_enable_voters {
	/* userspace has disabled battery charging */
	BATTCHG_USER_EN_VOTER,
	/* battery charging disabled while loading battery profiles */
	BATTCHG_UNKNOWN_BATTERY_EN_VOTER,
	NUM_BATTCHG_EN_VOTERS,
};

/* FG_MEMIF data index */
enum fg_mem_data_index {
	FG_DATA_BATT_TEMP = 0,
	FG_DATA_OCV,
	FG_DATA_VOLTAGE,
	FG_DATA_CURRENT,
	FG_DATA_BATT_ESR,
	FG_DATA_BATT_ESR_COUNT,
	FG_DATA_BATT_SOC,
	FG_DATA_CC_CHARGE,
	FG_DATA_VINT_ERR,
	FG_DATA_CPRED_VOLTAGE,
	/* values below this only gets read once per profile reload */
	FG_DATA_BATT_ID,
	FG_DATA_BATT_ID_INFO,
	FG_DATA_MAX,
};

/* global variable */
bool g_probe = true;
#if defined(ENG_BUILD)
int charging_limit_threshold = CHARGING_LIMIT_THRESHOLD;
EXPORT_SYMBOL(charging_limit_threshold);
#endif

#define BATT_RE_CHG_LEVEL	98
#define BATT_LOW_LEVEL	1
#define BATT_LOW_VOLT	3250

static int smbchg_debug_mask;
module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int smbchg_parallel_en = 1;
module_param_named(
	parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_fcc_percent = 50;
module_param_named(
	main_chg_fcc_percent, smbchg_main_chg_fcc_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_icl_percent = 60;
module_param_named(
	main_chg_icl_percent, smbchg_main_chg_icl_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_hvdcp_icl_ma = 900;//3000;

module_param_named(
	default_hvdcp_icl_ma, smbchg_default_hvdcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
static int smbchg_default_dcp_icl_ma = 900;
#else
static int smbchg_default_dcp_icl_ma = 1800;
#endif
module_param_named(
	default_dcp_icl_ma, smbchg_default_dcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
	dynamic_icl_wipower_en, wipower_dyn_icl_en,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
	wipower_dcin_interval, wipower_dcin_interval,
	int, S_IRUSR | S_IWUSR
);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
	wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
	int, S_IRUSR | S_IWUSR
);

/*#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)*/
#define pr_smb(reason, fmt, ...) pr_info(fmt, ##__VA_ARGS__)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

/*
 * Writes an arbitrary number of bytes to a specified register
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_write(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	//pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * This function holds a spin lock to ensure secure access register writes goes
 * through. If the secure access unlock register is armed, any old register
 * write can unarm the secure access unlock, causing the next write to fail.
 *
 * Note: do not use this for sec_access registers. Instead use the function
 * below: smbchg_sec_masked_write
 */
static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

/*
 * Unlocks sec access and writes to the register specified.
 *
 * This function holds a spin lock to exclude other register writes while
 * the two writes are taking place.
 */
#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}

#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define AICL_DONE_RT_STS		BIT(5)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;
	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Sleep for 20 ms (minimum msleep time) to wait for the conversion to
	 * finish and the USB RID status register to be updated before trying
	 * to detect OTG insertions.
	 */

	msleep(20);

	/*
	 * There is a problem with USBID conversions on PMI8994 revisions
	 * 2.0.0. As a workaround, check that the cable is not
	 * detected as factory test before enabling OTG.
	 */
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_GND_DET_STS			BIT(2)
static bool is_otg_present_schg_lite(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_GND_DET_STS);
}

static bool is_otg_present(struct smbchg_chip *chip)
{
	if (chip->schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	if(is_cc_logic)
		return true;

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static char *usb_type_str[] = {
	"SDP",		/* bit 0 */
	"OTHER",	/* bit 1 */
	"DCP",		/* bit 2 */
	"CDP",		/* bit 3 */
	"NONE",		/* bit 4 error case */
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

/* helper to return the string of USB type */
static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		/* bit 0 */
	POWER_SUPPLY_TYPE_UNKNOWN,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

/* helper to return enum power_supply_type of USB type */
static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static void read_usb_type(struct smbchg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
	type = get_type(reg);
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

#define CHGR_STS			0x0E
#define CHG_DONE_STS			BIT(5)
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_capacity(struct smbchg_chip *chip);
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;

	charger_present = is_usb_present(chip) | is_dc_present(chip) |
			  chip->hvdcp_3_det_ignore_uv| (g_sdp_retry_flag!=0);
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & BAT_TCC_REACHED_BIT){
		dev_err(chip->dev, "[%s] POWER_SUPPLY_STATUS_FULL, reg & BAT_TCC_REACHED_BIT = 0x%lx\n", __func__, reg & BAT_TCC_REACHED_BIT);
		return POWER_SUPPLY_STATUS_FULL;
	}

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit){
		dev_err(chip->dev, "[%s] POWER_SUPPLY_STATUS_FULL, reg & CHG_INHIBIT_BIT = %d\n", __func__, chg_inhibit);
		return POWER_SUPPLY_STATUS_FULL;
	}

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}else if (g_demochg_res){
		pr_smb(PR_STATUS, "force disable charging\n");
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL && !chip->hvdcp_3_det_ignore_uv)
	{
		pr_smb(PR_STATUS, "not charging\n");
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		if (g_sdp_retry_flag != 0)
			status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
#if defined(ENG_BUILD)
		status = POWER_SUPPLY_STATUS_CHARGING;
#else
		//if ((chip->charging_type == CHG_TYPE_DCP_QC_2) || (chip->charging_type == CHG_TYPE_DCP_QC_3))
		if(chip->charging_type == CHG_TYPE_DCP_QC_3){ // ASUS_750K(5V/2A) does not show QC icon "+"
			status = POWER_SUPPLY_STATUS_QC_V2_CHARGING;
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
			if (chip_fg->fg_capacity > 70)
				status = POWER_SUPPLY_STATUS_NOT_QC_CHARGING;
#endif				
		}
		else if (chip->charging_type == CHG_TYPE_DCP_NOT_QC)
			status = POWER_SUPPLY_STATUS_NOT_QC_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
#endif
	}
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);

	if((chip_fg->fg_capacity >= 99) && ((status == POWER_SUPPLY_STATUS_CHARGING) || (status == POWER_SUPPLY_STATUS_QC_V2_CHARGING))){
		dev_err(chip->dev, "[%s] POWER_SUPPLY_STATUS_FULL %d\n", __func__, chip_fg->fg_capacity);
//		status = POWER_SUPPLY_STATUS_FULL;
	}

	if(charger_present){
		if(get_prop_batt_capacity(chip) == 100)
			return POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	int capacity, rc;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smbchg_chip *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}
	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smbchg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4380000
static int get_prop_batt_voltage_max_design(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

/*
 * finds the index of the closest value in the array. If there are two that
 * are equally close, the lower index will be returned
 */
static int find_closest_in_array(const int *arr, int len, int val)
{
	int i, closest = 0;

	if (len == 0)
		return closest;
	for (i = 0; i < len; i++)
		if (abs(val - arr[i]) < abs(val - arr[closest]))
			closest = i;

	return closest;
}

/* finds the index of the closest smaller value in the array. */
static int find_smaller_in_array(const int *table, int val, int len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}

static const int iterm_ma_table_8994[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	500,
	600
};

static const int iterm_ma_table_8996[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	400,
	500
};

static const int usb_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int usb_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000
};

static int dc_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static int dc_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
};

static const int fcc_comp_table_8994[] = {
	250,
	700,
	900,
	1200,
};

static const int fcc_comp_table_8996[] = {
	250,
	1100,
	1200,
	1500,
};

static const int aicl_rerun_period[] = {
	45,
	90,
	180,
	360,
};

static const int aicl_rerun_period_schg_lite[] = {
	3,	/* 2.8s  */
	6,	/* 5.6s  */
	11,	/* 11.3s */
	23,	/* 22.5s */
	45,
	90,
	180,
	360,
};

static void use_pmi8994_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8994;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8994);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8994;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8994);
	chip->tables.iterm_ma_table = iterm_ma_table_8994;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8994);
	chip->tables.fcc_comp_table = fcc_comp_table_8994;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8994);
	chip->tables.rchg_thr_mv = 200;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

static void use_pmi8996_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8996;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8996);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8996;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8996);
	chip->tables.iterm_ma_table = iterm_ma_table_8996;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8996);
	chip->tables.fcc_comp_table = fcc_comp_table_8996;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8996);
	chip->tables.rchg_thr_mv = 150;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
	int rc;
	/* The en bit is configured active low */
	if(g_demochg_res){
		pr_smb(PR_STATUS, "force disable charging\n");
		en = false;
	}
	pr_info("%s Charging\n", en ? "Enabling" : "Disabling");
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT,en ? 0 : EN_BAT_CHG_BIT);
	return rc;
}

#define CMD_IL			0x40
#define	USB_CMD_IL_MASK		SMB_MASK(7, 0)
#define USB_CMD_IL_CONFIG	0x05
#define USB_CMD_IL_CONFIG_2	0x06
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1200_MA		1200
#define CURRENT_1500_MA		1500
#define CURRENT_2100_MA		2100
#define CURRENT_2300_MA		2300
#define	CURRENT_3000_MA		3000
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			current_ma, chip->tables.dc_ilim_ma_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = chip->tables.dc_ilim_ma_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_RESTART_TIMER		SMB_MASK(2, 0)
#define AICL_RESTART_TIMER_6MIN	0x07
#define AICL_WL_SEL_SCHG_LITE_MASK	SMB_MASK(2, 0)
static int smbchg_set_aicl_rerun_period_s(struct smbchg_chip *chip,
								int period_s)
{
	int i;
	u8 reg, mask;

	i = find_smaller_in_array(chip->tables.aicl_rerun_period_table,
			period_s, chip->tables.aicl_rerun_period_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %ds in aicl rerun period\n",
				period_s);
		return -EINVAL;
	}

	if (chip->schg_version == QPNP_SCHG_LITE)
		mask = AICL_WL_SEL_SCHG_LITE_MASK;
	else
		mask = AICL_WL_SEL_MASK;

	reg = i & mask;

	pr_smb(PR_STATUS, "aicl rerun period set to %ds\n",
			chip->tables.aicl_rerun_period_table[i]);
	return smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			mask, reg);
}

static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				usb_set_online_work);
	bool user_enabled = !get_client_vote(chip->usb_suspend_votable,
						USER_EN_VOTER);
	int online;

	online = user_enabled && chip->usb_present && !chip->very_weak_charger;

	if (g_cable_poweron)
		online = 1;

	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		pr_smb(PR_MISC, "setting usb psy online = %d\n", online);
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)

#ifndef ENG_BUILD
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	if (current_ma == CURRENT_100_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (rc < 0) {
			pr_err("Couldn't set CFG_USB_2 rc=%d\n", rc);
			return rc;
		}

		rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT | USB51_MODE_BIT | ICL_OVERRIDE_BIT,
			USBIN_LIMITED_MODE | USB51_100MA | ICL_OVERRIDE_BIT);
		if (rc < 0) {
			pr_err("Couldn't set ICL_OVERRIDE rc=%d\n", rc);
			return rc;
		}

		pr_smb(PR_STATUS,
			"Forcing 100mA current limit\n");
		chip->usb_max_current_ma = CURRENT_100_MA;
		return rc;
	}

	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = chip->tables.usb_ilim_ma_table[i];
	return rc;
}
#endif
// #define IL_CFG			0xF2  // The register address same as IL_CFG of DCIN, so mask it.
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_IL_500MA		0x4
#define USBIN_IL_700MA		0x8
#define USBIN_IL_900MA		0x9
#define USBIN_IL_1000MA		0xB
#define USBIN_IL_1400MA		0xE
#define USBIN_IL_1500MA		0x10
#define USBIN_IL_1800MA		0x12
#define USBIN_IL_1910MA		0x15
#define USBIN_IL_2000MA		0x19
#define USBIN_IL_3000MA		0x1F

/* if APSD results are used
 *	if SDP is detected it will look at 500mA setting
 *		if set it will draw 500mA
 *		if unset it will draw 100mA
 *	if CDP/DCP it will look at 0x0C setting
 *		i.e. values in 0x41[1, 0] does not matter
 */
static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	/* we use our dual charging settings, so skip this check or settings */
	pr_smb(PR_STATUS, "skip this function\n");
	return 0;
#if 0 //remove qualcomm parallel charge
	int rc = 0;

	/*
	 * if the battery is not present, do not allow the usb ICL to lower in
	 * order to avoid browning out the device during a hotswap.
	 */
	if (!chip->batt_present && current_ma < chip->usb_max_current_ma) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}
	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma <= SUSPEND_CURRENT_MA) {
		/* suspend the usb if current <= 2mA */
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, true, 0);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, false, 0);
	}

	switch (chip->usb_supply_type) {
	case POWER_SUPPLY_TYPE_USB:
		if ((current_ma < CURRENT_150_MA) &&
				(chip->wa_flags & SMBCHG_USB100_WA))
			current_ma = CURRENT_150_MA;

		/* handle special SDP case when USB reports high current */
		if (current_ma > CURRENT_900_MA) {
			if (chip->cfg_override_usb_current) {
				/*
				 * allow setting the current value as reported
				 * by USB driver.
				 */
				rc = smbchg_set_high_usb_chg_current(chip,
							current_ma);
				if (rc < 0) {
					pr_err("Couldn't set %dmA rc = %d\n",
							current_ma, rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
				if (rc < 0)
					pr_err("Couldn't set ICL override rc = %d\n",
							rc);
			} else {
				/* default to 500mA */
				current_ma = CURRENT_500_MA;
			}
			pr_smb(PR_STATUS,
				"override_usb_current=%d current_ma set to %d\n",
				chip->cfg_override_usb_current, current_ma);
		}

		if (current_ma < CURRENT_150_MA) {
			/* force 100mA */
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 100;
		}
		/* specific current values */
		if (current_ma == CURRENT_150_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 150;
		}
		if (current_ma == CURRENT_500_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 500;
		}
		if (current_ma == CURRENT_900_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 900;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (current_ma < CURRENT_1500_MA) {
			/* use override for CDP */
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (rc < 0)
				pr_err("Couldn't set override rc = %d\n", rc);
		}
		/* fall through */
	default:
		rc = smbchg_set_high_usb_chg_current(chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;
	}

out:
	pr_smb(PR_STATUS, "usb type = %d current set to %d mA\n",
			chip->usb_supply_type, chip->usb_max_current_ma);
	return rc;
#endif
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
#if 0 //remove qualcomm parallel charge
static int smbchg_get_min_parallel_current_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->parallel.min_9v_current_thr_ma;
	return chip->parallel.min_current_thr_ma;
}
#endif

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_1200MA_VAL		0xD
#define FCC_2100MA_VAL		0x1B
#define FCC_2300MA_VAL		0x1C
#define FCC_3000MA_VAL		0x1F
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	/* we use our dual charging settings, so skip this check or settings */
	pr_smb(PR_STATUS, "skip this function\n");
	return 0;

#if 0 //remove qualcomm parallel charge
	int i, rc;
	u8 cur_val;

	/* the fcc enumerations are the same as the usb currents */
	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	if (chip->tables.usb_ilim_ma_table[i] == chip->fastchg_current_ma) {
		pr_smb(PR_STATUS, "skipping fastchg current request: %d\n",
				chip->fastchg_current_ma);
		return 0;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "fastcharge current requested %d, set to %d\n",
			current_ma, chip->tables.usb_ilim_ma_table[cur_val]);

	chip->fastchg_current_ma = chip->tables.usb_ilim_ma_table[cur_val];
	return rc;
#endif
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define ICL_MODE_MASK			SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
#define ICL_MODE_100MA			0x10
#define ICL_MODE_500MA			0x20
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	1000
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
#define PARALLEL_CHG_THRESHOLD_CURRENT	3000
#else
#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
#endif
static bool smbchg_is_usbin_active_pwr_src(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	return !(reg & USBIN_SUSPEND_STS_BIT)
		&& (reg & USBIN_ACTIVE_PWR_SRC_BIT);
}

static void smbchg_detect_parallel_charger(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (parallel_psy)
		chip->parallel_charger_detected =
			power_supply_set_present(parallel_psy, true) ?
								false : true;
}

#if 0 //remove qualcomm parallel charge
static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

#define ESR_PULSE_CURRENT_DELTA_MA	200
static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc, fg_current_now, icl_ma;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW,
						&fg_current_now);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	icl_ma = max(chip->iterm_ma + ESR_PULSE_CURRENT_DELTA_MA,
				fg_current_now - ESR_PULSE_CURRENT_DELTA_MA);
	rc = vote(chip->fcc_votable, ESR_PULSE_FCC_VOTER, en, icl_ma);
	if (rc < 0) {
		pr_err("Couldn't Vote FCC en = %d rc = %d\n", en, rc);
		return rc;
	}
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}
#endif

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
	pr_smb(PR_STATUS, "Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct smbchg_chip *chip, bool en)
{
	mutex_lock(&chip->taper_irq_lock);
	if (en != chip->taper_irq_enabled) {
		if (en) {
			enable_irq(chip->taper_irq);
			enable_irq_wake(chip->taper_irq);
		} else {
			disable_irq_wake(chip->taper_irq);
			disable_irq_nosync(chip->taper_irq);
		}
		chip->taper_irq_enabled = en;
	}
	mutex_unlock(&chip->taper_irq_lock);
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= chip->tables.usb_ilim_ma_len) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return chip->tables.usb_ilim_ma_table[reg];
}

static void smbchg_parallel_usb_disable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->parallel.initial_aicl_ma = 0;
	chip->parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
				SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	smbchg_set_fastchg_current_raw(chip,
			get_effective_result_locked(chip->fcc_votable));
	smbchg_set_usb_current_max(chip,
			get_effective_result_locked(chip->usb_icl_votable));
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	smbchg_detect_parallel_charger(chip);
	if (!chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
				|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
			* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
		pval.intval);
	/* Change it to uA */
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/*
	 * sleep here for 100 ms in order to make sure the charger has a chance
	 * to go back into constant current charging
	 */
	mutex_unlock(&chip->parallel.lock);
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

#if 0 //remove qualcomm parallel charge
static void smbchg_parallel_usb_enable(struct smbchg_chip *chip,
		int total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int new_parallel_cl_ma, set_parallel_cl_ma, new_pmi_cl_ma, rc;
	int current_table_index, target_icl_ma;
	int fcc_ma, main_fastchg_current_ma;
	int target_parallel_fcc_ma, supplied_parallel_fcc_ma;
	int parallel_chg_fcc_percent;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");

	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set Vflt on parallel psy rc: %d\n", rc);
		return;
	}
	/* Set USB ICL */
	target_icl_ma = get_effective_result_locked(chip->usb_icl_votable);
	new_parallel_cl_ma = total_current_ma
			* (100 - smbchg_main_chg_icl_percent) / 100;
	taper_irq_en(chip, true);
	power_supply_set_present(parallel_psy, true);
	power_supply_set_current_limit(parallel_psy,
				new_parallel_cl_ma * 1000);
	/* read back the real amount of current we are getting */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	set_parallel_cl_ma = pval.intval / 1000;
	chip->parallel.current_max_ma = new_parallel_cl_ma;
	pr_smb(PR_MISC, "Requested ICL = %d from parallel, got %d\n",
		new_parallel_cl_ma, set_parallel_cl_ma);
	new_pmi_cl_ma = max(0, target_icl_ma - set_parallel_cl_ma);
	pr_smb(PR_STATUS, "New Total USB current = %d[%d, %d]\n",
		total_current_ma, new_pmi_cl_ma,
		set_parallel_cl_ma);
	smbchg_set_usb_current_max(chip, new_pmi_cl_ma);

	/* begin splitting the fast charge current */
	fcc_ma = get_effective_result_locked(chip->fcc_votable);
	parallel_chg_fcc_percent =
		100 - smbchg_main_chg_fcc_percent;
	target_parallel_fcc_ma =
		(fcc_ma * parallel_chg_fcc_percent) / 100;
	pval.intval = target_parallel_fcc_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/* check how much actual current is supplied by the parallel charger */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	supplied_parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_MISC, "Requested FCC = %d from parallel, got %d\n",
		target_parallel_fcc_ma, supplied_parallel_fcc_ma);

	/* then for the main charger, use the left over FCC */
	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			fcc_ma - supplied_parallel_fcc_ma,
			chip->tables.usb_ilim_ma_len);
	main_fastchg_current_ma =
		chip->tables.usb_ilim_ma_table[current_table_index];
	smbchg_set_fastchg_current_raw(chip, main_fastchg_current_ma);
	pr_smb(PR_STATUS, "FCC = %d[%d, %d]\n", fcc_ma, main_fastchg_current_ma,
					supplied_parallel_fcc_ma);

	chip->parallel.enabled_once = true;

	return;
}

static bool smbchg_is_parallel_usb_ok(struct smbchg_chip *chip,
		int *ret_total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int min_current_thr_ma, rc, type;
	int total_current_ma, current_limit_ma, parallel_cl_ma;
	ktime_t kt_since_last_disable;
	u8 reg;
	int fcc_ma = get_effective_result_locked(chip->fcc_votable);
	int fcc_voter_id = get_effective_client_id_locked(chip->fcc_votable);
	int usb_icl_ma = get_effective_result_locked(chip->usb_icl_votable);

	if (!parallel_psy || !smbchg_parallel_en
			|| !chip->parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
					chip->parallel.last_disabled);
	if (chip->parallel.current_max_ma == 0
		&& chip->parallel.enabled_once
		&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	/*
	 * If the battery is not present, try not to change parallel charging
	 * from OFF to ON or from ON to OFF, as it could cause the device to
	 * brown out in the instant that the USB settings are changed.
	 *
	 * Only allow parallel charging check to report false (thereby turnin
	 * off parallel charging) if the battery is still there, or if parallel
	 * charging is disabled in the first place.
	 */
	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST
			&& (get_prop_batt_present(chip)
				|| chip->parallel.current_max_ma == 0)) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	/*
	 * If USBIN is suspended or not the active power source, do not enable
	 * parallel charging. The device may be charging off of DCIN.
	 */
	if (!smbchg_is_usbin_active_pwr_src(chip)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}

	if (usb_icl_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
			usb_icl_ma, min_current_thr_ma);
		return false;
	}

	/*
	 * Suspend the parallel charger if the charging current is < 1800 mA
	 * and is not because of an ESR pulse.
	 */
	if (fcc_voter_id != ESR_PULSE_FCC_VOTER
			&& fcc_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "FCC %d lower than %d\n",
			fcc_ma,
			PARALLEL_CHG_THRESHOLD_CURRENT);
		return false;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		return false;

	if (chip->parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
				current_limit_ma, min_current_thr_ma);
			return false;
		}
		chip->parallel.initial_aicl_ma = current_limit_ma;
	}

	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	parallel_cl_ma = pval.intval / 1000;
	/*
	 * Read back the real amount of current we are getting
	 * Treat 2mA as 0 because that is the suspend current setting
	 */
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	/*
	 * Set the parallel charge path's input current limit (ICL)
	 * to the total current / 2
	 */
	total_current_ma = min(current_limit_ma + parallel_cl_ma, usb_icl_ma);

	if (total_current_ma < chip->parallel.initial_aicl_ma
			- chip->parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
			"Total current reduced a lot: %d (%d + %d) < %d - %d\n",
			total_current_ma,
			current_limit_ma, parallel_cl_ma,
			chip->parallel.initial_aicl_ma,
			chip->parallel.allowed_lowering_ma);
		return false;
	}

	*ret_total_current_ma = total_current_ma;
	return true;
}

#define PARALLEL_CHARGER_EN_DELAY_MS	500
static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				parallel_en_work.work);
	int previous_aicl_ma, total_current_ma, aicl_ma;
	bool in_progress;

	/* do a check to see if the aicl is stable */
	previous_aicl_ma = smbchg_get_aicl_level_ma(chip);
	msleep(PARALLEL_CHARGER_EN_DELAY_MS);
	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (previous_aicl_ma == aicl_ma) {
		pr_smb(PR_STATUS, "AICL at %d\n", aicl_ma);
	} else {
		pr_smb(PR_STATUS,
			"AICL changed [%d -> %d], recheck %d ms\n",
			previous_aicl_ma, aicl_ma,
			PARALLEL_CHARGER_EN_DELAY_MS);
		goto recheck;
	}

	mutex_lock(&chip->parallel.lock);
	in_progress = (chip->parallel.current_max_ma != 0);
	if (smbchg_is_parallel_usb_ok(chip, &total_current_ma)) {
		smbchg_parallel_usb_enable(chip, total_current_ma);
	} else {
		if (in_progress) {
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
		}
	}
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_CHECK);
	return;

recheck:
	schedule_delayed_work(&chip->parallel_en_work, 0);
}

static void smbchg_parallel_usb_check_ok(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
	schedule_delayed_work(&chip->parallel_en_work, 0);
}
#endif

static int charging_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_charging_en(chip, !suspend);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't configure batt chg: 0x%x rc = %d\n",
			!suspend, rc);
	}

	return rc;
}

static int usb_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_usb_suspend(chip, suspend);
	if (rc < 0)
		return rc;

#if 0 //remove qualcomm parallel charge
	if (client == THERMAL_EN_VOTER || client == POWER_SUPPLY_EN_VOTER ||
				client == USER_EN_VOTER)
		smbchg_parallel_usb_check_ok(chip);
#endif
	return rc;
}

static int dc_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_dc_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
		power_supply_changed(&chip->dc_psy);

	return rc;
}

static int set_fastchg_current_vote_cb(struct device *dev,
						int fcc_ma,
						int client,
						int last_fcc_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc;

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_fastchg_current_raw(chip, fcc_ma);
		if (rc < 0) {
			pr_err("Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
			return rc;
		}
	}
#if 0 //remove qualcomm parallel charge
	/*
	 * check if parallel charging can be enabled, and if enabled,
	 * distribute the fcc
	 */
	smbchg_parallel_usb_check_ok(chip);
#endif
	return 0;
}

static int smbchg_set_fastchg_current_user(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);

	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true, current_ma);
	if (rc < 0)
		pr_err("Couldn't vote en rc %d\n", rc);
	return rc;
}

static struct ilim_entry *smbchg_wipower_find_entry(struct smbchg_chip *chip,
				struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
			uv))
			ret = &map->entries[i];
	}
	return ret;
}

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct smbchg_chip *chip, int offset, int ma)
{
	int i, rc;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			ma, chip->tables.dc_ilim_ma_len);

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct smbchg_chip *chip,
						struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->current_ilim.vmax_uv)
		return 0;

	chip->param.channel = DCIN;
	chip->param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->param.timer_interval = wipower_dcin_interval;
	chip->param.threshold_notification = &btm_notify_dcin;
	chip->param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->vadc_dev, &chip->param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->current_ilim.vmin_uv = ilim->vmin_uv;
		chip->current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct smbchg_chip *chip,
						int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->wipower_div2 : &chip->wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct smbchg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->wipower_default.entries[0]);

	if (!chip->wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->wipower_configured = false;
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	chip->current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct smbchg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			 && !usb_present
			&& dc_present
			&& chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		/* check div_by_2 */
		rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
			"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
			chg_type, usb_present, dc_present, dcin_uv,
			adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
			"deconfig ICL chg_type = %d usb = %d dc = %d\n",
			chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct smbchg_chip *chip)
{
	if (!chip->wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct smbchg_chip *chip = ctx;

	mutex_lock(&chip->wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct smbchg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

/*
 * set the dc charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_dc_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	return smbchg_set_dc_current_max(chip, icl_ma);
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_usb_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc, aicl_ma, effective_id;

	effective_id = get_effective_client_id_locked(chip->usb_icl_votable);

	/* disable parallel charging if HVDCP is voting for 300mA */
	if (effective_id == HVDCP_ICL_VOTER)
		smbchg_parallel_usb_disable(chip);

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_usb_current_max(chip, icl_ma);
		if (rc) {
			pr_err("Failed to set usb current max: %d\n", rc);
			return rc;
		}
	}

	/* skip the aicl rerun if hvdcp icl voter is active */
	if (effective_id == HVDCP_ICL_VOTER)
		return 0;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (icl_ma > aicl_ma)
		smbchg_rerun_aicl(chip);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	return 0;
}

#define T1 510
#define T2 570
#define HVDCP_ADAPTER_SEL_MASK	SMB_MASK(5, 4)
#define HVDCP_5V		0x00
#define HVDCP_9V		0x01
#define USB_CMD_HVDCP_1		0x42
#define FORCE_HVDCP_2p0		BIT(3)

static int smbchg_force_charging_limit(struct smbchg_chip *chip, bool b_stop_charging);
static int smbchg_dp_dm(struct smbchg_chip *chip, int val);
#ifndef ENG_BUILD
static void thermal_policy_QC3(struct smbchg_chip *chip, int temp, int *thermal_ma)
{
	int i;

	pr_smb(PR_STATUS, "<%s> temp = %d\n", __func__, temp);
	thermal_policy_flag = TP_LEVEL0;
	if (temp >= T1){
		if (chip->pulse_cnt > 0){
			smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_PREPARE);
			for(i = 0; i < 4; i++){
				smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_DM_PULSE);
				msleep(50);
			}
			smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
		}
	}
	else{
		if (chip->pulse_cnt < 20){
			smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_PREPARE);
			for(i = 0; i < 4; i++){
				smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_DP_PULSE);
				msleep(50);
			}
			smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
		}
	}
}

static void thermal_policy_QC2(struct smbchg_chip *chip, int temp, int *thermal_ma)
{
	pr_smb(PR_STATUS, "<%s> temp = %d\n", __func__, temp);
	thermal_policy_flag = TP_LEVEL0;
	if (temp >= T1)
		smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG, HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	else
		smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG, HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
}

static void thermal_policy_ASUS10W(struct smbchg_chip *chip, int temp, int *thermal_ma)
{
	pr_smb(PR_STATUS, "<%s> temp = %d\n", __func__, temp);
	if (temp >= T1){
		*thermal_ma = USBIN_IL_900MA;
		thermal_policy_flag = TP_LEVEL1;
#ifdef SUPPORT_PARALLEL_CHG
		dual_chg_disable();
#endif
	}
	else{
		*thermal_ma = USBIN_IL_1800MA;
		thermal_policy_flag = TP_LEVEL0;
	}
}

#ifdef SUPPORT_PD_ADAPTER
static void thermal_policy_PD(struct smbchg_chip *chip, int temp, int *thermal_ma)
{
	pr_smb(PR_STATUS, "<%s> temp = %d\n", __func__, temp);
	thermal_policy_flag = TP_LEVEL0;
	if (temp >= T1){
		thermal_policy_flag = TP_LEVEL1;
#ifdef SUPPORT_PARALLEL_CHG
		dual_chg_disable();
#endif
	}
	else{
		thermal_policy_flag = TP_LEVEL0;
	}
}
#endif

static void thermal_policy_TYPC(struct smbchg_chip *chip, int temp, int *thermal_ma)
{
	pr_smb(PR_STATUS, "<%s> temp = %d\n", __func__, temp);
	thermal_policy_flag = TP_LEVEL0;
	if (temp >= T1){
		thermal_policy_flag = TP_LEVEL1;
#ifdef SUPPORT_PARALLEL_CHG
		dual_chg_disable();
#endif
	}
	else{
		thermal_policy_flag = TP_LEVEL0;
	}
}
#endif
static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl = 0;
	int thermal_icl_ma;
	int temp;
#ifdef SUPPORT_PD_ADAPTER
	int pd_supply;
	union power_supply_propval value;
#endif
#ifndef ENG_BUILD
	int soc;
	int thermal_policy_ma = USBIN_IL_500MA;
	char *usb_type_name = "null";
	enum power_supply_type usb_supply_type;
#endif
	temp = get_prop_batt_temp(chip);

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

#ifndef ENG_BUILD
	pr_smb(PR_STATUS, "system lvl_sel = %d ###\n", lvl_sel);
	
	read_usb_type(chip, &usb_type_name, &usb_supply_type);

	if(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
	{
	switch(lvl_sel)
	{
		case 0:
			smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
			thermal_policy_ma = USBIN_IL_1800MA;
			break;
		case 1:
			if(thermal_policy_flag == 3)
				smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
			else if(thermal_policy_flag == 2){
				// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
				rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
				if (rc < 0)
					pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);

				// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
				rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
				if (rc < 0 )
					pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);

				// 3. Enable USBIN_AICL , 0x13F3[2] = "1"
				rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
				if (rc < 0 )
					pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
			}

#ifdef SUPPORT_PD_ADAPTER
			chip->pd_psy->get_property(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE, &value);
			pd_supply = value.intval;
			if (pd_supply)
				thermal_policy_PD(chip, temp, &thermal_policy_ma);
			else if ((dual_chg_flag == Single||dual_chg_flag == ASUS_2A) && hvdcp_flag == HVDCP_2)
#else
			if ((dual_chg_flag == Single||dual_chg_flag == ASUS_2A) && hvdcp_flag == HVDCP_2)
#endif
				thermal_policy_QC2(chip, temp, &thermal_policy_ma);
			else if ((dual_chg_flag == Single||dual_chg_flag == ASUS_2A) && hvdcp_flag == HVDCP_3)
				thermal_policy_QC3(chip, temp, &thermal_policy_ma);
			else if (dual_chg_flag == ASUS_2A && hvdcp_flag == NO_HVDCP)
				thermal_policy_ASUS10W(chip, temp, &thermal_policy_ma);
			else if (dual_chg_flag == TYPC_3A)
				thermal_policy_TYPC(chip, temp, &thermal_policy_ma);
			else if (dual_chg_flag == Single && hvdcp_flag == NO_HVDCP)
				return 0;

			break;
		case 2:
			if (thermal_policy_flag == 3)
				smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
			else if(thermal_policy_flag != 2){
			}

			thermal_policy_flag = 2;
#ifdef SUPPORT_PARALLEL_CHG
			dual_chg_disable();
#endif
			soc = get_prop_batt_capacity(chip);
			if(soc < 8)
				thermal_policy_ma = USBIN_IL_900MA;
			else if((soc < 15) && (soc >= 8))
				thermal_policy_ma = USBIN_IL_700MA;
			else // soc >= 15%
				thermal_policy_ma = USBIN_IL_500MA;				
			break;
		case 3:
			// Set charger enter suspend mode
			smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, USBIN_SUSPEND_BIT);
#ifdef SUPPORT_PARALLEL_CHG
			dual_chg_disable();
#endif
			break;
	}
	
	if(lvl_sel < 3)
	{
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, thermal_policy_ma);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// Set charger exit suspend mode
		smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
	}
	
	smbchg_rerun_aicl(chip);
	return rc;
	}
#endif
	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->therm_lvl_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/*
		 * Disable charging if highest value selected by
		 * setting the DC and USB path in suspend
		 */
		rc = vote(chip->dc_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->usb_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	if (chip->therm_lvl_sel == 0) {
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable USB thermal ICL vote rc=%d\n",
				rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable DC thermal ICL vote rc=%d\n",
				rc);
	} else {
		thermal_icl_ma =
			(int)chip->thermal_mitigation[chip->therm_lvl_sel];
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for USB thermal ICL rc=%d\n", rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for DC thermal ICL rc=%d\n", rc);
	}

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = vote(chip->dc_suspend_votable, THERMAL_EN_VOTER, false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->usb_suspend_votable, THERMAL_EN_VOTER,
								false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->therm_lvl_lock);
	return rc;
}

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	/*
	 * Calculate the maximum current that can pulled out of the battery
	 * before the battery voltage dips below a safe threshold.
	 */
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		/*
		 * If the calculated current is below the OCP threshold, then
		 * use it as the possible flash current.
		 */
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		/*
		 * If the calculated current is above the OCP threshold, then
		 * use the ocp threshold instead.
		 *
		 * Any higher current will be tripping the battery OCP.
		 */
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	/* Calculate the input voltage of the flash module. */
	vin_flash_uv = max((chip->vled_max_uv + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	/* Calculate the available power for the flash module. */
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	/*
	 * Calculate the available amount of current the flash module can draw
	 * before collapsing the battery. (available power/ flash input voltage)
	 */
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
		"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct smbchg_chip *chip,
					int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < chip->tables.fcc_comp_len; i++)
		if (comp_current == chip->tables.fcc_comp_table[i])
			break;

	if (i >= chip->tables.fcc_comp_len)
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
			rc);

	return rc;
}

#define CFG_TCC_REG			0xF9
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
static int smbchg_iterm_set(struct smbchg_chip *chip, int iterm_ma)
{
	int rc;
	u8 reg;

	reg = find_closest_in_array(
			chip->tables.iterm_ma_table,
			chip->tables.iterm_ma_len,
			iterm_ma);

	rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CFG_TCC_REG,
			CHG_ITERM_MASK, reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
			iterm_ma, reg);
	chip->iterm_ma = iterm_ma;

	return 0;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
			rc);

	return rc;
}

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)
#define VFLOAT_4P10			0x1E
#define VFLOAT_4P38			0x2D

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
				/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
				/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		/* very high range */
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
				/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
				rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->vfloat_mv = vfloat_mv;

	return rc;
}

static int smbchg_float_voltage_get(struct smbchg_chip *chip)
{
	return chip->vfloat_mv;
}

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
static int smbchg_safety_timer_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s safety timer rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->safety_timer_en = enable;
	return 0;
}

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
static int smbchg_force_tlim_en(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg force tlim rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}

static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#if 0 //remove qualcomm parallel charge
#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	if (!(chip->wa_flags & SMBCHG_CC_ESR_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	/*
	 * The esr_count is counting down the number of fuel gauge cycles
	 * before a ESR pulse is needed.
	 *
	 * After a successful ESR pulse, this count is reset to some
	 * high number like 28. If this reaches 0, then the fuel gauge
	 * hardware should force a ESR pulse.
	 *
	 * However, if the device is in constant current charge mode while
	 * being input current limited, the ESR pulse will not affect the
	 * battery current, so the measurement will fail.
	 *
	 * As a failsafe, force a manual ESR pulse if this value is read as
	 * 0.
	 */
	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}
#endif

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_en(struct smbchg_chip *chip, bool en)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, en ? AICL_RERUN_ON : AICL_RERUN_OFF);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);
	return rc;
}

static int smbchg_aicl_config(struct smbchg_chip *chip)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	if (!chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Couldn't enable AICL rerun rc= %d\n", rc);
	}
	return rc;
}


static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	if (chip->force_aicl_rerun)
		return;
	if (en && !chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		if (!chip->very_weak_charger) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc) {
				pr_err("Couldn't enable AICL rerun rc= %d\n",
						rc);
				return;
			}
		}
		pr_smb(PR_STATUS, "AICL deglitch set to short\n");
	} else if (!en && chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_hw_aicl_rerun_en(chip, false);
		if (rc) {
			pr_err("Couldn't disable AICL rerun rc= %d\n", rc);
			return;
		}
		pr_smb(PR_STATUS, "AICL deglitch set to normal\n");
	}
	chip->aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	u8 reg;
	bool low_volt_chgr = true;

	if (!(chip->wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->bms_psy)
		return;

	if (is_usb_present(chip)) {
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
		if (rc < 0) {
			pr_err("Couldn't read hvdcp status rc = %d\n", rc);
			return;
		}
		if (reg & USBIN_HVDCP_SEL_BIT)
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	/* It is possible that battery voltage went high above threshold
	 * when the charger is inserted and can go low because of system
	 * load. We shouldn't be reconfiguring AICL deglitch when this
	 * happens as it will lead to oscillation again which is being
	 * fixed here. Do it once when the battery voltage crosses the
	 * threshold (e.g. 4.2 V) and clear it only when the charger
	 * is removed.
	 */
	if (!chip->vbat_above_headroom) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->vbat_above_headroom);
}
#define MISC_TEST_REG		0xE2
#define BB_LOOP_DISABLE_ICL	BIT(2)
static int smbchg_icl_loop_disable_check(struct smbchg_chip *chip)
{
	bool icl_disabled = !chip->chg_otg_enabled && chip->flash_triggered;
	int rc = 0;

	if ((chip->wa_flags & SMBCHG_FLASH_ICL_DISABLE_WA)
			&& icl_disabled != chip->icl_disabled) {
		rc = smbchg_sec_masked_write(chip,
				chip->misc_base + MISC_TEST_REG,
				BB_LOOP_DISABLE_ICL,
				icl_disabled ? BB_LOOP_DISABLE_ICL : 0);
		chip->icl_disabled = icl_disabled;
	}

	return rc;
}

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
#if 0 //remove qualcomm parallel charge
static int smbchg_config_chg_battery_type(struct smbchg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0, iterm_ua = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
		!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	/* quit if there is no change in the battery-type from previous */
	if (chip->battery_type && !strcmp(prop.strval, chip->battery_type))
		return 0;
#if defined(CONFIG_ZT582KL)
	batt_node = of_parse_phandle(node, "qcom,battery-data-zt582kl", 0);
#elif defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
	batt_node = of_parse_phandle(node, "qcom,battery-data-zt581kl", 0);
#elif defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	batt_node = of_parse_phandle(node, "qcom,battery-data-zt500kl", 0);
#else
	batt_node = of_parse_phandle(node, "qcom,battery-data-zt581kl", 0);
#endif
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}
	else
#if defined(CONFIG_ZT582KL)
		pr_smb(PR_MISC, "ZT582KL batterydata available\n");
#elif defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
		pr_smb(PR_MISC, "ZT581KL batterydata available\n");
#elif defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
		pr_smb(PR_MISC, "ZT500KL batterydata available\n");
#else
		pr_smb(PR_MISC, "Default batterydata available\n");
#endif

	profile_node = of_batterydata_get_best_profile(batt_node,
							"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->battery_type = prop.strval;

	/* change vfloat */
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
						&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
				chip->vfloat_mv, (max_voltage_uv / 1000),
				chip->battery_type);
			rc = smbchg_float_voltage_set(chip,
						(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	/* change chg term */
	rc = of_property_read_u32(profile_node, "qcom,chg-term-ua",
						&iterm_ua);
	if (rc && rc != -EINVAL) {
		pr_warn("couldn't read battery term current=%d\n", rc);
		ret = rc;
	} else if (!rc) {
		if (chip->iterm_ma != (iterm_ua / 1000)
				&& !chip->iterm_disabled) {
			pr_info("Term current changed from %dmA to %dmA for battery-type %s\n",
				chip->iterm_ma, (iterm_ua / 1000),
				chip->battery_type);
			rc = smbchg_iterm_set(chip,
						(iterm_ua / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
		chip->iterm_ma = iterm_ua / 1000;
	}

	/*
	 * Only configure from profile if fastchg-ma is not defined in the
	 * charger device node.
	 */
	if (!of_find_property(chip->spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
				"fastchg-ma changed from to %dma for battery-type %s\n",
				fastchg_ma, chip->battery_type);
			rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,
							fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't vote for fastchg current rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	return ret;
}
#endif

static void check_battery_type(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		vote(chip->battchg_suspend_votable,
				BATTCHG_UNKNOWN_BATTERY_EN_VOTER, !en, 0);
	}
}

#ifdef ENG_BUILD
#define CHG_TYPE_NOW_FILE_PATH	"/data/data/CHG_TYPE"

enum adapter_type {
	ASUS_NONE = 0,
	ASUS_750K = 1,
	ASUS_200K = 2,	
	PD_ADAPTER = 3,
};

static int set_adapter_type(enum adapter_type adapter_type_value)
{
	struct file* filp = NULL;
	char buf[50]={0};
	//mm_segment_t old_fs;
		
	filp = filp_open(CHG_TYPE_NOW_FILE_PATH, O_RDWR | O_CREAT, 0666);
	if(IS_ERR_OR_NULL(filp))
	{		
		pr_err("%s: file open error (%s)\n", __func__, CHG_TYPE_NOW_FILE_PATH);
		return -EFAULT;		
	}
	
	//old_fs=get_fs();
	//set_fs(get_ds());
	
	switch(adapter_type_value)
	{
		case ASUS_750K:
			sprintf(buf, "%s", "DCP_ASUS_750K_2A\n");
			break;
		
		case ASUS_200K:
			sprintf(buf, "%s", "HVDCP_ASUS_200K_2A\n");
			break;
			
		case PD_ADAPTER:
			sprintf(buf, "%s", "PD\n");
			break;
			
		case ASUS_NONE:
		default:
			sprintf(buf, "%s", "None\n");
			break;
	}
	
	filp->f_op->write(filp, buf, sizeof(buf), &filp->f_pos);
	//filp->f_op->read(filp, buf, sizeof(buf), &filp->f_pos);
	//pr_err("Adapter type is %s ####\n", buf);	
 	
	filp_close(filp, NULL);
	
	return 0;
}
#endif

#ifdef SUPPORT_PD_ADAPTER
static void pd_supply_dump_status(struct power_supply *psy)
{
	static char *scope_text[] = {
		"Unknown", "System", "Device", "source", "sink"
	};
	int rc;
	union power_supply_propval value;

	pr_smb(PR_MISC,"++ PD supply changed, dump properties ++\n");
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_SCOPE, &value);
	if (rc == 0) {
		pr_smb(PR_MISC,"PD supply: SCOPE=%s\n", scope_text[value.intval]);
	}
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if (rc == 0) {
		pr_smb(PR_MISC,"PD supply: ONLINE=%d\n",value.intval);
	}
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &value);
	if (rc == 0) {
		pr_smb(PR_MISC,"PD supply: VOLTAGE_MAX=%u uV\n", value.intval);
	}
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MIN, &value);
	if (rc == 0) {
		pr_smb(PR_MISC,"PD supply: VOLTAGE_MIN=%u uV\n", value.intval);
	}
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (rc == 0) {
		pr_smb(PR_MISC,"PD supply: CURRENT_MAX=%u uA\n", value.intval);
	}
	pr_smb(PR_MISC,"-- PD supply changed -------------------\n");
}
#endif

static void smbchg_external_power_changed(struct power_supply *psy)
{
#ifdef SUPPORT_PD_ADAPTER
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);
	union power_supply_propval value;
	int rc, current_limit = 0;
	int pd_supply;
	int pd_current;
#endif	
#if 0 //remove qualcomm parallel charge
	int soc;
	union power_supply_propval prop = {0,};
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
#endif
#ifdef SUPPORT_PD_ADAPTER
	if (chip->pd_psy) {
		/* We have external power supply - Power Delivery */
		/* @TODO.
		 * Do things reflect PD property change.
		 *
		 * @NOTE.
		 * Don't use PD supply change event to determine VBUS present
		 * of absent. */
		chip->pd_psy->get_property(chip->pd_psy, POWER_SUPPLY_PROP_ONLINE, &value);
		pd_supply = value.intval;
//		pr_smb(PR_MISC, "Enter PD supply check, PD supply status =  %d\n", pd_supply);	
		
		if(pd_supply)
		{
			pr_smb(PR_MISC, "Enter PD supply setting \n");
			pd_supply_dump_status(chip->pd_psy);
			chip->charging_type = CHG_TYPE_DCP_TYPC_3;
			boost_up_voltage_done = true;

			rc = chip->pd_psy->get_property(chip->pd_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
			if (value.intval == 0){
				pr_smb(PR_MISC,"PD supply: CURRENT_MAX=%u uA\n", value.intval);
				rc = chip->pd_psy->get_property(chip->pd_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &value);
				if(value.intval == 0)
					pr_smb(PR_MISC,"PD supply: VOLTAGE_MAX=%u uV\n", value.intval);

				smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 1);
				return;
			}
			else{
				smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
			}

			pd_current = value.intval;
			smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USB_CMD_IL_MASK, USB_CMD_IL_CONFIG);

			rc = chip->pd_psy->get_property(chip->pd_psy, POWER_SUPPLY_PROP_VOLTAGE_MIN, &value);
			if(value.intval<=9000000 && value.intval>6000000){
				if(pd_current >= 2000000)
					pd_current = 2000000;
			}
			else if (value.intval > 9000000)
				pd_current = 0;

			if(pd_current >= 3000000){
				pr_smb(PR_MISC, "PD Enter 3000000 setting \n");
				dual_chg_flag = TYPC_3A;	
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
					USBIN_INPUT_MASK, USBIN_IL_1400MA);
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, 0);
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, AICL_EN_BIT);
			}
			else if (pd_current >= 2000000 && pd_current < 3000000){
				pr_smb(PR_MISC, "PD Enter 2000000 setting \n");
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, 0);
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, AICL_EN_BIT);
				dual_chg_flag = ASUS_2A;
			}
			else if (pd_current >= 1000000 && pd_current < 2000000){
				pr_smb(PR_MISC, "PD Enter 1000000 setting \n");
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, 0);
				smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, AICL_EN_BIT);
				dual_chg_flag = Single;
			}
			current_limit = value.intval / 1000;
#ifdef ENG_BUILD
			set_adapter_type(PD_ADAPTER);
#endif
		}
	}
	else
	{
		pr_smb(PR_MISC, "It is not PD supply. \n");
	}
	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
#endif
	/* we use our dual charging settings, so skip this check or settings */
	pr_smb(PR_STATUS, "skip this function\n");
	return;
#if 0 //remove qualcomm parallel charge
	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->bms_psy) {
		check_battery_type(chip);
		soc = get_prop_batt_capacity(chip);
		if (chip->previous_soc != soc) {
			chip->previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
				"Couldn't update charger configuration rc=%d\n",
									rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc == 0)
		vote(chip->usb_suspend_votable, POWER_SUPPLY_EN_VOTER,
				!prop.intval, 0);

#ifdef SUPPORT_PD_ADAPTER
	if (chip->pd_psy && pd_supply) {
		pr_smb(PR_MISC, "current is limited by PD\n");
	} else 
#endif
	{
		rc = chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		if (rc == 0)
			current_limit = prop.intval / 1000;
	}

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type != POWER_SUPPLY_TYPE_USB)
		goto  skip_current_for_non_sdp;

	pr_smb(PR_MISC, "usb type = %s current_limit = %d\n",
			usb_type_name, current_limit);

	rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit);
	if (rc < 0)
		pr_err("Couldn't update USB PSY ICL vote rc=%d\n", rc);
skip_current_for_non_sdp:
//	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
#endif
}

#define OTG_CFG			0xF1
#define	OTG_CFG_MASK	SMB_MASK(7, 0)
#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
#define OTG_CFG_CONFIG	0x68
#else //defined (CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
#define OTG_CFG_CONFIG	0x28
#endif

#define OTG_CFG_BATTUV			0xF2
#define	OTG_CFG_BATTUV_MASK		SMB_MASK(7, 0)
#define OTG_CFG_BATTUV_CONFIG	0x01

#define OTG_ICFG			0xF3
#define OTG_ILIMT_MASK		SMB_MASK(1, 0)
#define OTG_ILIMT_250MA		0x00
#define OTG_ILIMT_600MA		0x01
#define OTG_ILIMT_1000MA	0x03

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	chip->chg_otg_enabled = true;
	smbchg_icl_loop_disable_check(chip);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);
	/* sleep to make sure the pulse skip is actually disabled */
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_ICFG,
			OTG_ILIMT_MASK, OTG_ILIMT_1000MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set OTG ILIMT config rc = %d\n", rc);

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	chip->chg_otg_enabled = false;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	smbchg_icl_loop_disable_check(chip);

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_ICFG,
			OTG_ILIMT_MASK, OTG_ILIMT_250MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set OTG ILIMT config rc = %d\n", rc);

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_CONT	0x2
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, true, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	/*
	 * To disallow source detect and usbin_uv interrupts, set the adapter
	 * allowance to 9V, so that the audio boost operating in reverse never
	 * gets detected as a valid input
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	/*
	 * Reenable HVDCP and set the adapter allowance back to the original
	 * value in order to allow normal USBs to be recognized as a valid
	 * input.
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, chip->original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	return get_client_vote(chip->usb_suspend_votable, OTG_EN_VOTER);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->ext_otg_vreg.rdev = regulator_register(
					&chip->ext_otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->ext_otg_vreg.rdev);
			chip->ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static void smbchg_regulator_deinit(struct smbchg_chip *chip)
{
	if (chip->otg_vreg.rdev)
		regulator_unregister(chip->otg_vreg.rdev);
	if (chip->ext_otg_vreg.rdev)
		regulator_unregister(chip->ext_otg_vreg.rdev);
}

#define CMD_CHG_LED_REG		0x43
#define CHG_LED_CTRL_BIT		BIT(0)
#define LED_SW_CTRL_BIT		0x1
#define LED_CHG_CTRL_BIT		0x0
#define CHG_LED_ON		0x03
#define CHG_LED_OFF		0x00
#define LED_BLINKING_PATTERN1		0x01
#define LED_BLINKING_PATTERN2		0x02
#define LED_BLINKING_CFG_MASK		SMB_MASK(2, 1)
#define CHG_LED_SHIFT		1
static int smbchg_chg_led_controls(struct smbchg_chip *chip)
{
	u8 reg, mask;
	int rc;

	if (chip->cfg_chg_led_sw_ctrl) {
		/* turn-off LED by default for software control */
		mask = CHG_LED_CTRL_BIT | LED_BLINKING_CFG_MASK;
		reg = LED_SW_CTRL_BIT;
	} else {
		mask = CHG_LED_CTRL_BIT;
		reg = LED_CHG_CTRL_BIT;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_LED_REG,
			mask, reg);
	if (rc < 0)
		dev_err(chip->dev,
				"Couldn't write LED_CTRL_BIT rc=%d\n", rc);
	return rc;
}

static void smbchg_chg_led_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg;
	int rc;

	reg = (value > LED_OFF) ? CHG_LED_ON << CHG_LED_SHIFT :
		CHG_LED_OFF << CHG_LED_SHIFT;

	pr_smb(PR_STATUS,
			"set the charger led brightness to value=%d\n",
			value);
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static enum
led_brightness smbchg_chg_led_brightness_get(struct led_classdev *cdev)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg_val, chg_led_sts;
	int rc;

	rc = smbchg_read(chip, &reg_val, chip->bat_if_base + CMD_CHG_LED_REG,
			1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read CHG_LED_REG sts rc=%d\n",
				rc);
		return rc;
	}

	chg_led_sts = (reg_val & LED_BLINKING_CFG_MASK) >> CHG_LED_SHIFT;

	pr_smb(PR_STATUS, "chg_led_sts = %02x\n", chg_led_sts);

	return (chg_led_sts == CHG_LED_OFF) ? LED_OFF : LED_FULL;
}

static void smbchg_chg_led_blink_set(struct smbchg_chip *chip,
		unsigned long blinking)
{
	u8 reg;
	int rc;

	if (blinking == 0)
		reg = CHG_LED_OFF << CHG_LED_SHIFT;
	else if (blinking == 1)
		reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
	else if (blinking == 2)
		reg = LED_BLINKING_PATTERN2 << CHG_LED_SHIFT;
	else
		reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;

	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static ssize_t smbchg_chg_led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct smbchg_chip *chip = container_of(cdev, struct smbchg_chip,
			led_cdev);
	unsigned long blinking;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &blinking);
	if (rc)
		return rc;

	smbchg_chg_led_blink_set(chip, blinking);

	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, smbchg_chg_led_blink_store);

#if defined(VZW)
extern int g_soc_in;
extern int g_soc_out;
extern int g_soc_diff;
static ssize_t show_q023_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("g_soc_in = %d\n", g_soc_in);
	return sprintf(buf, "%d\n", g_soc_in);
}

static DEVICE_ATTR(q023_in, 0444, show_q023_in, NULL);

static ssize_t show_q023_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("g_soc_out = %d\n", g_soc_out);
	return sprintf(buf, "%d\n", g_soc_out);
}

static DEVICE_ATTR(q023_out, 0444, show_q023_out, NULL);

static ssize_t show_q023_diff(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("g_soc_diff = %d\n", g_soc_diff);
	return sprintf(buf, "%d\n", g_soc_diff);
}

static DEVICE_ATTR(q023_diff, 0444, show_q023_diff, NULL);

static int q023_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_q023_in);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_q023_out);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_q023_diff);

	return 0;
}

struct platform_device q023_space_device = {
	.name = "q023",
	.id = -1,
};

static struct platform_driver q023_space_driver = {
	.probe	= q023_space_probe,
	.driver	= {
		.name = "q023",
	},
};
#endif
unsigned long para1 = 0;
static ssize_t asus_battery_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc, asus_battery_id;

	rc = get_property_from_fg(chip_fg, POWER_SUPPLY_PROP_RESISTANCE_ID, &asus_battery_id);
	pr_err("[asus_battery_id_show] POWER_SUPPLY_PROP_RESISTANCE_ID = %d\n", asus_battery_id);

	if (para1 == 1){
		para1 = 0;
		return sprintf(buf, "LGC_51K\n");
	}
	else if ( para1 == 0){
		if (asus_battery_id >= 45000 && asus_battery_id < 56000)
			return sprintf(buf, "PASS\n");
		else
			return sprintf(buf, "FAIL\n");
	}
	return sprintf(buf, "no support this parameter (%ld)\n",para1); 
}
static ssize_t asus_battery_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &para1);
	if (rc){
		pr_err("[asus_battery_id_store] write para1 fail!\n");
		return rc;
	}

	return len;
}
static DEVICE_ATTR(asus_battery_id, 0664, asus_battery_id_show, asus_battery_id_store);

static ssize_t asus_charging_flags_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *s_dual_chg_flag = dual_chg_flag_str[dual_chg_flag];
	char *s_hvdcp_flag = hvdcp_flag_str[hvdcp_flag];
	char *s_chg_type_flag = chg_type_flag_str[chg_type_flag];
	char *s_thermal_policy_flag = thermal_policy_flag_str[thermal_policy_flag];
	char *s_dfp_type_flag = dfp_type_flag_str[dfp_type_flag];

	return sprintf(buf, "dual_chg_flag = %s\n hvdcp_flag = %s\n chg_type_flag = %s\n thermal_policy_flag =%s\n dfp_type_flag = %s\n", s_dual_chg_flag, s_hvdcp_flag, s_chg_type_flag, s_thermal_policy_flag, s_dfp_type_flag);
}
static DEVICE_ATTR(asus_charging_flags, 0444, asus_charging_flags_show, NULL);

// Declare the rerun_apsd() function before using it
static int rerun_apsd(struct smbchg_chip *chip);
static ssize_t smbchg_rerun_apsd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("rerun_apsd_test START\n");
	pr_err("rerun_apsd_test DONE\n");

	return sprintf(buf,"rerun_apsd_test DONE\n");
}
static ssize_t smbchg_rerun_apsd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long need_to_rerun_apsd = 0;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &need_to_rerun_apsd);
	if (rc){
		pr_err("[smbchg_rerun_apsd_store] rerun_apsd fail!\n");
		return rc;
	}

	pr_err("[smbchg_rerun_apsd_store] need_to_rerun_apsd = %lu\n",need_to_rerun_apsd);
	if (need_to_rerun_apsd){
		g_cable_poweron = true;
		queue_delayed_work(adapter_wq ,&chip_fg->usb_insertion_work, msecs_to_jiffies(0));	
//		queue_delayed_work(adapter_wq ,&chip_fg->cable_poweron_work, msecs_to_jiffies(0));
	}

	return len;
}

static DEVICE_ATTR(RerunAPSD, 0664, smbchg_rerun_apsd_show, smbchg_rerun_apsd_store);

static int rerun_apsd_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_RerunAPSD);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_asus_battery_id);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_asus_charging_flags);

	return 0;
}

struct platform_device rerun_apsd_space_device = {
	.name = "rerun_apsd",
	.id = -1,
};

static struct platform_driver rerun_apsd_space_driver = {
	.probe	= rerun_apsd_space_probe,
	.driver	= {
		.name = "rerun_apsd",
	},
};

#if defined(ENG_BUILD)
static ssize_t show_charger_limit(struct device *dev, struct device_attribute *attr, char *buf)
{
        pr_err("charging_limit_threshold = %d\n", charging_limit_threshold);
        return sprintf(buf, "%d\n", charging_limit_threshold);
}
static ssize_t store_charger_limit(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int limit_threshold;
	static char b_buf[sizeof(int)];
	mm_segment_t old_fs;
	struct file *fp = NULL;

	sscanf(buf, "%du", &limit_threshold);
	charging_limit_threshold = limit_threshold;
	pr_err("charging_limit_threshold = %d\n", charging_limit_threshold);

	fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR|O_CREAT , 0666);
	if (!IS_ERR_OR_NULL(fp) ) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		sprintf(b_buf, "%d", charging_limit_threshold);
		fp->f_op->llseek(fp, 0, 0);
		fp->f_op->write(fp,b_buf,sizeof(b_buf),&fp->f_pos);
		pr_err("writr new charging_limit_threshold = %d to the file.\n", charging_limit_threshold);
		set_fs(old_fs);
		if(fp!=NULL)
			filp_close(fp, NULL);
	}
	
	return len;
}
static DEVICE_ATTR(charger_limit, 0664, show_charger_limit, store_charger_limit);

#ifdef SUPPORT_PARALLEL_CHG
static ssize_t store_smb1351_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int enable;

	sscanf(buf, "%du", &enable);
	pr_smb(PR_STATUS, "smb1351 enable = %d\n", enable);

	if(enable)
		dual_chg_enable(0x2, 0x70);
	else
		dual_chg_disable();

	return len;		
}
static DEVICE_ATTR(smb1351_enable, 0664, NULL, store_smb1351_enable);
#endif
static int charger_limit_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_charger_limit);
#ifdef SUPPORT_PARALLEL_CHG
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_smb1351_enable);
#endif
	
	return 0;
}

struct platform_device charger_limit_space_device = {
	.name = "charger_limit",
	.id = -1,
};

static struct platform_driver charger_limit_space_driver = {
	.probe  = charger_limit_space_probe,
		.driver = {
		.name = "charger_limit",
	},
};
#endif
static struct attribute *led_blink_attributes[] = {
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group smbchg_led_attr_group = {
	.attrs = led_blink_attributes
};

static int smbchg_register_chg_led(struct smbchg_chip *chip)
{
	int rc;

	chip->led_cdev.name = "red";
	chip->led_cdev.brightness_set = smbchg_chg_led_brightness_set;
	chip->led_cdev.brightness_get = smbchg_chg_led_brightness_get;

	rc = led_classdev_register(chip->dev, &chip->led_cdev);
	if (rc) {
		dev_err(chip->dev, "unable to register charger led, rc=%d\n",
				rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->led_cdev.dev->kobj,
			&smbchg_led_attr_group);
	if (rc) {
		dev_err(chip->dev, "led sysfs rc: %d\n", rc);
		return rc;
	}

	return rc;
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	/* check if we can accomodate delta steps with just the offset */
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	/* changing offset cannot satisfy delta steps, change the scale bits */
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	/*
	 * now that we have changed scale which is a 17mV jump, change the
	 * offset bits (10mV) too so the effective change is just 7mV
	 */
	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			/* treat no trim change as an error */
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (-ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	/* if max vbat > target vfloat, delta_vfloat_mv could be negative */
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	/*
	 * enough valid samples has been collected, adjust trim codes
	 * based on maximum of collected vbat samples if necessary
	 */
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

#define BAT_IF_TRIM7_REG	0xF7
#define CFG_750KHZ_BIT		BIT(1)
#define BB_CLMP_SEL		0xF8
#define BB_CLMP_MASK		SMB_MASK(1, 0)
#define BB_CLMP_VFIX_3338MV	0x1
#define BB_CLMP_VFIX_3512MV	0x2
static int smbchg_set_optimal_charging_mode(struct smbchg_chip *chip, int type)
{
	int rc;
	bool hvdcp2 = (type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& smbchg_is_usbin_active_pwr_src(chip));

	/*
	 * Set the charger switching freq to 1MHZ if HVDCP 2.0,
	 * or 750KHZ otherwise
	 */
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, hvdcp2 ? 0 : CFG_750KHZ_BIT);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	/*
	 * Set the charger switch frequency clamp voltage threshold to 3.338V
	 * if HVDCP 2.0, or 3.512V otherwise.
	 */
	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BB_CLMP_SEL,
			BB_CLMP_MASK,
			hvdcp2 ? BB_CLMP_VFIX_3338MV : BB_CLMP_VFIX_3512MV);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	return 0;
}

#define DEFAULT_SDP_MA		500
#define DEFAULT_CDP_MA		1500
static int smbchg_change_usb_supply_type(struct smbchg_chip *chip,
						enum power_supply_type type)
{
	int rc, current_limit_ma;

	/*
	 * if the type is not unknown, set the type before changing ICL vote
	 * in order to ensure that the correct current limit registers are
	 * used
	 */
	if (type != POWER_SUPPLY_TYPE_UNKNOWN)
		chip->usb_supply_type = type;

	if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_CDP)
		current_limit_ma = DEFAULT_CDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP
			|| type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		current_limit_ma = smbchg_default_hvdcp_icl_ma;
	else
		current_limit_ma = smbchg_default_dcp_icl_ma;

	pr_smb(PR_STATUS, "Type %d: setting mA = %d\n",
		type, current_limit_ma);
	rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit_ma);
	if (rc < 0) {
		pr_err("Couldn't vote for new USB ICL rc=%d\n", rc);
		goto out;
	}

	if (!chip->skip_usb_notification)
		power_supply_set_supply_type(chip->usb_psy, type);

	/*
	 * otherwise if it is unknown, remove vote
	 * and set type after the vote
	 */
	if (type == POWER_SUPPLY_TYPE_UNKNOWN) {
		rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, false,
				current_limit_ma);
		if (rc < 0)
			pr_err("Couldn't remove ICL vote rc=%d\n", rc);

		chip->usb_supply_type = type;
	}

	/* set the correct buck switching frequency */
	rc = smbchg_set_optimal_charging_mode(chip, type);
	if (rc < 0)
		pr_err("Couldn't set charger optimal mode rc=%d\n", rc);

out:
	return rc;
}

static bool is_hvdcp_present(struct  smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		pr_err("Couldn't read hvdcp status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	/*
	 * If a valid HVDCP is detected, notify it to the usb_psy only
	 * if USB is still present.
	 */
	if (chip->schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip))
		return true;

	return false;
}


#ifdef SUPPORT_9V_HVDCP
static int force_9v_hvdcp(struct smbchg_chip *chip)
{
	int rc;

	/* Force 5V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc) {
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		return rc;
	}

	/* Force QC2.0 */
	rc = smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, FORCE_HVDCP_2p0);
	rc |= smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, 0);
	if (rc < 0) {
		pr_err("Couldn't force QC2.0 rc=%d\n", rc);
		return rc;
	}

	/* wait for QC2.0 */
	msleep(500);

	/* Force 9V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc)
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	return rc;
}

static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				hvdcp_det_work.work);
	int rc;

	if (is_hvdcp_present(chip)) {
		if (!chip->hvdcp3_supported &&
			(chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
			/* force HVDCP 2.0 */
			rc = force_9v_hvdcp(chip);
			if (rc)
				pr_err("could not force 9V HVDCP continuing rc=%d\n",
						rc);
		}
		smbchg_change_usb_supply_type(chip,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
	}
	else
		hvdcp_flag = NO_HVDCP;
}
#endif
void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force);
static void  asus_smbchg_SDP_setting_worker(struct work_struct *work){
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, asus_sdp_delayed_work.work);
	int rc;
	pr_smb(PR_STATUS, "g_sdp_retry_flag = %d\n", g_sdp_retry_flag);
	if(g_sdp_retry_flag == 0){//first SDP in
		reinit_completion(&chip->sdp_retry_complete_flag);
		pr_smb(PR_STATUS, "reinit_completion sdp_retry_complete_flag\n");
		g_sdp_retry_flag = 1;
		/* rerun bc 1.2 in sdp retry*/
		pr_smb(PR_STATUS, "Allow only 9V only charger(%d)\n",g_sdp_retry_flag);
		// set pmi 13f1[2:0] = 0x3
		rc = smbchg_sec_masked_write(chip,chip->usb_chgpth_base + USBIN_CHGR_CFG,ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_9V);
		if (rc < 0) {
			pr_smb(PR_STATUS, "Couldn't write usb allowance rc=%d\n", rc);
		}
		rc = wait_for_completion_interruptible_timeout( &chip->sdp_retry_complete_flag,   msecs_to_jiffies(2000));
		msleep(400);
		pr_smb(PR_STATUS, "Allow only 5-9V charger\n");
		// set pmi 13f1[2:0] = 0x2
		rc = smbchg_sec_masked_write(chip,chip->usb_chgpth_base + USBIN_CHGR_CFG,ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_9V_CONT);
		if (rc < 0) {
			pr_smb(PR_STATUS, "Couldn't write usb allowance rc=%d\n", rc);
		}
		rc = wait_for_completion_interruptible_timeout( &chip->sdp_retry_complete_flag,   msecs_to_jiffies(2000));
		msleep(500);
		g_sdp_retry_flag = 2;
		if (!is_usb_present(chip)) {
			g_sdp_retry_flag = 0;
			pr_smb(PR_STATUS, "detect usb truly remove, force to update \n");
			update_usb_status(chip, 0, true);
		}
	}else{
		g_sdp_retry_flag = 0;
		smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, 0xFF, 0x6);
		if (!chip->skip_usb_notification) {
			pr_smb(PR_MISC, "setting usb psy present = %d\n",chip->usb_present);
			power_supply_set_present(chip->usb_psy, chip->usb_present);
		}

		if (dfp_type_flag == Others){
			pr_smb(PR_STATUS, "[SMBCHG] %s:USB mode(5V/0.5A).\n", __func__);
#if defined(CONFIG_ZT582KL)
			dual_chg_flag = Single;
			chg_type_flag = SDP_0P5A;
#elif defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
			dual_chg_flag = Single;
			chg_type_flag = SDP_0P5A;
#endif
			boost_up_voltage_done = true;
		}else{
			/*TYPC DFP setting*/
		}

		if (wake_lock_active(&alarm_cable_lock)){
			pr_err("[%s] Cable WakeLock: *UNLOCK*\n", __func__);
			wake_unlock(&alarm_cable_lock);
		}
	}
	return;
}
static int set_usb_psy_dp_dm(struct smbchg_chip *chip, int state)
{
	int rc;
	u8 reg;

	/*
	 * ensure that we are not in the middle of an insertion where usbin_uv
	 * is low and src_detect hasnt gone high. If so force dp=F dm=F
	 * which guarantees proper type detection
	 */
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (!rc && !(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "overwriting state = %d with %d\n",
				state, POWER_SUPPLY_DP_DM_DPF_DMF);
		state = POWER_SUPPLY_DP_DM_DPF_DMF;
	}
	pr_smb(PR_MISC, "setting usb psy dp dm = %d\n", state);
	return power_supply_set_dp_dm(chip->usb_psy, state);
}

#define APSD_CFG		0xF5
#define AUTO_SRC_DETECT_EN_BIT	BIT(0)
#define APSD_TIMEOUT_MS		1500
static void restore_from_hvdcp_detection(struct smbchg_chip *chip)
{
	int rc;

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);
#ifdef SUPPORT_9V_HVDCP
	/* switch to 9V HVDCP */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0)
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
#endif
	/* enable HVDCP */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);

	/* enable APSD */
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);
#ifdef SUPPORT_9V_HVDCP
	/* allow 5 to 9V chargers */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_9V_CONT);
	if (rc < 0)
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
#endif
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable AICL rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	chip->pulse_cnt = 0;

#ifdef SUPPORT_9V_HVDCP
	if ((chip->schg_version == QPNP_SCHG_LITE)
				&& is_hvdcp_present(chip)) {
		pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
		rc = force_9v_hvdcp(chip);
		if (rc)
			pr_err("Failed to force 9V HVDCP=%d\n",	rc);
	}
#endif

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);
}

#define RESTRICTED_CHG_FCC_PERCENT	50
static int smbchg_restricted_charging(struct smbchg_chip *chip, bool enable)
{
	int current_table_index, fastchg_current;
	int rc = 0;

	/* If enable, set the fcc to the set point closest
	 * to 50% of the configured fcc while remaining below it
	 */
	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			chip->cfg_fastchg_current_ma
				* RESTRICTED_CHG_FCC_PERCENT / 100,
			chip->tables.usb_ilim_ma_len);
	fastchg_current =
		chip->tables.usb_ilim_ma_table[current_table_index];
	rc = vote(chip->fcc_votable, RESTRICTED_CHG_FCC_VOTER, enable,
			fastchg_current);

	pr_smb(PR_STATUS, "restricted_charging set to %d\n", enable);
	chip->restricted_charging = enable;

	return rc;
}

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	union power_supply_propval ret = {0, };

	if(!g_probe){
		ret.intval = 0;
		chip->batt_psy.set_property(&chip->batt_psy,POWER_SUPPLY_PROP_ALLOW_HVDCP3, &ret);
	}
	if (is_usb_present(chip)) {
		pr_smb(PR_STATUS, "VBUS exist so ignore!!!\n");
		return;
	}
	cancel_delayed_work(&chip->pule_down_5v_work);
#endif
	cancel_delayed_work(&chip->usb_insertion_work);
	cancel_delayed_work(&chip->adapter_detect_work);
	cancel_delayed_work(&chip->cable_poweron_work);
	cancel_delayed_work(&chip->asus_sdp_delayed_work);
	pr_smb(PR_STATUS, "triggered, g_sdp_retry_flag = %d\n", g_sdp_retry_flag);
#if defined(CONFIG_ZT582KL)
	gt6108_cable_status_handler(0);
#endif	
	
	chip->charging_type = CHG_TYPE_UNDEFINED;
	dual_chg_flag = Undefined;
	chip->cable_power_on = false;

	chip->stop_chg_via_temp_cold = false;
	chip->stop_chg_via_temp_cool = false;
	chip->stop_chg_via_temp_warm = false;
	chip->stop_chg_via_temp_hot = false;
	chip->dedicated_adapter = false;
	chip->dedicated_power_bank = false;

	boost_up_voltage_done = false;
	is_cc_logic = false;
	if (wake_lock_active(&alarm_cable_lock)){
		pr_err("Cable WakeLock: *UNLOCK*\n");
		wake_unlock(&alarm_cable_lock);
	}
		
	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->force_aicl_rerun && !chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Error enabling AICL rerun rc= %d\n",
				rc);
	}
	/* Clear the OV detected status set before */
	if (chip->usb_ov_det)
		chip->usb_ov_det = false;
#ifdef SUPPORT_9V_HVDCP
	/* cancel/wait for hvdcp pending work if any */
	cancel_delayed_work_sync(&chip->hvdcp_det_work);
#endif
	if(g_sdp_retry_flag == 1){
		pr_smb(PR_STATUS, "SDP retry not setting usb psy as UNKNOWN\n");
	}else{
		smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_UNKNOWN);
		if (!chip->skip_usb_notification) {
			pr_smb(PR_STATUS, "setting usb psy present = %d\n",
					chip->usb_present);
			power_supply_set_present(chip->usb_psy, chip->usb_present);
		}
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPR_DMR);

	if(g_sdp_retry_flag == 1){
		pr_smb(PR_STATUS, "SDP retry not setting usb psy health UNKNOWN\n");
	}else{
		schedule_work(&chip->usb_set_online_work);
		pr_smb(PR_STATUS, "setting usb psy health UNKNOWN\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNKNOWN);
		if (rc < 0)
			pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_UNKNOWN, rc);
	}

	if (parallel_psy && chip->parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->parallel.avail && chip->aicl_done_irq
			&& chip->enable_aicl_wake) {
		disable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = false;
	}
	chip->parallel.enabled_once = false;
	chip->vbat_above_headroom = false;
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't set override rc = %d\n", rc);

	vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER, false, 0);
	chip->usb_icl_delta = 0;
	vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, false, 0);
	restore_from_hvdcp_detection(chip);

	complete_all(&chip->sdp_retry_complete_flag);
	pr_smb(PR_STATUS, "complete_all sdp_retry_complete_flag\n");
	
#ifdef ENG_BUILD
		set_adapter_type(ASUS_NONE);
#endif
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

static bool is_usbin_uv_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_UV_BIT;
}

extern int us5587_enable(bool bEnable);
extern int ADCPWR_enable(bool bEnable);
extern int DPM_SW_enable(bool bEnable);
extern unsigned char us5587_reg_val(int Reg);

static int detect_dedicated_adapter(struct smbchg_chip *chip)
{
	u8 reg;
	int rc;	
	unsigned char val = 0;
	
	ADCPWR_enable(false);
	DPM_SW_enable(true);
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	msleep(100);  //delay 0.1s or 30s
#else
	msleep(30000);
#endif
/*
	smbchg_stay_awake(chip, PM_OEM_DCP_TYPE_DETECT);
	i = 0;
	do{
		i++;
		msleep(500);

		//pr_smb(PR_STATUS, "Waitting ADC stable ...... (%d), cable power on status is %d\n", i, chip->cable_power_on);
		smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
		if(reg == 0x00)
		{
			pr_smb(PR_INTERRUPT, "charger bunk disable or loss : 0x%02x\n", reg);

			// Charger bunk has disabled or loss, so turn off ADC switch and power
			DPM_SW_enable(false);
			ADCPWR_enable(false);
			return rc;
		}		
	}while((i < 30) && (chip->cable_power_on == false));

	smbchg_relax(chip, PM_OEM_DCP_TYPE_DETECT);
#endif
*/
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_SUSPEND_BIT, USBIN_SUSPEND_BIT);
	msleep(5);
		
	val = us5587_reg_val(0x04);
	pr_smb(PR_STATUS, "First read Adapter ADC channel value is 0x%x\n", val);
	smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);

	if(val <= 0x40 && !(reg & 0x20)) // firs adc value is ADC_0.6V = 0x40
	{				
		ADCPWR_enable(true);
		msleep(5);
			
		val = us5587_reg_val(0x04);
		pr_smb(PR_STATUS, "Second read Adapter ADC channel value is 0x%x\n", val);

		if(val > 0xd4) // second adc value > 2.0V
		{
			pr_smb(PR_STATUS, "The Adapter is non-legal !\n");
			chip->dedicated_adapter = false;
			chg_type_flag = UNDEFINED;
			dual_chg_flag = Single;
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
#ifdef ENG_BUILD
			set_adapter_type(ASUS_NONE);
#endif
		}
		else  // second adc value <= 2.0 V
		{
			/* 5V/2A adapter(750K) ADC value range is 0xb3 ~ 0xc9 <-- ZT581KL_ER2 & ZT581KL_PR
			* 5V/2A adapter(750K) ADC value range is 0x9e ~ 0xbc <-- ZT581KL_MP & Z581KL_ALL & ZT500KL_ALL
			* 9V/2A QC adapter(200K) ADC value range is 0x51 ~ 0x67 <-- ZT581KL_ER2 & ZT581KL_PR
			* 9V/2A QC adapter(200K) ADC value range is 0x3f ~ 0x5d <-- ZT581KL_MP & Z581KL_ALL & ZT500KL_ALL
			*/
#if defined(CONFIG_ZT582KL)
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
#endif
			if((((val >= 0xb3) && (val <= 0xc9)) && (chip->proj_id == PROJ_ID_ZT581KL) && (chip->hw_id <= HW_ID_PR))
				|| ((val >= 0x9e) && (val <= 0xbc)))
			{
				pr_smb(PR_STATUS, "The Adapter is legal, ASUS_750K !\n");
				chip->dedicated_adapter = true;
				if (hvdcp_flag == NO_HVDCP){
					chg_type_flag = DCP_ASUS_750K_2A;
#if defined(CONFIG_ZT582KL)
					dual_chg_flag = ASUS_10W;
#else
					dual_chg_flag = ASUS_2A;
#endif
				}
				else{
					chg_type_flag = HVDCP_OTHERS_1A;
					dual_chg_flag = Single;
				}
#ifdef SUPPORT_9V_HVDCP
#ifndef ENG_BUILD  // Eng build is not support Quick charging icon
				chip->charging_type = CHG_TYPE_DCP_QC_2; // ASUS_750K(5V/2A) does not show QC icon "+"
#endif
#endif

#ifdef ENG_BUILD
				set_adapter_type(ASUS_750K);
#endif
			}
			else if((((val >= 0x51) && (val <= 0x67)) && (chip->proj_id == PROJ_ID_ZT581KL) && (chip->hw_id <= HW_ID_PR))
				|| ((val >= 0x3f) && (val <= 0x5d)))
			{
				pr_smb(PR_STATUS, "The Adapter is legal, ASUS_200K !\n");
				chip->dedicated_adapter = true;
				if (hvdcp_flag == HVDCP_2){
					chg_type_flag = HVDCP_OTHERS_1A;
					dual_chg_flag = ASUS_2A;
				}
				else if ( hvdcp_flag == HVDCP_3){
					chg_type_flag = HVDCP_ASUS_200K_2A;
					dual_chg_flag = ASUS_2A;
				}
				else{
#if defined(CONFIG_ZT582KL)
					dual_chg_flag = ASUS_18W;
#else
					dual_chg_flag = Single;
#endif
					chg_type_flag = UNDEFINED;
				}

#ifdef SUPPORT_9V_HVDCP
#ifndef ENG_BUILD  // Eng build is not support Quick charging icon
				chip->charging_type = CHG_TYPE_DCP_QC_3;
#endif
#endif

#ifdef ENG_BUILD
				set_adapter_type(ASUS_200K);
#endif
			}
			else
			{
				pr_smb(PR_STATUS, "The Adapter is non-legal !\n");
				rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
				chip->dedicated_adapter = false;
				if (hvdcp_flag == NO_HVDCP){
					chg_type_flag = UNDEFINED;
					dual_chg_flag = Single;
				}
				else{
					chg_type_flag = HVDCP_OTHERS_1A;
					dual_chg_flag = Single;
				}
#ifdef ENG_BUILD
				set_adapter_type(ASUS_NONE);
#endif
			}
		}
	}
	else // first adc value > 0.6V
	{
		pr_smb(PR_STATUS, "The Adapter is non-legal (others Adapter) !\n");
		chip->dedicated_adapter = false;
#ifdef ENG_BUILD
		set_adapter_type(ASUS_NONE);	
#endif
		if(val > 0xc0) // first adc value is ADC_1.8V = 0xc0
		{
			pr_smb(PR_STATUS, "The Power Source is legal Power Bank !\n");
			chip->dedicated_power_bank = true;
#if defined(CONFIG_ZT582KL)
			gt6108_cable_status_handler(3);
#endif
			if (hvdcp_flag == NO_HVDCP){
				chg_type_flag = DCP_PB_2A;
#if defined(CONFIG_ZT582KL)
				dual_chg_flag = ASUS_10W;
#else
				dual_chg_flag = ASUS_2A;
#endif
			}
			else{
				chg_type_flag = HVDCP_PB_1A;
				dual_chg_flag = Single;
			}
		}
		else{
			chip->dedicated_power_bank = false;
			chg_type_flag = UNDEFINED;
			dual_chg_flag = Single;
		}
	}

#ifdef ENG_BUILD
	pr_smb(PR_STATUS, "This is eng build, so don't care the adc value !\n");
	chip->dedicated_adapter = true;
#endif
	boost_up_voltage_done = true;
#if defined(CONFIG_ZT582KL)
	if(chip->dedicated_power_bank)
	{
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
	}
	else
	{
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
	}
#elif defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL) 
	if(chip->dedicated_adapter || chip->dedicated_power_bank)
	{
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
	}
	else
	{
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
	}
#endif
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USBIN_SUSPEND_BIT, 0);
	DPM_SW_enable(false);
	ADCPWR_enable(false);
	//us5587_enable(false);
#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL) || defined(CONFIG_ZT582KL)
	//msleep(2000);
	
	smbchg_rerun_aicl(chip);
	
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
	else
	{
		if(reg & SCHG_LITE_USBIN_HVDCP_SEL_BIT)
		{
			//msleep(2000);
			msleep(20);
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
						HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
		}
		else
			pr_smb(PR_STATUS, "HVDCP Not Detected !\n");
	}
#endif	
	return rc;
}

#define HVDCP_NOTIFY_MS		2500
int support_cnt = 0;
static void handle_usb_insertion(struct smbchg_chip *chip)
{
#if !defined(CONFIG_ZT582KL)
	struct power_supply *parallel_psy = get_parallel_psy(chip);
#endif
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";
//	u8 reg;
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	union power_supply_propval ret = {1, };
	if(!g_probe){
		ret.intval = 1;
		chip->batt_psy.set_property(&chip->batt_psy,POWER_SUPPLY_PROP_ALLOW_HVDCP3, &ret);
	}
	if (!chip->hvdcp3_supported && support_cnt < 20){
		queue_delayed_work(adapter_wq ,&chip_fg->usb_insertion_work, msecs_to_jiffies(1000));
		support_cnt ++;
		return;		
	}
	g_cable_poweron = false;
	complete_all(&chip->sdp_retry_complete_flag);
	pr_smb(PR_STATUS, "complete_all sdp_retry_complete_flag\n");
#endif
	
	pr_smb(PR_STATUS, "triggered\n");

	/* usb inserted */
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_smb(PR_STATUS, "inserted type = %d (%s)", usb_supply_type, usb_type_name);

	smbchg_aicl_deglitch_wa_check(chip);
	smbchg_change_usb_supply_type(chip, usb_supply_type);
	
	/*Add by frank +++*/
	if ((usb_supply_type == POWER_SUPPLY_TYPE_USB) && (g_sdp_retry_flag == 0))  {
		pr_smb(PR_STATUS, "ignore first usb present event when SDP first\n");
	}else if (usb_supply_type != POWER_SUPPLY_TYPE_USB){
		if (!chip->skip_usb_notification) {
			pr_smb(PR_STATUS, "setting usb psy present = %d\n", chip->usb_present);
			power_supply_set_present(chip->usb_psy, chip->usb_present);
		}
	}
	/*Add by frank ---*/

	/* Notify the USB psy if OV condition is not present */
	if (!chip->usb_ov_det) {
		/*
		* Note that this could still be a very weak charger
		* if the handle_usb_insertion was triggered from
		* the falling edge of an USBIN_OV interrupt
		*/
		pr_smb(PR_MISC, "setting usb psy health %s\n",
						chip->very_weak_charger
						? "UNSPEC_FAILURE" : "GOOD");
		rc = power_supply_set_health_state(chip->usb_psy,
						chip->very_weak_charger
						? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
						: POWER_SUPPLY_HEALTH_GOOD);
		if (rc < 0)
			pr_smb(PR_STATUS, "usb psy does not allow updating prop %d rc = %d\n", POWER_SUPPLY_HEALTH_GOOD, rc);
	}
	schedule_work(&chip->usb_set_online_work);
#ifdef SUPPORT_PD_ADAPTER
	if((usb_supply_type == POWER_SUPPLY_TYPE_USB) && (chip->charging_type != CHG_TYPE_DCP_TYPC_3))
#else
	if(usb_supply_type == POWER_SUPPLY_TYPE_USB)
#endif
	{
		atomic_set(&AC_charging_alarm,0);
		if (g_sdp_retry_flag == 0)
			queue_delayed_work(chip->smbchg_work_queue, &chip->asus_sdp_delayed_work, msecs_to_jiffies(500));
		else
			queue_delayed_work(chip->smbchg_work_queue, &chip->asus_sdp_delayed_work, msecs_to_jiffies(0));
	}
	else if(usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP){
		boost_up_voltage_done = true;
		if (g_sdp_retry_flag != 0) {
			pr_smb(PR_STATUS, "CDP type after SDP retry\n");
			g_sdp_retry_flag = 0;
		}
		smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USB_CMD_IL_MASK, USB_CMD_IL_CONFIG);
		smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
		if (dfp_type_flag == Others){
			smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1400MA);
			smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
			smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
			chg_type_flag = CDP_1P5A;
			dual_chg_flag = Single;
		}
		else{
			/*TYPC DFP setting*/
		}
		if (wake_lock_active(&alarm_cable_lock)){
			pr_err("[%s] Cable WakeLock: *UNLOCK*\n", __func__);
			wake_unlock(&alarm_cable_lock);
		}
	}
	else if (usb_supply_type == POWER_SUPPLY_TYPE_UNKNOWN){
		boost_up_voltage_done = true;
		smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, USB_CMD_IL_MASK, USB_CMD_IL_CONFIG);
		smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
		if (dfp_type_flag == Others){
			chg_type_flag = OTHERS_1A;
			dual_chg_flag = Single;
		}
		else{
			/*TYPC DFP setting*/
		}
		if (wake_lock_active(&alarm_cable_lock)){
			pr_err("[%s] Cable WakeLock: *UNLOCK*\n", __func__);
			wake_unlock(&alarm_cable_lock);
		}
	}
#if !defined(CONFIG_ZT582KL)
#ifdef SUPPORT_9V_HVDCP
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		schedule_delayed_work(&chip->hvdcp_det_work, msecs_to_jiffies(HVDCP_NOTIFY_MS));
#endif

	if (parallel_psy) {
		rc = power_supply_set_present(parallel_psy, true);
		chip->parallel_charger_detected = rc ? false : true;
		if (rc)
			pr_debug("parallel-charger absent rc=%d\n", rc);
	}

	if (chip->parallel.avail && chip->aicl_done_irq
			&& !chip->enable_aicl_wake) {
		rc = enable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = true;
	}
#endif
	if(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP){
		g_dcp_insert = true;
		atomic_set(&AC_charging_alarm,0);
		if (g_sdp_retry_flag !=0 ) {
			pr_smb(PR_STATUS, "DCP type after SDP retry\n");
			g_sdp_retry_flag = 0;
		}
#if defined(CONFIG_ZT582KL)
		gt6108_cable_status_handler(1);
		if (dfp_type_flag == TYPEC_PD){
			/*TYPC DFP setting*/
		}
		else{
			queue_delayed_work(adapter_wq ,&chip->adapter_detect_work, msecs_to_jiffies(100));
		}
#else
		queue_delayed_work(adapter_wq ,&chip->adapter_detect_work, msecs_to_jiffies(10000));
#endif
	}
}


// Declare the smbchg_usbin_init_settings() function before using it
#ifdef SUPPORT_PARALLEL_CHG
static int parallel_chg_insert_init_settings(void);
#endif
static int smbchg_usbin_init_settings(struct smbchg_chip *chip);

static void smbchg_usb_insertion_work(struct work_struct *work){
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, usb_insertion_work.work);
	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);

	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, src_detect,
		chip->hvdcp_3_det_ignore_uv);

#if 0 //!defined(CONFIG_ZT582KL)
	if (chip->usb_present == usb_present){
		if (!g_cable_poweron){
			pr_smb(PR_STATUS,"goto out!\n");
			goto out;
		}
	}
#endif

	if (src_detect){
		complete_all(&chip->src_det_raised);
#if defined(VZW)
		get_capacity_remapping_in();
#endif
	}
	else{
		complete_all(&chip->src_det_lowered);
#if defined(VZW)
		get_capacity_remapping_out();
#endif
	}

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	/*
	 * When VBAT is above the AICL threshold (4.25V) - 180mV (4.07V),
	 * an input collapse due to AICL will actually cause an USBIN_UV
	 * interrupt to fire as well.
	 *
	 * Handle USB insertions and removals in the source detect handler
	 * instead of the USBIN_UV handler since the latter is untrustworthy
	 * when the battery voltage is high.
	 */
	chip->very_weak_charger = false;
//	rc = vote(chip->usb_suspend_votable, WEAK_CHARGER_EN_VOTER, false, 0);
//	if (rc < 0)
//		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		smbchg_usbin_init_settings(chip);
#ifdef SUPPORT_PARALLEL_CHG
		parallel_chg_insert_init_settings();
#endif
		if (g_cable_poweron){
			update_usb_status(chip, usb_present, 1);
		}
		else if (g_sdp_retry_flag){
			pr_smb(PR_STATUS, "triggered\n");
			update_usb_status(chip, 1, true);
		}
		else
			update_usb_status(chip, usb_present, 0);	
	} else {
		update_usb_status(chip, 0, false);
		chip->aicl_irq_count = 0;
	}

out:
	return;
//	handle_usb_insertion(chip);
}
static int smbchg_prepare_for_pulsing_lite(struct smbchg_chip *chip);
static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion);
static void smbchg_cable_poweron_work(struct work_struct *work){
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, cable_poweron_work.work);
	int rc;

//	pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");

	g_cable_poweron = true;
	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 1);
	if (rc < 0) {
		pr_err("Couldn't Enable HVDCP rc=%d\n", rc);
	}

	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	handle_usb_removal(chip);
	
	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	queue_delayed_work(adapter_wq ,&chip->usb_insertion_work, msecs_to_jiffies(0));	
}

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}

	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
	mutex_unlock(&chip->usb_status_lock);
}

static int otg_oc_reset(struct smbchg_chip *chip)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);

	/*
	 * There is a possibility that an USBID interrupt might have
	 * occurred notifying USB power supply to disable OTG. We
	 * should not enable OTG in such cases.
	 */
	if (!is_otg_present(chip)) {
		pr_smb(PR_STATUS,
			"OTG is not present, not enabling OTG_EN_BIT\n");
		goto out;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

out:
	return rc;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
static void increment_aicl_count(struct smbchg_chip *chip)
{
	bool bad_charger = false;
	int max_aicl_count, rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;

	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds);

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->aicl_complete = reg & AICL_STS_BIT;
	else
		chip->aicl_complete = false;

	if (chip->aicl_deglitch_short || chip->force_aicl_rerun) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		/*
		 * Double the amount of AICLs allowed if parallel charging is
		 * enabled.
		 */
		max_aicl_count = AICL_IRQ_LIMIT_COUNT
			* (chip->parallel.avail ? 2 : 1);
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > max_aicl_count) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			chip->very_weak_charger = true;
			bad_charger = true;

			/*
			 * Disable AICL rerun since many interrupts were
			 * triggered in a short time
			 */
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("Couldn't disable AICL reruns rc=%d\n",
					rc);

			/* Vote 100mA current limit */
			rc = vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER,
					true, CURRENT_100_MA);
			if (rc < 0) {
				pr_err("Can't vote %d current limit rc=%d\n",
					CURRENT_100_MA, rc);
			}

			chip->aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			/*
			 * If the AICL_SUSP_BIT is on, then AICL reruns have
			 * already been disabled. Set the very weak charger
			 * flag so that the driver reports a bad charger
			 * and does not reenable AICL reruns.
			 */
			chip->very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			pr_smb(PR_MISC,
				"setting usb psy health UNSPEC_FAILURE\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
			schedule_work(&chip->usb_set_online_work);
		}
	}
}

static int wait_for_usbin_uv(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->usbin_uv_lowered;
	bool usbin_uv;

	if (high)
		completion = &chip->usbin_uv_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	usbin_uv = is_usbin_uv_high(chip);

	if (high == usbin_uv)
		return 0;

	pr_err("usbin uv didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			usbin_uv ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int wait_for_src_detect(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->src_det_lowered;
	bool src_detect;

	if (high)
		completion = &chip->src_det_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	src_detect = is_src_detect_high(chip);

	if (high == src_detect)
		return 0;

	pr_err("src detect didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			src_detect ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion)
{
	int rc;
	bool src_detect;
	bool usbin_uv;

	if (insertion) {
		INIT_COMPLETION(chip->src_det_raised);
		INIT_COMPLETION(chip->usbin_uv_lowered);
	} else {
		INIT_COMPLETION(chip->src_det_lowered);
		INIT_COMPLETION(chip->usbin_uv_raised);
	}

	/* ensure that usbin uv real time status is in the right state */
	usbin_uv = is_usbin_uv_high(chip);
	if (usbin_uv != insertion) {
		pr_err("Skip faking, usbin uv is already %d\n", usbin_uv);
		return -EINVAL;
	}

	/* ensure that src_detect real time status is in the right state */
	src_detect = is_src_detect_high(chip);
	if (src_detect == insertion) {
		pr_err("Skip faking, src detect is already %d\n", src_detect);
		return -EINVAL;
	}

	pr_smb(PR_MISC, "Allow only %s charger\n",
			insertion ? "5-9V" : "9V only");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK,
			insertion ?
			USBIN_ADAPTER_5V_9V_CONT : USBIN_ADAPTER_9V);
	if (rc < 0) {
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s usbin uv\n",
			insertion ? "falling" : "rising");
	rc = wait_for_usbin_uv(chip, !insertion);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s src det\n",
			insertion ? "rising" : "falling");
	rc = wait_for_src_detect(chip, insertion);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static void smbchg_handle_hvdcp3_disable(struct smbchg_chip *chip)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "NULL";

	chip->pulse_cnt = 0;

	if (is_hvdcp_present(chip)) {
		smbchg_change_usb_supply_type(chip,POWER_SUPPLY_TYPE_USB_HVDCP);
	} else {
		read_usb_type(chip, &usb_type_name, &usb_supply_type);
		smbchg_change_usb_supply_type(chip, usb_supply_type);
#ifdef SUPPORT_9V_HVDCP
		if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
			schedule_delayed_work(&chip->hvdcp_det_work,msecs_to_jiffies(HVDCP_NOTIFY_MS));
#endif
	}
}

static int smbchg_prepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 reg;

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;
	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
		goto handle_removal;
	}

	/* disable APSD */
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
		goto out;
	}

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto handle_removal;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DMF);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if ((reg >> TYPE_BITS_OFFSET) != 0) {
		pr_smb(PR_MISC, "type bits set after 2s sleep - abort\n");
		rc = -EINVAL;
		goto out;
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DM3P3);
	/* Wait 60mS after entering continuous mode */
	msleep(60);

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
handle_removal:
	chip->hvdcp_3_det_ignore_uv = false;
	update_usb_status(chip, 0, 0);

	/* This could be because allow_hvdcp3 set to false runtime */
	if (is_usb_present(chip) && !chip->allow_hvdcp3_detection)
		smbchg_handle_hvdcp3_disable(chip);

	return rc;
}

static int smbchg_unprepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPF_DMF);
#ifdef SUPPORT_9V_HVDCP
	/* switch to 9V HVDCP */
	pr_smb(PR_MISC, "Switch to 9V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
		return rc;
	}
#endif
	/* enable HVDCP */
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	/* enable APSD */
	pr_smb(PR_MISC, "Enabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable APSD rc=%d\n", rc);
		return rc;
	}

	/* Disable AICL */
	pr_smb(PR_MISC, "Disable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable AICL rc=%d\n", rc);
		return rc;
	}

	/* fake a removal */
	chip->hvdcp_3_det_ignore_uv = true;
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal rc=%d\n", rc);
		goto out;
	}

	/*
	 * reset the enabled once flag for parallel charging so
	 * parallel charging can immediately restart after the HVDCP pulsing
	 * is complete
	 */
	chip->parallel.enabled_once = false;

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto out;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	/* Enable AICL */
	pr_smb(PR_MISC, "Enable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable AICL rc=%d\n", rc);
		return rc;
	}

out:
	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}

	/* This could be because allow_hvdcp3 set to false runtime */
	if (is_usb_present(chip) && !chip->allow_hvdcp3_detection)
		 smbchg_handle_hvdcp3_disable(chip);

	return rc;
}

#define USB_CMD_APSD		0x41
#define APSD_RERUN		BIT(0)
static int rerun_apsd(struct smbchg_chip *chip)
{
	int rc;

	INIT_COMPLETION(chip->src_det_raised);
	INIT_COMPLETION(chip->usbin_uv_lowered);
	INIT_COMPLETION(chip->src_det_lowered);
	INIT_COMPLETION(chip->usbin_uv_raised);

	/* re-run APSD */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + USB_CMD_APSD,
					APSD_RERUN, APSD_RERUN);
	if (rc) {
		pr_err("Couldn't re-run APSD rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising usbin uv\n");
	rc = wait_for_usbin_uv(chip, true);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling src det\n");
	rc = wait_for_src_detect(chip, false);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling usbin uv\n");
	rc = wait_for_usbin_uv(chip, false);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising src det\n");
	rc = wait_for_src_detect(chip, true);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#define SCHG_LITE_USBIN_HVDCP_5_9V		0x8
#define SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK	0x38
#define SCHG_LITE_USBIN_HVDCP_SEL_IDLE		BIT(3)
static bool is_hvdcp_5v_cont_mode(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
		chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc) {
		pr_err("Unable to read HVDCP status rc=%d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP status = %x\n", reg);

	if (reg & SCHG_LITE_USBIN_HVDCP_SEL_IDLE) {
		rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + INPUT_STS, 1);
		if (rc) {
			pr_err("Unable to read INPUT status rc=%d\n", rc);
			return false;
		}
		pr_smb(PR_STATUS, "INPUT status = %x\n", reg);
		if ((reg & SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK) ==
					SCHG_LITE_USBIN_HVDCP_5_9V)
			return true;
	}
	return false;
}

static int smbchg_prepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	/* check if HVDCP is already in 5V continuous mode */
	if (is_hvdcp_5v_cont_mode(chip)) {
		pr_smb(PR_MISC, "HVDCP by default is in 5V continuous mode\n");
		return 0;
	}

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;

	/* re-run APSD */
	rc = rerun_apsd(chip);
	if (rc) {
		pr_err("APSD rerun failed\n");
		goto out;
	}

	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	/* We are set if HVDCP in 5V continuous mode */
	if (!is_hvdcp_5v_cont_mode(chip)) {
		pr_err("HVDCP could not be set in 5V continuous mode\n");
		goto out;
	}

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed - force removal\n");
		update_usb_status(chip, 0, true);
	}
	return rc;
}

static int smbchg_unprepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;
#ifdef SUPPORT_9V_HVDCP
	pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
	rc = force_9v_hvdcp(chip);
	if (rc) {
		pr_err("Failed to force 9V HVDCP=%d\n",	rc);
		return rc;
	}
#endif
	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}
	/* This could be because allow_hvdcp3 set to false runtime */
	if (is_usb_present(chip) && !chip->allow_hvdcp3_detection)
		smbchg_handle_hvdcp3_disable(chip);

	return rc;
}

#define CMD_HVDCP_2		0x43
#define SINGLE_INCREMENT	BIT(0)
#define SINGLE_DECREMENT	BIT(1)
static int smbchg_dp_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Increment DP\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_INCREMENT, SINGLE_INCREMENT);
	if (rc)
		pr_err("Single-increment failed rc=%d\n", rc);

	return rc;
}

static int smbchg_dm_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Decrement DM\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_DECREMENT, SINGLE_DECREMENT);
	if (rc)
		pr_err("Single-decrement failed rc=%d\n", rc);

	return rc;
}

static int smbchg_hvdcp3_confirmed(struct smbchg_chip *chip)
{
	int rc = 0;

	/*
	 * reset the enabled once flag for parallel charging because this is
	 * effectively a new insertion.
	 */
	chip->parallel.enabled_once = false;

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_USB_HVDCP_3);

	return rc;
}

#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
static void smbchg_pule_down_5v_work(struct work_struct *work){
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, pule_down_5v_work.work);
	int i;

	smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_PREPARE);
	for(i = chip->pulse_cnt; i > 0; i--){
		smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_DM_PULSE);
		msleep(50);
	}
	smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
}
#endif

static int smbchg_dp_dm(struct smbchg_chip *chip, int val)
{
	int rc = 0;
	int target_icl_vote_ma;

	switch (val) {
	case POWER_SUPPLY_DP_DM_PREPARE:
		if (!is_hvdcp_present(chip)) {
			pr_err("No pulsing unless HVDCP\n");
			return -ENODEV;
		}
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_prepare_for_pulsing_lite(chip);
		else
			rc = smbchg_prepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_UNPREPARE:
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3:
		rc = smbchg_hvdcp3_confirmed(chip);
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
		if (g_dcp_insert == true ){
			queue_delayed_work(adapter_wq ,&chip->pule_down_5v_work, msecs_to_jiffies(2000));
			g_dcp_insert = false;
		}
#endif
		break;
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pulse_cnt++;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pulse_cnt)
			chip->pulse_cnt--;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_HVDCP3_SUPPORTED:
		chip->hvdcp3_supported = true;
		pr_smb(PR_MISC, "HVDCP3 supported\n");
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chip->usb_icl_delta -= 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
		chip->usb_icl_delta += 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	default:
		break;
	}

	return rc;
}

#define CHARGE_OUTPUT_VTG_RATIO		840
static int smbchg_get_iusb(struct smbchg_chip *chip)
{
	int rc, iusb_ua = -EINVAL;
	struct qpnp_vadc_result adc_result;

	if (!is_usb_present(chip) && !is_dc_present(chip))
		return 0;

	if (chip->vchg_vadc_dev && chip->vchg_adc_channel != -EINVAL) {
		rc = qpnp_vadc_read(chip->vchg_vadc_dev,
				chip->vchg_adc_channel, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS,
				"error in VCHG (channel-%d) read rc = %d\n",
						chip->vchg_adc_channel, rc);
			return 0;
		}
		iusb_ua = div_s64(adc_result.physical * 1000,
						CHARGE_OUTPUT_VTG_RATIO);
	}

	return iusb_ua;
}

static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_FLASH_TRIGGER,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_RESTRICTED_CHARGING,
	POWER_SUPPLY_PROP_ALLOW_HVDCP3,
};

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote(chip->battchg_suspend_votable, BATTCHG_USER_EN_VOTER,
				!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->usb_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		rc = vote(chip->dc_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		chip->flash_triggered = !!val->intval;
		smbchg_icl_loop_disable_check(chip);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smbchg_dp_dm(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		smbchg_rerun_aicl(chip);
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = smbchg_restricted_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_ALLOW_HVDCP3:
		if (chip->allow_hvdcp3_detection != val->intval) {
			chip->allow_hvdcp3_detection = !!val->intval;
			power_supply_changed(&chip->batt_psy);
		}
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
	case POWER_SUPPLY_PROP_ALLOW_HVDCP3:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval =
			!get_effective_result(chip->battchg_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->aicl_complete;
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		val->intval = (int)chip->restricted_charging;
		break;
	/* properties from fg */
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip) / 1000;  // change current_now unit from uA to mA
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		val->intval = chip->flash_triggered;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = smbchg_is_input_current_limited(chip);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = smbchg_get_iusb(chip);
		break;
	case POWER_SUPPLY_PROP_ALLOW_HVDCP3:
		val->intval = chip->allow_hvdcp3_detection;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->dc_suspend_votable, POWER_SUPPLY_EN_VOTER,
					!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = vote(chip->dc_icl_votable, USER_ICL_VOTER, true,
				val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !get_effective_result(chip->dc_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* return if dc is charging the battery */
		val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
					== POWER_SUPPLY_STATUS_CHARGING);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->dc_max_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

#define CHG_COMP_SFT_BIT	BIT(3)
static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc = 0;
	u8 reg;

	pr_smb(PR_INTERRUPT, "chg-error triggered\n");

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
	} else {
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
		if (reg & CHG_COMP_SFT_BIT)
			set_property_on_fg(chip,
					POWER_SUPPLY_PROP_SAFETY_TIMER_EXPIRED,
					1);
	}

#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "p2f triggered\n");
	smbchg_detect_parallel_charger(chip);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg = 0;
	bool terminated = false;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "Error reading RT_STS rc= %d\n", rc);
	} else {
		terminated = !!(reg & BAT_TCC_REACHED_BIT);
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	}
	/*
	 * If charging has not actually terminated, then this means that
	 * either this is a manual call to chg_term_handler during
	 * determine_initial_status(), or the charger has instantly restarted
	 * charging.
	 *
	 * In either case, do not do the usual status updates here. If there
	 * is something that needs to be updated, the recharge handler will
	 * handle it.
	 */
	if (terminated) {
#if 0 //remove qualcomm parallel charge
		smbchg_parallel_usb_check_ok(chip);
#endif
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_charging_status_change(chip);
		set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
	}
	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#if 0 //remove qualcomm parallel charge
	smbchg_parallel_usb_check_ok(chip);
#endif
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t wdog_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("wdog timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;
	
	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);

	return IRQ_HANDLED;
}

/**
 * dcin_uv_handler() - called when the dc voltage crosses the uv threshold
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating whether dc voltage is uv
 */
#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);
	int rc;

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		/* dc changed */
		chip->dc_present = dc_present;
		if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		if (chip->force_aicl_rerun && !dc_present) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc)
				pr_err("Error enabling AICL rerun rc= %d\n",
					rc);
		}
		chip->vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

/**
 * usbin_ov_handler() - this is called when an overvoltage condition occurs
 * @chip: pointer to smbchg_chip chip
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	/* OV condition is detected. Notify it to USB psy */
	if (reg & USBIN_OV_BIT) {
		chip->usb_ov_det = true;
		if (chip->usb_psy) {
			pr_smb(PR_MISC, "setting usb psy health OV\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->usb_ov_det = false;
		/* If USB is present, then handle the USB insertion */
		usb_present = is_usb_present(chip);
		if (usb_present)
			update_usb_status(chip, usb_present, false);
	}
out:
	return IRQ_HANDLED;
}

/**
 * usbin_uv_handler() - this is called when USB charger is removed
 * @chip: pointer to smbchg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}

	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, reg, chip->hvdcp_3_det_ignore_uv,
		aicl_level);

	/*
	 * set usb_psy's dp=f dm=f if this is a new insertion, i.e. it is
	 * not already src_detected and usbin_uv is seen falling
	 */
	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT) &&
		!chip->hvdcp_3_det_ignore_uv) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	if (reg & USBIN_UV_BIT)
		complete_all(&chip->usbin_uv_raised);
	else
		complete_all(&chip->usbin_uv_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
			/*
			 * If AICL is not even enabled, this is either an
			 * SDP or a grossly out of spec charger. Do not
			 * draw any current from it.
			 */
			rc = vote(chip->usb_suspend_votable,
					WEAK_CHARGER_EN_VOTER, true, 0);
			if (rc)
				pr_err("could not disable charger: %d", rc);
		} else if ((chip->aicl_deglitch_short || chip->force_aicl_rerun)
			&& aicl_level == chip->tables.usb_ilim_ma_table[0]) {
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("could not enable aicl reruns: %d", rc);
		}
		pr_smb(PR_MISC, "setting usb psy health UNSPEC_FAILURE\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		schedule_work(&chip->usb_set_online_work);
	}

	smbchg_wipower_check(chip);
out:
	return IRQ_HANDLED;
}

#ifdef SUPPORT_PARALLEL_CHG
/* Mask/Bit helpers */
#define _SMB1351_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB1351_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB1351_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))
				
/* Configuration registers */
#define CHG_CURRENT_CTRL_REG			0x0
#define FAST_CHG_CURRENT_MASK			SMB1351_MASK(7, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK		SMB1351_MASK(3, 0)
#define FAST_CHG_CURRENT_2000MA			0x60
#define FAST_CHG_CURRENT_2400MA			0x70
#define AC_INPUT_CURRENT_LIMIT_1000MA	0x2
#define AC_INPUT_CURRENT_LIMIT_1500MA	0x6

#define VARIOUS_FUNC_REG	0x2
#define SMB1351_AICL_EN_BIT	BIT(4)

#define VFLOAT_REG				0x3
#define SMB1351_VFLOAT_MASK		SMB1351_MASK(5, 0)
#define SMB1351_VFLOAT_4P38		0x2D

#define CHG_PIN_EN_CTRL_REG		0x6
#define EN_PIN_CTRL_MASK		SMB1351_MASK(6, 5)
#define EN_BY_I2C_0_DISABLE		0
#define EN_BY_I2C_0_ENABLE		0x20
#define EN_BY_PIN_CTRL_ENABLE	0x40  // EN_BY_PIN_HIGH_ENABLE, Pin control - active high
#define EN_BY_PIN_CTRL_DISABLE	0x60  // EN_BY_PIN_LOW_ENABLE , Pin control - active low

#define THERM_A_CTRL_REG			0x7
#define SOFT_COLD_TEMP_LIMIT_MASK	SMB1351_MASK(3, 2)
#define SOFT_HOT_TEMP_LIMIT_MASK	SMB1351_MASK(1, 0)

#define WDOG_SAFETY_TIMER_CTRL_REG	0x8
#define WDOG_TIMER_EN_BIT			BIT(0)

#define HARD_SOFT_LIMIT_CELL_TEMP_REG		0xB
#define HARD_LIMIT_HOT_TEMP_ALARM_TRIP_MASK	SMB1351_MASK(5, 4)
#define HARD_LIMIT_HOT_TEMP_ALARM_TRIP_65C	0x3

/* Command registers */
#define CMD_I2C_REG				0x30
#define CMD_BQ_CFG_ACCESS_BIT	BIT(6)

#define CMD_INPUT_LIMIT_REG			0x31
#define CMD_SUSPEND_MODE_BIT		BIT(6)
#define CMD_INPUT_CURRENT_MODE_BIT	BIT(3)
#define CMD_USB_1_5_AC_CTRL_MASK	SMB1351_MASK(1, 0)
#define CMD_USB_100_MODE			0
#define CMD_USB_500_MODE			0x2
#define CMD_USB_AC_MODE				0x1

#define AICL_RESULT_900MA			0x9

#ifndef ENG_BUILD
static int smbchg_check_dual_status(struct smbchg_chip *chip)
{
	bool b_dual_status = false;
	int rc;
	int aicl_complete = 0;
	u8 reg;
	
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		aicl_complete = reg & AICL_STS_BIT;
	else
		aicl_complete = false;
		
	if(aicl_complete)
	{
		reg &= ICL_STS_MASK;
		
		if (reg == AICL_RESULT_900MA)
			b_dual_status = true;
	}
	
	pr_smb(PR_STATUS, "aicl status = %d(%d), aicl result = 0x%02x !\n", aicl_complete, chip->aicl_complete, reg);
	
	return b_dual_status;
}

static int dual_chg_disable(void)
{
	int rc = 0,i;

	pr_smb(PR_STATUS, "\n");
	// 1. BQ configuration volatile access, 30h[6] = "1"
	rc = smb1351_masked_write_register(CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT, CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 BQ configuration fail !\n");

	// 1. Set smb1351 charger enter suspend , 0x31h[6] = "1"
	rc = smb1351_masked_write_register(CMD_INPUT_LIMIT_REG, CMD_SUSPEND_MODE_BIT, CMD_SUSPEND_MODE_BIT);
	if (rc)
	{
		pr_smb(PR_STATUS, "smb1351 enter suspend mode fail !\n");
		return rc;
	}

	// 2. Disable smb1351 charger , 0x06h[6:5] = "11"
	rc = smb1351_masked_write_register(CHG_PIN_EN_CTRL_REG, EN_PIN_CTRL_MASK, EN_BY_PIN_CTRL_DISABLE);
	if (rc){
		for (i = 0;i<5;i++){
			pr_smb(PR_STATUS, "smb1351 i2c write fail retry %d\n",i);
			msleep(200);
			rc = smb1351_masked_write_register(CHG_PIN_EN_CTRL_REG, EN_PIN_CTRL_MASK, EN_BY_PIN_CTRL_DISABLE);
			if(!rc) break;
		}
	}
	if (rc)
	{
		pr_smb(PR_STATUS, "smb1351 enter suspend mode fail !\n");
		return rc;
	}

	return rc;
}

static int parallel_chg_jeita_init_settings(void)
{
	int rc = 0,i;
	
	// 0. BQ configuration volatile access, 30h[6] = "1"
	rc = smb1351_masked_write_register(CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT, CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 BQ configuration fail !\n");

	// 1. set Hard Hot Limit = 72 Deg.C , 0x0Bh[5:4] = "11"
	rc = smb1351_masked_write_register(HARD_SOFT_LIMIT_CELL_TEMP_REG, 
				HARD_LIMIT_HOT_TEMP_ALARM_TRIP_MASK, HARD_LIMIT_HOT_TEMP_ALARM_TRIP_65C);
	if (rc){
		for (i = 0;i<5;i++){
			pr_smb(PR_STATUS, "smb1351 i2c write fail retry %d\n",i);
			msleep(200);
			rc = smb1351_masked_write_register(HARD_SOFT_LIMIT_CELL_TEMP_REG, 
				HARD_LIMIT_HOT_TEMP_ALARM_TRIP_MASK, HARD_LIMIT_HOT_TEMP_ALARM_TRIP_65C);
			if(!rc) break;
		}
	}
	
	if (rc)
	{
		pr_smb(PR_STATUS, "smb1351 exit suspend mode fail !\n");
		return rc;
	}
	
	// 2. set Soft Cold Limit Behavior = No Response , 0x07h[3:2] = "00"
	rc = smb1351_masked_write_register(THERM_A_CTRL_REG, SOFT_COLD_TEMP_LIMIT_MASK, 0);
	if (rc)
	{
		pr_smb(PR_STATUS, "set smb1351 Soft Cold temp limit fail !\n");
		return rc;
	}
	
	// 3. set Soft Hot temp limit = No Response , 0x07[1:0] = "00"
	rc = smb1351_masked_write_register(THERM_A_CTRL_REG, SOFT_HOT_TEMP_LIMIT_MASK, 0);
	if (rc)
	{
		pr_smb(PR_STATUS, "set smb1351 Soft Hot temp limit fail !\n");
		return rc;
	}
	
	return rc;
}
#endif

static int dual_chg_enable(int usb_in_current, int fast_chg_current)
{
	int rc = 0, i;
	pr_smb(PR_STATUS, "\n");
	
	// 0. BQ configuration volatile access, 30h[6] = "1"
	rc = smb1351_masked_write_register(CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT, CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 BQ configuration fail !\n");
#ifndef ENG_BUILD // Not control smb1351 suspend mode in eng build
	// 1. Set smb1351 charger non-suspend , 0x31h[6] = "0"
	rc = smb1351_masked_write_register(CMD_INPUT_LIMIT_REG, CMD_SUSPEND_MODE_BIT, 0);
	if (rc)
	{
		pr_smb(PR_STATUS, "smb1351 exit suspend mode fail !\n");
		return rc;
	}
#endif
	
	// 2. Set AC input current limit (maximun value)
	rc = smb1351_masked_write_register(CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, usb_in_current);
	if (rc)
	{
		pr_smb(PR_STATUS, "set smb1351 input current limit fail !\n");
		return rc;
	}
	
	// 3. Set fast charge current
	rc = smb1351_masked_write_register(CHG_CURRENT_CTRL_REG, FAST_CHG_CURRENT_MASK, fast_chg_current);
	if (rc)
	{
		pr_smb(PR_STATUS, "set smb1351 fast charge current fail !\n");
		return rc;
	}
	
	// 4. Disable smb1351 charger , 0x06h[6:5] = "10"
	rc = smb1351_masked_write_register(CHG_PIN_EN_CTRL_REG, EN_PIN_CTRL_MASK, EN_BY_PIN_CTRL_ENABLE);
	if (rc){
		for (i = 0;i<5;i++){
			pr_smb(PR_STATUS, "smb1351 i2c write fail retry %d\n",i);
			rc = smb1351_masked_write_register(CHG_PIN_EN_CTRL_REG, EN_PIN_CTRL_MASK, EN_BY_PIN_CTRL_ENABLE);
			msleep(200);
			if(!rc) break;
		}
	}
	if (rc)
	{
		pr_smb(PR_STATUS, "smb1351 enter suspend mode fail !\n");
		return rc;
	}
	
	return rc;
}

static int parallel_chg_insert_init_settings(void)
{
	int rc;	
	
	// 1. BQ configuration volatile access, 30h[6] = "1"
	rc = smb1351_masked_write_register(CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT, CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 BQ configuration fail !\n");

#ifndef ENG_BUILD // Not control smb1351 suspend mode in eng build
	// 2. Set Charger suspend, 31h[6] = "1"
	rc = smb1351_masked_write_register(CMD_INPUT_LIMIT_REG, CMD_SUSPEND_MODE_BIT, CMD_SUSPEND_MODE_BIT);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 enter suspend mode fail !\n");
#endif
	
	// 3. Set Charger Voltage = 4.38V, 03h[5:0] = "101100"
	rc = smb1351_masked_write_register(VFLOAT_REG, SMB1351_VFLOAT_MASK, SMB1351_VFLOAT_4P38);
	if (rc)
		pr_smb(PR_STATUS, "set smb1351 charger voltage fail !\n");
	
	// 4. Set HC mode, 31h[3] = "1" & 31h[0] = "1"
	rc = smb1351_masked_write_register(CMD_INPUT_LIMIT_REG, CMD_INPUT_CURRENT_MODE_BIT, CMD_INPUT_CURRENT_MODE_BIT);
	if (rc)
		pr_smb(PR_STATUS, "set smb1351 input current mode fail !\n");

	rc = smb1351_masked_write_register(CMD_INPUT_LIMIT_REG, CMD_USB_1_5_AC_CTRL_MASK, CMD_USB_AC_MODE);
	if (rc)
		pr_smb(PR_STATUS, "set smb1351 USB 1/5/AC mode fail !\n");

	// 5. Set Disable AICL, 02h[4] = "0"
	smb1351_masked_write_register(VARIOUS_FUNC_REG, SMB1351_AICL_EN_BIT, 0);
	if (rc)
		pr_smb(PR_STATUS, "smb1351 disable AICL fail !\n");
	
	// 6. set Watch Dog Timer Enable, 08h[0] = "1"
	smb1351_masked_write_register(WDOG_SAFETY_TIMER_CTRL_REG, WDOG_TIMER_EN_BIT, WDOG_TIMER_EN_BIT);
	if (rc)
		pr_smb(PR_STATUS, "set smb1351 watch dog timer fail !\n");
		
	return rc;
}
#endif

// Declare the smbchg_usbin_init_settings() function before using it
static int smbchg_usbin_init_settings(struct smbchg_chip *chip);

/**
 * src_detect_handler() - this is called on rising edge when USB charger type
 *			is detected and on falling edge when USB voltage falls
 *			below the coarse detect voltage(1V), use it for
 *			handling USB charger insertion and removal.
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	pr_smb(PR_STATUS, "\n");
	cancel_delayed_work(&chip->usb_insertion_work);
	queue_delayed_work(chip->smbchg_src_detect_queue ,&chip->usb_insertion_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

/**
 * otg_oc_handler() - called when the usb otg goes over current
 */
#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->schg_version == QPNP_SCHG_LITE) {
		pr_warn("OTG OC triggered - OTG disabled\n");
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	/*
	 * Due to a HW bug in the PMI8994 charger, the current inrush that
	 * occurs when connecting certain OTG devices can cause the OTG
	 * overcurrent protection to trip.
	 *
	 * The work around is to try reenabling the OTG when getting an
	 * overcurrent interrupt once.
	 */
	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

/**
 * otg_fail_handler() - called when the usb otg fails
 * (when vbat < OTG UVLO threshold)
 */
static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

/**
 * aicl_done_handler() - called when the usb AICL algorithm is finished
 *			and a current is set.
 */
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
#if 0 //remove qualcomm parallel charge
	bool usb_present = is_usb_present(chip);
#endif
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

#if 0 //remove qualcomm parallel charge
	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);
#endif

	if (chip->aicl_complete)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

/**
 * usbid_change_handler() - called when the usb RID changes.
 * This is used mostly for detecting OTG
 */
static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

	pr_smb(PR_INTERRUPT, "triggered\n");

	otg_present = is_otg_present(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy OTG = %d\n",
				otg_present ? 1 : 0);
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	}
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));

	return IRQ_HANDLED;
}

static int determine_initial_status(struct smbchg_chip *chip)
{
	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	chg_term_handler(0, chip);
	usbid_change_handler(0, chip);

	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present) {
		complete_all(&chip->sdp_retry_complete_flag);
		pr_smb(PR_STATUS, "complete_all sdp_retry_complete_flag\n");
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy, POWER_SUPPLY_DP_DM_DPF_DMF);
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
                if (entry_mode == 4)
                       schedule_work(&chip->usb_set_online_work);
		queue_delayed_work(chip->smbchg_src_detect_queue ,&chip->usb_insertion_work, msecs_to_jiffies(0));
#else
		src_detect_handler(0, chip);
#endif
	} else {
		handle_usb_removal(chip);
	}

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_POLARITY_BIT		BIT(6)
#define CHG_EN_POL_HIGH			0x0
#define CHG_EN_POL_LOW			BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_BAT_OV_ECC			BIT(4)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define HOLD_OFF_TIMER_CHG		BIT(1)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define RCHG_THRESH_100MV		0
#define RCHG_THRESH_200MV		1
#define VCHG_EN_BIT			BIT(1)
#define VCHG_INPUT_CURRENT_BIT		BIT(3)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
#define OTG_OC_CFG			0xF1
#define HICCUP_ENABLED_BIT		BIT(6)
#define AICL_ADC_BIT			BIT(6)
#define USBIN_RERUN_AICL		BIT(5)
static void batt_ov_wa_check(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	/* disable-'battery OV disables charging' feature */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
				CHG_BAT_OV_ECC, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return;
	}

	/*
	 * if battery OV is set:
	 * restart charging by disable/enable charging
	 */
	rc = smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read Battery RT status rc = %d\n", rc);
		return;
	}

	if (reg & BAT_OV_BIT) {
		rc = smbchg_charging_en(chip, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't disable charging: rc = %d\n", rc);
			return;
		}

		/* delay for charging-disable to take affect */
		msleep(200);

		rc = smbchg_charging_en(chip, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't enable charging: rc = %d\n", rc);
			return;
		}
	}
}

#define PCC_CFG			0xF1
#define PCC_250MA_VAL	0x3
#define PCC_550MA_VAL	0x4
#define PCC_MASK		SMB_MASK(2, 0)
#define PCC_CURRENT_250_MA		250
#define PCC_CURRENT_550_MA		550

#define CFG_TCC			0xF9
#define CFG_TCC_MASK	SMB_MASK(2, 0)
#define TREM_ANA_300MA	0x0
#define TREM_ANA_250MA	0x5

#define CHARGER_CONTROL1_MASK	(BIT(6) | BIT(2) | BIT(0))
#define CHARGER_CONTROL1_SET		(BIT(6) | BIT(2) | BIT(0))

//#define CHARGER_CONTROL2_MASK	(BIT(6) | BIT(4) | BIT(3))
//#define CHARGER_CONTROL2_SET		(BIT(6) | BIT(4) | BIT(3))
#define CHARGER_CONTROL2_MASK	BIT(6)
#define CHARGER_CONTROL2_SET	BIT(6)

#define WDOG_CFG			0xF1
#define WDOG_CONTROL_MASK	(BIT(6) | BIT(0))
#define WDOG_CONTROL_SET	(BIT(6))

#define CFG_SYSMIN			0xF4
#define CFG_SYSMIN_MASK		SMB_MASK(1, 0)
#define SYSMIN_3P6			0x2

static int smbchg_usbin_init_settings(struct smbchg_chip *chip)
{
	int rc;

#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL) || defined(CONFIG_ZT582KL)
	if (!wake_lock_active(&alarm_cable_lock)){
		pr_err("[%s] Cable WakeLock: *LOCK*\n", __func__);
		wake_lock(&alarm_cable_lock);
	}
#endif

#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL) || defined(CONFIG_ZT582KL)
	// 1. set SMBCHGL_CHGR_PCC_CFG = PCC_250MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + PCC_CFG,PCC_MASK, PCC_250MA_VAL);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set pre charge current %dmA rc=%d\n", PCC_CURRENT_250_MA, rc);

	// 2. set SMBCHGL_CHGR_FCC_CFG = FCC_2100MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2100MA_VAL);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2100_MA, rc);

	// 3. set SMBCHGL_CHGR_FV_CFG = FV_4P38
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);

	// 4. set SMBCHGL_CHGR_CFG[0] - CFG_RCHG_LVL = RCHG_THRESH_100MV
	rc = smbchg_sec_masked_write(chip,	chip->chgr_base + CHGR_CFG, RCHG_LVL_BIT, RCHG_THRESH_100MV);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set recharge rc = %d\n", rc);

	// 5. set SMBCHGL_USB_USBIN_IL_CFG = USBIN_IL_900MA (PR1 or later hw version)
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, 
				(chip->dedicated_adapter || chip->dedicated_power_bank) ? USBIN_IL_1800MA : USBIN_IL_900MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USBIN_IL rc = %d\n", rc);

	// 6. set SMBCHGL_CHGR_CFG_TCC = TERM_ANA_250MA
	rc = smbchg_sec_masked_write(chip,	chip->chgr_base + CFG_TCC, CFG_TCC_MASK, TREM_ANA_250MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set termination current threshold rc = %d\n", rc);
	
	// 7.1 set TERM_I_SRC = Fuel GaugeADC (0:Analog sensor, 1:Fuel GaugeADC)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,	TERM_I_SRC_BIT, TERM_SRC_FG);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg1 - term_i_src bit rc=%d\n", rc);
	
	// 7.2 set RECHG_THRESHOLD_SRC = Analog sensor (0:Analog sensor, 1:Fuel GaugeADC)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,	RECHG_THRESHOLD_SRC_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg1 - rechg_threshold_src bit rc=%d\n", rc);
		
	// 7.3 set qcom,fg-chg-iterm-ma = 150(device tree) and Update FG_SRAM[0x4F8 offset 2] = 150 (qpnp-fg.c)

	// 8. set SMBCHGL_USB_CFG[5:4] - HVDCP_ADAPTER_SEL = HVDCP_5V
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG, HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		
	// 9. set SMBCHGL_DC_AICL_WL_SEL_CFG = AICL_RESTART_TIMER_6MIN (delete in 20160222 porting guide
	/*rc = smbchg_sec_masked_write(chip, chip->dc_chgpth_base + AICL_WL_SEL_CFG, 
					AICL_RESTART_TIMER, AICL_RESTART_TIMER_6MIN);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set AICL_RESTART_TIMER rc=%d\n", rc);*/
	
	// 10. set SMBCHGL_MISC_CHGR_TRIM_OPTIONS_15_8[5] = AICL_USB_RERUN_DIS (0:AICL_USB_RERUN_DIS, 1:AICL_USB_RERUN_EN)
	rc = smbchg_sec_masked_write(chip, chip->misc_base + MISC_TRIM_OPT_15_8, USBIN_RERUN_AICL, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n", rc);
	
	// 11. set SMBCHGL_MISC_WD_CFG = EN_WDT_72s (enable watch dog timer, timeout is 72s)
#if !defined(CONFIG_ZT582KL)
	rc = smbchg_sec_masked_write(chip, chip->misc_base + WDOG_CFG, WDOG_CONTROL_MASK, WDOG_CONTROL_SET);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable watch dog timeout rc=%d\n",	rc);	
#endif	
	// 12. set USBIN_RERUN_AICL_EN (0x16F5[5]) (delete in 20160222 porting guide)

	// 13. set CHG_EN_COMMAND = Active High
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	CHG_EN_POLARITY_BIT, CHG_EN_POL_HIGH);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to EN_HIGH rc=%d\n", rc);

	// 14. set CHG_EN_COMMAND = Active Low
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	CHG_EN_POLARITY_BIT, CHG_EN_POL_LOW);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to EN_LOW rc=%d\n", rc);
	
	// 15. set Register Control Charger
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2, CHARGER_CONTROL2_MASK, CHARGER_CONTROL2_SET);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 Register Control rc=%d\n", rc);

	//Shilun ++++
	// 11. set EN_BAT_CHG = Disable , 0x1242[1] = "1" (Charge ensable/disable (polarity from CHGR_CFG2[6] of Charger peripheral))
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,	EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat_if_base EN_BAT_CHG_BIT to Disable rc=%d\n", rc);

	// 12. set EN_BAT_CHG = Enable , 0x1242[1] = "0" (Charge disable/enable (polarity from CHGR_CFG2[6] of Charger peripheral))

	if(b_last_charging)
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,	EN_BAT_CHG_BIT, 0);

	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat_if_base EN_BAT_CHG_BIT to Enable rc=%d\n", rc);
	//Shilun ----- 
	
	// 16. set SMBCHGL_BAT_IF_CFG_SYSMIN = SYSMIN_3P6
	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + CFG_SYSMIN, CFG_SYSMIN_MASK, SYSMIN_3P6);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat if config sysmin rc=%d\n", rc);
	
	// 17. set SMBCHGL_USB_COM_IL - enable DCIN suspend mode
	//rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, DCIN_SUSPEND_BIT, DCIN_SUSPEND_BIT);
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL, 0xFF, 0x8);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable dcin suspend mode rc=%d\n", rc);		

	// 18. CHG_TYPE = UNDEFFINED
	chg_type_flag = UNDEFINED;
/* 20170110 follow ZT582KL_1.0_SR1_CHG_Porting_20161216, delete workaround
	// 19. rerun apsd in battery low.
	if(((chip->fg_voltage < BATT_LOW_VOLT) || (chip->fg_capacity < BATT_LOW_LEVEL)) && (chip->cable_power_on == true))
	{
//		pr_err("APSD rerun @####@\n");
//		chip->hvdcp_3_det_ignore_uv = true;
//		rc = rerun_apsd(chip);
//		chip->hvdcp_3_det_ignore_uv = false;
		chip->cable_power_on = false;
//		msleep(1000);
	}
*/
#elif defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)

	// 1. set SMBCHGL_CHGR_PCC_CFG = PCC_550MA , 0x10F1[2:0] = 0x04
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + PCC_CFG,PCC_MASK, PCC_550MA_VAL);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set pre charge current %dmA rc=%d\n", PCC_CURRENT_550_MA, rc);

	// 2. set SMBCHGL_CHGR_FCC_CFG = FCC_2300MA , 0x10F2[4:0] = 0x1C
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2300MA_VAL);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2300_MA, rc);

	// 3. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);

	// 4. set SMBCHGL_CHGR_CFG[0] - CFG_RCHG_LVL = RCHG_THRESH_200MV , 0x10FF[0] = 1
	rc = smbchg_sec_masked_write(chip,	chip->chgr_base + CHGR_CFG, RCHG_LVL_BIT, RCHG_THRESH_200MV);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set recharge rc = %d\n", rc);

	// 5. set SMBCHGL_USB_USBIN_IL_CFG = USBIN_IL_900MA , 0x13F2[4:0] = 0x09 (default)
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USBIN_IL rc = %d\n", rc);

	// 6. set SMBCHGL_CHGR_CFG_TCC = TERM_ANA_300MA , 0x10F9[2:0] = 0x00
	rc = smbchg_sec_masked_write(chip,	chip->chgr_base + CFG_TCC, CFG_TCC_MASK, TREM_ANA_300MA);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set termination current threshold rc = %d\n", rc);

	// 7.1 set TERM_I_SRC = Fuel GaugeADC (0:Analog sensor, 1:Fuel GaugeADC) , 0x10FB[2] = "1"
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,	TERM_I_SRC_BIT, TERM_SRC_FG);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg1 - term_i_src bit rc=%d\n", rc);

	// 7.2 set RECHG_THRESHOLD_SRC = Analog sensor (0:Analog sensor, 1:Fuel GaugeADC) , 0x10FB[1] = "0"
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,	RECHG_THRESHOLD_SRC_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg1 - rechg_threshold_src bit rc=%d\n", rc);

	// 7.3 set qcom,fg-chg-iterm-ma = 200(device tree) and Update FG_SRAM[0x4F8 offset 2] = 200 (qpnp-fg.c)

	// 8. set SMBCHGL_USB_CFG[5:4] - HVDCP_ADAPTER_SEL = HVDCP_5V , 0x13F4[5:4] = "00"
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG, HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	// 9. set SMBCHGL_MISC_WD_CFG = EN_WDT_72s (enable watch dog timer, timeout is 72s) , 0x16F1[7:0] = 0x41
	/*rc = smbchg_sec_masked_write(chip, chip->misc_base + WDOG_CFG, WDOG_CONTROL_MASK, WDOG_CONTROL_SET);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable watch dog timeout rc=%d\n",	rc);	
	*/
	// 10. set CHG_EN_SRC = REGISTER CONTROL[7,0] && CHG_EN_COMMAND = Active low[6,1] && PRE_FAST_AUTO[5,0]
	//         BATT_OV_ENDS_CYSLE_DIS[4,0] && EN_AUTO_RECHARGH[2,0] && HOLDOFF_TMR_350ms[1,1] , 0x10FC[7:0] = 0x42
	
	// 10.1 CHG_EN_SRC = REGISTER CONTROL[7] = "0" (0:Command Register, 1:Enable Pin) 
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	CHG_EN_SRC_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to CHG_EN_SRC_BIT rc=%d\n", rc);

	// 10.2 CHG_EN_COMMAND = Active low[6] = "1" (0:Active high(1: enable charging), 1:Active low(0: enable charging))
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	CHG_EN_POLARITY_BIT, CHG_EN_POLARITY_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to CHG_EN_POLARITY_BIT rc=%d\n", rc);
	
	// 10.3 PRE_FAST_AUTO[5] = "0" (0:Automatic, 1:Requires command)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	P2F_CHG_TRAN, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to P2F_CHG_TRAN rc=%d\n", rc);
		
	// 10.4 BATT_OV_ENDS_CYSLE_DIS[4] = "0" (0:Disable, 1:Enable)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	CHG_BAT_OV_ECC, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to CHG_BAT_OV_ECC rc=%d\n", rc);
	
	// 10.5 EN_AUTO_RECHARGH[2] = "0" (0:Disable, 1:Enable)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	AUTO_RECHG_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to AUTO_RECHG_BIT rc=%d\n", rc);
	
	// 10.6 HOLDOFF_TMR_350ms[1] = "1" (0:750us, 1:350ms)
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,	HOLD_OFF_TIMER_CHG, HOLD_OFF_TIMER_CHG);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set chgr_cfg2 to HOLD_OFF_TIMER_CHG rc=%d\n", rc);

	// 11. set EN_BAT_CHG = Disable , 0x1242[1] = "1" (Charge ensable/disable (polarity from CHGR_CFG2[6] of Charger peripheral))
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,	EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat_if_base EN_BAT_CHG_BIT to Disable rc=%d\n", rc);

	// 12. set EN_BAT_CHG = Enable , 0x1242[1] = "0" (Charge disable/enable (polarity from CHGR_CFG2[6] of Charger peripheral))
	if(b_last_charging)
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,	EN_BAT_CHG_BIT, 0);

	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat_if_base EN_BAT_CHG_BIT to Disable rc=%d\n", rc);

	// 13. set SMBCHGL_BAT_IF_CFG_SYSMIN = SYSMIN_3P6 , 0x12F4[1:0] = "10"
	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + CFG_SYSMIN, CFG_SYSMIN_MASK, SYSMIN_3P6);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set bat if config sysmin rc=%d\n", rc);
	
	// 14. Set Dual-Charger flag = Undefined
	dual_chg_flag = Undefined;
	
	// 15. Set thermal policy flag = 0
	chg_type_flag = UNDEFINED;
	thermal_policy_flag = TP_LEVEL0;
	
	// 16. rerun apsd in battery low.
	if(chip->cable_power_on == true)
	{
//		pr_err("APSD rerun @####@\n");
//		chip->hvdcp_3_det_ignore_uv = true;
//		rc = rerun_apsd(chip);
//		chip->hvdcp_3_det_ignore_uv = false;
		chip->cable_power_on = false;
//		msleep(1000);
	}
#endif

	return rc;
}

static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	if (chip->aicl_rerun_period_s > 0) {
		rc = smbchg_set_aicl_rerun_period_s(chip,
				chip->aicl_rerun_period_s);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_POLARITY_BIT
			| (chip->chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * enable battery charging to make sure it hasn't been changed before
	 * boot.
	 */
	rc = smbchg_charging_en(chip, true);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", rc);
		return rc;
	}

	/*
	 * Based on the configuration, use the analog sensors or the fuelgauge
	 * adc for recharge threshold source.
	 */

	if (chip->chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
			TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	check_battery_type(chip);

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->vfloat_mv);
	}

	/* set the fast charge current compensation */
	if (chip->fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
			chip->fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
			chip->fastchg_current_comp);
	}

	/* set the float voltage compensation */
	if (chip->float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
			chip->float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
			chip->float_voltage_comp);
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			smbchg_iterm_set(chip, chip->iterm_ma);
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
		chip->safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
				rc);
		else if (!(reg & SFT_EN_MASK))
			chip->safety_timer_en = true;
	}

	/* configure jeita temperature hard limit */
	if (chip->jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT,
			chip->jeita_temp_hard_limit
			? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set jeita temp hard limit rc = %d\n",
				rc);
			return rc;
		}
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	if (chip->vchg_adc_channel != -EINVAL) {
		/* configure and enable VCHG */
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	smbchg_charging_status_change(chip);
	vote(chip->usb_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);
	vote(chip->dc_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);

	/* resume threshold */
	if (chip->resume_delta_mv != -EINVAL) {

		/*
		 * Configure only if the recharge threshold source is not
		 * fuel gauge ADC.
		 */
		if (!chip->chg_inhibit_source_fg) {
			if (chip->resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT,
				(chip->resume_delta_mv
				 < chip->tables.rchg_thr_mv)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	/* DC path current settings */
	if (chip->dc_psy_type != -EINVAL) {
		rc = vote(chip->dc_icl_votable, PSY_ICL_VOTER, true,
					chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't vote for initial DC ICL rc=%d\n", rc);
			return rc;
		}
	}


	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smbchargers go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */
	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}

	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,
			chip->cfg_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't vote fastchg ma rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
				rc);
			return rc;
		}
	}
	/* unsuspend dc path, it could be suspended by the bootloader */
	rc = smbchg_dc_suspend(chip, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend dc path= %d\n", rc);
		return rc;
	}

	if (chip->force_aicl_rerun)
		rc = smbchg_aicl_config(chip);

	if (chip->schg_version == QPNP_SCHG_LITE) {
		/* enable OTG hiccup mode */
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_OC_CFG,
					HICCUP_ENABLED_BIT, HICCUP_ENABLED_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set OTG OC config rc = %d\n",
				rc);
	}

	if (chip->wa_flags & SMBCHG_BATT_OV_WA)
		batt_ov_wa_check(chip);

	/* turn off AICL adc for improved accuracy */
	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8, AICL_ADC_BIT, 0);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);
			
#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL) || defined(CONFIG_ZT582KL)
	/* set OTG conifg */
	// 1. set OTG_EN(0x1242) = Disable
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);
	
	// 2. set SMBCHGL_OTG_OTG_CFG(0x11F1) = 0x68
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG, OTG_CFG_MASK, OTG_CFG_CONFIG);
	if (rc < 0)
		pr_err("Couldn't set OTG CFG config rc = %d\n", rc);
	
	// 3. set SMBCHGL_OTG_CFG_BATTUV(0x11F2) = 0x01, set OTG battery under-voltage lock out = 2.9V
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG_BATTUV, OTG_CFG_BATTUV_MASK, OTG_CFG_BATTUV_CONFIG);
	if (rc < 0)
		pr_err("Couldn't set OTG Battery under voltage rc = %d\n", rc);
		
	// 4. set SMBCHGL_OTG_OTG_ICFG(0x11F3) = OTG_ILIMIT_250MA
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_ICFG, OTG_ILIMT_MASK, OTG_ILIMT_250MA);
	if (rc < 0)
		pr_err("Couldn't set OTG ILIMT config rc = %d\n", rc);
#elif defined(ZT500KL) || defined(CONFIG_Z500KL)
	/* set OTG conifg */	
	// 1. set SMBCHGL_OTG_OTG_CFG(0x11F1) = 0x28
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG, OTG_CFG_MASK, OTG_CFG_CONFIG);
	if (rc < 0)
		pr_err("Couldn't set OTG CFG config rc = %d\n", rc);
	
	// 2. set SMBCHGL_OTG_CFG_BATTUV(0x11F2) = 0x01, set OTG battery under-voltage lock out = 2.9V
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG_BATTUV, OTG_CFG_BATTUV_MASK, OTG_CFG_BATTUV_CONFIG);
	if (rc < 0)
		pr_err("Couldn't set OTG Battery under voltage rc = %d\n", rc);
		
	// 3. set SMBCHGL_OTG_OTG_ICFG(0x11F3) = OTG_ILIMIT_250MA
	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_ICFG, OTG_ILIMT_MASK, OTG_ILIMT_250MA);
	if (rc < 0)
		pr_err("Couldn't set OTG ILIMT config rc = %d\n", rc);
#endif
	
	chip_fg->fg_capacity = fg_read_mem_data(FG_DATA_BATT_SOC);
	chip_fg->fg_voltage = fg_read_mem_data(FG_DATA_CPRED_VOLTAGE);

	pr_err("Read fg mem data : capacity = %d, voltage = %d \n", chip_fg->fg_capacity, chip_fg->fg_voltage);
	
	chip->cable_power_on = false;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	if(reg == 0x01)
	{
		chip->cable_power_on = true;
		g_cable_poweron = true;
		pr_smb(PR_STATUS, "Cable Power ON !\n");
	}
	//else
	//	pr_smb(PR_STATUS, "Not Cable Power ON ~~\n");

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct smbchg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->wipower_dyn_icl_avail = false;

	if (!chip->vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_default,
					"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_pt,
					"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_div2,
					"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->wipower_default.num = 0;
	chip->wipower_pt.num = 0;
	chip->wipower_default.num = 0;
	if (chip->wipower_default.entries)
		devm_kfree(chip->dev, chip->wipower_default.entries);
	if (chip->wipower_pt.entries)
		devm_kfree(chip->dev, chip->wipower_pt.entries);
	if (chip->wipower_div2.entries)
		devm_kfree(chip->dev, chip->wipower_div2.entries);
	chip->wipower_default.entries = NULL;
	chip->wipower_pt.entries = NULL;
	chip->wipower_div2.entries = NULL;
	chip->vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
#define DEFAULT_FCC_MA			3000
#else
#define DEFAULT_FCC_MA			2000
#endif
static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->cfg_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->cfg_fastchg_current_ma == -EINVAL)
		chip->cfg_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->vled_max_uv < 0)
		chip->vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->rpara_uohm < 0)
		chip->rpara_uohm = 0;
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
	if (chip->safety_time != -EINVAL &&
		(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->safety_time);
		return -EINVAL;
	}
	if (chip->prechg_safety_time != -EINVAL &&
		(chip->prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
#if defined(ZT500KL) || defined(CONFIG_Z500KL)
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"500KL-parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"500KL-parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"500KL-parallel-allowed-lowering-ma", rc, 1);
#else
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
#endif
	if (chip->parallel.min_current_thr_ma != -EINVAL
			&& chip->parallel.min_9v_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
	/*
	 * use the dt values if they exist, otherwise do not touch the params
	 */
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-fcc-percent",
					&smbchg_main_chg_fcc_percent);
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-icl-percent",
					&smbchg_main_chg_icl_percent);
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->parallel.min_current_thr_ma,
			chip->parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);
	OF_PROP_READ(chip, chip->aicl_rerun_period_s,
			"aicl-rerun-period-s", rc, 1);
	OF_PROP_READ(chip, chip->vchg_adc_channel,
				"vchg-adc-channel-id", rc, 1);

	/* read boolean configuration properties */
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
						"qcom,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
					"qcom,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
						"qcom,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");
	chip->force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");

	/* parse the battery missing detection pin source */
	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	/* read the battery power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	/* Get the charger led support property */
	chip->cfg_chg_led_sw_ctrl =
		of_property_read_bool(node, "qcom,chg-led-sw-controls");
	chip->cfg_chg_led_support =
		of_property_read_bool(node, "qcom,chg-led-support");

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	chip->skip_usb_notification
		= of_property_read_bool(node,
				"qcom,skip-usb-notification");

	chip->cfg_override_usb_current = of_property_read_bool(node,
				"qcom,override-usb-current");

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
				rc);
			if (chip->schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->usbid_change_irq, "usbid-change",
					usbid_change_handler,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->otg_oc_irq);
				enable_irq_wake(chip->usbid_change_irq);
				enable_irq_wake(chip->otg_fail_irq);
			}
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
			if (chip->parallel.avail && chip->usb_present) {
				rc = enable_irq_wake(chip->aicl_done_irq);
				chip->enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->wdog_timeout_irq,
				"wdog-timeout",
				wdog_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->wdog_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			enable_irq_wake(chip->usbid_change_irq);
			enable_irq_wake(chip->otg_oc_irq);
			enable_irq_wake(chip->otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
		case SMBCHG_LITE_OTG_SUBTYPE:
			chip->otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

/* dumps useful registers for debug */
static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Config");
	/* battery interface peripheral */
	dump_reg(chip, chip->bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF Config");
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Config");
	/* dc charge path peripheral */
	dump_reg(chip, chip->dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC Config");
	
	/* otg peripheral */
	for (addr = 0x00; addr <= 0xFF; addr++)
	{
		if((addr == 0x04) || (addr == 0x05) || (addr == 0x10) || (addr == 0x18) || (addr == 0x19))
			dump_reg(chip, chip->otg_base + addr, "OTG Status"); // register is Read Only
		else if((addr == 0x11) || (addr == 0x12) || (addr == 0x13) || (addr == 0x14) || (addr == 0x15) ||
				(addr == 0x16) || (addr == 0x1A) || (addr == 0x1B) || (addr == 0xF1) || (addr == 0xF2) ||
				(addr == 0xF3) || (addr == 0xF6))
			dump_reg(chip, chip->otg_base + addr, "OTG Config"); // register can Read / Write
	}
	
	/* misc peripheral */
	dump_reg(chip, chip->misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC CFG");
}

static int smbcharger_show(struct seq_file *s, void *data)
{
	struct smbchg_chip *chip = s->private;

	dump_regs(chip);
	seq_printf(s, "dump_regs\n");
	return 0;
}

static int smbcharger_open(struct inode *inode, struct file *file)
{
	struct smbchg_chip *chip = inode->i_private;
	return single_open(file, smbcharger_show, chip);
}

static const struct file_operations smbcharger_ops = {
	.open = smbcharger_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct smbchg_chip *chip_fg;
int vzw_demo_mode_control(bool b_stop_charging);

#if !defined(VZW)
static int enableADF_check(void)
{
	char buf[4] = {0, 0, 0, 0};
	struct file *fd;

	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = filp_open("/ADF/ADF", O_RDONLY, 0);

	if (!IS_ERR(fd)) {
		kernel_read(fd, fd->f_pos, buf, 4);
		filp_close(fd, NULL);
	} else {
		set_fs(old_fs);
		pr_info("open /ADF/ADF fail\n");
		return 0;
	}

	set_fs(old_fs);
	pr_info("***** check ADF: %d\n", buf[3]);
	if (buf[3] == 1 || buf[3] == 2) {
		pr_info("enableADF_check pass %d\n", buf[3]);
		return 1;
	} else {
		pr_info("enableADF_check fail %d\n", buf[3]);
		return 0;
	}
}
#endif

void demochg_judge(int en){
	int soc = get_prop_batt_capacity(chip_fg);
#if defined(VZW)
	if(en && (soc>= 35)){
		g_demochg_res = 1;//Stop Charging
	}else if(en && (soc <= 30)){
		g_demochg_res = 0;//Recharging
	}else if(!en){
		g_demochg_res = 0;//No control, No need to restrict charging
	}
	pr_smb(PR_STATUS, "g_demochg_flag(%d) en(%d) soc(%d) g_demochg_res(%d)\n", g_demochg_flag, en, soc, g_demochg_res);
#else
	if(enableADF_check()){
		if(en && (soc>= 60)){
			g_demochg_res = 1;//Stop Charging
		}else if(en && (soc <= 55)){
			g_demochg_res = 0;//Recharging
		}else if(!en){
			g_demochg_res = 0;//No control, No need to restrict charging
		}
	}else{
		g_demochg_res = 0;//No control, No need to restrict charging
	}

	pr_smb(PR_STATUS, "g_demochg_flag(%d) en(%d) soc(%d) g_demochg_res(%d)\n", g_demochg_flag, en, soc, g_demochg_res);
#endif
#ifndef ENG_BUILD
	if(en)
		vzw_demo_mode_control(g_demochg_res);
#endif
//	smbchg_charging_status_change(chip_fg);
}

static int smbcharger_demochg_read(struct seq_file *buf, void *v){
	seq_printf(buf, "g_demochg_flag = %d\n",g_demochg_flag);
	return 0;
}

static ssize_t smbcharger_demochg_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	char messages[256];
	read_usb_type(chip_fg, &usb_type_name, &usb_supply_type);
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ((buff[0] == '1') && (g_demochg_flag == false)) {
		g_demochg_flag = 1;
	}else if((buff[0] == '0')&&(g_demochg_flag== true)){
		g_demochg_flag = 0;
	}

	demochg_judge(g_demochg_flag);
	pr_smb(PR_STATUS, "[%s] %d\n", __func__, g_demochg_flag);
	return len;
}

static int smbcharger_demochg_proc_open(struct inode *inode, struct  file *file){
	return single_open(file, smbcharger_demochg_read, NULL);
}

static const struct file_operations smbcharger_demochg_ops = {
	.owner = THIS_MODULE,
	.open =  smbcharger_demochg_proc_open,
	.write = smbcharger_demochg_proc_write,
	.read = seq_read,
	.release = single_release,
};

static void create_demochg_proc_file(void){
	struct proc_dir_entry *smbcharger_demochg_proc_file = proc_create("driver/demochg",
				0664, NULL, &smbcharger_demochg_ops);

	if (smbcharger_demochg_proc_file) {
		printk("[SMBCHG][%s] demochg create ok!\n",__func__);
	} else{
		printk("[SMBCHG][%s] demochg create failed!\n",__func__);
	}
	return;
}

static int create_debugfs_entries(struct smbchg_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
				  S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root, chip,
				  &force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
			"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}

	 ent = debugfs_create_file("smbcharger",0444,
                                  chip->debug_root, chip,
                                  &smbcharger_ops);
        if (!ent) {
                dev_err(chip->dev,
                        "Couldn't create smbcharger file\n");
                return -EINVAL;
        }
	return 0;
}

void get_usb_type(enum power_supply_type *usb_supply_type)
{
	pr_err("usb_supply_type = %d\n", chip_fg->usb_supply_type);
	*usb_supply_type = chip_fg->usb_supply_type;
}
EXPORT_SYMBOL(get_usb_type);

#if SUPPORT_FORCE_STOP_CHARGING
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
static int smbchg_force_charging_limit(struct smbchg_chip *chip, bool b_stop_charging)
{
	int rc = 0;
	
	if(b_stop_charging)
	{
		if(b_last_charging)
		{
			rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
			if (rc < 0) {
				dev_err(chip->dev, "set CHG_EN_COMMAND to Active High Fail rc = %d\n", rc);
				return rc;
			}
			dual_chg_disable();
			pr_smb(PR_STATUS, "Force Stop charging ###\n");
			power_supply_changed(&chip->batt_psy);
			b_last_charging = false;
		}
		else
			pr_smb(PR_STATUS, "Now the status has to stop charging, no need to call stop charging command ###\n");		
	}
	else
	{
		if(!b_last_charging)
		{
			rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
			if (rc < 0) {
				dev_err(chip->dev, "set CHG_EN_COMMAND to Active Low Fail rc = %d\n", rc);
				return rc;
			}
			if (dual_chg_flag == ASUS_2A)
				dual_chg_enable(AC_INPUT_CURRENT_LIMIT_1000MA, FAST_CHG_CURRENT_2400MA);
			pr_smb(PR_STATUS, "Re-chargiing ###\n");
			power_supply_changed(&chip->batt_psy);
			b_last_charging = true;
		}
		else
			pr_smb(PR_STATUS, "Now the status has to charging, no need to call re-charging command ###\n");
	}

	return rc;	
}
#else //defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
static int smbchg_force_charging_limit(struct smbchg_chip *chip, bool b_stop_charging)
{
	int rc = 0;
	u8 reg;
	
	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_CFG2, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read CHG_EN_COMMAND Status rc = %d\n", rc);
		return rc;
	}
		
	if(b_stop_charging)
	{
		if(reg & CHG_EN_POLARITY_BIT)
		{
			rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2, CHG_EN_POLARITY_BIT, CHG_EN_POL_HIGH);
			if (rc < 0) {
				dev_err(chip->dev, "set CHG_EN_COMMAND to Active High Fail rc = %d\n", rc);
				return rc;
			}
			pr_smb(PR_STATUS, "Force Stop charging ###\n");
			power_supply_changed(&chip->batt_psy);
		}
#ifdef ENG_BUILD
		else
			pr_smb(PR_STATUS, "Now the status has to stop charging, no need to call stop charging command ###\n");		
#endif
	}
	else
	{
		if(!(reg & CHG_EN_POLARITY_BIT))
		{
			rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2, CHG_EN_POLARITY_BIT, CHG_EN_POL_LOW);
			if (rc < 0) {
				dev_err(chip->dev, "set CHG_EN_COMMAND to Active Low Fail rc = %d\n", rc);
				return rc;
			}
			pr_smb(PR_STATUS, "Re-chargiing ###\n");
			power_supply_changed(&chip->batt_psy);
		}
#ifdef ENG_BUILD
		else
			pr_smb(PR_STATUS, "Now the status has to charging, no need to call re-charging command ###\n");
#endif
	}
	
	return rc;	
}
#endif
#endif

#ifdef ENG_BUILD
bool b_eng_charging_limit = true; // flag status refer to charging limit node
static bool b_eng_stop_charging = false; // flag status refer to fg capacity
EXPORT_SYMBOL(b_eng_charging_limit);

static int smbchg_eng_charging_limit(struct smbchg_chip *chip)
{
	int rc = 0;
	
	if((b_eng_charging_limit) && (b_eng_stop_charging))
	{
		smbchg_force_charging_limit(chip, STOP_CHARGING_ENABLE);
	}
	else if((!b_eng_charging_limit) || (!b_eng_stop_charging))
	{
		smbchg_force_charging_limit(chip, STOP_CHARGING_DISABLE);
	}
	
	pr_smb(PR_STATUS, "Charging limit status : node_limit(%d), fg_limit(%d), threshold(%d) ###\n",	b_eng_charging_limit, b_eng_stop_charging, charging_limit_threshold);
	
	return rc;
}

static ssize_t nasus_charging_limit_toggle_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
	struct smbchg_chip *chip  = PDE_DATA(file_inode(file));
	char proc_buf[64] = {0};

    if (count > sizeof(proc_buf)) {
        pr_err("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        pr_err("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    pr_err(" %s: %s", __func__, proc_buf);

    if (!strncmp("1", proc_buf, 1)) {
        /* turn on charging limit in eng mode */
        b_eng_charging_limit = true;
        pr_err(" %s: turn on charging limit: %s", __func__, proc_buf);
    }
    else if (!strncmp("0", proc_buf, 1)) {
        /* turn off charging limit in eng mode */
        b_eng_charging_limit = false;
        pr_err(" %s: turn off charging limit: %s", __func__, proc_buf);
    }

    //aicl_dete_worker(NULL);
    //request_power_supply_changed();
    smbchg_eng_charging_limit(chip);

    return count;
}

static int nasus_charging_limit_toggle_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: ENG charging limit %s\n", __func__, b_eng_charging_limit ? "On" : "Off");
    return 0;
}

static int nasus_charging_limit_toggle_open(struct inode *inode, struct file *file)
{
    return single_open(file, nasus_charging_limit_toggle_read, NULL);
}

static int nasus_charging_threshold_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", charging_limit_threshold);
    pr_err("%s: %d\n", __func__, charging_limit_threshold);

    return 0;
}

static ssize_t nasus_charging_threshold_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[64];
    char backup_fbuf[4];
    char default_charging_limit[4];
    static bool backup_file_ready = false;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    int byte_count= 0;
    s32 res;

    if (count > sizeof(proc_buf)) {
        pr_smb(PR_STATUS, "data error\n");
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        pr_smb(PR_STATUS, "read data from user space error\n");
        return -EFAULT;
    }

    if (proc_buf[0] == 's') {

        pr_smb(PR_STATUS, "************ CHARGER LIMIT THRESHOLD start ************\n");

        /* partition is ready for writing/reading */
        backup_file_ready = true;

        /* read value from file */

        fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR|O_CREAT, 0666);
        if (!IS_ERR_OR_NULL(fp) ) {
            /* Get current segment descriptor and Set segment
            *  descriptor associated to kernel space.
            */
            old_fs = get_fs();
            set_fs(KERNEL_DS);

            /* read file */
            byte_count= fp->f_op->read(fp, backup_fbuf, sizeof(backup_fbuf), &fp->f_pos);
            pr_smb(PR_STATUS, "read data from file: %s\n", backup_fbuf);

            /* write default value to file if it's empty */
            if (!strncmp("", backup_fbuf, 1)) {
                pr_err(" %s: file exist but is empty! Write default to file\n", __func__);

                sprintf(default_charging_limit, "%d", charging_limit_threshold);
                fp->f_op->llseek(fp, 0, 0);
                fp->f_op->write(fp,
                    default_charging_limit,
                    sizeof(default_charging_limit),
                    &fp->f_pos);

                goto file_open_but_error;
            }

            /* give up if fail to transition */
            if (kstrtos32(backup_fbuf, 10, &res)) {
                pr_err("%s: kstrtos32 error!", __func__);

                goto file_open_but_error;
            }
            pr_smb(PR_STATUS, "read data from file(int): %d\n", res);

            /* replace the charging limit threshold with new value */
            charging_limit_threshold = res;

            /* Restore segment descriptor */
            set_fs(old_fs);

            /* Close file operation */
            filp_close(fp, NULL);
        }
        else {
            pr_err("%s: file open error (%s)\n", __func__, CHARGING_LIMIT_THRESHOLD_FILE);
            return -EFAULT;
        }
    }
    else {
        /* directly return if backup file is not ready */
        if (!backup_file_ready) {
            pr_err("%s: backup file not ready!\n", __func__);
            return count;
        }

        /* give up if fail to transition */
        if (kstrtos32_from_user(buf, count, 10, &res)) {
            pr_err("%s: kstrtos32 error!", __func__);
            pr_err("%s: proc_buf is %s", __func__, proc_buf);
            return -EFAULT;
        }

        /* replace the charging limit threshold with new value */
        if (0 <= res && res <= 100) {

            /* backup new value to file */
            fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE,
                    O_WRONLY | O_CREAT | O_TRUNC,
                    0666);
            if (!IS_ERR_OR_NULL(fp) ) {
                /* Get current segment descriptor and Set segment
                *  descriptor associated to kernel space.
                */
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                /* replace the charging limit threshold with new value */
                charging_limit_threshold = res;

                sprintf(default_charging_limit, "%d", charging_limit_threshold);
                fp->f_op->llseek(fp, 0, 0);
                fp->f_op->write(fp,
                    default_charging_limit,
                    sizeof(default_charging_limit),
                    &fp->f_pos);

                /* Restore segment descriptor */
                set_fs(old_fs);

                /* Close file operation */
                filp_close(fp, NULL);
            }
            else {
                pr_err("%s: file open error (%s)\n", __func__, CHARGING_LIMIT_THRESHOLD_FILE);
                return -EFAULT;
            }
        }
        else {
            pr_err("%s: out of normal range, skip it!\n", __func__);
            return -EFAULT;
        }
    }

    //aicl_dete_worker(NULL);

    return count;

file_open_but_error:
    /* Restore segment descriptor */
    set_fs(old_fs);

    /* Close file operation */
    filp_close(fp, NULL);
    return -EFAULT;
}

static int nasus_charging_threshold_open(struct inode *inode, struct file *file)
{
    return single_open(file, nasus_charging_threshold_read, NULL);
}

int init_eng_charging_limit_toggle(struct smbchg_chip *chip)
{
    struct proc_dir_entry *entry;
    struct proc_dir_entry *entry2;

    /* proc interface for charging limit threshold */
    static const struct file_operations asus_charging_threshold_fops = {
        .owner = THIS_MODULE,
        .open = nasus_charging_threshold_open,
        .read = seq_read,
        .write = nasus_charging_threshold_write,
        .llseek = seq_lseek,
        .release = single_release,
    };

    static const struct file_operations charging_limit_toggle_fops = {
        .owner = THIS_MODULE,
        .open = nasus_charging_limit_toggle_open,
        .read = seq_read,
        .write = nasus_charging_limit_toggle_write,
        .llseek = seq_lseek,
        .release = single_release,
    };
	
	entry = proc_create_data("driver/charger_limit_enable", 0666, NULL, &charging_limit_toggle_fops, chip);
    if (!entry) {
        pr_err("%s: Unable to create asus_charging_toggle\n", __func__);
        return -EINVAL;
    }

    entry2 = proc_create_data("driver/charging_limit_threshold", 0666, NULL, &asus_charging_threshold_fops, chip);
    if (!entry2) {
        pr_err("%s: Unable to create /proc/driver/charging_limit_threshold\n", __func__);
        return -EINVAL;
    }

    return 0;
}

int eng_charging_limit(bool b_stop_charging)
{
	int rc = 0;
			
	b_eng_stop_charging = b_stop_charging;
	
	smbchg_eng_charging_limit(chip_fg);
	
	return rc;
}
EXPORT_SYMBOL(eng_charging_limit);

#else // NOT ENG_BUILD

#define TWINSHEADDRAGON_CURRENT_MA	2000
static int smbchg_set_twinsheaddragon_current(struct smbchg_chip *chip)
{
	int rc = 0;
		
	if (!chip->batt_present) 
	{
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n", TWINSHEADDRAGON_CURRENT_MA);
		return -EINVAL;
	}
	
	rc = smbchg_set_high_usb_chg_current(chip, TWINSHEADDRAGON_CURRENT_MA);
	if (rc < 0)
	{
		pr_err("Couldn't set %d mA rc = %d ###\n", TWINSHEADDRAGON_CURRENT_MA, rc);
		return rc;
	}	
	pr_smb(PR_STATUS, "twinsheaddragon current  = %d mA ###\n", TWINSHEADDRAGON_CURRENT_MA);
	
	smbchg_rerun_aicl(chip);

	return rc;
}

static int twinsheaddragon_read(struct seq_file *s, void *v)
{	
	struct smbchg_chip *chip = (struct smbchg_chip *) s->private;
	seq_printf(s, "%s: Enable\n", __func__);
	smbchg_set_twinsheaddragon_current(chip);
	return 0;
}

static int twinsheaddragon_open(struct inode *inode, struct file *file)
{	
    return single_open(file, twinsheaddragon_read, PDE_DATA(inode));
}

int init_twinsheaddragon_toggle(struct smbchg_chip *chip)
{
    struct proc_dir_entry *entry;

    static const struct file_operations twinsheaddragon_fops = {
        .owner = THIS_MODULE,
        .open = twinsheaddragon_open,
        .read = seq_read,
        .write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create_data("driver/twinsheaddragon", 0666, NULL, &twinsheaddragon_fops, chip);
    if (!entry) 
    {
		pr_err("Unable to create twinsheaddragon\n");
        return -EINVAL;
    }

    return 0;
}
#if SUPPORT_DEMO_MODE_CHG_CTL
int vzw_demo_mode_control(bool b_stop_charging)
{
	int rc = 0;
	rc = smbchg_force_charging_limit(chip_fg, b_stop_charging);
	
	return rc;
}
EXPORT_SYMBOL(vzw_demo_mode_control);
#endif // SUPPORT_DEMO_MODE_CHG_CTL
#endif // NOT ENG_BUILD

#define JEITA_TEMP_HARD_LIMIT_EN	0x0		// 0 : Enable
#define JEITA_TEMP_HARD_LIMIT_DIS	BIT(5)	// 1 : Disable

#define HOT_COLD_SW_COMP_MASK		SMB_MASK(3, 0)
#define HOT_COLD_SL_FV_CHG_I_COMP	0x0000

int smbchg_jeita_init(struct smbchg_chip *chip)
{
	int rc = 0;
		
	// 1. Set JEITA_TEMP_HARD_LIMIT Enable
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT, 0x0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set jeita temp hard limit rc = %d\n", rc);
		return rc;
	}
	
	// 2. Set Software Limit about Hot/Cold Float Voltage Compensation and Hot/Cold Charge Curent Compensation
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CCMP_CFG,
			HOT_COLD_SW_COMP_MASK, HOT_COLD_SL_FV_CHG_I_COMP);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set Hot/Cold SW Float Voltage Compensation and Hot/Cold SWCharge Current Compensation rc = %d\n", rc);
		return rc;
	}

	return rc;
}
#define JEITA_LOG 0
int smbchg_jeita_control(struct smbchg_chip *chip, int ibat_capacity, int ibat_temp, int ibat_volt)
{
	int rc;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	u8 reg;
	
	//pr_smb(PR_STATUS, "%d %%, %d C, %d V ###\n", ibat_capacity, ibat_temp, ibat_volt);
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
#if !defined(CONFIG_ZT582KL)	
	chip_fg->fg_capacity = ibat_capacity;
	chip_fg->fg_voltage = ibat_volt;

	if(ibat_capacity == 100)
		b_full_capacity = true;

#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL) || defined(CONFIG_ZT582KL)
	if (dual_chg_flag == Undefined && ibat_capacity >= 1 && usb_supply_type != POWER_SUPPLY_TYPE_USB){
		queue_delayed_work(adapter_wq ,&chip->adapter_detect_work, msecs_to_jiffies(0));
#else
	if (chg_type_flag == UNDEFINED && ibat_capacity >= 1){
		detect_dedicated_adapter(chip);
#endif
		return 0;
	}
#endif

	//pr_smb(PR_STATUS, "inserted type = %d (%s)", usb_supply_type, usb_type_name);

	if(ibat_temp < 15){
		if(JEITA_LOG)
			pr_smb(PR_STATUS, "goto control_diagram_cold\n");
		goto control_diagram_cold;
	}
	
	if((ibat_temp >= 15) && (ibat_temp < 100))
	{
		if((!chip->stop_chg_via_temp_cool) ||
			((chip->stop_chg_via_temp_cold) && (ibat_temp > 45)))
		{
			chip->stop_chg_via_temp_cold = false;
			if(JEITA_LOG)
				pr_smb(PR_STATUS, "goto control_diagram_cool\n");
			goto control_diagram_cool;
		}
	}

#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	if((ibat_temp >= 100) && (ibat_temp < 200))
	{
		if(JEITA_LOG)
			pr_smb(PR_STATUS, "goto control_diagram_cool_II\n");
		goto control_diagram_cool_II;
	}
#endif
	
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	if((ibat_temp >= 200) && (ibat_temp < 500))
#else // defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
	if((ibat_temp >= 100) && (ibat_temp < 500))
#endif
	{
		if(((!chip->stop_chg_via_temp_cold) && (!chip->stop_chg_via_temp_cool) && 
			(!chip->stop_chg_via_temp_warm) && (!chip->stop_chg_via_temp_hot)) ||
			((chip->stop_chg_via_temp_cool) && (ibat_temp > 130)) ||
			((chip->stop_chg_via_temp_warm) && (ibat_temp < 470)) ||
			((chip->stop_chg_via_temp_cold) && (ibat_temp > 45)) ||
			((chip->stop_chg_via_temp_hot) && (ibat_temp <= 570)))
		{
			chip->stop_chg_via_temp_cold = false;
			chip->stop_chg_via_temp_cool = false;
			chip->stop_chg_via_temp_warm = false;
			chip->stop_chg_via_temp_hot = false;
			
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
//			pr_smb(PR_STATUS, "dual_chg_flag = %d, thermal_policy_flag =%d\n",dual_chg_flag,thermal_policy_flag);
			if (((dual_chg_flag == ASUS_2A) || (dual_chg_flag == TYPC_3A)) && (thermal_policy_flag == TP_LEVEL0)){ 
				if(JEITA_LOG)
					pr_smb(PR_STATUS, "goto control_diagram_normal_II\n");
				goto control_diagram_normal_II;
			}
			else{
				if(JEITA_LOG)
					pr_smb(PR_STATUS, "goto control_diagram_normal_I\n");
				goto control_diagram_normal_I;				
			}
#else // defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
			// Not need to support the normal I, II and III. Therefore, always jump to normal I
			goto control_diagram_normal_I;
#endif
		}
	}
	
	if((ibat_temp >= 500) && (ibat_temp < 600))
	{
		if((chip->stop_chg_via_temp_hot) && (ibat_temp <= 570))
		{
			chip->stop_chg_via_temp_hot = false;
		}

		if(!chip->stop_chg_via_temp_hot)
		{
			rc = smbchg_read(chip, &reg, chip->chgr_base + VFLOAT_CFG_REG, 1);
			if (rc) {
				dev_err(chip->dev, "Could not read Vfloat Config Register rc = %d\n", rc);
				goto control_diagram_warm_II;
			}
		
			if((ibat_volt >= 4100) && ((reg & VFLOAT_MASK) == VFLOAT_4P38)) {
				if(JEITA_LOG)
					pr_smb(PR_STATUS, "goto control_diagram_warm_I\n");
				goto control_diagram_warm_I;
			}
			else {
				if(JEITA_LOG)
					pr_smb(PR_STATUS, "goto control_diagram_warm_II\n");
				goto control_diagram_warm_II;
			}
		}
	}
	
	if(ibat_temp >= 600){
		if(JEITA_LOG)
			pr_smb(PR_STATUS, "goto control_diagram_hot\n");
		goto control_diagram_hot;
	}
	return rc;
	
control_diagram_cold: // temp < 1.5 Celsius
	pr_smb(PR_STATUS, "entry jeita cold mode, battery temperature < 1.5 Celsius ###\n");
	
	chip->stop_chg_via_temp_cold = true;
	
#ifdef SUPPORT_PARALLEL_CHG
	dual_chg_disable();

	if(smbchg_check_dual_status(chip) &&
			(dual_chg_flag == ASUS_2A || dual_chg_flag == TYPC_3A) &&
			hvdcp_flag == NO_HVDCP &&
			thermal_policy_flag == TP_LEVEL0)
	{
		// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
		if (rc < 0)
			pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
		// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
		
	}
#endif
#ifndef ENG_BUILD
	// 1. set CHG_EN_COMMAND = Disable , 0x1242[1] = "1"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Disable Fail rc = %d\n", rc);
		return rc;
	}
#endif	
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
					VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_1200MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
					FCC_MASK, FCC_1200MA_VAL);
#else // defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
					FCC_MASK, FCC_2300MA_VAL);
#endif
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_1200_MA, rc);
		return rc;
	}
	
	return POWER_SUPPLY_STATUS_DISCHARGING;
		
control_diagram_cool: // 1.5 <= temp < 10 Celsius
	pr_smb(PR_STATUS, "entry jeita cool mode, 1.5 Celsius <= battery temperature < 10 Celsius ###\n");

	chip->stop_chg_via_temp_cool = true;
	
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
control_diagram_cool_II:
#endif

#ifdef SUPPORT_PARALLEL_CHG
	//pr_smb(PR_STATUS, "entry jeita : control_diagram_cool_II ###\n");
	dual_chg_disable();

	if(smbchg_check_dual_status(chip) &&
			(dual_chg_flag == ASUS_2A || dual_chg_flag == TYPC_3A) &&
			hvdcp_flag == NO_HVDCP &&
			thermal_policy_flag == TP_LEVEL0)
	{
		// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
		if (rc < 0)
			pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
		// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
		
	}
#endif

#ifndef ENG_BUILD
	// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
		return rc;
	}
#endif
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
					VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_1200MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
					FCC_MASK, (ibat_temp >= 100) ? FCC_2300MA_VAL : FCC_2100MA_VAL);
#else
					FCC_MASK, FCC_1200MA_VAL);
#endif
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current in diagram_cool,  rc=%d\n", rc);
		return rc;
	}
	
	goto contorl_diagram_sw_recharge;

#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
control_diagram_normal_I: // (Single or Undefined) && (20 <= temp < 50 Celsius)
	//pr_smb(PR_STATUS, "entry jeita : control_diagram_normal_I ###\n");
	dual_chg_disable();

	// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
		return rc;
	}

	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2300MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2300MA_VAL);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2300_MA, rc);
		return rc;
	}

	goto contorl_diagram_sw_recharge;

control_diagram_normal_II: // (ASUS_2A) && (20 <= temp < 50 Celsius)
//control_diagram_normal_III: // (TYPC_3A) && (20 <= temp < 50 Celsius)
//	pr_smb(PR_STATUS, "entry jeita normal mode(ASUS_2A or TYPC_3A) , 20 Celsius <= battery temperature < 50 Celsius ###\n");
	
	if((ibat_volt >= 3000) && (ibat_volt <= 4300))
	{
		// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
			return rc;
		}
	
		// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
		if (rc < 0)
		{
			dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	
		// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2300MA , 0x10F2 = 0x1C
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2300MA_VAL);
		if (rc < 0)
		{
			dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2300_MA, rc);
			return rc;
		}

		// 4. Set Input Current Limt = 900mA , 0x13F2 = 0x09
	
		if (dual_chg_flag == ASUS_2A && thermal_policy_flag == TP_LEVEL0)
		{
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_900MA);
			if (rc < 0)
				pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			// Dual Enable 1 , smb1351_icl = 1000 , fast_chg = 2400
#ifdef SUPPORT_PARALLEL_CHG
			dual_chg_enable(AC_INPUT_CURRENT_LIMIT_1000MA, FAST_CHG_CURRENT_2400MA);
#endif
		}
		else if(dual_chg_flag == TYPC_3A && thermal_policy_flag == TP_LEVEL0)
		{
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1400MA);
			if (rc < 0)
				pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			// Dual Enable 2 , smb1351_icl = 1500 , fast_chg = 2400
#ifdef SUPPORT_PARALLEL_CHG
			dual_chg_enable(AC_INPUT_CURRENT_LIMIT_1500MA, FAST_CHG_CURRENT_2400MA);
#endif
		}
	}
	else
	{
		dual_chg_disable();

		if(smbchg_check_dual_status(chip) &&
				(dual_chg_flag == ASUS_2A || dual_chg_flag == TYPC_3A) &&
				hvdcp_flag == NO_HVDCP &&
				thermal_policy_flag == TP_LEVEL0)
		{
			// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
			if (rc < 0)
				pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
			// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
			if (rc < 0 )
				pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
			// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
			if (rc < 0 )
				pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);		
		}

		// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
			return rc;
		}

		// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
					VFLOAT_MASK, VFLOAT_4P38);
		if (rc < 0)
		{
			dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	
		// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2300MA
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2300MA_VAL);
		if (rc < 0)
		{
			dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2300_MA, rc);
			return rc;
		}
	}

	goto contorl_diagram_sw_recharge;

#else // defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
control_diagram_normal_I: // (Vbat <= 4V) && (20 <= temp < 45 Celsius)
//control_diagram_normal_II: // (20 <= temp < 45 Celsius) && (4V < Vbat <= 4.2V)
//control_diagram_normal_III: // ((10 <= temp < 20 Celsius) || (45 <= temp < 50 Celsius)) && (4.2V < Vbat)
	if(JEITA_LOG)
		pr_smb(PR_STATUS, "entry jeita normal mode, 10 Celsius <= battery temperature < 50 Celsius ###\n");
	
	// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
		return rc;
	}
	
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2100MA , 0x10F2 = 0x1B
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG, FCC_MASK, FCC_2100MA_VAL);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2100_MA, rc);
		return rc;
	}
	
	goto contorl_diagram_sw_recharge;
#endif	

control_diagram_warm_I: // ( 50 <= temp < 60 Celsius ) && ( 4.1V <= Vbat )
	pr_smb(PR_STATUS, "entry jeita warm I mode, 50 Celsius <= battery temperature < 60 Celsius ###\n");

	chip->stop_chg_via_temp_warm = true;
	
#ifdef SUPPORT_PARALLEL_CHG
	dual_chg_disable();

	if(smbchg_check_dual_status(chip))
	{
		// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
		if (rc < 0)
			pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
		// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
		
	}
#endif	

#ifndef ENG_BUILD
	// 1. set CHG_EN_COMMAND = Disable , 0x1242[1] = "1"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Disable Fail rc = %d\n", rc);
		return rc;
	}
#endif	
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG, VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2100MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
					FCC_MASK, FCC_2300MA_VAL);
#else
					FCC_MASK, FCC_2100MA_VAL);
#endif
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2100_MA, rc);
		return rc;
	}
	
	return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_warm_II: // ( 50 <= temp < 60 Celsius) || ( Vbat >= 4.1V )
	pr_smb(PR_STATUS, "entry jeita warm II mode, 50 Celsius <= battery temperature < 60 Celsius ###\n");
	
	chip->stop_chg_via_temp_warm = true;
	
#ifdef SUPPORT_PARALLEL_CHG
	dual_chg_disable();
	
	if(smbchg_check_dual_status(chip))
	{
		// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
		if (rc < 0)
			pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
		// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
		
	}
#endif

	// 1. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
	
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
		return rc;
	}
	
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P10 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
					VFLOAT_MASK, VFLOAT_4P10);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2100MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
					FCC_MASK, FCC_2300MA_VAL);
#else
					FCC_MASK, FCC_2100MA_VAL);
#endif
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2100_MA, rc);
		return rc;
	}
	
	return POWER_SUPPLY_STATUS_CHARGING;
	
control_diagram_hot: // 60 Celsius <= temp
	pr_smb(PR_STATUS, "entry jeita hot mode, 60 Celsius < battery temperature ###\n");

	chip->stop_chg_via_temp_hot = true;

#ifdef SUPPORT_PARALLEL_CHG
	dual_chg_disable();

	if(smbchg_check_dual_status(chip))
	{
		// 1. Set Input Current Limt = 1800mA , 0x13F2 = 0x12
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG, USBIN_INPUT_MASK, USBIN_IL_1800MA);
		if (rc < 0)
			pr_smb(PR_STATUS, "Couldn't set USBIN_IL rc = %d\n", rc);
			
		// 2. Disable USBIN_AICL , 0x13F3[2] = "0"
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Disable AICL rc = %d\n", rc);
		
		// 3. Enable USBIN_AICL , 0x13F3[2] = "1"	
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, AICL_EN_BIT);
		if (rc < 0 )
			pr_smb(PR_STATUS, "Couldn't Enable AICL rc = %d\n", rc);
		
	}
#endif

#ifndef ENG_BUILD
	// 1. set CHG_EN_COMMAND = Disable , 0x1242[1] = "1"
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "set CHG_EN_COMMAND to Disable Fail rc = %d\n", rc);
		return rc;
	}
#endif
	
	// 2. set SMBCHGL_CHGR_FV_CFG = FV_4P38 , 0x10F4 = 0x2D
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
					VFLOAT_MASK, VFLOAT_4P38);
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}
	
	// 3. set SMBCHGL_CHGR_FCC_CFG = FCC_2100MA
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
					FCC_MASK, FCC_2300MA_VAL);
#else
					FCC_MASK, FCC_2100MA_VAL);
#endif
	if (rc < 0)
	{
		dev_err(chip->dev, "Couldn't set fast charge current %dmA rc=%d\n", CURRENT_2100_MA, rc);
		return rc;
	}
	
	return POWER_SUPPLY_STATUS_DISCHARGING;

contorl_diagram_sw_recharge:

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	if(ibat_capacity > 97)
		pr_smb(PR_STATUS, "REG 100E = %x, b_full_capacity = %d\n",reg,b_full_capacity);
//	if((reg & CHG_DONE_STS) && (ibat_capacity <= BATT_RE_CHG_LEVEL))
	if(b_full_capacity && (ibat_capacity <= BATT_RE_CHG_LEVEL))
	{
		pr_smb(PR_STATUS, "Battery level is re charging level, so need recharging !\n");
		b_full_capacity = false;
		// 1. set CHG_EN_COMMAND = Disable , 0x1242[1] = "1"
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, EN_BAT_CHG_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "set CHG_EN_COMMAND to Disable Fail rc = %d\n", rc);
			return rc;
		}

		// 2. set CHG_EN_COMMAND = Enable , 0x1242[1] = "0"
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG, EN_BAT_CHG_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev, "set CHG_EN_COMMAND to Enable Fail rc = %d\n", rc);
			return rc;
		}
		
		return POWER_SUPPLY_STATUS_CHARGING;	
	}

	return rc;
}


int jeita_control(int i_bat_capacity, int i_bat_temp, int i_bat_volt)
{
	int rc = 0;

	chip_fg->fg_capacity = i_bat_capacity;
	chip_fg->fg_voltage = i_bat_volt;

	if(i_bat_capacity == 100)
		b_full_capacity = true;

#if defined(CONFIG_ZT582KL)
	if (dual_chg_flag == Undefined && i_bat_capacity >= 1){
		queue_delayed_work(adapter_wq ,&chip_fg->adapter_detect_work, msecs_to_jiffies(0));
		return rc;
	}
#endif
	rc = smbchg_jeita_init(chip_fg);
#ifdef SUPPORT_PARALLEL_CHG
	rc = parallel_chg_jeita_init_settings();
#endif

	rc = smbchg_jeita_control(chip_fg, i_bat_capacity, i_bat_temp, i_bat_volt);
	
	return rc;
}
EXPORT_SYMBOL(jeita_control);

static int smbchg_check_chg_version(struct smbchg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		pr_err("Unable to get pmic_revid rc=%ld\n",
				PTR_ERR(pmic_rev_id));
		return -EPROBE_DEFER;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA
				| SMBCHG_BATT_OV_WA
				| SMBCHG_CC_ESR_WA;
		use_pmi8994_tables(chip);
		chip->schg_version = QPNP_SCHG;
		break;
	case PMI8950:
		chip->wa_flags |= SMBCHG_BATT_OV_WA;
		if (pmic_rev_id->rev4 < 2) /* PMI8950 1.0 */ {
			chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		} else	{ /* rev > PMI8950 v1.0 */
			chip->wa_flags |= SMBCHG_HVDCP_9V_EN_WA
					| SMBCHG_USB100_WA;
		}
		use_pmi8994_tables(chip);
		chip->tables.aicl_rerun_period_table =
				aicl_rerun_period_schg_lite;
		chip->tables.aicl_rerun_period_len =
			ARRAY_SIZE(aicl_rerun_period_schg_lite);

		chip->schg_version = QPNP_SCHG_LITE;
		break;
	case PMI8996:
		chip->wa_flags |= SMBCHG_CC_ESR_WA
				| SMBCHG_FLASH_ICL_DISABLE_WA;
		use_pmi8996_tables(chip);
		chip->schg_version = QPNP_SCHG;
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}
	chip->allow_hvdcp3_detection = true;

	pr_smb(PR_STATUS, "pmic=%s, wa_flags=0x%x\n",
			pmic_rev_id->pmic_name, chip->wa_flags);

	return 0;
}
#if defined(ENG_BUILD)
static void smbchg_charger_limit_work(struct work_struct *work){
//	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, charger_limit_work.work);
	struct file *fp = NULL;
	mm_segment_t old_fs;
	static char b_buf[sizeof(int)];
	int byte_count = 0;
	s32 res;

	pr_smb(PR_STATUS, "************ CHARGER LIMIT THRESHOLD read from file ************\n");
	fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR|O_CREAT, 0666);
	if (!IS_ERR_OR_NULL(fp) ) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		/* read file */
		byte_count= fp->f_op->read(fp, b_buf, sizeof(b_buf), &fp->f_pos);
		pr_smb(PR_STATUS, "read data from file: %s\n", b_buf);

		/* using default 60 if it's empty */
		if (!strncmp("", b_buf, 1)) 
			goto file_open_but_error;

		if (kstrtos32(b_buf, 10, &res)) {
			pr_err("%s: kstrtos32 error!", __func__);
			goto file_open_but_error;
		}
		pr_smb(PR_STATUS, "read data from file(int): %d\n", res);

		/* replace the charging limit threshold with new value */
		charging_limit_threshold = res;

		/* Restore segment descriptor */
		set_fs(old_fs);

		/* Close file operation */
		filp_close(fp, NULL);			
	}
	else{
		pr_err("%s: file open error (%s)\n", __func__, CHARGING_LIMIT_THRESHOLD_FILE);
		return;
	}

	return;
file_open_but_error:
	/* Restore segment descriptor */
	set_fs(old_fs);

	/* Close file operation */
	filp_close(fp, NULL);
	return;
}
#endif

//#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
static void smbchg_adapter_detect_work(struct work_struct *work){
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip, adapter_detect_work.work);
	int i, rc;
	enum power_supply_type usb_supply_type;

	get_usb_type(&usb_supply_type);
#if !defined(CONFIG_ZT582KL)
	if (is_hvdcp_present(chip)){
		pr_smb(PR_STATUS, "usb_supply_type = %d \n", usb_supply_type);
		if(usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP)
			hvdcp_flag = HVDCP_2;
		if(usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
			hvdcp_flag = HVDCP_3;
	}
	chip->charging_type = CHG_TYPE_DCP;
	if((chip_fg->fg_voltage > BATT_LOW_VOLT) || (chip_fg->fg_capacity > BATT_LOW_LEVEL))
#else
	pr_smb(PR_STATUS, "usb_supply_type = %d \n", usb_supply_type);
	chip->charging_type = CHG_TYPE_DCP;
	if(chip_fg->fg_capacity >= BATT_LOW_LEVEL)
#endif
	{	
		chip->hw_id = Read_HW_ID();
		chip->proj_id = Read_PROJ_ID();
		pr_smb(PR_STATUS, "HW_ID = %d, PROJ_ID = %d \n", chip->hw_id, chip->proj_id);

		// detect exclusive adapter support ER2, PR and later HW version
		detect_dedicated_adapter(chip);

		if (hvdcp_flag != NO_HVDCP || dual_chg_flag == ASUS_2A){
			/* DCP */
			if(hvdcp_flag == NO_HVDCP && dual_chg_flag != ASUS_2A){
				pr_smb(PR_STATUS," charger type = DCP!\n");
				boost_up_voltage_done = true;
				return;
			}
	
			/* HVDCP or ASUS_2A */
			if (is_hvdcp_present(chip)){
				/* if ASUS_2A show + icon */
				if(dual_chg_flag == ASUS_2A)
					set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,POWER_SUPPLY_STATUS_QC_V2_CHARGING);
				chip->hvdcp_3_det_ignore_uv = true;
				rerun_apsd(chip);
				chip->hvdcp_3_det_ignore_uv = false;
				if (hvdcp_flag == HVDCP_2){
					msleep(3000);
	
					/* Force 9V HVDCP */
					rc = smbchg_sec_masked_write(chip,
						chip->usb_chgpth_base + CHGPTH_CFG,
						HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
					if (rc)
						pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
				}
				else{
					msleep(3000);
					smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_PREPARE);
					for (i=0;i<20;i++){
						smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_DP_PULSE);
						msleep(50);
					}
					smbchg_dp_dm(chip, POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
				}
			}
		}
		else{
			if (dfp_type_flag == Others){
				chg_type_flag = OTHERS_1A;
			}
			else{
				/*TYPC DFP setting*/
			}
		}			
	}
	else
	{
		pr_smb(PR_STATUS, "Now status is battery low, so not need to detect adapter whether legal !\n");
		chip->dedicated_adapter = false;
		dual_chg_flag = Undefined;
	}
	boost_up_voltage_done = true;

	if (wake_lock_active(&alarm_cable_lock)){
		pr_err("[%s] Cable WakeLock: *UNLOCK*\n", __func__);
		wake_unlock(&alarm_cable_lock);
	}
}
//#endif

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct qpnp_vadc_chip *vadc_dev, *vchg_vadc_dev;
#ifdef SUPPORT_PD_ADAPTER
	struct power_supply *usb_psy, *pd_psy;
#else
	struct power_supply *usb_psy;
#endif
#if defined(CONFIG_ZT582KL)
	int err = 0;
#endif

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

#ifdef SUPPORT_PD_ADAPTER
#if defined(CONFIG_ZT500KL)
	if (of_find_property(spmi->dev.of_node, "ext_power_supply-cap-PD", NULL)) {
		pd_psy = power_supply_get_by_name("power_delivery");
		if (!pd_psy) {
			pr_smb(PR_STATUS, "PD supply not found, deferring probe\n");
			return -EPROBE_DEFER;
		}
	} else {
		pd_psy = NULL;
	}
#else
	pd_psy = NULL;
#endif
#endif
	if (of_find_property(spmi->dev.of_node, "qcom,dcin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "dcin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,vchg_sns-vadc", NULL)) {
		vchg_vadc_dev = qpnp_get_vadc(&spmi->dev, "vchg_sns");
		if (IS_ERR(vchg_vadc_dev)) {
			rc = PTR_ERR(vchg_vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc 'vchg' rc=%d\n",
						rc);
			return rc;
		}
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	
	chip_fg = chip;
#if defined(CONFIG_ZT582KL)
	chip->INDCAT_EN = of_get_named_gpio(spmi->dev.of_node, "INDCAT_EN", 0);
	err = gpio_request(chip->INDCAT_EN, "LEDS_INDCAT_EN");
	if (err < 0)
		dev_err(&spmi->dev, "Unable to request gpio LEDS_INDCAT_EN for tca6507\n");
	else
		gpio_direction_output(chip->INDCAT_EN, 1);
#endif
	chip->fcc_votable = create_votable(&spmi->dev,
			"SMBCHG: fcc",
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
			VOTE_MIN, NUM_FCC_VOTER, 2300,
#else
			VOTE_MIN, NUM_FCC_VOTER, 2000,
#endif
			set_fastchg_current_vote_cb);
	if (IS_ERR(chip->fcc_votable))
		return PTR_ERR(chip->fcc_votable);

	chip->usb_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_usb_current_limit_vote_cb);
	if (IS_ERR(chip->usb_icl_votable))
		return PTR_ERR(chip->usb_icl_votable);

	chip->dc_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: dcl_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_dc_current_limit_vote_cb);
	if (IS_ERR(chip->dc_icl_votable))
		return PTR_ERR(chip->dc_icl_votable);

	chip->usb_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			usb_suspend_vote_cb);
	if (IS_ERR(chip->usb_suspend_votable))
		return PTR_ERR(chip->usb_suspend_votable);

	chip->dc_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: dc_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			dc_suspend_vote_cb);
	if (IS_ERR(chip->dc_suspend_votable))
		return PTR_ERR(chip->dc_suspend_votable);

	chip->battchg_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: battchg_suspend",
			VOTE_SET_ANY, NUM_BATTCHG_EN_VOTERS, 0,
			charging_suspend_vote_cb);
	if (IS_ERR(chip->battchg_suspend_votable))
		return PTR_ERR(chip->battchg_suspend_votable);
	adapter_wq = create_singlethread_workqueue("adapter_wq");
	chip->smbchg_work_queue = create_singlethread_workqueue("smbchg_wq");
	if (!chip->smbchg_work_queue) {
		pr_err("fail to create smbchg_wq\n");
		return -ENOMEM;
	}
	chip->smbchg_src_detect_queue = create_singlethread_workqueue("smbchg_src_detect_queue");
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	INIT_DEFERRABLE_WORK(&chip->pule_down_5v_work, smbchg_pule_down_5v_work);
#endif
	INIT_DEFERRABLE_WORK(&chip->adapter_detect_work, smbchg_adapter_detect_work);
	INIT_DEFERRABLE_WORK(&chip->cable_poweron_work, smbchg_cable_poweron_work);
	INIT_DEFERRABLE_WORK(&chip->usb_insertion_work, smbchg_usb_insertion_work);
	INIT_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
#if 0 //remove qualcomm parallel charge
	INIT_DELAYED_WORK(&chip->parallel_en_work, smbchg_parallel_usb_en_work);
#endif
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);

#if defined(ENG_BUILD)
	INIT_DELAYED_WORK(&chip->charger_limit_work, smbchg_charger_limit_work);
#endif
	INIT_DELAYED_WORK(&chip->asus_sdp_delayed_work, asus_smbchg_SDP_setting_worker);
#ifdef SUPPORT_9V_HVDCP	
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
#endif
	init_completion(&chip->sdp_retry_complete_flag);
	init_completion(&chip->src_det_lowered);
	init_completion(&chip->src_det_raised);
	init_completion(&chip->usbin_uv_lowered);
	init_completion(&chip->usbin_uv_raised);
	chip->vadc_dev = vadc_dev;
	chip->vchg_vadc_dev = vchg_vadc_dev;
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
#ifdef SUPPORT_PD_ADAPTER
	chip->pd_psy = pd_psy;
#endif
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->therm_lvl_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->taper_irq_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->wipower_config);
	mutex_init(&chip->usb_status_lock);
	device_init_wakeup(chip->dev, true);
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL) || defined(CONFIG_ZT582KL)
	/* init wake lock */
	wake_lock_init(&alarm_cable_lock,WAKE_LOCK_SUSPEND, "alarm_cable_wakelock");
#endif
	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}

	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		goto free_regulator;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto free_regulator;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto free_regulator;
	}

	chip->previous_soc = -EINVAL;
	chip->batt_psy.name		= chip->battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto free_regulator;
	}
	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;

	if (chip->cfg_chg_led_support &&
			chip->schg_version == QPNP_SCHG_LITE) {
		rc = smbchg_register_chg_led(chip);
		if (rc) {
			dev_err(chip->dev,
					"Unable to register charger led: %d\n",
					rc);
			goto unregister_dc_psy;
		}

		rc = smbchg_chg_led_controls(chip);
		if (rc) {
			dev_err(chip->dev,
					"Failed to set charger led controld bit: %d\n",
					rc);
			goto unregister_led_class;
		}
	}

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_led_class;
	}

	if (!chip->skip_usb_notification) {
		pr_smb(PR_STATUS, "setting usb psy present = %d\n",
			chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}
#if defined(CONFIG_ZT581KL) || defined(CONFIG_Z581KL)
	chip->charging_type = CHG_TYPE_UNDEFINED;
#endif
	chip->stop_chg_via_temp_cold = false;
	chip->stop_chg_via_temp_cool = false;
	chip->stop_chg_via_temp_warm = false;
	chip->stop_chg_via_temp_hot = false;
	chip->dedicated_adapter = false;
	chip->dedicated_power_bank = false;

	DPM_SW_enable(false);
	ADCPWR_enable(false);

#ifdef ENG_BUILD
	init_eng_charging_limit_toggle(chip);
#else // NOT ENG_BUILD
	init_twinsheaddragon_toggle(chip);
#endif

        if (!chip->skip_usb_notification) {
                pr_smb(PR_STATUS, "setting usb psy present = %d\n",
                        chip->usb_present);
                power_supply_set_present(chip->usb_psy, chip->usb_present);
        }
#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	g_probe = false;
#else
	dump_regs(chip);
#endif
#ifdef ENG_BUILD
	schedule_delayed_work(&chip->charger_limit_work, msecs_to_jiffies(15000));
#endif
	create_demochg_proc_file();
	create_debugfs_entries(chip);
	dev_info(chip->dev,
		"SMBCHG successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d\n",
			version_str[chip->schg_version],
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);
	return 0;

unregister_led_class:
	if (chip->cfg_chg_led_support && chip->schg_version == QPNP_SCHG_LITE)
		led_classdev_unregister(&chip->led_cdev);
unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
free_regulator:
	smbchg_regulator_deinit(chip);
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(chip->debug_root);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);

	power_supply_unregister(&chip->batt_psy);
	smbchg_regulator_deinit(chip);

	return 0;
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
};

static int __init smbchg_init(void)
{
//#if !defined(CONFIG_Z581KL)
	int ret = 0;
//#endif
#if defined(ENG_BUILD)
	ret = platform_device_register(&charger_limit_space_device);
	if (ret)
		pr_smb(PR_MISC, "[charger_limit] Unable to device register (%d)\n", ret);

	ret = platform_driver_register(&charger_limit_space_driver);
	if (ret)
		pr_smb(PR_MISC, "[charger_limit] Unable to register driver (%d)\n", ret);
#endif

#if defined(VZW)
	ret = platform_device_register(&q023_space_device);
	if (ret) 
		pr_smb(PR_MISC, "[Q023] Unable to device register (%d)\n", ret);

	ret = platform_driver_register(&q023_space_driver);
	if (ret)
		pr_smb(PR_MISC, "[Q023] Unable to register driver (%d)\n", ret);
#endif

//#if defined(CONFIG_ZT500KL) || defined(CONFIG_Z500KL)
	ret = platform_device_register(&rerun_apsd_space_device);
	if (ret) 
		pr_smb(PR_MISC, "[RERUN_APSD] Unable to device register (%d)\n", ret);

	ret = platform_driver_register(&rerun_apsd_space_driver);
	if (ret)
		pr_smb(PR_MISC, "[RERUN_APSD] Unable to register driver (%d)\n", ret);
//#endif
	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
