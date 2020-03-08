#include <linux/gcd.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/msm_mdp.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include "mdss_debug.h"

#include "mdss_dsi_iris2.h"
#include "mdss_dsi_iris2_def.h"
#include "mdss_dsi_iris2_extern.h"
#include "mdss_dsi_iris2_ioctl.h"

#define LPMeMcLevel   (5)

#define IRIS_CONFIGURE_GET_VALUE_CORRECT 0
#define IRIS_CONFIGURE_GET_VALUE_ERROR 1

static uint16_t m_sLPMeMCLevelSelection_16to10[LPMeMcLevel][2] = {
	{1728, 1080},  //16:10
	{1280, 800},    //16:10
	{1024, 640},
	{640, 400},
	{480, 300}
};

static uint16_t m_sLPMeMCLevelSelection_16to9[LPMeMcLevel][2] = {
	{1920, 1080},  //16:9
	{1280, 720},    //16:9
	{1024, 576},
	{640, 360},
	{480, 270}
};

static uint16_t m_sLPMeMCLevelSelection_4to3[LPMeMcLevel][2] = {
	{1200, 900},//{1600, 1200},   //4:3
	{1024, 768},  //4:3
	{800, 600},    //4:3
	{640, 480},
	{480, 360}
};


#define LPRatio_NUM  (8)
//----- 6/8, 5/8, 4/8, 3/8, 2/8---
static uint16_t m_sLPMeMcRation720Over[LPMeMcLevel] = {
	8,
	6,
	5,
	4,
	3
};

static uint16_t m_sLPMeMcRationSmall[LPMeMcLevel] = {
	8,
	8,
	6,
	5,
	4
};

static struct completion iris_vsync_comp;

static void mdss_iris_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	pr_debug("#### %s:%d vtime=%lld\n", __func__, __LINE__, vtime.tv64);
	complete(&iris_vsync_comp);
}

static struct mdss_mdp_vsync_handler iris_vsync_handler = {
	.vsync_handler = mdss_iris_vsync_handler,
};

static int iris_wait_for_vsync(struct mdss_mdp_ctl *ctl)
{
	int rc;

	pr_debug("#### %s:%d\n", __func__, __LINE__);
	init_completion(&iris_vsync_comp);
	ctl->ops.add_vsync_handler(ctl, &iris_vsync_handler);
	rc = wait_for_completion_interruptible_timeout(
		&iris_vsync_comp, msecs_to_jiffies(100));
	ctl->ops.remove_vsync_handler(ctl, &iris_vsync_handler);
	if (rc < 0)
		pr_err("#### %s:%d: error %d\n", __func__, __LINE__, rc);
	else if (rc == 0) {
		pr_debug("#### %s:%d: timeout\n", __func__, __LINE__);
		rc = -ETIMEDOUT;
	}
	return rc;
}

static int irisDsiStsGet(struct mdss_dsi_ctrl_pdata *ctrl)
{
	//8094_TODO;
	return 0;
}

u32 iris2_lp_memc_calc(u32 value)
{
	struct iris_timing_para *iris_timing;
	//u8 uratio4to3;
	uint32_t uHres;//, uVres;
	uint32_t uFrcLPTiming = (uint32_t)iris_LPMeMcTiming[1] << 16 | (uint32_t)iris_LPMeMcTiming[0];

	iris_timing = &(iris_mipi_info.iris_in_timing);
	uHres = iris_mipi_info.mipi_mode.rx_ch ? (uint32_t)iris_timing->hres * 2 : (uint32_t)iris_timing->hres;

	if (value >= LPMeMcLevel) {
		value = LPMeMcLevel - 1;
		pr_debug("#### %s:%d, Low Power MEMC level is out of range.\n", __func__, __LINE__);
	}
	pr_debug("#### %s:%d, Low Power MEMC level uHres = %d, vres = %d, rx_ch = %d.\n", __func__, __LINE__, uHres, iris_timing->vres, iris_mipi_info.mipi_mode.rx_ch);
	if (((uint32_t)uHres * (uint32_t)iris_timing->vres) >= ((uint32_t) IMG1080P_HSIZE * (uint32_t) IMG1080P_VSIZE)) {
		if (uHres > iris_timing->vres) {
			if ((uHres / 4)  == (iris_timing->vres / 3)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value-1][1];
				}
			} else if ((uHres / 16)  == (iris_timing->vres / 10)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value-1][1];
				}
			} else {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value-1][1];
				}
			}
		} else {
			if ((uHres / 3)  == (iris_timing->vres / 4)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_4to3[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_4to3[value-1][0];
				}
			} else if ((uHres / 10)  == (iris_timing->vres / 16)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_16to10[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_16to10[value-1][0];
				}
			} else {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_16to9[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_16to9[value-1][0];
				}
			}
		}
	} else if ((uHres * (uint32_t)iris_timing->vres) > ((uint32_t) IMG720P_HSIZE * (uint32_t) IMG720P_VSIZE)) {
		iris_LPMeMcTiming[0] = (uint32_t)uHres * m_sLPMeMcRation720Over[value] / LPRatio_NUM;
		iris_LPMeMcTiming[1] = (uint32_t)iris_timing->vres * m_sLPMeMcRation720Over[value] / LPRatio_NUM;
	} else {
		iris_LPMeMcTiming[0] = (uint32_t)uHres * m_sLPMeMcRationSmall[value] / LPRatio_NUM;
		iris_LPMeMcTiming[1] = (uint32_t)iris_timing->vres * m_sLPMeMcRationSmall[value] / LPRatio_NUM;
	}

	uFrcLPTiming = (uint32_t)iris_LPMeMcTiming[1] << 16 | (uint32_t)iris_LPMeMcTiming[0];
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	guFrcLPTiming = uFrcLPTiming;
#endif
	pr_debug("#### %s:%d,wHres: %d, wVres: %d, uFrcLPTiming: 0x%x\n", __func__, __LINE__, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1], uFrcLPTiming);

	return uFrcLPTiming;
}


static int iris_configure(struct msm_fb_data_type *mfd, u32 type, u32 value)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	u32 configAddr = 0;
	u32 configValue = 0;

	pr_debug("iris_configure: %d - 0x%x\n", type, value);

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	mutex_lock(&iris_cfg->config_mutex);
	switch (type) {
	case IRIS_PEAKING:
		pq_setting_current.peaking = value & 0xf;
		pq_setting_current.peakingUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_SHARPNESS:
		pq_setting_current.sharpness = value & 0xf;
		pq_setting_current.sharpnessUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_MEMC_DEMO:
		pq_setting_current.memcDemo = value & 0x3;
		pq_setting_current.memcDemoUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_PEAKING_DEMO:
		pq_setting_current.peakingDemo = value & 0x3;
		pq_setting_current.peakingDemoUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_GAMMA:
		pq_setting_current.gamma = value & 0x3;
		pq_setting_current.gammeUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_MEMC_LEVEL:
		pq_setting_current.memcLevel = value & 0x3;
		pq_setting_current.memcLevelUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_CONTRAST:
		pq_setting_current.contrast = value & 0xff;
		pq_setting_current.contrastUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_BRIGHTNESS:
		dbc_setting_current.brightness = value & 0x7f;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_EXTERNAL_PWM:
		dbc_setting_current.externalPWM = value & 0x1;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DBC_QUALITY:
		dbc_setting_current.dbcQuality = value & 0xf;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DLV_SENSITIVITY:
		dbc_setting_current.dlvSensitivity = value & 0xfff;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DBC_CONFIG:
		dbc_setting_current = *((struct iris_dbc_setting *)&value);
		dbc_setting_update = true;
		break;
	case IRIS_PQ_CONFIG:
		pq_setting_current = *((struct iris_pq_setting *)&value);
		pq_setting_update = value;
		break;
	case IRIS_LPMEMC_CONFIG:
		LPMemc_setting_current.level = value;
		LPMemc_setting_current.value = iris2_lp_memc_calc(value);
		LPMemc_setting_update = true;
		break;
	case IRIS_DCE_LEVEL:
		dbc_setting_current.DCELevel = value & 0x0f;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_BLACK_BORDER:
		black_border_value = value != 0;
		black_border_update = true;
		break;
	case IRIS_CINEMA_MODE:
		pq_setting_current.cinema = value & 0x01;
		pq_setting_update = true;
		break;
	case IRIS_COLOR_ADJUST:
		color_adjust_current_value = value & 0xff;
		color_adjust_update = true;
		break;
	default:
		mutex_unlock(&iris_cfg->config_mutex);
		return -EINVAL;
	}

	//update dbc mode
	if (dbc_setting_update) {
		if (dbc_setting_current.dbcQuality == 5)
			iris_dbc_mode &= ~(1 << 1);
		else
			iris_dbc_mode |= 1 << 1;
		if (dbc_setting_current.dlvSensitivity == 0)
			iris_dbc_mode &= ~1;
		else
			iris_dbc_mode |= 1;
	}

	// other mode, use meta method
	if (iris_cfg->sf_notify_mode != IRIS_MODE_RFB) {
		mutex_unlock(&iris_cfg->config_mutex);
		return 0;
	}

	// PQ setting, MB3
	if (pq_setting_update) {
		configValue = *((u32 *)&pq_setting_current);
		configAddr = IRIS_PQ_SETTING_ADDR;
		pq_setting_update = false;
	} else if (dbc_setting_update) {
		configValue = *((u32 *)&dbc_setting_current);
		configAddr = IRIS_PROXY_ADDR + 0x28;
		dbc_setting_update = false;
	} else if (LPMemc_setting_update) {
		configValue = LPMemc_setting_current.value | 0x80000000;
		configAddr = IRIS_PROXY_ADDR + 0x20;
		LPMemc_setting_update = false;
	} else if (black_border_update) {
		configValue = black_border_value ? 0x100c201f : 0x1006201f;
		configAddr = IRIS_MVC_ADDR + 0x17c;
		black_border_update = false;
	} else if (color_adjust_update) {
		configValue = ( u32 )color_adjust_current_value;
		configAddr = IRIS_PROXY_ADDR + 0x30;
		color_adjust_update = false;
	}
	mutex_unlock(&iris_cfg->config_mutex);

	if (0 == configValue && 0 == configAddr) {
		pr_warn(" no configValue and configAddr specified, possibly wrong type(%d)!\n", type);
		return -EINVAL;
	}

	return iris_register_write(mfd, configAddr, configValue);
}


static int iris_configure_ex(struct msm_fb_data_type *mfd, u32 type, u32 count, u32 *values)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct demo_win_info *pdemo_win_info;
	int width, height, frcEndX, frcEndY, frcStartX, frcStartY;
	int color = 0, colsize = 0, rowsize = 0, modectrl = 0x3f00, peakingctrl = 0, winstart = 0, winend = 0;
	int displaywidth = (iris_mipi_info.mipi_mode.tx_ch  ?  iris_mipi_info.iris_out_timing.hres * 2 : iris_mipi_info.iris_out_timing.hres);
	int ret;

	pdemo_win_info = (struct demo_win_info *)values;
	memcpy(&demo_win_info_setting, values, sizeof(struct demo_win_info));
	pr_debug("%s: startx =%x, starty=%x, endx=%x, endy=%x, color=%x, boardwidth=%x, MEMCdemoEn = %x, peakingdemoEn = %x\n",
			__func__, demo_win_info_setting.startX,
			demo_win_info_setting.startY, demo_win_info_setting.endX, demo_win_info_setting.endY,
			demo_win_info_setting.color, demo_win_info_setting.BorderWidth,
			demo_win_info_setting.MEMCEn, demo_win_info_setting.SharpnessEn);
	if (displaywidth < 100 || iris_mipi_info.iris_out_timing.vres < 100) {
		pr_err("panel size too small!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->startX >  displaywidth ||
			pdemo_win_info->startY >  iris_mipi_info.iris_out_timing.vres) {
		pr_err("user defined window start point over range!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->endX >  displaywidth ||
		pdemo_win_info->endY >  iris_mipi_info.iris_out_timing.vres) {
		pr_err("user defined end point over range!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->startX >  pdemo_win_info->endX ||
		pdemo_win_info->startY >  pdemo_win_info->endY) {
		pr_err("user defined start point > end point!\n");
		return -EINVAL;
	}

	pr_debug("iris_cfg->nrv_enable: %d\n", iris_cfg->nrv_enable);
	if (iris_cfg->nrv_enable) {
		width = iris_cfg->meta.nrv.captureRight - iris_cfg->meta.nrv.captureLeft;
		height = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
		frcStartX = pdemo_win_info->startX * width/displaywidth;
		frcStartY = pdemo_win_info->startY * height/iris_mipi_info.iris_out_timing.vres;
		frcEndX = pdemo_win_info->endX * width/displaywidth;
		frcEndY = pdemo_win_info->endY * height/iris_mipi_info.iris_out_timing.vres;
	} else {
		frcStartX = pdemo_win_info->startX * iris_LPMeMcTiming[0] / displaywidth;
		frcStartY = pdemo_win_info->startY * iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
		frcEndX = pdemo_win_info->endX *  iris_LPMeMcTiming[0] / displaywidth;
		frcEndY = pdemo_win_info->endY *  iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
	}

	pr_debug("frc mode resolution: %d - %d - %d - %d - %d - %d\n", frcStartX, frcStartY, frcEndX, frcEndY, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1]);
	if (frcEndY + pdemo_win_info->BorderWidth >= iris_LPMeMcTiming[1])
		frcEndY = iris_LPMeMcTiming[1] - pdemo_win_info->BorderWidth;
	winstart = (pdemo_win_info->startX & 0x3fff) + ((pdemo_win_info->startY & 0x3fff) << 16);
	winend =  (pdemo_win_info->endX & 0x3fff) + ((pdemo_win_info->endY & 0x3fff) << 16);

	peakingctrl = 1 | pdemo_win_info->SharpnessEn<<1;

	color = pdemo_win_info->color;

	colsize = (frcStartX & 0xfff) | ((frcEndX & 0xfff)<<16);
	rowsize = (frcStartY & 0xfff) | ((frcEndY & 0xfff)<<16);
	pr_debug("%s:BorderWidth =%x\n", __func__, pdemo_win_info->BorderWidth);
	modectrl = modectrl | pdemo_win_info->MEMCEn;
	modectrl = modectrl | 1<<1;
	modectrl = modectrl | ((pdemo_win_info->BorderWidth & 0x7)<<4);

	pr_debug("%s: COL_SIZE =%x, MODE_RING=%x, ROW_SIZE=%x, STARTWIN=%x, ENDWIN=%x, MODE_CTRL=%x, PEAKING_CTRL = %x\n",
		__func__, colsize, color, rowsize, winstart, winend, modectrl, peakingctrl);

	if (pdemo_win_info->MEMCEn) {
		//backup FI setting for demo window, because when setting demo window, the work mode may can't be MEMC mode, so
		//FI setting can't write.
		demowinFI_setting_default.colsize = colsize;
		demowinFI_setting_default.color = color;
		demowinFI_setting_default.rowsize = rowsize;
		demowinFI_setting_default.modectrl = modectrl;
		demo_win_FI_update = true;

		//set registers
		if ((iris_cfg->sf_notify_mode == IRIS_MODE_FRC) ||
			(iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE)) {
			demo_win_FI_update = false;
			iris_reg_add(FI_DEMO_COL_SIZE, colsize);
			iris_reg_add(FI_DEMO_MODE_RING, color);
			iris_reg_add(FI_DEMO_ROW_SIZE, rowsize);
			iris_reg_add(FI_DEMO_MODE_CTRL, modectrl);
			iris_reg_add(FI_SHADOW_UPDATE, 1);
		}
	}
	if (pdemo_win_info->SharpnessEn) {
		if (iris_cfg->sf_notify_mode != IRIS_MODE_RFB) {
			//mutex_lock(&iris_cfg->cmd_mutex);
			iris_reg_add(PEAKING_STARTWIN, winstart);
			iris_reg_add(PEAKING_ENDWIN, winend);
			iris_reg_add(PEAKING_CTRL, peakingctrl);
			iris_reg_add(PEAKING_SHADOW_UPDATE, 1);
			//mutex_unlock(&iris_cfg->cmd_mutex);
		} else {
			ret = iris_register_write(mfd, PEAKING_STARTWIN, winstart);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_ENDWIN, winend);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_CTRL, peakingctrl);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_SHADOW_UPDATE, 1);
			if (ret != 0)
				return ret;
		}
	}
	return 0;
}

static int iris_configure_get(struct msm_fb_data_type *mfd, u32 type, u32 count, u32 *values)
{
	int ret = 0;

	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if ((type >= IRIS_CONFIG_TYPE_MAX) || (mfd->panel_power_state == MDSS_PANEL_POWER_OFF))
		return -EFAULT;

	ret = irisDsiStsGet(ctrl);
	if (ret != IRIS_CONFIGURE_GET_VALUE_CORRECT)
		return ret;

	switch (type) {
	case IRIS_PEAKING:
		*values = pq_setting_current.peaking;
		break;
	case IRIS_SHARPNESS:
		*values = pq_setting_current.sharpness;
		break;
	case IRIS_MEMC_DEMO:
		*values = pq_setting_current.memcDemo;
		break;
	case IRIS_PEAKING_DEMO:
		*values = pq_setting_current.peakingDemo;
		break;
	case IRIS_GAMMA:
		*values = pq_setting_current.gamma;
		break;
	case IRIS_MEMC_LEVEL:
		*values = pq_setting_current.memcLevel;
		break;
	case IRIS_CONTRAST:
		*values = pq_setting_current.contrast;
		break;
	case IRIS_BRIGHTNESS:
		*values = pq_setting_current.sharpness;
		break;
	case IRIS_EXTERNAL_PWM:
		*values = dbc_setting_current.externalPWM;
		break;
	case IRIS_DBC_QUALITY:
		*values = dbc_setting_current.dbcQuality;
		break;
	case IRIS_DLV_SENSITIVITY:
		*values = dbc_setting_current.dlvSensitivity;
		break;
	case IRIS_DCE_LEVEL:
		*values = dbc_setting_current.DCELevel;
		break;
	case IRIS_DBC_CONFIG:
		*values = *((u32 *)&dbc_setting_current);
		break;
	case IRIS_PQ_CONFIG:
		*values = *((u32 *)&pq_setting_current);
		break;
	case IRIS_LPMEMC_CONFIG:
		*values = LPMemc_setting_current.level;
		break;
	case IRIS_USER_DEMO_WND:
		memcpy(values, &demo_win_info_setting, count * sizeof(u32));
		break;
	case IRIS_CINEMA_MODE:
		*values = pq_setting_current.cinema;
		break;
	case IRIS_CHIP_VERSION:
		*values = IRIS_CHIP_HW_VER;
		break;
	default:
		return -EFAULT;
	}
	return ret;
}

int iris_set_rotation(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	bool rotationen;

	ret = copy_from_user(&rotationen, argp, sizeof(bool));
	pr_debug("rotationen = %d\n", rotationen);

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_MVC_ADDR + 0xc, rotationen << 1);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);
	mutex_unlock(&mfd->iris_conf.cmd_mutex);

	return ret;
}

int iris_notify_video_frame_rate(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	uint32_t frame_rate_ms = 0;
	uint32_t r;
	int ret = copy_from_user(&frame_rate_ms, argp, sizeof(frame_rate_ms));

	pr_info("frame_rate_ms = %u\n", frame_rate_ms);

	// round to integer for 23976 and 29976
	mfd->iris_conf.input_frame_rate = (frame_rate_ms + 100) / 1000;
	mfd->iris_conf.output_frame_rate = 60;

	r = gcd(mfd->iris_conf.input_frame_rate, mfd->iris_conf.output_frame_rate);
	mfd->iris_conf.in_ratio = mfd->iris_conf.input_frame_rate / r;
	mfd->iris_conf.out_ratio = mfd->iris_conf.output_frame_rate / r;
	pr_debug("%s, in_ratio = %d, out_ratio = %d\n", __func__, mfd->iris_conf.in_ratio, mfd->iris_conf.out_ratio);

	return ret;
}

int msmfb_iris_tuning(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	static struct msmfb_iris_tuning old_iris_tuning = {
		.color_manager = 0,
		.flesh_tone_control = 0,
		.peaking = 0,
		.memc = 0,
		.picture_quality = 0,
		.sensitivity = 0,
		.brightness = 0,
	};
	struct msmfb_iris_tuning iris_tuning;
	int ret;

	ret = copy_from_user(&iris_tuning, argp, sizeof(iris_tuning));
	if (ret)
		return ret;

	//PQ MB3
	if (iris_tuning.color_manager != -1) {
		iris_register_write(mfd, IRIS_PROXY_ADDR + 0x18,
			(1 << 17) | iris_tuning.color_manager);
		pr_info("%s: set CM 0x%x", __func__,
			(1 << 17) | iris_tuning.color_manager);
	}

	if (iris_tuning.peaking != -1) {
		iris_register_write(mfd, IRIS_PROXY_ADDR + 0x18,
			(1 << 18) | iris_tuning.peaking);
		pr_info("%s: set Peaking 0x%x", __func__,
			(1 << 18) | iris_tuning.peaking);
	}

	if (iris_tuning.flesh_tone_control != -1) {
		iris_register_write(mfd, IRIS_PROXY_ADDR + 0x18,
			(1 << 23) | iris_tuning.flesh_tone_control);
		pr_info("%s: set FTC 0x%x", __func__,
			(1 << 23) | iris_tuning.flesh_tone_control);
	}

	if (iris_tuning.memc != -1) {
		iris_register_write(mfd, IRIS_PROXY_ADDR + 0x18,
			(1 << 24) | iris_tuning.memc);
		pr_info("%s: set memc 0x%x", __func__,
			(1 << 24) | iris_tuning.memc);
	}


	//DBC MB5
	if (iris_tuning.picture_quality != -1
		|| iris_tuning.sensitivity != -1
		|| iris_tuning.brightness != -1) {

		int picture_quality;
		int sensitivity;
		int brightness;

		if (iris_tuning.picture_quality != -1) {
			picture_quality = iris_tuning.picture_quality;
			old_iris_tuning.picture_quality = iris_tuning.picture_quality;
		} else {
			picture_quality = old_iris_tuning.picture_quality;
		}

		switch (iris_tuning.sensitivity) {
		case 3:
			sensitivity = 50;
			old_iris_tuning.sensitivity = iris_tuning.sensitivity;
			break;
		case 2:
			sensitivity = 100;
			old_iris_tuning.sensitivity = iris_tuning.sensitivity;
			break;
		case 1:
			sensitivity = 200;
			old_iris_tuning.sensitivity = iris_tuning.sensitivity;
			break;
		case 0:
			sensitivity = 0;
			old_iris_tuning.sensitivity = iris_tuning.sensitivity;
			break;
		case -1:
			sensitivity = old_iris_tuning.sensitivity;
		default:
			break;
		}

		if (iris_tuning.brightness != -1) {
			if (iris_tuning.brightness == 0)
				brightness = 10;
			else
				brightness = 25 + iris_tuning.brightness * 5;
			old_iris_tuning.brightness = iris_tuning.brightness;
		} else {
			brightness = old_iris_tuning.brightness;
		}

		pr_info("%s: set DBC 0x%x", __func__,
			1 | (brightness << 8) | (picture_quality << 16) | (sensitivity << 20));

		iris_register_write(mfd, IRIS_PROXY_ADDR + 0x28,
			1 | (brightness << 8) | (picture_quality << 16) | (sensitivity << 20));
	}

	return 0;
}

int msmfb_iris_write_reg(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	struct msmfb_iris_reg_value regValue;
	int ret;

	ret = copy_from_user(&regValue, argp, sizeof(regValue));
	if (ret)
		return ret;

	pr_info("addr = 0x%x, value = 0x%x\n",
			regValue.addr, regValue.value);
	ret = iris_register_write(mfd, regValue.addr, regValue.value);
	return ret;
}

int msmfb_iris_notify_scenario(struct msm_fb_data_type *mfd,
				int s)
{
	pr_info("Scenario = %s\n",
		(s == 1) ? "GRAPHICS"
		: (s == 2) ? "VIDEO"
		: (s == 3) ? "PHOTO" : "GAME");

	return 0;
}

int msmfb_iris_notify_work_mode(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	struct msmfb_iris_work_mode m;
	int ret;

	ret = copy_from_user(&m, argp, sizeof(m));
	if (ret)
		return ret;

	pr_info("Mode = %s, Value = %d\n",
			(m.mode == 1) ? "MEMC" : "UNDEFINED", m.enable);

	switch (m.mode) {
	case 1:
		mfd->iris_conf.memc_enable = (m.enable == 1) ? true : false;
		break;
	default:
		break;
	}

	return 0;
}

int msmfb_iris_frc_switch(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	int ret;
	bool video_on;

	ret = copy_from_user(&video_on, argp, sizeof(video_on));
	if (ret)
		return ret;

	mfd->iris_conf.video_on = video_on;
	return 0;
}

int mdss_mipi_dsi_command(struct mdss_panel_data *pdata, void __user *argp)
{
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct msmfb_mipi_dsi_cmd cmd;
	struct dsi_cmd_desc desc = {
		.payload = NULL,
	};
	struct dsi_cmd_desc *pdesc_muti, *pdesc;
	char read_response_buf[16] = {0};
	struct dcs_cmd_req req = {
		.cmds = &desc,
		.cmds_cnt = 1,
		.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_NO_MAX_PKT_SIZE,
		.rlen = 16,
		.rbuf = (char *)&read_response_buf,
		.cb = NULL
	};
	int ret, indx, cmd_len, cmd_cnt;
	char *pcmd_indx;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	pr_debug("%s:%d: mdss_panel_data: %p mdss_dsi_ctrl_pdata: %p\n", __func__, __LINE__, pdata, ctrl);
	ret = copy_from_user(&cmd, argp, sizeof(cmd));
	if (ret)
		return ret;
	pr_debug("#### %s:%d vc=%u d=%02x f=%u l=%u\n", __func__, __LINE__,
	       cmd.vc, cmd.dtype, cmd.flags, cmd.length);
	if (cmd.length) {
		desc.payload = kmalloc(cmd.length, GFP_KERNEL);
		if (!desc.payload)
			return -ENOMEM;
	}
	desc.dchdr.dtype = cmd.dtype;
	desc.dchdr.vc = cmd.vc;
	desc.dchdr.last = !!(cmd.flags & MSMFB_MIPI_DSI_COMMAND_LAST);
	desc.dchdr.ack = !!(cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK);
	desc.dchdr.dlen = cmd.length;
	desc.dchdr.wait = 0;
	ret = copy_from_user(desc.payload, cmd.payload, cmd.length);
	if (ret)
		goto err;
	if (cmd.dtype == 0x0f) {
		cmd_cnt = *desc.payload;
		pdesc_muti = kmalloc(sizeof(struct dsi_cmd_desc) * cmd_cnt, GFP_KERNEL);
		pcmd_indx = desc.payload + cmd_cnt + 1;
		for (indx = 0; indx < cmd_cnt; indx++) {
			pdesc = pdesc_muti + indx;
			cmd_len = *(desc.payload + 1 + indx);
			pdesc->dchdr.dtype = *pcmd_indx;
			pdesc->dchdr.vc = 0;
			pdesc->dchdr.last = 0;
			pdesc->dchdr.ack = 0;
			pdesc->dchdr.dlen = cmd_len - 1;
			pdesc->dchdr.wait = 0;
			pdesc->payload = pcmd_indx + 1;

			pcmd_indx += cmd_len;
			if (indx == (cmd_cnt - 1))
				pdesc->dchdr.last = 1;
			pr_debug("dtype:%x, dlen: %d, last: %d\n", pdesc->dchdr.dtype, pdesc->dchdr.dlen, pdesc->dchdr.last);
		}
		req.cmds = pdesc_muti;
		req.cmds_cnt = cmd_cnt;
		req.flags = CMD_REQ_COMMIT;
	}

	// This is debug for switch from BFRC Mode directly to PSR Mode
	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_DEBUG) {
		struct mdss_data_type *mdata = mdss_mdp_get_mdata();
		static char iris_psr_update_cmd[2] = { 0x1, 0x2 };
		struct dsi_cmd_desc iris_psr_update = {
			{ DTYPE_GEN_WRITE2, 1, 0, 0, 0,
			  sizeof(iris_psr_update_cmd) }, iris_psr_update_cmd
		};

		iris_wait_for_vsync(mdata->ctl_off);
		mdss_dsi_cmd_hs_mode(1, pdata);
		mdss_dsi_cmds_tx(ctrl, &iris_psr_update, 1, (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
		mdss_dsi_cmd_hs_mode(0, pdata);
	}

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_BLLP) {
		struct mdss_data_type *mdata = mdss_mdp_get_mdata();

		iris_wait_for_vsync(mdata->ctl_off);
	}

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		mdss_dsi_cmd_hs_mode(1, pdata);
	mdss_dsi_cmdlist_put(ctrl, &req);
	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		mdss_dsi_cmd_hs_mode(0, pdata);

	memcpy(cmd.response, ctrl->rx_buf.data, sizeof(cmd.response));
	ret = copy_to_user(argp, &cmd, sizeof(cmd));
err:
	kfree(desc.payload);
	if (cmd.dtype == 0x0f)
		kfree(pdesc_muti);
	return ret;
}

int msmfb_iris_configure(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	struct msmfb_iris_configure_value configure;
	int ret;

	ret = copy_from_user(&configure, argp, sizeof(configure));
	if (ret)
		return ret;

	pr_debug("type = %d, value = %d\n", configure.type, configure.value);
	ret = iris_configure(mfd, configure.type, configure.value);
	return ret;
}

int msmfb_iris_configure_ex(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	struct msmfb_iris_configure_ex_value configure;
	int ret;

	ret = copy_from_user(&configure, argp, sizeof(configure));
	if (ret)
		return ret;

	pr_debug("type = %d, cnt = %d\n", configure.type, configure.count);
	ret = iris_configure_ex(mfd, configure.type, configure.count, configure.values);
	return ret;
}

int msmfb_iris_configure_get(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	struct msmfb_iris_configure_ex_value configure;
	int ret;

	ret = copy_from_user(&configure, argp, sizeof(configure));
	if (ret)
		return ret;

	pr_debug("type = %d, cnt = %d\n", configure.type, configure.count);
	ret = iris_configure_get(mfd, configure.type, configure.count, configure.values);
	if (copy_to_user(argp, &configure, sizeof(configure)) != 0)
		ret = -EFAULT;
	return ret;
}

int iris_set_mode(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t mode;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	ret = copy_from_user(&mode, argp, sizeof(uint32_t));

	pr_debug("mode = %d, c_mode = %d\n", mode, iris_cfg->sf_notify_mode);

	if (mode == iris_cfg->sf_notify_mode)
		iris_cfg->mode_changed = false;
	else {
		iris_cfg->mode_changed = true;
		iris_cfg->sf_notify_mode = mode;
		pw_iris2_status = mode;
	}
	return ret;
}

int iris_get_mode(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t mode;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	mode = iris_cfg->sf_notify_mode;
	pr_debug("mode = %d\n", iris_cfg->sf_notify_mode);
	ret = copy_to_user(argp, &mode, sizeof(uint32_t));

	return ret;
}

int iris_set_meta(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct iris_meta user_meta;

	ret = copy_from_user((void *)&user_meta, argp, sizeof(struct iris_meta));
	if (ret != 0)
		return -EINVAL;

	mutex_lock(&iris_cfg->meta_mutex);
	iris_cfg->meta_set.op |= user_meta.op;
	if (user_meta.op & MDP_IRIS_OP_NEW_FRAME)
		iris_cfg->meta_set.new_frame = user_meta.new_frame;
	if (user_meta.op & MDP_IRIS_OP_RESTART)
		iris_cfg->meta_set.restart = user_meta.restart;
	if (user_meta.op & MDP_IRIS_OP_VTS)
		iris_cfg->meta_set.video_ts = user_meta.video_ts;
	if (user_meta.op & MDP_IRIS_OP_STS)
		iris_cfg->meta_set.sys_ts = user_meta.sys_ts;
	if (user_meta.op & MDP_IRIS_OP_VID)
		iris_cfg->meta_set.vid = user_meta.vid;
	if (user_meta.op & MDP_IRIS_OP_TE)
		iris_cfg->meta_set.te_period = user_meta.te_period;
	if (user_meta.op & MDP_IRIS_OP_CP) {
		iris_cfg->meta_set.content_period = user_meta.content_period;
		iris_cfg->meta_set.content_period_frac = user_meta.content_period_frac;
	}
	if (user_meta.op & MDP_IRIS_OP_MOTION)
		iris_cfg->meta_set.motion = user_meta.motion;
	if (user_meta.op & MDP_IRIS_OP_JITTER)
		iris_cfg->meta_set.jitter = user_meta.jitter;
	if (user_meta.op & MDP_IRIS_OP_NRV)
		iris_cfg->meta_set.nrv = user_meta.nrv;
	if (user_meta.op & MDP_IRIS_OP_FLG)
		iris_cfg->meta_set.flags = user_meta.flags;
	if (user_meta.op & MDP_IRIS_OP_RPT)
		iris_cfg->meta_set.repeat = user_meta.repeat;
	if (user_meta.op & MDP_IRIS_OP_IF1)
		iris_cfg->meta_set.iris_info1 = user_meta.iris_info1;
	if (user_meta.op & MDP_IRIS_OP_IF2)
		iris_cfg->meta_set.iris_info2 = user_meta.iris_info2;
	mutex_unlock(&iris_cfg->meta_mutex);

	pr_debug("op [%08x] vTimestamp [%u] sTimestamp [%u] flag [%u]\n",
		iris_cfg->meta_set.op, iris_cfg->meta_set.video_ts, iris_cfg->meta_set.sys_ts, iris_cfg->meta_set.flags);

	if (iris_cfg->meta_set.op & MDP_IRIS_OP_RPT)
		pr_debug("repeat: %d\n", iris_cfg->meta_set.repeat);
	if (iris_cfg->meta_set.op & MDP_IRIS_OP_NRV) {
		struct iris_nrv_meta *nrv_meta = &iris_cfg->meta_set.nrv;

		pr_debug("NRV enable [%u]\n", nrv_meta->nrvEnable);
		pr_debug("Capture [%u][%u] [%u][%u]\n", nrv_meta->captureLeft, nrv_meta->captureRight,
												nrv_meta->captureTop, nrv_meta->captureBottom);
		pr_debug("Display [%u][%u] [%u][%u]\n", nrv_meta->displayLeft, nrv_meta->displayRight,
												nrv_meta->displayTop, nrv_meta->displayBottom);
	}
	return ret;
}


int iris_frc_path_update(struct msm_fb_data_type *mfd, void __user *argp)
{
	struct msmfb_iris_frc_path frc_path;
	int ret;

	ret = copy_from_user(&frc_path, argp, sizeof(frc_path));
	if (ret)
		return ret;
	pr_debug("frc_path.bit_index=%d enable =%d\n", frc_path.bit_index, frc_path.enable);
	if (frc_path.enable)
		g_mfd->iris_conf.frc_path |= BIT(frc_path.bit_index);
	else
		g_mfd->iris_conf.frc_path &= ~BIT(frc_path.bit_index);

	return 0;
}

#if defined(CONFIG_IRIS2_DRC_SUPPORT)
int iris_get_frc_timing(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	pr_debug("guFrcLPTiming = %x\n", guFrcLPTiming);
	ret = copy_to_user(argp, &guFrcLPTiming, sizeof(uint32_t));
	//TODO
	return ret;
}
/*****
* DRC Dynamic resolution change
*
******/
int iris_set_drc_size(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t utemp;
	ret = copy_from_user(&utemp, argp, sizeof(uint32_t));

	g_mfd->iris_conf.drc_enable = (utemp > 0) ? true : false;
	g_mfd->iris_conf.drc_size = utemp;

	return ret;
}
#endif
