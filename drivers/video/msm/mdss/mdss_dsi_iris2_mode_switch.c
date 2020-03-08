#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/msm_mdp.h>
#include <linux/gpio.h>
#include <linux/circ_buf.h>
#include <linux/gcd.h>
#include <asm/uaccess.h>

#include "mdss_mdp.h"
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_debug.h"

#include "mdss_dsi_iris2.h"
#include "mdss_dsi_iris2_def.h"
#include "mdss_dsi_iris2_extern.h"
#include "mdss_dsi_iris2_mode_switch.h"

#define FI_RANGE_CTRL          0xf2160014
#define UNIT_CONTRL_ADDR       0xf0060000

static u32 m_fiVrangeTop = 0xa;
static int rfb_delay;
static int frc_delay;
static int prep_delay;

static char iris_memc_enter_cmds[] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x00000005),	//valid word number
	0x00,					//burst mode
	0x00,					//reserved
	PWIL_U16(0x0004),		//burst length
	PWIL_U32(IRIS_PROXY_ADDR + 0x10),	//proxy MB2
	PWIL_U32(0x800000),
	PWIL_U32(IRIS_PROXY_ADDR + 0x08), //proxy MB1
	PWIL_U32(0x800002)
};
struct dsi_cmd_desc memc_data_path_config[1] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_memc_enter_cmds) }, iris_memc_enter_cmds},
};

static bool is_memc_doable(int vp_num, int gp_num)
{
	int ret = (vp_num == 1) ? 1 : 0;
	return ret;
}

static void iris_memc_path_commands_update(void)
{
	*(u32 *)(iris_memc_enter_cmds + 20) = cpu_to_le32(g_mfd->iris_conf.frc_path);
	iris_memc_enter_cmds[21] = (iris_memc_enter_cmds[21] & 0xFC) | iris_dbc_mode;
}

static void iris_set_scanline(struct iris_config *iris_cfg)
{
	u32 scanline;
	u32 m_hres = iris_LPMeMcTiming[0];
	u32 m_vres = iris_LPMeMcTiming[1];
	bool hmode = (m_hres*m_vres > IMG720P_HSIZE*IMG720P_VSIZE) ? true : false;
	struct iris_timing_para iris_dtg = iris_mipi_info.iris_out_timing;
	u32 vtotal = iris_dtg.vfp + iris_dtg.vsw + iris_dtg.vbp + iris_dtg.vres;
	u32 val_frcc_reg17 = iris_cfg->val_frcc_reg17;

	if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC) {
		// scanline =  (hmode == true)? (vtotal/5): (vtotal - IRIS_DTG_EVS_DLY);
		scanline = vtotal / 5;
	} else {
		scanline = vtotal - IRIS_DTG_EVS_DLY;
		val_frcc_reg17 &= 0xffff8000;
		val_frcc_reg17 |= 50;	// frcc_input_hw_meta0/hw_input_meta_vd_cf_idx will keep at 0
	}

	mutex_lock(&iris_cfg->cmd_mutex);
	// TE_CTRL
	iris_reg_add(IRIS_DTG_ADDR + 0x00034, (0xd | (scanline << 16)));
	// DTG_CTRL1
	//iris_reg_add(IRIS_DTG_ADDR + 0x00044, 0x3c01);
	// DVS_CTRL
	//iris_reg_add(IRIS_DTG_ADDR + 0x00064, (vtotal << 8));
	iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);   //reg_update
	iris_reg_add(FRCC_CTRL_REG17_ADDR, val_frcc_reg17);
	iris_reg_add(FRCC_REG_SHOW, 0x2);

	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("scanline %d m_hres %d m_vres %d hmode %i\n", scanline, m_hres, m_vres, hmode);
}

static void iris_fiSearchRangeTop(void)
{
	m_fiVrangeTop = 0x40000 / (u32)iris_LPMeMcTiming[0] - 4;

	if (m_fiVrangeTop > 510)
		m_fiVrangeTop = 510;

	m_fiVrangeTop = m_fiVrangeTop / 2 - 1;

	pr_debug("iris_fiSearchRangeTop: %d\n", m_fiVrangeTop);

}

#if defined(CONFIG_IRIS2_DRC_SUPPORT)
static int iris_Drc_LPMemc_update(struct msm_fb_data_type *mfd) {

	u32 configAddr = 0;
	u32 configValue = 0;

	configValue = g_mfd->iris_conf.drc_size | 0x80000000;
	configAddr = IRIS_PROXY_ADDR + 0x20;

	if (0 == configValue && 0 == configAddr) {
		pr_warn("iris_Drc_LPMemc_update failed!\n");
		return -EINVAL;
	}

	return iris_register_write(mfd, configAddr, configValue);

}

static void iris_calc_drc_enter(struct msm_fb_data_type *mfd)
{
	int width, height;
	int top, left;


	int Htotal = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);

	left = Htotal - (g_mfd->iris_conf.drc_size & 0xffff);
	top = iris_mipi_info.iris_in_timing.vres - (g_mfd->iris_conf.drc_size >> 16);

	width = g_mfd->iris_conf.drc_size & 0xffff;
	height = g_mfd->iris_conf.drc_size >> 16;

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x104c, ((uint32_t)top << 16 | left));
	iris_reg_add(IRIS_PWIL_ADDR + 0x1050, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x1054, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x106c, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	mutex_unlock(&mfd->iris_conf.cmd_mutex);
}

static void iris_calc_drc_exit(struct msm_fb_data_type *mfd)
{
	int width, height;
	int top, left;


	int Htotal = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);

	left = 0;
	top = 0;

	width = Htotal;
	height = iris_mipi_info.iris_in_timing.vres;

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x104c, ((uint32_t)top << 16 | left));
	iris_reg_add(IRIS_PWIL_ADDR + 0x1050, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x1054, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x106c, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	mutex_unlock(&mfd->iris_conf.cmd_mutex);
}
#endif

static int iris_MEMC_reg_write(struct msm_fb_data_type *mfd,
			struct demo_win_info_for_FI sdemowinFI_setting_default)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int displaywidth = (iris_mipi_info.mipi_mode.tx_ch  ?  iris_mipi_info.iris_out_timing.hres * 2 : iris_mipi_info.iris_out_timing.hres);
	int /*width, height,*/ frcEndX, frcEndY, frcStartX, frcStartY;
	static int lpmMemcHres, lpmMemcVres;

	if ((iris_LPMeMcTiming[0] != lpmMemcHres) || (iris_LPMeMcTiming[1] != lpmMemcVres)) {
		demo_win_FI_update = true;
		lpmMemcHres = iris_LPMeMcTiming[0];
		lpmMemcVres = iris_LPMeMcTiming[1];
	}

	pr_debug("iris_cfg->nrv_enable: %d\n", iris_cfg->nrv_enable);
	if (demo_win_FI_update && (pq_setting_current.memcDemo == 2)) {
		demo_win_FI_update = false;
		if (!iris_cfg->nrv_enable) {
			frcStartX = demo_win_info_setting.startX * iris_LPMeMcTiming[0] / displaywidth;
			frcStartY = demo_win_info_setting.startY * iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
			frcEndX = demo_win_info_setting.endX *  iris_LPMeMcTiming[0] / displaywidth;
			frcEndY = demo_win_info_setting.endY *  iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;

			if (frcEndY + demo_win_info_setting.BorderWidth >= iris_LPMeMcTiming[1])
				frcEndY = iris_LPMeMcTiming[1] - demo_win_info_setting.BorderWidth;

			pr_debug("iris: %s: frcStartX = %d, frcStartY = %d, frcEndX = %d, frcEndY = %d, iris_LPMeMcTiming[0] = %d, iris_LPMeMcTiming[1] = %d.\n",
					__func__, frcStartX, frcStartY, frcEndX, frcEndY, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1]);
			sdemowinFI_setting_default.colsize = (frcStartX & 0xfff) | ((frcEndX & 0xfff) << 16);
			sdemowinFI_setting_default.rowsize = (frcStartY & 0xfff) | ((frcEndY & 0xfff) << 16);
		}

		iris_reg_add(FI_DEMO_COL_SIZE, sdemowinFI_setting_default.colsize);
		iris_reg_add(FI_DEMO_MODE_RING, sdemowinFI_setting_default.color);
		iris_reg_add(FI_DEMO_ROW_SIZE, sdemowinFI_setting_default.rowsize);
		iris_reg_add(FI_DEMO_MODE_CTRL, sdemowinFI_setting_default.modectrl);
		iris_reg_add(FI_SHADOW_UPDATE, 1);
	}
	return 0;
}

int iris_mode_switch_video(struct msm_fb_data_type *mfd)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata;
	struct mdss_mdp_pipe *pipe;
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int used_vp = 0;
	int used_gp = 0;
	int used_dp = 0;

	if (!g_dsi_ctrl) {
		pdata = mdp5_data->ctl->panel_data;
		g_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	if (iris_cfg->mode_changed == false)
		return -EFAULT;

	iris_cfg->mode_changed = false;

	if (mfd->index != 0)
		return -EFAULT;

	pr_debug("memc_enable = %d, cur_mode = %d, video_on = %d\n",
		iris_cfg->memc_enable, iris_cfg->current_mode, iris_cfg->video_on);

	list_for_each_entry(pipe, &mdp5_data->pipes_used, list) {
		if (pipe->type == MDSS_MDP_PIPE_TYPE_VIG)
			used_vp++;

		if (pipe->type == MDSS_MDP_PIPE_TYPE_RGB)
			used_gp++;

		if (pipe->type == MDSS_MDP_PIPE_TYPE_DMA)
			used_dp++;
	}

	pr_debug("memc_enable %d, doable = %d, cur_mode = %d\n",
			iris_cfg->memc_enable,
			is_memc_doable(used_vp, used_gp),
			iris_cfg->sf_notify_mode);
	pr_debug("vp = %d, gp = %d dp = %d\n", used_vp, used_gp, used_dp);

	if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_prepare_work);
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_work);
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC_CANCEL
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_cancel_work);
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_RFB
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->pt_work);
	}

	return 0;
}

static void iris_check_rfb_ready(struct iris_config *iris_cfg){
	u32 val;
	if (iris_cfg->check_appcode_rfb_ready) {
		val = iris_pi_read(g_dsi_ctrl, IRIS_PROXY_ADDR + 0x08);
		if (val == IRIS_PT_MODE) {
			pr_info("appcode switch to RFB, rfb_delay: %d\n", rfb_delay);
			iris_cfg->check_appcode_rfb_ready = false;
		} else {
			pr_debug("appcode not switch to RFB\n");
		}
	}
	if (iris_cfg->check_pwil_rfb_ready) {
		val = iris_pi_read(g_dsi_ctrl, IRIS_PWIL_ADDR + 0x80);
		if ((val & 0x7e0) == 0x80) {
			pr_info("pwil switch to RFB, rfb_delay: %d\n", rfb_delay);
			iris_cfg->check_pwil_rfb_ready = false;
		} else {
			pr_debug("pwil not switch to RFB\n");
		}
	}
}

int iris_mode_switch_cmd(struct msm_fb_data_type *mfd)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata;
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc cmd_rfb;
	bool memc_enable;
	u32 val;
	if (!g_dsi_ctrl) {
		pdata = mdp5_data->ctl->panel_data;
		g_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	if (iris_cfg->mode_changed == false)
		return -EFAULT;

	if (mfd->index != 0)
		return -EFAULT;

	if (!iris_cfg->ready) {
		iris_cfg->mode_changed = false;
		iris_cfg->sf_notify_mode = IRIS_MODE_RFB;
		pr_info("forbid mode switch, iris not ready!\n");
		return -EFAULT;
	}

	if (debug_usb_w_enabled) {
		memc_enable = iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE ||
			iris_cfg->sf_notify_mode == IRIS_MODE_FRC;
		if (memc_enable && !iris_cfg->memc_perf_hack) {
			pr_info("memc_perf_hack enable\n");
			iris2_io_bimc_enable(true);
			iris2_io_snoc_enable(true);
			iris_cfg->memc_perf_hack = true;
		}
		else if (!memc_enable && iris_cfg->memc_perf_hack) {
			pr_info("memc_perf_hack disable\n");
			iris2_io_bimc_enable(false);
			iris2_io_snoc_enable(false);
			iris_cfg->memc_perf_hack = false;
		}
	}

	cmdreq.cmds = &cmd_rfb;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_HS_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	cmdreq.cmds_cnt = 1;

	iris_cfg->mode_changed = false;
	if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		if (prep_delay == 0) {
			iris_check_rfb_ready(iris_cfg);
			if (iris_cfg->check_appcode_rfb_ready || iris_cfg->check_pwil_rfb_ready) {
				iris_cfg->mode_changed = true;
				return 0;
			}
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc LP MEMC update, don't take effect now
			if (g_mfd->iris_conf.drc_enable && (g_mfd->iris_conf.drc_size != guFrcLPTiming))
				iris_Drc_LPMemc_update(mfd);
#endif
			iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
			iris_memc_path_commands_update();
			// memc_data_path_config
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, memc_data_path_config, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			prep_delay++;
			iris_cfg->mode_changed = true;
		} else if (prep_delay++ < 30) {
			if (debug_mode_switch_enabled)
				val = iris_pi_read(g_dsi_ctrl, IRIS_PROXY_ADDR + 0x08);
			else
				val = IRIS_MEMC_MODE;
			if (val != IRIS_MEMC_MODE) {
				pr_debug("iris: mode = %08x, cnt = %d\n", val, prep_delay);
				if (val == IRIS_PT_MODE) {
					pr_info("iris: still in RFB mode, retry mode switch\n");
					iris_cfg->sf_notify_mode = IRIS_MODE_FRC_PREPARE_RFB;
					prep_delay = 0;
					return 0;
				}
				iris_cfg->mode_changed = true;
			} else {
				prep_delay = 0;
				frc_delay = 0;
				frc_repeat_enter = false;
				iris_cfg->cap_enable = true;
				iris_cfg->cap_change = false;
				iris_cfg->sf_notify_mode = IRIS_MODE_FRC_PREPARE_DONE;
				pw_iris2_status = IRIS_MODE_FRC_PREPARE_DONE;
				pr_debug("iris: sf_notify_mode = %d\n", iris_cfg->sf_notify_mode);
				iris_proc_constant_ratio(iris_cfg);
			}
		} else {
			pr_info("iris: memc prep time out\n");
			iris_cfg->sf_notify_mode = IRIS_MODE_FRC_PREPARE_TIMEOUT;
			prep_delay = 0;
		}
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay);
		if (frc_delay == 0) {
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc enter
			if (g_mfd->iris_conf.drc_enable)
				iris_calc_drc_enter(mfd);
#endif
			iris_proc_frcc_setting(g_mfd);
			iris_MEMC_reg_write(g_mfd, demowinFI_setting_default);
			// memc_mode_enter
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, memc_mode_enter, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			rfb_delay = 0;
			frc_delay = 1;
			iris_cfg->mode_changed = true;
		} else if (frc_delay == 1) {
			frc_delay++;
			iris_cfg->mode_changed = true;
		} else if (frc_delay == 2) {
			iris_set_scanline(iris_cfg);
			if (debug_clock_gate) {
				frc_delay++;
				iris_cfg->mode_changed = true;
			}
		}
		else if (debug_clock_gate && (frc_delay <= 8)) {
			val = iris_pi_read(g_dsi_ctrl, IRIS_PWIL_ADDR + 0x80);
			if((val &0x7e0) == 0x100){
				iris_reg_add(IRIS_SYS_ADDR, 0x20000400);
				pr_err("/////////sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay);
				iris_cfg->mode_changed = false;
			}
			else{
				frc_delay++;
				iris_cfg->mode_changed = true;
				if(frc_delay >8){
					pr_err("***********sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay);
			}
			}
		}
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC_CANCEL
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d\n", iris_cfg->sf_notify_mode);
		iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
		if (debug_mode_switch_enabled) {
			memcpy(&cmd_rfb, pt_data_path_config, sizeof(struct dsi_cmd_desc));
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
		}
		iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
		prep_delay = 0;
	} else if (iris_cfg->sf_notify_mode == IRIS_MODE_RFB
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d, rfb_delay: %d\n", iris_cfg->sf_notify_mode, rfb_delay);
		iris_cfg->iris_ratio_updated = false;
		iris_cfg->prev_dvts = 0;
		iris_cfg->repeat = IRIS_REPEAT_NO;
		prep_delay = 0;
		iris_cfg->input_vfr = 0;
		if (0 == rfb_delay) {
			// workaround, change prameters before FRC exit
			u32 DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
					iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
			unsigned int val_frcc_cmd_th = iris_cfg->val_frcc_cmd_th;
			unsigned int val_frcc_reg8 = iris_cfg->val_frcc_reg8;
			unsigned int val_frcc_reg16 = iris_cfg->val_frcc_reg16;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc exit
			if (g_mfd->iris_conf.drc_enable)
				iris_calc_drc_exit(mfd);
#endif
			iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
			val_frcc_cmd_th &= 0x1fffffff;
			val_frcc_cmd_th |= 0x20000000;
			val_frcc_reg8 |= 0x3f00;
			val_frcc_reg16 &= 0xffff7fff;
			iris_fiSearchRangeTop();
			//iris_set_scanline(iris_cfg);
			mutex_lock(&iris_cfg->cmd_mutex);
			iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
			iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
			iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
			iris_reg_add(FRCC_REG_SHOW, 0x2);
			// set the ratio in/out as 1:1
			iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, ((1 << IRIS_PWIL_IN_FRAME_SHIFT) | (1 << 15)));
			iris_reg_add(IRIS_PWIL_ADDR + 0x0638, 1 << IRIS_PWIL_OUT_FRAME_SHIFT);
			iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
			iris_reg_add(IRIS_DTG_ADDR + 0x018, (DisplayVtotal - 12) | (1 << 16));
			iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);   //reg_update
			iris_reg_add(FI_RANGE_CTRL, m_fiVrangeTop | (11 << 9) | (511 << 18));
			iris_reg_add(FI_SHADOW_UPDATE, 1);
			mutex_unlock(&iris_cfg->cmd_mutex);
			rfb_delay = 1;
			iris_cfg->mode_changed = true;
			//return 0;
		} else if (rfb_delay == 1) {
			rfb_delay++;
			iris_set_scanline(iris_cfg);
			iris_cfg->mode_changed = true;
		} else if (rfb_delay == 2) {
			val = iris_pi_read(g_dsi_ctrl, UNIT_CONTRL_ADDR + 0x08);
			val |= 0x800; //enable bit 11
			iris_pi_write(g_dsi_ctrl, UNIT_CONTRL_ADDR + 0x08, val);
			if (iris_mipi_info.iris_timing_flag
					|| (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)) {
				pt_enable[0] = 0x3 << 2;
				pt_enable[1] = 0x1;
			} else {
				pt_enable[0] = 0x0;
				pt_enable[1] = 0x1;
			}
			// pt_mode_enter
			memcpy(&cmd_rfb, pt_mode_enter, sizeof(struct dsi_cmd_desc));
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			// pt_data_path_config
			iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, pt_data_path_config, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			if (debug_new_repeat == 0) {
				unsigned int reg_cap = 0xc0000003;
				mutex_lock(&iris_cfg->cmd_mutex);
				iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
				mutex_unlock(&iris_cfg->cmd_mutex);
			}
			iris_cfg->true_cut_enable = false;
			iris_cfg->frc_path &= ~BIT(11);
			if (debug_clock_gate) {
				rfb_delay++;
				iris_cfg->mode_changed = true;
			}
			iris_cfg->check_appcode_rfb_ready = true;
			iris_cfg->check_pwil_rfb_ready = true;
		}
		else if (debug_clock_gate && (rfb_delay <= 8)) {
			rfb_delay++;
			iris_cfg->mode_changed = true;
		}
		else if (debug_clock_gate && (rfb_delay <= 15)) {
			iris_check_rfb_ready(iris_cfg);

			if (!iris_cfg->check_appcode_rfb_ready && !iris_cfg->check_pwil_rfb_ready) {
				iris_reg_add(IRIS_SYS_ADDR, 0x20003000);
				iris_cfg->mode_changed = false;
				pr_info("RFB switch finished.\n");
			} else {
				rfb_delay++;
				iris_cfg->mode_changed = true;
				if(rfb_delay >15) {
					pr_info("RFB switch not finished, appcode ready: %d, pwil ready: %d\n",
						!iris_cfg->check_appcode_rfb_ready, !iris_cfg->check_pwil_rfb_ready);
				}
			}
		}
	}

	return 0;
}

int iris_proc_frcc_setting(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata = mdp5_data->ctl->panel_data;

	//default val of reference register which need host to set.
	u32 val_frcc_reg5 = 0x3c010000;
	u32 val_frcc_reg8 = 0x10000000;
	u32 val_frcc_reg16 = 0x413120c8;
	u32 val_frcc_reg17 = 0x8000;
	u32 val_frcc_reg18 = 0;
	u32 val_frcc_cmd_th = 0x8000;

	//formula variable
	u32 ThreeCoreEn, VD_CAP_DLY1_EN;
	u32 MaxFIFOFI, KeepTH, CarryTH, RepeatP1_TH;
	u32 RepeatCF_TH, TS_FRC_EN, INPUT_RECORD_THR, MERAREC_THR_VALID;
	u32 MetaGen_TH1, MetaGen_TH2, MetaRec_TH1, MetaRec_TH2;

	//timing and feature variable
	u32 te_fps, display_vsync, Input_Vres, Scaler_EN = false, Capture_EN, Input_Vtotal;
	u32 DisplayVtotal, HsyncFreqIn, HsyncFreqOut, InVactive, StartLine, Vsize;
	int inputwidth = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);
	u32 Infps = iris_cfg->input_frame_rate;
	int adjustmemclevel = 3;
	int hlmd_func_enable = 0;

	//init variable
	te_fps = mdss_panel_get_framerate(&pdata->panel_info);
	display_vsync = 60;//iris to panel, TODO, or 120
	Input_Vres = pdata->panel_info.yres;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	Capture_EN = iris_cfg->nrv_enable | iris_cfg->drc_enable;
#else
	Capture_EN = iris_cfg->nrv_enable;
#endif
	Input_Vtotal = mdss_panel_get_vtotal(&pdata->panel_info);
	if (iris_LPMeMcTiming[0] != inputwidth)
		Scaler_EN = true;
	else
		Scaler_EN = false;
	DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
		iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
	HsyncFreqIn = te_fps * Input_Vtotal;
	HsyncFreqOut = display_vsync * DisplayVtotal;
	InVactive = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	if (iris_cfg->drc_enable)
		InVactive = (iris_cfg->drc_size >> 16);
#endif
	if (Capture_EN)
		StartLine = Input_Vres - InVactive;
	else if (Scaler_EN)
		StartLine = 5;
	else
		StartLine = 0;
	if (Capture_EN)
		Vsize = InVactive;
	else
		Vsize = DisplayVtotal;

	pr_debug("%s: get timing info, infps=%d, displayVtotal = %d, InVactive = %d, StartLine = %d, Vsize = %d\n",
		__func__, Infps, DisplayVtotal, InVactive, StartLine, Vsize);
	pr_debug("TE_fps = %d, display_vsync = %d, inputVres = %d, Scaler_EN = %d, capture_en = %d, InputVtotal = %d\n",
		te_fps, display_vsync, Input_Vres, Scaler_EN, Capture_EN, Input_Vtotal);

	if (mfd->panel_info->type == MIPI_VIDEO_PANEL) {
		//video mode
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		goto VAL_CALC;
	}

	if (iris_cfg->fbo_enable) {
		//TODO mbo mode
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		goto VAL_CALC;
	}

	pr_debug("iris_cfg->input_vfr: %d", iris_cfg->input_vfr);
	//check input is variable frame rate or not.
	switch (iris_cfg->input_vfr) {
	case 15:// 15 fps from 24/25 fps.
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		if (debug_hlmd_enabled && !iris_cfg->true_cut_enable) {
			hlmd_func_enable = 1;
			RepeatP1_TH = 1;
			CarryTH = 1;
		} else {
			hlmd_func_enable = 0;
		}
		goto VAL_CALC;
	case 50:// vfr from 50 drop
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		goto VAL_CALC;
	case 60:// vfr from 60 drop
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		goto VAL_CALC;
	case 0:// vfr is invalid, frame rate is constant
	default :
		break;
	}

	switch (Infps) {
	case 24://24fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 3; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		break;
	case 30://30fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 2; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		break;
	case 25://25fps
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 3; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 3 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		break;
	case 15://15fps
		if (debug_hlmd_enabled && !iris_cfg->true_cut_enable) {
			hlmd_func_enable = 1;
			RepeatP1_TH = 1;
			CarryTH = 1;
		} else {
			hlmd_func_enable = 0;
			RepeatP1_TH = 2;
			CarryTH = 2;
		}
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61;
		RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
	case 12://12fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		break;
	default:
		pr_err("%s, using default frcc parameters\n", __func__);
		goto SET_REG;
	}

VAL_CALC:
	if (pq_setting_current.memcLevel == 3)
		adjustmemclevel = 3;
	else if (pq_setting_current.memcLevel == 2)
		adjustmemclevel = 3;
	else if (pq_setting_current.memcLevel == 1)
		adjustmemclevel = 2;
	else if (pq_setting_current.memcLevel == 0)
		adjustmemclevel = 0;

	//val_frcc_reg5 = val_frcc_reg5 + ((pq_setting_current.memcLevel & 0x3) << 17) + (KeepTH * 2 << 7) + CarryTH;
	val_frcc_reg5 = val_frcc_reg5 + ((adjustmemclevel & 0x3) << 17) + (KeepTH * 2 << 7) + CarryTH;
	val_frcc_reg8 = val_frcc_reg8 + (RepeatP1_TH * 2 << 7) + RepeatCF_TH * 2;
	val_frcc_reg16 = val_frcc_reg16 + (TS_FRC_EN * 2 << 30) + (ThreeCoreEn*2 << 14) + VD_CAP_DLY1_EN;
	val_frcc_reg17 = val_frcc_reg17 + (DisplayVtotal * 2 << 15) + INPUT_RECORD_THR;
	val_frcc_reg18 = val_frcc_reg18 + (MERAREC_THR_VALID * 2 << 30) + (MetaRec_TH2 * 2 << 15) + MetaRec_TH1;
	val_frcc_cmd_th = val_frcc_cmd_th + (MaxFIFOFI * 2 << 28) + (MetaGen_TH2 * 2 << 15) + MetaGen_TH1;

SET_REG:
	pr_debug("%s: reg5=%x, reg8=%x, reg16=%x, reg17=%x, reg18=%x, cmd_th=%x\n", __func__,
		val_frcc_reg5, val_frcc_reg8, val_frcc_reg16, val_frcc_reg17, val_frcc_reg18, val_frcc_cmd_th);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(FRCC_CTRL_REG5_ADDR, val_frcc_reg5);
	iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
	iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
	iris_reg_add(FRCC_CTRL_REG17_ADDR, val_frcc_reg17);
	iris_reg_add(FRCC_CTRL_REG18_ADDR, val_frcc_reg18);
	iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
	if (debug_hlmd_enabled) {
		if (hlmd_func_enable)
			iris_reg_add(IRIS_MVC_ADDR + 0x1ffe8, 0x00200000);
	}
	if (iris_cfg->true_cut_enable) {
		iris_reg_add(IRIS_MVC_ADDR + 0x1ffe8, 0x00200000);
	} else {
		iris_reg_add(IRIS_MVC_ADDR + 0x1ffe8, 0x00000000);
	}
	mutex_unlock(&iris_cfg->cmd_mutex);
	iris_cfg->val_frcc_cmd_th = val_frcc_cmd_th;
	iris_cfg->val_frcc_reg8 = val_frcc_reg8;
	iris_cfg->val_frcc_reg16 = val_frcc_reg16;
	iris_cfg->val_frcc_reg17 = val_frcc_reg17;
	return 0;
}

void iris_mode_switch_reset(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	iris_cfg->mode_changed = false;
	iris_cfg->sf_notify_mode = IRIS_MODE_RFB;
	rfb_delay = 0;
	frc_delay = 0;
	prep_delay = 0;
	iris_cfg->iris_ratio_updated = false;
	iris_cfg->repeat = IRIS_REPEAT_NO;
	iris_cfg->true_cut_enable = false;
	iris_cfg->frc_path &= ~BIT(11);
}
