/* Copyright (c) 2013, Pixelworks, Inc.
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
#include <linux/uaccess.h>
#include <linux/clk.h>

#include "mdss_mdp.h"
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_debug.h"
#include "mdss_dsi_iris2.h"
#include "mdss_dsi_iris2_def.h"
#include "mdss_dsi_iris2_extern.h"
#include "mdss_dsi_iris2_mode_switch.h"
#include "mdss_dsi_iris2_dbg.h"
#include "mdss_dsi_iris2_ioctl.h"

struct iris_mgmt_t {
	struct work_struct iris_worker;
	struct workqueue_struct *iris_wq;
	void (*iris_handler)(void);
	bool fbo_enable;
	bool sbs_enable;
	struct msm_fb_data_type *mfd;
};

struct iris_reg_t {
	u32 addr;
	u32 val;
};

#define MAX_CAD_LEN 5
// FIXME mdp5 use add vsync handler and no find DMA_P bit
//void mdp4_dsi_video_wait4dmap_done(int cndx);
#define IRIS_REPEAT_FORCE  1
#define META_HEADER 16
#define META_PKT_SIZE 512
#define IRIS_REPEAT_CAPDIS 2
#define IRIS_REGS 40

static int iris_reg_cnt;
static struct iris_mgmt_t iris_mgmt;
/* Activate Delay 0, FBO Enable: 1, Display Mode: FRC Enable,
* PSR Command: PSR update, Capture Enable: Video
*/
static char fbo_update[2] = {0x15, 0x02};

static char imeta_header[META_HEADER] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x3),
	0x00,
	0x00,
	PWIL_U16(0x2),
};
static char imeta[META_PKT_SIZE] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x3),
	0x00,
	0x00,
	PWIL_U16(0x2),
};

//iris_meta_pkts[1] will also be updated  by memc_enable
static struct dsi_cmd_desc iris_meta_pkts[] = {
	{{DTYPE_GEN_LWRITE, 0, 0, 0, 0, sizeof(imeta)}, imeta},
	{{ DTYPE_GEN_WRITE2, 0, 0, 0, 0, sizeof(fbo_update) }, fbo_update },
};

static struct iris_reg_t iris_regs[IRIS_REGS];

static int iris_set_ratio(struct iris_config *iris_cfg);
static int iris_regs_meta_build(void);

void iris_update_configure(void)
{
	pr_debug("iris_update_configure enter\n");

	// There is no default settings, so we update all settings.
	pq_setting_current.peakingUpdate = 1;
	pq_setting_current.sharpnessUpdate = 1;
	if (pq_setting_current.memcDemo == 2) {
		//user define level
		demo_win_FI_update = true; //first panel on, enter MEMC setting
		pr_debug("iris: first time configure user demo window for MEMC setting ---\n");
	}
	pq_setting_current.memcDemoUpdate = 1;
	if (pq_setting_current.peakingDemo == 2) {
		//user define level
		iris_reg_add(PEAKING_STARTWIN, (demo_win_info_setting.startX & 0x3fff) + ((demo_win_info_setting.startY & 0x3fff) << 16));
		iris_reg_add(PEAKING_ENDWIN, (demo_win_info_setting.endX & 0x3fff) + ((demo_win_info_setting.endY & 0x3fff) << 16));
		iris_reg_add(PEAKING_CTRL, 1 | demo_win_info_setting.SharpnessEn<<1);
		iris_reg_add(PEAKING_SHADOW_UPDATE, 1);
		pr_debug("iris: first time configure user demo window for peaking setting ---\n");
	}
	pq_setting_current.peakingDemoUpdate = true;
	pq_setting_current.gammeUpdate = 1;
	pq_setting_current.memcLevelUpdate = 1;
	pq_setting_current.contrastUpdate = 1;
	pq_setting_update = true;

	dbc_setting_current.dbcUpdate = 1;
	dbc_setting_update = true;

	LPMemc_setting_current.value = iris2_lp_memc_calc(LPMemc_setting_current.level);
	LPMemc_setting_update = true;

	color_adjust_update = true;

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
}

void mdss_dsi_panel_cmds_send_ex(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	/*please pay attention to call this funtion, it only used to send write cmd when panel on/off*/
	pinfo = &(ctrl->panel_data.panel_info);
	/* TODO:
		Comment below code for partial update, no impact current system.
		If enable dcs_cmd_by_left, the Iris + panel can't light up.
		Need to debug later.
	*/

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

int iris_register_write(struct msm_fb_data_type *mfd,	u32 addr, u32 value)
{
	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	static char pwil_write[24] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x3),
		0x00,
		0x00,
		PWIL_U16(0x2),
		PWIL_U32(IRIS_PROXY_ADDR + 0x00), //default set to proxy MB0
		PWIL_U32(0x00000000)
	};

	static struct dsi_cmd_desc iris_pwil_write_cmd = {
		{ DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pwil_write) }, pwil_write };

	struct dcs_cmd_req cmdreq;

	if (!iris_cfg->ready) {
		pr_err("%s:%u: iris not ready!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (mfd->panel_power_state == MDSS_PANEL_POWER_OFF)
		return 0;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_debug("%s, addr: 0x%x, value: 0x%x\n", __func__, addr, value);

	pwil_write[16] = addr         & 0xff;
	pwil_write[17] = (addr >>  8) & 0xff;
	pwil_write[18] = (addr >> 16) & 0xff;
	pwil_write[19] = (addr >> 24) & 0xff;
	pwil_write[20] = value          & 0xff;
	pwil_write[21] = (value  >>  8) & 0xff;
	pwil_write[22] = (value  >> 16) & 0xff;
	pwil_write[23] = (value  >> 24) & 0xff;

	cmdreq.cmds = &iris_pwil_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode) {
		while ((atomic_read(&mfd->iris_conf.mode_switch_cnt)))
			usleep(17000);
	}

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode) {
		/* wait 1 vsync to sure command issue */
		usleep(17000);
	}

	return 0;
}

static inline u32 mdss_mdp_cmd_vsync_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_mixer *mixer;
	u32 cnt = 0xffff;	/* init to an invalid value */

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		if (!mixer) {
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
			goto exit;
		}
	}
	cnt = (mdss_mdp_pingpong_read(mixer->pingpong_base, MDSS_MDP_REG_PP_INT_COUNT_VAL) >> 16) & 0xffff;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

exit:
	return cnt;
}

static void iris_proc_te(struct iris_config *iris_cfg, u32 fcnt, u32 lcnt, u32 fps, u32 vper, u32 hper)
{
	static u32 fcnt0, lcnt0;
	static u64 time0;
	static u32 te_period;
	ktime_t ktime = ktime_get();
	u64 time = ktime_to_us(ktime);

	if (fcnt - fcnt0 >= 1200) {
		if (time - time0) {
			u32 detla_t = time - time0;

			te_period = ((fcnt - fcnt0) * vper + lcnt - lcnt0)*1000/fps/(detla_t/1000);
			pr_debug("te_period=%u\n", te_period);
			if (abs(te_period - vper) > (vper >> 5))
				te_period = vper;
		}
		fcnt0 = fcnt;
		lcnt0 = lcnt;
		time0 = time;
	}

	te_period = vper;

	iris_cfg->meta.te_period = te_period;
	pr_debug("fcnt %u fcnt0 %u lcnt %u lcnt0 %u fps %u\n", fcnt, fcnt0, lcnt, lcnt0, fps);
	pr_debug("time %llu time0 %llu\n", time, time0);
	pr_debug("te %u vper %u\n", te_period, vper);
}

static int iris_vary_te(struct iris_config *iris_cfg, u32 fcnt, int vper)
{
#define THRESHOLD 0
#define FRAME_CNT 120
#define PLAYING 0x01
#define FIRST_FRAME 0x04
	static u32 fcnt0, vts0, time0, sts0;
	static bool player_sts;
	u32 time = iris_cfg->meta.sys_ts;
	u32 vts = iris_cfg->meta.video_ts;
	int delta_time, delta_period;
	int delta_t, delta_v, delta_sts, ret_val = false;
	u32 DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
		iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
	ktime_t ktime = ktime_get();
	u32 sts = (u32) ktime_to_us(ktime);

	pr_debug("meta.op=0x%x, meta.flags=0x%x, meta.video_ts=%u, delta_period_range(%d, %d) \n",
		iris_cfg->meta.op, iris_cfg->meta.flags, iris_cfg->meta.video_ts,
		iris_mipi_info.delta_period_max, iris_mipi_info.delta_period_min);

	if (!(iris_cfg->meta.op & MDP_IRIS_OP_FLG)) {
		pr_debug("flag invalid\n");
		if (iris_cfg->sw_te_period != DisplayVtotal) {
			iris_cfg->sw_te_period = DisplayVtotal;
			ret_val = true;
		}
		return ret_val;
	}
	iris_cfg->meta.op &= ~MDP_IRIS_OP_FLG;

	if (!(iris_cfg->meta.flags & PLAYING)) {
		if (player_sts)
			pr_debug("play stop\n");
		//if video is stopped, retore TE to 60hz
		player_sts = 0;
		if (iris_cfg->sw_te_period != DisplayVtotal) {
			iris_cfg->sw_te_period = DisplayVtotal;
			ret_val = true;
		}
		return ret_val;
	}

	//get reference frame
	if (iris_cfg->meta.flags & FIRST_FRAME) {
		player_sts = 1;
		vts0 = iris_cfg->meta.video_ts;
		time0 = time;
		fcnt0 = fcnt;
		sts0 = sts;
		pr_debug("get reference frame ats0 %u vts0 %u sts0 %u f0 %u\n", time0, vts0, sts0, fcnt0);
	}

	delta_t = time - time0;
	delta_v = vts - vts0;
	delta_sts = sts - sts0;
	delta_time = delta_v - delta_t;

	if ((fcnt - fcnt0 >= FRAME_CNT) && vts && delta_t && delta_v && player_sts) {
		if (abs(delta_v - delta_t) > THRESHOLD) {
			// line_time = 1000000us / (60 * vper);
			// delta_period = delta_time / line_time;
			delta_period = (delta_time * vper) / 16667;
			delta_period = DIV_ROUND_CLOSEST(delta_period, FRAME_CNT);

			if (delta_period < iris_mipi_info.delta_period_min) {
				pr_debug("delta_period:%d out of min range\n", delta_period);
				delta_period = iris_mipi_info.delta_period_min;
			} else if (delta_period > iris_mipi_info.delta_period_max) {
				pr_debug("delta_period:%d out of max range\n", delta_period);
				delta_period = iris_mipi_info.delta_period_max;
			}
			iris_cfg->sw_te_period = vper + delta_period;
			ret_val = true;
		}
		pr_debug("fcnt %u fcnt0 %u vts %u vts0 %u delta_v %u\n", fcnt, fcnt0, vts, vts0, delta_v);
		pr_debug("time %u time0 %u delta_t %u\n", time, time0, delta_t);
		pr_debug("sts %u sts0 %u delta_sts %u\n", sts, sts0, delta_sts);
		pr_debug("delta_time %i delta_period %i vper %u ret_val %u\n", delta_time, delta_period, vper, ret_val);

		fcnt0 = fcnt;
		return ret_val;
	}
	return false;
}

static void iris_proc_ct(struct iris_config *iris_cfg, u32 fps)
{
	u32 prev_vts;
	u32 vts;
	u32 te;

	prev_vts = iris_cfg->prev_vts;
	vts = iris_cfg->meta.video_ts;
	te = iris_cfg->meta.te_period;
	iris_cfg->meta.content_period = (vts - prev_vts) * fps / 1000 * te / 1000;
	iris_cfg->meta.content_period_frac = (((vts - prev_vts) * fps / 1000 * te) & 0xfff) / (1000 >> 8);
	iris_cfg->meta.content_period_frac &= 0xff;
}

static void iris_proc_vp(struct iris_config *iris_cfg)
{
	iris_cfg->meta.vs_period = iris_cfg->meta.te_period;
	iris_cfg->meta.vs_period_frac = 0;
}

static void iris_proc_sts(struct iris_config *iris_cfg, u32 fps, u32 lcnt, u32 vper)
{
	u32 sts;
	ktime_t ktime = ktime_get();
	u64 time = ktime_to_us(ktime);

	if (iris_cfg->meta.op & MDP_IRIS_OP_STS) {
		pr_debug("sts %u\n", iris_cfg->meta.sys_ts);
		return;
	}

	sts = (u32) time;
	sts -= 1000000000 / fps / vper * lcnt / 1000;
	iris_cfg->meta.sys_ts = sts;
}

static void iris_proc_restart(struct iris_config *iris_cfg)
{
	if (!(iris_cfg->meta.op & MDP_IRIS_OP_RESTART))
		iris_cfg->meta.restart = 1;
	else
		iris_cfg->meta.restart = (iris_cfg->prev_vts == iris_cfg->meta.video_ts);
}

static int iris_proc_vts(struct iris_config *iris_cfg)
{
	int ret;

	ret = (iris_cfg->prev_vts != iris_cfg->meta.video_ts);
	iris_cfg->prev_vts = iris_cfg->meta.video_ts;
	return ret;
}

static void iris_set_te(struct iris_config *iris_cfg, int te_flag)
{
	if (!debug_te_enabled || !te_flag)
		return;

	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_DTG_ADDR + 0x00064, (iris_cfg->sw_te_period << 8));
	iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);	//reg_update
	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("set_te: %d\n", iris_cfg->sw_te_period);
}

static void iris_set_dtg(struct iris_config *iris_cfg)
{
	if (!debug_dtg_enabled)
		return;
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_DTG_ADDR + 0x10004, iris_cfg->meta.sys_ts);
	iris_reg_add(IRIS_DTG_ADDR + 0x10008, iris_cfg->meta.video_ts);
	iris_reg_add(IRIS_DTG_ADDR + 0x1000c, ((iris_cfg->meta.vs_period & 0xffff) << 8 |
					       (iris_cfg->meta.vs_period_frac & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x10010, iris_cfg->meta.te_period);
	iris_reg_add(IRIS_DTG_ADDR + 0x10014, ((iris_cfg->meta.content_period & 0xffff) << 8 |
					       (iris_cfg->meta.content_period_frac & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x10018, ((iris_cfg->meta.restart & 1) << 8 |
					       (iris_cfg->meta.motion & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x1001c, 1);
	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("dtg set\n");
}

static void iris_proc_scale(struct iris_config *iris_cfg, u32 dvts, u32 prev_dvts)
{
	u32 scale;

	if (abs(dvts-prev_dvts) <= ((dvts + prev_dvts) >> 5))
		scale = 64;
	else {
		scale = (dvts * 64 + prev_dvts / 2) / prev_dvts;
		scale = min_t(u32, 255, scale);
		scale = max_t(u32, 16, scale);
	}
	iris_cfg->scale = scale;
	pr_debug("pdvts %u dvts %u scale %u\n", prev_dvts, dvts, scale);
}

static void iris_set_constant_ratio(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out, reg_scale, reg_cap;

	reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
	reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
	reg_scale = 4096/iris_cfg->scale << 24 | 64 << 16 | iris_cfg->scale << 8 | iris_cfg->scale;
	/* duplicated video frame */
	reg_cap = (iris_cfg->meta.repeat != IRIS_REPEAT_CAPDIS) << 1;
	reg_cap |= 0xc0000001;
	iris_cfg->iris_ratio_updated = true;

	pr_debug("reg_cap 0x%08x\n", reg_cap);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
	iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
	if (debug_new_repeat == 0)
		iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	iris_reg_add(IRIS_MVC_ADDR + 0x1D0, reg_scale);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);
	mutex_unlock(&iris_cfg->cmd_mutex);
}

int iris_proc_constant_ratio(struct iris_config *iris_cfg)
{
	u32 dvts, in_t, out_t;
	uint32_t r;

	dvts = 1000000 / iris_cfg->input_frame_rate;
	in_t = (dvts * iris_cfg->output_frame_rate + 50000) / 100000;
	out_t = 10;

	r = gcd(in_t, out_t);
	pr_debug("in_t %u out_t %u r %u\n", in_t, out_t, r);
	iris_cfg->in_ratio = out_t / r;
	iris_cfg->out_ratio = in_t / r;
	iris_proc_scale(iris_cfg, dvts, dvts);
	// in true-cut case, always keep 1:1
	if (iris_cfg->true_cut_enable) {
		iris_cfg->in_ratio = 1;
		iris_cfg->out_ratio = 1;
		iris_cfg->scale = 64;
	}
	pr_debug("in/out %u:%u\n", iris_cfg->in_ratio, iris_cfg->out_ratio);
	// update register
	iris_set_constant_ratio(iris_cfg);

	return 0;
}

static int iris_proc_ratio(struct iris_config *iris_cfg)
{
	int ret = 0;
	u32 prev_dvts;
	u32 dvts, in_t, out_t;
	uint32_t r;

	if (!(iris_cfg->meta.op & MDP_IRIS_OP_VTS))
		return 0;

	dvts = iris_cfg->meta.video_ts - iris_cfg->prev_vts;
	prev_dvts = iris_cfg->prev_dvts;

	pr_debug("vts %u pvts %u dvts %u\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, dvts);
	if (dvts > 200000)
		return 0;

	if ((iris_cfg->iris_ratio_updated == true) && (abs(dvts - prev_dvts) < 3000))
		return 0;

	if (iris_cfg->repeat == IRIS_REPEAT_FORCE)
		return 0;

	if (iris_cfg->true_cut_enable)
		return 0;

	if (debug_hlmd_enabled) {
		pr_debug("enable hlmd function.\n");
		// constant ratio
		ret = iris_proc_constant_ratio(iris_cfg);
		iris_cfg->prev_dvts = dvts;
		return ret;
	} else {
		//pr_debug("don't enable hlmd function.\n");
	}

	if (prev_dvts && dvts) {
		in_t = (dvts * iris_cfg->output_frame_rate + 50000) / 100000;
		out_t = 10;

		r = gcd(in_t, out_t);
		pr_debug("in_t %u out_t %u r %u\n", in_t, out_t, r);
		iris_cfg->in_ratio = out_t / r;
		iris_cfg->out_ratio = in_t / r;
		iris_proc_scale(iris_cfg, dvts, prev_dvts);
		iris_cfg->iris_ratio_updated = (abs(dvts - prev_dvts) < 3000) ? true : false;
		ret = 1;
		pr_debug("in/out %u:%u\n", iris_cfg->in_ratio, iris_cfg->out_ratio);
	}

	if (prev_dvts && !dvts)
		ret = 1;

	if (dvts)
		iris_cfg->prev_dvts = dvts;

	return ret;
}

void iris_calc_nrv(struct mdss_mdp_ctl *ctl)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	uint16_t width, height;

#define VIDEO_CTRL3	0x104c
#define VIDEO_CTRL4	0x1050
#define VIDEO_CTRL5	0x1054
#define VIDEO_CTRL11	0x106c
#define VIDEO_CTRL12	0x1070
#define DISP_CTRL2		0x120c
#define REG_UPDATE      0x10000
#define MB1				0x0008
#define MB2				0x0010
#define MB4				0x0020
#define MB6				0x0030

	if (iris_cfg->meta.op & MDP_IRIS_OP_NRV) {
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL3, ((uint32_t)iris_cfg->meta.nrv.captureTop << 16) |
													(uint32_t)iris_cfg->meta.nrv.captureLeft);
		width = iris_cfg->meta.nrv.captureRight - iris_cfg->meta.nrv.captureLeft;
		height = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL4, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL5, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL11, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL12, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + DISP_CTRL2, ((uint32_t)iris_cfg->meta.nrv.displayTop << 16) |
													(uint32_t)iris_cfg->meta.nrv.displayLeft);
		iris_reg_add(IRIS_PWIL_ADDR + REG_UPDATE, 0x100);

		width = iris_cfg->meta.nrv.displayRight - iris_cfg->meta.nrv.displayLeft;
		height = iris_cfg->meta.nrv.displayBottom - iris_cfg->meta.nrv.displayTop;
		iris_reg_add(IRIS_PROXY_ADDR + MB4, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PROXY_ADDR + MB6, ((uint32_t)iris_cfg->meta.nrv.displayTop << 16) |
											(uint32_t)iris_cfg->meta.nrv.displayLeft);

		//iris_reg_add(IRIS_PROXY_ADDR + MB1, )
		//iris_reg_add(IRIS_PROXY_ADDR + MB2, 1<<10);
		if (iris_cfg->meta.nrv.nrvEnable)
			g_mfd->iris_conf.frc_path |= BIT(10);
		else
			g_mfd->iris_conf.frc_path &= ~BIT(10);
		iris_cfg->nrv_enable = iris_cfg->meta.nrv.nrvEnable;
	}
}

static void iris_calc_true_cut(struct msm_fb_data_type *mfd) {
	struct iris_config *iris_cfg = &mfd->iris_conf;
	if (iris_cfg->meta.op & MDP_IRIS_OP_IF1) {
		uint32_t info_header = iris_cfg->meta.iris_info1 >> 28;
		pr_debug("true cut: %x\n", iris_cfg->meta.iris_info1);
		if (info_header == 0x8 || info_header == 0x9 ||
				info_header == 0xa || info_header == 0xb) {
			if (debug_true_cut) {
				if (info_header == 0x8)
					iris_cfg->input_vfr = 50;
				else if (info_header == 0x9)
					iris_cfg->input_vfr = 60;
				else if (info_header == 0xa)
					iris_cfg->input_vfr = 15;
				else if (info_header == 0xb)
					iris_cfg->input_vfr = 15;
				iris_cfg->frc_path |= BIT(11);
				iris_cfg->true_cut_enable = true;
				if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC)
					iris_reg_add(IRIS_PROXY_ADDR + 0x0030, iris_cfg->meta.iris_info1);
			}
		} else {
			iris_cfg->input_vfr = 0;
			iris_cfg->frc_path &= ~BIT(11);
			iris_cfg->true_cut_enable = false;
		}
	}
}

int iris_calc_meta(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	struct mdss_panel_data *pdata = mdp5_data->ctl->panel_data;
	u32 fps, fcnt, lcnt;
	u32 vper, hper;
	int ret = 0, te_flag = 0;
	iris_calc_true_cut(ctl->mfd);
	if ((atomic_read(&mfd->iris_conf.mode_switch_cnt))) {
		iris_proc_vts(iris_cfg);
		return ret;
	}

	if (!debug_send_meta_enabled)
		return ret;
	// TODO
	//if (iris_cfg->current_mode != IRIS_MEMC_MODE)
	//	return 0;

	fps = mdss_panel_get_framerate(&pdata->panel_info);
	vper = mdss_panel_get_vtotal(&pdata->panel_info);
	hper = mdss_panel_get_htotal(&pdata->panel_info, false);

	if (fps == 0)
		return ret;

	lcnt = 0;//ctl->read_line_cnt_fnc(ctl);
	if (pdata->panel_info.type == MIPI_CMD_PANEL) {
		fcnt = mdss_mdp_cmd_vsync_count(ctl);
		iris_proc_sts(iris_cfg, fps, lcnt, vper);
		iris_proc_restart(iris_cfg);
		iris_proc_te(iris_cfg, fcnt, lcnt, fps, vper, hper);
		te_flag = iris_vary_te(iris_cfg, fcnt, (int)vper);
		iris_proc_ct(iris_cfg, fps);
		iris_proc_vp(iris_cfg);
		ret = iris_proc_vts(iris_cfg);
	} else if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		fcnt = mdss_mdp_video_vsync_count(ctl);
		iris_proc_sts(iris_cfg, fps, lcnt, vper);
		iris_proc_restart(iris_cfg);
		ret = iris_proc_vts(iris_cfg);
	}

	if (pdata->panel_info.type == MIPI_CMD_PANEL)
		iris_set_te(iris_cfg, te_flag);

	iris_set_dtg(iris_cfg);

	pr_debug("sts=%u fps=%u fcnt=%u vts=%u restart=%u in_ratio=%u out_ratio=%u\n", iris_cfg->meta.sys_ts, fps, fcnt,
		iris_cfg->meta.video_ts, iris_cfg->meta.restart, iris_cfg->in_ratio, iris_cfg->out_ratio);

	memset((void *)&iris_cfg->meta, 0, sizeof(struct iris_meta));

	return ret;
}

int iris_set_configure(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;

	if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE ||
		iris_cfg->sf_notify_mode == IRIS_MODE_FRC_PREPARE_DONE)
		return 0;

	if (first_boot) {
		iris_update_configure();
		first_boot = 0;
	}

	// no update
	if (!pq_setting_update && !dbc_setting_update && !LPMemc_setting_update
			&& !black_border_update)
		return 0;

	mutex_lock(&iris_cfg->config_mutex);
	// PQ setting, MB3
	if (pq_setting_update) {
		iris_reg_add(IRIS_PQ_SETTING_ADDR, *((u32 *)&pq_setting_current));
		pq_setting_current.peakingUpdate = 0;
		pq_setting_current.sharpnessUpdate = 0;
		pq_setting_current.memcDemoUpdate = 0;
		pq_setting_current.peakingDemoUpdate = 0;
		pq_setting_current.gammeUpdate = 0;
		pq_setting_current.memcLevelUpdate = 0;
		pq_setting_current.contrastUpdate = 0;
		pq_setting_update = false;
	}

	// DBC setting, MB5
	if (dbc_setting_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x28, *((u32 *)&dbc_setting_current));
		dbc_setting_current.dbcUpdate = 0;
		dbc_setting_update = false;
	}

	if (LPMemc_setting_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x20, LPMemc_setting_current.value | 0x80000000);
		LPMemc_setting_current.update = 0;
		LPMemc_setting_update = false;
	}

	if (black_border_update) {
		iris_reg_add(IRIS_MVC_ADDR + 0x17c, black_border_value ? 0x100c201f : 0x1006201f);
		black_border_update = false;
	}

	if (color_adjust_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x30, color_adjust_current_value);
		color_adjust_update = false;
	}
	mutex_unlock(&iris_cfg->config_mutex);

	return 0;
}

static int iris_set_repeat(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out;
	unsigned int val_frcc_cmd_th = iris_cfg->val_frcc_cmd_th;
	unsigned int val_frcc_reg8 = iris_cfg->val_frcc_reg8;
	unsigned int val_frcc_reg16 = iris_cfg->val_frcc_reg16;
	bool cap_enable = true;

	if (!debug_repeat_enabled || iris_cfg->sf_notify_mode != IRIS_MODE_FRC)
		return true;

	if ((iris_cfg->repeat == IRIS_REPEAT_FORCE) && (!frc_repeat_enter)) {
		reg_in = (1 << IRIS_PWIL_IN_FRAME_SHIFT) | (1 << 15);
		reg_out = 1 << IRIS_PWIL_OUT_FRAME_SHIFT;
		val_frcc_cmd_th &= 0x1fffffff;
		val_frcc_cmd_th |= 0x20000000;
		frc_repeat_enter = true;
		val_frcc_reg8 |= 0x3f00;
		val_frcc_reg16 &= 0xffff7fff;
		mutex_lock(&iris_cfg->cmd_mutex);
		iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
		iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
		iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
		iris_reg_add(FRCC_REG_SHOW, 0x2);
		if (!iris_cfg->true_cut_enable) {	// in true-cut case, always keep 1:1
			iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
			iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
			iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
		}
		mutex_unlock(&iris_cfg->cmd_mutex);
	} else if (iris_cfg->repeat != IRIS_REPEAT_FORCE) {
		reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
		reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
		cap_enable = (iris_cfg->repeat != IRIS_REPEAT_CAPDIS);
		mutex_lock(&iris_cfg->cmd_mutex);
		if (frc_repeat_enter) {
			frc_repeat_enter = false;
			iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
			iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
			iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
			iris_reg_add(FRCC_REG_SHOW, 0x2);
			if (!iris_cfg->true_cut_enable) {
				iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
				iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
				iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
			}
		}
		mutex_unlock(&iris_cfg->cmd_mutex);
	}

	pr_debug("vts %u pvts %u cap_en %d\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, cap_enable);

	return cap_enable;
}

void iris_cmd_cadence_check(struct mdss_mdp_ctl *ctl)
{
	static u32 prev_frame_addr;
	static u32 prev_frame_count, prev_frames;
	static u32 cadence[MAX_CAD_LEN][3];
	static int count[MAX_CAD_LEN];
	static int cadence_length, cadence_count;
	static int skip;

	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(ctl->mfd);
	struct mdss_mdp_mixer *mixer;
	u32 frame_addr, frame_count, frames;
	int i;
	char *addr;

	if (!mdp5_data)
		return;
	/* another CPU could change  mdp5_data->yuv_frame_addr_reg while the ISR is running
	use volatile to protect */
	addr = ACCESS_ONCE(mdp5_data->yuv_frame_addr_reg);
	if (!addr)
		return;

	// only check the left ctl
	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer)
		return;

	frame_count = (mdss_mdp_pingpong_read(mixer->pingpong_base, MDSS_MDP_REG_PP_INT_COUNT_VAL) >> 16) & 0xffff;
	frames = frame_count - prev_frame_count;
	frame_addr = readl_relaxed(addr);
	//pr_debug("=== frame %08x count %u diff %u\n",
	//	 frame_addr, frame_count, frames);
	if (frame_addr == prev_frame_addr)
		return;
	prev_frame_addr = frame_addr;
	prev_frame_count = frame_count;
	ATRACE_INT("KFrameDiff", frames);
	for (i = 0; i < MAX_CAD_LEN; i++)
		cadence[i][0] = ((cadence[i][0] << 4) | (frames & 0x0f)) &
				GENMASK((i + 1) * 4 - 1, 0);
	if (cadence_length && ++cadence_count >= cadence_length) {
		cadence_count = 0;
		i = cadence_length - 1;
		if (cadence[i][0] != cadence[i][1]) {
			ATRACE_INT("CadenceSkip", skip);
			skip = !skip;
			pr_debug("=== ctl %u frame %08x count %u: "
				 "cadence skip l %d (%08x %08x)\n",
				 ctl->num, frame_addr, frame_count,
				 cadence_length, cadence[i][1], cadence[i][0]);
			cadence_length = 0;
		}
	}
	if (!cadence_length) {
		for (i = 0; i < MAX_CAD_LEN; i++) {
			if (cadence[i][0] == cadence[i][1] &&
			    cadence[i][0] == cadence[i][2])
				cadence_length = i + 1;
		}
	}
	for (i = 0; i < MAX_CAD_LEN; i++) {
		if (++count[i] > i) {
			cadence[i][2] = cadence[i][1];
			cadence[i][1] = cadence[i][0];
			count[i] = 0;
		}
	}
	prev_frames = frames;
}

int auo_panel_bl_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 value)
{
	//struct mdss_overlay_private *mdp5_data;
	//struct mdss_panel_data *pdata;

	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	static u8 pwil_write[80] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x11),
		0x00,
		0x00,
		PWIL_U16(0x10),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014-0x40000),
		PWIL_U32(0x04510000),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010-0x40000),
		PWIL_U32(0x00008001),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018-0x40000),
		PWIL_U32(0x00000000),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020-0x40000),
		PWIL_U32(0x00000001),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014),
		PWIL_U32(0x04510000),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010),
		PWIL_U32(0x00008001),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018),
		PWIL_U32(0x00000000),
		PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020),
		PWIL_U32(0x00000001)
	};
	static struct dsi_cmd_desc iris_pwil_write_cmd = {
		{ DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pwil_write) }, pwil_write };

	struct dcs_cmd_req cmdreq;
	pwil_write[36] = value & 0xff;
	pwil_write[68] = value & 0xff;

	if (!iris_cfg->ready) {
		pr_err("%s:%u: iris not ready!\n", __func__, __LINE__);
		return -EINVAL;
	}
	cmdreq.cmds = &iris_pwil_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	//cmdreq.flags =  CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	pr_debug("%s: bl value: %d\n", __func__, value);
	return 0;
}

u32  iris_pi_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr, u32 value)
{
	//struct mdss_overlay_private *mdp5_data;
	//struct mdss_panel_data *pdata;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	static char pwil_write[24] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x3),
		0x00,
		0x00,
		PWIL_U16(0x2),
		PWIL_U32(IRIS_PROXY_ADDR + 0x00), //default set to proxy MB0
		PWIL_U32(0x00000000)
	};

	static struct dsi_cmd_desc iris_pwil_write_cmd = {
		{ DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pwil_write) }, pwil_write };

	struct dcs_cmd_req cmdreq;

	if (!iris_cfg->ready) {
		pr_err("%s:%u: iris not ready!\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_debug("%s, addr: 0x%x, value: 0x%x\n", __func__, addr, value);

	pwil_write[16] = addr         & 0xff;
	pwil_write[17] = (addr >>  8) & 0xff;
	pwil_write[18] = (addr >> 16) & 0xff;
	pwil_write[19] = (addr >> 24) & 0xff;
	pwil_write[20] = value          & 0xff;
	pwil_write[21] = (value  >>  8) & 0xff;
	pwil_write[22] = (value  >> 16) & 0xff;
	pwil_write[23] = (value  >> 24) & 0xff;

	cmdreq.cmds = &iris_pwil_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	return 0;
}

u32 iris_pi_read(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr)
{
	u32 value;

	char pi_address[16] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('S', 'G', 'L', 'W'),
		PWIL_U32(0x01),	//valid body word(4bytes)
		PWIL_U32(IRIS_PROXY_ADDR),   // proxy MB0
	};

	struct dsi_cmd_desc pi_read_addr_cmd[] = {
		{ { DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pi_address) }, pi_address },
	};

	//char pktsize[2] = {0x04, 0x00}; /* LSB tx first, 10 bytes */

	char pi_read[1] = { 0x00 };
	struct dsi_cmd_desc pi_read_cmd = {
		{ DTYPE_GEN_READ1,   1, 0, 1, 0, sizeof(pi_read) }, pi_read
	};

	char read_buf[16]; //total 4*32bit register
	struct dcs_cmd_req cmdreq;

	pi_address[12] = addr         & 0xff;
	pi_address[13] = (addr >>  8) & 0xff;
	pi_address[14] = (addr >> 16) & 0xff;
	pi_address[15] = (addr >> 24) & 0xff;

	cmdreq.cmds = pi_read_addr_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	cmdreq.cmds = &pi_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_NO_MAX_PKT_SIZE;
	cmdreq.rlen = 4;
	cmdreq.rbuf = (char *)read_buf;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	value = ctrl->rx_buf.data[0] | (ctrl->rx_buf.data[1] << 8) |
		(ctrl->rx_buf.data[2] << 16) | (ctrl->rx_buf.data[3] << 24);

	return value;
}

static int check_mode_status(struct mdss_dsi_ctrl_pdata *ctrl, int mode)
{
	int i;
	u32 val = 0;
	int try_cnt = 10;
	int ret = 0;

	if (!debug_mode_switch_enabled)
		return ret;

	for (i = 0; i < try_cnt; i++) {
		msleep(16);
		val = iris_pi_read(ctrl, IRIS_PROXY_ADDR + 0x08);
		if (val == mode)
			break;
		pr_err("%s:%d: %08x, cnt = %d\n", __func__, __LINE__, val, i);

	}

	if (i == try_cnt) {
		pr_err("%s: check mode (%d) error\n", __func__, mode);
		ret = -1;
	}
	return ret;
}

static void iris_pt_entry_wq_handler(struct work_struct *work)
{

	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);

	if (iris_mipi_info.iris_timing_flag
				|| (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)) {
		pt_enable[0] = 0x3 << 2;
		pt_enable[1] = 0x1;
	} else {
		pt_enable[0] = 0x0;
		pt_enable[1] = 0x1;
	}
	// pt_mode_enter
	mdss_dsi_cmds_tx(ctrl, pt_mode_enter,
			ARRAY_SIZE(pt_mode_enter), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	// pt_data_path_config
	iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
	mdss_dsi_cmds_tx(ctrl, pt_data_path_config,
			ARRAY_SIZE(pt_data_path_config), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

static void iris_memc_prepare_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_info *pinfo = g_mfd->panel_info;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || pinfo == NULL);

	if (check_mode_status(ctrl, IRIS_MEMC_MODE) == 0) {
		iris_cfg->sf_notify_mode = IRIS_MODE_FRC_PREPARE_DONE;
		pw_iris2_status = IRIS_MODE_FRC_PREPARE_DONE;
	}
	else
		pr_err("%s IRIS_MODE_FRC_PREPARE failed\n", __func__);

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------sf_notify_mode = %d\n", __func__, iris_cfg->sf_notify_mode);
}

static void iris_memc_cancel_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || iris_cfg == NULL);

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

static void iris_memc_entry_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_info *pinfo = g_mfd->panel_info;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || pinfo == NULL);
	iris_proc_frcc_setting(g_mfd);
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	mdss_dsi_cmds_tx(ctrl, memc_mode_enter,
			ARRAY_SIZE(memc_mode_enter), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

static void iris_meta_wq_handler(void)
{
	struct mdss_mdp_ctl *ctl = mfd_to_ctl(g_mfd);
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_data *pdata = ctl->panel_data;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	int cmd;
	int cnt = 2;

	//if (ctl->power_state == MDSS_PANEL_POWER_OFF)
	//	return;
	if (pdata->panel_info.panel_power_state == MDSS_PANEL_POWER_OFF)
		return;
	pr_debug("%s ++++++\n", __func__);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_regs_meta_build();
	iris_regs_clear();
	mutex_unlock(&iris_cfg->cmd_mutex);
	// TODO: when okay use ioctl or other side band to enable new frame
	iris_meta_pkts[0].dchdr.last = debug_new_frame_enabled ? 0 : 1;
	iris_meta_pkts[1].dchdr.last = debug_new_frame_enabled ? 1 : 0;
	for (cmd = 0; cmd < cnt; cmd++) {
		pr_debug("dchdr: %02x %02x %02x %02x %02x %02x\n",
		iris_meta_pkts[cmd].dchdr.dtype,
		iris_meta_pkts[cmd].dchdr.last,
		iris_meta_pkts[cmd].dchdr.vc,
		iris_meta_pkts[cmd].dchdr.ack,
		iris_meta_pkts[cmd].dchdr.wait,
		iris_meta_pkts[cmd].dchdr.dlen);
		{
		int i;
		for (i = 0; i < iris_meta_pkts[cmd].dchdr.dlen; i += 8)
			pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			iris_meta_pkts[cmd].payload[i],   iris_meta_pkts[cmd].payload[i+1],
			iris_meta_pkts[cmd].payload[i+2], iris_meta_pkts[cmd].payload[i+3],
			iris_meta_pkts[cmd].payload[i+4], iris_meta_pkts[cmd].payload[i+5],
			iris_meta_pkts[cmd].payload[i+6], iris_meta_pkts[cmd].payload[i+7]);
		}
	}
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	// TODO: assume 2 meta packet will both issued at same kickoff
	if (iris_meta_pkts[0].dchdr.dlen > META_HEADER)
		mdss_dsi_cmds_tx(ctrl, iris_meta_pkts, cnt, (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	memset(imeta, 0, sizeof(imeta));
	pr_debug("%s ------\n", __func__);
}

 void iris_regs_clear(void)
{
	iris_reg_cnt = 0;
	//memset(iris_regs, 0, sizeof(iris_regs));
}

void iris_reg_add(u32 addr, u32 val)
{
	if (iris_reg_cnt >= IRIS_REGS)
		return;
	pr_debug("regs[%i:%08x] = %08x\n", iris_reg_cnt, addr, val);
	iris_regs[iris_reg_cnt].addr = addr;
	iris_regs[iris_reg_cnt].val = val;
	iris_reg_cnt++;
}

static int  iris_regs_meta_build(void)
{
	int i;
	int size;

	pr_debug("reg_cnt: %02x", iris_reg_cnt);
	memcpy(imeta, imeta_header, META_HEADER);
	// pair
	for (i = 0; i < iris_reg_cnt; i++) {
		*(u32 *)(imeta + META_HEADER + i*8) = cpu_to_le32(iris_regs[i].addr);
		*(u32 *)(imeta + META_HEADER + i*8 + 4) = cpu_to_le32(iris_regs[i].val);
		/*
		imeta[META_HEADER + i*8    ] = iris_regs[i].addr         & 0xff;
		imeta[META_HEADER + i*8 + 1] = (iris_regs[i].addr >>  8) & 0xff;
		imeta[META_HEADER + i*8 + 2] = (iris_regs[i].addr >> 16) & 0xff;
		imeta[META_HEADER + i*8 + 3] = (iris_regs[i].addr >> 24) & 0xff;

		imeta[META_HEADER + i*8 + 4] = iris_regs[i].addr         & 0xff;
		imeta[META_HEADER + i*8 + 5] = (iris_regs[i].addr >>  8) & 0xff;
		imeta[META_HEADER + i*8 + 6] = (iris_regs[i].addr >> 16) & 0xff;
		imeta[META_HEADER + i*8 + 7] = (iris_regs[i].addr >> 24) & 0xff;
		*/
	}
	// size update
	size = iris_reg_cnt * 2;
	*(u32 *)(imeta + 8) = cpu_to_le32(size + 1);
	*(u16 *)(imeta + 14) = cpu_to_le16(size);
	iris_meta_pkts[0].dchdr.dlen = META_HEADER + iris_reg_cnt * 8;
	return iris_reg_cnt;
}

void iris_copy_meta(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	// copy meta
	mutex_lock(&iris_cfg->meta_mutex);
	if (iris_cfg->meta_set.op) {
		memcpy((void *)&iris_cfg->meta, (void *)&iris_cfg->meta_set, sizeof(struct iris_meta));
		memset((void *)&iris_cfg->meta_set, 0, sizeof(struct iris_meta));
		pr_debug("iris_copy_meta\n");
	}
	mutex_unlock(&iris_cfg->meta_mutex);
}

/*
 * There are 3 commands in iris_meta_pkts
 * first is meta, and second is for capture enable/disable. Third is FBO.
 * When capture changed:
 * If meta is null(iris_reg_cnt), send the capture command directly.
 * If meta is not null, copy capture command to iris_meta_pkts[1], and send together.
 *
 */

void iris_send_meta_cmd(struct mdss_mdp_ctl *ctl)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct dcs_cmd_req cmdreq;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	int cmd;

	BUG_ON(ctrl == NULL);

	if(entry_mode==2){
		return;
	}

	if (!iris_cfg->ready) {
		pr_info("%s ++++++, iris not ready!\n", __func__);
		return;
	}

	if (!debug_send_meta_enabled)
		return;

	if ((atomic_read(&g_mfd->iris_conf.mode_switch_cnt)))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	if (iris_cfg->cap_change) {
		static char cursor_enable[2] = {0x04, 0x10};
		static struct dsi_cmd_desc cursor_mode_enter[] = {
			{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(cursor_enable) }, cursor_enable},};
		if (iris_cfg->cap_enable)
			cmdreq.cmds = memc_mode_enter;
		else
			cmdreq.cmds = cursor_mode_enter;
		if(iris_reg_cnt == 0) {
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			iris_cfg->cap_change = false;
		} else {
			memcpy(&iris_meta_pkts[1], cmdreq.cmds, sizeof(struct dsi_cmd_desc));
		}
		pr_debug("cap_change: %d\n", iris_cfg->cap_enable);
	}

	if (iris_reg_cnt !=0)
		pr_debug("%s ++++++, iris_reg_cnt: %d\n", __func__, iris_reg_cnt);

	mutex_lock(&g_mfd->iris_conf.cmd_mutex);
	if (!iris_regs_meta_build()) {
		mutex_unlock(&g_mfd->iris_conf.cmd_mutex);
		return;
	}

	iris_regs_clear();
	mutex_unlock(&g_mfd->iris_conf.cmd_mutex);
	cmdreq.cmds = iris_meta_pkts;

	if (iris_cfg->cap_change == false) {
		iris_meta_pkts[0].dchdr.last = 1;
	} else {
		iris_meta_pkts[0].dchdr.last = 0;
		iris_meta_pkts[1].dchdr.last = 1;
		cmdreq.cmds_cnt = 2;
		iris_cfg->cap_change = false;
	}

	for (cmd = 0; cmd < cmdreq.cmds_cnt; cmd++) {
		pr_debug("dchdr: %02x %02x %02x %02x %02x %02x\n",
			iris_meta_pkts[cmd].dchdr.dtype,
			iris_meta_pkts[cmd].dchdr.last,
			iris_meta_pkts[cmd].dchdr.vc,
			iris_meta_pkts[cmd].dchdr.ack,
			iris_meta_pkts[cmd].dchdr.wait,
			iris_meta_pkts[cmd].dchdr.dlen);
		{
		int i;
		for (i = 0; i < iris_meta_pkts[cmd].dchdr.dlen; i += 8)
			pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
				iris_meta_pkts[cmd].payload[i],   iris_meta_pkts[cmd].payload[i+1],
				iris_meta_pkts[cmd].payload[i+2], iris_meta_pkts[cmd].payload[i+3],
				iris_meta_pkts[cmd].payload[i+4], iris_meta_pkts[cmd].payload[i+5],
				iris_meta_pkts[cmd].payload[i+6], iris_meta_pkts[cmd].payload[i+7]);
		}
	}

	mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);

	//memset(imeta, 0, sizeof(imeta));
}

void iris_send_meta_video(struct mdss_mdp_ctl *ctl)
{
	struct msm_fb_data_type *mfd = ctl->mfd;
	struct iris_config *iris_cfg = &mfd->iris_conf;

	BUG_ON(iris_cfg == NULL);

	if (!debug_send_meta_enabled)
		return;

	if ((atomic_read(&g_mfd->iris_conf.mode_switch_cnt)))
		return;

	//schedule_work(&iris_cfg->meta_work);
	iris_mgmt.iris_handler = iris_meta_wq_handler;
	queue_work(iris_mgmt.iris_wq, &iris_mgmt.iris_worker);
}

// shall be called before params_changed clear to 0
static int iris_proc_repeat(struct iris_config *iris_cfg)
{
	u8 prev_repeat;
	int ret;

	prev_repeat = iris_cfg->repeat;

	if (debug_repeat_enabled > 3)
		iris_cfg->repeat = debug_repeat_enabled - 3;
	else
		iris_cfg->repeat = (iris_cfg->meta.op & MDP_IRIS_OP_RPT) ? iris_cfg->meta.repeat : iris_cfg->repeat;

	pr_debug("repeat = %d\n", iris_cfg->repeat);

	ret = ((iris_cfg->repeat != prev_repeat) || (iris_cfg->repeat == IRIS_REPEAT_FORCE));
	return ret;
}

static int iris_set_ratio(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out, reg_scale;
	bool cap_enable;

	reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
	reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
	reg_scale = 4096/iris_cfg->scale << 24 | 64 << 16 | iris_cfg->scale << 8 | iris_cfg->scale;
	/* duplicated video frame */
	cap_enable = iris_cfg->repeat != IRIS_REPEAT_CAPDIS;
	/* set ratio after mode switch to FRC */
	if (!debug_ratio_enabled || iris_cfg->sf_notify_mode != IRIS_MODE_FRC)
		return true;

	if (iris_cfg->true_cut_enable)
		return cap_enable;

	pr_debug("vts %u pvts %u cap_enable %d\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, cap_enable);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
	iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	iris_reg_add(IRIS_MVC_ADDR + 0x1D0, reg_scale);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);
	mutex_unlock(&iris_cfg->cmd_mutex);
	return cap_enable;
}

bool iris_frc_repeat(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int ret_r, ret_p;
	int cap_enable = true;
	bool ret;

	if (iris_cfg->sf_notify_mode != IRIS_MODE_FRC)
		return cap_enable;

	ret_r = iris_proc_ratio(iris_cfg);
	ret_p = iris_proc_repeat(iris_cfg);
	if (ret_p)
		cap_enable = iris_set_repeat(iris_cfg);
	else if (ret_r)
		cap_enable = iris_set_ratio(iris_cfg);
	else {
		cap_enable = iris_cfg->cap_enable;
		pr_debug("keep the last value: %d!\n", cap_enable);
	}

	if (iris_cfg->sf_notify_mode == IRIS_MODE_FRC) {
		if (cap_enable != iris_cfg->cap_enable) {
			pr_debug("capture-change: %d!\n", cap_enable);
			if (debug_new_repeat == 1)
				iris_cfg->cap_change = true;
			else if (debug_new_repeat == 0) {
				unsigned int reg_cap;

				if (cap_enable)
					reg_cap = 0xc0000003;
				else
					reg_cap = 0xc0000001;
				mutex_lock(&iris_cfg->cmd_mutex);
				iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
				mutex_unlock(&iris_cfg->cmd_mutex);
			}
			iris_cfg->cap_enable = cap_enable;
		}
	}

	ret = ((debug_new_repeat == 2) ? cap_enable : true);
	return ret;
}

static void iris_cmds_tx(struct work_struct *data)
{
	struct iris_mgmt_t *mgmt = container_of(data, struct iris_mgmt_t, iris_worker);

	if (mgmt->iris_handler)
		mgmt->iris_handler();
}

void mdss_dsi_iris_init_ctl(struct mdss_mdp_ctl *ctl)
{
	static int flag;

	if (flag == 0) {
		g_ctl0 = ctl;
		pr_debug("###%s:%d: g_ctl: %p\n", __func__, __LINE__, g_ctl0);
	} else if (flag == 1) {
		g_ctl1 = ctl;
		pr_debug("###%s:%d: g_ctl1: %p\n", __func__, __LINE__, g_ctl1);
	}
	flag++;
}

void mdss_dsi_iris_init_pdata(struct mdss_panel_data *pdata)
{
	static int flag;

	if (flag == 0) {
		g_pdata0 = pdata;
		pr_debug("###%s:%d: g_pdata: %p\n", __func__, __LINE__, g_pdata0);
	} else if (flag == 1) {
		g_pdata1 = pdata;
		pr_debug("###%s:%d: g_pdata1: %p\n", __func__, __LINE__, g_pdata1);
	}
	flag++;
}

void mdss_dsi_iris_init(struct msm_fb_data_type *mfd)
{
	pr_info("###%s:%d: mfd->panel.type: %i mfd->panel.id: %i\n", __func__, __LINE__, mfd->panel.type, mfd->panel.id);
	if (mfd->index != 0)
		return;
	if (!(mfd->panel.type == MIPI_VIDEO_PANEL || mfd->panel.type == MIPI_CMD_PANEL))
		return;

	g_mfd = mfd;
	pr_info("###%s:%d: g_mfd: %p\n", __func__, __LINE__, g_mfd);
	iris_mgmt.iris_wq = create_singlethread_workqueue("iris_wq");
	INIT_WORK(&iris_mgmt.iris_worker, iris_cmds_tx);

	mfd->iris_conf.current_mode = IRIS_PT_MODE;
	mfd->iris_conf.sf_notify_mode  = IRIS_MODE_RFB;
	mfd->iris_conf.mode_changed = false;
	mfd->iris_conf.fbo_enable = false;
	mfd->iris_conf.memc_enable = false;
	mfd->iris_conf.memc_perf_hack = false;
	mfd->iris_conf.mode_switch_finish = true;
	mfd->iris_conf.repeat = IRIS_REPEAT_NO;
	mfd->iris_conf.video_on = false;
	mfd->iris_conf.frc_path = 0x800000;
	atomic_set(&mfd->iris_conf.mode_switch_cnt, 0);
	mfd->iris_conf.input_frame_rate = 60;
	mfd->iris_conf.output_frame_rate = 60;
	mfd->iris_conf.input_vfr = 0;
	mfd->iris_conf.in_ratio = 1;
	mfd->iris_conf.out_ratio = 1;
	mfd->iris_conf.vp_continous = 0;
	mfd->iris_conf.nrv_enable = false;
	mfd->iris_conf.true_cut_enable = false;
	mfd->iris_conf.ready = false;
	mfd->iris_conf.prev_dvts = 0;
	mfd->iris_conf.iris_ratio_updated = false;
	mfd->iris_conf.cap_change = false;
	mfd->iris_conf.check_appcode_rfb_ready = false;
	mfd->iris_conf.check_pwil_rfb_ready = false;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	mfd->iris_conf.drc_enable = false;
	mfd->iris_conf.drc_size = 0;
#endif
	memset((void *)&mfd->iris_conf.meta, 0, sizeof(struct iris_meta));
	memset((void *)&mfd->iris_conf.meta_set, 0, sizeof(struct iris_meta));
	spin_lock_init(&mfd->iris_conf.iris_reset_lock);
	iris_regs_clear();
	INIT_WORK(&mfd->iris_conf.pt_work, iris_pt_entry_wq_handler);
	INIT_WORK(&mfd->iris_conf.memc_work, iris_memc_entry_handler);
	INIT_WORK(&mfd->iris_conf.memc_prepare_work, iris_memc_prepare_handler);
	INIT_WORK(&mfd->iris_conf.memc_cancel_work, iris_memc_cancel_handler);
	mutex_init(&mfd->iris_conf.cmd_mutex);
	mutex_init(&mfd->iris_conf.config_mutex);
	mutex_init(&mfd->iris_conf.meta_mutex);
	/*allocate memory for firmware download*/
	g_firmware_buf = kzalloc(DSI_DMA_TX_BUF_SIZE, GFP_KERNEL);
	if (!g_firmware_buf)
		pr_err("%s: failed to alloc mem, size = %d\n", __func__, DSI_DMA_TX_BUF_SIZE);

	iris_debugfs_init(mfd);
}
