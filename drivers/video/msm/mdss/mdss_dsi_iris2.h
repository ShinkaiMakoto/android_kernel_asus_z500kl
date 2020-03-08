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
#ifndef MDSS_DSI_IRIS_H
#define MDSS_DSI_IRIS_H

#include "mdss_mdp.h"
#include "mdss_dsi.h"
#include "linux/fb.h"
#include <linux/types.h>

// #define IRIS_LOGO_DISPLAY
#ifdef IRIS_LOGO_DISPLAY
#define IRIS_BITMAP_NAME    "iris2.logo"
#endif

#define DSI_DMA_TX_BUF_SIZE	SZ_64K

//#define DSI_BUF_SIZE	1024
// demo board1 need to comment following efuse rewrite
#define EFUSE_REWRITE
// panel will be light up in MCU mode
#define POWER_MODE_SLEEP 0x10
//#define POWER_MODE_DISPLAY_NORMAL 0x8
#define POWER_MODE_DISPLAY_ON 0xC
//#define POWER_MODE_ZERO 0

#if 0
#define DGRESULT_IRIS_PANEL_ON 0x2
#define DGRESULT_FW_DOWNLOAD_BEGIN 0x4
#define DGRESULT_FW_DOWNLOAD_FINISH 0x5
#define DGRESULT_FW_DOWNLOAD_MIFUNIDLE 0x6
#define DGRESULT_FW_REMAP_SUCCESS 0xA
#define DGRESULT_IRIS_INFO 0xE
#endif



#define IRIS_BYPASS


#if 0
//PWIL View Descriptor Valid Word Number
#define PWIL_ViewD_LEN 0x0A
//PWIL Display Descriptor Valid Word Number
#define PWIL_DispD_LEN 0x05

// bypass iris when sent light-up-commands

#define PWIL_CHECK_FORMAT(cmds)	\
	do {	\
		int valid_word_num = (ARRAY_SIZE(cmds) - 12) / 4; \
		int non_burst_len = valid_word_num - 1; \
		if (!strncmp(cmds, "LIWP", 4)) { \
			if (!strncmp(cmds + 4, "PCRG", 4)) { \
				cmds[8] = valid_word_num & 0xFF; \
				cmds[9] = (valid_word_num >> 8) & 0xFF; \
				cmds[10] = (valid_word_num >> 16) & 0xFF; \
				cmds[11] = (valid_word_num >> 24) & 0xFF; \
				cmds[14] = non_burst_len & 0xFF; \
				cmds[15] = (non_burst_len >> 8) & 0xFF; \
			} else if (!strncmp(cmds + 4, "WEIV", 4)) { \
				cmds[8] = PWIL_ViewD_LEN & 0xFF; \
				cmds[9] = (PWIL_ViewD_LEN >> 8) & 0xFF; \
				cmds[10] = (PWIL_ViewD_LEN >> 16) & 0xFF; \
				cmds[11] = (PWIL_ViewD_LEN >> 24) & 0xFF; \
			} else if (!strncmp(cmds + 4, "PSID", 4)) { \
				cmds[8] = PWIL_DispD_LEN & 0xFF; \
				cmds[9] = (PWIL_DispD_LEN >> 8) & 0xFF; \
				cmds[10] = (PWIL_DispD_LEN >> 16) & 0xFF; \
				cmds[11] = (PWIL_DispD_LEN >> 24) & 0xFF; \
			} else { \
				\
			} \
		} else { \
			pr_err("PWIL Packet format error!\n"); \
		} \
	} while (0)
#endif


enum iris_cmdmode_cmds {
	PANEL_ON_CMDS = 0x01,	// means panel_videomode_on_cmds
	PANEL_OFF_CMDS = 0x02,	// means panel_videomode_off_cmds
	MIPIRX_CMDMODE_CMDS = 0x03,	// means mipirx_cmdmode_cmds
};




#if 0
enum iris_memc_option {
	IRIS_MEMC_PANEL_RESOLUTION,
	IRIS_MEMC_NATIVE_RESOLUTION,
	IRIS_MEMC_LOW_POWER,
};
#endif

int auo_panel_bl_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 value);
void mdss_dsi_iris_init(struct msm_fb_data_type *mfd);
//int iris_frc_enable(struct mdss_mdp_ctl *ctl, int enable);
//void iris_frc_new_frame(struct mdss_mdp_ctl *ctl);




void iris_firmware_download(struct mdss_dsi_ctrl_pdata *ctrl, const char *name);
void iris_initinfo_to_mcu(struct mdss_dsi_ctrl_pdata *ctrl, const char *name, u8 power_mode, int booted);


void mdss_dsi_panel_cmds_send_ex(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds);

int iris_register_write(struct msm_fb_data_type *mfd,	u32 addr, u32 value);
bool iris_frc_repeat(struct msm_fb_data_type *mfd);
int iris_set_configure(struct msm_fb_data_type *mfd);
int iris_calc_meta(struct msm_fb_data_type *mfd);
void iris_copy_meta(struct msm_fb_data_type *mfd);

void iris_send_meta_video(struct mdss_mdp_ctl *ctl);
void iris_send_meta_cmd(struct mdss_mdp_ctl *ctl);
void iris_cmd_cadence_check(struct mdss_mdp_ctl *ctl);
void iris_calc_nrv(struct mdss_mdp_ctl *ctl);
void mdss_dsi_iris_init_ctl(struct mdss_mdp_ctl *ctl);
void mdss_dsi_iris_init_pdata(struct mdss_panel_data *pdata);

void iris_regs_clear(void);
void iris_reg_add(u32 addr, u32 val);
u32 iris_pi_read(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr);
u32 iris_pi_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr, u32 value);
int iris_proc_constant_ratio(struct iris_config *iris_cfg);
void iris_update_configure(void);


#endif //MDSS_DSI_IRIS_H
