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
#ifndef _MDSS_DSI_IRIS2_LIGHTUP_ONLY_PRIV_H_
#define _MDSS_DSI_IRIS2_LIGHTUP_ONLY_PRIV_H_

#include "mdss_dsi.h"

#define MIPI_SWAP

#define IRIS_DTG_ADDR	0xF1200000
#define IRIS_PSR_MIF_ADDR	0xF1400000
#ifdef MIPI_SWAP
#define IRIS_MIPI_RX_ADDR	0xF0140000
#define IRIS_MIPI_TX_ADDR	0xF01c0000
#else
#define IRIS_MIPI_RX_ADDR	0xF0100000
#define IRIS_MIPI_TX_ADDR	0xF0180000
#endif
#define IRIS_SYS_ADDR		0xF0000000

// bypass iris when sent light-up-commands
#define IRIS_BYPASS
#define REF_PLL_19_2_MHZ    //19.2 Mhz
// demo board1 need to comment following efuse rewrite
#define EFUSE_REWRITE
// panel will be light up in MCU mode
#define POWER_MODE_SLEEP 0x10

// #define WAKEUP_TIME 50
#define CMD_PROC 2
// #define INIT_INT 100
#define INIT_WAIT 20

/* Per Panel different */
#define IRIS_DTG_EVS_DLY   124
#define IRIS_DTG_E2OVS_DLY 4

enum iris_mipirx_mode_enum {
	IRIS_MIPIRX_VIDEO = 0x0,
	IRIS_MIPIRX_CMD = 0x01,
};

struct  iris_timing_para {
	u16 hfp;
	u16 hres;
	u16 hbp;
	u16 hsw;
	u16 vfp;
	u16 vres;
	u16 vbp;
	u16 vsw;
};

struct  iris_dsc_para {
	u16 slice_number;
	u16 slice_height;
	u16 bpp;
};

struct iris_mipitx_config {
	u32 dpi_res;
	u32 hsync_count;
	u32 hbp_count;
	u32 hfp_count;
	u32 h_res;
	u32 vsync_count;
	u32 vbp_count;
	u32 vfp_count;
	u32 v_res;
};

struct mipi_mode_t {
	u32 rx_mode:1;    // 0-video/1-cmd
	u32 rx_ch:1;    // 0-single/1-dual
	u32 rx_dsc:1;    // 0-non DSC/1-DSC
	u32 bypass_en:1;   // 0-PT mode/ 1-bypass mode
	u32 rx_pxl_mode:1;
	u32 reversed0:11;

	u32 tx_mode:1;    // 0-video/1-cmd
	u32 tx_ch:1;    // 0-single/1-dual
	u32 tx_dsc:1;    // 0-non DSC/1-DSC
	u32 tx_pxl_mode:1;
	u32 te_120_to_60:1;	//half te frequency
	u32 reversed1:11;
};

// be used by iris2 & command mode
struct iris_mipi {
	struct dsi_panel_cmds panel_videomode_on_cmds[2];
	struct dsi_panel_cmds panel_videomode_off_cmds[2];
	struct dsi_panel_cmds mipirx_cmdmode_cmds;
	struct mipi_mode_t mipi_mode;
	struct iris_timing_para iris_in_timing;
	struct iris_timing_para iris_out_timing;
	struct iris_dsc_para iris_in_dsc;
	struct iris_dsc_para iris_out_dsc;
	u8 iris_timing_flag;
	int delta_period_max;
	int delta_period_min;
	bool panel_cmd_sync_wait_broadcast;
};

#endif // _MDSS_DSI_IRIS2_LIGHTUP_ONLY_PRIV_H_
