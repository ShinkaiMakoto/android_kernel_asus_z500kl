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
#ifndef _MDSS_DSI_IRIS2_LIGHTUP_ONLY_H_
#define _MDSS_DSI_IRIS2_LIGHTUP_ONLY_H_

#include "mdss_dsi.h"

// TODO: use it or not?
// #define DSI_BUF_SIZE	1024
// #define DSI_DMA_TX_BUF_SIZE	SZ_64K

struct iris_mipi_param_calc {
	u8 trim1_divider_ratio;
	u16 ratio_panel_iris;
};

enum iris_cmdmode_cmds {
	PANEL_ON_CMDS = 0x01,	// means panel_videomode_on_cmds
	PANEL_OFF_CMDS = 0x02,	// means panel_videomode_off_cmds
};

void mdss_dsi_parse_iris_params(struct device_node *np, struct mdss_panel_info *panel_info);
void mdss_dsi_parse_iris_mipi(struct device_node *np, char mode, u32 panel_destination);

void iris2_init(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightup(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightoff(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightoff_finish(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_panel_cmds(struct mdss_dsi_ctrl_pdata *ctrl, u8 cflag);

#endif // _MDSS_DSI_IRIS2_LIGHTUP_ONLY_H_
