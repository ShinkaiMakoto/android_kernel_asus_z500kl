#ifndef MDSS_DSI_IRIS2_LIGHTUP_H
#define MDSS_DSI_IRIS2_LIGHTUT_H

#define IRIS_FIRMWARE_NAME	"iris2.fw"
#define REF_PLL_19_2_MHZ    //19.2 Mhz
struct  iris_mipi_param_calc {
	u8 trim1_divider_ratio;
	u16 ratio_panel_iris;
};

void mdss_dsi_parse_iris_params(struct device_node *np, struct mdss_panel_info *panel_info);
void mdss_dsi_parse_iris_mipi(struct device_node *np, char mode, u32 panel_destination);

void iris2_init(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightup(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightoff(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_lightoff_finish(struct mdss_dsi_ctrl_pdata *ctrl);
void iris2_panel_cmds(struct mdss_dsi_ctrl_pdata *ctrl, u8 cflag);

int iris_fw_download_cont_splash(struct mdss_panel_data *pdata, bool debug);
void iris_abypass_switch_proc(struct mdss_dsi_ctrl_pdata *ctrl);

#endif
