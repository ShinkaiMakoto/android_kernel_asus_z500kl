#ifndef MDSS_DSI_IRIS2_MODE_SWITCH_H
#define MDSS_DSI_IRIS2_MODE_SWITCH_H

int iris_mode_switch_video(struct msm_fb_data_type *mfd);
int iris_mode_switch_cmd(struct msm_fb_data_type *mfd);
int iris_proc_frcc_setting(struct msm_fb_data_type *mfd);
void iris_mode_switch_reset(struct mdss_dsi_ctrl_pdata *ctrl);
#endif
