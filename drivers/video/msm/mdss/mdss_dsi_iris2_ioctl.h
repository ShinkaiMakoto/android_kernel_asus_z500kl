#ifndef MDSS_DSI_IRIS2_IOCTL_H
#define MDSS_DSI_IRIS2_IOCTL_H
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
int iris_set_drc_size(struct msm_fb_data_type *mfd, void __user *argp);
int iris_get_frc_timing(struct msm_fb_data_type *mfd, void __user *argp);
#endif

int msmfb_iris_tuning(struct msm_fb_data_type *mfd, void __user *argp);
int msmfb_iris_write_reg(struct msm_fb_data_type *mfd,
				void __user *argp);
int msmfb_iris_notify_scenario(struct msm_fb_data_type *mfd,
				int s);
int msmfb_iris_notify_work_mode(struct msm_fb_data_type *mfd,
				void __user *argp);
int msmfb_iris_frc_switch(struct msm_fb_data_type *mfd,
				void __user *argp);
int mdss_mipi_dsi_command(struct mdss_panel_data *pdata, void __user *argp);
int msmfb_iris_configure(struct msm_fb_data_type *mfd,
				void __user *argp);
int msmfb_iris_configure_ex(struct msm_fb_data_type *mfd,
				void __user *argp);

int msmfb_iris_configure_get(struct msm_fb_data_type *mfd,
				void __user *argp);
int iris_set_mode(struct msm_fb_data_type *mfd, void __user *argp);
int iris_get_mode(struct msm_fb_data_type *mfd, void __user *argp);
int iris_set_meta(struct msm_fb_data_type *mfd, void __user *argp);
int iris_frc_path_update(struct msm_fb_data_type *mfd, void __user *argp);
int iris_notify_video_frame_rate(struct msm_fb_data_type *mfd,
				void __user *argp);
int iris_set_rotation(struct msm_fb_data_type *mfd, void __user *argp);

u32 iris2_lp_memc_calc(u32 value);
#endif
