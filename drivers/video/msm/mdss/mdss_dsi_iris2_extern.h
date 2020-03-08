#ifndef MDSS_DSI_IRIS2_EXTERN_H
#define MDSS_DSI_IRIS2_EXTERN_H

extern int pw_iris2_status;
extern int entry_mode;
extern void iris2_io_bimc_enable(int on);
extern void iris2_io_snoc_enable(int on);

extern struct iris_mipi iris_mipi_info;
extern struct msm_fb_data_type *g_mfd;
extern struct mdss_dsi_ctrl_pdata *g_dsi_ctrl;
extern struct mdss_mdp_ctl *g_ctl0, *g_ctl1;
extern struct mdss_panel_data *g_pdata0, *g_pdata1;
extern u8 *g_firmware_buf;

/* debug send meta */
extern bool debug_send_meta_enabled;
/* debug new frame flag in video mode */
extern bool debug_new_frame_enabled;
/* debug in/out ratio flag */
extern bool debug_ratio_enabled;
extern bool debug_hlmd_enabled;
/* debug repeat flag */
extern u8 debug_repeat_enabled;
/* debug te flag */
extern bool debug_te_enabled;
/* debug dtg */
extern bool debug_dtg_enabled;
extern bool frc_repeat_enter;
extern int debug_new_repeat;
/* debug send mode switch */
extern bool debug_mode_switch_enabled;
extern bool debug_true_cut;

/* debug usb workaround */
#if defined(USER_BUILD)
extern bool debug_usb_w_enabled;
#else
extern bool debug_usb_w_enabled;
#endif
extern bool debug_clock_gate;

#if defined(CONFIG_IRIS2_DRC_SUPPORT)
/* drc scaling feature */
extern uint32_t guFrcLPTiming;
#endif

/* Activate Delay 0, FBO Enable: 0, Display Mode: Normal,
 * PSR Command: PSR Enable, Capture Enable: -
 */
extern int first_boot;
extern char pt_enable[2];
extern char memc_enable[2];
extern struct iris_pq_setting pq_setting_default;
extern struct iris_pq_setting pq_setting_current;
extern struct iris_dbc_setting dbc_setting_defalut;
extern struct iris_dbc_setting dbc_setting_current;
extern struct iris_config_setting LPMemc_setting_default;
extern struct iris_config_setting LPMemc_setting_current;
extern struct demo_win_info_for_FI demowinFI_setting_default;
extern struct demo_win_info demo_win_info_setting;
extern bool demo_win_FI_update;
extern bool pq_setting_update;
extern bool dbc_setting_update;
extern bool LPMemc_setting_update;
extern u8 iris_dbc_mode;
extern u8 color_adjust_default_value;
extern u8 color_adjust_current_value;
extern bool color_adjust_update;
extern bool black_border_value;
extern bool black_border_update;
extern u16 iris_LPMeMcTiming[];
extern char iris_pt_enter_cmds[];
extern struct dsi_cmd_desc pt_data_path_config[1];
extern struct dsi_cmd_desc pt_mode_enter[1];
extern struct dsi_cmd_desc memc_mode_enter[1];


#endif
