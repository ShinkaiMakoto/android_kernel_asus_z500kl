#include "mdss_dsi_iris2_def.h"

#define SCALE 0x1D0

struct iris_mipi iris_mipi_info;
struct msm_fb_data_type *g_mfd;
struct mdss_dsi_ctrl_pdata *g_dsi_ctrl;
struct mdss_mdp_ctl *g_ctl0, *g_ctl1;
struct mdss_panel_data *g_pdata0, *g_pdata1;
u8 *g_firmware_buf = NULL;


/* debug send meta */
bool debug_send_meta_enabled = 1;
/* debug new frame flag in video mode */
bool debug_new_frame_enabled = 1;
/* debug in/out ratio flag */
bool debug_ratio_enabled = 1;
bool debug_hlmd_enabled;
/* debug repeat flag */
u8 debug_repeat_enabled = 1;
/* debug te flag */
bool debug_te_enabled = 1;
/* debug dtg */
bool debug_dtg_enabled;
bool frc_repeat_enter;
int debug_new_repeat = 1;
/* debug send mode switch */
bool debug_mode_switch_enabled = 1;
bool debug_true_cut = true;

/* debug usb workaround */
#if defined(USER_BUILD)
bool debug_usb_w_enabled = 0;
#else
bool debug_usb_w_enabled = 1;
#endif
bool debug_clock_gate = 1;

#if defined(CONFIG_IRIS2_DRC_SUPPORT)
/* drc scaling feature */
uint32_t guFrcLPTiming = 0;
#endif

/* Activate Delay 0, FBO Enable: 0, Display Mode: Normal,
 * PSR Command: PSR Enable, Capture Enable: -
 */
int first_boot = 1;
char pt_enable[2] = { 0x00, 0x00 };
char memc_enable[2] = {0x04, 0x2};

struct iris_pq_setting pq_setting_current = {
	.peaking = 0,
	.memcDemo = 0,
	.peakingDemo = 0,
	.memcLevel = 3,
	.contrast = 50,
	.peakingUpdate = 1,
	.memcDemoUpdate = 1,
	.peakingDemoUpdate = 1,
	.memcLevelUpdate = 1,
	.contrastUpdate = 1,
	.cinema = 0,
};
struct iris_dbc_setting dbc_setting_current = {
	.dbcUpdate = 1,
	.DCELevel = 0,
	.dbcQuality = 5,
	.dlvSensitivity = 0,
};
struct iris_config_setting LPMemc_setting_current = {
	.level = 0,
	.value = 0x4b00384,
};
struct demo_win_info_for_FI demowinFI_setting_default = {0};
struct demo_win_info demo_win_info_setting = { 0 };
bool demo_win_FI_update;
bool pq_setting_update;
bool dbc_setting_update;
bool LPMemc_setting_update;

u8 iris_dbc_mode;
u8 color_adjust_current_value = 50;
bool color_adjust_update;

bool black_border_value;
bool black_border_update;

#ifdef ENABLE_IRIS2_480X800_PANEL
#define IRIS_DTG_HRES_SETTING 480
#define IRIS_DTG_VRES_SETTING 800
#else
#define IRIS_DTG_HRES_SETTING 768
#define IRIS_DTG_VRES_SETTING 2048
#endif
u16 iris_LPMeMcTiming[] = {0x0384, 0x04b0};

char iris_pt_enter_cmds[] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x00000005),	//valid word number
	0x00,					//burst mode
	0x00,					//reserved
	PWIL_U16(0x0004),		//burst length
	PWIL_U32(IRIS_PROXY_ADDR + 0x10),	//proxy MB2
	PWIL_U32(0x800000),
	PWIL_U32(IRIS_PROXY_ADDR + 0x08), //proxy MB1
	PWIL_U32(0x800000)
};

struct dsi_cmd_desc pt_data_path_config[1] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_pt_enter_cmds) }, iris_pt_enter_cmds},
};

struct dsi_cmd_desc pt_mode_enter[1] = {
	{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(pt_enable) }, pt_enable},
};

struct dsi_cmd_desc memc_mode_enter[1] = {
	{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(memc_enable) }, memc_enable},
};
