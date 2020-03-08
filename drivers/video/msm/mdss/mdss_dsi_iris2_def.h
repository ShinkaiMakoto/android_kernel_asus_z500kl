#ifndef MDSS_DSI_IRIS2_DEF_H
#define MDSS_DSI_IRIS2_DEF_H
#include "mdss_mdp.h"
#include "mdss_dsi.h"
#include "linux/fb.h"
#include <linux/types.h>

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

struct iris_pq_setting {
	uint32_t peaking:4;
	uint32_t sharpness:4;
	uint32_t memcDemo:2;
	uint32_t peakingDemo:2;
	uint32_t gamma:2;
	uint32_t memcLevel:2;
	uint32_t contrast:8;
	uint32_t peakingUpdate:1;
	uint32_t sharpnessUpdate:1;
	uint32_t memcDemoUpdate:1;
	uint32_t peakingDemoUpdate:1;
	uint32_t gammeUpdate:1;
	uint32_t memcLevelUpdate:1;
	uint32_t contrastUpdate:1;
	uint32_t cinema:1;
};

struct iris_dbc_setting {
	uint32_t dbcUpdate:1;
	uint32_t reserved:3;
	uint32_t DCELevel:4;
	uint32_t brightness:7;
	uint32_t externalPWM:1;
	uint32_t dbcQuality:4;
	uint32_t dlvSensitivity:12;
};

struct iris_config_setting {
	int    update;
	u8	level;
	union {
		uint32_t value;
		struct iris_pq_setting pqSetting;
	};
};

// ---------------------------------------------------------------------------
//! Structure definition for demo window.
// ---------------------------------------------------------------------------
struct demo_win_info {
	int   startX;    //12bits width
	int   startY;    //12bits width
	int   endX;      //12bits width
	int   endY;     //12bits width
	int   color;      //Y U V 8bits width,      Y[7:0], U[15:8], V[23:16]
	int   BorderWidth;    ///3bits width
	int   MEMCEn;          //bool
	int   SharpnessEn;   //bool
};

// ---------------------------------------------------------------------------
//! Structure  definition for demo window FI setting.
// ---------------------------------------------------------------------------
struct demo_win_info_for_FI {
	int   colsize;
	int   color;
	int   rowsize;
	int   modectrl;
};

//if it use debug info should open DEBUG, or not DEBUG info
//#define DEBUG

//#define FPGA_DEBUG

#define IRIS_CHIP_HW_VER 0

//#define IRIS_RETRY 5
//#define RESEND_TIMES 5
// assume 1 entry 1 ms
#define IRIS_REPEAT_NO     0

/* Per Panel different */
#define IRIS_DTG_EVS_DLY   124
/* Assume the panel can accept this period increase */
//#define IRIS_DELTA_VT_P        (31)
/* VFP needs at least 2 lines VSW needs at least 1 line */
//#define IRIS_DELTA_VT_M        (-4)

#define IRIS_DTG_ADDR	0xF1200000
//#define IRIS_DPORT_ADDR	0xF1220000
#define IRIS_PWIL_ADDR	0xF1240000
#define IRIS_PSR_MIF_ADDR	0xF1400000
//#define IRIS_PSR_COMP_ADDR	0xFF420000
//#define IRIS_BLENDING_ADDR	0xF1520000
//#define IRIS_FRC_ADDR	0xFF800000
//#define IRIS_COMP_ADDR	0xFF880000
//#define IRIS_GMD_ADDR	0xFF8A0000
//#define IRIS_FBD_ADDR	0xFF8C0000
//#define IRIS_CAD_ADDR	0xFF8E0000
//#define IRIS_DECOMP0_ADDR	0xFF9A0000
//#define IRIS_DECOMP1_ADDR	0xFF9C0000
#define IRIS_PROXY_ADDR	0xF0040000
#define IRIS_SYS_ADDR		0xF0000000
//#define PWIL_STATUS_ADDR	0xF1240080


#define FRCC_CTRL_REG5_ADDR		0xF2010014
//#define FRCC_CTRL_REG7_ADDR     0xF201001C
#define FRCC_CTRL_REG8_ADDR		0xF2010020
#define FRCC_CTRL_REG16_ADDR	0xF2010040
#define FRCC_CTRL_REG17_ADDR	0xF2010044
#define FRCC_CTRL_REG18_ADDR	0xF2010048


#define FRCC_CMD_MOD_TH			0xF201004c
//#define FRCC_INPUT_HW_META0		0xF2010150
#define FRCC_REG_SHOW       0xF2011198

#define IRIS_PWIL_OUT_FRAME_SHIFT 24
#define IRIS_PWIL_IN_FRAME_SHIFT 8
#define IRIS_MVC_ADDR 0xF2100000
//#define IRIS_BLC_PWM_ADDR 0xF10C0000
#define FI_DEMO_COL_SIZE       0xf2160018
#define FI_DEMO_MODE_CTRL      0xf216001c
#define FI_DEMO_MODE_RING      0xf2160020
#define FI_DEMO_ROW_SIZE       0xf2160024
#define FI_SHADOW_UPDATE       0xf217ff00
#define PEAKING_CTRL           0xf1a0005c
#define PEAKING_STARTWIN       0xf1a00060
#define PEAKING_ENDWIN         0xf1a00064
#define PEAKING_SHADOW_UPDATE  0xf1a1ff00
//#define FRC_DSC_ENCODER0       0xf1620000

#define IRIS_PQ_SETTING_ADDR	(IRIS_PWIL_ADDR+0x10e4)	// PWIL_OSD_CTRL9

#define IMG720P_HSIZE    (1280)
#define IMG720P_VSIZE    (720)
#define IMG1080P_HSIZE   (1920)
#define IMG1080P_VSIZE   (1080)

#define MIPI_SWAP
#ifdef MIPI_SWAP
#define IRIS_MIPI_RX_ADDR       0xF0140000
#define IRIS_MIPI_TX_ADDR       0xF01c0000
#else
#define IRIS_MIPI_RX_ADDR       0xF0100000
#define IRIS_MIPI_TX_ADDR       0xF0180000
#endif

#define PWIL_TAG(a, b, c, d) d, c, b, a
#define PWIL_U32(x) \
	(__u8)(((x)) & 0xff), \
	(__u8)(((x) >>  8) & 0xff), \
	(__u8)(((x) >> 16) & 0xff), \
	(__u8)(((x) >> 24) & 0xff)

#define PWIL_U16(x) \
	(__u8)(((x)) & 0xff), \
	(__u8)(((x) >> 8) & 0xff)


enum iris_work_mode {
	IRIS_PT_MODE = 0,
	IRIS_PSR_MODE,
	IRIS_MEMC_MODE,
	IRIS_FBO_MODE,
};

enum iris_mipirx_mode_enum {
	IRIS_MIPIRX_VIDEO = 0x0,
	IRIS_MIPIRX_CMD = 0x01,
};


#endif
