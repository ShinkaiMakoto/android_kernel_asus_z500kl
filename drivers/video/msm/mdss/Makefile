ccflags-y += -I$(src)

obj-$(CONFIG_FB_MSM_MDSS_MHL3) += mhl3/

mdss-mdp3-objs = mdp3.o mdp3_dma.o mdp3_ctrl.o dsi_status_v2.o
mdss-mdp3-objs += mdp3_ppp.o mdp3_ppp_hwio.o mdp3_ppp_data.o
obj-$(CONFIG_FB_MSM_MDSS_MDP3) += mdss-mdp3.o

mdss-mdp-objs := mdss_mdp.o mdss_mdp_ctl.o mdss_mdp_pipe.o mdss_mdp_util.o dsi_status_6g.o
mdss-mdp-objs += mdss_mdp_pp.o mdss_mdp_pp_debug.o
mdss-mdp-objs += mdss_mdp_intf_video.o
mdss-mdp-objs += mdss_mdp_intf_hr_video.o
mdss-mdp-objs += mdss_mdp_intf_cmd.o
mdss-mdp-objs += mdss_mdp_intf_writeback.o
mdss-mdp-objs += mdss_mdp_rotator.o
mdss-mdp-objs += mdss_mdp_overlay.o
mdss-mdp-objs += mdss_mdp_splash_logo.o
mdss-mdp-objs += mdss_mdp_wb.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss-mdp.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_mdp_debug.o

ifeq ($(CONFIG_FB_MSM_MDSS),y)
obj-$(CONFIG_DEBUG_FS) += mdss_debug.o mdss_debug_xlog.o
endif

ifneq ($(CONFIG_ARCH_MSM8909),y)
dsi-v2-objs = dsi_v2.o dsi_host_v2.o dsi_io_v2.o
obj-$(CONFIG_FB_MSM_MDSS_MDP3) += dsi-v2.o
endif

mdss-dsi-objs := mdss_dsi.o mdss_dsi_host.o mdss_dsi_cmd.o mdss_dsi_status.o
mdss-dsi-objs += mdss_dsi_panel.o
mdss-dsi-objs += msm_mdss_io_8974.o

ifeq ($(CONFIG_IRIS2_LIGHTUP_ONLY),y)
mdss-dsi-objs += mdss_dsi_iris2_lightup_only.o
endif
ifeq ($(CONFIG_IRIS2_FULL_SUPPORT),y)
mdss-dsi-objs += mdss_dsi_iris2_def.o mdss_dsi_iris2.o mdss_dsi_iris2_lightup.o
mdss-dsi-objs += mdss_dsi_iris2_ioctl.o mdss_dsi_iris2_mode_switch.o mdss_dsi_iris2_dbg.o
endif
ifeq ($(CONFIG_IRIS2P_FULL_SUPPORT),y)
mdss-dsi-objs += mdss_dsi_iris2p_def.o mdss_dsi_iris2p.o mdss_dsi_iris2p_lightup.o
mdss-dsi-objs += mdss_dsi_iris2p_ioctl.o mdss_dsi_iris2p_mode_switch.o mdss_dsi_iris2p_dbg.o
mdss-dsi-objs += mdss_dsi_iris2_lightup_only.o
mdss-dsi-objs += mdss_i2c_iris.o
endif
ifeq ($(CONFIG_IRIS2P_LIGHTUP_ONLY),y)
mdss-dsi-objs += mdss_dsi_iris2p_def.o mdss_dsi_iris2p_lightup_only.o
endif

mdss-dsi-objs += mdss_dsi_phy.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss-dsi.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_panel.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_hdmi_util.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_hdmi_edid.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_cec_abstract.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_dba_utils.o
obj-$(CONFIG_FB_MSM_MDSS_EDP_PANEL) += mdss_edp.o
obj-$(CONFIG_FB_MSM_MDSS_EDP_PANEL) += mdss_edp_aux.o

obj-$(CONFIG_FB_MSM_MDSS) += mdss_io_util.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_tx.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_hdcp.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_cec.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8334) += mhl_sii8334.o mhl_msc.o

obj-$(CONFIG_FB_MSM_MDSS_WRITEBACK) += mdss_wb.o

mdss-qpic-objs := mdss_qpic.o mdss_fb.o mdss_qpic_panel.o
obj-$(CONFIG_FB_MSM_QPIC) += mdss-qpic.o
obj-$(CONFIG_FB_MSM_QPIC_ILI_QVGA_PANEL) += qpic_panel_ili_qvga.o

obj-$(CONFIG_FB_MSM_MDSS) += mdss_fb.o mdss_util.o
obj-$(CONFIG_COMPAT) += mdss_compat_utils.o

ifeq ($(TARGET_PROJECT), ZT581KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
KBUILD_CFLAGS += -DTARGET_PROJECT_ZT581KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DUSER_BUILD
endif
endif

ifeq ($(TARGET_PROJECT), ZT582KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
KBUILD_CFLAGS += -DTARGET_PROJECT_ZT582KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DUSER_BUILD
endif
endif

ifeq ($(TARGET_PROJECT), ZT500KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
obj-$(CONFIG_BACKLIGHT_LP8557E01) += lp8557e01.o
KBUILD_CFLAGS += -DTARGET_PROJECT_ZT500KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DUSER_BUILD
endif
endif

ifeq ($(TARGET_PROJECT), Z581KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
KBUILD_CFLAGS += -DTARGET_PROJECT_Z581KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DUSER_BUILD
endif
endif

ifeq ($(TARGET_PROJECT), Z500KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
obj-$(CONFIG_BACKLIGHT_LP8557E01) += lp8557e01.o
KBUILD_CFLAGS += -DTARGET_PROJECT_Z500KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DUSER_BUILD
endif
endif

ifeq ($(TARGET_PROJECT), Z500KL)
obj-$(CONFIG_IRIS2_IO) += iris2_io.o
obj-$(CONFIG_BACKLIGHT_LP8557E01) += lp8557e01.o
KBUILD_CFLAGS += -DTARGET_PROJECT_Z500KL_DISPLAY
ifeq ($(TARGET_BUILD_VARIANT),user)
	KBUILD_CFLAGS += -DUSER_BUILD
endif
endif
