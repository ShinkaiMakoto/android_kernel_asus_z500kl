#include <linux/workqueue.h>
#include <linux/msm_mdp.h>
#include <linux/gpio.h>
#include <linux/circ_buf.h>
#include <linux/gcd.h>
#include <asm/uaccess.h>

#include "mdss_mdp.h"
#include "mdss_fb.h"
#include "mdss_dsi.h"

#include "mdss_dsi_iris2.h"
#include "mdss_dsi_iris2_def.h"
#include "mdss_dsi_iris2_extern.h"
#include "mdss_dsi_iris2_lightup.h"
#include "mdss_dsi_iris2_dbg.h"

static void iris_nfrv_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	//u32 off, mixercfg;

	pr_debug("#### %s:%d vtime=%lld\n", __func__, __LINE__, vtime.tv64);
	/*
	mixercfg = MDSS_MDP_LM_BORDER_COLOR;
	off = MDSS_MDP_REG_CTL_LAYER(0);
	mdss_mdp_ctl_write(ctl, off, mixercfg);
	*/
	ctl->force_screen_state = MDSS_SCREEN_FORCE_BLANK;
}

static struct mdss_mdp_vsync_handler nfrv_vsync_handler = {
	.vsync_handler = iris_nfrv_vsync_handler,
};

static int iris_fbo_enable(struct msm_fb_data_type *mfd, int enable)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;

	if (enable && !mfd->iris_fbo_enable) {
		mfd->iris_fbo_enable = true;
		ctl->ops.add_vsync_handler(ctl, &nfrv_vsync_handler);
		pr_err("%s:%d enable\n", __func__, __LINE__);
	} else if (!enable && mfd->iris_fbo_enable) {
		mfd->iris_fbo_enable = false;
		ctl->ops.remove_vsync_handler(ctl, &nfrv_vsync_handler);
		ctl->force_screen_state = MDSS_SCREEN_DEFAULT;
	}

	return 0;
}

static int iris_sbs_enable(struct msm_fb_data_type *mfd, int enable)
{
	if (enable && !mfd->iris_sbs_enable) {
		mfd->iris_sbs_enable = true;
		pr_err("%s:%d enable\n", __func__, __LINE__);
	} else if (!enable && mfd->iris_sbs_enable) {
		mfd->iris_sbs_enable = false;
		pr_err("%s:%d disable\n", __func__, __LINE__);
	}

	return 0;
}

static ssize_t iris_dbg_fw_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	ssize_t rc;
	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	rc = iris_fw_download_cont_splash(pdata, 1);
	if (rc)
		return rc;

	return count;
}

static const struct file_operations iris_dbg_fw_fops = {
	.open = simple_open,
	.write = iris_dbg_fw_write,
};

static ssize_t iris_dbg_fbo_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	switch (val) {
	case 0:
		pr_info("%s:%d native frame rate video disable\n", __func__, __LINE__);
		iris_fbo_enable(mfd, 0);
		break;
	case 1:
		pr_info("%s:%d native frame rate video enable\n", __func__, __LINE__);
		iris_fbo_enable(mfd, 1);
		break;
	default:
		pr_err("%s:%d invalid input\n", __func__, __LINE__);
		break;
	}

	return count;
}

static const struct file_operations iris_dbg_fbo_fops = {
	.open = simple_open,
	.write = iris_dbg_fbo_write,
};

static ssize_t iris_dbg_sbs_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	pr_info("%s:%d sbs_enable %li\n", __func__, __LINE__, val);
	switch (val) {
	case 0:
		iris_sbs_enable(mfd, 0);
		break;
	case 1:
		iris_sbs_enable(mfd, 1);
		break;
	default:
		pr_err("%s:%d invalid input\n", __func__, __LINE__);
		break;
	}

	return count;
}

static const struct file_operations iris_dbg_sbs_fops = {
	.open = simple_open,
	.write = iris_dbg_sbs_write,
};

static void debug_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	// NOP
}

static struct mdss_mdp_vsync_handler iris_debug_vsync_handler = {
	.vsync_handler = debug_vsync_handler,
};

static ssize_t iris_dbg_vsync_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	unsigned long val;
	static bool debug_vsync_enabled;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	pr_info("%s:%d vsync_enable %li\n", __func__, __LINE__, val);
	if (val && !debug_vsync_enabled) {
		ctl->ops.add_vsync_handler(ctl, &iris_debug_vsync_handler);
		debug_vsync_enabled = true;
	} else if (!val && debug_vsync_enabled) {
		ctl->ops.remove_vsync_handler(ctl, &iris_debug_vsync_handler);
		debug_vsync_enabled = false;
	}
	return count;
}

static const struct file_operations iris_dbg_vsync_fops = {
	.open = simple_open,
	.write = iris_dbg_vsync_write,
};

static ssize_t iris_dbg_meta_enable_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;
	uint32_t r;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	pr_info("%s:%d meta_enabled %u in/out %u/%u\n", __func__, __LINE__, (u32)val, iris_cfg->input_frame_rate, iris_cfg->output_frame_rate);
	debug_send_meta_enabled = val;

	r = gcd(mfd->iris_conf.input_frame_rate, mfd->iris_conf.output_frame_rate);
	mfd->iris_conf.in_ratio = mfd->iris_conf.input_frame_rate / r;
	mfd->iris_conf.out_ratio = mfd->iris_conf.output_frame_rate / r;

	iris_register_write(mfd, IRIS_PWIL_ADDR + 0x0638,
		(iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT));
	iris_register_write(mfd, IRIS_PWIL_ADDR + 0x12FC,
		(iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT));
	return count;
}

static ssize_t iris_dbg_meta_enable_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_send_meta_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_meta_fops = {
	.open = simple_open,
	.write = iris_dbg_meta_enable_write,
	.read = iris_dbg_meta_enable_read,
};

static ssize_t iris_dbg_ratio_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_ratio_enabled = val;

	pr_info("ratio_enabled %u in/out %u/%u ratio %u/%u\n", (u32)val, iris_cfg->input_frame_rate, iris_cfg->output_frame_rate,
		iris_cfg->in_ratio, iris_cfg->out_ratio);

	return count;
}

static ssize_t iris_dbg_ratio_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "ratio_enabled %u in/out %u/%u ratio %u/%u\n", debug_ratio_enabled,
			iris_cfg->input_frame_rate, iris_cfg->output_frame_rate, iris_cfg->in_ratio, iris_cfg->out_ratio);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_ratio_fops = {
	.open = simple_open,
	.write = iris_dbg_ratio_write,
	.read = iris_dbg_ratio_read,
};

static ssize_t iris_dbg_mode_switch_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_mode_switch_enabled = val;

	pr_debug("debug_mode_switch_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_mode_switch_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "debug_mode_switch_enabled %u\n", debug_mode_switch_enabled);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_mode_switch_fops = {
	.open = simple_open,
	.write = iris_dbg_mode_switch_write,
	.read = iris_dbg_mode_switch_read,
};

static ssize_t iris_dbg_repeat_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_repeat_enabled = val;

	pr_info("repeat_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_repeat_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_repeat_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_repeat_fops = {
	.open = simple_open,
	.write = iris_dbg_repeat_write,
	.read = iris_dbg_repeat_read,
};

static ssize_t iris_dbg_te_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_te_enabled = val;

	pr_info("te_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_te_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_te_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_te_fops = {
	.open = simple_open,
	.write = iris_dbg_te_write,
	.read = iris_dbg_te_read,
};

static ssize_t iris_dbg_dtg_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_dtg_enabled = val;

	pr_info("dtg_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_dtg_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_dtg_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_dtg_fops = {
	.open = simple_open,
	.write = iris_dbg_dtg_write,
	.read = iris_dbg_dtg_read,
};

static ssize_t iris_dbg_new_repeat_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_new_repeat = val;

	pr_info("debug_new_repeat %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_new_repeat_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_new_repeat);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_new_repeat_fops = {
	.open = simple_open,
	.write = iris_dbg_new_repeat_write,
	.read = iris_dbg_new_repeat_read,
};


static ssize_t iris_dbg_true_cut_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_true_cut = val;

	pr_info("debug_true_cut %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_true_cut_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_true_cut);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_true_cut_fops = {
	.open = simple_open,
	.write = iris_dbg_true_cut_write,
	.read = iris_dbg_true_cut_read,
};

static ssize_t iris_dbg_new_frame_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_new_frame_enabled = val;

	pr_info("repeat_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_new_frame_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_new_frame_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_new_frame_fops = {
	.open = simple_open,
	.write = iris_dbg_new_frame_write,
	.read = iris_dbg_new_frame_read,
};

static ssize_t iris_dbg_frc_path_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	g_mfd->iris_conf.frc_path = val;

	return count;
}

static ssize_t iris_dbg_frc_path_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%x\n", g_mfd->iris_conf.frc_path);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_frc_path_fops = {
	.open = simple_open,
	.write = iris_dbg_frc_path_write,
	.read = iris_dbg_frc_path_read,
};

static ssize_t iris_dbg_input_vfr_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	iris_cfg->input_vfr = val;

	pr_info("input_vfr = %d\n", iris_cfg->input_vfr);

	return count;
}

static ssize_t iris_dbg_input_vfr_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "input_vfr = %d\n", iris_cfg->input_vfr);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_input_vfr_fops = {
	.open = simple_open,
	.write = iris_dbg_input_vfr_write,
	.read = iris_dbg_input_vfr_read,
};

static ssize_t iris_dbg_hlmd_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	//struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_hlmd_enabled = val;

	pr_info("hlmd_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_hlmd_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	//struct iris_config *iris_cfg = &g_mfd->iris_conf;
	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_hlmd_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_hlmd_fops = {
	.open = simple_open,
	.write = iris_dbg_hlmd_write,
	.read = iris_dbg_hlmd_read,
};

static ssize_t iris_dbg_usb_w_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_usb_w_enabled = val;

	pr_info("usb_w_enabled %u\n", (u32)val);

	return count;
};

static ssize_t iris_dbg_usb_w_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_usb_w_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
};

static const struct file_operations iris_dbg_usb_w_fops = {
	.open = simple_open,
	.write = iris_dbg_usb_w_write,
	.read = iris_dbg_usb_w_read,
};

static ssize_t iris_dbg_clock_gate_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_clock_gate = val;

	pr_info("debug_clock_gate %u\n", (u32)val);

	return count;
};

static ssize_t iris_dbg_clock_gate_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_clock_gate);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
};

static const struct file_operations iris_dbg_clock_gate_fops = {
	.open = simple_open,
	.write = iris_dbg_clock_gate_write,
	.read = iris_dbg_clock_gate_read,
};

int iris_debugfs_init(struct msm_fb_data_type *mfd)
{
	if (debugfs_create_file("iris_fw", 0644, NULL, mfd, &iris_dbg_fw_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_fbo", 0644, NULL, mfd, &iris_dbg_fbo_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_sbs", 0644, NULL, mfd, &iris_dbg_sbs_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_vsync_debug", 0644, NULL, mfd,
				&iris_dbg_vsync_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_ratio", 0644, NULL, mfd,
				&iris_dbg_ratio_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_mode_switch", 0644, NULL, mfd,
				&iris_dbg_mode_switch_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_repeat", 0644, NULL, mfd,
				&iris_dbg_repeat_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_dtg", 0644, NULL, mfd,
				&iris_dbg_dtg_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_te", 0644, NULL, mfd,
				&iris_dbg_te_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_new_frame", 0644, NULL, mfd,
				&iris_dbg_new_frame_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_new_repeat", 0644, NULL, mfd,
				&iris_dbg_new_repeat_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_true_cut", 0644, NULL, mfd,
				&iris_dbg_true_cut_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_send_meta", 0644, NULL, mfd,
				&iris_dbg_meta_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_frc_path", 0644, NULL, mfd,
				&iris_dbg_frc_path_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_input_vfr", 0644, NULL, mfd,
				&iris_dbg_input_vfr_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_hlmd", 0644, NULL, mfd,
				&iris_dbg_hlmd_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_usb_w", 0644, NULL, mfd,
				&iris_dbg_usb_w_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_clock_gate", 0644, NULL, mfd,
				&iris_dbg_clock_gate_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	return 0;
}
