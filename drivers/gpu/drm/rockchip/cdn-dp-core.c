/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/extcon.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>

#include <sound/hdmi-codec.h>

#include "cdn-dp-core.h"
#include "cdn-dp-reg.h"
#include "rockchip_drm_vop.h"

#define connector_to_dp(c) \
		container_of(c, struct cdn_dp_device, connector)

#define encoder_to_dp(c) \
		container_of(c, struct cdn_dp_device, encoder)

#define GRF_SOC_CON9		0x6224
#define DP_SEL_VOP_LIT		BIT(12)
#define GRF_SOC_CON26		0x6268
#define UPHY_SEL_BIT		3
#define UPHY_SEL_MASK		BIT(19)
#define DPTX_HPD_SEL		(3 << 12)
#define DPTX_HPD_DEL		(2 << 12)
#define DPTX_HPD_SEL_MASK	(3 << 28)

#define MAX_FW_WAIT_SECS	64
#define CDN_DP_FIRMWARE		"rockchip/dptx.bin"

struct cdn_dp_data {
	u8 max_phy;
};

struct cdn_dp_data rk3399_cdn_dp = {
	.max_phy = 2,
};

static const struct of_device_id cdn_dp_dt_ids[] = {
	{ .compatible = "rockchip,rk3399-cdn-dp",
		.data = (void *)&rk3399_cdn_dp },
	{}
};

MODULE_DEVICE_TABLE(of, cdn_dp_dt_ids);

static int cdn_dp_grf_write(struct cdn_dp_device *dp,
			    unsigned int reg, unsigned int val)
{
	int ret;

	ret = clk_prepare_enable(dp->grf_clk);
	if (ret) {
		dev_err(dp->dev, "Failed to prepare_enable grf clock\n");
		return ret;
	}

	ret = regmap_write(dp->grf, reg, val);
	if (ret) {
		dev_err(dp->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	clk_disable_unprepare(dp->grf_clk);

	return 0;
}

static int cdn_dp_clk_enable(struct cdn_dp_device *dp)
{
	int ret;
	u32 rate;

	ret = clk_prepare_enable(dp->pclk);
	if (ret < 0) {
		dev_err(dp->dev, "cannot enable dp pclk %d\n", ret);
		goto err_pclk;
	}

	ret = clk_prepare_enable(dp->core_clk);
	if (ret < 0) {
		dev_err(dp->dev, "cannot enable core_clk %d\n", ret);
		goto err_core_clk;
	}

	ret = pm_runtime_get_sync(dp->dev);
	if (ret < 0) {
		dev_err(dp->dev, "cannot get pm runtime %d\n", ret);
		goto err_pclk;
	}

	reset_control_assert(dp->dptx_rst);
	reset_control_assert(dp->apb_rst);
	reset_control_deassert(dp->dptx_rst);
	reset_control_deassert(dp->apb_rst);

	rate = clk_get_rate(dp->core_clk);
	if (rate < 0) {
		dev_err(dp->dev, "get clk rate failed: %d\n", rate);
		goto err_set_rate;
	}

	cdn_dp_set_fw_clk(dp, rate);
	cdn_dp_clock_reset(dp);

	return 0;

err_set_rate:
	clk_disable_unprepare(dp->core_clk);
err_core_clk:
	clk_disable_unprepare(dp->pclk);
err_pclk:
	return ret;
}

static void cdn_dp_clk_disable(struct cdn_dp_device *dp)
{
	pm_runtime_put_sync(dp->dev);
	clk_disable_unprepare(dp->pclk);
	clk_disable_unprepare(dp->core_clk);
}

static enum drm_connector_status
cdn_dp_connector_detect(struct drm_connector *connector, bool force)
{
	struct cdn_dp_device *dp = connector_to_dp(connector);

	return dp->hpd_status;
}

static void cdn_dp_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs cdn_dp_atomic_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = cdn_dp_connector_detect,
	.destroy = cdn_dp_connector_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int cdn_dp_connector_get_modes(struct drm_connector *connector)
{
	struct cdn_dp_device *dp = connector_to_dp(connector);
	struct edid *edid;
	int ret = 0;

	edid = drm_do_get_edid(connector, cdn_dp_get_edid_block, dp);
	if (edid) {
		dev_dbg(dp->dev, "got edid: width[%d] x height[%d]\n",
			edid->width_cm, edid->height_cm);

		dp->sink_has_audio = drm_detect_monitor_audio(edid);
		ret = drm_add_edid_modes(connector, edid);
		if (ret) {
			drm_mode_connector_update_edid_property(connector,
								edid);
			drm_edid_to_eld(connector, edid);
		}
		kfree(edid);
	}

	return ret;
}

static struct drm_encoder *
cdn_dp_connector_best_encoder(struct drm_connector *connector)
{
	struct cdn_dp_device *dp = connector_to_dp(connector);

	return &dp->encoder;
}

static int cdn_dp_connector_mode_valid(struct drm_connector *connector,
				       struct drm_display_mode *mode)
{
	struct cdn_dp_device *dp = connector_to_dp(connector);
	struct drm_display_info *display_info = &dp->connector.display_info;
	u32 requested, actual, rate, sink_max, source_max = 0;
	u8 lanes, bpc, i;

	switch (display_info->bpc) {
	case 16:
	case 12:
	case 10:
		bpc = 10;
		break;
	case 6:
		bpc = 6;
		break;
	default:
		bpc = 8;
		break;
	}

	requested = mode->clock * bpc * 3 / 1000;

	/* find the running port */
	for (i = 0; i < dp->ports; i++) {
		if (dp->port[i]->phy_enabled) {
			source_max = dp->port[i]->cap_lanes;
			break;
		}
	}

	sink_max = drm_dp_max_lane_count(dp->dpcd);
	lanes = min(source_max, sink_max);

	source_max = drm_dp_bw_code_to_link_rate(CDN_DP_MAX_LINK_RATE);
	sink_max = drm_dp_max_link_rate(dp->dpcd);
	rate = min(source_max, sink_max);

	actual = rate * lanes / 100;

	/* efficiency is about 0.8 */
	actual = actual * 8 / 10;

	if (requested > actual) {
		dev_dbg(dp->dev, "requested=%d, actual=%d, clock=%d\n",
			requested, actual, mode->clock);
		return MODE_CLOCK_HIGH;
	}

	return MODE_OK;
}

static struct drm_connector_helper_funcs cdn_dp_connector_helper_funcs = {
	.get_modes = cdn_dp_connector_get_modes,
	.best_encoder = cdn_dp_connector_best_encoder,
	.mode_valid = cdn_dp_connector_mode_valid,
};

static void cdn_dp_commit(struct drm_encoder *encoder)
{
	struct cdn_dp_device *dp = encoder_to_dp(encoder);
	int ret;

	ret = cdn_dp_training_start(dp);
	if (ret)
		return;

	ret = cdn_dp_get_training_status(dp);
	if (ret)
		return;

	dev_info(dp->dev, "rate:0x%x, lanes:%d\n",
		 dp->link.rate, dp->link.num_lanes);

	if (cdn_dp_set_video_status(dp, CONTROL_VIDEO_IDLE))
		return;

	if (cdn_dp_config_video(dp))
		return;

	if (cdn_dp_set_video_status(dp, CONTROL_VIDEO_VALID))
		return;

	dp->dpms_mode = DRM_MODE_DPMS_ON;
}

static void cdn_dp_encoder_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted)
{
	struct cdn_dp_device *dp = encoder_to_dp(encoder);
	struct drm_display_info *display_info = &dp->connector.display_info;
	struct rockchip_crtc_state *state;
	struct video_info *video = &dp->video_info;
	int ret, val;

	switch (display_info->bpc) {
	case 16:
	case 12:
	case 10:
		video->color_depth = 10;
		break;
	case 6:
		video->color_depth = 6;
		break;
	default:
		video->color_depth = 8;
		break;
	}

	video->color_fmt = PXL_RGB;

	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	ret = drm_of_encoder_active_endpoint_id(dp->dev->of_node, encoder);
	if (ret < 0) {
		dev_err(dp->dev, "Could not get vop id, %d", ret);
		return;
	}

	dev_dbg(dp->dev, "vop %s output to cdn-dp\n", (ret) ? "LIT" : "BIG");
	state = to_rockchip_crtc_state(encoder->crtc->state);
	if (ret) {
		val = DP_SEL_VOP_LIT | (DP_SEL_VOP_LIT << 16);
		state->output_mode = ROCKCHIP_OUT_MODE_P888;
	} else {
		val = DP_SEL_VOP_LIT << 16;
		state->output_mode = ROCKCHIP_OUT_MODE_AAAA;
	}

	ret = cdn_dp_grf_write(dp, GRF_SOC_CON9, val);
	if (ret)
		return;

	memcpy(&dp->mode, adjusted, sizeof(*mode));
}

static void cdn_dp_encoder_enable(struct drm_encoder *encoder)
{
	struct cdn_dp_device *dp = encoder_to_dp(encoder);

	if (dp->dpms_mode != DRM_MODE_DPMS_ON)
		cdn_dp_commit(encoder);
}

static void cdn_dp_encoder_disable(struct drm_encoder *encoder)
{
	struct cdn_dp_device *dp = encoder_to_dp(encoder);

	dp->dpms_mode = DRM_MODE_DPMS_OFF;
}

static int cdn_dp_encoder_atomic_check(struct drm_encoder *encoder,
				       struct drm_crtc_state *crtc_state,
				       struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);

	s->output_mode = ROCKCHIP_OUT_MODE_AAAA;
	s->output_type = DRM_MODE_CONNECTOR_DisplayPort;

	return 0;
}

static struct drm_encoder_helper_funcs cdn_dp_encoder_helper_funcs = {
	.mode_set = cdn_dp_encoder_mode_set,
	.enable = cdn_dp_encoder_enable,
	.disable = cdn_dp_encoder_disable,
	.atomic_check = cdn_dp_encoder_atomic_check,
};

static struct drm_encoder_funcs cdn_dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int cdn_dp_firmware_init(struct cdn_dp_device *dp)
{
	int ret;
	const u32 *iram_data, *dram_data;
	const struct firmware *fw = dp->fw;
	const struct cdn_firmware_header *hdr;

	hdr = (struct cdn_firmware_header *)fw->data;
	if (fw->size != le32_to_cpu(hdr->size_bytes)) {
		dev_err(dp->dev, "firmware is invalid\n");
		return -EINVAL;
	}

	iram_data = (const u32 *)(fw->data + hdr->header_size);
	dram_data = (const u32 *)(fw->data + hdr->header_size + hdr->iram_size);

	ret = cdn_dp_load_firmware(dp, iram_data, hdr->iram_size,
				   dram_data, hdr->dram_size);
	if (ret)
		return ret;

	ret = cdn_dp_set_firmware_active(dp, true);
	if (ret) {
		dev_err(dp->dev, "active ucpu failed: %d\n", ret);
		return ret;
	}

	return cdn_dp_event_config(dp);
}

static int cdn_dp_parse_dt(struct cdn_dp_device *dp)
{
	struct device *dev = dp->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;

	dp->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(dp->grf)) {
		dev_err(dev, "cdn-dp needs rockchip,grf property\n");
		return PTR_ERR(dp->grf);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dp->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(dp->regs)) {
		dev_err(dev, "ioremap reg failed\n");
		return PTR_ERR(dp->regs);
	}

	dp->core_clk = devm_clk_get(dev, "core-clk");
	if (IS_ERR(dp->core_clk)) {
		dev_err(dev, "cannot get core_clk_dp\n");
		return PTR_ERR(dp->core_clk);
	}

	dp->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dp->pclk)) {
		dev_err(dev, "cannot get pclk\n");
		return PTR_ERR(dp->pclk);
	}

	dp->spdif_clk = devm_clk_get(dev, "spdif");
	if (IS_ERR(dp->spdif_clk)) {
		dev_err(dev, "cannot get spdif_clk\n");
		return PTR_ERR(dp->spdif_clk);
	}

	dp->grf_clk = devm_clk_get(dev, "grf");
	if (IS_ERR(dp->grf_clk)) {
		dev_err(dev, "cannot get grf clk\n");
		return PTR_ERR(dp->grf_clk);
	}

	dp->spdif_rst = devm_reset_control_get(dev, "spdif");
	if (IS_ERR(dp->spdif_rst)) {
		dev_err(dev, "no spdif reset control found\n");
		return PTR_ERR(dp->spdif_rst);
	}

	dp->dptx_rst = devm_reset_control_get(dev, "dptx");
	if (IS_ERR(dp->dptx_rst)) {
		dev_err(dev, "no uphy reset control found\n");
		return PTR_ERR(dp->dptx_rst);
	}

	dp->apb_rst = devm_reset_control_get(dev, "apb");
	if (IS_ERR(dp->apb_rst)) {
		dev_err(dev, "no apb reset control found\n");
		return PTR_ERR(dp->apb_rst);
	}

	return 0;
}

static int cdn_dp_audio_hw_params(struct device *dev,  void *data,
				  struct hdmi_codec_daifmt *daifmt,
				  struct hdmi_codec_params *params)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct audio_info audio = {
		.sample_width = params->sample_width,
		.sample_rate = params->sample_rate,
		.channels = params->channels,
	};
	int ret;

	if (!dp->encoder.crtc)
		return -ENODEV;

	switch (daifmt->fmt) {
	case HDMI_I2S:
		audio.format = AFMT_I2S;
		break;
	case HDMI_SPDIF:
		audio.format = AFMT_SPDIF;
		break;
	default:
		dev_err(dev, "%s: Invalid format %d\n", __func__, daifmt->fmt);
		return -EINVAL;
	}

	ret = cdn_dp_audio_config(dp, &audio);
	if (!ret)
		dp->audio_info = audio;

	return ret;
}

static void cdn_dp_audio_shutdown(struct device *dev, void *data)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	int ret;

	if (!dp->encoder.crtc)
		return;

	ret = cdn_dp_audio_stop(dp, &dp->audio_info);
	if (!ret)
		dp->audio_info.format = AFMT_UNUSED;
}

static int cdn_dp_audio_digital_mute(struct device *dev, void *data,
				     bool enable)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);

	if (!dp->encoder.crtc)
		return -ENODEV;

	return cdn_dp_audio_mute(dp, enable);
}

static int cdn_dp_audio_get_eld(struct device *dev, void *data,
				u8 *buf, size_t len)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct drm_mode_config *config = &dp->encoder.dev->mode_config;
	struct drm_connector *connector;
	int ret = -ENODEV;

	mutex_lock(&config->mutex);
	list_for_each_entry(connector, &config->connector_list, head) {
		if (&dp->encoder == connector->encoder) {
			memcpy(buf, connector->eld,
			       min(sizeof(connector->eld), len));
			ret = 0;
		}
	}
	mutex_unlock(&config->mutex);

	return ret;
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = cdn_dp_audio_hw_params,
	.audio_shutdown = cdn_dp_audio_shutdown,
	.digital_mute = cdn_dp_audio_digital_mute,
	.get_eld = cdn_dp_audio_get_eld,
};

static int cdn_dp_audio_codec_init(struct cdn_dp_device *dp,
				   struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.i2s = 1,
		.spdif = 1,
		.ops = &audio_codec_ops,
		.max_i2s_channels = 8,
	};

	dp->audio_pdev = platform_device_register_data(
			 dev, HDMI_CODEC_DRV_NAME, PLATFORM_DEVID_AUTO,
			 &codec_data, sizeof(codec_data));

	return PTR_ERR_OR_ZERO(dp->audio_pdev);
}

static int cdn_dp_get_cap_lanes(struct cdn_dp_device *dp,
				struct extcon_dev *edev)
{
	union extcon_property_value property;
	bool dptx;
	u8 lanes;

	if (dp->suspend)
		return 0;

	dptx = extcon_get_state(edev, EXTCON_DISP_DP);

	if (dptx) {
		extcon_get_property(edev, EXTCON_DISP_DP,
				    EXTCON_PROP_USB_SS, &property);
		if (property.intval)
			lanes = 2;
		else
			lanes = 4;
	} else {
		lanes = 0;
	}

	return lanes;
}

static int cdn_dp_get_dpcd(struct cdn_dp_device *dp, struct cdn_dp_port *port)
{
	u8 sink_count;
	int i, ret;

	/*
	 * Native read with retry for link status and receiver capability reads
	 * for cases where the sink may still not be ready.
	 *
	 * Sinks are *supposed* to come up within 1ms from an off state, but
	 * some DOCKs need about 5 seconds to power up, so read the dpcd every
	 * 100ms, if can not get a good dpcd in 10 seconds, give up.
	 */
	for (i = 0; i < 100; i++) {
		ret = cdn_dp_dpcd_read(dp, DP_SINK_COUNT,
				       &sink_count, 1);
		if (!ret) {
			dev_dbg(dp->dev, "get dpcd success!\n");

			sink_count = DP_GET_SINK_COUNT(sink_count);
			if (!sink_count)
				return -ENODEV;

			ret = cdn_dp_dpcd_read(dp, 0x000, dp->dpcd,
					       DP_RECEIVER_CAP_SIZE);
			if (ret)
				continue;

			return ret;
		} else if (!extcon_get_state(port->extcon, EXTCON_DISP_DP)) {
			break;
		}

		msleep(100);
	}

	dev_err(dp->dev, "get dpcd failed!\n");

	return -ETIMEDOUT;
}

static void cdn_dp_enter_standy(struct cdn_dp_device *dp,
				struct cdn_dp_port *port)
{
	int i, ret;

	ret = phy_power_off(port->phy);
	if (ret) {
		dev_err(dp->dev, "phy power off failed: %d", ret);
		return;
	}

	port->phy_enabled = false;
	port->cap_lanes = 0;
	for (i = 0; i < dp->ports; i++)
		if (dp->port[i]->phy_enabled)
			return;

	ret = cdn_dp_grf_write(dp, GRF_SOC_CON26,
			       DPTX_HPD_SEL_MASK | DPTX_HPD_DEL);
	if (ret)
		return;

	memset(dp->dpcd, 0, DP_RECEIVER_CAP_SIZE);
	dp->hpd_status = connector_status_disconnected;
	drm_helper_hpd_irq_event(dp->drm_dev);
	cdn_dp_set_firmware_active(dp, false);

	cdn_dp_clk_disable(dp);
}

static int cdn_dp_start_work(struct cdn_dp_device *dp,
			     struct cdn_dp_port *port,
			     u8 cap_lanes)
{
	union extcon_property_value property;
	int ret;

	ret = request_firmware(&dp->fw, CDN_DP_FIRMWARE, dp->dev);
	if (ret) {
		if (ret == -ENOENT && dp->fw_wait <= MAX_FW_WAIT_SECS) {
			unsigned long time = msecs_to_jiffies(dp->fw_wait * HZ);

			/*
			 * Keep trying to load the firmware for up to 1 minute,
			 * if can not find the file.
			 */
			schedule_delayed_work(&port->event_wq, time);
			dp->fw_wait *= 2;
		} else {
			dev_err(dp->dev, "failed to request firmware: %d\n",
				ret);
		}

		return ret;
	}

	ret = cdn_dp_clk_enable(dp);
	if (ret)
		return ret;

	ret = cdn_dp_grf_write(dp, GRF_SOC_CON26,
			       (port->id << UPHY_SEL_BIT) | UPHY_SEL_MASK);
	if (ret)
		goto err_phy;

	ret = phy_power_on(port->phy);
	if (ret) {
		dev_err(dp->dev, "phy power on failed: %d\n", ret);
		goto err_phy;
	}

	port->phy_enabled = true;

	ret = cdn_dp_firmware_init(dp);
	if (ret) {
		dev_err(dp->dev, "firmware init failed: %d", ret);
		goto err_firmware;
	}

	ret = cdn_dp_grf_write(dp, GRF_SOC_CON26,
			       DPTX_HPD_SEL_MASK | DPTX_HPD_SEL);
	if (ret)
		goto err_grf;

	ret = cdn_dp_get_hpd_status(dp);
	if (ret <= 0) {
		if (!ret)
			dev_err(dp->dev, "hpd does not exist\n");
		goto err_hpd;
	}

	ret = extcon_get_property(port->extcon, EXTCON_DISP_DP,
				  EXTCON_PROP_USB_TYPEC_POLARITY, &property);
	if (ret) {
		dev_err(dp->dev, "get property failed\n");
		goto err_hpd;
	}

	ret = cdn_dp_set_host_cap(dp, cap_lanes, property.intval);
	if (ret) {
		dev_err(dp->dev, "set host capabilities failed: %d\n", ret);
		goto err_hpd;
	}

	ret = cdn_dp_get_dpcd(dp, port);
	if (ret)
		goto err_hpd;

	release_firmware(dp->fw);
	return 0;

err_hpd:
	cdn_dp_grf_write(dp, GRF_SOC_CON26,
			 DPTX_HPD_SEL_MASK | DPTX_HPD_DEL);

err_grf:
	cdn_dp_set_firmware_active(dp, false);

err_firmware:
	if (phy_power_off(port->phy))
		dev_err(dp->dev, "phy power off failed: %d", ret);
	else
		port->phy_enabled = false;

err_phy:
	cdn_dp_clk_disable(dp);
	release_firmware(dp->fw);
	return ret;
}

static void cdn_dp_pd_event_wq(struct work_struct *work)
{
	struct cdn_dp_port *port = container_of(work, struct cdn_dp_port,
						event_wq.work);
	struct cdn_dp_device *dp = port->dp;
	u8 new_cap_lanes, sink_count, i;
	int ret;

	new_cap_lanes = cdn_dp_get_cap_lanes(dp, port->extcon);

	if (new_cap_lanes == port->cap_lanes) {
		if (!new_cap_lanes)
			return;
		/*
		 * If HPD interrupt is triggered, and cable states is still
		 * attached, that means something on the Type-C Dock/Dongle
		 * changed, check the sink count by DPCD. If sink count became
		 * 0, this port phy can be powered off; if the sink count does
		 * not change, it means the sink device status has update,
		 * re-training to make it work again.
		 */
		ret = cdn_dp_dpcd_read(dp, DP_SINK_COUNT, &sink_count, 1);
		if (ret || sink_count) {
			if (dp->dpms_mode == DRM_MODE_DPMS_ON) {
				ret = cdn_dp_training_start(dp);
				if (!ret)
					cdn_dp_get_training_status(dp);
			}
			return;
		}

		new_cap_lanes = 0;
	}

	if (!new_cap_lanes)
		return cdn_dp_enter_standy(dp, port);

	/* if other phy is running, do not do anything, just return */
	for (i = 0; i < dp->ports; i++) {
		if (dp->port[i]->phy_enabled) {
			dev_warn(dp->dev, "busy, phy[%d] is running",
				 dp->port[i]->id);
			return;
		}
	}

	ret = cdn_dp_start_work(dp, port, new_cap_lanes);
	if (ret)
		return;

	port->cap_lanes = new_cap_lanes;
	dp->hpd_status = connector_status_connected;
	drm_helper_hpd_irq_event(dp->drm_dev);
}

static int cdn_dp_pd_event(struct notifier_block *nb,
			   unsigned long event, void *priv)
{
	struct cdn_dp_port *port;

	port = container_of(nb, struct cdn_dp_port, event_nb);

	schedule_delayed_work(&port->event_wq, 0);

	return 0;
}

static int cdn_dp_bind(struct device *dev, struct device *master, void *data)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct cdn_dp_port *port;
	struct drm_device *drm_dev = data;
	int ret, i;

	pm_runtime_enable(dev);

	ret = cdn_dp_parse_dt(dp);
	if (ret < 0)
		return ret;

	dp->drm_dev = drm_dev;
	dp->hpd_status = connector_status_disconnected;
	dp->dpms_mode = DRM_MODE_DPMS_OFF;
	dp->fw_wait = 1;

	encoder = &dp->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);
	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &cdn_dp_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &cdn_dp_encoder_helper_funcs);

	connector = &dp->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &cdn_dp_atomic_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector, &cdn_dp_connector_helper_funcs);

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

	cdn_dp_audio_codec_init(dp, dev);

	for (i = 0; i < dp->ports; i++) {
		port = dp->port[i];

		port->event_nb.notifier_call = cdn_dp_pd_event;
		INIT_DELAYED_WORK(&port->event_wq, cdn_dp_pd_event_wq);
		ret = extcon_register_notifier(port->extcon, EXTCON_DISP_DP,
					       &port->event_nb);
		if (ret) {
			dev_err(dev, "register EXTCON_DISP_DP notifier err\n");
			goto err_free_connector;
		}

		if (extcon_get_state(port->extcon, EXTCON_DISP_DP))
			schedule_delayed_work(&port->event_wq, 0);
	}

	return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void cdn_dp_unbind(struct device *dev, struct device *master, void *data)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dp->encoder;
	struct drm_connector *connector = &dp->connector;
	struct cdn_dp_port *port;
	int i;

	platform_device_unregister(dp->audio_pdev);
	cdn_dp_encoder_disable(encoder);
	encoder->funcs->destroy(encoder);
	connector->funcs->destroy(connector);

	for (i = 0; i < dp->ports; i++) {
		port = dp->port[i];
		extcon_unregister_notifier(port->extcon, EXTCON_DISP_DP,
					   &port->event_nb);
	}

	pm_runtime_disable(dev);
}

static const struct component_ops cdn_dp_component_ops = {
	.bind = cdn_dp_bind,
	.unbind = cdn_dp_unbind,
};

int cdn_dp_suspend(struct device *dev)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct cdn_dp_port *port;
	int i;

	for (i = 0; i < dp->ports; i++) {
		port = dp->port[i];
		if (port->phy_enabled) {
			cdn_dp_dpcd_write(dp, DP_SET_POWER, DP_SET_POWER_D3);
			dp->suspend = true;
			schedule_delayed_work(&port->event_wq, 0);
		}
	}

	return 0;
}

int cdn_dp_resume(struct device *dev)
{
	struct cdn_dp_device *dp = dev_get_drvdata(dev);
	struct cdn_dp_port *port;
	int i;

	if (dp->suspend) {
		dp->suspend = false;
		for (i = 0; i < dp->ports; i++) {
			port = dp->port[i];
			schedule_delayed_work(&port->event_wq, 0);
		}
	}

	return 0;
}

static int cdn_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct cdn_dp_data *dp_data;
	struct cdn_dp_port *port;
	struct cdn_dp_device *dp;
	struct extcon_dev *extcon;
	struct phy *phy;
	int i;

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->dev = dev;

	match = of_match_node(cdn_dp_dt_ids, pdev->dev.of_node);
	dp_data = (struct cdn_dp_data *)match->data;

	for (i = 0; i < dp_data->max_phy; i++) {
		extcon = extcon_get_edev_by_phandle(dev, i);
		phy = devm_of_phy_get_by_index(dev, dev->of_node, i);

		if (PTR_ERR(extcon) == -EPROBE_DEFER ||
		    PTR_ERR(phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		if (IS_ERR(extcon) || IS_ERR(phy))
			continue;

		port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
		if (!dp)
			return -ENOMEM;

		port->extcon = extcon;
		port->phy = phy;
		port->dp = dp;
		port->id = i;
		dp->port[dp->ports++] = port;
	}

	if (!dp->ports) {
		dev_err(dev, "missing extcon or phy\n");
		return -EINVAL;
	}

	dev_set_drvdata(dev, dp);

	return component_add(dev, &cdn_dp_component_ops);
}

static int cdn_dp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cdn_dp_component_ops);

	return 0;
}

static const struct dev_pm_ops cdn_dp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cdn_dp_suspend,
				cdn_dp_resume)
};

static struct platform_driver cdn_dp_driver = {
	.probe = cdn_dp_probe,
	.remove = cdn_dp_remove,
	.driver = {
		   .name = "cdn-dp",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(cdn_dp_dt_ids),
		   .pm = &cdn_dp_pm_ops,
	},
};

module_platform_driver(cdn_dp_driver);

MODULE_AUTHOR("Chris Zhong <zyw@rock-chips.com>");
MODULE_DESCRIPTION("cdn DP Driver");
MODULE_LICENSE("GPL v2");
