/*
 * Rockchip VPU codec driver
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *	Jeffy Chen <jeffy.chen@rock-chips.com>
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

#include "rockchip_vpu_common.h"

#include <linux/clk.h>

#include "rk3288_vpu_regs.h"

/*
 * Supported formats.
 */

static const struct rockchip_vpu_fmt rk3288_vpu_enc_fmts[] = {
	/* Source formats. */
	{
		.name = "4:2:0 3 planes Y/Cb/Cr",
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.codec_mode = RK_VPU_CODEC_NONE,
		.num_planes = 3,
		.depth = { 8, 4, 4 },
		.enc_fmt = RK3288_VPU_ENC_FMT_YUV420P,
	},
	{
		.name = "4:2:0 2 plane Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV12M,
		.codec_mode = RK_VPU_CODEC_NONE,
		.num_planes = 2,
		.depth = { 8, 8 },
		.enc_fmt = RK3288_VPU_ENC_FMT_YUV420SP,
	},
	{
		.name = "4:2:2 1 plane YUYV",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.codec_mode = RK_VPU_CODEC_NONE,
		.num_planes = 1,
		.depth = { 16 },
		.enc_fmt = RK3288_VPU_ENC_FMT_YUYV422,
	},
	{
		.name = "4:2:2 1 plane UYVY",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.codec_mode = RK_VPU_CODEC_NONE,
		.num_planes = 1,
		.depth = { 16 },
		.enc_fmt = RK3288_VPU_ENC_FMT_UYVY422,
	},
	/* Destination formats. */
	{
		.name = "VP8 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_VP8,
		.codec_mode = RK_VPU_CODEC_VP8E,
		.num_planes = 1,
	},
	{
		.name = "H264 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H264,
		.codec_mode = RK_VPU_CODEC_H264E,
		.num_planes = 1,
	},
};

static const struct rockchip_vpu_fmt rk3288_vpu_dec_fmts[] = {
	{
		.name = "4:2:0 1 plane Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV12,
		.codec_mode = RK_VPU_CODEC_NONE,
		.num_planes = 1,
		.depth = { 12 },
	},
	{
		.name = "Slices of H264 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.codec_mode = RK_VPU_CODEC_H264D,
		.num_planes = 1,
	},
	{
		.name = "Frames of VP8 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_VP8_FRAME,
		.codec_mode = RK_VPU_CODEC_VP8D,
		.num_planes = 1,
	},
};

/*
 * Interrupt handlers.
 */

static irqreturn_t rk3288_vepu_irq(int irq, void *dev_id)
{
	struct rockchip_vpu_dev *vpu = dev_id;
	u32 status = vepu_read(vpu, VEPU_REG_INTERRUPT);

	vepu_write(vpu, 0, VEPU_REG_INTERRUPT);

	if (status & VEPU_REG_INTERRUPT_BIT) {
		vepu_write(vpu, 0, VEPU_REG_AXI_CTRL);

		rockchip_vpu_irq_done(vpu);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rk3288_vdpu_irq(int irq, void *dev_id)
{
	struct rockchip_vpu_dev *vpu = dev_id;
	u32 status = vdpu_read(vpu, VDPU_REG_INTERRUPT);

	vdpu_write(vpu, 0, VDPU_REG_INTERRUPT);

	vpu_debug(3, "vdpu_irq status: %08x\n", status);

	if (status & VDPU_REG_INTERRUPT_DEC_IRQ) {
		vdpu_write(vpu, 0, VDPU_REG_CONFIG);

		rockchip_vpu_irq_done(vpu);
	}

	return IRQ_HANDLED;
}

/*
 * Initialization/clean-up.
 */

static int rk3288_vpu_hw_probe(struct rockchip_vpu_dev *vpu)
{
	struct resource *res;
	int irq_enc, irq_dec;
	int ret;

	vpu->aclk = devm_clk_get(vpu->dev, "aclk_vcodec");
	if (IS_ERR(vpu->aclk)) {
		dev_err(vpu->dev, "failed to get aclk\n");
		return PTR_ERR(vpu->aclk);
	}

	vpu->hclk = devm_clk_get(vpu->dev, "hclk_vcodec");
	if (IS_ERR(vpu->hclk)) {
		dev_err(vpu->dev, "failed to get hclk\n");
		return PTR_ERR(vpu->hclk);
	}

	/*
	 * Bump ACLK to max. possible freq. (400 MHz) to improve performance.
	 *
	 * VP8 encoding 1280x720@1.2Mbps 200 MHz: 39 fps, 400: MHz 77 fps
	 */
	clk_set_rate(vpu->aclk, 400 * 1000 * 1000);

	res = platform_get_resource(vpu->pdev, IORESOURCE_MEM, 0);
	vpu->base = devm_ioremap_resource(vpu->dev, res);
	if (IS_ERR(vpu->base))
		return PTR_ERR(vpu->base);

	vpu->enc_base = vpu->base + vpu->variant->enc_offset;
	vpu->dec_base = vpu->base + vpu->variant->dec_offset;

	ret = dma_set_coherent_mask(vpu->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(vpu->dev, "could not set DMA coherent mask\n");
		return ret;
	}

	irq_enc = platform_get_irq_byname(vpu->pdev, "vepu");
	if (irq_enc <= 0) {
		dev_err(vpu->dev, "could not get vepu IRQ\n");
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(vpu->dev, irq_enc, NULL,
					rk3288_vepu_irq, IRQF_ONESHOT,
					dev_name(vpu->dev), vpu);
	if (ret) {
		dev_err(vpu->dev, "could not request vepu IRQ\n");
		return ret;
	}

	irq_dec = platform_get_irq_byname(vpu->pdev, "vdpu");
	if (irq_dec <= 0) {
		dev_err(vpu->dev, "could not get vdpu IRQ\n");
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(vpu->dev, irq_dec, NULL,
					rk3288_vdpu_irq, IRQF_ONESHOT,
					dev_name(vpu->dev), vpu);
	if (ret) {
		dev_err(vpu->dev, "could not request vdpu IRQ\n");
		return ret;
	}

	return 0;
}

static void rk3288_vpu_clk_enable(struct rockchip_vpu_dev *vpu)
{
	clk_prepare_enable(vpu->aclk);
	clk_prepare_enable(vpu->hclk);
}

static void rk3288_vpu_clk_disable(struct rockchip_vpu_dev *vpu)
{
	clk_disable_unprepare(vpu->hclk);
	clk_disable_unprepare(vpu->aclk);
}

static void rk3288_vpu_enc_reset(struct rockchip_vpu_ctx *ctx)
{
	struct rockchip_vpu_dev *vpu = ctx->dev;

	vepu_write(vpu, VEPU_REG_INTERRUPT_DIS_BIT, VEPU_REG_INTERRUPT);
	vepu_write(vpu, 0, VEPU_REG_ENC_CTRL);
	vepu_write(vpu, 0, VEPU_REG_AXI_CTRL);
}

static void rk3288_vpu_dec_reset(struct rockchip_vpu_ctx *ctx)
{
	struct rockchip_vpu_dev *vpu = ctx->dev;

	vdpu_write(vpu, VDPU_REG_INTERRUPT_DEC_IRQ_DIS, VDPU_REG_INTERRUPT);
	vdpu_write(vpu, 0, VDPU_REG_CONFIG);
}

/*
 * Supported codec ops.
 */

static const struct rockchip_vpu_codec_ops rk3288_vpu_mode_ops[] = {
	[RK_VPU_CODEC_VP8E] = {
		.init = rk3288_vpu_vp8e_init,
		.exit = rk3288_vpu_vp8e_exit,
		.run = rk3288_vpu_vp8e_run,
		.done = rk3288_vpu_vp8e_done,
		.reset = rk3288_vpu_enc_reset,
	},
	[RK_VPU_CODEC_VP8D] = {
		.init = rk3288_vpu_vp8d_init,
		.exit = rk3288_vpu_vp8d_exit,
		.run = rk3288_vpu_vp8d_run,
		.done = rockchip_vpu_run_done,
		.reset = rk3288_vpu_dec_reset,
	},
	[RK_VPU_CODEC_H264E] = {
		.init = rk3288_vpu_h264e_init,
		.exit = rk3288_vpu_h264e_exit,
		.run = rk3288_vpu_h264e_run,
		.done = rk3288_vpu_h264e_done,
		.reset = rk3288_vpu_enc_reset,
	},
	[RK_VPU_CODEC_H264D] = {
		.init = rk3288_vpu_h264d_init,
		.exit = rk3288_vpu_h264d_exit,
		.run = rk3288_vpu_h264d_run,
		.done = rockchip_vpu_run_done,
		.reset = rk3288_vpu_dec_reset,
	},
};

/*
 * VPU variant.
 */

const struct rockchip_vpu_variant rk3288_vpu_variant = {
	.enc_offset = 0x0,
	.enc_reg_num = 164,
	.dec_offset = 0x400,
	.dec_reg_num = 60 + 41,
	.needs_enc_after_dec_war = 1,
	.needs_dpb_map = 1,
	.enc_fmts = rk3288_vpu_enc_fmts,
	.num_enc_fmts = ARRAY_SIZE(rk3288_vpu_enc_fmts),
	.dec_fmts = rk3288_vpu_dec_fmts,
	.num_dec_fmts = ARRAY_SIZE(rk3288_vpu_dec_fmts),
	.mode_ops = rk3288_vpu_mode_ops,
	.hw_probe = rk3288_vpu_hw_probe,
	.clk_enable = rk3288_vpu_clk_enable,
	.clk_disable = rk3288_vpu_clk_disable,
};
