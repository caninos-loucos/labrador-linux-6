// SPDX-License-Identifier: GPL-2.0
/*
 * DRM/KMS driver for Caninos Labrador
 *
 * Copyright (c) 2022-2023 ITEX - LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * Copyright (c) 2018-2020 LSITEC - Caninos Loucos
 * Author: Edgar Bernardi Righi <edgar.righi@lsitec.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_drv.h>

#include "caninos-drm-priv.h"

#define DRIVER_NAME "caninos-drm"
#define DRIVER_DESC "Caninos Labrador DRM/KMS driver"

static void caninos_update(struct drm_simple_display_pipe *pipe,
                           struct drm_plane_state *plane_state)
{
	struct caninos_gfx *priv = container_of(pipe, struct caninos_gfx, pipe);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_framebuffer *fb = pipe->plane.state->fb;
	struct drm_pending_vblank_event *event;
	struct drm_gem_dma_object *gem;
	
	spin_lock_irq(&crtc->dev->event_lock);
	event = crtc->state->event;
	
	if (event) {
		crtc->state->event = NULL;

		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
	}
	
	spin_unlock_irq(&crtc->dev->event_lock);
	
	if (!fb) {
		return;
	}
	
	gem = drm_fb_dma_get_gem_obj(fb, 0);
	
	if (gem) {
		caninos_vdc_set_fbaddr(priv->caninos_vdc, (u32)gem->dma_addr);
	}
}

static void caninos_enable(struct drm_simple_display_pipe *pipe,
                           struct drm_crtc_state *crtc_state,
                           struct drm_plane_state *plane_state)
{
	struct caninos_gfx *priv = container_of(pipe, struct caninos_gfx, pipe);
	struct drm_display_mode *drm_mode = &crtc_state->adjusted_mode;
	struct caninos_hdmi *caninos_hdmi = priv->caninos_hdmi;
	struct caninos_vdc *caninos_vdc = priv->caninos_vdc;
	struct caninos_vdc_mode vdc_mode;
	int width, height, vrefresh;
	
	width = drm_mode->hdisplay;
	height = drm_mode->vdisplay;
	vrefresh = drm_mode_vrefresh(drm_mode);
	
	if ((width == 640) && (height == 480) && (vrefresh == 60)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID640x480P_60_4VS3);
	}
	else if ((width == 720) && (height == 480) && (vrefresh == 60)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID720x480P_60_4VS3);
	}
	else if ((width == 720) && (height == 576) && (vrefresh == 50)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID720x576P_50_4VS3);
	}
	else if ((width == 1280) && (height == 720) && (vrefresh == 60)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID1280x720P_60_16VS9);
	}
	else if ((width == 1280) && (height == 720) && (vrefresh == 50)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID1280x720P_50_16VS9);
	}
	else if ((width == 1920) && (height == 1080) && (vrefresh == 50)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID1920x1080P_50_16VS9);
	}
	else if ((width == 1920) && (height == 1080) && (vrefresh == 60)) {
		caninos_hdmi_set_mode(caninos_hdmi, VID1920x1080P_60_16VS9);
	}
	else {
		return;
	}
	
	vdc_mode.width  = width;
	vdc_mode.height = height;
	vdc_mode.format = DRM_FORMAT_XRGB8888;
	
	#ifdef CONFIG_DRM_CANINOS_HDMI_AUDIO
	caninos_snd_hdmi_abort(priv->caninos_snd_hdmi);
	#endif /* CONFIG_DRM_CANINOS_HDMI_AUDIO */
	
	caninos_hdmi_disable(caninos_hdmi);
	
	caninos_vdc_disable(caninos_vdc);
	caninos_vdc_set_mode(caninos_vdc, &vdc_mode);
	caninos_vdc_enable(caninos_vdc);
	
	caninos_hdmi_enable(caninos_hdmi);
	
	drm_crtc_vblank_on(&pipe->crtc);
}

static void caninos_disable(struct drm_simple_display_pipe *pipe)
{
	struct caninos_gfx *priv = container_of(pipe, struct caninos_gfx, pipe);
	struct caninos_hdmi *caninos_hdmi = priv->caninos_hdmi;
	struct caninos_vdc *caninos_vdc = priv->caninos_vdc;
	
	drm_crtc_vblank_off(&pipe->crtc);
	
	#ifdef CONFIG_DRM_CANINOS_HDMI_AUDIO
	caninos_snd_hdmi_abort(priv->caninos_snd_hdmi);
	#endif /* CONFIG_DRM_CANINOS_HDMI_AUDIO */
	
	caninos_hdmi_disable(caninos_hdmi);
	caninos_vdc_disable(caninos_vdc);
}

static int caninos_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	return 0;
}

static void caninos_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	return;
}

static enum drm_mode_status
caninos_mode_valid(struct drm_simple_display_pipe *pipe, 
                   const struct drm_display_mode *mode)
{
	int w = mode->hdisplay, h = mode->vdisplay;
	int vrefresh = drm_mode_vrefresh(mode);
	
	if ((w == 640) && (h == 480) && (vrefresh == 60)) {
		return MODE_OK;
	}
	if ((w == 720) && (h == 480) && (vrefresh == 60)) {
		return MODE_OK;
	}
	if ((w == 720) && (h == 576) && (vrefresh == 50)) {
		return MODE_OK;
	}
	if ((w == 1280) && (h == 720) && ((vrefresh == 60) || (vrefresh == 50))) {
		return MODE_OK;
	}
	if ((w == 1920) && (h == 1080) && ((vrefresh == 60) || (vrefresh == 50))) {
		return MODE_OK;
	}
	return MODE_BAD;
}

static const uint32_t caninos_pipe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static struct drm_simple_display_pipe_funcs caninos_pipe_funcs = {
	.mode_valid     = caninos_mode_valid,
	.enable         = caninos_enable,
	.disable        = caninos_disable,
	.update         = caninos_update,
	.enable_vblank  = caninos_enable_vblank,
	.disable_vblank = caninos_disable_vblank,
	.prepare_fb     = drm_gem_simple_display_pipe_prepare_fb,
};

static int caninos_gfx_pipe_init(struct caninos_gfx *priv)
{
	return drm_simple_display_pipe_init(&priv->drm, &priv->pipe, 
	                                    &caninos_pipe_funcs,
	                                    caninos_pipe_formats,
	                                    ARRAY_SIZE(caninos_pipe_formats),
	                                    NULL, &priv->connector);
}

/// ------------------------------------------------------------------------ ///

static int caninos_conn_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	int count, max_width, max_height;
	
	max_width = drm->mode_config.max_width;
	max_height = drm->mode_config.max_height;
	
	count = drm_add_modes_noedid(connector, max_width, max_height);
	
	if (count) {
		drm_set_preferred_mode(connector, 1920, 1080);
	}
	return count;
}

static const struct drm_connector_funcs caninos_conn_funcs = {
	.fill_modes             = drm_helper_probe_single_connector_modes,
	.destroy                = drm_connector_cleanup,
	.reset                  = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs caninos_conn_helper = {
	.get_modes = caninos_conn_get_modes,
};

static int caninos_conn_init(struct caninos_gfx *priv)
{
	priv->connector.dpms = DRM_MODE_DPMS_OFF;
	priv->connector.polled = 0;
	
	drm_connector_helper_add(&priv->connector, &caninos_conn_helper);
	
	return drm_connector_init(&priv->drm, &priv->connector,
	                          &caninos_conn_funcs, DRM_MODE_CONNECTOR_HDMIA);
}

/// ------------------------------------------------------------------------ ///

static const struct drm_mode_config_funcs caninos_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create,
	.atomic_check  = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int caninos_mode_config_init(struct caninos_gfx *priv)
{
	struct drm_device *drm = &priv->drm;
	int ret;
	
	ret = drmm_mode_config_init(drm);
	
	if (ret) {
		return ret;
	}
	
	drm->mode_config.min_width = 480;
	drm->mode_config.min_height = 480;
	drm->mode_config.max_width = 1920;
	drm->mode_config.max_height = 1920;
	
	drm->mode_config.funcs = &caninos_mode_config_funcs;
	
	return 0;
}

/// ------------------------------------------------------------------------ ///

DEFINE_DRM_GEM_DMA_FOPS(caninos_fops);

static struct drm_driver caninos_drm_drv = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops            = &caninos_fops,
	.name            = DRIVER_NAME,
	.desc            = DRIVER_DESC,
	.date            = "20230908",
	.major           = 1,
	.minor           = 4,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
};

static inline void *caninos_get_drvdata_by_node(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	return pdev ? platform_get_drvdata(pdev) : NULL;
}

static irqreturn_t caninos_drm_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct caninos_gfx *priv = container_of(drm, struct caninos_gfx, drm);
	drm_crtc_handle_vblank(&priv->pipe.crtc);
	return IRQ_HANDLED;
}

static void caninos_drm_unload(struct drm_device *drm)
{
	struct caninos_gfx *priv = container_of(drm, struct caninos_gfx, drm);
	drm_kms_helper_poll_fini(drm);
	caninos_vdc_free_irq(priv->caninos_vdc, drm);
}

static int caninos_drm_load(struct drm_device *drm)
{
	struct caninos_gfx *priv = container_of(drm, struct caninos_gfx, drm);
	struct device *dev = drm->dev;
	struct device_node *np;
	int ret;
	
	if (!dev->of_node) {
		dev_err(dev, "missing device of node\n");
		return -ENODEV;
	}
	
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	
	if (ret) {
		dev_err(dev, "unable to set dma mask\n");
		return ret;
	}
	
	ret = of_reserved_mem_device_init(dev);
	
	if (ret) {
		dev_err(dev, "failed to initialize reserved mem: %d\n", ret);
		return ret;
	}
	
	np = of_parse_phandle(dev->of_node, "hdmi-controller", 0);
	
	if (np) {
		priv->caninos_hdmi = caninos_get_drvdata_by_node(np);
		of_node_put(np);
	}
	if (!priv->caninos_hdmi) {
		dev_info(dev, "unable to get handle of hdmi controller\n");
		return -EPROBE_DEFER;
	}
	
	np = of_parse_phandle(dev->of_node, "display-controller", 0);
	
	if (np) {
		priv->caninos_vdc = caninos_get_drvdata_by_node(np);
		of_node_put(np);
	}
	if (!priv->caninos_vdc) {
		dev_info(dev, "unable to get handle of video display controller\n");
		return -EPROBE_DEFER;
	}
	
	#ifdef CONFIG_DRM_CANINOS_HDMI_AUDIO
	np = of_parse_phandle(dev->of_node, "hdmi-sndcard", 0);
	
	if (np) {
		priv->caninos_snd_hdmi = caninos_get_drvdata_by_node(np);
		of_node_put(np);
	}
	if (!priv->caninos_snd_hdmi) {
		dev_info(dev, "unable to get handle of hdmi soundcard\n");
		return -EPROBE_DEFER;
	}
	#endif /* CONFIG_DRM_CANINOS_HDMI_AUDIO */
	
	ret = caninos_mode_config_init(priv);
	
	if (ret) {
		dev_err(dev, "mode config init failed\n");
		return ret;
	}
	
	ret = drm_vblank_init(drm, 1);
	
	if (ret) {
		dev_err(dev, "failed to initialise vblank\n");
		return ret;
	}
	
	ret = caninos_conn_init(priv);
	
	if (ret) {
		dev_err(dev, "connector init failed\n");
		return ret;
	}
	
	ret = caninos_gfx_pipe_init(priv);
	
	if (ret) {
	    dev_err(dev, "could not init display pipe\n");
		return ret;
	}
	
	ret = caninos_vdc_request_irq(priv->caninos_vdc,
	                              caninos_drm_irq_handler, drm);
	
	if (ret) {
		dev_err(dev, "failed to install irq handler\n");
		return ret;
	}
	
	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);
	return 0;
}

static int caninos_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct caninos_gfx *priv;
	int ret;
	
	priv = devm_drm_dev_alloc(dev, &caninos_drm_drv, struct caninos_gfx, drm);
	
	if (IS_ERR(priv)) {
		dev_err(dev, "unable to allocate drm device\n");
		return PTR_ERR(priv);
	}
	
	ret = caninos_drm_load(&priv->drm);
	
	if (ret) {
		return ret;
	}
	
	ret = drm_dev_register(&priv->drm, 0);
	
	if (ret) {
		caninos_drm_unload(&priv->drm);
		return ret;
	}
	
	drm_fbdev_generic_setup(&priv->drm, 32);
	return 0;
}

static int caninos_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	
	drm_dev_unregister(drm);
	caninos_drm_unload(drm);
	
	return 0;
}

static const struct of_device_id caninos_drm_match[] = {
	{ .compatible = "caninos,drm" },
	{ /* sentinel */ }
};

struct platform_driver caninos_drm_plat_driver = {
	.probe = caninos_drm_probe,
	.remove = caninos_drm_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(caninos_drm_match),
		.owner = THIS_MODULE,
	},
};

static struct platform_driver * const drivers[] = {
	&caninos_hdmi_plat_driver,
#ifdef CONFIG_DRM_CANINOS_HDMI_AUDIO
	&caninos_hdmi_audio_plat_driver,
#endif
	&caninos_vdc_plat_driver,
	&caninos_drm_plat_driver,
};

static int __init caninos_module_init(void)
{
	return platform_register_drivers(drivers, ARRAY_SIZE(drivers));
}

static void __exit caninos_module_fini(void)
{
	platform_unregister_drivers(drivers, ARRAY_SIZE(drivers));
}

module_init(caninos_module_init);
module_exit(caninos_module_fini);

MODULE_AUTHOR("Edgar Bernardi Righi <edgar.righi@lsitec.org.br>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
