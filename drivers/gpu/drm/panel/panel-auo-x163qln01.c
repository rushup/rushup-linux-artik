/*
 * MIPI-DSI based AUO x163qln01 TFT LCD 1.63 inch panel driver.
 *
 * Copyright (c) 2016 Kalpa srl
 *
 * Chanho Park <danilo.sia@kalpa.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/backlight.h>

struct x163qln {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data supplies[2];
	int reset_gpio;
	int psr_te_gpio;
	u32 power_on_delay;
	u32 reset_delay;
	u32 init_delay;
	bool flip_horizontal;
	bool flip_vertical;
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
	bool is_power_on;

	u8 id[3];
	/* This field is tested by functions directly accessing DSI bus before
	 * transfer, transfer is skipped if it is set. In case of transfer
	 * failure or unexpected response the field is set to error value.
	 * Such construct allows to eliminate many checks in higher level
	 * functions.
	 */
	int error;
};

static inline struct x163qln *panel_to_x163qln(struct drm_panel *panel)
{
	return container_of(panel, struct x163qln, panel);
}

static int x163qln_clear_error(struct x163qln *ctx)
{
	int ret = ctx->error;

	ctx->error = 0;
	return ret;
}

static void x163qln_dcs_turn_on_peripheral(struct x163qln *ctx)
{
	ssize_t ret;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (ctx->error < 0)
		return;

	ret = 	mipi_dsi_turn_on_peripheral(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd turning on  dcs seq\n", ret);
		ctx->error = ret;
	}
}


static void x163qln_dcs_display_on(struct x163qln *ctx)
{
	ssize_t ret;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (ctx->error < 0)
		return;

	ret = 	mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd turning on  dcs seq\n", ret);
		ctx->error = ret;
	}
}


static void x163qln_dcs_display_off(struct x163qln *ctx)
{
	ssize_t ret;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (ctx->error < 0)
		return;

	ret = 	mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd turning on  dcs seq\n", ret);
		ctx->error = ret;
	}
}

static void x163qln_dcs_exit_sleep_mode(struct x163qln *ctx)
{
	ssize_t ret;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (ctx->error < 0)
		return;

	ret = 	mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd turning on  dcs seq\n", ret);
		ctx->error = ret;
	}
}

static void x163qln_dcs_enter_sleep_mode(struct x163qln *ctx)
{
	ssize_t ret;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (ctx->error < 0)
		return;

	ret = 	mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd turning on  dcs seq\n", ret);
		ctx->error = ret;
	}
}


static void x163qln_dcs_write(struct x163qln *ctx, const void *data,
		size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing dcs seq: %*ph\n", ret,
			(int)len, data);
		ctx->error = ret;
	}
}

#define x163qln_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	x163qln_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define x163qln_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	x163qln_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static void x163qln_apply_power_cond(struct x163qln *ctx)
{
	x163qln_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00);
	x163qln_dcs_write_seq_static(ctx,	0xBD, 0x01, 0x90, 0x14, 0x14, 0x00);
	x163qln_dcs_write_seq_static(ctx,	0xBE, 0x01, 0x90, 0x14, 0x14, 0x01);
	x163qln_dcs_write_seq_static(ctx,	0xBF, 0x01, 0x90, 0x14, 0x14, 0x00);
	x163qln_dcs_write_seq_static(ctx,	0xBB, 0x07, 0x07, 0x07);
	x163qln_dcs_write_seq_static(ctx,	0xC7, 0x40);
	x163qln_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02);
	x163qln_dcs_write_seq_static(ctx, 0xFE, 0x08, 0x50);
	x163qln_dcs_write_seq_static(ctx, 0xC3, 0xF2, 0x85, 0x04);
	x163qln_dcs_write_seq_static(ctx, 0xCA, 0x04);
	x163qln_dcs_write_seq_static(ctx, 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01);
	x163qln_dcs_write_seq_static(ctx, 0xB0, 0x03, 0x03, 0x03);
	x163qln_dcs_write_seq_static(ctx, 0xB1, 0x05, 0x05, 0x05);
	x163qln_dcs_write_seq_static(ctx, 0xB2, 0x01, 0x01, 0x01);
	x163qln_dcs_write_seq_static(ctx, 0xB4, 0x07, 0x07, 0x07);
	x163qln_dcs_write_seq_static(ctx, 0xB5, 0x05, 0x05, 0x05);
	x163qln_dcs_write_seq_static(ctx, 0xB6, 0x53, 0x53, 0x53);
	x163qln_dcs_write_seq_static(ctx, 0xB7, 0x33, 0x33, 0x33);
	x163qln_dcs_write_seq_static(ctx, 0xB8, 0x23, 0x23, 0x23);
	x163qln_dcs_write_seq_static(ctx, 0xB9, 0x03, 0x03, 0x03);
	x163qln_dcs_write_seq_static(ctx, 0xBA, 0x13, 0x13, 0x13);
	x163qln_dcs_write_seq_static(ctx, 0xBE, 0x22, 0x30, 0x70);
	x163qln_dcs_write_seq_static(ctx, 0xCF, 0xFF, 0xD4, 0x95, 0xEF, 0x4F, 0x00, 0x04);


	x163qln_dcs_write_seq_static(ctx, 0x35, 0x01);
	x163qln_dcs_write_seq_static(ctx, 0x36, 0x00);
	x163qln_dcs_write_seq_static(ctx, 0xC0, 0x20);
	x163qln_dcs_write_seq_static(ctx, 0xC2, 0x17, 0x17, 0x17, 0x17, 0x17, 0x0B);
	ctx->error =0;

	x163qln_dcs_turn_on_peripheral(ctx);
	x163qln_dcs_exit_sleep_mode(ctx);
	msleep(300);
	x163qln_dcs_display_on(ctx);

}

static void x163qln_gamma_setting(struct x163qln *ctx)
{

}

static void x163qln_apply_display_parameter(struct x163qln *ctx)
{

}

static void x163qln_set_maximum_return_packet_size(struct x163qln *ctx,
						   u16 size)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_set_maximum_return_packet_size(dsi, size);
	if (ret < 0) {
		dev_err(ctx->dev,
			"error %d setting maximum return packet size to %d\n",
			ret, size);
		ctx->error = ret;
	}
}

static int x163qln_dcs_read(struct x163qln *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->error < 0)
		return ctx->error;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void x163qln_read_mtp_id(struct x163qln *ctx)
{
	int ret;
	int id_len = ARRAY_SIZE(ctx->id);

	ret = x163qln_dcs_read(ctx, 0x04, ctx->id, id_len);
	if (ret < id_len || ctx->error < 0) {
		dev_err(ctx->dev, "read id failed\n");
		ctx->error = -EIO;
		return;
	}
}

static void x163qln_panel_init(struct x163qln *ctx)
{
	x163qln_apply_power_cond(ctx);
/*
	x163qln_set_maximum_return_packet_size(ctx, 3);

	x163qln_read_mtp_id(ctx);
	if (ctx->error != 0)
		return;
*/
	x163qln_apply_display_parameter(ctx);
}

static int x163qln_power_on(struct x163qln *ctx)
{
	int ret;

	if (ctx->is_power_on)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;


	msleep(ctx->power_on_delay);

	gpio_direction_output(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpio_set_value(ctx->reset_gpio, 1);

	msleep(ctx->reset_delay);

	ctx->is_power_on = true;

	return 0;
}

static int x163qln_power_off(struct x163qln *ctx)
{
	if (!ctx->is_power_on)
		return 0;
	gpio_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	ctx->is_power_on = false;

	return 0;
}

static int x163qln_disable(struct drm_panel *panel)
{
	struct x163qln *ctx = panel_to_x163qln(panel);
	x163qln_dcs_display_off(ctx);
	x163qln_dcs_enter_sleep_mode(ctx);
	msleep(120);

	return 0;
}

static int x163qln_unprepare(struct drm_panel *panel)
{
	struct x163qln *ctx = panel_to_x163qln(panel);
	int ret;

	ret = x163qln_power_off(ctx);
	if (ret)
		return ret;

	x163qln_clear_error(ctx);

	return 0;
}

static int x163qln_prepare(struct drm_panel *panel)
{
	struct x163qln *ctx = panel_to_x163qln(panel);
	int ret;

	ret = x163qln_power_on(ctx);
	if (ret < 0)
		return ret;

	x163qln_panel_init(ctx);
	ret = ctx->error;

	if (ret < 0)
		x163qln_unprepare(panel);

	return ret;
}

static int x163qln_enable(struct drm_panel *panel)
{
	struct x163qln *ctx = panel_to_x163qln(panel);

	x163qln_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_ON);
	if (ctx->error != 0)
		return ctx->error;

	x163qln_dcs_write_seq_static(ctx, MIPI_DCS_WRITE_MEMORY_START);
	if (ctx->error != 0)
		return ctx->error;

	return 0;
}

static int x163qln_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct x163qln *ctx = panel_to_x163qln(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	drm_display_mode_from_videomode(&ctx->vm, mode);
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs x163qln_drm_funcs = {
	.disable = x163qln_disable,
	.unprepare = x163qln_unprepare,
	.prepare = x163qln_prepare,
	.enable = x163qln_enable,
	.get_modes = x163qln_get_modes,
};

static int x163qln_parse_dt(struct x163qln *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_get_videomode(np, &ctx->vm, 0);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "power-on-delay", &ctx->power_on_delay);
	of_property_read_u32(np, "reset-delay", &ctx->reset_delay);
	of_property_read_u32(np, "init-delay", &ctx->init_delay);
	of_property_read_u32(np, "panel-width-mm", &ctx->width_mm);
	of_property_read_u32(np, "panel-height-mm", &ctx->height_mm);

	ctx->flip_horizontal = of_property_read_bool(np, "flip-horizontal");
	ctx->flip_vertical = of_property_read_bool(np, "flip-vertical");

	return 0;
}

static int x163qln_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct x163qln *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct x163qln), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	ctx->is_power_on = false;
	dsi->lanes = 1;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
		| MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_VSYNC_FLUSH
		| MIPI_DSI_MODE_VIDEO_AUTO_VERT;
    */
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
                      | MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
                      | MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_VSYNC_FLUSH
                      | MIPI_DSI_MODE_VIDEO_AUTO_VERT;

	ret = x163qln_parse_dt(ctx);
	if (ret < 0)
		return ret;

	ctx->supplies[0].supply = "vdd3";
	ctx->supplies[1].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		dev_warn(dev, "failed to get regulators: %d\n", ret);

	ctx->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	if (ctx->reset_gpio < 0) {
		dev_err(dev, "cannot get reset-gpios %d\n",
			ctx->reset_gpio);
		return ctx->reset_gpio;
	}

	ret = devm_gpio_request(dev, ctx->reset_gpio, "reset-gpio");
	if (ret) {
		dev_err(dev, "failed to request reset-gpio\n");
		return ret;
	}

	ctx->psr_te_gpio = of_get_named_gpio(dev->of_node, "psr-te-gpio", 0);
	if (ctx->psr_te_gpio < 0) {
		dev_err(dev, "cannot get psr te gpios %d\n",
			ctx->psr_te_gpio);
		return ctx->psr_te_gpio;
	}

	ret = devm_gpio_request(dev, ctx->psr_te_gpio, "psr-te-gpio");
	if (ret) {
		dev_err(dev, "failed to request psr te gpio\n");
		return ret;
	}

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &x163qln_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	return ret;
}

static int x163qln_remove(struct mipi_dsi_device *dsi)
{
	struct x163qln *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	x163qln_power_off(ctx);

	return 0;
}

static void x163qln_shutdown(struct mipi_dsi_device *dsi)
{
	struct x163qln *ctx = mipi_dsi_get_drvdata(dsi);

	x163qln_power_off(ctx);
}

static const struct of_device_id x163qln_of_match[] = {
	{ .compatible = "auo,x163qln" },
	{ }
};

MODULE_DEVICE_TABLE(of, x163qln_of_match);

static struct mipi_dsi_driver x163qln_driver = {
	.probe = x163qln_probe,
	.remove = x163qln_remove,
	.shutdown = x163qln_shutdown,
	.driver = {
		.name = "panel-auo-x163qln",
		.of_match_table = x163qln_of_match,
	},
};
module_mipi_dsi_driver(x163qln_driver);

MODULE_AUTHOR("Danilo Sia <danilo.sia@kalpa.it>");
MODULE_DESCRIPTION("MIPI-DSI based x163qln TFT LCD Panel Driver");
MODULE_LICENSE("GPL v2");
