/*
 * Copyright (C) Purism SPC 2018
 *
 * Authors: Guido Günther <agx@sigxcpu.org>
 *
 * SPDX-License-Identifier: GPL-2.0
 */
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <video/mipi_display.h>
#include <video/display_timing.h>

#define DRV_NAME "jh057n00900"

/* The Rockteck jh057n00900 uses a Sitronix ST7703 */

/* Manufacturer Command Set */
#define ST7703_CMD_SETDISP  0xB2    /* display resolution */
#define ST7703_CMD_SETRGBIF 0xB3    /* porch adjustment */
#define ST7703_CMD_SETCYC   0xB4    /* display inversion type */
#define ST7703_CMD_SETBGP   0xB5    /* Reference Voltage */
#define ST7703_CMD_SETVCOM  0xB6    /* VCom */
#define ST7703_CMD_SETOTP   0xB7
#define ST7703_CMD_SETPOWER_EXT 0xB8
#define ST7703_CMD_SETEXTC  0xB9
#define ST7703_CMD_SETMIPI  0xBA
#define ST7703_CMD_SETVDC   0xBC
#define ST7703_CMD_SETSCR   0xC0
#define ST7703_CMD_SETPOWER 0xC1
#define ST7703_CMD_SETPANEL 0xCC
#define ST7703_CMD_SETGAMMA 0xE0
#define ST7703_CMD_SETEQ    0xE3
#define ST7703_CMD_SETGIP1  0xE9
#define ST7703_CMD_SETGIP2  0xEA

struct jh057n {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.hdisplay    = 720,
	.hsync_start = 720 + 90 /* front porch */,
	.hsync_end   = 720 + 90 + 20 /* sync_len */,
	.htotal      = 720 + 90 + 20 + 20 /* back porch */,
	.vdisplay    = 1440,
	.vsync_start = 1440 + 20 /* front porch */,
	.vsync_end   = 1440 + 20 + 4 /* sync_len */,
	.vtotal      = 1440 + 20 + 4 + 12 /* back porch */,
	.vrefresh    = 60,
	.clock       = 75276, /* kHz */
	.flags       = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 65,
	.height_mm   = 130,
};


static inline struct jh057n *panel_to_jh057n(struct drm_panel *panel)
{
	return container_of(panel, struct jh057n, panel);
}

static void jh057n_dcs_write_buf(struct jh057n *ctx, const void *data,
				   size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (mipi_dsi_generic_write(dsi, data, len) < 0)
		DRM_DEV_ERROR(ctx->dev, "mipi dsi dcs write buffer failed\n");
}

#define dcs_write_seq(ctx, seq...)			\
({							\
	static const u8 d[] = { seq };			\
	jh057n_dcs_write_buf(ctx, d, ARRAY_SIZE(d));	\
})

static int jh057n_init_sequence(struct jh057n *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct device *dev = ctx->dev;
	int ret;

	msleep(200);

	/* Enable user command */
	dcs_write_seq(ctx, ST7703_CMD_SETEXTC, /* 3 */
		      0xF1, 0x12, 0x83);
	/* 6 params in ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETMIPI, /* 27 */
		      0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25,
		      0x00, 0x91, 0x0A, 0x00, 0x00, 0x02, 0x4F, 0x11,
		      0x00, 0x00, 0x37);
	/* 6 params in ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETPOWER_EXT, /* 4 */
		      0x76, 0x22, 0x20, 0x03);
	/* 4 params in ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETRGBIF, /* 10 */
		      0x10, 0x10, 0x05, 0x05, 0x03, 0xFF, 0x00, 0x00,
		      0x00, 0x00);
	/* 8 params in ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETSCR, /* 9 */
		      0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x08, 0x70,
		      0x00);
	/* -1.6V & + 1.9V */
	dcs_write_seq(ctx, ST7703_CMD_SETVDC, 0x4E);
	/* SS_PANEL, REV_PANEL, BGR_PANEL */
	dcs_write_seq(ctx, ST7703_CMD_SETPANEL, 0x0B);
	/* 2 params in ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETCYC, 0x80);
	/* weird values, e.g. a 720 panel has BIT(1) 3rd param */
	dcs_write_seq(ctx, ST7703_CMD_SETDISP, 0xF0, 0x12, 0x30);
	dcs_write_seq(ctx, ST7703_CMD_SETEQ, /* 14 */
		      0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B, 0x00, 0x00,
		      0x00, 0x00, 0xFF, 0x00, 0xC0, 0x10);
	dcs_write_seq(ctx, ST7703_CMD_SETPOWER, /* 12 */
		      0x54, 0x00, 0x1E, 0x1E, 0x77, 0xF1, 0xFF, 0xFF,
		      0xCC, 0xCC, 0x77, 0x77);
	/* setbgp is different from our first data set*/
	dcs_write_seq(ctx, ST7703_CMD_SETBGP, 0x08, 0x08);

	mdelay(100);
	/* setvcom is different from our first data set*/
	dcs_write_seq(ctx, ST7703_CMD_SETVCOM, 0x3F, 0x3F);
	/* undocumented */
	dcs_write_seq(ctx, 0xBF, 0x02, 0x11, 0x00);
	dcs_write_seq(ctx, ST7703_CMD_SETGIP1, /* 63 */
		      0x82, 0x10, 0x06, 0x05, 0x9E, 0x0A, 0xA5, 0x12,
		      0x31, 0x23, 0x37, 0x83, 0x04, 0xBC, 0x27, 0x38,
		      0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x00,
		      0x03, 0x00, 0x00, 0x00, 0x75, 0x75, 0x31, 0x88,
		      0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x88, 0x64,
		      0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
		      0x02, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	/* 39 parameters accordin to ST7703 docs */
	dcs_write_seq(ctx, ST7703_CMD_SETGIP2, /* 61 */
		      0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x02, 0x46, 0x02, 0x88,
		      0x88, 0x88, 0x88, 0x88, 0x88, 0x64, 0x88, 0x13,
		      0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
		      0x75, 0x88, 0x23, 0x14, 0x00, 0x00, 0x02, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x0A,
		      0xA5, 0x00, 0x00, 0x00, 0x00);
	dcs_write_seq(ctx, ST7703_CMD_SETGAMMA, /* 34 */
		      0x00, 0x09, 0x0E, 0x29, 0x2D, 0x3C, 0x41, 0x37,
		      0x07, 0x0B, 0x0D, 0x10, 0x11, 0x0F, 0x10, 0x11,
		      0x18, 0x00, 0x09, 0x0E, 0x29, 0x2D, 0x3C, 0x41,
		      0x37, 0x07, 0x0B, 0x0D, 0x10, 0x11, 0x0F, 0x10,
		      0x11, 0x18);

	msleep(78); /* docs say nothing here */

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode");
		return ret;
	}
	msleep(150); /* docs say 120ms */

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(120); /* docs say 5ms, vendor uses 120 ms */

	DRM_DEV_DEBUG_DRIVER (dev, "Panel init sequence done");
	return 0;
}

static int jh057n_disable(struct drm_panel *panel)
{
	struct jh057n *ctx = panel_to_jh057n(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->enabled)
		return 0; /* This is not an issue so we return 0 here */

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(120);

	ctx->enabled = false;

	return 0;
}

static int jh057n_unprepare(struct drm_panel *panel)
{
	struct jh057n *ctx = panel_to_jh057n(panel);

	if (!ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(20, 40); /* docs say 20 usec */
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(140); /* docs say 120 msec */
	} else {
		DRM_DEV_DEBUG_DRIVER(ctx->dev, "No reset gpio found");
		return -EINVAL;
	}

	ctx->prepared = false;

	return 0;
}

static int jh057n_prepare(struct drm_panel *panel)
{
	struct jh057n *ctx = panel_to_jh057n(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		DRM_DEV_DEBUG_DRIVER(ctx->dev, "Resetting the panel.");
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(20, 40); /* docs say 20 usec */
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(140); /* docs say 120 msec */
	} else {
		DRM_DEV_DEBUG_DRIVER(ctx->dev, "No reset gpio found");
		return -EINVAL;
	}

	ret = jh057n_init_sequence(ctx);
	if (ret) {
		DRM_DEV_ERROR(ctx->dev, "Panel init sequence failed");
		return ret;
	}

	ctx->prepared = true;

	return 0;
}

static int jh057n_enable(struct drm_panel *panel)
{
	struct jh057n *ctx = panel_to_jh057n(panel);

	ctx->enabled = true;

	return 0;
}

static int jh057n_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	int ret;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u",
			  default_mode.hdisplay, default_mode.vdisplay,
			  default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.height_mm = mode->height_mm;

	ret = drm_display_info_set_bus_formats(&panel->connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static const struct drm_panel_funcs jh057n_drm_funcs = {
	.disable   = jh057n_disable,
	.unprepare = jh057n_unprepare,
	.prepare   = jh057n_prepare,
	.enable    = jh057n_enable,
	.get_modes = jh057n_get_modes,
};

static int jh057n_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct jh057n *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio");
		return PTR_ERR(ctx->reset_gpio);
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO         /* mdss-dsi-panel-type */
		/* Vendor says panel does not support burst mode,
		   but mdss-dsi-traffic-mode says the opposite */
		/* | MIPI_DSI_MODE_VIDEO_BURST */
		| MIPI_DSI_MODE_VIDEO_SYNC_PULSE
		/* the st7703 supports LPM and HSM */
		| MIPI_DSI_MODE_LPM
		/* allow to shut down serial clock */
		/* | MIPI_DSI_CLOCK_NON_CONTINUOUS */
		;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &jh057n_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "mipi_dsi_attach failed. Is host ready?");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	DRM_INFO(DRV_NAME "_panel %ux%u@%u %ubpp dsi %udl - ready",
		 default_mode.hdisplay, default_mode.vdisplay,
		 default_mode.vrefresh,
		 mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	return 0;
}

static int jh057n_remove(struct mipi_dsi_device *dsi)
{
	struct jh057n *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id jh057n_of_match[] = {
	{ .compatible = "rocktech,jh057n00900" },
	{ }
};
MODULE_DEVICE_TABLE(of, jh057n_of_match);

static struct mipi_dsi_driver jh057n_driver = {
	.probe  = jh057n_probe,
	.remove = jh057n_remove,
	.driver = {
		.name = DRV_NAME "_panel",
		.of_match_table = jh057n_of_match,
	},
};
module_mipi_dsi_driver(jh057n_driver);

MODULE_AUTHOR("Guido Günther <agx@sigxcpu.org>");
MODULE_DESCRIPTION("DRM driver for Rocktech JH057N00900IPI DSI panel");
MODULE_LICENSE("GPL v2");
