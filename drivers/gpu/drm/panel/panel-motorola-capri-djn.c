// SPDX-License-Identifier: GPL-2.0-only
/*
 * DJN NT36525B 6.52-inch panel on Motorola Capri.
 * Timings/commands: Motorola/LineageOS sm6225 vendor DTS, commit
 * b7e7df6ede7464fffad52f1842f24df29bb1a5bf, dsi-panel-mot-djn-nt36525b-
 * 652-hdplus-video.dtsi (Copyright 2020 The Linux Foundation), checked
 * against this handset's live DT. NOT the different vid-v0 command table.
 * Power-management limitation: PMI632 LCDB bias remains firmware-managed. This is
 * not a complete panel power/suspend implementation or an upstream binding.
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct capri_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
	struct regulator *vddio;
	struct mutex lock;
	bool ready;
};

static struct capri_panel *to_capri(struct drm_panel *panel)
{
	return container_of(panel, struct capri_panel, panel);
}

static int capri_panel_prepare(struct drm_panel *panel)
{
	struct capri_panel *p = to_capri(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };
	int ret;

	mutex_lock(&p->lock);
	ret = regulator_enable(p->vddio);
	if (ret)
		goto out;
	/* Active-low GPIO: physical H1/L4/H10/L4/H10 milliseconds. */
	gpiod_set_value_cansleep(p->reset, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(p->reset, 1);
	usleep_range(4000, 4500);
	gpiod_set_value_cansleep(p->reset, 0);
	msleep(10);
	gpiod_set_value_cansleep(p->reset, 1);
	usleep_range(4000, 4500);
	gpiod_set_value_cansleep(p->reset, 0);
	msleep(10);

	p->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x04, 0x05);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x05, 0x2d);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x06, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x68, 0x03, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x51, 0x00, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x53, 0x2c);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x55, 0x01);
	/* Deliberately retain the observed vendor ordering. */
	mipi_dsi_dcs_set_display_on_multi(&ctx);
	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 100);
	ret = ctx.accum_err;
	if (ret) {
		regulator_disable(p->vddio);
		dev_err(panel->dev, "DJN panel initialization failed: %d\n", ret);
	} else {
		p->ready = true;
		dev_info(panel->dev, "DJN NT36525B prepared, 720x1600@60\n");
	}
out:
	mutex_unlock(&p->lock);
	return ret;
}

static int capri_panel_unprepare(struct drm_panel *panel)
{
	struct capri_panel *p = to_capri(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = p->dsi };

	mutex_lock(&p->lock);
	p->ready = false;
	p->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	mipi_dsi_dcs_set_display_off_multi(&ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&ctx);
	mipi_dsi_msleep(&ctx, 60);
	p->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	/* Vendor keeps reset deasserted; bias and shared VDDIO remain on. */
	regulator_disable(p->vddio);
	mutex_unlock(&p->lock);
	return ctx.accum_err;
}

static const struct drm_display_mode capri_mode = {
	.clock = 94332,
	.hdisplay = 720, .hsync_start = 784, .hsync_end = 788, .htotal = 848,
	.vdisplay = 1600, .vsync_start = 1608, .vsync_end = 1610, .vtotal = 1854,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int capri_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode = drm_mode_duplicate(connector->dev, &capri_mode);

	if (!mode)
		return -ENOMEM;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 151;
	connector->display_info.bpc = 8;
	return 1;
}

static const struct drm_panel_funcs capri_panel_funcs = {
	.prepare = capri_panel_prepare,
	.unprepare = capri_panel_unprepare,
	.get_modes = capri_panel_get_modes,
};

static int capri_bl_update(struct backlight_device *bl)
{
	struct capri_panel *p = bl_get_data(bl);
	int ret = 0;

	mutex_lock(&p->lock);
	if (p->ready) {
		p->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_set_display_brightness_large(p->dsi,
						backlight_get_brightness(bl));
		p->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	}
	mutex_unlock(&p->lock);
	return ret;
}

static const struct backlight_ops capri_bl_ops = { .update_status = capri_bl_update };

static int capri_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct capri_panel *p;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW, .brightness = 500, .max_brightness = 1691,
	};
	int ret;

	if (!of_machine_is_compatible("motorola,capri"))
		return -ENODEV;
	p = devm_drm_panel_alloc(dev, struct capri_panel, panel,
				&capri_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(p))
		return PTR_ERR(p);
	mutex_init(&p->lock);
	p->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, p);
	p->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(p->vddio))
		return dev_err_probe(dev, PTR_ERR(p->vddio), "vddio unavailable\n");
	p->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(p->reset))
		return dev_err_probe(dev, PTR_ERR(p->reset), "reset unavailable\n");
	p->panel.prepare_prev_first = true;
	p->panel.backlight = devm_backlight_device_register(dev, dev_name(dev), dev,
							p, &capri_bl_ops, &props);
	if (IS_ERR(p->panel.backlight))
		return PTR_ERR(p->panel.backlight);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;
	ret = devm_drm_panel_add(dev, &p->panel);
	if (ret)
		return ret;
	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id capri_panel_match[] = {
	{ .compatible = "motorola,capri-djn-nt36525b" }, { }
};
MODULE_DEVICE_TABLE(of, capri_panel_match);
static struct mipi_dsi_driver capri_panel_driver = {
	.probe = capri_panel_probe,
	.driver = { .name = "panel-motorola-capri-djn", .of_match_table = capri_panel_match },
};
module_mipi_dsi_driver(capri_panel_driver);
MODULE_DESCRIPTION("Motorola Capri DJN NT36525B panel");
MODULE_LICENSE("GPL");
