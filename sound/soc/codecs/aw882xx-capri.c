// SPDX-License-Identifier: GPL-2.0-only
/* Capri speaker path for Awinic ID 0x1852.
 * Register protocol and trim arithmetic from Motorola's pinned aw882xx driver.
 * Only this handset's checked speaker profile is accepted. Calibration fuses
 * are read, never programmed. Gain is deliberately limited to -18 dB.
 */
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AW_ID 0x00
#define AW_STATUS 0x01
#define AW_POWER 0x04
#define AW_MUTE 0x05
#define AW_I2S 0x06
#define AW_GAIN 0x0c
#define AW_FAULTS (BIT(15) | BIT(14) | BIT(11) | BIT(10) | BIT(3) | BIT(1))

struct capri_amp {
	struct device *dev;
	struct regmap *map;
	struct gpio_desc *reset;
	struct mutex lock;
	struct delayed_work monitor;
	bool loaded, active;
	unsigned int volume, starts;
};

static int capri_amp_gain(struct capri_amp *amp, unsigned int attenuation)
{
	unsigned int code = (attenuation / 6) * 16 + (attenuation % 6) * 2;
	return regmap_update_bits(amp->map, AW_GAIN, 0xff00, code << 8);
}

/* Caller holds lock. Always attempt power-down even when muting fails. */
static int capri_amp_off(struct capri_amp *amp)
{
	int ret = regmap_update_bits(amp->map, AW_MUTE, BIT(4), BIT(4));
	int power_ret = regmap_update_bits(amp->map, AW_POWER, BIT(0), BIT(0));
	amp->active = false;
	return ret ? ret : power_ret;
}

static void capri_amp_monitor(struct work_struct *work)
{
	struct capri_amp *amp = container_of(to_delayed_work(work), struct capri_amp, monitor);
	unsigned int status, temperature, voltage;
	int ret;

	mutex_lock(&amp->lock);
	if (!amp->active)
		goto out;
	ret = regmap_read(amp->map, AW_STATUS, &status);
	ret = ret ?: regmap_read(amp->map, 0x13, &temperature);
	ret = ret ?: regmap_read(amp->map, 0x12, &voltage);
	if (ret || (status & AW_FAULTS) || (status & 0x11) != 0x11 ||
	    sign_extend32(temperature & 0x3ff, 9) > 75 ||
	    ((voltage & 0x3ff) * 6025 / 1023) < 3000) {
		dev_err(amp->dev, "speaker protection stopped playback, read=%d\n", ret);
		capri_amp_off(amp);
	} else {
		schedule_delayed_work(&amp->monitor, HZ);
	}
out:
	mutex_unlock(&amp->lock);
}

static int capri_amp_load(struct capri_amp *amp)
{
	const struct firmware *fw;
	unsigned int reg, value, ef_i, ef_v;
	int ret, current_trim, voltage_trim, calibration;
	size_t offset;

	ret = request_firmware(&fw, "aw882xx_spk_reg.bin", amp->dev);
	if (ret)
		return ret;
	if (fw->size != 144 || (crc32_le(~0U, fw->data, fw->size) ^ ~0U) != 0x7b0f9fcb) {
		ret = -EINVAL;
		goto release;
	}
	for (offset = 0; offset < fw->size; offset += 4) {
		reg = get_unaligned_le16(fw->data + offset);
		value = get_unaligned_le16(fw->data + offset + 2);
		/* The validated blob contains no efuse programming registers. */
		if (reg > 0x70) { ret = -EINVAL; goto release; }
		if (reg == AW_POWER) value |= BIT(0);
		if (reg == AW_MUTE) value |= BIT(4);
		if (reg == AW_GAIN) value = (value & 0xff) | 0xf000; /* -90 dB */
		ret = regmap_write(amp->map, reg, value);
		if (ret)
			goto release;
	}
	ret = regmap_read(amp->map, 0x7a, &ef_i);
	ret = ret ?: regmap_read(amp->map, 0x78, &ef_v);
	if (ret)
		goto release;
	current_trim = 1000 + sign_extend32(ef_i & 0x3ff, 9);
	voltage_trim = 1000 + sign_extend32(ef_v & 0x3ff, 9);
	calibration = 8192 * current_trim / voltage_trim;
	if (calibration < 1 || calibration > 0xffff) { ret = -ERANGE; goto release; }
	ret = regmap_write(amp->map, 0x56, calibration);
	if (!ret) {
		amp->loaded = true;
		dev_info(amp->dev, "checked handset profile loaded, eFuse trim 0x%x, gain limited to -18 dB\n", calibration);
	}
release:
	release_firmware(fw);
	if (ret)
		capri_amp_off(amp);
	return ret;
}

static int capri_amp_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct capri_amp *amp = snd_soc_component_get_drvdata(dai->component);
	int ret;
	if (params_rate(params) != 48000 || params_width(params) != 16)
		return -EINVAL;
	mutex_lock(&amp->lock);
	ret = amp->loaded ? 0 : capri_amp_load(amp);
	/* Vendor 48kHz / 16-bit clock values; preserve channel and RX enable. */
	ret = ret ?: regmap_update_bits(amp->map, 0x66, BIT(14), BIT(14));
	ret = ret ?: regmap_update_bits(amp->map, AW_I2S, 0xcf, 8);
	mutex_unlock(&amp->lock);
	return ret;
}

static int capri_amp_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

static int capri_amp_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct capri_amp *amp = snd_soc_component_get_drvdata(dai->component);
	unsigned int status = 0, level;
	int ret, attempt;
	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	if (mute)
		cancel_delayed_work_sync(&amp->monitor);
	mutex_lock(&amp->lock);
	if (mute) { ret = capri_amp_off(amp); goto out; }
	if (!amp->loaded) { ret = -EINVAL; goto out; }
	ret = capri_amp_gain(amp, 90);
	ret = ret ?: regmap_update_bits(amp->map, AW_POWER, BIT(0), 0);
	if (ret)
		goto fail;
	for (attempt = 0; attempt < 25; attempt++) {
		usleep_range(2000, 2500);
		ret = regmap_read(amp->map, AW_STATUS, &status);
		if (ret)
			goto fail;
		if ((status & 0x11) == 0x11 && !(status & AW_FAULTS))
			break;
	}
	if (attempt == 25) {
		dev_err(amp->dev, "I2S clock / protection not ready: status=0x%x\n", status);
		ret = -EIO;
		goto fail;
	}
	ret = regmap_update_bits(amp->map, AW_MUTE, BIT(4), 0);
	for (level = 90; !ret && level >= 30 - amp->volume; level--) {
		ret = capri_amp_gain(amp, level);
		usleep_range(2000, 2500);
	}
	if (ret)
		goto fail;
	amp->active = true;
	amp->starts++;
	schedule_delayed_work(&amp->monitor, HZ);
	dev_info(amp->dev, "speaker stream started, status=0x%x, attenuation=%u dB\n", status, 30 - amp->volume);
	goto out;
fail:
	capri_amp_off(amp);
out:
	mutex_unlock(&amp->lock);
	return ret;
}

static int capri_volume_get(struct snd_kcontrol *k, struct snd_ctl_elem_value *v)
{
	struct capri_amp *amp = snd_soc_component_get_drvdata(snd_kcontrol_chip(k));
	mutex_lock(&amp->lock);
	v->value.integer.value[0] = amp->volume;
	mutex_unlock(&amp->lock);
	return 0;
}

static int capri_volume_put(struct snd_kcontrol *k, struct snd_ctl_elem_value *v)
{
	struct capri_amp *amp = snd_soc_component_get_drvdata(snd_kcontrol_chip(k));
	long value = v->value.integer.value[0];
	int ret = 0;
	if (value < 0 || value > 12)
		return -EINVAL;
	mutex_lock(&amp->lock);
	if (amp->volume != value) {
		if (amp->active)
			ret = capri_amp_gain(amp, 30 - value);
		if (!ret) { amp->volume = value; ret = 1; }
	}
	mutex_unlock(&amp->lock);
	return ret;
}

static const DECLARE_TLV_DB_SCALE(capri_volume_tlv, -3000, 100, 0);
static const struct snd_kcontrol_new capri_controls[] = {
	SOC_SINGLE_EXT_TLV("Speaker Playback Volume", SND_SOC_NOPM, 0, 12, 0,
			   capri_volume_get, capri_volume_put, capri_volume_tlv),
};
static const struct snd_soc_dapm_widget capri_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Speaker Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};
static const struct snd_soc_dapm_route capri_routes[] = { { "OUT", NULL, "DAC" } };
static const struct snd_soc_component_driver capri_component = {
	.controls = capri_controls, .num_controls = ARRAY_SIZE(capri_controls),
	.dapm_widgets = capri_widgets, .num_dapm_widgets = ARRAY_SIZE(capri_widgets),
	.dapm_routes = capri_routes, .num_dapm_routes = ARRAY_SIZE(capri_routes),
};
static const struct snd_soc_dai_ops capri_dai_ops = {
	.set_fmt = capri_amp_set_fmt, .hw_params = capri_amp_hw_params, .mute_stream = capri_amp_mute,
	.no_capture_mute = 1,
};
static struct snd_soc_dai_driver capri_dai = {
	.name = "capri-amp-aif",
	.playback = { .stream_name = "Speaker Playback", .channels_min = 1, .channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000, .formats = SNDRV_PCM_FMTBIT_S16_LE },
	.ops = &capri_dai_ops,
};
static const struct regmap_config capri_regmap = {
	.reg_bits = 8, .val_bits = 16, .max_register = 0x7c,
	.val_format_endian = REGMAP_ENDIAN_BIG, .cache_type = REGCACHE_NONE,
};

static void capri_amp_cleanup(void *data)
{
	struct capri_amp *amp = data;
	cancel_delayed_work_sync(&amp->monitor);
	mutex_lock(&amp->lock);
	capri_amp_off(amp);
	gpiod_set_value_cansleep(amp->reset, 1);
	mutex_unlock(&amp->lock);
}

static int capri_amp_probe(struct i2c_client *client)
{
	struct capri_amp *amp;
	unsigned int id;
	int ret;
	amp = devm_kzalloc(&client->dev, sizeof(*amp), GFP_KERNEL);
	if (!amp)
		return -ENOMEM;
	amp->dev = &client->dev;
	mutex_init(&amp->lock);
	INIT_DELAYED_WORK(&amp->monitor, capri_amp_monitor);
	amp->map = devm_regmap_init_i2c(client, &capri_regmap);
	if (IS_ERR(amp->map))
		return PTR_ERR(amp->map);
	amp->reset = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(amp->reset))
		return dev_err_probe(&client->dev, PTR_ERR(amp->reset), "reset GPIO\n");
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(amp->reset, 0);
	usleep_range(2000, 2500);
	i2c_set_clientdata(client, amp);
	ret = devm_add_action_or_reset(&client->dev, capri_amp_cleanup, amp);
	if (ret)
		return ret;
	ret = regmap_read(amp->map, AW_ID, &id);
	if (ret || id != 0x1852)
		return dev_err_probe(&client->dev, ret ?: -ENODEV, "expected amplifier ID 0x1852\n");
	ret = capri_amp_off(amp);
	if (ret)
		return ret;
	dev_info(&client->dev, "Awinic 0x1852 identified and muted\n");
	return devm_snd_soc_register_component(&client->dev, &capri_component, &capri_dai, 1);
}

static void capri_amp_shutdown(struct i2c_client *client)
{
	capri_amp_cleanup(i2c_get_clientdata(client));
}
static const struct of_device_id capri_amp_of[] = {
	{ .compatible = "motorola,capri-aw882xx" }, {}
};
MODULE_DEVICE_TABLE(of, capri_amp_of);
static struct i2c_driver capri_amp_driver = {
	.driver = { .name = "capri-aw882xx", .of_match_table = capri_amp_of },
	.probe = capri_amp_probe, .shutdown = capri_amp_shutdown,
};
module_i2c_driver(capri_amp_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Capri Awinic 0x1852 speaker codec");
