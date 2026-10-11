// SPDX-License-Identifier: GPL-2.0-only
/*
 * NT36525B SPI driver for the Motorola Capri mainline RAM port.
 * Protocol derived from Novatek's GPL nova_0flash_mmi driver, Copyright (C)
 * 2010-2020 Novatek, in LineageOS/android_kernel_motorola_sm6225 at
 * b7e7df6ede7464fffad52f1842f24df29bb1a5bf. See vendor-touch/sources.json.
 *
 * A DRM panel follower reloads volatile firmware after panel reset. Full
 * system suspend, gesture wake and ESD recovery are not implemented yet.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <drm/drm_panel.h>

#define NVT_CHUNK 1024
#define NVT_EVENT 0x21c00
#define NVT_CONTACTS 10

struct capri_nvt {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct drm_panel_follower follower;
	const struct firmware *fw;
	struct mutex state_lock;
	bool irq_enabled, stopped;
	u8 *tx, *rx, *verify;
	unsigned long irqs, frames, contacts, errors, fw_loads;
};

struct capri_partition {
	u32 address, offset, size, crc;
};

static int capri_transfer(struct capri_nvt *ts, size_t length, bool receive)
{
	struct spi_transfer transfer = {
		.tx_buf = ts->tx,
		.rx_buf = receive ? ts->rx : NULL,
		.len = length,
	};

	return spi_sync_transfer(ts->spi, &transfer, 1);
}

static int capri_page(struct capri_nvt *ts, u32 address)
{
	ts->tx[0] = 0xff;
	ts->tx[1] = address >> 15;
	ts->tx[2] = address >> 7;
	return capri_transfer(ts, 3, false);
}

static int capri_write(struct capri_nvt *ts, u32 address, const u8 *data, size_t length)
{
	int ret;

	if (!length || length > NVT_CHUNK)
		return -EINVAL;
	ret = capri_page(ts, address);
	if (ret)
		return ret;
	ts->tx[0] = (address & 0x7f) | 0x80;
	memcpy(ts->tx + 1, data, length);
	return capri_transfer(ts, length + 1, false);
}

static int capri_read(struct capri_nvt *ts, u32 address, u8 *data, size_t length)
{
	int ret;

	if (!length || length > NVT_CHUNK)
		return -EINVAL;
	ret = capri_page(ts, address);
	if (ret)
		return ret;
	memset(ts->tx, 0, length + 2);
	ts->tx[0] = address & 0x7f;
	ret = capri_transfer(ts, length + 2, true);
	if (!ret)
		memcpy(data, ts->rx + 2, length);
	return ret;
}

static int capri_byte(struct capri_nvt *ts, u32 address, u8 value)
{
	return capri_write(ts, address, &value, 1);
}

static int capri_reset(struct capri_nvt *ts)
{
	int ret = capri_byte(ts, 0x3f0fe, 0x69);

	if (ret)
		return ret;
	usleep_range(5000, 6000);
	return capri_byte(ts, 0x3f310, 0);
}

static int capri_identify(struct capri_nvt *ts)
{
	u8 id[6];
	int ret, attempt;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = capri_reset(ts);
		if (ret)
			return ret;
		ret = capri_read(ts, 0x1f64e, id, sizeof(id));
		if (ret)
			return ret;
		if (id[0] == 0x0b && id[3] == 0x25 && id[4] == 0x65 && id[5] == 3) {
			dev_info(&ts->spi->dev, "NT36525B identity %*ph\n", 6, id);
			return 0;
		}
		usleep_range(10000, 11000);
	}
	return dev_err_probe(&ts->spi->dev, -ENODEV, "Unexpected touch identity %*ph\n", 6, id);
}

/* Validate the complete image before the first SRAM write. */
static int capri_parse(const struct firmware *fw, struct capri_partition **out)
{
	struct capri_partition *parts;
	u32 end, count, i, j;
	const u8 *p;

	if (fw->size < 0x30)
		return -EINVAL;
	end = get_unaligned_le32(fw->data);
	if (end < 0x30 || end > min_t(size_t, fw->size, 4096) ||
	    (end - 0x30) % 16 || (fw->data[0x28] & 0x10))
		return -EINVAL;
	count = 2 + (end - 0x30) / 16;
	parts = kcalloc(count, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;
	for (i = 0; i < count; i++) {
		if (i < 2) {
			p = fw->data + i * 12;
			parts[i].offset = get_unaligned_le32(p);
			parts[i].address = get_unaligned_le32(p + 4);
			parts[i].size = get_unaligned_le32(p + 8);
			parts[i].crc = get_unaligned_le32(fw->data + 0x18 + i * 4);
			if (!parts[i].size || parts[i].size >= 65536)
				goto invalid;
		} else {
			p = fw->data + 0x30 + (i - 2) * 16;
			parts[i].address = get_unaligned_le32(p);
			parts[i].size = get_unaligned_le32(p + 4);
			parts[i].offset = get_unaligned_le32(p + 8);
			parts[i].crc = get_unaligned_le32(p + 12);
		}
		if (!parts[i].size)
			continue;
		if (parts[i].offset >= fw->size || parts[i].size >= fw->size - parts[i].offset ||
		    parts[i].address >= 0x30000 || parts[i].size >= 0x30000 - parts[i].address)
			goto invalid;
		for (j = 0; j < i; j++) {
			if (parts[j].size && parts[i].address <= parts[j].address + parts[j].size &&
			    parts[j].address <= parts[i].address + parts[i].size)
				goto invalid;
		}
	}
	*out = parts;
	return count;
invalid:
	kfree(parts);
	return -EINVAL;
}

static int capri_crc_bank(struct capri_nvt *ts, struct capri_partition *p,
			  u32 dest, u32 length, u32 checksum)
{
	u8 value[4];
	int ret;

	put_unaligned_le32(p->address, value);
	ret = capri_write(ts, dest, value, 3);
	if (ret)
		return ret;
	put_unaligned_le16(p->size, value);
	ret = capri_write(ts, length, value, 2);
	if (ret)
		return ret;
	put_unaligned_le32(p->crc, value);
	return capri_write(ts, checksum, value, 4);
}

static int capri_load(struct capri_nvt *ts, const struct firmware *fw)
{
	struct capri_partition *parts = NULL;
	u32 i, start, length, written = 0;
	u8 value, state[5], info[16];
	int ret, count, attempt;

	count = capri_parse(fw, &parts);
	if (count < 0)
		return count;
	ret = capri_reset(ts);
	if (ret)
		goto out;
	for (i = 0; i < count; i++) {
		if (!parts[i].size)
			continue;
		for (start = 0; start <= parts[i].size; start += length) {
			length = min_t(u32, NVT_CHUNK, parts[i].size + 1 - start);
			ret = capri_write(ts, parts[i].address + start,
					  fw->data + parts[i].offset + start, length);
			if (ret)
				goto out;
			ret = capri_read(ts, parts[i].address + start, ts->verify, length);
			if (ret)
				goto out;
			if (memcmp(ts->verify, fw->data + parts[i].offset + start, length)) {
				ret = -EILSEQ;
				goto out;
			}
			written += length;
		}
	}
	ret = capri_crc_bank(ts, &parts[0], 0x3f128, 0x3f118, 0x3f100);
	if (ret)
		goto out;
	ret = capri_crc_bank(ts, &parts[1], 0x3f12c, 0x3f130, 0x3f104);
	if (ret)
		goto out;
	ret = capri_read(ts, 0x3f30e, &value, 1);
	if (ret)
		goto out;
	ret = capri_byte(ts, 0x3f30e, value | 0x80);
	if (ret)
		goto out;
	ret = capri_byte(ts, NVT_EVENT + 0x60, 0);
	if (ret)
		goto out;
	ret = capri_byte(ts, NVT_EVENT + 0x50, 0xae);
	if (ret)
		goto out;
	ret = capri_byte(ts, 0x3f10d, 1);
	if (ret)
		goto out;
	for (attempt = 0; attempt < 60; attempt++) {
		usleep_range(10000, 11000);
		ret = capri_read(ts, NVT_EVENT + 0x60, state, sizeof(state));
		if (ret)
			goto out;
		if (state[0] >= 0xa1 && state[0] <= 0xaf)
			break;
	}
	if (attempt == 60) {
		ret = -ETIMEDOUT;
		goto out;
	}
	/* Running firmware reuses the bootloader CRC result registers. Do not
	 * compare those after startup; all payload bytes were read back above.
	 */
	ret = capri_read(ts, NVT_EVENT + 0x78, info, sizeof(info));
	if (ret)
		goto out;
	if ((u8)(info[0] + info[1]) != 255 || get_unaligned_be16(info + 4) != 720 ||
	    get_unaligned_be16(info + 6) != 1600) {
		ret = -EINVAL;
		goto out;
	}
	dev_info(&ts->spi->dev, "Firmware %u running, 720x1600, %u SRAM bytes verified, state %02x\n",
		 info[0], written, state[0]);
out:
	kfree(parts);
	return ret;
}

static irqreturn_t capri_irq(int irq, void *data)
{
	struct capri_nvt *ts = data;
	u8 packet[65], checksum = 0;
	u32 i, slot, x, y;
	unsigned long seen = 0;
	int ret;

	ts->irqs++;
	ret = capri_read(ts, NVT_EVENT, packet, sizeof(packet));
	if (ret)
		goto error;
	for (i = 0; i < sizeof(packet); i++)
		checksum += packet[i];
	if (checksum)
		goto error;
	ts->frames++;
	for (i = 0; i < NVT_CONTACTS; i++) {
		u8 *p = packet + i * 6;

		slot = p[0] >> 3;
		if (!slot || slot > NVT_CONTACTS || ((p[0] & 7) != 1 && (p[0] & 7) != 2))
			continue;
		slot--;
		x = (p[1] << 4) | (p[3] >> 4);
		y = (p[2] << 4) | (p[3] & 15);
		if (x >= 720 || y >= 1600 || test_and_set_bit(slot, &seen))
			continue;
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		ts->contacts++;
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	return IRQ_HANDLED;
error:
	ts->errors++;
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	dev_warn_ratelimited(&ts->spi->dev, "Touch report error (SPI %d, checksum %u)\n", ret, checksum);
	return IRQ_HANDLED;
}

static ssize_t touch_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct capri_nvt *ts = dev_get_drvdata(dev);

	return sysfs_emit(buf, "irqs=%lu frames=%lu contacts=%lu errors=%lu fw_loads=%lu active=%u\n",
			 READ_ONCE(ts->irqs), READ_ONCE(ts->frames),
			 READ_ONCE(ts->contacts), READ_ONCE(ts->errors),
			 READ_ONCE(ts->fw_loads), READ_ONCE(ts->irq_enabled));
}
static DEVICE_ATTR_RO(touch_stats);
static struct attribute *capri_attrs[] = { &dev_attr_touch_stats.attr, NULL };
static const struct attribute_group capri_group = { .attrs = capri_attrs };

/* state_lock protects lifecycle state; disable_irq synchronizes SPI/input IRQ
 * activity before firmware access. The IRQ handler never takes state_lock. */
static void capri_quiesce(struct capri_nvt *ts)
{
	if (ts->irq_enabled) {
		disable_irq(ts->spi->irq);
		ts->irq_enabled = false;
		input_mt_sync_frame(ts->input);
		input_sync(ts->input);
	}
}

static int capri_start(struct capri_nvt *ts)
{
	int ret;

	mutex_lock(&ts->state_lock);
	if (ts->stopped) {
		ret = -ESHUTDOWN;
		goto out;
	}
	capri_quiesce(ts);
	ret = capri_identify(ts);
	if (!ret)
		ret = capri_load(ts, ts->fw);
	if (!ret) {
		ts->fw_loads++;
		ts->irq_enabled = true;
		enable_irq(ts->spi->irq);
	}
out:
	mutex_unlock(&ts->state_lock);
	return ret;
}

static int capri_panel_prepared(struct drm_panel_follower *follower)
{
	return capri_start(container_of(follower, struct capri_nvt, follower));
}

static int capri_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct capri_nvt *ts = container_of(follower, struct capri_nvt, follower);

	mutex_lock(&ts->state_lock);
	capri_quiesce(ts);
	mutex_unlock(&ts->state_lock);
	return 0;
}

static const struct drm_panel_follower_funcs capri_follower_funcs = {
	.panel_prepared = capri_panel_prepared,
	.panel_unpreparing = capri_panel_unpreparing,
};

static void capri_release_fw(void *data)
{
	release_firmware(data);
}

static int capri_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const char *filename;
	struct gpio_desc *reset;
	struct capri_nvt *ts;
	int ret;

	if (!of_machine_is_compatible("motorola,capri"))
		return -ENODEV;
	if (spi->mode != SPI_MODE_0 || spi_get_chipselect(spi, 0) != 0 || spi->max_speed_hz > 9600000)
		return dev_err_probe(dev, -EINVAL, "Expected Capri SPI mode 0 / CS0 / <=9.6 MHz\n");
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	mutex_init(&ts->state_lock);
	ts->tx = devm_kmalloc(dev, NVT_CHUNK + 2, GFP_KERNEL);
	ts->rx = devm_kmalloc(dev, NVT_CHUNK + 2, GFP_KERNEL);
	ts->verify = devm_kmalloc(dev, NVT_CHUNK, GFP_KERNEL);
	if (!ts->tx || !ts->rx || !ts->verify)
		return -ENOMEM;
	spi_set_drvdata(spi, ts);
	reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Touch reset GPIO\n");
	usleep_range(10000, 11000);
	ret = device_property_read_string(dev, "firmware-name", &filename);
	if (ret)
		return ret;
	ret = request_firmware(&ts->fw, filename, dev);
	if (ret)
		return dev_err_probe(dev, ret, "Touch firmware %s\n", filename);
	ret = devm_add_action_or_reset(dev, capri_release_fw, (void *)ts->fw);
	if (ret)
		return ret;
	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;
	ts->input->name = "Capri NT36525B Touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 719, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 1599, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	ret = input_mt_init_slots(ts->input, NVT_CONTACTS, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, capri_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "Touch IRQ\n");
	ret = input_register_device(ts->input);
	if (ret)
		return ret;
	ret = devm_device_add_group(dev, &capri_group);
	if (ret)
		return ret;
	if (drm_is_panel_follower(dev)) {
		ts->follower.funcs = &capri_follower_funcs;
		return devm_drm_panel_add_follower(dev, &ts->follower);
	}
	return capri_start(ts);
}

static const struct of_device_id capri_match[] = {
	{ .compatible = "novatek,nt36525b-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, capri_match);

static const struct spi_device_id capri_ids[] = {
	{ "nt36525b-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, capri_ids);

static void capri_stop(struct spi_device *spi)
{
	struct capri_nvt *ts = spi_get_drvdata(spi);

	/* Quiesce the thread before devres unregisters/frees the input device. */
	mutex_lock(&ts->state_lock);
	ts->stopped = true;
	capri_quiesce(ts);
	mutex_unlock(&ts->state_lock);
}

static struct spi_driver capri_driver = {
	.driver = { .name = "novatek-nt36525b-capri", .of_match_table = capri_match },
	.probe = capri_probe,
	.remove = capri_stop,
	.shutdown = capri_stop,
	.id_table = capri_ids,
};
module_spi_driver(capri_driver);
MODULE_DESCRIPTION("Motorola Capri NT36525B SPI touchscreen");
MODULE_LICENSE("GPL");
