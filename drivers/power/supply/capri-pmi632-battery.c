// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only battery reporting for the Moto G10's PMI632 QGauge and SMB5.
 * Register layout/scales: Qualcomm's downstream qg-reg.h, qg-util.c and
 * smb5-reg.h (2018-2020 The Linux Foundation). See battery source-pins.json.
 * QG sample decoding follows the GPL-2.0 qcom_qg driver by Danila Tikhonov.
 *
 * No charger, gauge, interrupt or SDAM registers are written. Charging remains
 * under the existing firmware/hardware configuration. State of charge is an
 * estimate: seed from the pack's OCV table, then integrate measured current.
 * Correct slowly at rest; do not let CPU-load voltage sag move the percentage.
 */
#include <linux/bitfield.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define QG_TYPE                 0x04
#define QG_SUBTYPE              0x05
#define QG_STATUS1              0x08
#define QG_LAST_V_I             0xc0
#define CHGR_STATUS1            0x1006
#define CHGR_STATUS2            0x1007
#define CHGR_STATUS7            0x100d
#define USB_INT_RT              0x1310
#define SAMPLE_MS               5000

struct capri_battery {
	struct device *dev;
	struct regmap *map;
	unsigned int base, current_lsb;
	struct power_supply *battery, *usb;
	struct power_supply_battery_info *info;
	struct delayed_work work;
	struct mutex lock;
	bool valid, initialized;
	int voltage, current_ua, previous_current, present, online, status, health;
	int capacity, rest_samples;
	s64 charge_uas, full_uas, last_ms;
};

static int capri_sample(struct capri_battery *chip)
{
	unsigned int qg_status, usb, state, error, temperature;
	u8 buf[4];
	int ret, voltage, current_ua, capacity;
	s64 now, elapsed, target, step;

	ret = regmap_bulk_read(chip->map, chip->base + QG_LAST_V_I, buf, sizeof(buf));
	if (ret)
		return ret;
	if (get_unaligned_le16(buf) == 0x8000 || get_unaligned_le16(buf + 2) == 0x8000)
		return -ENODATA;
	voltage = div_u64((u64)get_unaligned_le16(buf) * 194637, 1000);
	current_ua = -div_s64((s64)(s16)get_unaligned_le16(buf + 2) * chip->current_lsb, 1000);
	if (voltage < 2500000 || voltage > 4600000)
		return -ERANGE;
	ret = regmap_read(chip->map, chip->base + QG_STATUS1, &qg_status);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, USB_INT_RT, &usb);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, CHGR_STATUS1, &state);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, CHGR_STATUS2, &error);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, CHGR_STATUS7, &temperature);
	if (ret)
		return ret;

	chip->voltage = voltage;
	chip->current_ua = current_ua;
	chip->present = !!(qg_status & BIT(0));
	chip->online = !!(usb & BIT(4));
	state &= GENMASK(2, 0);
	if (!chip->present)
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (!chip->online || current_ua < -20000)
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (state == 5 && voltage >= 4300000)
		chip->status = POWER_SUPPLY_STATUS_FULL;
	else if (current_ua > 20000 && state >= 1 && state <= 4)
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	chip->health = POWER_SUPPLY_HEALTH_GOOD;
	if (temperature & BIT(3))
		chip->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (temperature & BIT(2))
		chip->health = POWER_SUPPLY_HEALTH_COLD;
	else if (error & BIT(1))
		chip->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	now = ktime_to_ms(ktime_get_boottime());
	capacity = power_supply_batinfo_ocv2cap(chip->info, voltage, 25);
	if (capacity < 0)
		return capacity;
	capacity = clamp(capacity, 0, 100);
	target = div_s64(chip->full_uas * capacity, 100);
	if (!chip->initialized) {
		chip->charge_uas = target;
		chip->initialized = true;
	} else {
		elapsed = now - chip->last_ms;
		/* Do not extrapolate across suspend or a long unreadable interval. */
		if (elapsed > 0 && elapsed <= 30000)
			chip->charge_uas += div_s64(((s64)current_ua + chip->previous_current) * elapsed, 2000);
		if (abs(current_ua) < 100000) {
			if (++chip->rest_samples >= 6) {
				step = div_s64(chip->full_uas, 1000);
				chip->charge_uas += clamp_t(s64, target - chip->charge_uas, -step, step);
			}
		} else {
			chip->rest_samples = 0;
		}
	}
	if (chip->status == POWER_SUPPLY_STATUS_FULL)
		chip->charge_uas = chip->full_uas;
	chip->charge_uas = clamp_t(s64, chip->charge_uas, 0, chip->full_uas);
	chip->capacity = div64_s64(chip->charge_uas * 100 + chip->full_uas / 2, chip->full_uas);
	chip->previous_current = current_ua;
	chip->last_ms = now;
	chip->valid = true;
	return 0;
}

static void capri_poll(struct work_struct *work)
{
	struct capri_battery *chip = container_of(to_delayed_work(work), struct capri_battery, work);
	int ret;

	mutex_lock(&chip->lock);
	ret = capri_sample(chip);
	if (ret)
		chip->valid = false;
	mutex_unlock(&chip->lock);
	if (ret)
		dev_warn_ratelimited(chip->dev, "Battery sample unavailable: %d\n", ret);
	power_supply_changed(chip->battery);
	power_supply_changed(chip->usb);
	schedule_delayed_work(&chip->work, msecs_to_jiffies(SAMPLE_MS));
}

static enum power_supply_property capri_properties[] = {
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY, POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN, POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static int capri_get(struct power_supply *psy, enum power_supply_property prop,
		     union power_supply_propval *value)
{
	struct capri_battery *chip = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chip->lock);
	if (!chip->valid) {
		ret = -ENODATA;
		goto out;
	}
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT: value->intval = chip->present; break;
	case POWER_SUPPLY_PROP_STATUS: value->intval = chip->status; break;
	case POWER_SUPPLY_PROP_HEALTH: value->intval = chip->health; break;
	case POWER_SUPPLY_PROP_TECHNOLOGY: value->intval = chip->info->technology; break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: value->intval = chip->voltage; break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: value->intval = chip->current_ua; break;
	case POWER_SUPPLY_PROP_CAPACITY: value->intval = chip->capacity; break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN: value->intval = chip->info->charge_full_design_uah; break;
	case POWER_SUPPLY_PROP_CHARGE_NOW: value->intval = div_s64(chip->charge_uas, 3600); break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN: value->intval = chip->info->voltage_min_design_uv; break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN: value->intval = chip->info->voltage_max_design_uv; break;
	default: ret = -EINVAL;
	}
out:
	mutex_unlock(&chip->lock);
	return ret;
}

static enum power_supply_property capri_usb_properties[] = { POWER_SUPPLY_PROP_ONLINE };

static int capri_usb_get(struct power_supply *psy, enum power_supply_property prop,
			 union power_supply_propval *value)
{
	struct capri_battery *chip = power_supply_get_drvdata(psy);
	int ret = 0;

	if (prop != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;
	mutex_lock(&chip->lock);
	if (!chip->valid)
		ret = -ENODATA;
	else
		value->intval = chip->online;
	mutex_unlock(&chip->lock);
	return ret;
}

static const struct power_supply_desc capri_desc = {
	.name = "capri-battery", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = capri_properties, .num_properties = ARRAY_SIZE(capri_properties),
	.get_property = capri_get,
};
static const struct power_supply_desc capri_usb_desc = {
	.name = "capri-usb", .type = POWER_SUPPLY_TYPE_USB,
	.properties = capri_usb_properties, .num_properties = ARRAY_SIZE(capri_usb_properties),
	.get_property = capri_usb_get,
};

static void capri_release_info(void *data)
{
	struct capri_battery *chip = data;
	power_supply_put_battery_info(chip->battery, chip->info);
}

static void capri_cancel(void *data)
{
	struct capri_battery *chip = data;
	cancel_delayed_work_sync(&chip->work);
}

static int capri_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct capri_battery *chip;
	unsigned int type, subtype;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;
	mutex_init(&chip->lock);
	INIT_DELAYED_WORK(&chip->work, capri_poll);
	chip->map = dev_get_regmap(dev->parent, NULL);
	if (!chip->map)
		return -ENODEV;
	ret = device_property_read_u32(dev, "reg", &chip->base);
	if (ret || chip->base != 0x4800)
		return -EINVAL;
	ret = regmap_read(chip->map, chip->base + QG_TYPE, &type);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, chip->base + QG_SUBTYPE, &subtype);
	if (ret)
		return ret;
	if (type != 0x0d || (subtype != 3 && subtype != 4))
		return dev_err_probe(dev, -ENODEV, "Unexpected QGauge type %x subtype %x\n", type, subtype);
	chip->current_lsb = subtype == 3 ? 152588 : 305176;
	config.drv_data = chip;
	config.fwnode = dev_fwnode(dev);
	chip->battery = devm_power_supply_register(dev, &capri_desc, &config);
	if (IS_ERR(chip->battery))
		return PTR_ERR(chip->battery);
	ret = power_supply_get_battery_info(chip->battery, &chip->info);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, capri_release_info, chip);
	if (ret)
		return ret;
	if (chip->info->charge_full_design_uah <= 0 || !chip->info->ocv_table[0])
		return -EINVAL;
	chip->full_uas = (s64)chip->info->charge_full_design_uah * 3600;
	chip->usb = devm_power_supply_register(dev, &capri_usb_desc, &config);
	if (IS_ERR(chip->usb))
		return PTR_ERR(chip->usb);
	ret = devm_add_action_or_reset(dev, capri_cancel, chip);
	if (ret)
		return ret;
	schedule_delayed_work(&chip->work, 0);
	dev_info(dev, "Read-only QGauge subtype %u; battery percentage is an OCV/current estimate\n", subtype);
	return 0;
}

static const struct of_device_id capri_match[] = {
	{ .compatible = "motorola,capri-pmi632-battery" }, {}
};
MODULE_DEVICE_TABLE(of, capri_match);
static struct platform_driver capri_driver = {
	.probe = capri_probe,
	.driver = { .name = "capri-pmi632-battery", .of_match_table = capri_match },
};
module_platform_driver(capri_driver);
MODULE_DESCRIPTION("Read-only Moto G10 PMI632 battery monitor");
MODULE_LICENSE("GPL");
