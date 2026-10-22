// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery reporting and conservative charging for the Moto G10 PMI632.
 * Register layout/scales: Qualcomm's downstream qg-reg.h, qg-util.c and
 * smb5-reg.h (2018-2020 The Linux Foundation). See battery source-pins.json.
 * QG sample decoding follows the GPL-2.0 qcom_qg driver by Danila Tikhonov.
 *
 * Charger limits follow the Capri JK50 battery data. State of charge is an
 * estimate: seed from the pack's OCV table, then integrate measured current.
 * Correct slowly at rest; do not let CPU-load voltage sag move the percentage.
 */
#include <linux/bitfield.h>
#include <linux/ktime.h>
#include <linux/iio/consumer.h>
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
#define CHGR_ENABLE             0x1042
#define CHGR_CFG2               0x1051
#define CHGR_FCC                0x1061
#define CHGR_FV                 0x1070
#define WD_PET                  0x1643
#define WD_CFG                  0x1651
#define WD_TIMEOUT              0x1653

/* Allows a RAM test to boot with charging inhibited until ADC validation. */
static bool charging = true;
module_param(charging, bool, 0444);
MODULE_PARM_DESC(charging, "Initial charging policy; writable through charge_behaviour");

struct capri_battery {
	struct device *dev;
	struct regmap *map;
	unsigned int base, current_lsb;
	struct power_supply *battery, *usb;
	struct power_supply_battery_info *info;
	struct iio_channel *therm;
	int temperature, behaviour;
	bool charger_ready, thermal_blocked, stopping;
	unsigned int fcc_code, fv_code;
	struct delayed_work work;
	struct mutex lock;
	bool valid, initialized;
	int voltage, current_ua, previous_current, present, online, status, health;
	int capacity, rest_samples;
	s64 charge_uas, full_uas, last_ms;
};

/* Motorola qcom-vadc-common.c, GPL-2.0-only: 10 kOhm NTC, 30 kOhm
 * pull-up, 1.875 V reference. Entries are millivolts and tenths Celsius.
 */
static const struct { int mv, decic; } capri_therm_table[] = {
	{1618,	-400},
	{1593,	-380},
	{1566,	-360},
	{1538,	-340},
	{1508,	-320},
	{1477,	-300},
	{1444,	-280},
	{1410,	-260},
	{1374,	-240},
	{1338,	-220},
	{1300,	-200},
	{1261,	-180},
	{1221,	-160},
	{1181,	-140},
	{1140,	-120},
	{1099,	-100},
	{1057,	-80},
	{1016,	-60},
	{974,	-40},
	{934,	-20},
	{893,	0},
	{853,	20},
	{814,	40},
	{776,	60},
	{738,	80},
	{702,	100},
	{667,	120},
	{633,	140},
	{600,	160},
	{569,	180},
	{539,	200},
	{510,	220},
	{482,	240},
	{456,	260},
	{431,	280},
	{407,	300},
	{384,	320},
	{363,	340},
	{342,	360},
	{323,	380},
	{305,	400},
	{288,	420},
	{272,	440},
	{256,	460},
	{242,	480},
	{228,	500},
	{215,	520},
	{203,	540},
	{192,	560},
	{181,	580},
	{171,	600},
	{162,	620},
	{153,	640},
	{145,	660},
	{137,	680},
	{130,	700},
	{123,	720},
	{116,	740},
	{110,	760},
	{104,	780},
	{99,	800},
	{94,	820},
	{89,	840},
	{84,	860},
	{80,	880},
	{76,	900},
	{72,	920},
	{69,	940},
	{65,	960},
	{62,	980}
};

static int capri_temperature(struct capri_battery *chip)
{
	int uv, mv, i, ret;

	ret = iio_read_channel_processed(chip->therm, &uv);
	if (ret < 0)
		return ret;
	mv = uv / 1000;
	if (mv > capri_therm_table[0].mv ||
	    mv < capri_therm_table[ARRAY_SIZE(capri_therm_table) - 1].mv)
		return -ERANGE;
	for (i = 1; i < ARRAY_SIZE(capri_therm_table); i++) {
		if (mv >= capri_therm_table[i].mv) {
			chip->temperature = capri_therm_table[i - 1].decic +
				(capri_therm_table[i - 1].mv - mv) *
				(capri_therm_table[i].decic - capri_therm_table[i - 1].decic) /
				(capri_therm_table[i - 1].mv - capri_therm_table[i].mv);
			return 0;
		}
	}
	return -ERANGE;
}

static int capri_disable_charging(struct capri_battery *chip)
{
	return regmap_update_bits(chip->map, CHGR_ENABLE, BIT(0), 0);
}

static int capri_write16(struct capri_battery *chip, unsigned int reg, u16 value)
{
	u8 buf[2];

	put_unaligned_be16(value, buf);
	return regmap_bulk_write(chip->map, reg, buf, sizeof(buf));
}

static int capri_charger_init(struct capri_battery *chip)
{
	int ret;
	u16 raw;

	/* Refuse limits outside the tested JK50 configuration. */
	if (chip->info->constant_charge_current_max_ua != 500000 ||
	    chip->info->constant_charge_voltage_max_uv != 4400000 ||
	    chip->info->charge_term_current_ua != 100000)
		return -EINVAL;
	chip->fcc_code = 500000 / 50000;
	chip->fv_code = (4400000 - 3600000) / 10000;
	ret = capri_disable_charging(chip);
	if (ret)
		return ret;
#define CAPRI_WRITE(reg, val) do { \
	ret = regmap_write(chip->map, (reg), (val)); \
	if (ret) \
		return ret; \
} while (0)
#define CAPRI_UPDATE(reg, mask, val) do { \
	ret = regmap_update_bits(chip->map, (reg), (mask), (val)); \
	if (ret) \
		return ret; \
} while (0)
#define CAPRI_WRITE16(reg, val) do { \
	ret = capri_write16(chip, (reg), (val)); \
	if (ret) \
		return ret; \
} while (0)
	CAPRI_WRITE(CHGR_FCC, chip->fcc_code);
	CAPRI_WRITE(CHGR_FV, chip->fv_code);
	CAPRI_UPDATE(0x1060, GENMASK(2, 0), 3); /* 150 mA precharge */
	/* Limit USB input to 500 mA; leave APSD, AICL and Type-C ownership alone. */
	CAPRI_WRITE(0x1370, 500000 / 50000);
	/* Hardware hard stops at 5 and 45 C. Codes use ADC5's 0x70e4 full scale. */
	CAPRI_WRITE16(0x1098, DIV_ROUND_CLOSEST(264 * 0x70e4, 1875));
	CAPRI_WRITE16(0x109a, DIV_ROUND_CLOSEST(795 * 0x70e4, 1875));
	/* Soft zones 10/40 C reduce charge current by 250 mA. */
	CAPRI_WRITE16(0x1094, DIV_ROUND_CLOSEST(305 * 0x70e4, 1875));
	CAPRI_WRITE16(0x1096, DIV_ROUND_CLOSEST(702 * 0x70e4, 1875));
	CAPRI_WRITE(0x1092, 250000 / 25000);
	CAPRI_WRITE(0x1093, 250000 / 25000);
	CAPRI_UPDATE(0x1090, GENMASK(3, 0), GENMASK(3, 0));
	/* PMI632 ADC current is negative into the pack; terminate at 100 mA. */
	raw = -DIV_ROUND_CLOSEST(100000000, chip->current_lsb);
	CAPRI_WRITE16(0x1067, raw);
	CAPRI_WRITE16(0x1069, DIV_ROUND_CLOSEST(50000000, chip->current_lsb));
	CAPRI_UPDATE(0x10c0, BIT(3), 0); /* ADC termination comparator */
	CAPRI_UPDATE(CHGR_CFG2, GENMASK(7, 6) | GENMASK(3, 0), BIT(3) | BIT(2) | BIT(0));
	/* Recharge below 4.25 V after three samples, using voltage rather than SOC. */
	CAPRI_WRITE16(0x107e, DIV_ROUND_CLOSEST_ULL(4250000000ULL, 194637));
	CAPRI_UPDATE(0x106b, GENMASK(3, 2), 2 << 2);
	/* A 32 s bark plus 8 s bite disables charging if polling stops. */
	CAPRI_WRITE(WD_TIMEOUT, BIT(7) | BIT(2) | GENMASK(1, 0));
	CAPRI_UPDATE(WD_CFG, BIT(7) | BIT(6) | BIT(1), BIT(1));
	CAPRI_WRITE(WD_PET, BIT(0));
#undef CAPRI_WRITE
#undef CAPRI_UPDATE
#undef CAPRI_WRITE16
	chip->charger_ready = true;
	return 0;
}

static int capri_charger_update(struct capri_battery *chip)
{
	unsigned int fcc, fv, therm, hard_hot, hard_cold, wd_cfg, wd_timeout;
	u8 thresholds[4];
	int ret;
	bool enable;

	if (!chip->charger_ready)
		return 0;
	ret = capri_temperature(chip);
	if (ret)
		return ret;
	/* Hysteresis is inside the independent 5..45 C hardware cutoffs. */
	if (chip->temperature <= 50 || chip->temperature >= 450)
		chip->thermal_blocked = true;
	else if (chip->temperature >= 100 && chip->temperature <= 400)
		chip->thermal_blocked = false;
	if (chip->thermal_blocked)
		chip->health = chip->temperature < 100 ?
			POWER_SUPPLY_HEALTH_COLD : POWER_SUPPLY_HEALTH_OVERHEAT;
	ret = regmap_read(chip->map, CHGR_FCC, &fcc);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, CHGR_FV, &fv);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, 0x1090, &therm);
	if (ret)
		return ret;
	ret = regmap_bulk_read(chip->map, 0x1098, thresholds, sizeof(thresholds));
	if (ret)
		return ret;
	ret = regmap_read(chip->map, WD_CFG, &wd_cfg);
	if (ret)
		return ret;
	ret = regmap_read(chip->map, WD_TIMEOUT, &wd_timeout);
	if (ret)
		return ret;
	hard_hot = get_unaligned_be16(thresholds);
	hard_cold = get_unaligned_be16(thresholds + 2);
	if (fcc != chip->fcc_code || fv != chip->fv_code ||
	    (wd_cfg & (BIT(7) | BIT(6) | BIT(1))) != BIT(1) ||
	    wd_timeout != (BIT(7) | BIT(2) | GENMASK(1, 0)) ||
	    (therm & 0xf) != 0xf ||
	    hard_hot != DIV_ROUND_CLOSEST(264 * 0x70e4, 1875) ||
	    hard_cold != DIV_ROUND_CLOSEST(795 * 0x70e4, 1875))
		return -EIO;
	if (chip->voltage > 4450000 || chip->current_ua > 700000)
		return -ERANGE;
	enable = chip->present && chip->online && !chip->stopping &&
		 !chip->thermal_blocked && chip->health == POWER_SUPPLY_HEALTH_GOOD &&
		 chip->behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	ret = regmap_write(chip->map, WD_PET, BIT(0));
	if (ret)
		return ret;
	return regmap_update_bits(chip->map, CHGR_ENABLE, BIT(0), enable ? BIT(0) : 0);
}

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
	if (!ret)
		ret = capri_charger_update(chip);
	if (ret) {
		capri_disable_charging(chip);
		chip->valid = false;
	}
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
	POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
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
	case POWER_SUPPLY_PROP_TEMP: value->intval = chip->temperature; break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR: value->intval = chip->behaviour; break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX: value->intval = 500000; break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX: value->intval = 4400000; break;
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

static int capri_set(struct power_supply *psy, enum power_supply_property prop,
		     const union power_supply_propval *value)
{
	struct capri_battery *chip = power_supply_get_drvdata(psy);
	int ret;

	if (prop != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
	    (value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
	     value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE))
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->behaviour = value->intval;
	ret = capri_disable_charging(chip);
	mutex_unlock(&chip->lock);
	mod_delayed_work(system_wq, &chip->work, 0);
	return ret;
}

static int capri_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const struct power_supply_desc capri_desc = {
	.name = "capri-battery", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = capri_properties, .num_properties = ARRAY_SIZE(capri_properties),
	.get_property = capri_get, .set_property = capri_set,
	.property_is_writeable = capri_writable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
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
	mutex_lock(&chip->lock);
	chip->stopping = true;
	mutex_unlock(&chip->lock);
	cancel_delayed_work_sync(&chip->work);
	capri_disable_charging(chip);
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
	if (subtype != 3)
		return -ENODEV;
	ret = capri_disable_charging(chip);
	if (ret)
		return ret;
	chip->therm = devm_iio_channel_get(dev, "batt-therm");
	if (IS_ERR(chip->therm))
		return dev_err_probe(dev, PTR_ERR(chip->therm), "Battery thermistor unavailable\n");
	chip->behaviour = charging ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
		POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
	chip->thermal_blocked = true;
	platform_set_drvdata(pdev, chip);
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
	ret = capri_temperature(chip);
	if (ret)
		return dev_err_probe(dev, ret, "Battery temperature read failed\n");
	ret = capri_charger_init(chip);
	if (ret)
		return dev_err_probe(dev, ret, "Charger initialization failed\n");
	schedule_delayed_work(&chip->work, 0);
	dev_info(dev, "PMI632: 500 mA / 4.4 V; hardware thermal cutoffs 5..45 C; OCV/current battery estimate\n");
	return 0;
}

static void capri_shutdown(struct platform_device *pdev)
{
	struct capri_battery *chip = platform_get_drvdata(pdev);

	capri_cancel(chip);
}

static int capri_suspend(struct device *dev)
{
	struct capri_battery *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->lock);
	chip->stopping = true;
	mutex_unlock(&chip->lock);
	cancel_delayed_work_sync(&chip->work);
	ret = capri_disable_charging(chip);
	if (ret) {
		mutex_lock(&chip->lock);
		chip->stopping = false;
		mutex_unlock(&chip->lock);
		schedule_delayed_work(&chip->work, 0);
	}
	return ret;
}

static int capri_resume(struct device *dev)
{
	struct capri_battery *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	chip->stopping = false;
	mutex_unlock(&chip->lock);
	schedule_delayed_work(&chip->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(capri_pm_ops, capri_suspend, capri_resume);

static const struct of_device_id capri_match[] = {
	{ .compatible = "motorola,capri-pmi632-battery" }, {}
};
MODULE_DEVICE_TABLE(of, capri_match);
static struct platform_driver capri_driver = {
	.probe = capri_probe, .shutdown = capri_shutdown,
	.driver = { .name = "capri-pmi632-battery", .of_match_table = capri_match, .pm = pm_sleep_ptr(&capri_pm_ops) },
};
module_platform_driver(capri_driver);
MODULE_DESCRIPTION("Moto G10 PMI632 battery and charger");
MODULE_LICENSE("GPL");
