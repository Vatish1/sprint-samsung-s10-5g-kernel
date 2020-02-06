 /*
 * ln2702_regulator.c
 * Samsung LN2702 Voltage Regulator Driver
 *
 * Copyright (C) 2019 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
//#define DEBUG

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */
#include "include/regulator/ln2702_regulator.h"

static enum power_supply_property ln2702_regulator_props[] = {
};

static const struct regmap_config ln2702_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= LN2702_MAX_REGISTER,
};

static int ln2702_read_reg(struct ln2702_info *info, u8 addr, u8 *value)
{
	int ret = -EIO;
	unsigned int reg, val;

	reg = addr;
	ret = regmap_read(info->regmap, reg, &val);
	if (ret) {
		pr_debug("%s: error(%d), reg(0x%02x)\n", __func__, ret, reg);
		*value = 0xFF;
		return ret;
	}

	*value = (val & 0xFF);
	return ret;
}

#if 0 //temporary build
static int ln2702_write_reg(struct ln2702_info *info, u8 addr, u8 value)
{
	int ret = -EIO;
	unsigned int reg, val;
	
	reg = addr;
	val = value;
	ret = regmap_write(info->regmap, reg, val);
	if (ret) {
		pr_debug("%s: error(%d), reg(0x%02x), val(0x%02x)\n",
			__func__, ret, reg, val);
		return ret;
	}
	return ret;
}
#endif

static int ln2702_update_reg(struct ln2702_info *info, u8 addr, u8 value, u8 bits)
{
	int ret = -EIO;
	unsigned int reg, mask, val;

	reg = addr;
	mask = (bits & 0xFF);
	val = (value & 0xFF);
	ret = regmap_update_bits(info->regmap, reg, mask, val);
	if (ret) {
		pr_debug("%s: error(%d), reg(0x%02x), mask(0x%02x), val(0x%02x)\n",
			__func__, ret, reg, mask, val);
		return ret;
	}
	return ret;
}

static int ln2702_bulk_read(struct ln2702_info *info, u8 addr, u8 *val, size_t val_count)
{
	int ret = -EIO;
	unsigned int reg;

	reg = addr;
	ret = regmap_bulk_read(info->regmap, addr, val, val_count);
	if (reg) {
		pr_debug("%s: error(%d), reg(0x%02x), count(%d)\n",
			__func__, ret, reg, val_count);
		return ret;
	}

	return ret;
}

static int ln2702_bulk_write(struct ln2702_info *info, u8 addr, u8 *val, size_t val_count)
{
	int ret = -EIO;
	unsigned int reg;

	reg = addr;
	ret = regmap_bulk_write(info->regmap, addr, val, val_count);
	if (reg) {
		pr_debug("%s: error(%d), reg(0x%02x), count(%d)\n",
			__func__, ret, reg, val_count);
		return ret;
	}

	return ret;
}

static void ln2702_test_read(struct ln2702_info *info)
{
	u8 addr, val;
	char str[256] = { 0, };

	for (addr = LN2702_REG_DEVICE_ID; addr <= LN2702_REG_TRACK_CTRL; addr++) {
		if (LN2702_REG_INT_DEVICE_0 <= addr && addr <= LN2702_REG_INT_HV_SC_1)
			continue;
		ln2702_read_reg(info, addr, &val);
		sprintf(str + strlen(str), "[0x%02X]0x%02X, ", addr, val);
	}

	addr = LN2702_REG_STS_D;
	ln2702_read_reg(info, addr, &val);
	sprintf(str + strlen(str), "[0x%02X]0x%02X, ", addr, val);

	if (info->pdata->en_gpio)
		pr_info("%s: [EN:%d] %s\n", __func__, gpio_get_value(info->pdata->en_gpio), str);
	else
		pr_info("%s: [EN:-] %s\n", __func__, str);
}

static void ln2702_set_int_mask(struct ln2702_info *info, bool msk)
{
	u8 int_msk[REG_INT_MAX];

	if (msk)
		memset(int_msk, 0xFF, sizeof(int_msk)); 
	else
		memset(int_msk, 0x00, sizeof(int_msk));

	ln2702_bulk_write(info, LN2702_REG_INT_DEVICE_0_MSK, int_msk, REG_INT_MAX);	
}

/*********************************************************************************
 * supported functionality
 *********************************************************************************/


static inline void ln2702_set_infet(struct ln2702_info *info, unsigned int enable)
{
#if 1
	u8 val;
	int ret;

	val = enable << MASK2SHIFT(LN2702_BIT_INFET_EN);
	pr_info("%s: %s\n", __func__, enable ? "ON" : "OFF");
	ret = ln2702_update_reg(info, LN2702_REG_DEVICE_CTRL_0, val, LN2702_BIT_INFET_EN);
#else
	regmap_update_bits(info->regmap, LN2702_REG_DEVICE_CTRL_0,
				LN2702_MASK_INFET_EN, 
				(enable & 0x01));
#endif
}

static inline void ln2702_set_powerpath(struct ln2702_info *info, bool forward_path) {
	info->reverse_power = (!forward_path);
}

static inline int ln2702_get_opmode(struct ln2702_info *info)
{
	int val;
	if (regmap_read(info->regmap, LN2702_REG_HV_SC_CTRL_0, &val) < 0)
	   val = LN2702_OPMODE_UNKNOWN;
	return (val & LN2702_MASK_SC_OPERATION_MODE);
}

/* configure minimum set of control registers */
static inline void ln2702_set_base_opt(struct ln2702_info *info,
			unsigned int sc_out_precharge_cfg,
			unsigned int precharge_fault_chk_en,
			unsigned int track_cfg)
{
	regmap_update_bits(info->regmap, LN2702_REG_HV_SC_CTRL_1, 0x20, 
				(sc_out_precharge_cfg<<5));
	regmap_update_bits(info->regmap, LN2702_REG_TRACK_CTRL,
				0x30,
				(precharge_fault_chk_en<<5) |
				(track_cfg<<4));
}

static inline void ln2702_set_fault_opt(struct ln2702_info *info,
			bool set_ov,
			unsigned int disable_vbus_in_switch_ok,
			unsigned int disable_sc_out_min_ok,
			unsigned int disable_wpc_in_min_ok,
			unsigned int disable_vbus_in_min_ok)
{
	if (set_ov) {
	   regmap_update_bits(info->regmap, LN2702_REG_FAULT_CTRL, 0x3F,
				(3<<4) |
				(disable_vbus_in_switch_ok<<3) |
				(disable_sc_out_min_ok<<2) |
				(disable_wpc_in_min_ok<<1) |
				(disable_vbus_in_min_ok));
	} else {
	   regmap_update_bits(info->regmap, LN2702_REG_FAULT_CTRL, 0x3F,
				(disable_vbus_in_switch_ok<<3) |
				(disable_sc_out_min_ok<<2) |
				(disable_wpc_in_min_ok<<1) |
				(disable_vbus_in_min_ok));
	}
}


static void ln2702_enter_standby(struct ln2702_info *info)
{
	//update opmode
	regmap_update_bits(info->regmap, LN2702_REG_HV_SC_CTRL_0, 
				LN2702_MASK_SC_OPERATION_MODE,
				LN2702_OPMODE_STANDBY);

	ln2702_set_base_opt(info, 1, 1, 1);
	ln2702_set_fault_opt(info, false, 0, 1, 0, 0);
}


static void ln2702_enter_bypass(struct ln2702_info *info)
{
	ln2702_set_base_opt(info, 1, 1, 1);

	if (info->reverse_power)
	   ln2702_set_fault_opt(info, false, 1, 0, 1, 1);
	else
	   ln2702_set_fault_opt(info, false, 1, 1, 0, 0);

	//update opmode
	regmap_update_bits(info->regmap, LN2702_REG_HV_SC_CTRL_0, 
			LN2702_MASK_SC_OPERATION_MODE, LN2702_OPMODE_BYPASS);
}

static void ln2702_enter_switching(struct ln2702_info *info)
{
	if (info->reverse_power) {
	   ln2702_set_base_opt(info, 0, 0, 0);
	   ln2702_set_fault_opt(info, true, 1, 1, 0, 1);//temporarily set
	} else {
#ifdef LN2702_HW_REV_A1
	   ln2702_set_base_opt(info, 0, 0, 0);
#else
	   ln2702_set_base_opt(info, 0, 1, 0);
#endif
	   ln2702_set_fault_opt(info, true, 1, 1, 0, 0);//temporarily set
	}

	//update opmode
	regmap_update_bits(info->regmap, LN2702_REG_HV_SC_CTRL_0, 
			LN2702_MASK_SC_OPERATION_MODE, LN2702_OPMODE_SWITCHING);

	if (info->reverse_power)
	   ln2702_set_fault_opt(info, false, 1, 0, 0, 1);
	else
	   ln2702_set_fault_opt(info, false, 0, 1, 0, 0);
}


/* main function for setting/changing operation mode */
static bool ln2702_change_opmode(struct ln2702_info *info, unsigned int target_mode)
{
	bool ret;
//	bool reverse;

	info->op_mode = ln2702_get_opmode(info);

	if (target_mode < 0 || target_mode > LN2702_OPMODE_SWITCHING_ALT) {
	   pr_err("%s: target operation mode (0x%02X) is invalid\n", 
		  __func__, target_mode);
	   return false;
	}

	pr_info("%s: current_mode(0x%02X)->target_mode(0x%02X)\n",
		__func__, info->op_mode, target_mode);

	/* NOTE: 
	 *      CUSTOMER should know/indicate if power path is forward/reverse mode
	 *      based on power connections before attempting to change operation mode
         */
	//info->reverse_power = false;

	ret = true;
	switch(target_mode) {
	  case LN2702_OPMODE_STANDBY:
		ln2702_enter_standby(info);
		break;
	  case LN2702_OPMODE_BYPASS:
		ln2702_enter_bypass(info);
		break;
	  case LN2702_OPMODE_SWITCHING:
	  case LN2702_OPMODE_SWITCHING_ALT:
		ln2702_enter_switching(info);
		break;
	  default:
		ret = false;
	}
	return ret;
}

static int ln2702_regulator_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct ln2702_info *info = power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = (enum power_supply_ext_property) psp;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->cable_type;
		break;
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		switch (ext_psp) {
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int ln2702_regulator_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct ln2702_info *info = power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = (enum power_supply_ext_property) psp;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (!is_nocharge_type(val->intval) &&
			info->cable_type != val->intval)
			ln2702_set_int_mask(info, false);

		info->cable_type = val->intval;

		if (is_wired_type(info->cable_type)) {			
			ln2702_change_opmode(info, LN2702_OPMODE_BYPASS);
			ln2702_set_infet(info, false);
		} else if (is_wireless_type(info->cable_type)) {
			ln2702_set_infet(info, true);
			//ln2702_change_opmode(info, LN2702_OPMODE_STANDBY);
		} else {
			// skip
		}

		if (!is_nocharge_type(info->cable_type))
			ln2702_test_read(info);
		break;
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		switch (ext_psp) {
		case POWER_SUPPLY_EXT_PROP_WIRELESS_TX_ENABLE:
			ln2702_set_infet(info, val->intval);
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t ln2702_interrupt_handler(int irq, void *data)
{
	struct ln2702_info *info = data;
	u8 irq_reg[REG_INT_MAX], sts_reg[REG_STS_MAX];

	ln2702_set_int_mask(info, true);

	ln2702_bulk_read(info, LN2702_REG_INT_DEVICE_0, irq_reg, REG_INT_MAX);
	ln2702_bulk_read(info, LN2702_REG_INT_DEVICE_0_STS, sts_reg, REG_STS_MAX);

	pr_info("%s: irq(0x%02X,0x%02X,0x%02X,0x%02X), sts(0x%02X,0x%02X,0x%02X,0x%02X)\n",
		__func__,
		irq_reg[0], irq_reg[1], irq_reg[2], irq_reg[3],
		sts_reg[0], sts_reg[1], sts_reg[2], sts_reg[3]);

	ln2702_set_int_mask(info, false);

	return IRQ_HANDLED;
}

/*********************************************************************************
 * device layer
 *********************************************************************************/
static int ln2702_irq_init(struct ln2702_info *info)
{
	const ln2702_regulator_platform_data_t *pdata = info->pdata;
	int ret;

	info->irq = gpio_to_irq(pdata->irq_gpio);	
	pr_info("%s: irq=%d, irq_gpio=%d\n", __func__,
			info->irq, pdata->irq_gpio);
	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, "ln2702-irq");
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(info->irq, NULL, ln2702_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "ln2702-irq", info);
	if (ret < 0)
		goto fail_gpio;

	return 0;

fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	info->irq = 0;
	return ret;
}

static int ln2702_hw_init(struct ln2702_info *info)
{
	/* CUSTOMER should add basic initialization tasks
	 * (unmasking interrupts, changing control settings, etc)
	 */
	pr_info("%s: HW initialization\n", __func__);

	return 0;
}

#ifdef CONFIG_OF
static int ln2702_regulator_parse_dt(struct device *dev, ln2702_regulator_platform_data_t *pdata)
{
	struct device_node *np_ln2702  = dev->of_node;
	int ret = 0;

	/* nINT gpio */
	ret = pdata->irq_gpio = of_get_named_gpio(np_ln2702, "ln2702,irq-gpio", 0);
	if (ret < 0)
		pr_info("%s: can't get irq-gpio (%d)\n", __func__, pdata->irq_gpio);

	/* nEN gpio */
	ret = pdata->en_gpio = of_get_named_gpio(np_ln2702, "ln2702,en-gpio", 0);
	if (ret < 0) {
		pdata->en_gpio = 0;
		pr_info("%s: can't get en-gpio (%d)\n", __func__, pdata->en_gpio);
	}
	pr_info("%s: irq-gpio: %u, en-gpio: %u\n",
		__func__, pdata->irq_gpio, pdata->en_gpio);

	return 0;
}
#endif

static const struct power_supply_desc ln2702_regulator_power_supply_desc = {
	.name = "ln2702-regulator",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = ln2702_regulator_props,
	.num_properties = ARRAY_SIZE(ln2702_regulator_props),
	.get_property = ln2702_regulator_get_property,
	.set_property = ln2702_regulator_set_property,
	.no_thermal = true,
};

static int ln2702_regulator_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{ 
	struct ln2702_info *info;
	ln2702_regulator_platform_data_t *pdata = client->dev.platform_data;
	struct device *dev = &client->dev;
	struct power_supply_config regulator_cfg = {};
	int ret = 0;

	pr_info("%s: Driver Loading\n", __func__);

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(ln2702_regulator_platform_data_t),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			ret = -ENOMEM;
			goto err_pdata_nomem;
		}

		ret = ln2702_regulator_parse_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			ret = -ENOMEM;
			goto err_parse_dt;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata) {
		ret = -EINVAL;
		goto err_pdata_nomem;
	}

	info->client = client;
	info->dev = &client->dev;
	info->pdata = pdata;

	// init variables
	info->cable_type = SEC_BATTERY_CABLE_NONE;

	i2c_set_clientdata(client, info);

	info->regmap = devm_regmap_init_i2c(client, &ln2702_regmap_config);
	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		goto err_regmap_init;
	}

	if (ln2702_hw_init(info) < 0) {
	   pr_err("%s: hardware initialization error\n", __func__);
	   ret = -EINVAL;
	   goto err_hw_init;
	}

	// register_power_supply
	regulator_cfg.drv_data = info;
	info->psy_regulator = power_supply_register(dev,
		&ln2702_regulator_power_supply_desc, &regulator_cfg);
	if (IS_ERR(info->psy_regulator)) {
		ret = PTR_ERR(info->psy_regulator);
		goto err_power_supply_register;
	}

	if (info->pdata->irq_gpio >= 0) {
		ret = ln2702_irq_init(info);
		if (ret < 0)
			pr_warn("%s: failed to initialize IRQ(%d), disable IRQ\n",
				__func__, info->pdata->irq_gpio);
	}

	pr_info("%s: Loaded\n", __func__);

	return 0;

err_power_supply_register:
err_hw_init:
err_regmap_init:
err_parse_dt:
	devm_kfree(&client->dev, pdata);
err_pdata_nomem:
	kfree(info);

	return ret;
}

static int ln2702_regulator_remove(struct i2c_client *client)
{
	struct ln2702_info *info = i2c_get_clientdata(client);

	pr_info("%s: ++\n", __func__);

	if (info->irq) {
		free_irq(info->irq, info);
		gpio_free(info->pdata->irq_gpio);
	}

	if (info->psy_regulator)
		power_supply_unregister(info->psy_regulator);

	pr_info("%s: --\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int ln2702_regulator_suspend(struct device *dev)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	struct ln2702_info *info = i2c_get_clientdata(client);

	ln2702_change_opmode(info, LN2702_OPMODE_BYPASS);

	pr_info("%s: cancel delayed work\n", __func__);
#endif

	return 0;
} 

static int ln2702_regulator_resume(struct device *dev)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	struct ln2702_info *info = i2c_get_clientdata(client);

	pr_info("%s: update/resume\n", __func__);
#endif

	return 0;
}
#else
#define ln2702_regulator_suspend NULL
#define ln2702_regulator_resume NULL
#endif
static SIMPLE_DEV_PM_OPS(ln2702_regulator_pm_ops,
			ln2702_regulator_suspend, ln2702_regulator_resume);

static void ln2702_regulator_shutdown(struct i2c_client *client)
{
	pr_info("%s: ++\n", __func__);
	pr_info("%s: --\n", __func__);
}

#ifdef CONFIG_OF
static const struct of_device_id ln2702_regulator_dt_match_table[] = {
	{ .compatible = "lionsemi,ln2702" },
	{ },
};
MODULE_DEVICE_TABLE(of, ln2702_regulator_dt_match_table);
#else
#define ln2702_regulator_dt_match_table NULL
#endif

static const struct i2c_device_id ln2702_regulator_id_table[] = {
	{ "ln2702-regulator", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ln2702_regulator_id_table);

static struct i2c_driver ln2702_regulator = {
	.driver   = {
		.name = LN2702_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ln2702_regulator_dt_match_table,
#if defined(CONFIG_PM)
		.pm = &ln2702_regulator_pm_ops,
#endif /* CONFIG_PM */
	},
	.probe    = ln2702_regulator_probe,
	.remove   = ln2702_regulator_remove,
	.shutdown	= ln2702_regulator_shutdown,
	.id_table = ln2702_regulator_id_table,
};
module_i2c_driver(ln2702_regulator);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("LN2702 driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2.0");
