/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"ZSPMI: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/spmi.h>
#include <linux/debugfs.h>
#include <linux/qpnp/power-on.h>
#include <linux/qpnp/qpnp-adc.h>

#include "ztemt_qpnp_spmi.h"

/* Config / Data registers */
#define REVISION1_REG			0x0
#define BMS_SOC_REG			0xB0
#define BMS_OCV_REG			0xB1 /* B1 & B2 */
#define SOC_STORAGE_MASK		0xFE
#define OCV_INVALID			0xFFFF
#define SOC_INVALID			0xFF

#define ZTEMT_QPNP_SPMI_DEV_NAME		"qcom,ztemt-spmi-dev"

struct qpnp_spmi_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	struct mutex			backup_soc_mutex;
	dev_t				dev_no;
	u16				base;
	u8				revision[2];
	u32				batt_pres_addr;
	u32				chg_pres_addr;
	u8				shutdown_soc;
	bool			shutdown_soc_invalid;
	u32				shutdown_ocv;
	bool				warm_reset;
	struct mutex			state_change_mutex;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
};

static struct qpnp_spmi_chip *spmi_chip;

static int qpnp_read_wrapper(struct qpnp_spmi_chip *chip, u8 *val,
					u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed rc=%d\n", rc);

	return rc;
}

static int qpnp_write_wrapper(struct qpnp_spmi_chip *chip, u8 *val,
			u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI write failed rc=%d\n", rc);

	return rc;
}

static int qpnp_masked_write_base(struct qpnp_spmi_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = qpnp_read_wrapper(chip, &reg, addr, 1);
	if (rc) {
		pr_err("read failed addr = %03X, rc = %d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = qpnp_write_wrapper(chip, &reg, addr, 1);
	if (rc)
		pr_err("write failed addr = %03X, val = %02x, mask = %02x, reg = %02x, rc = %d\n",
					addr, val, mask, reg, rc);

	return rc;
}

static int backup_ocv_soc(struct qpnp_spmi_chip *chip, int ocv_uv, int soc)
{
	int rc;
	u16 ocv_mv = ocv_uv / 1000;

	rc = qpnp_write_wrapper(chip, (u8 *)&ocv_mv,
				chip->base + BMS_OCV_REG, 2);
	if (rc)
		pr_err("Unable to backup OCV rc=%d\n", rc);

	rc = qpnp_masked_write_base(chip, chip->base + BMS_SOC_REG,
				SOC_STORAGE_MASK, (soc + 1) << 1);
	if (rc)
		pr_err("Unable to backup SOC rc=%d\n", rc);

	pr_debug("ocv_mv=%d soc=%d\n", ocv_mv, soc);

	return rc;
}

static int read_shutdown_ocv_soc(struct qpnp_spmi_chip *chip)
{
	u8 stored_soc = 0;
	u16 stored_ocv = 0;
	int rc;

	rc = qpnp_read_wrapper(chip, (u8 *)&stored_ocv,
				chip->base + BMS_OCV_REG, 2);
	if (rc) {
		pr_err("failed to read addr = %d %d\n",
				chip->base + BMS_OCV_REG, rc);
		return -EINVAL;
	}

	/* if shutdwon ocv is invalid, reject shutdown soc too */
	if (!stored_ocv || (stored_ocv == OCV_INVALID)) {
		pr_debug("shutdown OCV %d - invalid\n", stored_ocv);
		chip->shutdown_ocv = OCV_INVALID;
		chip->shutdown_soc = SOC_INVALID;
		return -EINVAL;
	}
	chip->shutdown_ocv = stored_ocv * 1000;

	/*
	 * The previous SOC is stored in the first 7 bits of the register as
	 * (Shutdown SOC + 1). This allows for register reset values of both
	 * 0x00 and 0xFF.
	 */
	rc = qpnp_read_wrapper(chip, &stored_soc, chip->base + BMS_SOC_REG, 1);
	if (rc) {
		pr_err("failed to read addr = %d %d\n",
				chip->base + BMS_SOC_REG, rc);
		return -EINVAL;
	}

	if (!stored_soc || stored_soc == SOC_INVALID) {
		chip->shutdown_soc = SOC_INVALID;
		chip->shutdown_ocv = OCV_INVALID;
		return -EINVAL;
	} else {
		chip->shutdown_soc = (stored_soc >> 1) - 1;
	}

	pr_debug("shutdown_ocv=%d shutdown_soc=%d\n",
			chip->shutdown_ocv, chip->shutdown_soc);

	return 0;
}

int qpnp_backup_ocv_soc(int soc, int vbatt_mv)
{
	int rc;

	if(!spmi_chip){
		pr_err("spmi_chip is not inited\n");
		return -EINVAL;
	}
	
	mutex_lock(&spmi_chip->backup_soc_mutex);	
	rc = backup_ocv_soc(spmi_chip, 1000*vbatt_mv, soc);
	if (rc)
		pr_err("Unable to backup SOC&OCV rc=%d\n", rc);
	mutex_unlock(&spmi_chip->backup_soc_mutex);
	
	return rc;
}

int qpnp_read_shutdown_ocv_soc(int *soc, int *vbatt_mv)
{
	int rc;

	if(!spmi_chip){
		pr_err("spmi_chip is not inited\n");
		return -EINVAL;
	}
	
	rc = read_shutdown_ocv_soc(spmi_chip);
	if(!rc){
		*vbatt_mv = spmi_chip->shutdown_ocv/1000;
		*soc = spmi_chip->shutdown_soc;
	}else
		pr_err("Unable to read shutdown SOC&OCV rc=%d\n", rc);
	return rc;
}

#define BAT_PRES_BIT		BIT(7)
bool pm_battery_present(void)
{
	int rc;
	u8 batt_pres;

	if(!spmi_chip){
		pr_err("spmi_chip is not inited\n");
		return 1;
	}

	if (spmi_chip->batt_pres_addr) {
		rc = qpnp_read_wrapper(spmi_chip, &batt_pres, spmi_chip->batt_pres_addr, 1);
		if (!rc && (batt_pres & BAT_PRES_BIT))
			return true;
		else
			return false;
	}

	return 1;
}

static int parse_spmi_dt_properties(struct qpnp_spmi_chip *chip, struct spmi_device *spmi)
{
	struct spmi_resource *spmi_resource;
	struct resource *resource;

	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("qpnp_vm_bms: spmi resource absent\n");
			return -ENXIO;
		}

		resource = spmi_get_resource(spmi, spmi_resource, IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",	spmi->dev.of_node->full_name);
			return -ENXIO;
		}

		pr_debug("Node name = %s\n", spmi_resource->of_node->name);

		if (strcmp("qcom,batt-pres-status",	spmi_resource->of_node->name) == 0) {
			chip->batt_pres_addr = resource->start;
			continue;
		}

		chip->base = resource->start;
	}

	if (chip->base == 0) {
		dev_err(&spmi->dev, "BMS peripheral was not registered\n");
		return -EINVAL;
	}

	pr_info("bms-base=0x%04x bat-pres-reg=0x%04x\n", chip->base, chip->batt_pres_addr);

	return 0;
}

static int ztemt_qpnp_spmi_probe(struct spmi_device *spmi)
{
	struct qpnp_spmi_chip *chip;
	int rc;

	pr_debug("enter probe: \n");

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("kzalloc() failed.\n");
		return -ENOMEM;
	}

	rc = qpnp_pon_is_warm_reset();
	if (rc < 0) {
		pr_err("Error reading warm reset status rc=%d\n", rc);
		return rc;
	}
	chip->warm_reset = !!rc;

	rc = parse_spmi_dt_properties(chip, spmi);
	if (rc) {
		pr_err("Error registering spmi resource rc=%d\n", rc);
		return rc;
	}
	
	dev_set_drvdata(&spmi->dev, chip);
	mutex_init(&chip->backup_soc_mutex);

	rc = qpnp_read_wrapper(chip, chip->revision, chip->base + REVISION1_REG, 2);
	if (rc) {
		pr_err("Error reading version register rc=%d\n", rc);
		return rc;
	}

	spmi_chip = chip;

	pr_info("probe success: warm_reset=%d\n",chip->warm_reset);

	return rc;

}

static int ztemt_qpnp_spmi_remove(struct spmi_device *spmi)
{
	struct qpnp_spmi_chip *chip = dev_get_drvdata(&spmi->dev);

	mutex_destroy(&chip->backup_soc_mutex);
	
	dev_set_drvdata(&spmi->dev, NULL);
	spmi_chip = NULL;

	return 0;
}

static int ztemt_spmi_suspend(struct device *dev)
{

	return 0;
}

static int ztemt_spmi_resume(struct device *dev)
{

	return 0;
}

static const struct dev_pm_ops ztemt_qpnp_spmi_pm_ops = {
	.suspend	= ztemt_spmi_suspend,
	.resume		= ztemt_spmi_resume,
};

static struct of_device_id ztemt_qpnp_spmi_match_table[] = {
	{ .compatible = ZTEMT_QPNP_SPMI_DEV_NAME },
	{}
};

static struct spmi_driver ztemt_qpnp_spmi_driver = {
	.probe		= ztemt_qpnp_spmi_probe,
	.remove		= ztemt_qpnp_spmi_remove,
	.driver		= {
		.name		= ZTEMT_QPNP_SPMI_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= ztemt_qpnp_spmi_match_table,
		.pm		= &ztemt_qpnp_spmi_pm_ops,
	},
};

int  ztemt_qpnp_spmi_init(void)
{
	return spmi_driver_register(&ztemt_qpnp_spmi_driver);
}

