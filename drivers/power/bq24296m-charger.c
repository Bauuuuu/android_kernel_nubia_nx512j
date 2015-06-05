/*
 * BQ24296 charger driver
 *
 * Copyright (C) 2013 ZTEMT
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#define pr_fmt(fmt)	"BQ: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include "linux/power/bq24296m_charger.h"

#define DRIVER_VERSION			"1.0.0"

#define TRUE			1
#define FALSE		0

#define PM_INFO 1
#define PM_DEBUG 4
//log level < bqlog_level will show
int bqlog_level = 2;  
module_param(bqlog_level, int, 0644);

#define BQLOG_INFO(fmt, args...) \
		if (PM_INFO < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
	
#define BQLOG_DEBUG(fmt, args...) \
		if (PM_DEBUG < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)

struct bq24296_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq24296_chg_chip {
	struct device	  *dev;
	struct i2c_client *i2c;
	struct power_supply	   *batt_psy;
	struct power_supply	   *usb_psy;
	struct power_supply	     bq24296_batt_psy;	
	struct delayed_work		chg_work;
	struct work_struct      		usbin_work;
	struct pinctrl *chg_pinctrl;
	struct pinctrl_state *gpio_state_active;	
	struct pinctrl_state *gpio_state_suspend;
	struct bq24296_otg_regulator otg_vreg;
	struct device_node *dev_node;
	struct wake_lock wlock;
	struct qpnp_vadc_chip	*vadc_dev;
	enum usb_chg_type chg_type;
	int chg_en_gpio;
	int otg_gpio;
	int psel_gpio;
	int irq_gpio;
	int irq;

	int charge_sense;
	
	//batt status info
	int batt_status; 
	int temp_status;
	int batt_temp;
	int chg_status;
	int batt_i;
	int batt_vol;
	int batt_soc;
	int chg_vol;
	int rechg_soc;
	int rechg_vol;
	int batt_present;
	
	int ibatmax_ma;
	int vusb_min;
	int iusbmax_ma;
	int iterm_ma;
	int vbatt_max;
	int set_ibatt;
	int vol_cutoff;
	int chg_cycle_timer;
	
	int usb_in;
	int config_mode;
	bool in_work;
	bool temp_abnormal;
	bool soc_chg_done;
	bool in_rechging;
	bool is_hvdcp_chg;
	bool is_chg_full;
};

enum power_supply_property bq24296_batt_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
//	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
};



static struct bq24296_chg_chip   *bq_chip;
static int usbin_current = 1500;
module_param(usbin_current, int, 0644);

static int ibat_current = 1200;
module_param(ibat_current, int, 0644);
static int charger_online = 0;

static int bq_ibatt_current_limit [IBATT_MAX]={0x00,0x04,0x08,0x10,0x18,0x28};
static int bq_ibatt_current [IBATT_MAX]={512,768,1024,1536,2048,3008};
static int bq_iusb_current_limit [IUSB_MAX]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
static int iusb_current [IUSB_MAX]={100,150,500,900,1000,1500,2000,3000};
static int  ibatt_term = 0x00;// ibatt_term = 128mA

static DEFINE_MUTEX(bq24296_i2c_mutex);
//extern int qpnp_get_battery_temp(void);
//extern int qpnp_dcdc_enable(int enable);

static int bq24296_get_temp_status(const struct temp_status_map *pts,
		uint32_t tablesize, int input, int *temp_status);
static int bq24296_chg_hw_init(struct bq24296_chg_chip *chip);
static int  bq24296_get_prop_chg_status(void);

	#define BATT_COLD_CURRENT      0 
	#define BATT_COOL2_CURRENT    768 	
	#define BATT_GOOD_CURRENT     1536 
	#define BATT_WARM_CURRENT    1024 
	#define BATT_HOT_CURRENT        0 
static const struct temp_status_map batt_temp_map[] = {
	{CHG_TEMP_MIN, CHG_TEMP_COLD, BATT_STATUS_COLD,BATT_COLD_CURRENT}, 
	{CHG_TEMP_COLD, CHG_TEMP_COOL2, BATT_STATUS_COOL2, BATT_COOL2_CURRENT}, 	
	{CHG_TEMP_COLD, CHG_TEMP_WARM, BATT_STATUS_GOOD, BATT_GOOD_CURRENT}, 
	{CHG_TEMP_WARM, CHG_TEMP_HOT, BATT_STATUS_WARM, BATT_WARM_CURRENT}, 
	{CHG_TEMP_HOT, CHG_TEMP_MAX, BATT_STATUS_HOT,BATT_HOT_CURRENT}, 
};

/* Charge status register values
*/
enum bq24296_chg_status {
	CHARGE_STATE_NO_CHG	= 0,
	CHARGE_STATE_PRE_CHG	= 1,
	CHARGE_STATE_FAST_CHG	= 2,
	CHARGE_STATE_CHG_DONE	= 3,
};

enum bq24296_boost_mode_hot_temp_threshold {
	TEMP_55_DEGREE	= 0,
	TEMP_60_DEGREE	= 1,
	TEMP_65_DEGREE	= 2,
	TEMP_MONITOR_DISABLE	= 3,
};

enum bq24296_boost_mode_cold_temp_threshold {
	TEMP_SUBZERO_10_DEGREE	= 0,
	TEMP_SUBZERO_20_DEGREE	= 1,
};


/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/

static int bq24296_i2c_readb(
		struct i2c_client *i2c,
		u8  reg,
		u8* buf)
{
	struct i2c_msg msgs[2];
	int ret = 0;

	//write message: this is the sub address that we are reading from
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	//read message: now we read into the buffer
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;
	
	mutex_lock(&bq24296_i2c_mutex);
	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		ret = -4;
	}
	mutex_unlock(&bq24296_i2c_mutex);
	
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return ret;
}

//write bq24296 i2c function
static int bq24296_i2c_writeb(
		struct i2c_client *i2c,
		u8 reg, 
		u8 buf)
{
	struct i2c_msg msgs;
	u8 data[2];
	int ret = 0;

	data[0] = reg;
	data[1] = buf;

	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = data;
	
	mutex_lock(&bq24296_i2c_mutex);
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		ret = -4;
	}
	mutex_unlock(&bq24296_i2c_mutex);

	return ret;
}
/****************************************************************/

static int  bq24296_masked_write(struct bq24296_chg_chip *chip, u8 reg,
							u8 mask, u8 val)
{
	int rc;
	u8 buf;

	rc = bq24296_i2c_readb(chip->i2c,reg,&buf);
	if (rc) {
		pr_err("bq24296_i2c_readb failed: reg=0x%x, rc=%d\n", reg, rc);
		return rc;
	}
	
	buf &= ~mask;//clear mask bit 
	buf |= val & mask;

	rc = bq24296_i2c_writeb(chip->i2c,reg,buf);
	if (rc) {
		pr_err("bq24296_i2c_writeb failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}

	return 0;
}

static int bq24296_chg_gpio_select(struct bq24296_chg_chip *chip,int enable)
{
	struct pinctrl_state *pins_state;
	int ret;
	
	BQLOG_DEBUG(" select gpio state: %s\n",enable? "active":"suspend");

	pins_state = chip->gpio_state_active;
	
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(chip->chg_pinctrl, pins_state);
		if (ret) {
			pr_err(	"can not set gpio pins(%d)\n",ret );
			return ret;
		}
	} else 
		pr_err("not a valid gpio pinstate\n");
	
	pr_err("gpio_select success\n");

	return 0;
}

static int bq24296_chg_gpio_enable(struct bq24296_chg_chip *chip,int enable)
{
	int ret;
	
	BQLOG_DEBUG("set enable gpio: %d\n",enable );

	ret = gpio_direction_output(chip->chg_en_gpio,enable);
	if(ret)
		pr_err("can not set gpio %d\n",chip->chg_en_gpio);
	return 0;
}

//Reg 0x0 ---------------------------------------------------------------

static int  bq24296_hiz_mode_enable(struct bq24296_chg_chip *chip,int enable)
{
	int rc;
	u8 val;

	val = enable << BQ24296_HZ_MODE_SHIFT;
	
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_SOURCE_CNTL,
		                      BQ24296_HZ_MODE_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int  bq24296_is_hiz_mode(struct bq24296_chg_chip *chip)
{
	int rc;
	u8  val;

	rc = bq24296_i2c_readb(chip->i2c,BQ24296_REG_SOURCE_CNTL,&val);
    if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_HZ_MODE_MASK;
	val = val >> BQ24296_HZ_MODE_SHIFT;
		
	return val;
}

static int  bq24296_set_chg_vol(struct bq24296_chg_chip *chip, int vol_mv)
{
	u8 val;

	if (vol_mv < BQ24296_CHG_MIN_VCHG || vol_mv > BQ24296_CHG_MAX_VCHG) {
		pr_err("bad vbattmax = %dmV asked to set\n", vol_mv);
		return -EINVAL;
	}

	val = (vol_mv - BQ24296_CHG_MIN_VCHG)/BQ24296_CHG_VCHG_STEP_MV;
	val = val<<BQ24296_INPUT_VOLT_LIMIT_SHIFT;
	
	BQLOG_DEBUG("set vchgmin[0x02]=%d\n",val);
	
	return bq24296_masked_write(chip, 
		                        BQ24296_REG_SOURCE_CNTL,
		                        BQ24296_INPUT_VOLT_LIMIT_MASK,
		                        val);
}


static int	bq24296_set_chg_iusb(struct bq24296_chg_chip *chip, int iusb_ma)
{
	int rc;
	int i;
	int val;
	u8 read_val;

	for(i=IUSB_100mA;i<IUSB_MAX;i++){
		if(iusb_current[i]==iusb_ma){
			break;
		}
	}
	val = bq_iusb_current_limit[i] << BQ24296_INPUT_CURR_LIMIT_SHIFT;
	
	if ((val <0)||(val > 7)){
		val  = 0x05;
	}
			
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_SOURCE_CNTL,
		                      BQ24296_INPUT_CURR_LIMIT_MASK,
		                      val);
	rc |= bq24296_i2c_readb(chip->i2c, BQ24296_REG_SOURCE_CNTL,&read_val);

	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	BQLOG_DEBUG("REG[0x01] write_iusb=%d--read_iusb=%d\n",val,read_val);
	
	return 0;
}

static int	bq24296_get_chg_iusb(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;
	int iusb;

	rc = bq24296_i2c_readb(chip->i2c, BQ24296_REG_SOURCE_CNTL, &val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_INPUT_CURR_LIMIT_MASK;
	val = val >> BQ24296_INPUT_CURR_LIMIT_SHIFT;

	iusb = iusb_current[val];
	
	BQLOG_DEBUG("REG[0x02]get reg_val=%d--iusb=%d \n",val,iusb);

	return iusb;
}


//Reg 0x1 ---------------------------------------------------------------
static int  bq24296_reset_regs(struct bq24296_chg_chip *chip)
{
	int rc;
	
    	BQLOG_DEBUG("reset_regs\n");
	rc = bq24296_masked_write(chip, BQ24296_REG_POWER_ON_CONFIG, BQ24296_REG_RESET_MASK,BIT(7));
	if (rc) {
		pr_err("bq24296_i2c_write failed: reg=0x%x, rc=%d\n", 0x01, rc);
		return rc;
	}
	return rc;
}

static int  bq24296_reset_wdog(struct bq24296_chg_chip *chip)
{
	int rc ;
	
	rc = bq24296_masked_write(chip, BQ24296_REG_POWER_ON_CONFIG, BQ24296_WD_TIMER_RESET_MASK, BIT(6));
	if (rc) {
		pr_err("bq24296_i2c_readb failed: reg=0x%x, rc=%d\n", 0x01, rc);
		return rc;
	}
	return rc;
}

static int  bq24296_set_otg_mode_enable(struct bq24296_chg_chip *chip,bool flag)
{
	int rc;
	int val = flag<<BQ24296_OTG_EN_SHIFT;

	rc = bq24296_masked_write(chip,BQ24296_REG_POWER_ON_CONFIG,BQ24296_OTG_EN_MASK,val);
	if (rc) {
		pr_err("bq24296_i2c_readb failed: reg=0x%x, rc=%d\n", 0x01, rc);
		return rc;
	}
	return rc;
}

static int  bq24296_get_otg_mode(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;

	rc = bq24296_i2c_readb(chip->i2c,BQ24296_REG_POWER_ON_CONFIG,&val);
	if (rc) {
		pr_err("bq24296_i2c_readb failed: reg=0x%x, rc=%d\n", 0x01, rc);
		return rc;
	}
	
	val &=  BQ24296_OTG_EN_MASK;
	val = val >> BQ24296_OTG_EN_SHIFT;
	return val;
}

static int  bq24296_charge_enable(int enable)
{
	int rc;
	u8 val;

    if (!bq_chip) {
		pr_err("called before init\n");
		return -1;
	}
		
	val = (enable) << BQ24296_CHG_EN_SHIFT;
	rc = bq24296_masked_write(bq_chip,
		                      BQ24296_REG_POWER_ON_CONFIG,
		                      BQ24296_CHG_EN_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int  
bq24296_is_charge_enable(void)
{
	int rc;
	u8 val;

    if (!bq_chip) {
		pr_err("called before init\n");
		return 0;
	}

	rc = bq24296_i2c_readb(bq_chip->i2c,BQ24296_REG_POWER_ON_CONFIG,&val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_CHG_EN_MASK;
	val = val >> BQ24296_CHG_EN_SHIFT;
	
	return (val);
}

static int  
bq24296_set_sys_min_voltage(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;

	val = (SYS_MIN_VOLTAGE_3500MV) << BQ24296_MIN_SYS_VOLT_LIMIT_SHIFT;
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_POWER_ON_CONFIG,
		                      BQ24296_MIN_SYS_VOLT_LIMIT_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int  
bq24296_set_boost_current(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;

	val = (BOOST_CURRENT_1000mA) << BQ24296_BOOST_LIMIT_SHIFT;
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_POWER_ON_CONFIG,
		                      BQ24296_BOOST_LIMIT_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

//Reg 0x2 ---------------------------------------------------------------
static int  
bq24296_set_chg_ibatt(struct bq24296_chg_chip *chip, int ibatt_ma)
{
	u8 val;
	int i ;

	if (chip->charge_sense < 0)
		return -ENOSYS;
	for(i=IBATT_512mA;i<IBATT_MAX;i++){
		if(bq_ibatt_current[i]==ibatt_ma){
			break;
		}
	}
	
	if (i < IBATT_512mA)
		i = IBATT_512mA;
	else if (i >IBATT_3008mA )
		i = IBATT_3008mA;
	
	chip->set_ibatt = bq_ibatt_current[i] ;
	
	val = bq_ibatt_current_limit[i] ;
	
	BQLOG_DEBUG("REG[0x04] set ibatt=%d val=%d charge_sense=%d\n",chip->set_ibatt,val,chip->charge_sense);

	val = val << BQ24296_FAST_CHG_CURR_LIMIT_SHIFT;
	return bq24296_masked_write(chip,  
		                        BQ24296_REG_FAST_CURRENT_CNTL,
		                        BQ24296_FAST_CHG_CURR_LIMIT_MASK,
		                        val);
}

 int bq24296_set_boost_mode_low_temp_monitor(struct bq24296_chg_chip *chip, bool flag )
 {
	u8 val;
	int rc;

	val = flag<<BQ24296_BOOST_COLD_SHIFT;
	rc = bq24296_masked_write(chip, 
							BQ24296_REG_FAST_CURRENT_CNTL,
							BQ24296_BOOST_COLD_MASK,
							val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
 }

//Reg 0x3 ---------------------------------------------------------------
static int  bq24296_set_chg_iterm(struct bq24296_chg_chip *chip, int iterm_ma)
{
	u8 val;

	if (chip->charge_sense < 0)
		return -ENOSYS;

    	//enable charger current termination
    	bq24296_masked_write(chip, 
		                 BQ24296_REG_TIMER_CNTL,
		                 BQ24296_CHG_TERM_EN_MASK,
		                 BIT(7));
	
	val = ibatt_term<<BQ24296_TERM_CURR_LIMIT_SHIFT;
	BQLOG_DEBUG("REG[0x04] set iterm=%d val=%d\n",iterm_ma,val);
	return bq24296_masked_write(chip, 
		                        BQ24296_REG_PRE_AND_TERM_CURRENT_CNTL,
		                        BQ24296_TERM_CURR_LIMIT_MASK,
		                        val);
}

//Reg 0x4 ---------------------------------------------------------------
static int  bq24296_set_vbattmax(struct bq24296_chg_chip *chip, int vbatt)
{
	u8 val;

	if (vbatt < BQ24296_CHG_MIN_VBATT || vbatt > BQ24296_CHG_MAX_VBATT) {
		pr_err("bad vbattmax = %dmV asked to set\n", vbatt);
		return -EINVAL;
	}

	val = (vbatt - BQ24296_CHG_MIN_VBATT)/BQ24296_CHG_VBATT_STEP_MV;
	BQLOG_DEBUG("REG[0x02]set vbattmax=%d val=%d\n",vbatt,val);
	
	val = val << BQ24296_CHG_VOLT_LIMIT_SHIFT;
	return bq24296_masked_write(chip, 
		                        BQ24296_REG_CHARGE_VOLTAGE_CNTL,
		                        BQ24296_CHG_VOLT_LIMIT_MASK,
		                        val);
}

static int  bq24296_get_vbattmax(struct bq24296_chg_chip *chip)
{
	u8 val;
	int rc;
	int vbatt;
	
	rc = bq24296_i2c_readb(chip->i2c, BQ24296_REG_CHARGE_VOLTAGE_CNTL, &val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_CHG_VOLT_LIMIT_MASK;
	val = val >> BQ24296_CHG_VOLT_LIMIT_SHIFT;

	vbatt = BQ24296_CHG_MIN_VBATT + val*BQ24296_CHG_VBATT_STEP_MV;
	
	BQLOG_DEBUG("REG[0x02]get vbattmax=%d val=%d\n",vbatt,val);

	return vbatt;
}


//Reg 0x5 ---------------------------------------------------------------
static int bq24296_set_wdog_timer(struct bq24296_chg_chip *chip,int delay_ms)
{
	int rc;
	u8 val;

	val = delay_ms << BQ24296_WD_TIMER_SHIFT;
	
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_TIMER_CNTL,
		                      BQ24296_WD_TIMER_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int bq24296_chg_safety_timer_enable(struct bq24296_chg_chip *chip,bool flag)
{
	int rc;
	u8 val;

	val = flag << BQ24296_CHG_TIMER_EN_SHIFT;
	
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_TIMER_CNTL,
		                      BQ24296_CHG_TIMER_EN_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int bq24296_set_chg_safety_timer(struct bq24296_chg_chip *chip,int delay_ms)
{
	int rc;
	u8 val = 0;
	
	val = delay_ms << BQ24296_CHG_TIMER_SHIFT;
	
	bq24296_chg_safety_timer_enable(chip,TRUE);
	
	rc = bq24296_masked_write(chip,
		                      BQ24296_REG_TIMER_CNTL,
		                      BQ24296_CHG_TIMER_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

//Reg 0x6 ---------------------------------------------------------------

 int bq24296_set_boost_mode_high_temp_monitor(struct bq24296_chg_chip *chip, bool flag )
{
	u8 val;
	int rc;

	val = flag<<BQ24296_BOOST_TEMP_SHIFT;
	rc = bq24296_masked_write(chip, 
							BQ24296_REG_BOOST_AND_THERM_CNTL,
							BQ24296_BOOST_TEMP_MASK,
							val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

//Reg 0x7 ---------------------------------------------------------------
static int bq24296_chg_set_tmr2x_enable(struct bq24296_chg_chip *chip, bool flag )
{
	u8 val;
	int rc;
	
	 val = flag<<BQ24296_TMR2X_EN_SHIFT;
	 rc = bq24296_masked_write(chip, 
							BQ24296_REG_OPERATION_CNTL,
							BQ24296_TMR2X_EN_MASK,
							val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

/*Batfet Disable 后, 电池将不能充放电*/
 int bq24296_chg_batfet_enable(struct bq24296_chg_chip *chip, bool flag )
{
	u8 val;
	int rc;
	
	 val = flag<<BQ24296_BATFET_EN_SHIFT;
	 rc = bq24296_masked_write(chip, 
							BQ24296_REG_OPERATION_CNTL,
							BQ24296_BATFET_EN_MASK,
							val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

//Reg 0x8 ---------------------------------------------------------------
static int  bq24296_get_vbus_status(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;

	if (!chip) {
		pr_err("called before init\n");
		return UNKNOW_CONFIG;
	}

	rc = bq24296_i2c_readb(chip->i2c,BQ24296_REG_SYSTEM_STATUS,&val);
    	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_VBUS_STATUS_MASK;
	val = val >> BQ24296_VBUS_STATUS_SHIFT;

	return val;
}

static int  bq24296_get_chg_status(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 val;

	if (!chip) {
		pr_err("called before init\n");
		return BQ_READY_CHGING;
	}

	rc = bq24296_i2c_readb(chip->i2c,BQ24296_REG_SYSTEM_STATUS,&val);
    	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_CHG_STATUS_MASK;
	val = val >> BQ24296_CHG_STATUS_SHIFT;
	
	return val;
}

//Reg 0x9 ---------------------------------------------------------------
//static 
int  bq24296_is_wdog_timeout(void)
{
	int rc;
	u8 val;

	if (!bq_chip) {
		pr_err("called before init\n");
		return FALSE;
	}

	rc = bq24296_i2c_readb(bq_chip->i2c,BQ24296_REG_FAULT_STATUS,&val);
    	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_WD_FAULT_MASK;
	val = val >> BQ24296_WD_FAULT_SHIFT;
	
	return val;
}

 int  bq24296_get_otg_fault_status(void)
{
	int rc;
	u8  val;

	if (!bq_chip) {
		pr_err("called before init\n");
		return FALSE;
	}

	rc = bq24296_i2c_readb(bq_chip->i2c,BQ24296_REG_FAULT_STATUS,&val);
    	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_OTG_FAULT_MASK;
	val = val >> BQ24296_OTG_FAULT_SHIFT;
	
	return val;
}

 int  bq24296_get_chg_fault_status(void)
{
	int rc;
	u8  val;

	if (!bq_chip) {
		pr_err("called before init\n");
		return 0;
	}

	rc = bq24296_i2c_readb(bq_chip->i2c,BQ24296_REG_FAULT_STATUS,&val);
    	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	val &= BQ24296_CHG_FAULT_MASK;
	val = val >> BQ24296_CHG_FAULT_SHIFT;
	
	return val;
}

static int bq24296_set_boost_mode_temp_monitor(struct bq24296_chg_chip *chip)
{
	int ret;
	
	ret = bq24296_set_boost_mode_high_temp_monitor(chip, TEMP_55_DEGREE);
	ret |= bq24296_set_boost_mode_low_temp_monitor(chip,TEMP_SUBZERO_10_DEGREE);
	
	return ret;
}

//----------------------------------------------------------------------
static void bq24296_dump_regs(struct bq24296_chg_chip *chip)
{
	u8 val;
	int i;
	
	printk("%s:start ",__func__);

  	if (PM_INFO > bqlog_level)
		return;
	for(i=0;i<10;i++){
		bq24296_i2c_readb(chip->i2c,i,&val);
		printk("0x%x ",val);
	}
	
	printk(" end\n");
}

//battery  power supply property----------------------------------------------------------
static void bq24296_update_power_supply(struct bq24296_chg_chip *chip)
{
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy > 0)
		power_supply_changed(chip->batt_psy);
}

static int bq24296_get_prop_batt_present(struct bq24296_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
	
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(chip->batt_psy)
  		chip->batt_psy->get_property(chip->batt_psy,
			    POWER_SUPPLY_PROP_PRESENT, &ret);
	
	return ret.intval;
}

#define DEFAULT_BATT_TEMP		250
static int  bq24296_get_prop_batt_temp(struct bq24296_chg_chip *chip)
{
	int ret = 0;
	struct qpnp_vadc_result results;
	
	if (!chip->batt_present){
		return DEFAULT_BATT_TEMP;
	}

	ret = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (ret) {
		pr_info("Unable to read batt temperature rc=%d\n", ret);
		return DEFAULT_BATT_TEMP;
	}
	
	pr_debug("adc_code=%d,temp=%lld\n", results.adc_code,results.physical);

	return (int)results.physical;
}

static int bq24296_get_prop_charger_voltage(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read charger rc=%d\n", rc);
		return 0;
	}
	return (int)results.physical/1000;
		
}

static int bq24296_get_prop_battery_voltage(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}

	return (int)results.physical/1000;
}

static int bq24296_get_prop_health_stauts(void)
{
	int batt_temp;
	int temp_status;

	if (!bq_chip) {
	    pr_err("called before init\n");
		return BATT_STATUS_GOOD;
	}

	if(!bq_chip->in_work){
		batt_temp = bq24296_get_prop_batt_temp(bq_chip);
		bq24296_get_temp_status(batt_temp_map, ARRAY_SIZE(batt_temp_map), batt_temp, &temp_status);
		return temp_status;
	}

    return bq_chip->temp_status;
}

 int bq24296_get_prop_batt_health(void)
{
    int batt_temp_status;
	batt_temp_status = bq24296_get_prop_health_stauts();

	if(batt_temp_status == BATT_STATUS_HOT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if(batt_temp_status == BATT_STATUS_COLD)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

 int bq24296_get_prop_is_charge_enable(void)
{
	return bq24296_is_charge_enable();
}

int bq24296_get_prop_set_charge_enable(void)
{
	int rc;
	
	if (!bq_chip) {
	       pr_err("called before init\n");
		return 0;
	}
	
	rc = bq24296_hiz_mode_enable(bq_chip,TRUE);
	rc |= bq24296_hiz_mode_enable(bq_chip,FALSE);
	rc |= bq24296_charge_enable(TRUE);

	if (rc) {
		pr_err("%set chg enable failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int bq24296_update_batt_param(struct bq24296_chg_chip *chip)
{
    union power_supply_propval ret = {0,};
	
	chip->batt_temp = bq24296_get_prop_batt_temp(chip);
	chip->chg_vol = bq24296_get_prop_charger_voltage(chip);
	chip->batt_vol = bq24296_get_prop_battery_voltage(chip);
	chip->batt_present = bq24296_get_prop_batt_present(chip);

	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(!chip->batt_psy)
		return -1;

       chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&ret);
	chip->batt_i = ret.intval/1000;

	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
	chip->batt_soc = ret.intval;
	
	BQLOG_DEBUG("batt_temp=%d chg_status=%d  batt_i=%d  batt_vol=%d  batt_soc=%d  chg_vol=%d batt_present =%d\n",
		chip->batt_temp,chip->chg_status,chip->batt_i,chip->batt_vol,chip->batt_soc,chip->chg_vol,chip->batt_present );

	return 0;
}

//Module Interface---------------------------------------------------------------------- 
static int bq24296_is_charger_online(void)
{
	int chg_online = 0;
	
	if (!bq_chip) {
	       pr_err("called before init\n");
		return chg_online;
	}
	
	chg_online = 	bq24296_get_vbus_status(bq_chip);
	if(chg_online== CHG_AC_CONFIG||chg_online==CHG_USB_CONFIG){
		chg_online = 1;
	}
    return bq_chip->usb_in ||chg_online;
}

int  bq24296_notify_charger(enum usb_chg_type chg_type)
{	
    	int set_iusb;
	int ret;
	
	if (!bq_chip) {
		pr_err("called before init\n");
		return -1;
	}
	
	if(chg_type == USB_DCP_CHARGER 
	|| chg_type == USB_CDP_CHARGER 
	|| chg_type == USB_FLOATED_CHARGER
	|| chg_type == USB_PROPRIETARY_CHARGER){
		bq_chip->chg_cycle_timer = CHG_SAFETY_TIME_5_H;
		set_iusb = IUSB_1500mA;
	}
	else{
		bq_chip->chg_cycle_timer = CHG_SAFETY_TIME_12_H;
		set_iusb = IUSB_500mA;
	}
	
	 bq_chip->iusbmax_ma = iusb_current[set_iusb];
	bq_chip->chg_type = chg_type;
	BQLOG_INFO("chg->chg_type=%d iusb=%d\n",chg_type,bq_chip->iusbmax_ma);

	ret = bq24296_chg_hw_init(bq_chip);
	
	if (ret) {
		pr_err("%s	failed: rc=%d\n", __func__,ret);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq24296_notify_charger);

#define BATT_TEMTP_DELTA    20
#define TEMP_INIT    -500
static int bq24296_get_temp_status(const struct temp_status_map *pts,
		uint32_t tablesize, int input, int *batt_temp_status)
{
	static int current_index = 0;
	static int init_status = 1;

	if ( pts == NULL || batt_temp_status == NULL)
		return BATT_STATUS_UNKNOW;
		
	if(init_status){
		while (current_index < tablesize) {
			if ( (pts[current_index].low_temp <= input) && (input <= pts[current_index].high_temp) ) 
				break;
			else 
				current_index++;
		}
		init_status = 0;
		BQLOG_DEBUG("-First-input=%d  current_index=%d \n",input,current_index);
	}else{
		if(input < (pts[current_index].low_temp - BATT_TEMTP_DELTA))
			current_index--;
		else if(input > pts[current_index].high_temp)
			current_index++;
	}

    if(current_index < 0)
		*batt_temp_status = BATT_STATUS_COLD;
	else if(current_index >= tablesize)
		*batt_temp_status = BATT_STATUS_HOT;
	else
		*batt_temp_status = pts[current_index].batt_st;

	BQLOG_DEBUG("input=%d  batt_temp_status=%d \n",input,*batt_temp_status);
	
	return current_index;

}

static void bq24296_chg_temp_cntl(struct bq24296_chg_chip *chip,int batt_temp)
{
    	int temp_status;
	int batt_current = 0;
	int state_index;

	state_index = bq24296_get_temp_status(batt_temp_map,
		                                     ARRAY_SIZE(batt_temp_map),
		                                     batt_temp,
		                                     &temp_status);
	if(temp_status != BATT_STATUS_UNKNOW)
	    batt_current = batt_temp_map[state_index].batt_current;
		
	if(temp_status != chip->temp_status && chip->usb_in){
		BQLOG_INFO("last chip->temp_status=%d new_temp_status=%d\n",chip->temp_status,temp_status);
		if(batt_current > 0){
		    bq24296_charge_enable(TRUE);
			bq24296_set_chg_ibatt(chip,batt_current);
			chip->temp_abnormal = FALSE;
			
		    BQLOG_INFO("batt_temp=%d temp_status=%d batt_current=%d start charging...\n",batt_temp,temp_status,batt_current);
		}else{
			bq24296_charge_enable(FALSE);
			chip->temp_abnormal = TRUE;
			
			BQLOG_INFO("batt_temp=%d out of rangge,stop charging!\n",batt_temp);
		}
		bq24296_update_power_supply(chip);
	}
	
	chip->temp_status = temp_status;

}

static void bq24296_recharging_enable(struct bq24296_chg_chip *chip)
{
	 enum bq24296_chg_status chg_status;
	 chg_status = bq24296_get_chg_status(chip);

	if(chg_status ==CHARGE_STATE_CHG_DONE){
		bq24296_hiz_mode_enable(chip,TRUE);
		mdelay(3);
		bq24296_hiz_mode_enable(chip,FALSE);
		BQLOG_DEBUG (" :recharge  start\n");		
	}	
}

static void bq24296_chg_status_reset(struct bq24296_chg_chip *chip)
{	
	chip->temp_abnormal = FALSE;
	chip->soc_chg_done = FALSE;
	chip->is_chg_full = FALSE;
	chip->temp_status = BATT_STATUS_UNKNOW;
	chip->chg_type = USB_INVALID_CHARGER;
}

#define CONSECUTIVE_COUNT	3
static int bq24296_check_recharge_condition(struct bq24296_chg_chip* chip)
{
	static int count = 0;
	enum bq24296_chg_status chg_status;
	chg_status = bq24296_get_chg_status(chip);

	if(chg_status ==CHARGE_STATE_CHG_DONE){
		if(chip->batt_vol < chip->rechg_vol){
			count++;
		}
		if(count > CONSECUTIVE_COUNT){
			count = 0;
		}
	}else{
		if(count){
			count = 0;
		}	
	}
	
	BQLOG_DEBUG (" count==%d--chg_status==%d\n", count,chg_status);
	
	return (count == CONSECUTIVE_COUNT)||(chip->batt_soc <= chip->rechg_soc);
}

#define CHG_NORMAL_PERIOD_MS	  10000
#define CHG_QUICK_PERIOD_MS	      1000
#define CHG_WORK_CHECK_NUM 	      3

static void bq24296_chg_worker(struct work_struct *work)
{
    int ret;
	struct delayed_work *cwork = to_delayed_work(work);
	struct bq24296_chg_chip *chip = container_of(cwork,	struct bq24296_chg_chip,chg_work);
	int set_vbattmax = bq24296_get_vbattmax(chip);
	int set_usb_current = bq24296_get_chg_iusb(chip);
	int chg_status;
	int delay_time_ms = CHG_NORMAL_PERIOD_MS;
	static int usbin_cnt;


	bq24296_reset_wdog(chip);
	chip->config_mode = bq24296_get_vbus_status(chip);
	
	if(!chip->usb_in)
		goto out_work;

	if(chip->config_mode == OTG_CONFIG){
		if(wake_lock_active(&chip->wlock)){
			wake_unlock(&chip->wlock);
			BQLOG_INFO("free the chip->wlock");
		}
		if(!bq24296_get_otg_mode(chip) )
			bq24296_set_otg_mode_enable(chip,TRUE);
		goto next;
	}
	
	if(!chip->in_work){
		chip->set_ibatt = chip->ibatmax_ma;
		bq24296_chg_hw_init(chip);
		chip->in_work = true;
		usbin_cnt = CHG_WORK_CHECK_NUM;
	}else if((set_vbattmax != chip->vbatt_max)||(set_usb_current != chip->iusbmax_ma)){
	   	bq24296_chg_hw_init(chip);
	}

	ret = bq24296_update_batt_param(chip);
	if(ret < 0)
		goto next;
		
	bq24296_chg_temp_cntl(chip,chip->batt_temp);

	chg_status = bq24296_get_prop_chg_status();
	if(chip->chg_status != chg_status){
		BQLOG_DEBUG("new charge status=%d\n",chg_status);
		bq24296_update_power_supply(chip);
		chip->chg_status = chg_status;
		if(chip->chg_status==POWER_SUPPLY_STATUS_FULL){
			chip->is_chg_full = TRUE;
		}
	}

    	if(chg_status == POWER_SUPPLY_STATUS_UNKNOWN){
		if(usbin_cnt){
			usbin_cnt--;
			delay_time_ms = CHG_QUICK_PERIOD_MS;
		}
    	}else{
        	usbin_cnt = 0;
	}
	
      if(chip->is_chg_full){
		if(bq24296_check_recharge_condition(chip)){
			bq24296_recharging_enable(chip);
		}
	  }

      bq24296_dump_regs(chip);
	BQLOG_INFO("usb_in=%d chg_st=%d temp_status=%d chg_vol=%d\n",
		        chip->usb_in,chip->chg_status,chip->temp_status,chip->chg_vol);
	
next:
	schedule_delayed_work(&chip->chg_work,
		round_jiffies_relative(msecs_to_jiffies(delay_time_ms)));
	return;

out_work:
	chip->in_work = false;
	bq24296_chg_status_reset(chip);
	chip->iusbmax_ma =  iusb_current[IUSB_500mA];
	bq24296_set_chg_iusb(chip,chip->iusbmax_ma);
	bq24296_set_wdog_timer(chip,  DISABLE_WD_TIMER);
	
	if(wake_lock_active(&chip->wlock)){
		wake_unlock(&chip->wlock);
		BQLOG_INFO("free the chip->wlock");
	}

}

static void bq24296_usbin_worker(struct work_struct *work)
{
	struct bq24296_chg_chip *chip = container_of(work, struct bq24296_chg_chip, usbin_work);

	if(chip->usb_in){
		if(!wake_lock_active(&chip->wlock)){
			wake_lock(&chip->wlock);
		}	
		cancel_delayed_work_sync(&chip->chg_work);
		chip->in_work = false;
		schedule_delayed_work(&chip->chg_work, round_jiffies_relative(msecs_to_jiffies(0)));
	}
	
	return;
}


int  bq24296_start_chg_work(int usb_in)
{
	BQLOG_INFO("usb_in=%d\n",usb_in);
		
	if (!bq_chip) {
		pr_err("called before init\n");
		charger_online = usb_in;
		return -1;
	}  

    	bq_chip->usb_in = usb_in;  	
	schedule_work(&bq_chip->usbin_work);
	
 	return 0;
}
EXPORT_SYMBOL_GPL(bq24296_start_chg_work);

static int debug_reg;
module_param(debug_reg, int, 0644);

static int bq24296_debug;
static int bq24296_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    int i;
    u8 buf;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	
	printk("__%s: bq24296_debug=%d!\n",__func__,bq24296_debug);

	if(bq24296_debug < 0){
		buf = abs(bq24296_debug);
        bq24296_i2c_writeb(bq_chip->i2c,debug_reg,buf);
		return 0;
	}
		
	switch(bq24296_debug){
    case 0:
		bq24296_charge_enable(0);
		break;
	case 1:
		bq24296_charge_enable(1);
		break;
	case 2:
		bq24296_set_chg_ibatt(bq_chip,ibat_current);
		break;
	case 3:
		//bq24296_get_dev_info(bq_chip);
		break;
	case 4:
		for(i=0;i<9;i++){
			bq24296_i2c_readb(bq_chip->i2c,i,&buf);
            printk("---reg[%d] buf=0x%x\n",i,buf);
		}
		break;
    case 5:
		bq24296_reset_regs(bq_chip);
		break;
	case 6:
		bq24296_set_chg_iusb(bq_chip,1000);
		break;
	case 7:
		bq24296_hiz_mode_enable(bq_chip,0);
		break;
	case 8:
		bq24296_hiz_mode_enable(bq_chip,1);
		break;
	case 10:
		bq24296_chg_gpio_enable(bq_chip,0);
		break;
	case 11:
		bq24296_chg_gpio_enable(bq_chip,1);
		break;
	default:
		break;
	};
	printk("__%s: debug end\n",__func__);
	return 0;
}
module_param_call(bq24296_debug, bq24296_debug_mode, param_get_uint,
					&bq24296_debug, 0644);


/****************************************************************\
 Base Function 
\****************************************************************/

int bq24296_is_usb_present(struct bq24296_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
	
	if (!chip->usb_psy){
		chip->usb_psy = power_supply_get_by_name("usb");
	}
	
	if (!chip->usb_psy){
		chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_ONLINE, &ret);
	}

	 return ret.intval || bq24296_is_charger_online();
}



static int bq24296_get_prop_chg_status(void)
{
 	enum bq24296_chg_status chg_status;
	int batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
	
  	if(!bq_chip){
  		return POWER_SUPPLY_STATUS_UNKNOWN;
	}	
  
	//if (!bq_chip->batt_present)
	//	return POWER_SUPPLY_STATUS_NOT_CHARGING;

  	if(!bq24296_is_usb_present(bq_chip)){
  		return POWER_SUPPLY_STATUS_DISCHARGING;
  	}
	chg_status = bq24296_get_chg_status(bq_chip);
	
	if (chg_status == CHARGE_STATE_NO_CHG){
		batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}	
	else if (chg_status == CHARGE_STATE_CHG_DONE){
		batt_status = POWER_SUPPLY_STATUS_FULL;
	}
	else if(( (chg_status == CHARGE_STATE_PRE_CHG) ||
	          	(chg_status == CHARGE_STATE_FAST_CHG))||
	          	bq_chip->usb_in){
		batt_status = POWER_SUPPLY_STATUS_CHARGING;
	}		
  	else{
  		 return POWER_SUPPLY_STATUS_NOT_CHARGING;
  	}
  
 	pr_debug("chg_status = %d \n" , chg_status);  
 
	return batt_status;

}

static int
 bq24296_get_batt_property(struct power_supply *psy,
				  enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = bq24296_get_prop_chg_status();
			break;
	
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = bq24296_get_prop_batt_health();
			break;

		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = bq24296_get_prop_is_charge_enable();
			break;
		
		default:
			return -EINVAL;

	}


	pr_debug("%s psp=%d intval=%d\n",__func__,psp,val->intval);
	return rc;
}

static int
 bq24296_set_batt_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	int rc = 0;

	pr_debug("%s psp=%d intval=%d\n",__func__,psp,val->intval);
	return rc;
}

int bq24296_set_power_supply(struct power_supply *batt_psy)
{
    if(!batt_psy)
		return -EINVAL;

	batt_psy->properties = bq24296_batt_props;
  	batt_psy->num_properties = ARRAY_SIZE(bq24296_batt_props);
	batt_psy->get_property = bq24296_get_batt_property;
	batt_psy->set_property = bq24296_set_batt_property;

	return 0;
}

static int
bq24296_charger_read_dt_props(struct bq24296_chg_chip *chip)
{
    int rc;
	rc = of_property_read_u32(chip->dev_node, "bq24296,vusb-min", &chip->vusb_min);
	if (rc) {
		pr_err( "Unable to parse 'bq-vusb-min'\n");
		return rc;
	}
	
	rc = of_property_read_u32(chip->dev_node, "bq24296,v-cutoff-mv", &chip->vol_cutoff);
	if (rc) {
		pr_err( "Unable to parse 'v-cutoff-m'\n");
		return rc;
	}
	
	rc = of_property_read_u32(chip->dev_node, "bq24296,ibatmax-ma", &chip->ibatmax_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-ibatmax-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,ibatterm-ma", &chip->iterm_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-iterm-ma'\n");
		return rc;
	}


	rc = of_property_read_u32(chip->dev_node, "bq24296,iusbmax-ma", &chip->iusbmax_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-initusb-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,vbattmax-mv", &chip->vbatt_max);
	if (rc) {
		pr_err( "Unable to parse 'vbatmax-mv'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,bq-charge-sense", &chip->charge_sense);
	if (rc) {
		pr_err( "Unable to parse 'bq-charge-sense'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,max-charger-cycle-mins", &chip->chg_cycle_timer);
	if (rc) {
		pr_err( "Unable to parse 'chg_cycle_timere'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,bq-recharge-soc", &chip->rechg_soc);
	if (rc) {
		pr_err( "Unable to parse 'bq-recharge-soc'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq24296,bq-recharge-vol", &chip->rechg_vol);
	if (rc) {
		pr_err( "Unable to parse 'bq-recharge-vol'\n");
		return rc;
	}
	
	chip->chg_en_gpio = of_get_named_gpio_flags(chip->dev_node,"qcom,en-gpio", 0, NULL);
	if (chip->chg_en_gpio < 0){
		pr_err( "Unable to parse 'chg_en_gpio'\n");
		return chip->chg_en_gpio;
	}
#ifdef GPIO_IRQ		
	chip->irq_gpio = of_get_named_gpio_flags(chip->dev_node, "qcom,irq-gpio",0, NULL);
	if (chip->irq_gpio < 0){
		pr_err( "Unable to parse 'irq_gpio'\n");
		return chip->irq_gpio;
	}
#endif
	return 0;
}

 int bq24296_gpio_init(struct bq24296_chg_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->chg_pinctrl = devm_pinctrl_get(&(chip->i2c->dev));
	if (IS_ERR_OR_NULL(chip->chg_pinctrl)) {
		pr_err(	"Target does not use pinctrl! \n");
		retval = PTR_ERR(chip->chg_pinctrl);
		chip->chg_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_active
		= pinctrl_lookup_state(chip->chg_pinctrl,"bq24296_default");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		pr_err("Can not get ba24296 default pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_active);
		chip->chg_pinctrl = NULL;
		return retval;
	}
	
	//pr_info("Charge CE PinCtrl Init Success! \n");
	return 0;
}

static int bq24296_chg_hw_init(struct bq24296_chg_chip *chip)
{
    int ret;

	ret = bq24296_set_chg_vol(chip,chip->vusb_min);
	if (ret) {
		pr_err("failed to set spec chg vol rc=%d\n", ret);
		return ret;
	}
	
	ret = bq24296_set_chg_ibatt(chip,chip->set_ibatt);
	if (ret) {
		pr_err("failed set charging ibatt rc=%d\n", ret);
		return ret;
	}
	
       ret = bq24296_set_chg_iterm(chip,chip->iterm_ma);
	if (ret) {
		pr_err("failed set charging iterm rc=%d\n", ret);
		return ret;
	}

	ret = bq24296_set_vbattmax(chip,chip->vbatt_max);
	if (ret) {
		pr_err("failed set charging vbattmax rc=%d\n", ret);
		return ret;
	}
	
	ret = bq24296_set_chg_iusb(chip,chip->iusbmax_ma);
	if (ret) {
		pr_err("failed set charging iusbmax_ma rc=%d\n", ret);
		return ret;
	}
	
	ret = bq24296_set_sys_min_voltage(chip);
	if (ret) {
		pr_err("failed set systerm min voltage sys_min_voltage rc=%d\n", ret);
		return ret;
	}

	ret = bq24296_set_boost_current(chip);
	if (ret) {
		pr_err("failed set boost_current rc=%d\n", ret);
		return ret;
	}

	ret = bq24296_set_wdog_timer(chip,DELAY_160_S);
	if (ret) {
		pr_err("failed set systerm min voltage sys_min_voltage rc=%d\n", ret);
		return ret;
	}
	
	ret = bq24296_set_chg_safety_timer(chip,chip->chg_cycle_timer);
	if (ret) {
		pr_err("failed set systerm min voltage sys_min_voltage rc=%d\n", ret);
		return ret;
	}
	
	ret = bq24296_hiz_mode_enable(chip,FALSE);
	if (ret){ 
		pr_err("failed disable hiz mode rc=%d\n", ret);
	}

	ret = bq24296_charge_enable(TRUE);
	if (ret){ 
		pr_err("failed enable charging rc=%d\n", ret);
	}

	ret = bq24296_set_boost_mode_temp_monitor(chip);
	if (ret) {
		pr_err("failed enable charging rc=%d\n", ret);
	}
	
	ret = bq24296_chg_set_tmr2x_enable(chip,FALSE);
	if (ret) {
		pr_err("failed enable charging rc=%d\n", ret);
	}
	
	bq24296_reset_wdog(chip);
	
	return 0;
}

static int bq24296_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	BQLOG_INFO("To Enable OTG \n" );
	
	bq24296_chg_gpio_enable(chip,FALSE);

	rc = bq24296_set_boost_mode_temp_monitor(chip);
 	rc = bq24296_set_otg_mode_enable(chip,TRUE);
 	rc |= bq24296_charge_enable(FALSE);
	if(rc){
		pr_err( "Fail to enable vbus_otg regulator.\n");
	}
	
	return rc;
}

static int bq24296_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;
	
	BQLOG_INFO("To Disable OTG \n" );
	
	bq24296_chg_gpio_enable(chip,TRUE);

 	rc = bq24296_set_otg_mode_enable(chip,FALSE);
	rc |= bq24296_charge_enable(TRUE);
	if(rc){
		pr_err( "Fail to disable vbus_otg regulator.\n");
	}
   
	return rc;
}

static int bq24296_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int in_boost;
	
	in_boost = bq24296_get_otg_mode(chip);
	
	BQLOG_INFO("in_boost=%d\n",in_boost);
	
	return in_boost;
}

struct regulator_ops bq24296_otg_reg_ops = {
	.enable		= bq24296_otg_regulator_enable,
	.disable	=  bq24296_otg_regulator_disable,
	.is_enabled	= bq24296_otg_regulator_is_enable,
};

static int bq24296_regulator_init(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		pr_err( "Unable to allocate memory\n");
		return -ENOMEM;
	}

	BQLOG_INFO( "regulator name : %s \n", init_data->constraints.name);
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &bq24296_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				pr_err(	"OTG reg failed, rc=%d\n", rc);
		}
	}
  	BQLOG_INFO( "regulator name register success  : %s \n", init_data->constraints.name);

	return rc;
}

static int  bq24296_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct bq24296_chg_chip *chip;
	int ret;

	chip = kzalloc(sizeof(struct bq24296_chg_chip),GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate bq24296_chg_chip\n");
		return -ENOMEM;
	}
	
    	BQLOG_DEBUG("enter driver probe:\n");
	
	chip->i2c = client;
	chip->dev = &client->dev;
	chip->dev_node = client->dev.of_node;
	chip->in_work = 0;
	chip->temp_abnormal = 0;
	chip->in_rechging = 0;
	chip->temp_status = BATT_STATUS_UNKNOW;
	chip->vadc_dev = qpnp_get_vadc(&chip->i2c->dev, "bq24296");

	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret == -EPROBE_DEFER){
			pr_err("vadc not found - defer ret=%d\n", ret);
			return ret;
		}else
			pr_err("vadc property missing, ret=%d\n", ret);
	}
	
	bq24296_charger_read_dt_props(chip);
	bq_chip = chip;  

	chip->set_ibatt = chip->ibatmax_ma;
	ret = bq24296_chg_hw_init(chip);
	if (ret) 
		pr_err("bq24296 hw init fail, ret=%d\n",ret); 

	ret =  bq24296_regulator_init(chip);
	if (ret) {
		if (ret == -EPROBE_DEFER){
			pr_err("regulator not found - defer ret=%d\n", ret);
			return ret;
		}else
			pr_err("regulator property missing, ret=%d\n", ret);
	}

	
	wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "bq24296_charger");
	INIT_DELAYED_WORK(&chip->chg_work, bq24296_chg_worker);
	INIT_WORK(&chip->usbin_work, bq24296_usbin_worker);

	//----------------------------------------------
	ret = bq24296_gpio_init(chip);
	if (ret) {
		pr_err("failed init gpio ret=%d\n", ret);
	}
	
	bq24296_chg_gpio_select(chip,TRUE);
	
	if (gpio_is_valid(chip->chg_en_gpio)) {
		ret = gpio_request(chip->chg_en_gpio, "bq24296_en_gpio");
		if (ret) {
			pr_err("%s:Fail To Request GPIO %d (%d)\n",__func__, chip->chg_en_gpio, ret);
			goto err_en_gpio;
		}
		bq24296_chg_gpio_enable(chip,TRUE);
	}
#ifdef GPIO_IRQ		
	if (gpio_is_valid(chip->irq_gpio)) {
		ret = gpio_request(chip->irq_gpio, "bq24296_irq");
		if (unlikely(ret < 0)) {
			dev_err(&client->dev, "gpio request failed");
		}
		ret = gpio_direction_input(chip->irq_gpio);
		 
		chip->irq = gpio_to_irq(chip->irq_gpio);	
		if (unlikely(chip->irq < 0)) {
			dev_err(&client->dev, "gpio request to isr failed");
			goto err_irq_gpio;
		}

		ret = request_irq(chip->irq, bq24296_status_warning_irq_handler,
				IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "bq24296_irq", chip);
		if (unlikely(ret < 0)) {
			dev_err(&client->dev, "request_irq failed\n");	
			goto err_irq;
		}
	}
#endif	

	chip->bq24296_batt_psy.name = "bq24296m-battery";
	chip->bq24296_batt_psy.type = POWER_SUPPLY_TYPE_EXT_CHG;
	chip->bq24296_batt_psy.properties = bq24296_batt_props;
	chip->bq24296_batt_psy.num_properties = ARRAY_SIZE(bq24296_batt_props);
	chip->bq24296_batt_psy.get_property = bq24296_get_batt_property;
	chip->bq24296_batt_psy.set_property = bq24296_set_batt_property;
	ret = power_supply_register(chip->dev, &chip->bq24296_batt_psy);
	if (ret < 0) {
		pr_err("power_supply_register bq24296_batt_psy failed rc = %d\n", ret);
		goto fail_psy;
	}


	bq24296_set_wdog_timer(chip,  DISABLE_WD_TIMER);
	if(bq24296_is_usb_present(chip)||charger_online){
		bq24296_start_chg_work(TRUE);
		bq24296_set_wdog_timer(chip,DELAY_40_S);
		if(!wake_lock_active(&bq_chip->wlock)){
			wake_lock(&bq_chip->wlock);
		}	
		BQLOG_DEBUG("start chg work:\n");
	}

	BQLOG_INFO("enter driver probe:end\n");

    return 0;

fail_psy:
#ifdef GPIO_IRQ		
err_irq:
	free_irq(chip->irq, chip);

err_irq_gpio:	
	gpio_free(chip->irq_gpio);	
#endif	
err_en_gpio:
	gpio_free(chip->chg_en_gpio);
	cancel_delayed_work(&chip->chg_work);
	wake_lock_destroy(&chip->wlock);
	kfree(chip);
	bq_chip = NULL;
	return 0;

	
}

static int bq24296_charger_remove(struct i2c_client *client)
{	
	struct bq24296_chg_chip *chip = i2c_get_clientdata(client);
#ifdef GPIO_IRQ		
	free_irq(chip->irq, chip);
	gpio_free(chip->irq_gpio);	
#endif	
	gpio_free(chip->chg_en_gpio);	
	cancel_delayed_work(&chip->chg_work);
	wake_lock_destroy(&chip->wlock);
	kfree(chip);	
	bq_chip = NULL;
	return 0;
}

static int bq24296_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	BQLOG_DEBUG(" suspend:\n");

	return 0;
};

static int bq24296_resume(struct i2c_client *cl)
{
	BQLOG_DEBUG(" resume:\n");

	return 0;
};

static struct of_device_id bq_24296_match_table[] = {
	{ .compatible = "ti,bq24296-chg",},
	{}
};

static const struct i2c_device_id bq24296_id[] = {
	{ "bq24296", 1 },
	{},
};

static struct i2c_driver bq24296_charger_driver = {
	.driver = {
		.name = "bq24296",
		.of_match_table = bq_24296_match_table,
	},
	.id_table 	= bq24296_id,
	.probe 		= bq24296_charger_probe,
	.remove 	= bq24296_charger_remove,

	.suspend	= bq24296_suspend,
	.resume 	= bq24296_resume,
};

static int __init bq24296_charger_init(void)
{
	printk( "%s:enter...\n", __func__);

	return i2c_add_driver(&bq24296_charger_driver);
}

static void __exit bq24296_charger_exit(void)
{
	printk( "%s:bq24296 is exiting\n", __func__);

	i2c_del_driver(&bq24296_charger_driver);
}

module_init(bq24296_charger_init);
module_exit(bq24296_charger_exit);

MODULE_AUTHOR("ztemt-ruiguang<jing.ruiguang@zte.com.cn>");
MODULE_DESCRIPTION("bq24296 charger driver");
MODULE_LICENSE("GPL");

