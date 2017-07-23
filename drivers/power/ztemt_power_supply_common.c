/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"ZPSY: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#include "ztemt_power_supply_common.h"

#define ZTEMT_MAX_POWER_PSY 5
struct ztemt_power_psy_type{
    struct power_supply  *power_psy[ZTEMT_MAX_POWER_PSY];
	struct power_supply   main_power_psy;
	int ext_psy_num;
	enum power_supply_property power_props[50];
	size_t props_num;
};
static struct ztemt_power_psy_type ztemt_ext_power_psy;

static char *ztemt_power_psy_name[ZTEMT_MAX_POWER_PSY]={
	#ifdef CONFIG_ZTEMT_BQ27520_BATTERY
	"bq27520-battery",
	#endif
	
	#ifdef CONFIG_ZTEMT_BQ24296_CHARGE
	"bq24296-battery",
	#endif

	#ifdef CONFIG_ZTEMT_BQ24296M_CHARGE
	"bq24296m-battery",
	#endif
	
	'\0',
};

//----------------------------------------------------------------------
//  !!!NOTE!!!:  if it not support  the  enum power_supply_property,    
// the set_property()/get_propery() MUST return -EINVAL, else it return 0.                                                                            
//-----------------------------------------------------------------------
static int is_property_inside_psy(struct power_supply *psy,  enum power_supply_property psp)
{
    int i;

    for(i=0; i<psy->num_properties; i++){
		if(psp == psy->properties[i])
			return 1;
	}

	return 0;
}

static int ztemt_batt_power_get_property(struct power_supply *batt_psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
    struct power_supply *temp_psy;
	int ret = -EINVAL;
    int i;

	for(i=0; i<ztemt_ext_power_psy.ext_psy_num; i++){
		temp_psy = ztemt_ext_power_psy.power_psy[i];
		if(temp_psy && is_property_inside_psy(temp_psy,psp)){
			ret = temp_psy->get_property(temp_psy, psp, val);
			if(!ret)
				return 0;
		}
	}

	if(is_property_inside_psy(&ztemt_ext_power_psy.main_power_psy, psp))
		ret = ztemt_ext_power_psy.main_power_psy.get_property(batt_psy, psp, val);

	return ret;
}

static int ztemt_batt_power_set_property(struct power_supply *batt_psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
    struct power_supply *temp_psy;
	int ret = -EINVAL;
	int i;

    for(i=0; i<ztemt_ext_power_psy.ext_psy_num; i++){
		temp_psy = ztemt_ext_power_psy.power_psy[i];
		if(temp_psy && is_property_inside_psy(temp_psy,psp)){
       		ret = temp_psy->set_property(temp_psy, psp, val);
			if(!ret)
				return 0;
		}
	}
	
	if(is_property_inside_psy(&ztemt_ext_power_psy.main_power_psy, psp))
    	ret = ztemt_ext_power_psy.main_power_psy.set_property(batt_psy, psp, val);

	return ret;
}

int ztemt_chg_batt_prop_init( struct power_supply *batt_psy)
{
	size_t props_lens = 0;
	int enum_len = sizeof(enum power_supply_property);
	struct power_supply *temp_psy;
    int i;

    //get ext power supply
    for(i=0; i<ZTEMT_MAX_POWER_PSY; i++){
		if(ztemt_power_psy_name[i]){
            temp_psy = power_supply_get_by_name(ztemt_power_psy_name[i]);
			if(!temp_psy)
				return -EPROBE_DEFER;
			
			memcpy(ztemt_ext_power_psy.power_props + props_lens, 
			       temp_psy->properties, 
			       temp_psy->num_properties * enum_len);
			props_lens += temp_psy->num_properties;
			
			ztemt_ext_power_psy.power_psy[i] = temp_psy;
			pr_info("find ext psy: %s\n",temp_psy->name);
		}else
		    break;
	} 

	ztemt_ext_power_psy.ext_psy_num = i;

    //copy main power supply properties
	memcpy(ztemt_ext_power_psy.power_props + props_lens, 
		   batt_psy->properties,
		   batt_psy->num_properties * enum_len);
	props_lens += batt_psy->num_properties;
	
	ztemt_ext_power_psy.props_num = props_lens;

	memcpy(&ztemt_ext_power_psy.main_power_psy, 
		   batt_psy,
		   sizeof(struct power_supply));

    //change to ztemt power supply
    batt_psy->properties = ztemt_ext_power_psy.power_props;
    batt_psy->num_properties = ztemt_ext_power_psy.props_num;
	batt_psy->get_property = ztemt_batt_power_get_property;
	batt_psy->set_property = ztemt_batt_power_set_property;

	pr_info("ext psy num=%d, prop num=%d\n",ztemt_ext_power_psy.ext_psy_num,
		                                (int)ztemt_ext_power_psy.props_num);
    return 0;
}


