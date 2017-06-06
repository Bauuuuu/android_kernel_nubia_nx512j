/******************************************************************************

  Copyright (C), 2001-2014, ZTEMT Co., Ltd.

 ******************************************************************************
  File Name     : ztemt_hw_version.c
  Version       : Initial Draft
  Author        : LiuYongfeng
  Created       : 2014/6/5
  Last Modified :
  Description   : hardware version
  Function List :
  History       :
  1.Date        : 2014/6/5
    Author      : LiuYongfeng
    Modification: Created file
  2.Date        : 2015/6/29
    Author      : AHF
    Modification: Add hw logical here base on bootloader
******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>

#include "ztemt_hw_version.h"

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

//#define HW_DEBUG

#ifdef HW_DEBUG
#define hw_debug(fmt, args...) printk(KERN_INFO "[hw]"fmt,##args)
#else
#define hw_debug(fmt, args...) do {} while(0)
#endif

#define HW_VER_MAX_LEN 60

static char board_info_cmdline[HW_VER_MAX_LEN] = {0};


//all board_info are here
static board_orig_value_st g_board_info;

#if defined (CONFIG_ZTEMT_HW_VERSION_NX511J)
#define HW_PROJ     "NX511J"
#elif defined (CONFIG_ZTEMT_HW_VERSION_NX519J)
#define HW_PROJ     "NX519J"
#elif defined (CONFIG_ZTEMT_HW_VERSION_NX511J_V3)
//Be carefull!! NX511J not _V3
#define HW_PROJ     "NX511J"
#else
#define HW_PROJ      "NUBIA"
#endif

#if defined (CONFIG_ZTEMT_HW_VERSION_NX511J)
static gpio_map_2_st g_hw_ver_map[] = {
{ZTE_GPIO_FLOAT, ZTE_GPIO_FLOAT, HW_PROJ"MB_A"},
{ZTE_GPIO_FLOAT, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_B"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_C"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, HW_PROJ"MB_D"},
};
#elif defined (CONFIG_ZTEMT_HW_VERSION_NX519J)
static gpio_map_2_st g_hw_ver_map[] = {
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, HW_PROJ"MB_A"},
{ZTE_GPIO_FLOAT, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_B"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_C"},
{ZTE_GPIO_FLOAT, ZTE_GPIO_FLOAT, HW_PROJ"MB_D"},
};
#elif defined (CONFIG_ZTEMT_HW_VERSION_NX511J_V3)
static gpio_map_2_st g_hw_ver_map[] = {
{ZTE_GPIO_FLOAT, ZTE_GPIO_FLOAT, HW_PROJ"MB_A"},
{ZTE_GPIO_FLOAT, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_B"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, HW_PROJ"MB_C"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, HW_PROJ"MB_D"},
};
#else
static gpio_map_2_st g_hw_ver_map[] = {
{ZTE_GPIO_UNKNOWN, ZTE_GPIO_UNKNOWN, HW_PROJ"ERR"},
};
#endif

#if defined (CONFIG_ZTEMT_HW_VERSION_NX511J) || defined (CONFIG_ZTEMT_HW_VERSION_NX519J) || defined (CONFIG_ZTEMT_HW_VERSION_NX511J_V3)
static gpio_map_3_st g_hw_wwan_map[] = {
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, ZTE_GPIO_FLOAT, "CM"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, ZTE_GPIO_PULL_DOWN, "CT"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, "COMMON"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, "CU"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_FLOAT, ZTE_GPIO_PULL_UP, "MARKETING"},
{ZTE_GPIO_PULL_DOWN, ZTE_GPIO_PULL_UP, ZTE_GPIO_FLOAT, "CU_CM"},
};
#else
static gpio_map_3_st g_hw_wwan_map[] = {
{ZTE_GPIO_UNKNOWN, ZTE_GPIO_UNKNOWN, ZTE_GPIO_UNKNOWN, "ERR"},
};
#endif

//NX511J donot need this base on the old baseline.
#if defined (CONFIG_ZTEMT_HW_VERSION_NX519J) || defined (CONFIG_ZTEMT_HW_VERSION_NX511J) || defined (CONFIG_ZTEMT_HW_VERSION_NX511J_V3)
static adc_map_st g_adc_device_map[] = {
{1600, 1800, "ADC-ty1"},
//{1400, 1600, "ADC-ty2"},
//{1200, 1400, "ADC-ty3"},
//{1000, 1200, "ADC-ty4"},
{800, 1000, "ADC-ty5"},
//{600, 800, "ADC-ty6"},
//{400, 600, "ADC-ty7"},
//{200, 400, "ADC-ty8"},
//{0, 200, "ADC-ty9"},
};
#else
static adc_map_st g_adc_device_map[] = {
{0, 0, "err"},
};
#endif

static char badval_str[] = "BADVAL"; 
static char unknow_str[] = "UNKNOW";



static int __init ztemt_hw_board_setup(char *str)
{

       if(str == NULL) return -1;
       strncpy(board_info_cmdline, str, HW_VER_MAX_LEN);
       board_info_cmdline[HW_VER_MAX_LEN-1] = 0;
       pr_info("[hw] cl may be cut:(%s)\n", board_info_cmdline);
	return 0;
}
__setup("BOARD_INFO=", ztemt_hw_board_setup);

static char *ztemt_get_hw_expose( void )
{    
    if( g_board_info.inited == 0 ) return badval_str;
    if( g_board_info.hw_index == HW_UN ) return unknow_str;
    return g_hw_ver_map[ g_board_info.hw_index].expose_str;
}

static char *ztemt_get_wwan_expose( void )
{
    if( g_board_info.inited == 0) return badval_str;
    if( g_board_info.wwan_index == HW_UN ) return unknow_str;
    return g_hw_wwan_map[ g_board_info.wwan_index].expose_str;
}

#if 0
static char *ztemt_get_adc_expose( void )
{
    if( g_board_info.inited == 0 ) return badval_str;
    if( g_board_info.adc_index == HW_UN ) return unknow_str;
    return g_adc_device_map[ g_board_info.adc_index].expose_str;
}
#endif

// 0x 00xx xxxx   -- wrong input
// 0x F0xx xxxx   -- no value
// 0x 0Fxx xxxx   -- correct
static char *ztemt_get_gpio_value(char *str, unsigned int *val, int num)
{
    unsigned tmp_val;
    int tmp_num;
    char *p;
    char chr;

    tmp_val = 0;//wrong input
    if(( str == NULL) || (val == NULL)) { *val = tmp_val; return NULL;};
    if( *str != ':' ){ *val = tmp_val; return str;}
    if(num <= 0  || num > 12) { *val = tmp_val; return str;}

    tmp_val = 0xF0000000;//no value
    tmp_num = 0;
    p = str + 1;

    while( *p != 0 )
    {
        chr = *p;
        if( (chr == '0') || (chr == '1') || (chr == '2') )
        {
            tmp_val <<= 2;
            tmp_val += chr - '0';
            tmp_val = ( tmp_val & 0x00FFFFFF ) | (0x0F000000);
         }else if (chr == ',')
         {
            tmp_val = ( tmp_val & 0x00FFFFFF ) | (0xF0000000);
            ++tmp_num;
         }else
         {
            break;
         }
         p++;
    };

    if( ( chr == ':' )  && ( tmp_num == (num - 1) ) )
    {
        *val = tmp_val;
        return p;
    }

    *val = 0;
    return str;
}


/* return: value = -1 formate err, all data not inited
                value = 0  adc is emty or with 0 value
                value = int  data ok
*/
static char *ztemt_get_adc_value(char *str, int *val)
{
    int tmp_val;
    char chr;
    char *p;
     if((str == NULL) || (val == NULL)) { *val = -1; return NULL;};
     if( *str != ':' ){ *val = -1; return str;}
     tmp_val = 0;
     p = str + 1;
     /* longest length is 4, no need to use while. */
     while( *p != 0 )
     {
        chr = *p;
        if( (chr <='9') && (chr >= '0'))
        {
            if( tmp_val == -1 )
                tmp_val = 0;
            chr = chr - '0';
            tmp_val = tmp_val * 10;
            tmp_val += chr;
        }else
        {
            break;
        }
        p++;
     }

     if( chr==':' )
     {
        *val = tmp_val;
        return p;
     }
     //foramt error.
     *val = -1;
     return str;
}

static void ztemt_hw_board_put(unsigned int gpio2, unsigned int gpio3, int adc)
{
    g_board_info.gpio_hw_b = (int)(gpio2 & 0x03);
    gpio2 >>= 2;
    g_board_info.gpio_hw_a = (int)(gpio2 & 0x03);

    g_board_info.gpio_wwan_c = gpio3 & 0x03;
    gpio3 >>= 2;
    g_board_info.gpio_wwan_b = gpio3 & 0x03;
    gpio3 >>= 2;
    g_board_info.gpio_wwan_a = gpio3 & 0x03;
    g_board_info.adc_mv = adc;
}

static void ztemt_hw_board_fullfill( void )
{
    int i;
    for(i=0; i<sizeof(g_hw_ver_map)/sizeof(g_hw_ver_map[0]); i++)
    {
        if( (g_hw_ver_map[i].gpio1 == g_board_info.gpio_hw_a) && (g_hw_ver_map[i].gpio2 == g_board_info.gpio_hw_b) )
            break;
    }
    g_board_info.hw_index = (i == sizeof(g_hw_ver_map)/sizeof(g_hw_ver_map[0]) )?(HW_UN):i;

    for(i=0; i<sizeof(g_hw_wwan_map)/sizeof(g_hw_wwan_map[0]); i++)
    {
        if( (g_hw_wwan_map[i].gpio1 == g_board_info.gpio_wwan_a) && (g_hw_wwan_map[i].gpio2 == g_board_info.gpio_wwan_b) && \
            (g_hw_wwan_map[i].gpio3 == g_board_info.gpio_wwan_c) )
            break;
    }
    g_board_info.wwan_index =  (i == sizeof(g_hw_wwan_map)/sizeof(g_hw_wwan_map[0]) )?(HW_UN):i;

    for(i=0; i<sizeof(g_adc_device_map)/sizeof(g_adc_device_map[0]); i++)
    {
            if( (g_board_info.adc_mv > g_adc_device_map[i].low_mv) && (g_board_info.adc_mv <= g_adc_device_map[i].high_mv) )
                break;
    }
     g_board_info.adc_index =  (i == sizeof(g_adc_device_map)/sizeof(g_adc_device_map[0]) )?(HW_UN):i;

     g_board_info.inited = 1;
    return;
}


static ssize_t ztemt_hw_version_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s hw_ver = %s\n",__func__,ztemt_get_hw_expose());
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_get_hw_expose());
}

static ssize_t ztemt_hw_operators_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s operator = %s\n",__func__,ztemt_get_wwan_expose());
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_get_wwan_expose());
}

#if 0
static ssize_t ztemt_hw_adc_type_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s adc_type = %s\n",__func__,ztemt_get_adc_expose());
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_get_adc_expose());
}
#endif

static struct kobj_attribute attrs[] = {
	__ATTR(version, 0664, ztemt_hw_version_show, NULL),
	__ATTR(operators, 0664, ztemt_hw_operators_show, NULL),
#if 0
	__ATTR(adcty, 0664, ztemt_hw_adc_type_show, NULL),
#endif
};

static void ztemt_board_init( void )
{
     char *buf = NULL;
     unsigned int gpio2, gpio3, adc;
     int val_pass = 0;

    do{
        if( board_info_cmdline[0] != ':' )
            break;
        buf = board_info_cmdline;
        buf = ztemt_get_gpio_value(buf, &gpio2, 2);
        if( gpio2 == 0 )  break;
        buf = ztemt_get_gpio_value(buf, &gpio3, 3);
        if( gpio3 == 0 )  break;

        buf = ztemt_get_adc_value(buf, &adc);
        if(adc == -1) break;

        val_pass = 1;        
    }while(0);

    /* adc can be empty, while gpio cannot; modify it in next version. */
    if( val_pass == 1 && !(gpio2 & 0xF0000000) && !(gpio3 & 0xF0000000))
    {
        ztemt_hw_board_put(gpio2, gpio3, adc);
        ztemt_hw_board_fullfill();
     }
    else
     {
        g_board_info.inited = 0;
     }

      hw_debug("hw:%s .\n", ztemt_get_hw_expose());
      hw_debug("wwan:%s .\n", ztemt_get_wwan_expose());
      hw_debug("adc:%s .\n", ztemt_get_adc_expose());

      return;
}


struct kobject *hw_version__kobj;

static int ztemt_hw_version_init(void)
{
	int retval;
	int attr_count = 0;

	hw_version__kobj = kobject_create_and_add("ztemt_hw_version", NULL);
	if (!hw_version__kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		
		retval = sysfs_create_file(hw_version__kobj, &attrs[attr_count].attr);
		if (retval < 0) {
			pr_err("%s: Failed to create sysfs attributes\n", __func__);
			goto err_sys;
		}
	}

       ztemt_board_init();
	return retval;
	
err_sys:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(hw_version__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(hw_version__kobj);
	
	pr_err("[hw]%s init ERR.\n",__func__);

	return retval;
}

static void __exit ztemt_hw_version_exit(void)
{
	int attr_count = 0;
	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(hw_version__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(hw_version__kobj);
}


module_init(ztemt_hw_version_init);
module_exit(ztemt_hw_version_exit);


MODULE_DESCRIPTION("QPNP ZTEMT HARDWARE VERSION");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:Board Hardware version");
