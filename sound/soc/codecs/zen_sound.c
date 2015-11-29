/*
 * Author : Haikal Izzuddin
 *
 * Version 1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*
 * Changelog
 *
 * Version 1 (07/10/2015)
 *	- Initial version for Xiaomi Mi4i
 *
 */

#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <linux/mfd/wcd9xxx/wcd9330_registers.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include "wcd9320.h"
#include "wcd9xxx-resmgr.h"
#include "wcd9xxx-common.h"

#include "zen_sound.h"


/* Varibles */

// Pointer to codec structure
static struct snd_soc_codec *codec;

// Internal zen sound variables
static int zen_sound;	//master switch
static int debug;	//debug

static int headphone_volume_l;
static int headphone_volume_r;
static int speaker_volume_l;
static int speaker_volume_r;
// static int mic_level;

/* Zen Sound Hook Function */

void zen_sound_hook_tomtom_codec_probe(struct snd_soc_codec *codec_pointer) {
	//Store a copy
	codec = codec_pointer;

	// Print debug info
	printk("Zen Sound : Codec pointer received");
}

unsigned int zen_sound_hook_tomtom_write(unsigned int reg, unsigned int value) {
	// if zen sound is switched off, hook should not have any effect
	if(!zen_sound)
		return value;

	switch(reg){
		case TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL: // headphone L
		{
			value = headphone_volume_l;
			break;
		}

		case TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL: // headphone R
		{
			value = headphone_volume_r;
			break;
		}
		
		case TOMTOM_A_CDC_RX3_VOL_CTL_B2_CTL: // speaker L
		{
			value = speaker_volume_l;
			break;
		}

		case TOMTOM_A_CDC_RX7_VOL_CTL_B2_CTL: // speaker R
		{
			value = speaker_volume_r;
			break;
		}

//		case TOMTOM_A_CDC_RX2_VOL_CTL_GAIN: // mice level general
//		{
//			value = mic_level;
//			break;
//		}

		default:
			break;
	}

	return value;
}

/* Internal helper functions */

static void reset_zen_sound(void) {
	// set all config settings to defaults
	headphone_volume_l = HEADPHONE_DEFAULT;
	headphone_volume_r =HEADPHONE_DEFAULT;
	speaker_volume_l = SPEAKER_DEFAULT;
	speaker_volume_r = SPEAKER_DEFAULT;
//	mic_level = MICLEVEL_DEFAULT;

	if(debug)
		printk("Zen Sound: Reset done\n");
}

static void reset_audio_hub(void) {
	// reset all audio hub registers
	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL, HEADPHONE_DEFAULT + HEADPHONE_REG_OFFSET);
	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL, HEADPHONE_DEFAULT + HEADPHONE_REG_OFFSET);

	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX3_VOL_CTL_B2_CTL, SPEAKER_DEFAULT + SPEAKER_REG_OFFSET);
	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX7_VOL_CTL_B2_CTL, SPEAKER_DEFAULT + SPEAKER_REG_OFFSET);
//	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX7_VOL_CTL_B2_CTL, MICLEVEL_DEFAULT + MICLEVEL_REG_OFFSET);

	if(debug)
		printk("Zen Sound: wcd9330 audio hub reset done\n");
}

/* sysfs interface functions */

// Zen Sound master switch

static ssize_t zen_sound_show(struct device *dev, struct device_attribute *attr, char *buf) {
	//print current value
	return sprintf(buf, "Zen Sound status: %d\n", zen_sound);
}

static ssize_t zen_sound_store(struct device *dev, struct device_attribute *attr,
									const char *buf, size_t count) {
	unsigned int ret = -EINVAL;
	int val;

	// read values from input buffer
	ret = sscanf(buf, "%d", &val);

	if(ret != 1)
		return -EINVAL;

	// store if valid data
	if(((val == 0 ) || (val == 1))) {
		// set new status
		zen_sound = val;
		
		// reinitialize settings
		reset_zen_sound();
		reset_audio_hub();

		// print debug
		if(debug)
			printk("Zen Soun: status %d\n", zen_sound);
	}
	return count;
}

// Headphone volume

static ssize_t headphone_volume_show(struct device *dev, struct device_attribute *attr, char *buf){
	//print current value
	return sprintf(buf, "Headphone volume:\n LEft: %d\nRight: %d\n", headphone_volume_l, headphone_volume_r);
}

static ssize_t headphone_volume_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	unsigned int ret = -EINVAL;
	int val_l;
	int val_r;

	if(!zen_sound)
		return count;
	
	ret = sscanf(buf, "%d %d", &val_l, &val_r);

	if(ret !=2)
		return -EINVAL;

	if(val_l > HEADPHONE_MAX)
		val_l = HEADPHONE_MAX;

	if (val_l < HEADPHONE_MIN)
		val_l = HEADPHONE_MIN;

	if(val_r > HEADPHONE_MAX)
		val_r = HEADPHONE_MAX;

	if (val_r < HEADPHONE_MIN)
		val_r = HEADPHONE_MIN;

	headphone_volume_l = val_l;
	headphone_volume_r = val_r;

	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL, headphone_volume_l + HEADPHONE_REG_OFFSET);
	tomtom_write_no_hook(codec, TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL, headphone_volume_r + HEADPHONE_REG_OFFSET);

	if(debug)
		printk("Zen Sound: headphone volume L=%d R=%d\n", headphone_volume_l, headphone_volume_r);

	return count;
}

/* Speaker volume and Mic Volume haven't been implemented yet */
/* Remember to add this soon */

// Debug Status

static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "Debug status: %d\n", debug);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	unsigned int ret = -EINVAL;
	unsigned int val;
	
	ret = sscanf(buf, "%d",&val);

	if(ret != 1)
		return -EINVAL;

	if((val == 0) || (val == 1))
		debug = val;

	return count;
}

// Register dump

static ssize_t register_dump_show(struct device *dev, struct device_attribute *attr, char *buf) {
	sprintf(buf, "Register dump\n");
	sprintf(buf+strlen(buf), "==========\n");

	sprintf(buf+strlen(buf), "** Output values bank 1\n");
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX1_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX1_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX2_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX2_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX3_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX3_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX4_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX4_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX5_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX5_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX6_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX6_VOL_CTL_B1_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX7_VOL_CTL_B1_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX7_VOL_CTL_B1_CTL));

	sprintf(buf+strlen(buf), "** Output values bank 2\n");
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX1_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX2_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX3_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX3_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX4_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX4_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX5_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX5_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX6_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX6_VOL_CTL_B2_CTL));
	sprintf(buf+strlen(buf), "TOMTOM_A_CDC_RX7_VOL_CTL_B2_CTL : %d\n", tomtom_read(codec, TOMTOM_A_CDC_RX7_VOL_CTL_B2_CTL));

	return strlen(buf);
}

// Version info

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%s\n", ZEN_SOUND_VERSION);
}

// Initialize Sound Sysfs Folder

//define objects
	static DEVICE_ATTR(zen_sound, S_IRUGO | S_IWUGO, zen_sound_show, zen_sound_store);
	static DEVICE_ATTR(headphone_volume, S_IRUGO | S_IWUGO, headphone_volume_show, headphone_volume_store);
//	static DEVICE_ATTR(speaker_volume, S_IRUGO | S_IWUGO, speaker_volume_show, speaker_volume_store);
//	static DEVICE_ATTR(mic_level_general, S_IRUGO | S_IWUGO, mic_level_show, mic_level_store);	
	static DEVICE_ATTR(debug, S_IRUGO | S_IWUGO, debug_show, debug_store);	
	static DEVICE_ATTR(register_dump, S_IRUGO | S_IWUGO, register_dump_show, NULL);
	static DEVICE_ATTR(version, S_IRUGO | S_IWUGO, version_show, NULL);

// define attributes
static struct attribute *zen_sound_attributes[] = {
	&dev_attr_zen_sound.attr,
	&dev_attr_headphone_volume.attr,
	&dev_attr_debug.attr,
	&dev_attr_register_dump.attr,
	&dev_attr_version.attr,
	NULL
};

// define attribute group
static struct attribute_group zen_sound_control_group = {
	.attrs = zen_sound_attributes,
};

// define control device
static struct miscdevice zen_sound_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zen_sound",
};

/* Driver Init & Exit*/

static int zen_sound_init(void) {
	misc_register(&zen_sound_control_device);
	if(sysfs_create_group(&zen_sound_control_device.this_device->kobj, &zen_sound_control_group) < 0) {
		printk("Zen Sound: failed to create sysfs object.\n");
		return 0;
	}

	zen_sound = ZEN_SOUND_DEFAULT;
	debug = DEBUG_DEFAULT;

	reset_zen_sound();
	printk("Zen Sound: engine version %s started\n", ZEN_SOUND_VERSION);

	return 0;
}

static void zen_sound_exit(void) {
	sysfs_remove_group(&zen_sound_control_device.this_device->kobj, &zen_sound_control_group);
	printk("Zen Sound: engine stopped\n");
}

/* Define driver entry points */
module_init(zen_sound_init);
module_exit(zen_sound_exit);
