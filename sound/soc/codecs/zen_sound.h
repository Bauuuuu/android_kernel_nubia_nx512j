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
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <linux/mfd/wcd9xxx/wcd9330_registers.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include "wcd9320.h"
#include "wcd9xxx-resmgr.h"
#include "wcd9xxx-common.h"

/* Function Declarations */

// wcd9320 functions
extern unsigned int tomtom_read(struct snd_soc_codec *codec, unsigned int reg);
extern int tomtom_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value);
extern int tomtom_write_no_hook(struct snd_soc_codec *codec, unsigned int reg, unsigned int value);

// Zen function
void zen_sound_hook_tomtom_codec_probe(struct snd_soc_codec *codec_pointer);
unsigned int zen_sound_hook_tomtom_write(unsigned int reg, unsigned int value);

/* Definitions */

// General
#define ZEN_SOUND_DEFAULT	0
#define ZEN_SOUND_VERSION	"1.0"
#define DEBUG_DEFAULT	0

// Headphone levels
#define HEADPHONE_DEFAULT	0
#define HEADPHONE_REG_OFFSET	0
#define HEADPHONE_MIN	-30
#define HEADPHONE_MAX	30

// Speaker levels
#define SPEAKER_DEFAULT	0
#define SPEAKER_REG_OFFSET	-4
#define SPEAKER_MIN	-30
#define SPEAKER_MAX	30

// Mic Control
#define MICLEVEL_DEFAULT	0
#define MICLEVEL_REG_OFFSET	0
#define MICLEVEL_MIN	-30
#define MICLEVEL_MAX	30

