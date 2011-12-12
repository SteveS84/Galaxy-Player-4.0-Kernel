/*
 * MC-1N2 ASoC codec driver - private header
 *
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef MC1N2_PRIV_H
#define MC1N2_PRIV_H

#include "mcdriver.h"

/*
 * Virtual registers
 */
enum {
	MC1N2_RCV_VOL_L = 0,
	MC1N2_RCV_VOL_R,
	MC1N2_SPEAKER_VOL_L,
	MC1N2_SPEAKER_VOL_R,
	MC1N2_HEADPHONE_VOL_L,
	MC1N2_HEADPHONE_VOL_R,
	MC1N2_ADC_VOL_L,
	MC1N2_ADC_VOL_R,
	MC1N2_MIC_ADC_VOL,// yosef added for mic gain test
	MC1N2_N_REG,
	MC1N2_MIC1_GAIN, // yosef added for mic gain test
	MC1N2_MIC2_GAIN, // yosef added for mic gain test
	MC1N2_DAC_VOL_L,
	MC1N2_DAC_MASTER,
	MC1N2_DAC_DAT_VAL,
};

/*
 * Path settings
 */
enum {
	PLAYBACK_OFF = 0,
	RCV,
	SPK,
	HP,
	BT,
	DUAL,
	RING_SPK,
	RING_HP,
	RING_DUAL,
	EXTRA_DOCK_SPEAKER,
	TV_OUT
};

enum {
	MIC_MAIN = 0,
	MIC_SUB,
	MIC_BT
};

enum {
	FMR_OFF = 0,
	FMR_SPK,
	FMR_HP,
	FMR_SPK_MIX,
	FMR_HP_MIX,
	FMR_DUAL_MIX
};

enum {
	CMD_FMR_INPUT_DEACTIVE = 0,
	CMD_FMR_INPUT_ACTIVE,
	CMD_FMR_FLAG_CLEAR,
	CMD_FMR_END,
	CMD_CALL_FLAG_CLEAR,
	CMD_CALL_END,
	CMD_RECOGNITION_DEACTIVE,
	CMD_RECOGNITION_ACTIVE,
	CMD_VOIP_ON = 10,
	CMD_VOIP_OFF = 11,
	#ifdef CONFIG_TDMB	// VenturiGB_Usys_jypark 2011.08.08 - DMB [[
    CMD_DMB_ON=17,
    CMD_DMB_OFF =18,
	#endif				// VenturiGB_Usys_jypark 2011.08.08 - DMB ]]
	CMD_FAKE_PROXIMITY_SENSOR_ON = 81,
	CMD_FAKE_PROXIMITY_SENSOR_OFF = 82,
};

#define MC1N2_N_VOL_REG MC1N2_ADCL_MIC1_SW	 

#define MC1N2_DSOURCE_OFF		0
#define MC1N2_DSOURCE_ADC		1
#define MC1N2_DSOURCE_DIR0		2
#define MC1N2_DSOURCE_DIR1		3
#define MC1N2_DSOURCE_DIR2		4
#define MC1N2_DSOURCE_MIX		5

#define MC1N2_AE_PARAM_1		0
#define MC1N2_AE_PARAM_2		1
#define MC1N2_AE_PARAM_3		2
#define MC1N2_AE_PARAM_4		3
#define MC1N2_AE_PARAM_5		4

#define mc1n2_i2c_read_byte(c,r) i2c_smbus_read_byte_data((c), (r)<<1)

extern struct i2c_client *mc1n2_get_i2c_client(void);

#define DISABLE_ANALOG_VOLUME

/*
 * For debugging
 */
#ifdef CONFIG_SND_SOC_MC1N2_DEBUG

#define dbg_info(format, arg...) snd_printd(KERN_INFO format, ## arg)
#define TRACE_FUNC() snd_printd(KERN_INFO "<trace> %s()\n", __FUNCTION__)


#define _McDrv_Ctrl McDrv_Ctrl_dbg
extern SINT32 McDrv_Ctrl_dbg(UINT32 dCmd, void *pvPrm, UINT32 dPrm);

#else

#define dbg_info(format, arg...)
#define TRACE_FUNC()

#define _McDrv_Ctrl McDrv_Ctrl

#endif /* CONFIG_SND_SOC_MC1N2_DEBUG */

#define CODEC_DEBUG

#ifdef CODEC_DEBUG
#define SUBJECT "mc1n2"
#define CODECDBG(format, ...)\
	printk(KERN_INFO "[ "SUBJECT " (%s,%d) ] " format "\n", \
			__func__, __LINE__, ## __VA_ARGS__);
#else
#define CODECDBG(format, ...)
#endif

#endif /* MC1N2_PRIV_H */
