/*
 * MC-1N2 ASoC codec driver
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

#ifndef MC1N2_H
#define MC1N2_H

#include "mcdriver.h"

/*
 * dai: set_sysclk
 */
/* clk_id */
#define MC1N2_CLKI              0

/* default freq for MC1N2_CLKI */
#define MC1N2_DEFAULT_CLKI      19200000

/*
 * dai: set_clkdiv
 */
/* div_id */
#define MC1N2_CKSEL             0
#define MC1N2_DIVR0             1
#define MC1N2_DIVF0             2
#define MC1N2_DIVR1             3
#define MC1N2_DIVF1             4
#define MC1N2_BCLK_MULT         5

/* div for MC1N2_BCLK_MULT */
#define MC1N2_LRCK_X8           0
#define MC1N2_LRCK_X16          1
#define MC1N2_LRCK_X24          2
#define MC1N2_LRCK_X32          3
#define MC1N2_LRCK_X48          4
#define MC1N2_LRCK_X64          5
#define MC1N2_LRCK_X128         6
#define MC1N2_LRCK_X256         7
#define MC1N2_LRCK_X512         8

/*
 * hwdep: ioctl
 */
#define MC1N2_MAGIC             'N'
#define MC1N2_IOCTL_NR_GET      1
#define MC1N2_IOCTL_NR_SET      2
#define MC1N2_IOCTL_NR_BOTH     3
#define MC1N2_IOCTL_NR_NOTIFY   4

#define MC1N2_IOCTL_GET_CTRL \
	_IOR(MC1N2_MAGIC, MC1N2_IOCTL_NR_GET, struct mc1n2_ctrl_args)
#define MC1N2_IOCTL_SET_CTRL \
	_IOW(MC1N2_MAGIC, MC1N2_IOCTL_NR_SET, struct mc1n2_ctrl_args)

#define MC1N2_IOCTL_READ_REG \
	_IOWR(MC1N2_MAGIC, MC1N2_IOCTL_NR_BOTH, struct mc1n2_ctrl_args)

#define MC1N2_IOCTL_NOTIFY \
	_IOW(MC1N2_MAGIC, MC1N2_IOCTL_NR_NOTIFY, struct mc1n2_ctrl_args)

struct mc1n2_ctrl_args {
	unsigned long dCmd;
	void *pvPrm;
	unsigned long dPrm;
};

/*
 * MC1N2_IOCTL_NOTIFY dCmd definitions
 */
#define MCDRV_NOTIFY_CALL_START		0x00000000
#define MCDRV_NOTIFY_CALL_STOP		0x00000001
#define MCDRV_NOTIFY_MEDIA_PLAY_START	0x00000002
#define MCDRV_NOTIFY_MEDIA_PLAY_STOP	0x00000003
#define MCDRV_NOTIFY_FM_PLAY_START	0x00000004
#define MCDRV_NOTIFY_FM_PLAY_STOP	0x00000005
#define MCDRV_NOTIFY_BT_SCO_ENABLE	0x00000006
#define MCDRV_NOTIFY_BT_SCO_DISABLE	0x00000007
#define MCDRV_NOTIFY_VOICE_REC_START	0x00000008
#define MCDRV_NOTIFY_VOICE_REC_STOP		0x00000009
#define MCDRV_NOTIFY_HDMI_START	0x0000000A
#define MCDRV_NOTIFY_HDMI_STOP	0x0000000B

/*
 * Setup parameters
 */
struct mc1n2_setup {
	MCDRV_INIT_INFO init;
	unsigned char pcm_extend[IOPORT_NUM];
	unsigned char pcm_hiz_redge[IOPORT_NUM];
	unsigned char pcm_hperiod[IOPORT_NUM];
	unsigned char slot[IOPORT_NUM][SNDRV_PCM_STREAM_LAST+1][DIO_CHANNELS];
};

/*
 * Exported symbols
 */
extern struct snd_soc_dai mc1n2_dai[];
extern struct snd_soc_codec_device soc_codec_dev_mc1n2;


#define CONFIG_FMRADIO_CODEC_GAIN

#ifdef CONFIG_FMRADIO_CODEC_GAIN
unsigned int McDrv_Ctrl_fm(unsigned int volume);
unsigned int McDrv_Ctrl_fm_mute(void);
unsigned int McDrv_Ctrl_fm_recovery(void);
unsigned char McDrv_Ctrl_get_fm_vol(void);
#endif

/* venturi add by park dong yun move static function to header for voip controll */
struct snd_soc_codec *mc1n2_get_codec_data(void);
int mc1n2_write_reg(struct snd_soc_codec *codec,unsigned int reg, unsigned int value);
unsigned int mc1n2_read_reg(struct snd_soc_codec *codec, unsigned int reg);
int mc1n2_set_path(struct snd_soc_codec *codec, MCDRV_PATH_INFO *info);
void 	McDrv_Ctrl_DNG_ctrl(int en , int path, int threshold);
void mc1n2_set_analog_volume_hp(int index);
void mc1n2_set_analog_volume_spk(int index);



/*
 * Driver private data structure
 */


#define MC1N2_N_PATH_CHANNELS 19

struct mc1n2_port_params {
	UINT8 rate;
	UINT8 bits[SNDRV_PCM_STREAM_LAST+1];
	UINT8 pcm_mono[SNDRV_PCM_STREAM_LAST+1];
	UINT8 pcm_order[SNDRV_PCM_STREAM_LAST+1];
	UINT8 pcm_law[SNDRV_PCM_STREAM_LAST+1];
	UINT8 master;
	UINT8 inv;
	UINT8 format;
	UINT8 bckfs;
	UINT8 pcm_clkdown;
	UINT8 channels;
	UINT8 stream;                     /* bit0: Playback, bit1: Capture */
	UINT8 dir[MC1N2_N_PATH_CHANNELS]; /* path settings for DIR */
	MCDRV_CHANNEL dit;                /* path settings for DIT */
};

struct mc1n2_data {
	struct mutex mutex;
	struct mc1n2_setup setup;
	struct mc1n2_port_params port[IOPORT_NUM];
	struct snd_hwdep *hwdep;
	int clk_update;
	MCDRV_PATH_INFO path_store;
	MCDRV_VOL_INFO vol_store;
	MCDRV_DIO_INFO dio_store;
	MCDRV_DAC_INFO dac_store;
	MCDRV_ADC_INFO adc_store;
	MCDRV_SP_INFO sp_store;
	MCDRV_DNG_INFO dng_store;
	MCDRV_SYSEQ_INFO syseq_store;
	MCDRV_AE_INFO ae_store;
	MCDRV_PDM_INFO pdm_store;
	int config_flag;
	int playback_path;
	int call_path;
	int mic_path;
	int fmr_path;
	int codec_tuning;
	int codec_status;
	u16 rcv_vol_l;
	u16 rcv_vol_r;
	u16 sp_vol_l;
	u16 sp_vol_r;
	u16 hp_vol_l;
	u16 hp_vol_r;
	u16 ad_vol_l;
	u16 ad_vol_r;
      u16 da_vol_l;
      u16 da_mas_vol;
      u16 da_dit_vol;
      u16 analog_vol;
};


#endif
