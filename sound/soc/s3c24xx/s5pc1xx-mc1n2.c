/*
 * s5pc1xx_mc1n2.c
 *
 * Copyright (C) 2010, Samsung Elect. Ltd. - 
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/regs-clock.h>
#include <plat/regs-iis.h>
#include "../codecs/mc1n2/mc1n2.h"
#include "s3c-dma.h"
#include "s5pc1xx-i2s.h"
#include "s3c-i2s-v2.h"

#include <linux/io.h>

#define I2S_NUM 0
#define SRC_CLK 66738000 	 

/* #define CONFIG_SND_DEBUG */
#ifdef CONFIG_SND_DEBUG
#define debug_msg(x...) printk(x)
#else
#define debug_msg(x...)
#endif

/*  BLC(bits-per-channel) --> BFS(bit clock shud be >= FS*(Bit-per-channel)*2)  */
/*  BFS --> RFS(must be a multiple of BFS)                                  */
/*  RFS & SRC_CLK --> Prescalar Value(SRC_CLK / RFS_VAL / fs - 1)           */
int smdkc110_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int bfs, rfs, ret;
	u32 ap_codec_clk;

	//struct clk    *clk_out, *clk_epll;
	//int psr;

	debug_msg("%s\n", __func__);

	/* Choose BFS and RFS values combination that is supported by
	 * both the WM8994 codec as well as the S5P AP
	 *
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	/* Can take any RFS value for AP */
			bfs = 16;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S16_LE:
	/* Can take any RFS value for AP */
			bfs = 32;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
			bfs = 48;
			rfs = 512;
			break;
	/* Impossible, as the AP doesn't support 64fs or more BFS */
	case SNDRV_PCM_FORMAT_S32_LE:
	default:
			return -EINVAL;
	}


	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	
	if (ret < 0) {
		printk(KERN_ERR "smdkc110_wm8994_hw_params :\
				 Codec DAI configuration error!\n");
		return ret;
	}

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
        
	if (ret < 0) {
		printk(KERN_ERR
			"smdkc110_wm8994_hw_params :\
				AP DAI configuration error!\n");
		return ret;
	}

	/* Select the AP Sysclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CDCLKSRC_EXT,
					params_rate(params), SND_SOC_CLOCK_IN);

	if (ret < 0) {
		printk(KERN_ERR
			"smdkc110_wm8994_hw_params :\
			AP sys clock INT setting error!\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_I2SEXT,
					params_rate(params), SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR
			"smdkc110_wm8994_hw_params :\
			AP sys clock I2SEXT setting error!\n");
		return ret;
	}

	ret  = snd_soc_dai_set_clkdiv(codec_dai, MC1N2_BCLK_MULT, MC1N2_LRCK_X32);
	if (ret < 0)
	{
		printk("smdkc110_wm8994_hw_params : MC1N2_BCLK_MULT setting error!\n");
		return ret;
	}

	switch (params_rate(params)) {

	case 8000:
		ap_codec_clk = 4096000;
		break;
	case 11025:
		ap_codec_clk = 2822400;
		break;
	case 12000:
		ap_codec_clk = 6144000;
		break;
	case 16000:
		ap_codec_clk = 4096000;
		break;
	case 22050:
		ap_codec_clk = 6144000;
		break;
	case 24000:
		ap_codec_clk = 6144000;
		break;
	case 32000:
		ap_codec_clk = 8192000;
		break;
	case 44100:
		ap_codec_clk = 11289600;
		break;
	case 48000:
		ap_codec_clk = 12288000;
		break;
	default:
		ap_codec_clk = 11289600;
		break;
	}

	return 0;

}

/* machine stream operations */
static struct snd_soc_ops smdkc110_ops = {
	.hw_params = smdkc110_hw_params,
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link smdkc1xx_dai = {
	.name = "mc1n2",
	.stream_name = "mc1n2 HiFi Playback",
	.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
	.codec_dai = &mc1n2_dai[0],
	.ops = &smdkc110_ops,
};

static struct snd_soc_card smdkc100 = {
	.name = "smdkc110",
	.platform = &s3c_dma_wrapper,
	.dai_link = &smdkc1xx_dai,
	.num_links = 1,
};

static struct mc1n2_setup smdkc110_mc1n2_setup = {
	 {  /* MCDRV_INIT_INFO */
	   MCDRV_CKSEL_CMOS, /* bCkSel */
	   28,                /* bDivR0 20MHz*/ //28 for 24, 19 for 20
	   86,               /* bDivF0 20MHz*/ //86 for 24, 70 for 20
	   28,                /* bDivR1*/
	   86,                /* bDivF1*/
	   0,                /* bRange0*/
	   0,                /* bRange1*/
	   0,                /* bBypass*/
	   MCDRV_DAHIZ_LOW,  /* bDioSdo0Hiz */
	   MCDRV_DAHIZ_LOW,  /* bDioSdo1Hiz */
	   MCDRV_DAHIZ_LOW,  /* bDioSdo2Hiz */
	   MCDRV_DAHIZ_HIZ,  /* bDioClk0Hiz */
	   MCDRV_DAHIZ_HIZ,  /* bDioClk1Hiz */
	   MCDRV_DAHIZ_HIZ,  /* bDioClk2Hiz */
	   MCDRV_PCMHIZ_HIZ, /* bPcmHiz */
	   MCDRV_LINE_STEREO,/* bLineIn1Dif */
	   0,                /* bLineIn2Dif */
	   MCDRV_LINE_STEREO,/* bLineOut1Dif */
	   MCDRV_LINE_STEREO,/* bLineOUt2Dif */
	   MCDRV_SPMN_ON,    /* bSpmn */
	   MCDRV_MIC_DIF,    /* bMic1Sng */
	   MCDRV_MIC_DIF,    /* bMic2Sng */
	   MCDRV_MIC_DIF,    /* bMic3Sng */
	   MCDRV_POWMODE_NORMAL, /* bPowerMode */
	   MCDRV_SPHIZ_PULLDOWN, /* bSpHiz */
	   MCDRV_LDO_ON,     /* bLdo */
	   MCDRV_PAD_GPIO,   /* bPad0Func */
	   MCDRV_PAD_GPIO,   /* bPad1Func */
	   MCDRV_PAD_GPIO,   /* bPad2Func */
//	   MCDRV_OUTLEV_4,   /* bAvddLev */
	   MCDRV_OUTLEV_7,   /* bAvddLev */   //yosef added for mic bias test 2.53v-> 2.75v
	   0,                /* bVrefLev */
	   MCDRV_DCLGAIN_12, /* bDclGain */
	   MCDRV_DCLLIMIT_0, /* bDclLimit */
	   0,			/* set Hi-power mode 0: HP mode 1: normal */
	   0,                /* bReserved1 */
	   0,                /* bReserved2 */
	   0,                /* bReserved3 */
	   0,                /* bReserved4 */
	   0,                /* bReserved5 */
	   {                 /* sWaitTime */
	     130000,         /* dAdHpf */
	     25000,          /* dMic1Cin */
	     25000,          /* dMic2Cin */
	     25000,          /* dMic3Cin */
	     25000,          /* dLine1Cin */
	     25000,          /* dLine2Cin */
	     5000,           /* dVrefRdy1 */
	     15000,          /* dVrefRdy2 */
	     9000,           /* dHpRdy */
	     13000,          /* dSpRdy */
	     0,              /* dPdm */
	     1000,           /* dAnaRdyInterval */
	     1000,           /* dSvolInterval */
	     1000,           /* dAnaRdyTimeOut */
	     1000            /* dSvolTimeOut */
	   }
	 }, /* MCDRV_INIT_INFO end */
	 {  /* pcm_extend */
	   0, 0, 0
	 }, /* pcm_extend end */
	 {  /* pcm_hiz_redge */
	   MCDRV_PCMHIZTIM_FALLING, MCDRV_PCMHIZTIM_FALLING, MCDRV_PCMHIZTIM_FALLING
	 }, /* pcm_hiz_redge end */
	 {  /* pcm_hperiod */
	   1, 1, 1
	 }, /* pcm_hperiod end */
	 {  /* slot */
	   {{0,1},{0,1}},
	   {{0,1},{0,1}},
	   {{0,1},{0,1}}
	 }  /* slot end */
};

/* audio subsystem */
static struct snd_soc_device smdkc1xx_snd_devdata = {
	.card = &smdkc100,
	.codec_dev = &soc_codec_dev_mc1n2,
	.codec_data = &smdkc110_mc1n2_setup,
};

static struct platform_device *smdkc1xx_snd_device; 
static int __init smdkc110_audio_init(void)
{
	int ret;

	debug_msg("%s\n", __func__);

	smdkc1xx_snd_device = platform_device_alloc("soc-audio", 0);
	if (!smdkc1xx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdkc1xx_snd_device, &smdkc1xx_snd_devdata);
	smdkc1xx_snd_devdata.dev = &smdkc1xx_snd_device->dev;
	ret = platform_device_add(smdkc1xx_snd_device);

	if (ret)
		platform_device_put(smdkc1xx_snd_device);

	return ret;
}

static void __exit smdkc110_audio_exit(void)
{
	debug_msg("%s\n", __func__);

	platform_device_unregister(smdkc1xx_snd_device);
}

module_init(smdkc110_audio_init);
module_exit(smdkc110_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC SMDKC110 WM8994");
MODULE_LICENSE("GPL");
