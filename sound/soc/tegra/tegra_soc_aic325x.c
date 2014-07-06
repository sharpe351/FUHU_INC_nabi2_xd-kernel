/*
 * tegra_aic325x.c - Tegra machine ASoC driver for boards using TI 3262 codec.
 *
 * Author: Vinod G. <vinodg@nvidia.com>
 * Copyright (C) 2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2010, 2011 Nvidia Graphics Pvt. Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include "../codecs/tlv320aic325x.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#else
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#endif


#define DRV_NAME "tegra-snd-aic325x"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
//&*&*&*CT1_130122: headset detection using gpio interrupt
#define GPIO_HP_DET     BIT(4)
//&*&*&*CT2_130122: headset detection using gpio interrupt

#define DAI_LINK_HIFI		0
#define DAI_LINK_SPDIF		1
#define DAI_LINK_BTSCO		2
#define DAI_LINK_VOICE_CALL	3
#define DAI_LINK_BT_VOICE_CALL	4
#define NUM_DAI_LINKS	5

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const char *tegra_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
	"tegra30-i2s.0",
	"tegra30-i2s.1",
	"tegra30-i2s.2",
	"tegra30-i2s.3",
	"tegra30-i2s.4",
};
#endif

//&*&*&*CT1_130129: add enable speaker gpio function
static int spk_enable = 0;
//&*&*&*CT2_130129: add enable speaker gpio function

struct tegra_aic325x {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	struct regulator *audio_reg;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct codec_config codec_info[NUM_I2S_DEVICES];
	struct snd_soc_card *pcard;
#endif
//&*&*&*CT1_130122: headset detection using gpio interrupt
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
//&*&*&*CT2_130122: headset detection using gpio interrupt
};

static int tegra_aic325x_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic325x_call_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic325x *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_aic325x_call_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic325x *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int codec_dap_id, codec_dap_sel, bb_dap_id, bb_dap_sel;
#else /*assumes tegra3*/
	int codec_index;
	unsigned int i;
#endif

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	bb_dap_id = TEGRA20_DAS_DAP_ID_3;
	bb_dap_sel = TEGRA20_DAS_DAP_SEL_DAP3;

	if (machine->is_device_bt) {
		codec_dap_id = TEGRA20_DAS_DAP_ID_4;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP4;
	} else {
		codec_dap_id = TEGRA20_DAS_DAP_ID_2;
		codec_dap_sel = TEGRA20_DAS_DAP_SEL_DAP2;
	}
#else /*assumes tegra3*/
	if (machine->is_device_bt)
		codec_index = BT_SCO;
	else
		codec_index = HIFI_CODEC;
#endif

	if (is_call_mode_new) {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			codec_dap_sel, 1, 0, 0);
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
#else /*assumes tegra3*/
		if (machine->codec_info[codec_index].rate == 0 ||
			machine->codec_info[codec_index].channels == 0)
				return -EINVAL;

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;

		tegra30_make_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);
#endif
	} else {
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		tegra20_das_set_tristate(codec_dap_id, 1);
		tegra20_das_set_tristate(bb_dap_id, 1);
		tegra20_das_connect_dap_to_dap(bb_dap_id,
			bb_dap_sel, 0, 0, 0);
		tegra20_das_connect_dap_to_dap(codec_dap_id,
			codec_dap_sel, 0, 0, 0);
		tegra20_das_set_tristate(codec_dap_id, 0);
		tegra20_das_set_tristate(bb_dap_id, 0);
#else /*assumes tegra3*/
		tegra30_break_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;
#endif
	}

	machine->is_call_mode = is_call_mode_new;
	g_is_call_mode = machine->is_call_mode;

	return 1;
}

struct snd_kcontrol_new tegra_aic325x_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_aic325x_call_mode_info,
	.get = tegra_aic325x_call_mode_get,
	.put = tegra_aic325x_call_mode_put
};

static int tegra_aic325x_get_mclk(int srate)
{
	int mclk = 0;
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = -EINVAL;
		break;
	}

	return mclk;
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_set_dam_cif(int dam_ifc, int srate,
			int channels, int bit_size, int src_on, int src_srate,
			int src_channels, int src_bit_size)
{
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT,
				srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1,
				srate);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				bit_size);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				bit_size);

	if (src_on) {
		tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 16);
	}

	return 0;
}
#endif

static int tegra_aic325x_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, sample_size, daifmt;
	int err, rate;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
#endif

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0) {
		dev_err(card->dev, "tegra_aic325x_get_mclk < 0\n");
		return mclk;
	}
	daifmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);
	err = snd_soc_dai_set_fmt(codec_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#if 0
	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}
#endif
	err = snd_soc_dai_set_pll(codec_dai, 0, AIC3256_CLK_REG_1,
					rate, params_rate(params));
	if (err < 0) {
		dev_err(card->dev, "Failed to set pll\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

#else /*assumes tegra3*/
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK  && i2s->is_dam_used)
		tegra_aic325x_set_dam_cif(i2s->dam_ifc, srate,
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif
	return 0;
}
//&*&*&*CT1_130304: Add BT-SOC function
//#if 0
static int tegra_aic325x_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}
//#endif
//#if 0
static int tegra_aic325x_bt_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	int err, srate, mclk, min_mclk;
	int sample_size;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(rtd->cpu_dai,
			SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(rtd->codec->card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#else
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && i2s->is_dam_used)
		tegra_aic325x_set_dam_cif(i2s->dam_ifc, params_rate(params),
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif

	return 0;
}
//#endif
//&*&*&*CT2_130304: Add BT-SOC function
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic325x_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);
	struct codec_config *codec_info;
	struct codec_config *bb_info;
	int codec_index;

	if (!i2s->is_dam_used)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*dam configuration*/
		if (!i2s->dam_ch_refcount)
			i2s->dam_ifc = tegra30_dam_allocate_controller();

		tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount++;
		tegra30_dam_enable_clock(i2s->dam_ifc);
		tegra30_dam_set_gain(i2s->dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);

		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
				(i2s->dam_ifc*2), i2s->txcif);

		/*
		*make the dam tx to i2s rx connection if this is the only client
		*using i2s for playback
		*/
		if (i2s->playback_ref_count == 1)
			tegra30_ahub_set_rx_cif_source(
				TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
				TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

		/* enable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
	} else {

		i2s->is_call_mode_rec = machine->is_call_mode;

		if (!i2s->is_call_mode_rec)
			return 0;

		if (machine->is_device_bt)
			codec_index = BT_SCO;
		else
			codec_index = HIFI_CODEC;

		codec_info = &machine->codec_info[codec_index];
		bb_info = &machine->codec_info[BASEBAND];

		/* allocate a dam for voice call recording */

		i2s->call_record_dam_ifc = tegra30_dam_allocate_controller();
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_enable_clock(i2s->call_record_dam_ifc);

		/* configure the dam */
		tegra_aic325x_set_dam_cif(i2s->call_record_dam_ifc,
			codec_info->rate, codec_info->channels,
			codec_info->bitsize, 1, bb_info->rate,
			bb_info->channels, bb_info->bitsize);

		/* setup the connections for voice call record */

		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + bb_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + codec_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(i2s->rxcif,
			TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->call_record_dam_ifc);

		/* enable the dam*/

		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN0_SRC);
	}

	return 0;
}

static void tegra_aic325x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	if (!i2s->is_dam_used)
		return;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* disable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
				TEGRA30_DAM_CHIN1);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
					(i2s->dam_ifc*2));

		/* disable the dam and free the controller */
		tegra30_dam_disable_clock(i2s->dam_ifc);
		tegra30_dam_free_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount--;
		if (!i2s->dam_ch_refcount)
			tegra30_dam_free_controller(i2s->dam_ifc);
	 } else {

		if (!i2s->is_call_mode_rec)
			return;

		i2s->is_call_mode_rec = 0;

		/* disable the dam*/
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN0_SRC);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2));
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2));

		/* free the dam channels and dam controller */
		tegra30_dam_disable_clock(i2s->call_record_dam_ifc);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_free_controller(i2s->call_record_dam_ifc);
	 }

	return;
}
#endif


static int tegra_aic325x_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}
#if 0
static int tegra_aic325x_voice_call_hw_params(
			struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err, pcmdiv, vxclkdiv;

	srate = params_rate(params);
	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;


	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	if (machine_is_tegra_enterprise()) {
		err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	} else {
		err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_DSP_B |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	}

	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	if (!machine_is_tegra_enterprise()) {
		if (params_rate(params) == 8000) {
			/* Change these Settings for 8KHz*/
			pcmdiv = 1;
			/* BB expecting 2048Khz bclk */
			vxclkdiv = 27;
		} else if (params_rate(params) == 16000) {
			pcmdiv = 1;
			/* BB expecting 2048Khz bclk */
			vxclkdiv = 27;
		} else {
			dev_err(card->dev, "codec_dai unsupported voice rate\n");
			return -EINVAL;
		}
	}

	/* snd_soc_dai_set_clkdiv(codec_dai, ASI2_BCLK_N, vxclkdiv);
	snd_soc_dai_set_clkdiv(codec_dai, ASI2_WCLK_N, pcmdiv); */

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[HIFI_CODEC].rate = params_rate(params);
	machine->codec_info[HIFI_CODEC].channels = params_channels(params);
	machine->codec_info[HIFI_CODEC].bitsize = 16;
	machine->codec_info[HIFI_CODEC].is_i2smaster = 1;
	machine->codec_info[HIFI_CODEC].is_format_dsp = 0;

	/* baseband configuration */
	machine->codec_info[BASEBAND].bitsize = 16;
	machine->codec_info[BASEBAND].is_i2smaster = 1;
	machine->codec_info[BASEBAND].is_format_dsp = 1;
#endif

	machine->is_device_bt = 0;

	return 0;
}

static void tegra_aic325x_voice_call_shutdown(
					struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

	machine->codec_info[HIFI_CODEC].rate = 0;
	machine->codec_info[HIFI_CODEC].channels = 0;
#endif
}

static int tegra_aic325x_bt_voice_call_hw_params(
			struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	int err, srate, mclk, min_mclk;

	srate = params_rate(params);

	mclk = tegra_aic325x_get_mclk(srate);
	if (mclk < 0)
		return mclk;

	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[BT_SCO].rate = params_rate(params);
	machine->codec_info[BT_SCO].channels = params_channels(params);
	machine->codec_info[BT_SCO].bitsize = 16;
	machine->codec_info[BT_SCO].is_i2smaster = 1;
	machine->codec_info[BT_SCO].is_format_dsp = 1;

	/* baseband configuration */
	machine->codec_info[BASEBAND].bitsize = 16;
	machine->codec_info[BASEBAND].is_i2smaster = 1;
	machine->codec_info[BASEBAND].is_format_dsp = 1;
#endif

	machine->is_device_bt = 1;

	return 0;
}

static void tegra_aic325x_bt_voice_call_shutdown(
				struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_aic325x *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

	machine->codec_info[BT_SCO].rate = 0;
	machine->codec_info[BT_SCO].channels = 0;
#endif
}
#endif
static struct snd_soc_ops tegra_aic325x_hifi_ops = {
	.hw_params = tegra_aic325x_hw_params,
	.hw_free = tegra_aic325x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_aic325x_startup,
	.shutdown = tegra_aic325x_shutdown,
#endif
};
//&*&*&*CT1_130304: Add BT-SOC function
//#if 0
static struct snd_soc_ops tegra_aic325x_spdif_ops = {
	.hw_params = tegra_aic325x_spdif_hw_params,
	.hw_free = tegra_aic325x_hw_free,
};
/*
static struct snd_soc_ops tegra_aic325x_voice_call_ops = {
	.hw_params = tegra_aic325x_voice_call_hw_params,
	.shutdown = tegra_aic325x_voice_call_shutdown,
	.hw_free = tegra_aic325x_hw_free,
};

static struct snd_soc_ops tegra_aic325x_bt_voice_call_ops = {
	.hw_params = tegra_aic325x_bt_voice_call_hw_params,
	.shutdown = tegra_aic325x_bt_voice_call_shutdown,
	.hw_free = tegra_aic325x_hw_free,
};
*/
static struct snd_soc_ops tegra_aic325x_bt_ops = {
	.hw_params = tegra_aic325x_bt_hw_params,
	.hw_free = tegra_aic325x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_aic325x_startup,
	.shutdown = tegra_aic325x_shutdown,
#endif
};

//#endif
//&*&*&*CT2_130304: Add BT-SOC function
static struct snd_soc_jack tegra_aic325x_hp_jack;

//&*&*&*CT1_130122: headset detection using gpio interrupt
static struct snd_soc_jack_gpio tegra_aic325x_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,   
	.invert = 1,       
};
//&*&*&*CT2_130122: headset detection using gpio interrupt

#ifdef CONFIG_SWITCH

static struct switch_dev aic325x_wired_switch_dev = {
	.name = "h2w",
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

//&*&*&*CT1_130129: add enable speaker gpio function
static int get_spk_enable (struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol){
	ucontrol->value.integer.value[0] = spk_enable;
	return 0;
}

static int set_spk_enable (struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol){

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);	
	int enable;
	
	enable = (ucontrol->value.integer.value[0]);
	printk("spk_enable = %d\n", enable);
	setSpkStatus(codec, enable);
	return 0;
}
//&*&*&*CT2_130129: add enable speaker gpio function
//&*&*&*CT1_130314: disable speaker when DAMP set mute as true

int getSpkStatus()
{
	return spk_enable;
}

int setSpkStatus(struct snd_soc_codec *codec, int enable)
{
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	
	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;
	spk_enable = enable;
	printk("pdata->gpio_spkr_en = %d, spk_enable : %d\n", pdata->gpio_spkr_en,spk_enable);	
	gpio_set_value_cansleep(pdata->gpio_spkr_en,
			spk_enable);
	return 0;		
}
//&*&*&*CT2_130314: disable speaker when DAMP set mute as true
//&*&*&*CT1_130122: headset detection using gpio interrupt
#if 0
static int aic325x_headset_switch_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	int state = 0;

	switch (action) {
	case SND_JACK_HEADPHONE:
		state |= BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state |= BIT_HEADSET;
		break;
	default:
		state |= BIT_NO_HEADSET;
	}

	switch_set_state(&aic325x_wired_switch_dev, state);

	return NOTIFY_OK;
}
#else

static int aic325x_headset_switch_notify(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	unsigned char status_jack = 0;

	printk("%s, %ld\n", __FUNCTION__, action);

	if (jack == &tegra_aic325x_hp_jack) {
		if (action) {
			
			status_jack = aic325x_headset_type(codec, 1);

			machine->jack_status &= ~SND_JACK_HEADPHONE;
			machine->jack_status &= ~SND_JACK_MICROPHONE;
			if (status_jack == AIC325X_HEADPHO_DET )
					machine->jack_status |=
							SND_JACK_HEADPHONE;
			else if (status_jack == AIC325X_HEADSET_DET ) {
					machine->jack_status |=
							SND_JACK_HEADPHONE;
					machine->jack_status |=
							SND_JACK_MICROPHONE;
			}
		} else {

			aic325x_headset_type(codec, 0);

			machine->jack_status &= ~SND_JACK_HEADPHONE;
			machine->jack_status &= ~SND_JACK_MICROPHONE;
		}
	}
	
	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state = BIT_HEADSET;
		break;
	case SND_JACK_MICROPHONE:
		/* mic: would not report */
	default:
		state = BIT_NO_HEADSET;
	}
	printk("%s, state 0x%x\n", __FUNCTION__, state);
	switch_set_state(&aic325x_wired_switch_dev, state);

	return NOTIFY_OK;
}

#endif
//&*&*&*CT2_130122: headset detection using gpio interrupt

static struct notifier_block aic325x_headset_switch_nb = {
	.notifier_call = aic325x_headset_switch_notify,
};


#else

static struct snd_soc_jack_pin tegra_aic325x_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif
#if 0
static int tegra_aic325x_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;
	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_aic325x_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

/* Mohan. typecast needs review */
	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}
#endif
static const struct snd_soc_dapm_widget tegra_aic325x_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone jack", NULL),
	SND_SOC_DAPM_LINE("Line out", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_LINE("Linein", NULL),
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("SPK out", NULL),
	//SND_SOC_DAPM_SPK("SPK out", tegra_aic325x_event_int_spk),
};

static const struct snd_soc_dapm_route aic325x_audio_map[] = {

	/* Headphone connected to HPL, HPR */
	{"Headphone jack", NULL, "HPL"},
	{"Headphone jack", NULL, "HPR"},


	/* Line Out connected to LOL, LOR */
	{"Line out", NULL, "LOL"},
	{"Line out", NULL, "LOR"},

	{"SPK out", NULL, "LOL"},
	{"SPK out", NULL, "LOR"},

	{"IN1_L", NULL, "Linein"},
	{"IN1_R", NULL, "Linein"},
	{"IN3_L", NULL, "Ext Mic"},
	{"IN3_R", NULL, "Ext Mic"},

	{"IN1_L", NULL, "Int Mic"}, /*nikesh*/
	{"IN1_R", NULL, "Int Mic"},
};

static const struct snd_kcontrol_new tegra_aic325x_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),

	SOC_DAPM_PIN_SWITCH("Ext Mic"),
	SOC_DAPM_PIN_SWITCH("Linein"),
	SOC_DAPM_PIN_SWITCH("Line out"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("SPK out"),
	//&*&*&*CT1_130129: add enable speaker gpio function
	SOC_SINGLE_EXT("Enable SPK", 0x0, 0, 7, 0, get_spk_enable, set_spk_enable)
	//&*&*&*CT2_130129: add enable speaker gpio function

};

static int tegra_aic325x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	int ret;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (machine->codec_info[BASEBAND].i2s_id != -1)
		i2s->is_dam_used = true;
#endif

	if (machine->init_done)
		return 0;

	machine->init_done = true;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	machine->pcard = card;
#endif

	if (machine_is_whistler()) {
		machine->audio_reg = regulator_get(NULL, "avddio_audio");
		if (IS_ERR(machine->audio_reg)) {
			dev_err(card->dev, "cannot get avddio_audio reg\n");
			ret = PTR_ERR(machine->audio_reg);
			return ret;
		}

		ret = regulator_enable(machine->audio_reg);
		if (ret) {
			dev_err(card->dev, "cannot enable avddio_audio reg\n");
			regulator_put(machine->audio_reg);
			machine->audio_reg = NULL;
			return ret;
		}
	}

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}

//&*&*&*CT1_130122: headset detection using gpio interrupt
#if 0
	ret = snd_soc_jack_new(codec, "Headphone jack", SND_JACK_HEADSET,
			&tegra_aic325x_hp_jack);
	if (ret < 0)
		return ret;

#ifdef CONFIG_SWITCH

	snd_soc_jack_notifier_register(&tegra_aic325x_hp_jack,
		&aic325x_headset_switch_nb);
#else /*gpio based headset detection*/

	snd_soc_jack_add_pins(&tegra_aic325x_hp_jack,
		ARRAY_SIZE(tegra_aic325x_hp_jack_pins),
		tegra_aic325x_hp_jack_pins);

#endif

#else

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_aic325x_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_aic325x_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_aic325x_hp_jack,
					ARRAY_SIZE(tegra_aic325x_hp_jack_pins),
					tegra_aic325x_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_aic325x_hp_jack,
					&aic325x_headset_switch_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_aic325x_hp_jack,
					1,
					&tegra_aic325x_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;
	}

#endif
//&*&*&*CT2_130122: headset detection using gpio interrupt


	/* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_aic325x_call_mode_control,
				machine));
	if (ret < 0)
		return ret;

	snd_soc_dapm_nc_pin(dapm, "IN2_L");
	snd_soc_dapm_nc_pin(dapm, "IN2_R");
	snd_soc_dapm_sync(dapm);

	ret = tegra_asoc_utils_register_ctls(&machine->util_data);
	if (ret < 0) {
		dev_err(card->dev, "Register ctls failed\n");
		return ret;
	}
	return 0;
}

//&*&*&*CT1_130307: headset can't detect after device resume
static int tegra_aic325x_driver_suspend(struct snd_soc_card *card)
{
	
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	printk("tegra_aic325x_driver_suspend\n");
	if(spk_enable == 1)
	{
		spk_enable = 0;
		gpio_set_value_cansleep(pdata->gpio_spkr_en,
			spk_enable);
	}
	return 0;
}

static int tegra_aic325x_driver_resume(struct snd_soc_card *card)
{
	printk("tegra_aic325x_driver_resume\n");
	snd_soc_jack_gpio_detect(&tegra_aic325x_hp_jack_gpio);

	return 0;
}
//&*&*&*CT2_130307: headset can't detect after device resume

static struct snd_soc_dai_link tegra_aic325x_dai[] = {
	[DAI_LINK_HIFI] = {
		.name = "TLV320AIC325x",
		.stream_name = "TLV320AIC325x",
		/*.codec_name = "tlv320aic325x.4-0018",*/
		.codec_name = "tlv320aic325x-codec",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.0",
#else
		.cpu_dai_name = "tegra30-i2s.1",
#endif
		.codec_dai_name = "tlv320aic325x-MM_EXT",
		.init = tegra_aic325x_init,
		.ops = &tegra_aic325x_hifi_ops,
		},
//&*&*&*CT1_130304: Add BT-SOC function
	[DAI_LINK_SPDIF] = {
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-spdif",
#else
		.cpu_dai_name = "tegra30-spdif",
#endif
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_aic325x_spdif_ops,
		},
	[DAI_LINK_BTSCO] = {
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.1",
#else
		.cpu_dai_name = "tegra30-i2s.3",
#endif
		.codec_dai_name = "dit-hifi",
		.init = tegra_aic325x_init,
		.ops = &tegra_aic325x_bt_ops,
		},
//&*&*&*CT2_130304: Add BT-SOC function	
};

static struct snd_soc_card snd_soc_tegra_aic325x = {
	.name = "tegra-aic325x",
	.dai_link = tegra_aic325x_dai,
	.num_links = ARRAY_SIZE(tegra_aic325x_dai),
	.controls =  tegra_aic325x_controls,
	.num_controls = ARRAY_SIZE(tegra_aic325x_controls),
	.dapm_widgets = tegra_aic325x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_aic325x_dapm_widgets),
	.dapm_routes = aic325x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(aic325x_audio_map),
//&*&*&*CT1_130307: headset can't detect after device resume
	.suspend_post = tegra_aic325x_driver_suspend,
	.resume_pre = tegra_aic325x_driver_resume,	
//&*&*&*CT2_130307: headset can't detect after device resume
};

static __devinit int tegra_aic325x_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_aic325x;
	struct tegra_aic325x *machine;
	struct tegra_asoc_platform_data *pdata;
	int ret;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	int i;
#endif
	printk(KERN_ALERT "Entered %s\n", __func__);

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_aic325x), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_aic325x struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);

	if (ret) {
		dev_err(card->dev, "tegra_asoc_utils_init failed\n");
		goto err_free_machine;
	}
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

#ifdef CONFIG_SWITCH
	/* Add h2w switch class support */

	ret = switch_dev_register(&aic325x_wired_switch_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device %d\n",
			ret);
		goto err_fini_utils;
	}

#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC

	for (i = 0; i < NUM_I2S_DEVICES ; i++)
		machine->codec_info[i].i2s_id =
					pdata->i2s_param[i].audio_port_id;

	tegra_aic325x_dai[DAI_LINK_HIFI].cpu_dai_name =
	tegra_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];

#endif

	if (machine_is_tegra_enterprise()) {
#if 1
		tegra_aic325x_dai[DAI_LINK_HIFI].codec_name =
					"tlv320aic325x.0-0018";
#endif
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_switch_unregister;
	}
	if (!card->instantiated) {
		dev_err(&pdev->dev, "No TI AIC3256 codec\n");
		goto err_unregister_card;
	}
	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_switch_unregister:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&aic325x_wired_switch_dev);
#endif
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_aic325x_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_aic325x *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	snd_soc_unregister_card(card);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&aic325x_wired_switch_dev);
#endif

	tegra_asoc_utils_fini(&machine->util_data);
//&*&*&*CT1_130122: headset detection using gpio interrupt
	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_aic325x_hp_jack,
					1,
					&tegra_aic325x_hp_jack_gpio);
//&*&*&*CT2_130122: headset detection using gpio interrupt
	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_aic325x_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_aic325x_driver_probe,
	.remove = __devexit_p(tegra_aic325x_driver_remove),
};

static int __init tegra_aic325x_modinit(void)
{
	return platform_driver_register(&tegra_aic325x_driver);
}

module_init(tegra_aic325x_modinit);

static void __exit tegra_aic325x_modexit(void)
{
	platform_driver_unregister(&tegra_aic325x_driver);
}
module_exit(tegra_aic325x_modexit);

/* Module information */
MODULE_AUTHOR("Vinod G. <vinodg@nvidia.com>");
MODULE_DESCRIPTION("Tegra+AIC3262 machine ASoC driver");
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");

