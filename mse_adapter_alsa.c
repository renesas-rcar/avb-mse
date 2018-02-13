/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2018 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME "/" fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "ravb_mse_kernel.h"

/* ALSA PCM device max */
#define ALSA_PCM_DEVICE_MAX		(8)
#define MSE_ADAPTER_ALSA_DEVICE_MAX	(ALSA_PCM_DEVICE_MAX)
#define MSE_ADAPTER_ALSA_DEVICE_DEFAULT	(2)
#define MSE_ADAPTER_ALSA_PAGE_SIZE	(64 * 1024)

static int alsa_devices = MSE_ADAPTER_ALSA_DEVICE_DEFAULT;
module_param(alsa_devices, int, 0440);

/*************/
/* Structure */
/*************/
/* Strean info */
struct alsa_stream {
	struct snd_pcm_substream	*substream;
	struct mse_audio_config		audio_config;
	int				period_pos;
	int                             next_period_pos;
	int                             periods;
	int				byte_per_period;
	int				index;
	bool				streaming;
};

static inline int pos_inc(struct alsa_stream *io, int pos)
{
	return (pos + 1) % io->periods;
}

/* Device info */
struct alsa_device {
	struct snd_pcm			*pcm;
	int				adapter_index;

	struct alsa_stream		playback;
	struct alsa_stream		capture;
};

/* Adapter info */
struct alsa_adapter {
	struct device			dev;
	struct snd_card			*card;
};

/* hw - Playback */
struct snd_pcm_hardware g_mse_adapter_alsa_playback_hw = {
	.info			= (SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP_VALID),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S24_BE |
				  SNDRV_PCM_FMTBIT_S32_LE |
				  SNDRV_PCM_FMTBIT_S32_BE |
				  SNDRV_PCM_FMTBIT_S18_3LE |
				  SNDRV_PCM_FMTBIT_S18_3BE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S20_3BE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S24_3BE,
	.rates			= SNDRV_PCM_RATE_8000_192000,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 24,
	.buffer_bytes_max	= 65536,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 32,
};

/* hw - Capture */
struct snd_pcm_hardware g_mse_adapter_alsa_capture_hw = {
	.info			= (SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP_VALID),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S24_BE |
				  SNDRV_PCM_FMTBIT_S32_LE |
				  SNDRV_PCM_FMTBIT_S32_BE |
				  SNDRV_PCM_FMTBIT_S18_3LE |
				  SNDRV_PCM_FMTBIT_S18_3BE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S20_3BE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S24_3BE,
	.rates			= SNDRV_PCM_RATE_8000_192000,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 24,
	.buffer_bytes_max	= 65536,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 32,
};

/************/
/* Function */
/************/
static int mse_adapter_alsa_callback(void *priv, int size)
{
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io = priv;
	int err;
	void *buffer;

	mse_debug("START\n");

	if (!io) {
		mse_err("private data is NULL\n");
		return -EPERM;
	}
	runtime = io->substream->runtime;

	if (size < 0) {
		unsigned long flags;

		mse_err("error from mse core %d\n", size);

		snd_pcm_stream_lock_irqsave(io->substream, flags);
		if (snd_pcm_running(io->substream))  {
			snd_pcm_stop(io->substream,
				     SNDRV_PCM_STATE_DISCONNECTED);
		}
		snd_pcm_stream_unlock_irqrestore(io->substream, flags);
		return 0;
	}

	io->period_pos = pos_inc(io, io->period_pos);

	snd_pcm_period_elapsed(io->substream);

	if (!io->streaming) {
		mse_err("stop streaming\n");
		return 0;
	}

	buffer = runtime->dma_area + io->next_period_pos * io->byte_per_period,

	err = mse_start_transmission(
		io->index,
		buffer,
		io->byte_per_period,
		io,
		mse_adapter_alsa_callback);
	if (err == -EAGAIN) {
		return 0;
	} else if (err < 0) {
		mse_err("Failed mse_start_transmission() err=%d\n", err);
		return -EPERM;
	}

	io->next_period_pos = pos_inc(io, io->next_period_pos);

	return 0;
}

static inline struct alsa_stream *mse_adapter_alsa_pcm_to_io(
					struct alsa_device *chip,
					struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return &chip->playback;
	else
		return &chip->capture;
}

static int mse_adapter_alsa_playback_open(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;
	int index;
	int ret;

	mse_debug("START\n");

	/* parameter check */
	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	/* hw setting */
	runtime->hw = g_mse_adapter_alsa_playback_hw;

	/* substream init */
	io->substream = substream;

	/* constraint to be buffer size multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		mse_err("snd_pcm_hw_constraint_integer() %d\n", ret);
		return ret;
	}

	/* MSE Core open */
	index = mse_open(chip->adapter_index, true);
	if (index < 0) {
		mse_err("Failed mse_open() index=%d\n", index);
		return -EPERM;
	}
	io->index = index;

	return 0;
}

static int mse_adapter_alsa_capture_open(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;
	int index;
	int ret;

	mse_debug("START\n");

	/* parameter check */
	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	/* hw setting */
	runtime->hw = g_mse_adapter_alsa_capture_hw;

	/* substream init */
	io->substream = substream;

	/* constraint to be buffer size multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		mse_err("snd_pcm_hw_constraint_integer() %d\n", ret);
		return ret;
	}

	/* MSE Core open */
	index = mse_open(chip->adapter_index, false);
	if (index < 0) {
		mse_err("Failed mse_open() index=%d\n", index);
		return -EPERM;
	}
	io->index = index;

	return 0;
}

static int mse_adapter_alsa_close(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip;
	struct alsa_stream *io;
	int err;

	mse_debug("START\n");

	/* parameter check */
	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	/* MSE Core close */
	err = mse_close(io->index);
	if (err < 0) {
		mse_err("Failed mse_close() err=%d\n", err);
		return -EPERM;
	}

	return 0;
}

static int mse_adapter_alsa_trigger(
				struct snd_pcm_substream *substream,
				int cmd)
{
	struct alsa_device *chip;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;
	void *buffer;
	int rtn = 0;
	int err, i;
	int periods;

	mse_debug("cmd=%d\n", cmd);

	/* parameter check */
	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		io->substream		= substream;
		io->period_pos		= 0;
		io->byte_per_period	= runtime->period_size
					  * runtime->channels
					  * samples_to_bytes(runtime, 1);
		io->next_period_pos	= 0;
		io->periods		= runtime->periods;

		/* config check */
		mse_debug("ch=%u period_size=%lu fmt_size=%zu\n",
			  runtime->channels, runtime->period_size,
			  samples_to_bytes(runtime, 1));

		err = mse_start_streaming(io->index);
		if (err < 0) {
			mse_err("Failed mse_start_streaming() err=%d\n", err);
			rtn = -EPERM;
			break;
		}
		io->streaming = true;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			periods = runtime->start_threshold / runtime->period_size;
		else
			periods = io->periods;

		for (i = 0; i < periods; i++) {
			buffer = runtime->dma_area + io->next_period_pos * io->byte_per_period,

			err = mse_start_transmission(io->index,
						     buffer,
						     io->byte_per_period,
						     io,
						     mse_adapter_alsa_callback);
			if (err == -EAGAIN) {
				break;
			} else if (err < 0) {
				mse_err("Failed mse_start_transmission() err=%d\n",
					err);
				rtn = -EPERM;
				break;
			}

			io->next_period_pos = pos_inc(io, io->next_period_pos);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		err = mse_stop_streaming(io->index);
		if (err < 0) {
			mse_err("Failed mse_stop_streaming() err=%d\n", err);
			rtn = -EPERM;
			break;
		}
		io->streaming = false;
		break;

	default:
		mse_err("Invalid argument. cmd=%d\n", cmd);
		rtn = -EINVAL;
		break;
	}

	return rtn;
}

static int mse_adapter_alsa_ioctl(struct snd_pcm_substream *substream,
				  unsigned int cmd,
				  void *arg)
{
	mse_debug("START\n");
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int mse_adapter_alsa_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *hw_params)
{
	mse_debug("START\n");
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int mse_adapter_alsa_hw_free(struct snd_pcm_substream *substream)
{
	mse_debug("START\n");
	return snd_pcm_lib_free_pages(substream);
}

static enum MSE_AUDIO_BIT get_alsa_bit_depth(int alsa_format)
{
	switch (alsa_format) {
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		return MSE_AUDIO_BIT_32;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
		return MSE_AUDIO_BIT_24;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		return MSE_AUDIO_BIT_20;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_S18_3BE:
		return MSE_AUDIO_BIT_18;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		return MSE_AUDIO_BIT_16;
	default:
		return MSE_AUDIO_BIT_INVALID;
	}
}

static bool is_alsa_big_endian(int alsa_format)
{
	switch (alsa_format) {
	case SNDRV_PCM_FORMAT_S16_BE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S32_BE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_S18_3BE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		return true;
	default:
		return false;
	}
}

static int mse_adapter_alsa_prepare(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;
	struct mse_audio_config config;
	int err;

	mse_debug("START\n");

	/* parameter check */
	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	err = mse_get_audio_config(io->index, &config);
	if (err < 0) {
		mse_err("Failed mse_get_audio_config() err=%d\n", err);
		return -EPERM;
	}

	config.sample_rate		= runtime->rate;
	config.channels			= runtime->channels;
	config.period_size		= runtime->period_size;
	config.bytes_per_sample		= samples_to_bytes(runtime, 1);
	config.sample_bit_depth		= get_alsa_bit_depth(runtime->format);
	config.is_big_endian		= is_alsa_big_endian(runtime->format);

	err = mse_set_audio_config(io->index, &config);
	if (err < 0) {
		mse_err("Failed mse_set_audio_config() err=%d\n", err);
		return -EPERM;
	}
	io->audio_config = config;

	return 0;
}

static snd_pcm_uframes_t mse_adapter_alsa_pointer(
					struct snd_pcm_substream *substream)
{
	struct alsa_device *chip;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;

	if (!substream) {
		mse_err("Invalid argument. substream\n");
		return -EINVAL;
	}

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	io = mse_adapter_alsa_pcm_to_io(chip, substream);

	mse_debug("bytes_to_frames()\n");

	/* return current proccessed bytes that calculated by period position */
	return bytes_to_frames(runtime, io->period_pos * io->byte_per_period);
}

struct snd_pcm_ops g_mse_adapter_alsa_playback_ops = {
	.open		= mse_adapter_alsa_playback_open,
	.close		= mse_adapter_alsa_close,
	.ioctl		= mse_adapter_alsa_ioctl,
	.hw_params	= mse_adapter_alsa_hw_params,
	.hw_free	= mse_adapter_alsa_hw_free,
	.prepare	= mse_adapter_alsa_prepare,
	.trigger	= mse_adapter_alsa_trigger,
	.pointer	= mse_adapter_alsa_pointer,
};

struct snd_pcm_ops g_mse_adapter_alsa_capture_ops = {
	.open		= mse_adapter_alsa_capture_open,
	.close		= mse_adapter_alsa_close,
	.ioctl		= mse_adapter_alsa_ioctl,
	.hw_params	= mse_adapter_alsa_hw_params,
	.hw_free	= mse_adapter_alsa_hw_free,
	.prepare	= mse_adapter_alsa_prepare,
	.trigger	= mse_adapter_alsa_trigger,
	.pointer	= mse_adapter_alsa_pointer,
};

/* Global variable */
static struct alsa_device **g_rchip;
static struct alsa_adapter g_adapter;

static int mse_adapter_alsa_free(struct alsa_adapter *chip)
{
	int err, i;

	mse_debug("START\n");

	if (!chip)
		return 0;

	for (i = 0; i < alsa_devices; i++) {
		if (!g_rchip[i])
			continue;

		err = mse_unregister_adapter_media(g_rchip[i]->adapter_index);
		if (err < 0)
			mse_err("Failed unregister adapter err=%d\n", err);

		kfree(g_rchip[i]);
	}

	return 0;
}

static int mse_adapter_alsa_dev_free(struct snd_device *device)
{
	mse_debug("START\n");

	return mse_adapter_alsa_free(device->device_data);
}

static void alsa_chip_dev_release(struct device *dev)
{
	/* reserved */
}

static int mse_adapter_alsa_probe(struct snd_card *card, int devno)
{
	struct snd_pcm *pcm;
	struct alsa_device *chip;
	int err;
	int index;
	char device_name[MSE_NAME_LEN_MAX];
	char pcm_name[MSE_NAME_LEN_MAX];

	mse_debug("devno=%d\n", devno);

	snprintf(device_name, MSE_NAME_LEN_MAX,
		 "hw:%d,%d", card->number, devno);

	/* regist mse */
	index = mse_register_adapter_media(MSE_TYPE_ADAPTER_AUDIO,
					   "ALSA Adapter",
					   device_name);
	if (index < 0) {
		mse_err("Failed register adapter index=%d\n", index);
		return -EPERM;
	}

	snprintf(pcm_name, MSE_NAME_LEN_MAX, "ravb_mse.mse%d", index);

	/* pcm device initialize */
	err = snd_pcm_new(card, pcm_name, devno, 1, 1, &pcm);
	if (err < 0) {
		mse_err("Failed snd_pcm_new() err=%d\n", err);
		mse_unregister_adapter_media(index);
		return -EPERM;
	}
	strlcpy(pcm->name, pcm_name, sizeof(pcm->name));

	/* set operators */
	snd_pcm_set_ops(pcm,
			SNDRV_PCM_STREAM_PLAYBACK,
			&g_mse_adapter_alsa_playback_ops);
	snd_pcm_set_ops(pcm,
			SNDRV_PCM_STREAM_CAPTURE,
			&g_mse_adapter_alsa_capture_ops);

	/* pre-allocation of buffers */
	err = snd_pcm_lib_preallocate_pages_for_all(
					pcm,
					SNDRV_DMA_TYPE_CONTINUOUS,
					snd_dma_continuous_data(GFP_KERNEL),
					MSE_ADAPTER_ALSA_PAGE_SIZE,
					MSE_ADAPTER_ALSA_PAGE_SIZE);
	if (err < 0) {
		mse_err("Failed pre-allocation err=%d\n", err);
		mse_unregister_adapter_media(index);
		return -EPERM;
	}

	/* allocate a chip-specific data with zero filled */
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		mse_unregister_adapter_media(index);
		return -ENOMEM;
	}

	pcm->private_data = chip;
	chip->pcm = pcm;
	chip->adapter_index = index;
	g_rchip[devno] = chip;

	return 0;
}

static int __init mse_adapter_alsa_init(void)
{
	int i, err;
	struct snd_card *card;
	static struct snd_device_ops ops = {
		.dev_free = mse_adapter_alsa_dev_free,
	};

	mse_debug("Start ALSA adapter\n");

	if (alsa_devices > MSE_ADAPTER_ALSA_DEVICE_MAX) {
		mse_err("Too many devices %d\n", alsa_devices);
		return -EINVAL;
	} else if (alsa_devices <= 0) {
		mse_err("Invalid devices %d\n", alsa_devices);
		return -EINVAL;
	} else {
		;
	}

	/* device initialize */
	device_initialize(&g_adapter.dev);
	g_adapter.dev.release = alsa_chip_dev_release;
	dev_set_name(&g_adapter.dev, "mse_adapter_alsa");
	err = device_add(&g_adapter.dev);
	if (err) {
		mse_err("Failed device_add() err=%d\n", err);
		return -EPERM;
	}

	/* card initialize */
	err = snd_card_new(&g_adapter.dev,
			   SNDRV_DEFAULT_IDX1,
			   SNDRV_DEFAULT_STR1,
			   THIS_MODULE,
			   0,
			   &card);
	if (err < 0) {
		mse_err("Failed snd_card_new() err=%d\n", err);
		device_del(&g_adapter.dev);
		return -EPERM;
	}

	err = snd_device_new(card, SNDRV_DEV_PCM, &g_adapter, &ops);
	if (err < 0) {
		mse_err("Failed snd_device_new() err=%d\n", err);
		goto init_fail;
	}

	/* card name */
	strlcpy(card->id, "ravbmse", sizeof(card->id));
	strlcpy(card->driver, "ravb_mse", sizeof(card->driver));
	strlcpy(card->shortname, "ravb_mse", sizeof(card->shortname));
	strlcpy(card->longname, "ravb_mse", sizeof(card->longname));

	g_rchip = kcalloc(alsa_devices, sizeof(*g_rchip), GFP_KERNEL);
	if (!g_rchip)
		return -ENOMEM;

	for (i = 0; i < alsa_devices; i++) {
		err = mse_adapter_alsa_probe(card, i);
		if (err < 0)
			goto init_fail;
	}

	/* register card */
	err = snd_card_register(card);
	if (err < 0) {
		mse_err("Failed snd_card_register() err=%d\n", err);
		goto init_fail;
	}

	g_adapter.card = card;

	return 0;

init_fail:
	err = snd_card_free(card);
	if (err < 0)
		mse_err("Failed snd_card_free() err=%d\n", err);

	device_del(&g_adapter.dev);

	kfree(g_rchip);

	return -EPERM;
}

static void __exit mse_adapter_alsa_exit(void)
{
	int err;

	mse_debug("Stop ALSA adapter\n");

	err = snd_card_free(g_adapter.card);
	if (err < 0)
		mse_err("Failed snd_card_free() err=%d\n", err);

	device_del(&g_adapter.dev);

	kfree(g_rchip);
}

module_init(mse_adapter_alsa_init)
module_exit(mse_adapter_alsa_exit)

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
