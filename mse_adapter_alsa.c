/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2016 Renesas Electronics Corporation

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

#include "mse_core.h"

#define MSE_ADAPTER_ALSA_DEVICE_MAX	(2)
#define MSE_ADAPTER_ALSA_PAGE_SIZE	(64 * 1024)

/*************/
/* Structure */
/*************/
/* Strean info */
struct alsa_stream {
	struct snd_pcm_substream	*substream;
	struct mse_audio_config		audio_config;
	int				byte_pos;
	int				period_pos;
	int				byte_per_period;
	int				next_period_byte;
	int				index;
	bool				streaming;
};

/* Device info */
struct alsa_device {
	struct device			dev;
	struct snd_card			*card;
	struct snd_pcm			*pcm;
	int				adapter_index;
	/* spin lock */
	spinlock_t			lock;
	struct alsa_stream		playback;
	struct alsa_stream		capture;
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
				  SNDRV_PCM_FMTBIT_S24_BE,
	.rates			= SNDRV_PCM_RATE_8000_96000,
	.rate_min		= 8000,
	.rate_max		= 96000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 65536,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 1024,
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
				  SNDRV_PCM_FMTBIT_S24_BE,
	.rates			= SNDRV_PCM_RATE_8000_96000,
	.rate_min		= 8000,
	.rate_max		= 96000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 65536,
	.period_bytes_min	= 64,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 1024,
};

/************/
/* Function */
/************/
static inline struct alsa_stream *mse_adapter_alsa_mse_to_io(
						struct alsa_device *chip,
						int index)
{
	if (mse_get_inout(index) == MSE_DIRECTION_INPUT)
		return &chip->playback;
	else
		return &chip->capture;
}

static int mse_adapter_alsa_callback(int index, int size)
{
	struct alsa_device *chip = NULL;
	struct snd_pcm_runtime *runtime;
	struct alsa_stream *io;
	int err;

	pr_debug("[%s]\n", __func__);

	err = mse_get_private_data(index, (void **)&chip);
	if (err < 0) {
		pr_err("[%s] Failed mse_get_private_data() err=%d\n",
		       __func__, err);
		return -EPERM;
	}

	io = mse_adapter_alsa_mse_to_io(chip, index);
	runtime = io->substream->runtime;

	io->byte_pos += io->byte_per_period;
	io->period_pos++;
	io->next_period_byte += io->byte_per_period;
	if (io->period_pos >= runtime->periods) {
		io->byte_pos = 0;
		io->period_pos = 0;
		io->next_period_byte = io->byte_per_period;
	}

	snd_pcm_period_elapsed(io->substream);

	if (!io->streaming) {
		pr_err("[%s] stop streaming\n", __func__);
		return 0;
	}

	err = mse_start_transmission(io->index,
				     runtime->dma_area + io->byte_pos,
				     io->byte_per_period,
				     mse_adapter_alsa_callback);
	if (err < 0) {
		pr_err("[%s] Failed mse_start_transmission() err=%d\n",
		       __func__, err);
		return -EPERM;
	}

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
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int index;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	/* hw setting */
	runtime->hw = g_mse_adapter_alsa_playback_hw;

	/* substream init */
	io->substream = substream;

	/* MSE Core open */
	index = mse_open(chip->adapter_index, MSE_DIRECTION_INPUT);
	if (index < 0) {
		pr_err("[%s] Failed mse_open() index=%d\n", __func__, index);
		return -EPERM;
	}
	io->index = index;

	return 0;
}

static int mse_adapter_alsa_playback_close(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int err;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	/* MSE Core close */
	err = mse_close(io->index);
	if (err < 0) {
		pr_err("[%s] Failed mse_close() err=%d\n", __func__, err);
		return -EPERM;
	}

	return 0;
}

static int mse_adapter_alsa_playback_trigger(
					struct snd_pcm_substream *substream,
					int cmd)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int rtn = 0;
	int err;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		io->substream		= substream;
		io->byte_pos		= 0;
		io->period_pos		= 0;
		io->byte_per_period	= runtime->period_size
					  * runtime->channels
					  * samples_to_bytes(runtime, 1);
		io->next_period_byte	= io->byte_per_period;

		/* config check */
		pr_debug("[%s] ch=%u period_size=%lu fmt_size=%zu\n",
			 __func__, runtime->channels, runtime->period_size,
			 samples_to_bytes(runtime, 1));

		err = mse_start_streaming(io->index);
		if (err < 0) {
			pr_err("[%s] Failed mse_start_streaming() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		io->streaming = true;
		err = mse_start_transmission(io->index,
					     runtime->dma_area + io->byte_pos,
					     io->byte_per_period,
					     mse_adapter_alsa_callback);
		if (err < 0) {
			pr_err("[%s] Failed mse_start_transmission() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		err = mse_stop_streaming(io->index);
		if (err < 0) {
			pr_err("[%s] Failed mse_stop_streaming() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		io->streaming = false;
		break;

	default:
		pr_err("[%s] Invalid argument. cmd=%d\n", __func__, cmd);
		rtn = -EINVAL;
		break;
	}

	return rtn;
}

static int mse_adapter_alsa_capture_open(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int index;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	/* hw setting */
	runtime->hw = g_mse_adapter_alsa_capture_hw;

	/* substream init */
	io->substream = substream;

	/* MSE Core open */
	index = mse_open(chip->adapter_index, MSE_DIRECTION_OUTPUT);
	if (index < 0) {
		pr_err("[%s] Failed mse_open() index=%d\n", __func__, index);
		return -EPERM;
	}
	io->index = index;

	return 0;
}

static int mse_adapter_alsa_capture_close(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int err;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	err = mse_close(io->index);
	if (err < 0) {
		pr_err("[%s] Failed mse_close() err=%d\n", __func__, err);
		return -EPERM;
	}

	return 0;
}

static int mse_adapter_alsa_capture_trigger(
					struct snd_pcm_substream *substream,
					int cmd)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	int rtn = 0;
	int err;

	pr_debug("[%s] cmd=%d\n", __func__, cmd);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		io->substream		= substream;
		io->byte_pos		= 0;
		io->period_pos		= 0;
		io->byte_per_period	= runtime->period_size
					  * runtime->channels
					  * samples_to_bytes(runtime, 1);
		io->next_period_byte	= io->byte_per_period;

		/* config check */
		pr_debug("[%s] ch=%u period_size=%lu fmt_size=%zu\n",
			 __func__, runtime->channels, runtime->period_size,
			 samples_to_bytes(runtime, 1));

		err = mse_start_streaming(io->index);
		if (err < 0) {
			pr_err("[%s] Failed mse_start_streaming() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		io->streaming = true;
		err = mse_start_transmission(io->index,
					     runtime->dma_area + io->byte_pos,
					     io->byte_per_period,
					     mse_adapter_alsa_callback);
		if (err < 0) {
			pr_err("[%s] Failed mse_start_transmission() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		err = mse_stop_streaming(io->index);
		if (err < 0) {
			pr_err("[%s] Failed mse_stop_streaming() err=%d\n",
			       __func__, err);
			rtn = -EPERM;
			break;
		}
		io->streaming = false;
		break;

	default:
		pr_err("[%s] Invalid argument. cmd=%d\n", __func__, cmd);
		rtn = -EINVAL;
		break;
	}

	return rtn;
}

static int mse_adapter_alsa_ioctl(struct snd_pcm_substream *substream,
				  unsigned int cmd,
				  void *arg)
{
	pr_debug("[%s]\n", __func__);
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int mse_adapter_alsa_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *hw_params)
{
	pr_debug("[%s]\n", __func__);
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int mse_adapter_alsa_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("[%s]\n", __func__);
	return snd_pcm_lib_free_pages(substream);
}

static int mse_adapter_alsa_prepare(struct snd_pcm_substream *substream)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);
	struct mse_audio_config config;
	int err;

	pr_debug("[%s]\n", __func__);

	/* parameter check */
	if (!substream) {
		pr_err("[%s] Invalid argument. substream\n", __func__);
		return -EINVAL;
	}

	err = mse_get_audio_config(io->index, &config);
	if (err < 0) {
		pr_err("[%s] Failed mse_get_audio_config() err=%d\n",
		       __func__, err);
		return -EPERM;
	}

	config.sample_rate		= runtime->rate;
	config.sample_format		= runtime->format;
	config.channels			= runtime->channels;
	config.period_size		= runtime->period_size;
	config.bytes_per_sample		= samples_to_bytes(runtime, 1);

	err = mse_set_audio_config(io->index, &config);
	if (err < 0) {
		pr_err("[%s] Failed mse_set_audio_config() err=%d\n",
		       __func__, err);
		return -EPERM;
	}
	io->audio_config = config;

	return 0;
}

static snd_pcm_uframes_t mse_adapter_alsa_pointer(
					struct snd_pcm_substream *substream)
{
	struct alsa_device *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct alsa_stream *io = mse_adapter_alsa_pcm_to_io(chip, substream);

	pr_debug("[%s]\n", __func__);
	return bytes_to_frames(runtime, io->byte_pos);
}

struct snd_pcm_ops g_mse_adapter_alsa_playback_ops = {
	.open		= mse_adapter_alsa_playback_open,
	.close		= mse_adapter_alsa_playback_close,
	.ioctl		= mse_adapter_alsa_ioctl,
	.hw_params	= mse_adapter_alsa_hw_params,
	.hw_free	= mse_adapter_alsa_hw_free,
	.prepare	= mse_adapter_alsa_prepare,
	.trigger	= mse_adapter_alsa_playback_trigger,
	.pointer	= mse_adapter_alsa_pointer,
};

struct snd_pcm_ops g_mse_adapter_alsa_capture_ops = {
	.open		= mse_adapter_alsa_capture_open,
	.close		= mse_adapter_alsa_capture_close,
	.ioctl		= mse_adapter_alsa_ioctl,
	.hw_params	= mse_adapter_alsa_hw_params,
	.hw_free	= mse_adapter_alsa_hw_free,
	.prepare	= mse_adapter_alsa_prepare,
	.trigger	= mse_adapter_alsa_capture_trigger,
	.pointer	= mse_adapter_alsa_pointer,
};

/* Global variable */
static struct alsa_device *g_rchip[MSE_ADAPTER_ALSA_DEVICE_MAX];

static int mse_adapter_alsa_free(struct alsa_device *chip)
{
	int err;

	pr_debug("[%s]\n", __func__);

	if (!chip)
		return 0;

	err = mse_unregister_adapter_media(chip->adapter_index);
	if (err < 0)
		pr_err("[%s] Failed unregister adapter err=%d\n",
		       __func__, err);

	kfree(chip);

	return 0;
}

static int mse_adapter_alsa_dev_free(struct snd_device *device)
{
	pr_debug("[%s]\n", __func__);

	return mse_adapter_alsa_free(device->device_data);
}

static void alsa_chip_dev_release(struct device *dev)
{
	/* reserved */
}

static int mse_adapter_alsa_probe(int devno)
{
	struct snd_pcm *pcm;
	static struct snd_device_ops ops = {
		.dev_free = mse_adapter_alsa_dev_free,
	};
	struct alsa_device *chip;
	struct snd_card *card;
	int err;
	int index;
	char device_name[MSE_NAME_LEN_MAX];

	pr_debug("[%s] devno=%d\n", __func__, devno);

	/* allocate a chip-specific data with zero filled */
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	/* device initialize */
	device_initialize(&chip->dev);
	chip->dev.release = alsa_chip_dev_release;
	dev_set_name(&chip->dev, "mse_alsa%d", devno);
	err = device_add(&chip->dev);
	if (err) {
		pr_err("[%s] Failed device_add() err=%d\n", __func__, err);
		return -EPERM;
	}

	err = snd_card_new(
		&chip->dev,
		SNDRV_DEFAULT_IDX1,
		SNDRV_DEFAULT_STR1,
		THIS_MODULE,
		0,
		&card);
	if (err < 0) {
		pr_err("[%s] Failed snd_card_new() err=%d\n", __func__, err);
		return -EPERM;
	}

	chip->card = card;
	chip->adapter_index = MSE_INDEX_UNDEFINED;

	err = snd_device_new(card, SNDRV_DEV_PCM, chip, &ops);
	if (err < 0) {
		pr_err("[%s] Failed snd_device_new() err=%d\n", __func__, err);
		kfree(chip);
		return -EPERM;
	}

	/* driver name */
	strcpy(card->driver, "renesas-mse");
	strcpy(card->shortname, "Renesas ALSA Adapter");
	sprintf(card->longname, "Renesas MSE ALSA Adapter %d", devno);

	/* other setup */
	err = snd_pcm_new(chip->card, "ALSA Adapter", 0, 1, 1, &pcm);
	if (err < 0) {
		pr_err("[%s] Failed snd_pcm_new() err=%d\n", __func__, err);
		return -EPERM;
	}
	pcm->private_data = chip;
	strcpy(pcm->name, "ALSA Adapter");
	chip->pcm = pcm;

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
		pr_err("[%s] Failed pre-allocation err=%d\n", __func__, err);
		return -EPERM;
	}

	/* regist card */
	err = snd_card_register(card);
	if (err < 0) {
		pr_err("[%s] Failed snd_card_register() err=%d\n",
		       __func__, err);
		return -EPERM;
	}

	sprintf(device_name, "hw:%d,0", card->number);

	/* regist mse */
	index = mse_register_adapter_media(MSE_TYPE_ADAPTER_AUDIO_PCM,
					   "ALSA Adapter",
					   chip,
					   device_name);
	if (index < 0) {
		pr_err("[%s] Failed register adapter index=%d\n",
		       __func__, index);
		return -EPERM;
	}
	chip->adapter_index = index;
	g_rchip[devno] = chip;

	return 0;
}

static int __init mse_adapter_alsa_init(void)
{
	int i, err;

	pr_debug("Start ALSA adapter\n");

	for (i = 0; i < MSE_ADAPTER_ALSA_DEVICE_MAX; i++) {
		err = mse_adapter_alsa_probe(i);
		if (err < 0)
			return err;
	}

	return 0;
}

static void __exit mse_adapter_alsa_exit(void)
{
	int i, err;

	pr_debug("Stop ALSA adapter\n");

	for (i = 0; i < MSE_ADAPTER_ALSA_DEVICE_MAX; i++) {
		err = snd_card_free(g_rchip[i]->card);
		if (err < 0)
			pr_err("[%s] Failed snd_card_free() err=%d\n",
			       __func__, err);
	}
}

module_init(mse_adapter_alsa_init)
module_exit(mse_adapter_alsa_exit)

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
