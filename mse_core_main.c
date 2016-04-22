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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include "avtp.h"
#include "mse_core.h"
#include "mse_packet_ctrl.h"
#include "mse_sysfs.h"
#include "mse_packetizer.h"

#define NSEC_SCALE              (1000000000ul)
#define BUF_SIZE                (32)

#define MSE_RADIX_HEXADECIMAL   (16)
#define MSE_DEFAULT_BITRATE     (50000000) /* 50Mbps */
#define MSE_DEFAULT_TIMER       (100000000) /* 100msec */
#ifdef DEBUG_TIMER
#define DEBUG_TIMER_INTERVAL    (50000000) /* 50msec */
#endif

/** @brief MSE's media adapter max */
#define MSE_ADAPTER_MEDIA_MAX   (10)
/** @brief MSE's network adapter max */
#define MSE_ADAPTER_NETWORK_MAX (10)
/** @brief MSE's packetizer max */
#define MSE_PACKETIZER_MAX      (10)
/** @brief MSE's instance max */
#define MSE_INSTANCE_MAX        (10)

#define MSE_DMA_MAX_PACKET         (128)
#define MSE_DMA_MAX_PACKET_SIZE    (1526)
#define MSE_DMA_MAX_RECEIVE_PACKET (100)
#define MSE_DMA_MIN_RECEIVE_PACKET (60)

#define MSE_VIDEO_START_CODE_LEN   (4)

/** @brief mse state */
enum MSE_STATE {
	/** @brief state of close */
	MSE_STATE_CLOSE,
	/** @brief state of ready */
	MSE_STATE_OPEN,
	/** @brief state of execute */
	MSE_STATE_EXECUTE,
};

/** @brief instance by related adapter */
struct mse_instance {
	/** @brief instance used flag */
	bool used_f;
	bool first_f;

	/** @brief instance direction */
	enum MSE_DIRECTION inout;
	/** @brief instance state */
	enum MSE_STATE state;
	/** @brief media adapter IDs */
	int index_media;
	int index_network;
	int index_packetizer;

	/** @brief media adapter info */
	struct mse_adapter *media;
	/** @brief network adapter ops */
	struct mse_adapter_network_ops *network;
	/** @brief packetizer ops */
	struct mse_packetizer_ops *packetizer;

	/** @brief streaming queue */
	struct work_struct wk_stream;
	/** @brief paketize queue */
	struct work_struct wk_packetize;
	/** @brief depaketize queue */
	struct work_struct wk_depacketize;
	/** @brief callback queue */
	struct work_struct wk_callback;
	/** @brief stop queue */
	struct work_struct wk_stop;

	/** @brief stream workqueue */
	struct workqueue_struct *wq_stream;
	/** @brief packet workqueue */
	struct workqueue_struct *wq_packet;

	/** @brief timer handler */
	struct hrtimer timer;
	int timer_delay;
	int timer_cnt;

	/** @brief timestamp(nsec) */
	unsigned int timestamp;

	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;
	/** @brief media buffer */
	unsigned char *media_buffer;
	/** @brief buffer length for packetize */
	size_t media_buffer_size;
	/** @brief complete function pointer */
	int (*mse_completion)(int index, int size);
	/** @brief work length for media buffer */
	int work_length;

	/** @brief network configuration */
	struct mse_network_config net_config;
	/** @brief media specific configuration */
	union mse_media_config {
		struct mse_audio_config audio;
		struct mse_video_config video;
	} media_config;
};

struct mse_device {
	/** @brief device */
	struct platform_device *pdev;
	/** @brief device lock */
	spinlock_t lock;

	struct mse_packetizer_ops *packetizer_table[MSE_PACKETIZER_MAX];
	struct mse_adapter_network_ops *network_table[MSE_ADAPTER_NETWORK_MAX];
	struct mse_adapter media_table[MSE_ADAPTER_MEDIA_MAX];
	struct mse_instance instance_table[MSE_INSTANCE_MAX];
};

/*
 * global parameters
 */
static struct mse_sysfs_config mse_sysfs_config_audio[] = {
	{
		.name = MSE_SYSFS_NAME_STR_TYPE,
		.type = MSE_TYPE_ENUM,
		.int_value = 2,
		.enum_list = {"talker", "listener", "both", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_KIND,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"audio", "video", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_DST_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "91e0f0000e80",
	},
	{
		.name = MSE_SYSFS_NAME_STR_SRC_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "769050000000",
	},
	{
		.name = MSE_SYSFS_NAME_STR_VLAN,
		.type = MSE_TYPE_INT,
		.int_value = 2,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PRIORITY,
		.type = MSE_TYPE_INT,
		.int_value = 3,
	},
	{
		.name = MSE_SYSFS_NAME_STR_UNIQUE_ID,
		.type = MSE_TYPE_INT,
		.int_value = 1,
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_ADAPTER_NAME,
		.type = MSE_TYPE_STR,
		.str_value = "ravb",
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_TX,
		.type = MSE_TYPE_STR,
		.str_value = "ravb_tx0",
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_RX,
		.type = MSE_TYPE_STR,
		.str_value = "ravb_rx0",
	},
	{
		.name = MSE_SYSFS_NAME_STR_PACKETIZER_NAME,
		.type = MSE_TYPE_STR,
		.str_value = MSE_PACKETIZER_NAME_STR_AAF_PCM,
	},
	{
		.name = NULL, /* end of table */
	}
};

static struct mse_sysfs_config mse_sysfs_config_video[] = {
	{
		.name = MSE_SYSFS_NAME_STR_TYPE,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"talker", "listener", "both", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_KIND,
		.type = MSE_TYPE_ENUM,
		.int_value = 1,
		.enum_list = {"audio", "video", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_DST_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "91e0f0000e80",
	},
	{
		.name = MSE_SYSFS_NAME_STR_SRC_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "769050000000",
	},
	{
		.name = MSE_SYSFS_NAME_STR_VLAN,
		.type = MSE_TYPE_INT,
		.int_value = 2,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PRIORITY,
		.type = MSE_TYPE_INT,
		.int_value = 3,
	},
	{
		.name = MSE_SYSFS_NAME_STR_UNIQUE_ID,
		.type = MSE_TYPE_INT,
		.int_value = 1,
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_ADAPTER_NAME,
		.type = MSE_TYPE_STR,
		.str_value = "ravb",
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_TX,
		.type = MSE_TYPE_STR,
		.str_value = "ravb_tx0",
	},
	{
		.name = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_RX,
		.type = MSE_TYPE_STR,
		.str_value = "ravb_rx0",
	},
	{
		.name = MSE_SYSFS_NAME_STR_PACKETIZER_NAME,
		.type = MSE_TYPE_STR,
		.str_value = MSE_PACKETIZER_NAME_STR_CVF_H264_D13,
	},
	{
		.name = MSE_SYSFS_NAME_STR_FPS_SECONDS,
		.type = MSE_TYPE_INT,
		.int_value = 1,
	},
	{
		.name = MSE_SYSFS_NAME_STR_FPS_FRAMES,
		.type = MSE_TYPE_INT,
		.int_value = 30,
	},
	{
		.name = MSE_SYSFS_NAME_STR_BITRATE,
		.type = MSE_TYPE_INT,
		.int_value = MSE_DEFAULT_BITRATE,
	},
	{
		.name = NULL, /* end of table */
	}
};

static int mse_create_config_device(struct mse_adapter *adapter)
{
	int ret;

	pr_debug("[%s]\n", __func__);

	switch (MSE_TYPE_KING_GET(adapter->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
		adapter->sysfs_config = kzalloc(sizeof(mse_sysfs_config_audio),
						GFP_KERNEL);
		memcpy(adapter->sysfs_config, mse_sysfs_config_audio,
		       sizeof(mse_sysfs_config_audio));
		break;
	case MSE_TYPE_ADAPTER_VIDEO:
		adapter->sysfs_config = kzalloc(sizeof(mse_sysfs_config_video),
						GFP_KERNEL);
		memcpy(adapter->sysfs_config, mse_sysfs_config_video,
		       sizeof(mse_sysfs_config_video));
		break;
	default:
		pr_err("[%s] undefined type=%d\n", __func__, adapter->type);
		return -EPERM;
	}

	ret = mse_sysfs_init(adapter->sysfs_config);
	if (ret < 0) {
		pr_err("[%s] failed mse_sysfs_init() ret=%d\n", __func__, ret);
		kfree(adapter->sysfs_config);
		return ret;
	}
	adapter->index_sysfs = ret;

	return 0;
}

static int mse_delete_config_device(struct mse_adapter *adapter)
{
	pr_debug("[%s]\n", __func__);

	mse_sysfs_exit(adapter->index_sysfs);
	kfree(adapter->sysfs_config);
	adapter->index_sysfs = MSE_INDEX_UNDEFINED;

	return 0;
}

static int change_mac_addr(unsigned char *dest, const char *in_str)
{
	union {
		unsigned long long hex;
		unsigned char byte[sizeof(unsigned long long)];
	} decode;
	int err, i, j;

	err = kstrtoll(in_str, MSE_RADIX_HEXADECIMAL, &decode.hex);
	for (i = 0, j = MSE_MAC_LEN_MAX - 1; i < MSE_MAC_LEN_MAX; i++, j--)
		dest[i] = decode.byte[j];

	return err;
}

static int mse_get_default_config(int index, struct mse_instance *instance)
{
	int ret = 0, err;
	unsigned char mac[BUF_SIZE];
	struct mse_network_config *network = &instance->net_config;
	struct mse_video_config *video = &instance->media_config.video;

	err = mse_sysfs_get_config_str(index,
				       MSE_SYSFS_NAME_STR_DST_MAC,
				       mac,
				       sizeof(mac));
	if (err < 0) {
		pr_err("[%s] undefined destination MAC\n", __func__);
		ret = -EPERM;
	}
	err = change_mac_addr(network->dest_addr, mac);
	if (err < 0) {
		pr_err("[%s] irregular destination MAC\n", __func__);
		ret = -EPERM;
	}
	err = mse_sysfs_get_config_str(index,
				       MSE_SYSFS_NAME_STR_SRC_MAC,
				       mac,
				       sizeof(mac));
	if (err < 0) {
		pr_err("[%s] undefined source MAC\n", __func__);
		ret = -EPERM;
	}
	err = change_mac_addr(network->source_addr, mac);
	if (err < 0) {
		pr_err("[%s] irregular source MAC\n", __func__);
		ret = -EPERM;
	}
	err = mse_sysfs_get_config_int(index,
				       MSE_SYSFS_NAME_STR_PRIORITY,
				       &network->priority);
	if (err < 0) {
		pr_err("[%s] undefined priority\n", __func__);
		ret = -EPERM;
	}
	err = mse_sysfs_get_config_int(index,
				       MSE_SYSFS_NAME_STR_VLAN,
				       &network->vlanid);
	if (err < 0) {
		pr_err("[%s] undefined VLAN ID\n", __func__);
		ret = -EPERM;
	}
	err = mse_sysfs_get_config_int(index,
				       MSE_SYSFS_NAME_STR_UNIQUE_ID,
				       &network->uniqueid);
	if (err < 0) {
		pr_err("[%s] undefined unique ID\n", __func__);
		ret = -EPERM;
	}

	switch (MSE_TYPE_KING_GET(instance->media->type)) {
	case MSE_TYPE_ADAPTER_VIDEO:
		err = mse_sysfs_get_config_int(index,
					       MSE_SYSFS_NAME_STR_FPS_SECONDS,
					       &video->fps.n);
		if (err < 0) {
			pr_err("[%s] undefined seconds of fps\n", __func__);
			ret = -EPERM;
		}
		err = mse_sysfs_get_config_int(index,
					       MSE_SYSFS_NAME_STR_FPS_FRAMES,
					       &video->fps.m);
		if (err < 0) {
			pr_err("[%s] undefined frames of fps\n", __func__);
			ret = -EPERM;
		}
		err = mse_sysfs_get_config_int(index,
					       MSE_SYSFS_NAME_STR_BITRATE,
					       &video->bitrate);
		if (err < 0) {
			pr_err("[%s] undefined bitrate\n", __func__);
			ret = -EPERM;
		}
		break;

	default:
		/* fall through */
		break;
	}

	return ret;
}

static void mse_work_stream(struct work_struct *work)
{
	struct mse_instance *instance;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_stream);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		/* request send packet */
		mse_packet_ctrl_send_packet(instance->index_network,
					    instance->packet_buffer,
					    instance->network);
	} else {
		/* request receive packet */
		mse_packet_ctrl_receive_packet(instance->index_network,
					       MSE_DMA_MAX_RECEIVE_PACKET,
					       instance->packet_buffer,
					       instance->network);
	}
}

static void mse_work_packetize(struct work_struct *work)
{
	struct mse_instance *instance;
	int ret;
	struct timespec time;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_packetize);

	switch (MSE_TYPE_KING_GET(instance->media->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
		/* make AVTP packet */
		ret = mse_packet_ctrl_make_packet(instance->index_packetizer,
						  instance->media_buffer,
						  instance->media_buffer_size,
						  &instance->timestamp,
						  instance->packet_buffer,
						  instance->packetizer);
		if (ret < 0)
			return;
		pr_debug("[%s] media_buffer=%p packetized=%d\n",
			 __func__, instance->media_buffer, ret);
		/* start workqueue for streaming */
		queue_work(instance->wq_stream, &instance->wk_stream);
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
		/* get timestamp(nsec) */
		getnstimeofday(&time);
		instance->timestamp = time.tv_nsec;
		/* make AVTP packet */
		ret = mse_packet_ctrl_make_packet(
			instance->index_packetizer,
			instance->media_buffer + instance->work_length,
			instance->media_buffer_size - instance->work_length,
			&instance->timestamp,
			instance->packet_buffer,
			instance->packetizer);
		if (ret < 0)
			return;
		pr_debug("[%s] media_buffer=%p packetized=%d\n",
			 __func__, instance->media_buffer, ret);
		instance->work_length += ret;
		/* start workqueue for streaming */
		queue_work(instance->wq_stream, &instance->wk_stream);
		break;

	default:
		pr_err("[%s] unknown type=0x%08x\n",
		       __func__, instance->media->type);
		break;
	}
}

static void mse_work_depacketize(struct work_struct *work)
{
	struct mse_instance *instance;
	int timestamp = 0;
	int received, ret = -1;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_depacketize);

	received = mse_packet_ctrl_check_packet_remain(
						instance->packet_buffer);

	switch (MSE_TYPE_KING_GET(instance->media->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
		if (received > MSE_DMA_MIN_RECEIVE_PACKET ||
		    instance->first_f) {
			/* make AVTP packet */
			ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						instance->media_buffer,
						instance->media_buffer_size,
						&timestamp,
						instance->packet_buffer,
						instance->packetizer);
			instance->first_f = true;
		}
		pr_debug("[%s] media_buffer=%p received=%d depacketized=%d\n",
			 __func__, instance->media_buffer, received, ret);
		/* complete callback */
		instance->mse_completion(instance->index_media, 0);
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
		/* make AVTP packet */
		ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						instance->media_buffer,
						instance->media_buffer_size,
						&timestamp,
						instance->packet_buffer,
						instance->packetizer);
		pr_debug("[%s] media_buffer=%p received=%d depacketized=%d\n",
			 __func__, instance->media_buffer, received, ret);
		/* complete callback */
		instance->mse_completion(instance->index_media, ret);
		break;

	default:
		pr_err("[%s] unknown type=0x%08x\n",
		       __func__, instance->media->type);
		break;
	}
}

static void mse_work_callback(struct work_struct *work)
{
	struct mse_instance *instance;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_callback);

	/* complete callback */
	instance->mse_completion(instance->index_media, 0);
}

static void mse_work_stop(struct work_struct *work)
{
	struct mse_instance *instance;
	int ret;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_stop);

	ret = instance->network->cancel(instance->index_network);
	if (ret)
		pr_err("[%s] failed cancel() ret=%d\n", __func__, ret);

	/* dequeue for packetize */
	if (instance->inout == MSE_DIRECTION_INPUT)
		flush_work(&instance->wk_callback);
	else
		flush_work(&instance->wk_depacketize);

	/* dequeue */
	flush_work(&instance->wk_stream);
}

static enum hrtimer_restart mse_timer_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	ktime_t ktime;
	int ret;

	instance = container_of(arg, struct mse_instance, timer);
	adapter = instance->media;

	mutex_lock(&adapter->lock);

	/* timer update */
	ktime = ktime_set(0, instance->timer_delay);
	hrtimer_forward(&instance->timer,
			hrtimer_get_expires(&instance->timer),
			ktime);

	if (!instance->timer_cnt) {
		mutex_unlock(&adapter->lock);
		return HRTIMER_RESTART;
	}
	instance->timer_cnt--;

	if (MSE_TYPE_KING_GET(adapter->type) == MSE_TYPE_ADAPTER_VIDEO &&
	    (instance->inout == MSE_DIRECTION_OUTPUT)) {
		ret = instance->network->cancel(instance->index_network);
		if (ret)
			pr_err("[%s] failed cancel() ret=%d\n", __func__, ret);
	}

	mutex_unlock(&adapter->lock);
	if (instance->inout == MSE_DIRECTION_INPUT)
		/* start workqueue for completion */
		queue_work(instance->wq_packet, &instance->wk_callback);
	else
		/* start workqueue for depacketize */
		queue_work(instance->wq_packet, &instance->wk_depacketize);

	return HRTIMER_RESTART;
}

/* MSE device data */
static struct mse_device *mse;

/* External function */
int mse_register_adapter_media(enum MSE_TYPE type, char *name, void *data)
{
	int i;

	/* check argument */
	if (!name) {
		pr_err("[%s] invalid argument. name\n", __func__);
		return -EINVAL;
	}
	if (!data) {
		pr_err("[%s] invalid argument. data\n", __func__);
		return -EINVAL;
	}

	switch (MSE_TYPE_KING_GET(type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
	case MSE_TYPE_ADAPTER_VIDEO:
		break;
	default:
		pr_err("[%s] unknown type=%d\n", __func__, type);
		return -EINVAL;
	}

	pr_debug("[%s] type=%d name=%s\n", __func__, type, name);
	spin_lock(&mse->lock);

	/* register table */
	for (i = 0; i < ARRAY_SIZE(mse->media_table); i++) {
		if (!mse->media_table[i].used_f) {
			/* init table */
			mse->media_table[i].used_f = true;
			mse->media_table[i].type = type;
			mse->media_table[i].private_data = data;
			strncpy(mse->media_table[i].name, name,
				MSE_NAME_LEN_MAX);
			/* create control device */
			mse_create_config_device(&mse->media_table[i]);
			pr_debug("[%s] registered index=%d\n", __func__, i);
			spin_unlock(&mse->lock);
			return i;
		}
	}
	pr_err("[%s] %s was unregistered\n", __func__, name);
	spin_unlock(&mse->lock);
	return -EBUSY;
}
EXPORT_SYMBOL(mse_register_adapter_media);

int mse_unregister_adapter_media(int index)
{
	int i;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);
	spin_lock(&mse->lock);

	if (!mse->media_table[index].used_f) {
		pr_err("[%s] %d was unregistered\n", __func__, index);
		spin_unlock(&mse->lock);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].index_media == index) {
			pr_err("[%s] module is in use. instance=%d\n",
			       __func__, i);
			spin_unlock(&mse->lock);
			return -EPERM;
		}
	}

	/* delete control device */
	mse_delete_config_device(&mse->media_table[index]);

	/* table delete */
	memset(&mse->media_table[index], 0, sizeof(struct mse_adapter));
	pr_debug("[%s] unregistered\n", __func__);
	spin_unlock(&mse->lock);

	return 0;
}
EXPORT_SYMBOL(mse_unregister_adapter_media);

int mse_register_adapter_network(struct mse_adapter_network_ops *ops)
{
	int i;

	/* check argument */
	if (!ops) {
		pr_err("[%s] invalid argument. ops\n", __func__);
		return -EINVAL;
	}
	if (!ops->name) {
		pr_err("[%s] empty data. ops->name\n", __func__);
		return -EINVAL;
	}

	switch (MSE_TYPE_KING_GET(ops->type)) {
	case MSE_TYPE_ADAPTER_NETWORK:
		break;
	default:
		pr_err("[%s] unknown type=%d\n", __func__, ops->type);
		return -EINVAL;
	}

	pr_debug("[%s] type=%d name=%s\n", __func__, ops->type, ops->name);
	spin_lock(&mse->lock);

	/* register table */
	for (i = 0; i < ARRAY_SIZE(mse->network_table); i++) {
		if (!mse->network_table[i]) {
			mse->network_table[i] = ops;
			pr_debug("[%s] registered index=%d\n", __func__, i);
			spin_unlock(&mse->lock);
			return i;
		}
	}
	pr_err("[%s] %s was unregistered\n", __func__, ops->name);
	spin_unlock(&mse->lock);
	return -EPERM;
}
EXPORT_SYMBOL(mse_register_adapter_network);

int mse_unregister_adapter_network(int index)
{
	if ((index < 0) || (index >= MSE_ADAPTER_NETWORK_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);
	spin_lock(&mse->lock);

	mse->network_table[index] = NULL;

	spin_unlock(&mse->lock);
	return 0;
}
EXPORT_SYMBOL(mse_unregister_adapter_network);

int mse_register_packetizer(struct mse_packetizer_ops *ops)
{
	int i;

	/* check argument */
	if (!ops) {
		pr_err("[%s] invalid argument. ops\n", __func__);
		return -EINVAL;
	}
	if (!ops->name) {
		pr_err("[%s] empty data. ops->name\n", __func__);
		return -EINVAL;
	}

	switch (MSE_TYPE_KING_GET(ops->type)) {
	case MSE_TYPE_PACKETIZER:
		break;
	default:
		pr_err("[%s] unknown type=%d\n", __func__, ops->type);
		return -EINVAL;
	}

	pr_debug("[%s] type=%d name=%s\n", __func__, ops->type, ops->name);
	spin_lock(&mse->lock);

	/* register table */
	for (i = 0; i < ARRAY_SIZE(mse->packetizer_table); i++) {
		if (!mse->packetizer_table[i]) {
			mse->packetizer_table[i] = ops;
			pr_debug("[%s] registered index=%d\n", __func__, i);
			spin_unlock(&mse->lock);
			return i;
		}
	}
	pr_err("[%s] %s was unregistered\n", __func__, ops->name);
	spin_unlock(&mse->lock);
	return -EPERM;
}
EXPORT_SYMBOL(mse_register_packetizer);

int mse_unregister_packetizer(int index)
{
	if ((index < 0) || (index >= MSE_PACKETIZER_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);
	spin_lock(&mse->lock);

	mse->packetizer_table[index] = NULL;

	spin_unlock(&mse->lock);
	return 0;
}
EXPORT_SYMBOL(mse_unregister_packetizer);

int mse_get_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	if (!config) {
		pr_err("[%s] invalid argument. config\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d data=%p\n", __func__, index, config);

	instance = &mse->instance_table[index];

	/* get config */
	memcpy(config, &instance->media_config.audio, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_audio_config);

int mse_set_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;
	u64 timer;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	if (!config) {
		pr_err("[%s] invalid argument. config\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d data=%p\n", __func__, index, config);
	pr_info("[%s]\n  sample_rate=%d sample_format=%d channels=%d\n"
		"  period_size=%d bytes_par_sample=%d\n", __func__,
		config->sample_rate, config->sample_format, config->channels,
		config->period_size, config->bytes_par_sample);

	instance = &mse->instance_table[index];

	/* set config */
	memcpy(&instance->media_config.audio, config, sizeof(*config));

	/* calc timer value */
#ifdef DEBUG_TIMER
	timer = DEBUG_TIMER_INTERVAL;
#else /* DEBUG_TIMER */
	timer = NSEC_SCALE * (u64)config->period_size;
	do_div(timer, config->sample_rate);
#endif /* DEBUG_TIMER */
	instance->timer_delay = timer;
	pr_notice("[%s] timer_delay=%d\n", __func__, instance->timer_delay);

	/* set AVTP header info */
	instance->packetizer->set_network_config(0, &instance->net_config);
	/* init packet header */
	instance->packetizer->set_audio_config(0, config);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		struct eavb_cbsparam cbs;

		instance->packetizer->calc_cbs(0, &cbs);
		instance->network->set_cbs_param(instance->index_network,
						 &cbs);
	} else {
		u8 streamid[AVTP_STREAMID_SIZE];

		mse_make_streamid(streamid,
				  instance->net_config.source_addr,
				  instance->net_config.uniqueid);
		instance->network->set_streamid(instance->index_network,
						streamid);
	}

	/* get packet memory */
	instance->packet_buffer = mse_packet_ctrl_alloc(
						&mse->pdev->dev,
						MSE_DMA_MAX_PACKET,
						MSE_DMA_MAX_PACKET_SIZE);

	return 0;
}
EXPORT_SYMBOL(mse_set_audio_config);

int mse_get_video_config(int index, struct mse_video_config *config)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	if (!config) {
		pr_err("[%s] invalid argument. config\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d data=%p\n", __func__, index, config);

	instance = &mse->instance_table[index];

	/* get config */
	memcpy(config, &instance->media_config.video, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_video_config);

int mse_set_video_config(int index, struct mse_video_config *config)
{
	struct mse_instance *instance;
	u64 framerate;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	if (!config) {
		pr_err("[%s] invalid argument. config\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d data=%p\n", __func__, index, config);
	pr_info("[%s]\n  format=%d bitrate=%d fps=%d/%d\n"
		"  buffers=%d height=%d width=%d color_space=%d\n"
		"  interlaced=%d bytes_per_frame=%d\n", __func__,
		config->format, config->bitrate, config->fps.n, config->fps.m,
		config->buffers, config->height, config->width,
		config->color_space, config->interlaced,
		config->bytes_per_frame);

	instance = &mse->instance_table[index];

	/* set config */
	memcpy(&instance->media_config.video, config, sizeof(*config));

	/* calc timer value */
#ifdef DEBUG_TIMER
	framerate = DEBUG_TIMER_INTERVAL;
#else /* DEBUG_TIMER */
	framerate = NSEC_SCALE * (u64)config->fps.n;
	do_div(framerate, config->fps.m);
#endif /* DEBUG_TIMER */
	instance->timer_delay = framerate;
	pr_notice("[%s] timer_delay=%d\n", __func__, instance->timer_delay);

	/* set AVTP header info */
	instance->packetizer->set_network_config(0, &instance->net_config);
	/* init packet header */
	instance->packetizer->set_video_config(0, config);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		struct eavb_cbsparam cbs;

		instance->packetizer->calc_cbs(0, &cbs);
		instance->network->set_cbs_param(instance->index_network,
						 &cbs);
	} else {
		u8 streamid[AVTP_STREAMID_SIZE];

		mse_make_streamid(streamid,
				  instance->net_config.source_addr,
				  instance->net_config.uniqueid);
		instance->network->set_streamid(instance->index_network,
						streamid);
	}

	/* get packet memory */
	instance->packet_buffer = mse_packet_ctrl_alloc(
						&mse->pdev->dev,
						MSE_DMA_MAX_PACKET,
						MSE_DMA_MAX_PACKET_SIZE);

	return 0;
}
EXPORT_SYMBOL(mse_set_video_config);

int mse_open(int index_media, enum MSE_DIRECTION inout)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mse_adapter_network_ops *network;
	struct mse_packetizer_ops *packetizer;
	int ret, i, j;
	char name[MSE_NAME_LEN_MAX];
	char eavbname[MSE_NAME_LEN_MAX], eavb_device[MSE_NAME_LEN_MAX];
	char *sysfsnet;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n",
		       __func__, index_media);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d inout=%d\n", __func__, index_media, inout);

	adapter = &mse->media_table[index_media];
	mutex_lock(&adapter->lock);

	if (!adapter->used_f) {
		pr_err("[%s] undefined media adapter index=%d\n",
		       __func__, index_media);
		mutex_unlock(&adapter->lock);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (!mse->instance_table[i].used_f)
			break;
	}
	if (ARRAY_SIZE(mse->instance_table) <= i) {
		pr_err("[%s] resister instance full!\n", __func__);
		mutex_unlock(&adapter->lock);
		return -EBUSY;
	}

	instance = &mse->instance_table[i];

	/* get sysfs value */
	memset(eavbname, 0, MSE_NAME_LEN_MAX);
	ret = mse_sysfs_get_config_str(adapter->index_sysfs,
				       MSE_SYSFS_NAME_STR_NETWORK_ADAPTER_NAME,
				       eavbname,
				       MSE_NAME_LEN_MAX);
	if (ret < 0) {
		pr_err("[%s] undefined network adapter name\n", __func__);
		mutex_unlock(&adapter->lock);
		return -EPERM;
	}

	/* search network adapter name for configuration value */
	for (j = 0; j < ARRAY_SIZE(mse->network_table); j++) {
		network = mse->network_table[j];
		if (!network)
			continue;
		if (!strncmp(network->name, eavbname, strlen(network->name)))
			break;
	}
	if (j >= ARRAY_SIZE(mse->network_table)) {
		pr_err("[%s] network adapter not found\n", __func__);
		mutex_unlock(&adapter->lock);
		return -ENODEV;
	}
	pr_debug("[%s] network adapter index=%d name=%s\n",
		 __func__, j, network->name);

	/* get configuration value */
	memset(name, 0, MSE_NAME_LEN_MAX);
	ret = mse_sysfs_get_config_str(adapter->index_sysfs,
				       MSE_SYSFS_NAME_STR_PACKETIZER_NAME,
				       name,
				       MSE_NAME_LEN_MAX);
	if (ret < 0) {
		pr_err("[%s] undefined packetizer name\n", __func__);
		mutex_unlock(&adapter->lock);
		return -EPERM;
	}

	/* search packetizer name for configuration value */
	for (j = 0; j < ARRAY_SIZE(mse->packetizer_table); j++) {
		packetizer = mse->packetizer_table[j];
		if (!packetizer)
			continue;
		if (!strncmp(packetizer->name, name, strlen(packetizer->name)))
			break;
	}
	if (j >= ARRAY_SIZE(mse->packetizer_table)) {
		pr_err("[%s] packetizer not found\n", __func__);
		mutex_unlock(&adapter->lock);
		return -ENODEV;
	}
	pr_debug("[%s] packetizer index=%d name=%s\n",
		 __func__, j, packetizer->name);

	/* open network adapter */
	if (inout != MSE_DIRECTION_INPUT)
		sysfsnet = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_RX;
	else
		sysfsnet = MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_TX;
	ret = mse_sysfs_get_config_str(adapter->index_sysfs,
				       sysfsnet,
				       eavb_device,
				       MSE_NAME_LEN_MAX);
	ret = network->open(eavb_device);
	if (ret < 0) {
		pr_err("[%s] cannot open network adapter ret=%d\n",
		       __func__, ret);
		mutex_unlock(&adapter->lock);
		return ret;
	}
	instance->index_network = ret;

	/* open paketizer */
	ret = packetizer->open();
	if (ret < 0) {
		pr_err("[%s] cannot open packetizer ret=%d\n", __func__, ret);
		network->release(instance->index_network);
		mutex_unlock(&adapter->lock);
		return ret;
	}
	instance->index_packetizer = ret;

	/* set selector */
	instance->used_f = true;
	instance->inout = inout;
	instance->state = MSE_STATE_OPEN;
	instance->media = adapter;
	instance->media->inout = inout;
	instance->packetizer = packetizer;
	instance->network = network;
	instance->index_media = index_media;

	/* set table */
	INIT_WORK(&instance->wk_packetize, mse_work_packetize);
	INIT_WORK(&instance->wk_depacketize, mse_work_depacketize);
	INIT_WORK(&instance->wk_callback, mse_work_callback);
	INIT_WORK(&instance->wk_stream, mse_work_stream);
	INIT_WORK(&instance->wk_stop, mse_work_stop);

	instance->wq_stream = create_singlethread_workqueue("mse_streamq");
	instance->wq_packet = create_singlethread_workqueue("mse_packetq");
	hrtimer_init(&instance->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	instance->timer_delay = MSE_DEFAULT_TIMER;
	instance->timer.function = &mse_timer_callback;

	mutex_unlock(&adapter->lock);

	ret = mse_get_default_config(adapter->index_sysfs, instance);
	if (ret < 0)
		pr_err("[%s] cannot get configurations\n", __func__);

	return i;
}
EXPORT_SYMBOL(mse_open);

int mse_close(int index)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	adapter = instance->media;

	mutex_lock(&adapter->lock);

	/* free packet memory */
	mse_packet_ctrl_free(instance->packet_buffer);

	/* destroy workqueue */
	destroy_workqueue(instance->wq_packet);
	destroy_workqueue(instance->wq_stream);

	/* release network adapter */
	instance->network->release(instance->index_network);

	/* release packetizer */
	instance->packetizer->release(instance->index_packetizer);

	/* set table */
	memset(instance, 0, sizeof(*instance));
	instance->state = MSE_STATE_CLOSE;
	instance->index_media = MSE_INDEX_UNDEFINED;
	instance->index_network = MSE_INDEX_UNDEFINED;
	instance->index_packetizer = MSE_INDEX_UNDEFINED;

	mutex_unlock(&adapter->lock);
	return 0;
}
EXPORT_SYMBOL(mse_close);

int mse_start_streaming(int index)
{
	struct mse_instance *instance;
	ktime_t ktime;
	struct timespec time;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	instance->first_f = false;

	/* get timestamp(nsec) */
	getnstimeofday(&time);
	instance->timestamp = time.tv_nsec;
	instance->timer_cnt = 0;

	/* start streaming */
	instance->network->start(instance->index_network);

	/* init paketizer */
	instance->packetizer->init(instance->index_packetizer);

	/* start timer */
	ktime = ktime_set(0, instance->timer_delay);
	hrtimer_start(&instance->timer, ktime, HRTIMER_MODE_REL);

	if (instance->inout == MSE_DIRECTION_OUTPUT)
		mse_packet_ctrl_receive_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);
	return 0;
}
EXPORT_SYMBOL(mse_start_streaming);

int mse_stop_streaming(int index)
{
	struct mse_instance *instance;
	int ret;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	instance->state = MSE_STATE_OPEN;

	/* timer stop */
	ret = hrtimer_try_to_cancel(&instance->timer);
	if (ret)
		pr_err("[%s] The timer was still in use...\n", __func__);

	/* start workqueue for stop */
	queue_work(instance->wq_packet, &instance->wk_stop);

	return 0;
}
EXPORT_SYMBOL(mse_stop_streaming);

int mse_start_transmission(int index,
			   void *buffer,
			   size_t buffer_size,
			   int (*mse_completion)(int index, int size))
{
	struct mse_instance *instance;
	int ret;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d buffer=%p size=%zu\n",
		 __func__, index, buffer, buffer_size);

	instance = &mse->instance_table[index];

	instance->state = MSE_STATE_EXECUTE;
	instance->media_buffer = (unsigned char *)buffer;
	instance->media_buffer_size = buffer_size;
	instance->mse_completion = mse_completion;
	instance->work_length = 0;

	ret = instance->network->set_option(instance->index_network);
	if (ret)
		pr_err("[%s] failed set_option() ret=%d\n", __func__, ret);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		/* start workqueue for packetize */
		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		memset(buffer, 0, buffer_size);
		/* start workqueue for streaming */
		queue_work(instance->wq_stream, &instance->wk_stream);
	}
	instance->timer_cnt++;
	return 0;
}
EXPORT_SYMBOL(mse_start_transmission);

int mse_get_private_data(int index, void **private_data)
{
	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	*private_data = mse->media_table[index].private_data;
	return 0;
}
EXPORT_SYMBOL(mse_get_private_data);

enum MSE_DIRECTION mse_get_inout(int index)
{
	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	return mse->media_table[index].inout;
}
EXPORT_SYMBOL(mse_get_inout);

/*
 * initialize MSE API
 */
static int mse_probe(void)
{
	int i, err;

	/* allocate device data */
	mse = kzalloc(sizeof(*mse), GFP_KERNEL);
	if (!mse)
		return -ENOMEM;

	/* register platform device */
	mse->pdev = platform_device_register_simple("mse", -1, NULL, 0);
	if (IS_ERR(mse->pdev)) {
		pr_err("[%s] Failed to register platform device. ret=%p\n",
		       __func__, mse->pdev);
		return -EINVAL;
	}

	/* W/A for cannot using DMA APIs */
	of_dma_configure(&mse->pdev->dev, NULL);

	/* initialize packetizer */
	err = mse_packetizer_init();
	if (err)
		return err;

	/* init table */
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		mse->instance_table[i].index_media = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].index_network = MSE_INDEX_UNDEFINED;
	}
	for (i = 0; i < ARRAY_SIZE(mse->media_table); i++) {
		mutex_init(&mse->media_table[i].lock);
		mse->media_table[i].index_sysfs = MSE_INDEX_UNDEFINED;
	}

	pr_debug("[%s] success\n", __func__);
	return 0;
}

/*
 * cleanup MSE API
 */
static int mse_remove(void)
{
	/* release packetizer */
	mse_packetizer_exit();
	/* unregister platform device */
	platform_device_unregister(mse->pdev);
	/* release device data */
	kfree(mse);
	pr_debug("[%s] success\n", __func__);
	return 0;
}

static int __init mse_module_init(void)
{
	pr_debug("[%s]\n", __func__);
	return mse_probe();
}

static void __exit mse_module_exit(void)
{
	pr_debug("[%s]\n", __func__);
	mse_remove();
}

module_init(mse_module_init);
module_exit(mse_module_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
