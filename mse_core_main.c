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
#include <linux/ptp_clock.h>
#include "avtp.h"
#include "mse_core.h"
#include "mse_packet_ctrl.h"
#include "mse_sysfs.h"
#include "mse_packetizer.h"
#include "mse_ptp.h"

#define NSEC_SCALE              (1000000000ul)
#define BUF_SIZE                (32)

#define NANO_SCALE              (1000000000ul)

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
/** @brief MCH table max */
#define MSE_MCH_MAX             (10)

#define MSE_DMA_MAX_PACKET         (128)
#define MSE_DMA_MAX_PACKET_SIZE    (1526)
#define MSE_DMA_MAX_RECEIVE_PACKET (100)
#define MSE_DMA_MIN_RECEIVE_PACKET (60)

#define MSE_VIDEO_START_CODE_LEN   (4)

#define q_next(pos, max)        (((pos) + 1) % max)

#define PTP_TIMESTAMPS_MAX   (512)
#define PTP_DELAY            (20 * 1000000)  /* 1/300 sec * 6 = 20ms */

#define CRF_TIMESTAMPS_MAX   (512)
#define CRF_DELAY            (20 * 1000000)  /* 20ms */
#define CRF_PTP_TIMESTAMPS   (1)     /* timestamps per CRF packet using ptp */
#define CRF_AUDIO_TIMESTAMPS (6)     /* audio timestamps per CRF packet */

#define AVTP_TIMESTAMPS_MAX  (512)

#define MSE_DECODE_BUFFER_NUM (8)
#define MSE_DECODE_BUFFER_NUM_START_MIN (2)
#define MSE_DECODE_BUFFER_NUM_START_MAX (6)
#define MAX_DECODE_SIZE       (8192) /* ALSA Period byte size */

/** @brief mse state */
enum MSE_STATE {
	/** @brief state of close */
	MSE_STATE_CLOSE,
	/** @brief state of ready */
	MSE_STATE_OPEN,
	/** @brief state of execute */
	MSE_STATE_EXECUTE,
};

struct timestamp_queue {
	int head;
	int tail;
	unsigned long std_times[PTP_TIMESTAMPS_MAX];
	struct ptp_clock_time times[PTP_TIMESTAMPS_MAX];
};

struct crf_queue {
	int head;
	int tail;
	unsigned long std_times[CRF_TIMESTAMPS_MAX];
	u64 times[CRF_TIMESTAMPS_MAX];
};

struct avtp_queue {
	int head;
	int tail;
	unsigned long std_times[AVTP_TIMESTAMPS_MAX];
	unsigned int times[AVTP_TIMESTAMPS_MAX];
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
	/** @brief timestamp queue */
	struct work_struct wk_timestamp;
	/** @brief crf send queue */
	struct work_struct wk_crf_send;
	/** @brief crf receive queue */
	struct work_struct wk_crf_receive;

	/** @brief stream workqueue */
	struct workqueue_struct *wq_stream;
	/** @brief packet workqueue */
	struct workqueue_struct *wq_packet;
	/** @brief timestamp workqueue */
	struct workqueue_struct *wq_tstamp;
	/** @brief crf packet workqueue */
	struct workqueue_struct *wq_crf_packet;

	/** @brief timestamp processed flag */
	bool is_tstamp_processed;
	/** @brief crf processed flag */
	bool is_crf_processed;

	/** @brief timer handler */
	struct hrtimer timer;
	int timer_delay;
	int timer_cnt;

	/** @brief timestamp timer handler */
	struct hrtimer tstamp_timer;
	int tstamp_timer_delay;
	int tstamp_timer_cnt;

	/** @brief crf timer handler */
	struct hrtimer crf_timer;
	int crf_timer_delay;
	int crf_timer_cnt;
	/* @brief crf packetizer index */
	int crf_index;
	struct mse_audio_info crf_audio_info;

	int ptp_dev_id;
	int mch_dev_id;
	int mch_index;
	int mch_revovery_value;
	bool f_match_ptp_clock;
	unsigned long mch_std_time;
	unsigned long std_time_counter;
	unsigned long add_std_time;
	unsigned long add_crf_std_time;
	unsigned long std_time_avtp;
	unsigned long std_time_crf;
	unsigned long start_present_time;

	/* @brief timestamp ptp|capture */
	struct timestamp_queue tstamp_que;
	struct crf_queue crf_que;
	struct avtp_queue avtp_que;

	/** @brief timestamp(nsec) */
	unsigned int timestamp;

	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;
	/** @brief media buffer */
	unsigned char *media_buffer;
	/** @brief buffer length for packetize */
	size_t media_buffer_size;
	/** @brief AVTP timestampes */
	unsigned int avtp_timestamps[1024];
	int avtp_timestamps_size;
	/** @brief complete function pointer */
	int (*mse_completion)(int index, int size);
	/** @brief work length for media buffer */
	size_t work_length;
	/** @brief streaming flag */
	bool f_streaming;
	/** @brief stopping streaming flag */
	bool f_stopping;
	/** @brief continue streaming flag */
	bool f_continue;
	bool f_depacketizing;
	bool f_completion;
	bool f_trans_start;

	/** @brief network configuration */
	struct mse_network_config net_config;
	struct mse_network_config crf_net_config;

	/** @brief media specific configuration */
	union mse_media_config {
		struct mse_audio_config audio;
		struct mse_video_config video;
	} media_config;
	struct mse_audio_info audio_info;

	/** @brief MCH & CRF Settings **/
	int ptp_clock;
	int ptp_clock_device;
	int ptp_clock_ch;
	int ptp_capture_freq;
	int media_clock_recovery;
	int media_clock_type;
	int media_capture_freq;
	int send_clock;
	int max_transit_time;
	int talker_delay_time;
	int listener_delay_time;
	int remain;
	char network_device_name_tx[MSE_NAME_LEN_MAX];
	char network_device_name_rx[MSE_NAME_LEN_MAX];

	bool f_present;

	/** @brief packet buffer */
	int crf_index_network;
	struct mse_packet_ctrl *crf_packet_buffer;
	bool f_crf_sending;

	/** @brief media clcok recovery work */
	struct ptp_clock_time timestamps[PTP_TIMESTAMPS_MAX];
	unsigned int master_timestamps[AVTP_TIMESTAMPS_MAX];
	unsigned int device_timestamps[AVTP_TIMESTAMPS_MAX];

	/** @brief video buffer  */
	bool f_first_vframe;
	bool use_temp_video_buffer_mjpeg;
	bool f_temp_video_buffer_rewind;
	int eoi_pos;
	int temp_vw;
	unsigned char temp_video_buffer[128 * 1024];
	/** @brief audio buffer  */
	int temp_w;
	int temp_r;
	unsigned char temp_buffer[MSE_DECODE_BUFFER_NUM][MAX_DECODE_SIZE];
	unsigned char guard_buffer[8192];
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
	struct mch_ops *mch_table[MSE_MCH_MAX];
};

/* MSE device data */
static struct mse_device *mse;

/*
 * module parameters
 */
static int debug;
module_param(debug, int, 0660);

/*
 * global parameters
 */
static struct mse_sysfs_config mse_sysfs_config_audio[] = {
	{
		.name = MSE_SYSFS_NAME_STR_TYPE,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"both", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_KIND,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"audio", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_DEVICE,
		.type = MSE_TYPE_STR,
		.str_value = "",
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
		.name = MSE_SYSFS_NAME_STR_BYTES_PER_FRAME,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CLOCK,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"ptp", "capture", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CLOCK_DEVICE,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CAPTURE_CH,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CAPTURE_FREQ,
		.type = MSE_TYPE_INT,
		.int_value = 300,
	},
	{
		.name = MSE_SYSFS_NAME_STR_MEDIA_CLOCK_RECOVERY,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_MEDIA_CLOCK_TYPE,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"avtp", "crf", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_SEND_CLOCK,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_SEND_CLOCK_DST_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "91e0f0000e80",
	},
	{
		.name = MSE_SYSFS_NAME_STR_SEND_CLOCK_SRC_MAC,
		.type = MSE_TYPE_STR,
		.str_value = "769050000000",
	},
	{
		.name = MSE_SYSFS_NAME_STR_SEND_CLOCK_UNIQUE_ID,
		.type = MSE_TYPE_INT,
		.int_value = 2,
	},
	{
		.name = MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME,
		.type = MSE_TYPE_INT,
		.int_value = 2000000,
	},
	{
		.name = MSE_SYSFS_NAME_STR_TALKER_DELAY_TIME,
		.type = MSE_TYPE_INT,
		.int_value = 2000000,
	},
	{
		.name = MSE_SYSFS_NAME_STR_LISTENER_DELAY_TIME,
		.type = MSE_TYPE_INT,
		.int_value = 2000000,
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
		.enum_list = {"talker", "listener", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_KIND,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"video", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_DEVICE,
		.type = MSE_TYPE_STR,
		.str_value = "",
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
		.str_value = MSE_PACKETIZER_NAME_STR_CVF_H264,
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
		.name = MSE_SYSFS_NAME_STR_BYTES_PER_FRAME,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CLOCK,
		.type = MSE_TYPE_ENUM,
		.int_value = 0,
		.enum_list = {"ptp", "capture", NULL},
	},
	{
		.name = MSE_SYSFS_NAME_STR_PTP_CLOCK_DEVICE,
		.type = MSE_TYPE_INT,
		.int_value = 0,
	},
	{
		.name = NULL, /* end of table */
	}
};

static int sysfs_name_cmp(char *defname, char *sysfsname)
{
	int ret = 0;

	if (!defname || (*defname == '\0'))
		return 1;
	if (!sysfsname)
		return 1;

	while (*defname) {
		if ((*sysfsname == '\0') || (*sysfsname == '\n')) {
			ret = 1;
			break;
		}
		if (*defname != *sysfsname) {
			ret = 1;
			break;
		}
		defname++;
		sysfsname++;
	}

	if ((*sysfsname != '\0') && (*sysfsname != '\n'))
		ret = 1;

	return ret;
}

static int mse_create_config_device(int index_media,
				    struct mse_adapter *adapter,
				    char *device_name)
{
	int ret, type;

	pr_debug("[%s]\n", __func__);

	switch (MSE_TYPE_KING_GET(adapter->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
		adapter->sysfs_config = kzalloc(sizeof(mse_sysfs_config_audio),
						GFP_KERNEL);
		memcpy(adapter->sysfs_config, mse_sysfs_config_audio,
		       sizeof(mse_sysfs_config_audio));

		type = 0; /* both */

		break;
	case MSE_TYPE_ADAPTER_VIDEO:
		adapter->sysfs_config = kzalloc(sizeof(mse_sysfs_config_video),
						GFP_KERNEL);
		memcpy(adapter->sysfs_config, mse_sysfs_config_video,
		       sizeof(mse_sysfs_config_video));

		switch (adapter->inout) {
		case MSE_DIRECTION_INPUT:
			type = 0; /* talker */
			break;
		case MSE_DIRECTION_OUTPUT:
			type = 1; /* listener */
			break;
		default:
			pr_err("[%s] undefined type=%d\n",
			       __func__, adapter->inout);
			return -EPERM;
		}

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

	/* device name */
	ret = mse_sysfs_set_config_str(
			index_media,
			MSE_SYSFS_NAME_STR_DEVICE,
			device_name,
			MSE_NAME_LEN_MAX);
	if (ret < 0) {
		pr_err("[%s] failed setting for sysfs ret=%d type=%s\n",
		       __func__, ret, MSE_SYSFS_NAME_STR_DEVICE);
		return ret;
	}

	/* type */
	ret = mse_sysfs_set_config_int(
			index_media,
			MSE_SYSFS_NAME_STR_TYPE,
			type);
	if (ret < 0) {
		pr_err("[%s] failed setting for sysfs ret=%d type=%s\n",
		       __func__, ret, MSE_SYSFS_NAME_STR_TYPE);
		return ret;
	}

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
	struct mse_network_config *crf_network = &instance->crf_net_config;
	struct mse_video_config *video = &instance->media_config.video;
	struct mse_audio_config *audio = &instance->media_config.audio;

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

	case MSE_TYPE_ADAPTER_AUDIO:
		err = mse_sysfs_get_config_int(
					index,
					MSE_SYSFS_NAME_STR_BYTES_PER_FRAME,
					&audio->bytes_per_frame);
		if (err < 0) {
			pr_err("[%s] undefined payload size\n", __func__);
			ret = -EPERM;
		}

		/* MCH & CRF Setting */
		mse_sysfs_get_config_int(index,
					 MSE_SYSFS_NAME_STR_PTP_CLOCK,
					 &instance->ptp_clock);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_PTP_CLOCK_DEVICE,
				&instance->ptp_clock_device);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_PTP_CAPTURE_CH,
				&instance->ptp_clock_ch);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_PTP_CAPTURE_FREQ,
				&instance->ptp_capture_freq);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_MEDIA_CLOCK_RECOVERY,
				&instance->media_clock_recovery);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_MEDIA_CLOCK_TYPE,
				&instance->media_clock_type);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ,
				&instance->media_capture_freq);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_SEND_CLOCK,
				&instance->send_clock);

		mse_sysfs_get_config_str(
				index,
				MSE_SYSFS_NAME_STR_SEND_CLOCK_DST_MAC,
				mac,
				sizeof(mac));

		err = change_mac_addr(crf_network->dest_addr, mac);
		mse_sysfs_get_config_str(
				index,
				MSE_SYSFS_NAME_STR_SEND_CLOCK_SRC_MAC,
				mac,
				sizeof(mac));

		err = change_mac_addr(crf_network->source_addr, mac);
		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_SEND_CLOCK_UNIQUE_ID,
				&instance->crf_net_config.uniqueid);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME,
				&instance->max_transit_time);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_TALKER_DELAY_TIME,
				&instance->talker_delay_time);

		mse_sysfs_get_config_int(
				index,
				MSE_SYSFS_NAME_STR_LISTENER_DELAY_TIME,
				&instance->listener_delay_time);

		crf_network->priority = network->priority;
		crf_network->vlanid = network->vlanid;

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
	int err = 0;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_stream);
	if (instance->f_stopping)
		return;

	if (instance->inout == MSE_DIRECTION_INPUT)
		/* request send packet */
		err = mse_packet_ctrl_send_packet(
			instance->index_network,
			instance->packet_buffer,
			instance->network);
	else
		/* request receive packet */
		err = mse_packet_ctrl_receive_packet(
			instance->index_network,
			MSE_DMA_MAX_RECEIVE_PACKET,
			instance->packet_buffer,
			instance->network);

	instance->f_streaming = false;
}

/* calculate timestamp from std_time */
static int tstamps_calc_tstamp(struct timestamp_queue *que,
			       unsigned long std_time,
			       unsigned long *clock_time)
{
	int p, p1;
	u64 std2, std1,  t2, t1, t;

	if (que->head == que->tail) {
		pr_debug("[%s] error\n", __func__);
		return -1;
	}

	p1 = que->head;
	for (p = que->head;
	     q_next(p, PTP_TIMESTAMPS_MAX) != que->tail;
	     p1 = p, p = q_next(p, PTP_TIMESTAMPS_MAX)) {
		if (que->std_times[p] > std_time)
			break;
	}

	if (p == que->head) {
		pr_debug("[%s] error %lu range %lu - %lu\n", __func__,
			 std_time, que->std_times[que->head],
			 que->std_times[(que->tail - 1) % PTP_TIMESTAMPS_MAX]);
		return -1;
	}

	std1 = que->std_times[p1];
	std2 = que->std_times[p];
	t1 = (u64)que->times[p1].sec * NSEC_SCALE + que->times[p1].nsec;
	t2 = (u64)que->times[p].sec * NSEC_SCALE + que->times[p].nsec;

	t = (t2 - t1) * (std_time - std1);
	do_div(t, std2 - std1);
	*clock_time = t + t1;

	return 0;
}

static int tstamps_search_tstamp(
	struct timestamp_queue *que,
	unsigned long *std_time,
	unsigned int avtp_time)
{
	int p;
	unsigned int t, t_d;

	if (que->head == que->tail) {
		pr_debug("[%s] error no data\n", __func__);
		return -1;
	}
	for (p = que->head; p != que->tail;
	     p = q_next(p, PTP_TIMESTAMPS_MAX)) {
		t = (u64)que->times[p].sec * NSEC_SCALE + que->times[p].nsec;
		t_d = avtp_time - t;
		if (t_d < UINT_MAX / 2)
			break;
	}

	if (p == que->tail || p == que->head)
		return -1;
	*std_time = que->std_times[p];
	pr_debug("[%s] found %lu t= %u avtp= %u\n",
	       __func__,  *std_time, t, avtp_time);
	return 0;
}

static int tstamps_enq_tstamps(struct timestamp_queue *que,
			       unsigned long std_time,
			       struct ptp_clock_time *clock_time)
{
	pr_debug("[%s] START head=%d, tail=%d\n",
		 __func__, que->head, que->tail);

	que->std_times[que->tail] = std_time;
	que->times[que->tail] = *clock_time;
	que->tail = q_next(que->tail, PTP_TIMESTAMPS_MAX);

	/* if tail equal head, push out head */
	if (que->tail == que->head)
		que->head = q_next(que->head, PTP_TIMESTAMPS_MAX);

	return 0;
}

static int tstamps_deq_tstamps(struct timestamp_queue *que,
			       unsigned long *std_time,
			       struct ptp_clock_time *clock_time)
{
	if (que->head == que->tail)
		return -1;

	*std_time = que->std_times[que->head];
	*clock_time = que->times[que->head];
	que->head = q_next(que->head, PTP_TIMESTAMPS_MAX);

	return 0;
}

static int tstamps_get_tstamps_size(struct timestamp_queue *que)
{
	if (que->tail >= que->head)
		return que->tail - que->head;
	else
		return (que->tail + PTP_TIMESTAMPS_MAX) - que->head;
}

static int tstamps_clear_tstamps(struct timestamp_queue *que)
{
	que->tail = 0;
	que->head = 0;
	return 0;
}

static int tstamps_enq_avtp(struct avtp_queue *que, unsigned long std_time,
			    unsigned int clock_time)
{
	pr_debug("[%s] START head=%d, tail=%d\n",
		 __func__, que->head, que->tail);

	que->std_times[que->tail] = std_time;
	que->times[que->tail] = clock_time;
	que->tail = q_next(que->tail, AVTP_TIMESTAMPS_MAX);

	/* if tail equal head, push out head */
	if (que->tail == que->head)
		que->head = q_next(que->head, AVTP_TIMESTAMPS_MAX);

	return 0;
}

static int tstamps_get_avtp_size(struct avtp_queue *que)
{
	if (que->tail >= que->head)
		return que->tail - que->head;
	else
		return (que->tail + PTP_TIMESTAMPS_MAX) - que->head;
}

static int tstamps_deq_avtp(struct avtp_queue *que,
			    unsigned long *std_time,
			    unsigned int *clock_time)
{
	if (que->head == que->tail)
		return -1;

	*std_time = que->std_times[que->head];
	*clock_time = que->times[que->head];
	que->head = q_next(que->head, AVTP_TIMESTAMPS_MAX);

	return 0;
}

static int tstamps_enq_crf(struct crf_queue *que,
			   unsigned long *std_times,
			   u64 *clock_time)
{
	pr_debug("[%s] START head=%d, tail=%d\n",
		 __func__, que->head, que->tail);

	que->std_times[que->tail] = *std_times;
	que->times[que->tail] = *clock_time;
	que->tail = q_next(que->tail, CRF_TIMESTAMPS_MAX);

	/* if tail equal head, push out head */
	if (que->tail == que->head)
		que->head = q_next(que->head, CRF_TIMESTAMPS_MAX);

	return 0;
}

static int tstamps_get_crf_size(struct crf_queue *que)
{
	if (que->tail >= que->head)
		return que->tail - que->head;
	else
		return (que->tail + CRF_TIMESTAMPS_MAX) - que->head;
}

static int tstamps_deq_crf(struct crf_queue *que,
			   unsigned long *std_times,
			   u64 *clock_time)
{
	if (que->head == que->tail)
		return -1;

	*std_times  = que->std_times[que->head];
	*clock_time = que->times[que->head];
	que->head = q_next(que->head, CRF_TIMESTAMPS_MAX);

	return 0;
}

static int media_clock_recovery(struct mse_instance *instance)
{
	struct mch_ops *m_ops;
	int ret, count, out, i;
	unsigned int avtp_time;
	unsigned long device_time;
	unsigned long std_time;
	u64 crf_time;
	unsigned int d_t;

	ret = 0;
	out = 0;
	d_t = 0;

	if (instance->media_clock_type == 0) {
		/* avtp */
		d_t = instance->audio_info.frame_interval_time;
		if (!instance->f_present)
			return -1;
		count = tstamps_get_avtp_size(&instance->avtp_que);
		for (i = 0; i < count; i++) {
			device_time = 0;
			ret = tstamps_deq_avtp(&instance->avtp_que, &std_time,
					       &avtp_time);
			if (ret < 0)
				continue;

			if (!instance->f_match_ptp_clock) {
				ret = tstamps_search_tstamp(
					&instance->tstamp_que,
					&instance->mch_std_time,
					avtp_time);
				if (ret < 0)
					continue;
				instance->f_match_ptp_clock = true;
			} else {
				instance->mch_std_time += d_t;
				pr_debug("[%s] mch std_time %lu\n",
					 __func__, instance->mch_std_time);
			}

			ret = tstamps_calc_tstamp(
				&instance->tstamp_que,
				instance->mch_std_time,
				&device_time);
			if (ret < 0)
				continue;

			instance->master_timestamps[out] = avtp_time;
			instance->device_timestamps[out] = device_time;
			out++;

			if (out >= ARRAY_SIZE(instance->master_timestamps))
				break;
		}
	} else {
		/* crf */
		d_t = instance->crf_audio_info.frame_interval_time;
		count = tstamps_get_crf_size(&instance->crf_que);
		for (i = 0; i < count; i++) {
			ret = tstamps_deq_crf(&instance->crf_que, &std_time,
					      &crf_time);
			if (ret < 0)
				continue;

			if (!instance->f_match_ptp_clock) {
				ret = tstamps_search_tstamp(
					&instance->tstamp_que,
					&instance->mch_std_time,
					crf_time);
				if (ret < 0) {
					pr_err("[%s] error\n",
					       __func__);
					continue;
				}
				instance->f_match_ptp_clock = true;
			} else {
				instance->mch_std_time += d_t;
			}

			ret = tstamps_calc_tstamp(&instance->tstamp_que,
						  instance->mch_std_time,
						  &device_time);

			if (ret < 0)
				continue;

			instance->master_timestamps[out] =
						(unsigned int)crf_time;
			instance->device_timestamps[out] = device_time;
			out++;

			if (out >= ARRAY_SIZE(instance->master_timestamps))
				break;
		}
	}

	if (out <= 0) {
		pr_debug("[%s] could not get master timestamps\n", __func__);
		return ret;
	}

	/* not mch ops registered */
	if (instance->mch_index < 0) {
		pr_err("[%s] mch is not initialized.\n", __func__);
		return -1;
	}

	/* get mch ops */
	m_ops = mse->mch_table[instance->mch_index];
	m_ops->send_timestamps(instance->mch_dev_id,
			d_t,
			out,
			instance->master_timestamps,
			out,
			instance->device_timestamps);

	if (instance->media_capture_freq && instance->ptp_clock) {
		u64 value = (u64)NSEC_SCALE * NANO_SCALE;
		int div;

		m_ops->get_recovery_value(instance->mch_dev_id,
					  &instance->mch_revovery_value);
		div = NANO_SCALE + instance->mch_revovery_value;
		do_div(value, instance->ptp_capture_freq);
		do_div(value, div);
		instance->add_std_time = value;
		pr_debug("[%s] recover %lu\n", __func__,
			 instance->add_std_time);
	}

	return 0;
}

static void mse_work_timestamp(struct work_struct *work)
{
	struct mse_instance *instance;
	int count;
	int ret = -1;
	int i;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_timestamp);
	if (instance->f_stopping)
		return;

	/* capture timestamps */
	if (instance->ptp_clock == 1) {
		/* get timestamps */
		ret = mse_ptp_get_timestamps(instance->ptp_dev_id,
					     instance->ptp_clock_ch,
					     &count,
					     instance->timestamps);
		if (ret) {
			pr_warn("[%s] could not get timestamps ret=%d\n",
				__func__, ret);
			return;
		}

		/* store timestamps */
		for (i = 0; i < count; i++) {
			tstamps_enq_tstamps(&instance->tstamp_que,
					    instance->std_time_counter,
					    &instance->timestamps[i]);
			instance->std_time_counter += instance->add_std_time;
		}
	}

	if (instance->media_clock_recovery == 1) {
		/* mch */
		media_clock_recovery(instance);
	}

	instance->is_tstamp_processed = true;
}

static int get_timestamps(struct timestamp_queue *que,
			  int count,
			  struct ptp_clock_time timestamps[])
{
	int i;
	struct ptp_clock_time clock_time;
	unsigned long std_time;

	if (tstamps_get_tstamps_size(que) < count)
		return -1;

	pr_debug("[%s] total=%d\n", __func__, count);

	for (i = 0; i < count; i++) {
		tstamps_deq_tstamps(que, &std_time, &clock_time);
		timestamps[i] = clock_time;
	}

	return 0;
}

static int tstamps_store_ptp_timestamp(struct mse_instance *instance)
{
	struct ptp_clock_time now;

	/* get now time form ptp and save */
	mse_ptp_get_time(instance->ptp_dev_id, &now);
	tstamps_enq_tstamps(&instance->tstamp_que,
			    instance->std_time_counter, &now);
	instance->std_time_counter += instance->add_std_time;

	return 0;
}

static int create_avtp_timestamps(struct mse_instance *instance)
{
	int i;
	int num_t, size;
	unsigned int d_t;
	struct mse_audio_config *audio = &instance->media_config.audio;
	unsigned int offset;

	instance->packetizer->get_audio_info(
		instance->index_packetizer,
		&instance->audio_info);

	if (instance->ptp_clock == 0) {
		offset = instance->max_transit_time;
	} else {
		offset = instance->max_transit_time +
			instance->max_transit_time;
	}

	num_t = audio->period_size / instance->audio_info.sample_per_packet;

	instance->remain +=
		audio->period_size -
		num_t * instance->audio_info.sample_per_packet;

	if (instance->remain >= instance->audio_info.sample_per_packet) {
		instance->remain -= instance->audio_info.sample_per_packet;
		num_t++;
	}
	pr_debug("[%s] create %d\n", __func__, num_t);
	d_t = instance->audio_info.frame_interval_time;
	size = tstamps_get_tstamps_size(&instance->tstamp_que);
	if (size < 2) {
		struct ptp_clock_time now;
		u64 t;

		/* no data (ptp_clock + std_time) */
		mse_ptp_get_time(instance->ptp_dev_id, &now);
		t = (u64)now.sec * NSEC_SCALE + now.nsec;
		for (i = 0; i < num_t; i++)
			instance->avtp_timestamps[i] = t + d_t * i + offset;

		instance->avtp_timestamps_size = num_t;
		instance->std_time_avtp += d_t * num_t;

		return 0;
	}
	/* get timestamps from private table */

	spin_lock(&mse->lock);

	for (i = 0; i < num_t; i++) {
		unsigned long avtp_timestamp = 0;

		tstamps_calc_tstamp(&instance->tstamp_que,
				    instance->std_time_avtp,
				    &avtp_timestamp);
		instance->avtp_timestamps[i] = avtp_timestamp + offset;
		instance->std_time_avtp += d_t;
	}
	instance->avtp_timestamps_size = num_t;

	spin_unlock(&mse->lock);

	return 0;
}

static int mse_initialize_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;
	struct mse_audio_config *audio = &instance->media_config.audio;
	struct mse_audio_config config;

	instance->crf_index = crf->open();

	crf->init(instance->crf_index);

	crf->set_network_config(instance->crf_index,
				&instance->crf_net_config);

	/* base_frequency */
	config.sample_rate     = audio->sample_rate;
	/* timestamp_interval */
	if (!instance->ptp_clock) {
		config.bytes_per_frame = audio->period_size;
	} else {
		config.bytes_per_frame = audio->sample_rate /
			instance->ptp_capture_freq;
	}
	crf->set_audio_config(instance->crf_index, &config);

	return 0;
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
		ret = mse_packet_ctrl_make_packet(
			instance->index_packetizer,
			instance->media_buffer,
			instance->media_buffer_size,
			instance->ptp_clock,
			instance->avtp_timestamps_size,
			instance->avtp_timestamps,
			instance->packet_buffer,
			instance->packetizer);
		if (ret < 0) {
			pr_err("[%s] erorr=%d\n", __func__, ret);
			return;
		}

		pr_debug("[%s] media_buffer=%p packetized=%d\n",
			 __func__, instance->media_buffer, ret);

		/* start workqueue for streaming */
		instance->f_streaming = true;
		queue_work(instance->wq_stream, &instance->wk_stream);
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
		/* get timestamp(nsec) */
		getnstimeofday(&time);
		instance->timestamp = time.tv_nsec;
		/* make AVTP packet */
		instance->work_length = 0;
		ret = mse_packet_ctrl_make_packet(
			instance->index_packetizer,
			instance->media_buffer + instance->work_length,
			instance->media_buffer_size - instance->work_length,
			-1,
			1,
			&instance->timestamp,
			instance->packet_buffer,
			instance->packetizer);

		if (ret < 0)
			return;

		pr_debug("[%s] media_buffer=%p packetized=%d\n",
			 __func__, instance->media_buffer, ret);
		instance->work_length += ret;

		if (instance->use_temp_video_buffer_mjpeg)
			instance->f_temp_video_buffer_rewind = true;

		/* start workqueue for streaming */
		queue_work(instance->wq_stream, &instance->wk_stream);
		break;

	default:
		pr_err("[%s] unknown type=0x%08x\n",
		       __func__, instance->media->type);
		break;
	}
}

static int tstamps_store_avtp_timestamp(struct mse_instance *instance,
					unsigned long std_time,
					unsigned int timestamp)
{
	/* save avtp timestamp */
	spin_lock(&mse->lock);
	tstamps_enq_avtp(&instance->avtp_que, std_time, timestamp);
	spin_unlock(&mse->lock);

	return 0;
}

static bool is_presentable(struct mse_instance *instance)
{
	struct ptp_clock_time now;
	unsigned int t = 0, t_d = 0;

	if (instance->f_present)
		return true;

	if (!instance->media_clock_recovery) {
		if (instance->temp_w < MSE_DECODE_BUFFER_NUM_START_MIN)
			return false;
	} else {
		mse_ptp_get_time(instance->ptp_dev_id, &now);
		t = (unsigned long)now.sec * NSEC_SCALE + now.nsec;
		t_d = instance->timestamp - t;
		if (t_d > UINT_MAX / 2 &&
		    instance->temp_w < MSE_DECODE_BUFFER_NUM_START_MAX)
			return false;
		pr_debug("[%s] start present avtp %u ptp %u q %d\n", __func__,
			 instance->timestamp, t, instance->temp_w);
	}
	instance->f_present = true;
	instance->start_present_time = instance->timestamp;
	pr_debug("[%s] start present listener avtp %u ptp %u q %d\n", __func__,
		 instance->timestamp, t, instance->temp_w);
	return true;
}

static void mse_work_depacketize(struct work_struct *work)
{
	struct mse_instance *instance;
	int timestamp = 0;
	int received, ret = -1;
	unsigned int timestamps[128];
	int t_stored, i;
	unsigned int d_t;
	struct mse_audio_config *audio;

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_depacketize);
	if (instance->f_stopping) {
		pr_err("[%s]  depaketize stopping\n", __func__);
		return;
	}

	received = mse_packet_ctrl_check_packet_remain(
						instance->packet_buffer);

	switch (MSE_TYPE_KING_GET(instance->media->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
		/* get AVTP packet payload */
		audio = &instance->media_config.audio;
		if (received > MSE_DMA_MIN_RECEIVE_PACKET ||
		    instance->first_f) {
			/* get AVTP packet payload */
			instance->work_length = 0;
			ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						instance->temp_buffer[instance->temp_w],
						instance->media_buffer_size,
						timestamps,
						ARRAY_SIZE(timestamps),
						&t_stored,
						instance->packet_buffer,
						instance->packetizer,
						&instance->work_length);

			/* samples per packet */
			instance->packetizer->get_audio_info(
				instance->index_packetizer,
				&instance->audio_info);
			d_t = instance->audio_info.frame_interval_time;
			for (i = 0; i < t_stored; i++) {
				tstamps_store_avtp_timestamp(
					instance,
					instance->std_time_avtp,
					timestamps[i]);
				instance->std_time_avtp += d_t;
			}
			instance->first_f = true;
			if (instance->work_length >=
			    instance->media_buffer_size) {
				instance->temp_w = (instance->temp_w + 1) %
					MSE_DECODE_BUFFER_NUM;
				instance->work_length = 0;
			}
		}
		pr_debug("[%s] media_buffer=%p received=%d depacketized=%d\n",
			 __func__, instance->media_buffer, received, ret);
		if (instance->f_stopping)
			break;

		if (MSE_TYPE_KING_GET(instance->media->type) ==
		    MSE_TYPE_ADAPTER_AUDIO &&
		    instance->inout == MSE_DIRECTION_OUTPUT) {
			if (instance->temp_w != instance->temp_r &&
			    is_presentable(instance)) {
				memcpy(instance->media_buffer,
				       instance->temp_buffer[instance->temp_r],
				       instance->media_buffer_size);
				instance->temp_r = (instance->temp_r + 1) %
					MSE_DECODE_BUFFER_NUM;
			}
		}

		/* complete callback */
		instance->mse_completion(instance->index_media, 0);
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
		/* get AVTP packet payload */
		ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						instance->media_buffer,
						instance->media_buffer_size,
						&timestamp,
						1,
						&t_stored,
						instance->packet_buffer,
						instance->packetizer,
						&instance->work_length);

		pr_debug("[%s] media_buffer=%p received=%d depacketized=%d\n",
			 __func__, instance->media_buffer, received, ret);

		if (instance->f_stopping)
			break;

		/* complete callback */
		if (ret >= 0) {
			instance->mse_completion(instance->index_media, ret);
			instance->work_length = 0;
		} else {
			instance->timer_cnt++; /* ad-hoc */
			queue_work(instance->wq_stream, &instance->wk_stream);
		}
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
	if (instance->f_stopping)
		return;

	/* complete callback anytime */
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
	if (instance->inout == MSE_DIRECTION_OUTPUT)
		flush_work(&instance->wk_depacketize);

	/* dequeue */
	flush_work(&instance->wk_stream);
}

static enum hrtimer_restart mse_timer_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	ktime_t ktime;

	instance = container_of(arg, struct mse_instance, timer);
	adapter = instance->media;

	if (instance->f_stopping) {
		pr_err("[%s] stopping ...\n", __func__);
		return HRTIMER_NORESTART;
	}

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

	mutex_unlock(&adapter->lock);

	if (instance->inout == MSE_DIRECTION_INPUT)
		/* start workqueue for completion */
		queue_work(instance->wq_packet, &instance->wk_callback);
	else
		/* start workqueue for depacketize */
		queue_work(instance->wq_packet, &instance->wk_depacketize);

	if (instance->ptp_clock == 0)
		tstamps_store_ptp_timestamp(instance);

	return HRTIMER_RESTART;
}

static void mse_work_crf_send(struct work_struct *work)
{
	struct mse_instance *instance;
	int err, tsize, size;
	struct ptp_clock_time timestamps[6];

	pr_debug("[%s]\n", __func__);

	instance = container_of(work, struct mse_instance, wk_crf_send);

	tsize = instance->ptp_clock == 0 ?
		CRF_PTP_TIMESTAMPS : CRF_AUDIO_TIMESTAMPS;
	size = tstamps_get_tstamps_size(&instance->tstamp_que);

	while (size >= tsize) {
		/* get Timestamps */
		pr_debug("[%s] size %d tsize %d\n", __func__, size, tsize);
		get_timestamps(&instance->tstamp_que, tsize, timestamps);

		/* create CRF packets */
		err = mse_packet_ctrl_make_packet_crf(
			instance->crf_index,
			timestamps,
			tsize,
			instance->crf_packet_buffer);

		/* send packets */
		err = mse_packet_ctrl_send_packet(
			instance->crf_index_network,
			instance->crf_packet_buffer,
			instance->network);

		size = tstamps_get_tstamps_size(&instance->tstamp_que);
	}

	instance->f_crf_sending = false;
}

static void mse_work_crf_receive(struct work_struct *work)
{
	struct mse_instance *instance;
	u8 streamid[AVTP_STREAMID_SIZE];
	int ret, err, count, i;
	u64 ptimes[6];
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;

	/* int timestamp = 0; */
	/* int received = 0, ret = -1; */

	instance = container_of(work, struct mse_instance, wk_crf_receive);

	pr_debug("[%s]\n", __func__);
	ret = instance->network->open(instance->network_device_name_rx);
	instance->crf_index_network = ret;
	if (ret < 0)
		pr_err("[%s] can not open %d\n", __func__, ret);

	/* setup network */
	mse_make_streamid(streamid,
			  instance->crf_net_config.source_addr,
			  instance->crf_net_config.uniqueid);
	instance->network->set_streamid(instance->crf_index_network,
					streamid);

	/* get packet memory */
	instance->crf_packet_buffer = mse_packet_ctrl_alloc(
		&mse->pdev->dev,
		MSE_DMA_MAX_PACKET * 2,
		MSE_DMA_MAX_PACKET_SIZE);

	/* prepare for receive */
	mse_packet_ctrl_receive_prepare_packet(
		instance->crf_index_network,
		instance->crf_packet_buffer,
		instance->network);

	while (!instance->f_stopping) {
		err = mse_packet_ctrl_receive_packet_crf(
			instance->crf_index_network,
			1,
			instance->crf_packet_buffer,
			instance->network);
		if (err < 0) {
			pr_err("[%s] receive error %d\n",
			       __func__, err);
			break;
		}

		count = mse_packet_ctrl_take_out_packet_crf(
			instance->crf_index,
			ptimes,
			ARRAY_SIZE(ptimes),
			instance->crf_packet_buffer);

		pr_debug("[%s] crf receive %d timestamp\n", __func__, count);

		crf->get_audio_info(
			instance->crf_index,
			&instance->crf_audio_info);

		for (i = 0; i < count; i++) {
			tstamps_enq_crf(&instance->crf_que,
					&instance->std_time_crf,
					&ptimes[i]);
			instance->std_time_crf +=
				instance->crf_audio_info.frame_interval_time;
		}
	}

	instance->network->release(instance->crf_index_network);
	mse_packet_ctrl_free(instance->crf_packet_buffer);
	instance->crf_packet_buffer = NULL;
}

static enum hrtimer_restart mse_crf_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	ktime_t ktime;

	instance = container_of(arg, struct mse_instance, crf_timer);

	if (instance->f_stopping) {
		pr_err("[%s] stopping ...\n", __func__);
		return HRTIMER_NORESTART;
	}

	/* start workqueue for send */
	if (!instance->f_crf_sending) {
		instance->f_crf_sending = true;
		queue_work(instance->wq_crf_packet, &instance->wk_crf_send);
	}

	/* timer update */
	ktime = ktime_set(0, instance->crf_timer_delay);
	hrtimer_forward(&instance->crf_timer,
			hrtimer_get_expires(&instance->crf_timer),
			ktime);

	if (!instance->is_crf_processed)
		return HRTIMER_RESTART;

	instance->is_crf_processed = false;

	return HRTIMER_RESTART;
}

static enum hrtimer_restart mse_timestamp_collect_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	ktime_t ktime;

	instance = container_of(arg, struct mse_instance, tstamp_timer);

	pr_debug("[%s]\n", __func__);
	/* FIX need mutex lock? */

	if (instance->f_stopping) {
		pr_err("[%s] stopping ...\n", __func__);
		return HRTIMER_NORESTART;
	}

	/* timer update */
	ktime = ktime_set(0, instance->tstamp_timer_delay);
	hrtimer_forward(&instance->tstamp_timer,
			hrtimer_get_expires(&instance->tstamp_timer),
			ktime);

	if (!instance->is_tstamp_processed)
		return HRTIMER_RESTART;

	/* FIX need mutex unlock? */

	queue_work(instance->wq_tstamp, &instance->wk_timestamp);
	instance->is_tstamp_processed = false;

	return HRTIMER_RESTART;
}

static bool is_audio_adapter(struct mse_adapter *adapter)
{
	bool is_audio = false;

	switch (MSE_TYPE_KING_GET(adapter->type)) {
	case MSE_TYPE_ADAPTER_AUDIO:
	case MSE_TYPE_ADAPTER_AUDIO_PCM:
		is_audio = true;
		break;
	default:
		break;
	}

	return is_audio;
}

/* External function */
int mse_register_adapter_media(enum MSE_TYPE type,
			       enum MSE_DIRECTION inout,
			       char *name,
			       void *data,
			       char *device_name)
{
	int i, err;

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
			mse->media_table[i].inout = inout;
			mse->media_table[i].private_data = data;
			strncpy(mse->media_table[i].name, name,
				MSE_NAME_LEN_MAX);
			/* create control device */
			err = mse_create_config_device(i,
						       &mse->media_table[i],
						       device_name);
			if (err < 0) {
				spin_unlock(&mse->lock);
				return -EPERM;
			}

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

int mse_unregister_adapter_media(int index_media)
{
	int i;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n",
		       __func__, index_media);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index_media);
	spin_lock(&mse->lock);

	if (!mse->media_table[index_media].used_f) {
		pr_err("[%s] %d was unregistered\n", __func__, index_media);
		spin_unlock(&mse->lock);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].index_media == index_media) {
			pr_err("[%s] module is in use. instance=%d\n",
			       __func__, i);
			spin_unlock(&mse->lock);
			return -EPERM;
		}
	}

	/* delete control device */
	mse_delete_config_device(&mse->media_table[index_media]);

	/* table delete */
	memset(&mse->media_table[index_media], 0, sizeof(struct mse_adapter));
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

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
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
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;
	struct mse_instance *instance;
	u64 timer;
	struct mse_adapter *adapter;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	if (!config) {
		pr_err("[%s] invalid argument. config\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d data=%p\n", __func__, index, config);
	pr_info("[%s]\n  sample_rate=%d sample_format=%d channels=%d\n"
		"  period_size=%d bytes_per_sample=%d\n", __func__,
		config->sample_rate, config->sample_format, config->channels,
		config->period_size, config->bytes_per_sample);

	instance = &mse->instance_table[index];
	adapter = instance->media;

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

	if (is_audio_adapter(adapter)) {
		if (instance->ptp_clock == 1 ||
		    instance->media_clock_recovery == 1) {
			instance->tstamp_timer_delay = PTP_DELAY;
			pr_notice("[%s] tstamp_timer_delay=%d\n",
				  __func__, instance->tstamp_timer_delay);
		}

		instance->crf_timer_delay = CRF_DELAY;
		pr_notice("[%s] crf_timer_delay=%d\n",
			  __func__, instance->crf_timer_delay);

		/* send clock using CRF */
		if (instance->send_clock) {
			int ret;
			struct eavb_cbsparam cbs;

			ret = instance->network->open(
				instance->network_device_name_tx);
			if (ret < 0)
				return -EINVAL;

			instance->crf_index_network = ret;

			crf->calc_cbs(instance->crf_index, &cbs);
			instance->network->set_cbs_param(
				instance->crf_index_network,
				&cbs);

			/* get packet memory */
			instance->crf_packet_buffer = mse_packet_ctrl_alloc(
				&mse->pdev->dev,
				MSE_DMA_MAX_PACKET,
				MSE_DMA_MAX_PACKET_SIZE);
			if (!instance->crf_packet_buffer)
				return -EINVAL;
		}
	}

	/* set AVTP header info */
	instance->packetizer->set_network_config(
		instance->index_packetizer, &instance->net_config);
	/* init packet header */
	instance->packetizer->set_audio_config(
		instance->index_packetizer, config);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		struct eavb_cbsparam cbs;

		instance->packetizer->calc_cbs(instance->index_packetizer,
					       &cbs);
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

	/* use CRF packet */
	if (instance->send_clock || (instance->media_clock_recovery == 1 &&
				     instance->media_clock_type == 1))
		mse_initialize_crf_packetizer(instance);

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

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
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

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
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

	if (instance->packet_buffer) {
		mse_packet_ctrl_free(instance->packet_buffer);
		instance->packet_buffer = NULL;
	}

	if (instance->inout == MSE_DIRECTION_INPUT) {
		struct eavb_cbsparam cbs;

		instance->packetizer->calc_cbs(0, &cbs);
		instance->network->set_cbs_param(instance->index_network,
						 &cbs);
		pr_debug("[%s] bandwidth fraction = %08x\n",
			 __func__, cbs.bandwidthFraction);
		/* get packet memory */
		instance->packet_buffer = mse_packet_ctrl_alloc(
			&mse->pdev->dev,
			MSE_DMA_MAX_PACKET,
			MSE_DMA_MAX_PACKET_SIZE);
	} else {
		u8 streamid[AVTP_STREAMID_SIZE];

		mse_make_streamid(streamid,
				  instance->net_config.source_addr,
				  instance->net_config.uniqueid);
		instance->network->set_streamid(instance->index_network,
						streamid);
		/* get packet memory */
		instance->packet_buffer = mse_packet_ctrl_alloc(
			&mse->pdev->dev,
			MSE_DMA_MAX_PACKET * 2,
			MSE_DMA_MAX_PACKET_SIZE);
	}

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
	char eavbname[MSE_NAME_LEN_MAX];

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

		if (!sysfs_name_cmp(network->name, eavbname))
			break;
	}

	if (j >= ARRAY_SIZE(mse->network_table)) {
		pr_err("[%s] network adapter module is not loaded\n",
		       __func__);
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

	/* if mjpeg input */
	if (sysfs_name_cmp(MSE_PACKETIZER_NAME_STR_CVF_MJPEG, name) == 0 &&
	    inout == MSE_DIRECTION_INPUT) {
		pr_err("[%s] use mjpeg\n",  __func__);
		instance->use_temp_video_buffer_mjpeg = true;
		instance->f_first_vframe = true;
	}

	/* search packetizer name for configuration value */
	for (j = 0; j < ARRAY_SIZE(mse->packetizer_table); j++) {
		packetizer = mse->packetizer_table[j];
		if (!packetizer)
			continue;

		if (!sysfs_name_cmp(packetizer->name, name))
			break;
	}

	if (j >= ARRAY_SIZE(mse->packetizer_table)) {
		pr_err("[%s] packetizer not found\n", __func__);
		mutex_unlock(&adapter->lock);
		return -ENODEV;
	}
	pr_debug("[%s] packetizer index=%d name=%s\n",
		 __func__, j, packetizer->name);

	/* ptp open */
	ret = mse_ptp_open(&instance->ptp_dev_id);
	if (ret < 0) {
		pr_err("[%s] cannot mse_ptp_open()\n", __func__);
		mutex_unlock(&adapter->lock);
		return ret;
	}

	/* open network adapter */
	ret = mse_sysfs_get_config_str(
		adapter->index_sysfs,
		MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_RX,
		instance->network_device_name_rx,
		MSE_NAME_LEN_MAX);

	ret = mse_sysfs_get_config_str(
		adapter->index_sysfs,
		MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_TX,
		instance->network_device_name_tx,
		MSE_NAME_LEN_MAX);

	if (inout != MSE_DIRECTION_INPUT)
		ret = network->open(instance->network_device_name_rx);
	else
		ret = network->open(instance->network_device_name_tx);
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
	instance->packetizer = packetizer;
	instance->network = network;
	instance->index_media = index_media;

	/* init work queue */
	INIT_WORK(&instance->wk_packetize, mse_work_packetize);
	INIT_WORK(&instance->wk_depacketize, mse_work_depacketize);
	INIT_WORK(&instance->wk_callback, mse_work_callback);
	INIT_WORK(&instance->wk_stream, mse_work_stream);
	INIT_WORK(&instance->wk_stop, mse_work_stop);
	INIT_WORK(&instance->wk_crf_send, mse_work_crf_send);
	INIT_WORK(&instance->wk_crf_receive, mse_work_crf_receive);
	INIT_WORK(&instance->wk_timestamp, mse_work_timestamp);

	instance->wq_stream = create_singlethread_workqueue("mse_streamq");
	instance->wq_packet = create_singlethread_workqueue("mse_packetq");
	hrtimer_init(&instance->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	instance->timer_delay = MSE_DEFAULT_TIMER;
	instance->timer.function = &mse_timer_callback;

	/* for timestamp */
	instance->wq_tstamp = create_singlethread_workqueue("mse_tstampq");
	if (is_audio_adapter(adapter)) {
		tstamps_clear_tstamps(&instance->tstamp_que);
		hrtimer_init(&instance->tstamp_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->tstamp_timer_delay = PTP_DELAY;
		instance->tstamp_timer.function =
					&mse_timestamp_collect_callback;
		instance->is_tstamp_processed = true;

		/* for crf */
		instance->wq_crf_packet =
			create_singlethread_workqueue("mse_crfpacketq");
		hrtimer_init(&instance->crf_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->crf_timer_delay = CRF_DELAY;
		instance->crf_timer.function = &mse_crf_callback;
		instance->is_crf_processed = false;
	}

	mutex_unlock(&adapter->lock);

	ret = mse_get_default_config(adapter->index_sysfs, instance);
	if (ret < 0)
		pr_err("[%s] cannot get configurations\n", __func__);

	return i;
}
EXPORT_SYMBOL(mse_open);

static int close_mch(struct mch_ops *m_ops, struct mse_instance *instance)
{
	int ret;

	if (!m_ops)
		return -EINVAL;

	ret = m_ops->close(instance->mch_dev_id);
	if (ret < 0)
		pr_err("[%s]mch open error(%d).\n", __func__, ret);

	return ret;
}

static int open_mch(struct mch_ops *m_ops, struct mse_instance *instance)
{
	int ret;

	if (!m_ops)
		return -EINVAL;

	ret = m_ops->open(&instance->mch_dev_id);
	if (ret < 0) {
		pr_err("[%s]mch open error(%d).\n", __func__, ret);
		return ret;
	}

	return 0;
}

int mse_close(int index)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int ret;
	bool is_audio;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	adapter = instance->media;

	/* wait stop */
	flush_work(&instance->wk_stop);

	/* free packet memory */
	if (instance->packet_buffer)
		mse_packet_ctrl_free(instance->packet_buffer);

	/* destroy workqueue */
	destroy_workqueue(instance->wq_packet);
	destroy_workqueue(instance->wq_stream);

	destroy_workqueue(instance->wq_tstamp);
	is_audio = is_audio_adapter(adapter);
	if (is_audio)
		destroy_workqueue(instance->wq_crf_packet);

	/* release network adapter */
	instance->network->release(instance->index_network);

	/* release packetizer */
	instance->packetizer->release(instance->index_packetizer);

	ret = mse_ptp_close(instance->ptp_dev_id);
	if (ret < 0)
		pr_err("[%s] cannot mse_ptp_close()\n", __func__);

	/* set table */
	memset(instance, 0, sizeof(*instance));
	instance->state = MSE_STATE_CLOSE;
	instance->index_media = MSE_INDEX_UNDEFINED;
	instance->index_network = MSE_INDEX_UNDEFINED;
	instance->index_packetizer = MSE_INDEX_UNDEFINED;

	return 0;
}
EXPORT_SYMBOL(mse_close);

int mse_start_streaming(int index)
{
	struct mse_instance *instance;
	ktime_t ktime;
	ktime_t ktime_ts;
	ktime_t ktime_crf;
	struct timespec time;
	struct mse_adapter *adapter;
	struct mch_ops *m_ops;
	int ret = 0, i;
	int remainder = 0;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	instance->first_f = false;
	adapter = instance->media;
	instance->mch_index = -1;
	for (i = 0; i < MSE_MCH_MAX; i++) {
		m_ops = mse->mch_table[i];
		if (m_ops) {
			instance->mch_index = i;
			break;
		}
	}

	/* Check bytes per frame */
	if (instance->media_config.audio.bytes_per_frame) {
		remainder = instance->media_config.audio.bytes_per_frame %
				(instance->media_config.audio.channels *
					sizeof(unsigned short));
		if (remainder) {
			pr_err("[%s] bytes per frame error\n", __func__);
			return -EINVAL;
		}
	}

	/* get timestamp(nsec) */
	getnstimeofday(&time);
	instance->timestamp = time.tv_nsec;
	instance->timer_cnt = 0;

	/* prepare std clock time */
	if (instance->ptp_clock) {
		instance->add_std_time =
			NSEC_SCALE / instance->ptp_capture_freq;
		instance->add_crf_std_time =
			NSEC_SCALE / instance->ptp_capture_freq;
	} else {
		instance->add_std_time = instance->timer_delay;
		instance->add_crf_std_time = instance->timer_delay;
	}

	instance->std_time_counter = 0;
	instance->std_time_avtp = 0;
	instance->std_time_crf = 0;

	instance->f_present = false;
	instance->remain = 0;

	/* start streaming */
	instance->network->start(instance->index_network);

	/* init paketizer */
	instance->packetizer->init(instance->index_packetizer);

	/* start timer */
	ktime = ktime_set(0, instance->timer_delay);
	hrtimer_start(&instance->timer, ktime, HRTIMER_MODE_REL);

	if (is_audio_adapter(adapter)) {
		/* capture timestamps */
		if (instance->ptp_clock == 1) {
			int count, i;
			/* get timestamps */
			ret = mse_ptp_get_timestamps(instance->ptp_dev_id,
						     instance->ptp_clock_ch,
						     &count,
						     instance->timestamps);
			if (ret) {
				pr_debug("[%s] could not get timestamps ret=%d\n",
					 __func__, ret);
				count = 0;
			}

			/* store timestamps */
			for (i = 0; i < count; i++) {
				tstamps_enq_tstamps(&instance->tstamp_que,
						    instance->std_time_counter,
						    &instance->timestamps[i]);
				instance->std_time_counter +=
					instance->add_std_time;
			}
		}

		/* media_clock_recovery is enable */
		if (instance->media_clock_recovery == 1)
			ret = open_mch(m_ops, instance);

		/* ptp_clock is capture or media_clock_recovery is enable */
		if (instance->ptp_clock == 1 ||
		    instance->media_clock_recovery == 1) {
			ktime_ts = ktime_set(0, instance->tstamp_timer_delay);
			hrtimer_start(&instance->tstamp_timer,
				      ktime_ts, HRTIMER_MODE_REL);
		}

		/* send clock using CRF */
		if (instance->send_clock) {
			ktime_crf = ktime_set(0, instance->crf_timer_delay);
			hrtimer_start(&instance->crf_timer,
				      ktime_crf,
				      HRTIMER_MODE_REL);
		}

		/* receive clcok using CRF */
		if (instance->media_clock_recovery == 1 &&
		    instance->media_clock_type == 1) {
			queue_work(instance->wq_crf_packet,
				   &instance->wk_crf_receive);
		}
	}

	if (instance->inout == MSE_DIRECTION_OUTPUT)
		mse_packet_ctrl_receive_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);
	instance->f_streaming = false;
	instance->f_stopping = false;
	instance->work_length = 0;
	instance->temp_w = 0;
	instance->temp_r = 0;

	return 0;
}
EXPORT_SYMBOL(mse_start_streaming);

int mse_stop_streaming(int index)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int ret;
	bool is_audio;
	struct mch_ops *m_ops = NULL;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	instance = &mse->instance_table[index];
	instance->state = MSE_STATE_OPEN;
	instance->f_stopping = true;
	adapter = instance->media;
	if (instance->mch_index >= 0)
		m_ops = mse->mch_table[instance->mch_index];

	if (MSE_TYPE_KING_GET(adapter->type) == MSE_TYPE_ADAPTER_VIDEO &&
	    (instance->inout == MSE_DIRECTION_OUTPUT)) {
		ret = instance->network->cancel(instance->index_network);
		if (ret)
			pr_err("[%s] failed cancel() ret=%d\n", __func__, ret);
	}

	if (instance->media_clock_recovery == 1 &&
	    instance->media_clock_type == 1) {
		ret = instance->network->cancel(instance->crf_index_network);
	}

	/* timer stop */
	ret = hrtimer_try_to_cancel(&instance->timer);
	if (ret)
		pr_err("[%s] The timer was still in use...\n", __func__);

	/* timestamp timer, crf timer stop */
	is_audio = is_audio_adapter(adapter);
	if (is_audio) {
		ret = hrtimer_try_to_cancel(&instance->tstamp_timer);
		if (ret)
			pr_err("[%s] The tstamp_timer was still in use...\n",
			       __func__);

		ret = hrtimer_try_to_cancel(&instance->crf_timer);
		if (ret)
			pr_err("[%s] The crf_timer was still in use...\n",
			       __func__);

		if (instance->media_clock_recovery == 1 &&
		    close_mch(m_ops, instance))
			pr_err("[%s] close mch erro\n", __func__);
	}

	/* start workqueue for stop */
	queue_work(instance->wq_packet, &instance->wk_stop);
	if (is_audio) {
		queue_work(instance->wq_tstamp, &instance->wk_stop);
		queue_work(instance->wq_crf_packet, &instance->wk_stop);
	}

	return 0;
}
EXPORT_SYMBOL(mse_stop_streaming);

static bool check_mjpeg(struct mse_instance *instance)
{
	int i;

	if (instance->eoi_pos > 0)
		return false;

	for (i = 0; i < instance->temp_vw - 1; i++) {
		if (instance->temp_video_buffer[i] == 0xFF &&
		    instance->temp_video_buffer[i + 1] == 0xD9) {
			instance->eoi_pos = i;
			return true;
		}
	}

	return false;
}

int mse_start_transmission(int index,
			   void *buffer,
			   size_t buffer_size,
			   int (*mse_completion)(int index, int size))
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int ret;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d buffer=%p size=%zu\n",
		 __func__, index, buffer, buffer_size);

	instance = &mse->instance_table[index];
	adapter = instance->media;

	if (instance->use_temp_video_buffer_mjpeg) {
		if (instance->f_temp_video_buffer_rewind) {
			memcpy(instance->temp_video_buffer,
			       instance->temp_video_buffer +
			       instance->eoi_pos + 2,
			       instance->temp_vw - instance->eoi_pos - 2);
			instance->temp_vw -= instance->eoi_pos + 2;
			instance->eoi_pos = 0;
			instance->f_temp_video_buffer_rewind = false;
		}
		if (!instance->f_first_vframe) {
			memcpy(instance->temp_video_buffer + instance->temp_vw,
			       buffer, buffer_size);
			instance->temp_vw += buffer_size;
		}
		instance->f_first_vframe = false;
		if (check_mjpeg(instance)) {
			instance->state = MSE_STATE_EXECUTE;
			instance->media_buffer = instance->temp_video_buffer;
			instance->media_buffer_size = instance->eoi_pos + 2;
			instance->f_trans_start = true;
		} else {
			/* not EOI */
			(*mse_completion)(instance->index_media, 0);

			return 0;
		}
	} else {
		instance->state = MSE_STATE_EXECUTE;
		instance->media_buffer = (unsigned char *)buffer;
		instance->media_buffer_size = buffer_size;
		instance->f_trans_start = true;
	}
	instance->mse_completion = mse_completion;
	instance->work_length = 0;

	ret = instance->network->set_option(instance->index_network);
	if (ret)
		pr_err("[%s] failed set_option() ret=%d\n", __func__, ret);

	if (instance->inout == MSE_DIRECTION_INPUT) {
		/* start workqueue for packetize */
		instance->f_continue = false;

		if (is_audio_adapter(adapter))
			create_avtp_timestamps(instance);

		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		memset(buffer, 0, buffer_size);

		/* start workqueue for streaming */
		if (!instance->f_streaming) {
			instance->f_streaming = true;
			queue_work(instance->wq_stream, &instance->wk_stream);
			pr_debug("[%s] start streaming\n", __func__);
		}
	}

	instance->timer_cnt++;

	return 0;
}
EXPORT_SYMBOL(mse_start_transmission);

int mse_get_private_data(int index_media, void **private_data)
{
	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n",
		       __func__, index_media);
		return -EINVAL;
	}

	*private_data = mse->media_table[index_media].private_data;

	return 0;
}
EXPORT_SYMBOL(mse_get_private_data);

enum MSE_DIRECTION mse_get_inout(int index_media)
{
	int index;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n",
		       __func__, index_media);
		return -EINVAL;
	}

	for (index = 0; index < ARRAY_SIZE(mse->instance_table); index++) {
		if (index_media == mse->instance_table[index].index_media)
			return mse->instance_table[index].inout;
	}

	pr_err("[%s] adapter was unregistered. index=%d\n",
	       __func__, index_media);

	return -EPERM;
}
EXPORT_SYMBOL(mse_get_inout);

int mse_register_mch(struct mch_ops *ops)
{
	int i;

	if (!ops) {
		pr_err("[%s] invalid argument. ops\n", __func__);
		return -EINVAL;
	}

	spin_lock(&mse->lock);

	for (i = 0; i < ARRAY_SIZE(mse->mch_table); i++) {
		if (!mse->mch_table[i]) {
			/* init table */
			mse->mch_table[i] = ops;
			pr_debug("[%s] registered index=%d\n", __func__, i);
			spin_unlock(&mse->lock);
			return i;
		}
	}

	pr_err("[%s] ops was unregistered\n", __func__);

	spin_unlock(&mse->lock);

	return -EBUSY;
}
EXPORT_SYMBOL(mse_register_mch);

int mse_unregister_mch(int index)
{
	if ((index < 0) || (index >= MSE_MCH_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] index=%d\n", __func__, index);

	spin_lock(&mse->lock);
	mse->mch_table[index] = NULL;
	spin_unlock(&mse->lock);

	return 0;
}
EXPORT_SYMBOL(mse_unregister_mch);

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
