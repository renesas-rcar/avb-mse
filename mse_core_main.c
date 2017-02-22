/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2017 Renesas Electronics Corporation

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
#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "mse_config.h"
#include "mse_packet_ctrl.h"
#include "mse_sysfs.h"
#include "mse_ptp.h"
#include "mse_ioctl_local.h"

#define NSEC_SCALE              (1000000000ul)
#define BUF_SIZE                (32)

#define NANO_SCALE              (1000000000ul)

#define MSE_RADIX_HEXADECIMAL   (16)
#define MSE_DEFAULT_BITRATE     (50000000) /* 50Mbps */

/** @brief MCH table max */
#define MSE_MCH_MAX                (10)
/** @brief PTP table max */
#define MSE_PTP_MAX                (10)

#define MSE_DMA_MAX_PACKET         (128)
#define MSE_DMA_MAX_PACKET_SIZE    (1526)
#define MSE_DMA_MAX_RECEIVE_PACKET (64)
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
#define CREATE_AVTP_TIMESTAMPS_MAX   (4096)

#define MSE_DECODE_BUFFER_NUM (8)
#define MSE_DECODE_BUFFER_NUM_START_MIN (2)
#define MSE_DECODE_BUFFER_NUM_START_MAX (6)
#define MAX_DECODE_SIZE       (8192) /* ALSA Period byte size */

#define mbit_to_bit(mbit)     (mbit * 1000000)

#define MPEG2TS_TIMER_NS        (10000000)           /* 10 msec */

#define MPEG2TS_TS_SIZE         (188)
#define MPEG2TS_SYNC            (0x47)
#define MPEG2TS_M2TS_OFFSET     (4)
#define MPEG2TS_M2TS_SIZE       (MPEG2TS_M2TS_OFFSET + MPEG2TS_TS_SIZE)
#define MPEG2TS_CLOCK_N         (9)                   /* 90kHz / 10K */
#define MPEG2TS_CLOCK_D         (100000)              /* 10^9(NSEC) / 10K */
#define MPEG2TS_PCR90K_BITS     (33)
#define MPEG2TS_PCR90K_INVALID  (BIT(MPEG2TS_PCR90K_BITS))
#define MPEG2TS_PCR_PID_IGNORE  (MSE_CONFIG_PCR_PID_MAX)

/**
 * @brief main data for Adapter
 */
struct mse_adapter {
	/** @brief instance used flag */
	bool used_f;
	/** @brief read-only flag for config */
	bool ro_config_f;
	/** @brief index */
	int index;
	/** @brief adapter name */
	char name[MSE_NAME_LEN_MAX];
	/** @brief type of Adapter */
	enum MSE_TYPE type;
	/** @brief adapter's private data */
	void *private_data;
	/** @brief device */
	struct device device;
	/** @brief configuration data */
	struct mse_config config;
};

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

	/** @brief instance direction */
	bool tx;
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
	/** @brief start streaming queue */
	struct work_struct wk_start_stream;
	/** @brief start transmission queue */
	struct work_struct wk_start_trans;

	/** @brief stream workqueue */
	struct workqueue_struct *wq_stream;
	/** @brief packet workqueue */
	struct workqueue_struct *wq_packet;
	/** @brief timestamp workqueue */
	struct workqueue_struct *wq_tstamp;
	/** @brief crf packet workqueue */
	struct workqueue_struct *wq_crf_packet;

	void *start_buffer;
	size_t start_buffer_size;
	int (*start_mse_completion)(void *priv, int size);

	/** @brief timer handler */
	struct hrtimer timer;
	int timer_delay;
	int timer_cnt;

	/** @brief spin lock for timer count */
	spinlock_t lock_timer;

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
	int ptp_index;
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
	spinlock_t lock_ques;
	struct timestamp_queue tstamp_que;
	struct crf_queue crf_que;
	struct avtp_queue avtp_que;

	/** @brief timestamp(nsec) */
	unsigned int timestamp;

	/** @brief private data */
	void *private_data;
	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;
	/** @brief media buffer */
	unsigned char *media_buffer;
	/** @brief buffer length for packetize */
	size_t media_buffer_size;
	/** @brief AVTP timestampes */
	unsigned int avtp_timestamps[CREATE_AVTP_TIMESTAMPS_MAX];
	int avtp_timestamps_size;
	int avtp_timestamps_current;
	/** @brief complete function pointer */
	int (*mse_completion)(void *priv, int size);
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
	bool f_work_timestamp;

	/** @brief network configuration */
	struct mse_network_config net_config;
	struct mse_network_config crf_net_config;

	/** @brief media specific configuration */
	struct mse_media_config {
		struct mse_audio_config audio;
		struct mse_video_config video;
		struct mse_mpeg2ts_config mpeg2ts;
	} media_config;
	struct mse_audio_info audio_info;

	/** @brief MCH & CRF Settings **/
	int ptp_clock;
	int ptp_clock_device;
	int ptp_clock_ch;
	int ptp_capture_freq;
	int media_clock_recovery;
	int media_capture_freq;
	enum MSE_CRF_TYPE crf_type;
	int max_transit_time;
	int talker_delay_time;
	int listener_delay_time;
	int remain;

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
	bool use_temp_video_buffer_mpeg2ts;
	bool f_temp_video_buffer_rewind;
	int parsed;
	int stored;
	u64 mpeg2ts_pcr_90k;
	u64 mpeg2ts_clock_90k;
	int mpeg2ts_pre_pcr_pid;
	u64 mpeg2ts_pre_pcr_90k;
	bool f_force_flush;
	unsigned char temp_video_buffer[128 * 1024];
	/** @brief audio buffer  */
	int temp_w;
	int temp_r;
	unsigned char temp_buffer[MSE_DECODE_BUFFER_NUM][MAX_DECODE_SIZE];
	unsigned char guard_buffer[8192];
};

static int mse_instance_max = MSE_INSTANCE_MAX;

struct mse_device {
	/** @brief device */
	struct platform_device *pdev;
	/** @brief device lock */
	spinlock_t lock_tables;           /* lock for talbes */
	/** @brief mutex lock for open */
	struct mutex mutex_open;
	/* @brief device class */
	struct class *class;

	struct mse_adapter_network_ops *network_table[MSE_ADAPTER_NETWORK_MAX];
	struct mse_adapter media_table[MSE_ADAPTER_MEDIA_MAX];
	struct mse_instance instance_table[MSE_INSTANCE_MAX];
	struct mse_ptp_ops *ptp_table[MSE_PTP_MAX];
	struct mch_ops *mch_table[MSE_MCH_MAX];
};

/* MSE device data */
static struct mse_device *mse;

/*
 * module parameters
 */
static int major;
module_param(major, int, 0440);

/*
 * internal functions
 */
static bool compare_pcr(u64 a, u64 b)
{
	u64 diff;

	diff = (a - b) & (BIT(MPEG2TS_PCR90K_BITS) - 1);
	return (diff < BIT(MPEG2TS_PCR90K_BITS - 1));
}

#if !(defined(CONFIG_MSE_SYSFS) || defined(CONFIG_MSE_IOCTL))
static inline int mse_create_config_device(int index_media) { return 0; }
static inline int mse_delete_config_device(int index_media) { return 0; }
#else
static void mse_device_release(struct device *dev)
{
	mse_debug("do noting\n");
}

static int mse_create_config_device(int index_media)
{
	struct device *device;
	struct mse_adapter *adapter = &mse->media_table[index_media];
	int err;

	mse_debug("index=%d\n", index_media);

	err = mse_ioctl_register(index_media);
	if (err < 0)
		return -EPERM;

	/* initialize device */
	device = &adapter->device;
	device->devt = MKDEV(major, index_media);
	device->class = mse->class;
	device->release = mse_device_release;

	mse_sysfs_set_device_groups(device, adapter->config.info.type);
	dev_set_name(device, "mse%d", index_media);

	err = device_register(device);
	if (err) {
		mse_err("failed device_register() mse%d\n", index_media);
		mse_ioctl_unregister(index_media);
		memset(&adapter->device, 0, sizeof(struct device));

		return err;
	}

	return 0;
}

static int mse_delete_config_device(int index_media)
{
	struct mse_adapter *adapter;

	mse_debug("START\n");

	adapter = &mse->media_table[index_media];

	mse_ioctl_unregister(index_media);
	device_unregister(&adapter->device);
	memset(&adapter->device, 0, sizeof(struct device));

	return 0;
}
#endif

static int mse_get_default_config(int index, struct mse_instance *instance)
{
	int ret = 0, err;
	struct mse_network_config *network = &instance->net_config;
	struct mse_network_config *crf_network = &instance->crf_net_config;
	struct mse_video_config *video = &instance->media_config.video;
	struct mse_audio_config *audio = &instance->media_config.audio;
	struct mse_mpeg2ts_config *mpeg2ts = &instance->media_config.mpeg2ts;
	struct mse_avtp_tx_param avtp_tx_param;
	struct mse_avtp_rx_param avtp_rx_param;
	struct mse_media_video_config video_config;
	struct mse_media_mpeg2ts_config mpeg2ts_config;
	struct mse_media_audio_config audio_config;
	struct mse_ptp_config ptp_config;
	struct mse_mch_config mch_config;
	struct mse_avtp_tx_param crf_tx;
	struct mse_avtp_rx_param crf_rx;
	struct mse_delay_time delay_time;

	err = mse_config_get_avtp_tx_param(index, &avtp_tx_param);
	if (err < 0) {
		mse_err("undefined avtp_tx_param\n");
		ret = -EPERM;
	}
	memcpy(network->dest_addr, avtp_tx_param.dst_mac,
	       sizeof(network->dest_addr));
	memcpy(network->source_addr, avtp_tx_param.src_mac,
	       sizeof(network->source_addr));
	network->priority = avtp_tx_param.priority;
	network->vlanid = avtp_tx_param.vlan;
	network->uniqueid = avtp_tx_param.uniqueid;

	err = mse_config_get_avtp_rx_param(index, &avtp_rx_param);
	if (err < 0) {
		mse_err("undefined avtp_rx_param\n");
		ret = -EPERM;
	}
	memcpy(network->streamid, avtp_rx_param.streamid,
	       sizeof(network->streamid));

	switch (instance->media->type) {
	case MSE_TYPE_ADAPTER_VIDEO:
		err = mse_config_get_media_video_config(index,
							&video_config);
		if (err < 0) {
			mse_err("undefined media_video_config\n");
			ret = -EPERM;
		}
		video->fps.numerator = video_config.fps_numerator;
		video->fps.denominator = video_config.fps_denominator;
		video->bitrate = video_config.bitrate;
		video->bytes_per_frame = video_config.bytes_per_frame;
		break;

	case MSE_TYPE_ADAPTER_MPEG2TS:
		err = mse_config_get_media_mpeg2ts_config(index,
							  &mpeg2ts_config);
		if (err < 0) {
			mse_err("undefined media_mpeg2ts_config\n");
			ret = -EPERM;
		}
		mpeg2ts->tspackets_per_frame =
			mpeg2ts_config.tspackets_per_frame;
		mpeg2ts->bitrate = mpeg2ts_config.bitrate;
		mpeg2ts->pcr_pid = mpeg2ts_config.pcr_pid;
		break;

	case MSE_TYPE_ADAPTER_AUDIO:
		err = mse_config_get_media_audio_config(index,
							&audio_config);
		if (err < 0) {
			mse_err("undefined audio config\n");
			ret = -EPERM;
		}
		audio->samples_per_frame = audio_config.samples_per_frame;

		err = mse_config_get_ptp_config(index,
						&ptp_config);
		instance->ptp_clock_device = 0;
		instance->ptp_clock = (ptp_config.type == MSE_PTP_TYPE_CAPTURE);
		instance->ptp_clock_ch = ptp_config.capture_ch;
		instance->ptp_capture_freq = ptp_config.capture_freq;

		err = mse_config_get_mch_config(index,
						&mch_config);
		instance->media_clock_recovery = mch_config.enable;
		instance->crf_type = audio_config.crf_type;
		instance->media_capture_freq =
			(ptp_config.recovery_capture_freq ==
			 MSE_RECOVERY_CAPTURE_FREQ_NOT_FIXED);

		err = mse_config_get_avtp_tx_param_crf(index,
						       &crf_tx);
		memcpy(crf_network->dest_addr, crf_tx.dst_mac,
		       sizeof(crf_network->dest_addr));
		memcpy(crf_network->source_addr, crf_tx.src_mac,
		       sizeof(crf_network->source_addr));
		crf_network->priority = crf_tx.priority;
		crf_network->vlanid = crf_tx.vlan;
		crf_network->uniqueid = crf_tx.uniqueid;

		err = mse_config_get_avtp_rx_param_crf(index,
						       &crf_rx);
		memcpy(crf_network->streamid, crf_rx.streamid,
		       sizeof(crf_network->streamid));

		err = mse_config_get_delay_time(index,
						&delay_time);
		instance->max_transit_time = delay_time.max_transit_time_ns;
		instance->talker_delay_time = delay_time.tx_delay_time_ns;
		instance->listener_delay_time = delay_time.rx_delay_time_ns;
		break;

	default:
		/* fall through */
		break;
	}

	return ret;
}

/*
 * PTP related functions
 */
static struct mse_ptp_ops mse_ptp_ops_dummy = {
	.open = mse_ptp_open_dummy,
	.close = mse_ptp_close_dummy,
	.get_time = mse_ptp_get_time_dummy,
	.get_timestamps = mse_ptp_get_timestamps_dummy,
};

static int mse_ptp_get_first_index(void)
{
	int i;

	for (i = 0; i < MSE_PTP_MAX; i++)
		if (mse->ptp_table[i])
			return i;

	return MSE_INDEX_UNDEFINED; /* not found, use dummy ops */
}

static struct mse_ptp_ops *mse_ptp_find_ops(int index)
{
	if ((index < 0) || (index >= MSE_PTP_MAX))
		return &mse_ptp_ops_dummy;
	else
		return mse->ptp_table[index];
}

static int mse_ptp_open(int index, int *dev_id)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->open(dev_id);
}

static int mse_ptp_close(int index, int dev_id)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->close(dev_id);
}

static int mse_ptp_get_time(int index,
			    int dev_id,
			    struct ptp_clock_time *clock_time)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->get_time(dev_id, clock_time);
}

static int mse_ptp_get_timestamps(int index,
				  int dev_id,
				  int ch,
				  int *count,
				  struct ptp_clock_time timestamps[])
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->get_timestamps(dev_id,
				     ch,
				     count,
				     timestamps);
}

static void mse_work_stream(struct work_struct *work)
{
	struct mse_instance *instance;
	int err = 0;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_stream);
	if (instance->f_stopping) {
		instance->f_streaming = false;
		return;
	}

	if (instance->tx) {
		/* request send packet */
		err = mse_packet_ctrl_send_packet(
			instance->index_network,
			instance->packet_buffer,
			instance->network);
		/* continue packetize workqueue */
		if (err >= 0 && instance->f_continue) {
			queue_work(instance->wq_packet,
				   &instance->wk_packetize);
		} else {
			instance->f_completion = true;
			mse_debug("f_completion = true\n");
			if (instance->timer_delay == 0 ||
			    instance->f_force_flush) {
				instance->f_force_flush = false;
				instance->f_trans_start = false;
				instance->mse_completion(
						instance->private_data, 0);
			}
		}
	} else {
		/* request receive packet */
		while (err >= 0 && !instance->f_stopping) {
			err = mse_packet_ctrl_receive_packet(
				instance->index_network,
				MSE_DMA_MAX_RECEIVE_PACKET,
				instance->packet_buffer,
				instance->network);
			if (instance->f_stopping)
				break;
			if (err < 0) {
				mse_err("receive error %d\n", err);
				break;
			}
			if (!instance->f_depacketizing) {
				instance->f_depacketizing = true;
				queue_work(instance->wq_packet,
					   &instance->wk_depacketize);
			}
		}
		mse_err("stop streaming %d\n", err);
	}
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
		mse_debug("time stamp queue is not updated\n");
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
		mse_debug("std_time %lu is over adjust range %lu - %lu\n",
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
		mse_debug("time stamp queue is not updated\n");
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
	mse_debug("found %lu t= %u avtp= %u\n", *std_time, t, avtp_time);
	return 0;
}

static int tstamps_enq_tstamps(struct timestamp_queue *que,
			       unsigned long std_time,
			       struct ptp_clock_time *clock_time)
{
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

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
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

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
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

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
	unsigned long flags;
	unsigned int d_t;

	ret = 0;
	out = 0;
	d_t = 0;

	if (instance->crf_type != MSE_CRF_TYPE_RX) {
		/* avtp */
		d_t = instance->audio_info.frame_interval_time;
		if (!instance->f_present)
			return -1;
		spin_lock_irqsave(&instance->lock_ques, flags);
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
				mse_debug("mch std_time %lu\n",
					  instance->mch_std_time);
			}

			ret = tstamps_calc_tstamp(
				&instance->tstamp_que,
				instance->mch_std_time,
				&device_time);
			if (ret < 0) {
				mse_debug("skip recovery\n");
				continue;
			}
			instance->master_timestamps[out] = avtp_time;
			instance->device_timestamps[out] = device_time;
			out++;

			if (out >= ARRAY_SIZE(instance->master_timestamps))
				break;
		}
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	} else {
		/* crf */
		spin_lock_irqsave(&instance->lock_ques, flags);
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
					mse_debug("skip recovery\n");
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
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	}

	if (out <= 0) {
		mse_debug("could not get master timestamps\n");
		return ret;
	}

	/* not mch ops registered */
	if (instance->mch_index < 0) {
		mse_err("mch is not initialized.\n");
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
		mse_debug("recover %lu\n", instance->add_std_time);
	}

	return 0;
}

static void mse_work_timestamp(struct work_struct *work)
{
	struct mse_instance *instance;
	int count;
	int ret;
	int i;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_timestamp);
	if (instance->f_stopping) {
		instance->f_work_timestamp = false;
		return;
	}

	/* capture timestamps */
	if (instance->ptp_clock == 1) {
		unsigned long flags;

		/* get timestamps */
		ret = mse_ptp_get_timestamps(instance->ptp_index,
					     instance->ptp_dev_id,
					     instance->ptp_clock_ch,
					     &count,
					     instance->timestamps);
		if (ret) {
			mse_warn("could not get timestamps ret=%d\n", ret);
			instance->f_work_timestamp = false;
			return;
		}

		/* store timestamps */
		spin_lock_irqsave(&instance->lock_ques, flags);
		for (i = 0; i < count; i++) {
			tstamps_enq_tstamps(&instance->tstamp_que,
					    instance->std_time_counter,
					    &instance->timestamps[i]);
			instance->std_time_counter += instance->add_std_time;
		}
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	}

	if (instance->media_clock_recovery == 1) {
		/* mch */
		media_clock_recovery(instance);
	}

	instance->f_work_timestamp = false;
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

	mse_debug("total=%d\n", count);

	for (i = 0; i < count; i++) {
		tstamps_deq_tstamps(que, &std_time, &clock_time);
		timestamps[i] = clock_time;
	}

	return 0;
}

static int tstamps_store_ptp_timestamp(struct mse_instance *instance)
{
	struct ptp_clock_time now;
	unsigned long flags;

	/* get now time form ptp and save */
	mse_ptp_get_time(instance->ptp_index,
			 instance->ptp_dev_id, &now);

	spin_lock_irqsave(&instance->lock_ques, flags);

	tstamps_enq_tstamps(&instance->tstamp_que,
			    instance->std_time_counter, &now);

	spin_unlock_irqrestore(&instance->lock_ques, flags);

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
	unsigned long flags;

	instance->packetizer->get_audio_info(
		instance->index_packetizer,
		&instance->audio_info);

	instance->avtp_timestamps_current = 0;

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

	if (num_t > CREATE_AVTP_TIMESTAMPS_MAX) {
		mse_err("too much packet, cannot create %d timestamps\n",
			num_t);
		return -EPERM;
	}

	mse_debug("create %d\n", num_t);
	d_t = instance->audio_info.frame_interval_time;
	spin_lock_irqsave(&instance->lock_ques, flags);
	size = tstamps_get_tstamps_size(&instance->tstamp_que);
	spin_unlock_irqrestore(&instance->lock_ques, flags);
	if (size < 2) {
		struct ptp_clock_time now;
		u64 t;

		/* no data (ptp_clock + std_time) */
		mse_ptp_get_time(instance->ptp_index,
				 instance->ptp_dev_id, &now);
		t = (u64)now.sec * NSEC_SCALE + now.nsec;
		for (i = 0; i < num_t; i++)
			instance->avtp_timestamps[i] = t + d_t * i + offset;

		instance->avtp_timestamps_size = num_t;
		instance->std_time_avtp += d_t * num_t;

		return 0;
	}
	/* get timestamps from private table */
	spin_lock_irqsave(&instance->lock_ques, flags);

	for (i = 0; i < num_t; i++) {
		unsigned long avtp_timestamp = 0;

		tstamps_calc_tstamp(&instance->tstamp_que,
				    instance->std_time_avtp,
				    &avtp_timestamp);
		instance->avtp_timestamps[i] = avtp_timestamp + offset;
		instance->std_time_avtp += d_t;
	}
	instance->avtp_timestamps_size = num_t;

	spin_unlock_irqrestore(&instance->lock_ques, flags);

	return 0;
}

static int mse_initialize_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;
	struct mse_audio_config *audio = &instance->media_config.audio;
	struct mse_audio_config config;
	struct mse_cbsparam cbs;
	int ret;

	ret = instance->crf_index = crf->open();
	if (instance->crf_index < 0) {
		mse_err("cannot open packetizer ret=%d\n", ret);
		return instance->crf_index;
	}

	crf->init(instance->crf_index);
	crf->set_network_config(instance->crf_index,
				&instance->crf_net_config);

	/* base_frequency */
	config.sample_rate = audio->sample_rate;
	/* timestamp_interval */
	if (!instance->ptp_clock) {
		config.samples_per_frame = audio->period_size;
	} else {
		config.samples_per_frame = audio->sample_rate /
			instance->ptp_capture_freq;
	}

	ret = crf->set_audio_config(instance->crf_index, &config);
	if (ret < 0)
		goto error_set_audio_config_fail;

	if (instance->crf_type == MSE_CRF_TYPE_TX) {
		ret = crf->calc_cbs(instance->crf_index, &cbs);
		if (ret < 0)
			goto error_calc_cbs_fail;

		ret = instance->network->set_cbs_param(
			instance->crf_index_network, &cbs);
		if (ret < 0)
			goto error_set_cbs_param_fail;
	}

	return 0;

error_set_cbs_param_fail:
error_calc_cbs_fail:
error_set_audio_config_fail:
	crf->release(instance->crf_index);
	instance->crf_index = MSE_INDEX_UNDEFINED;

	return ret;
}

static void mse_release_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;

	if (instance->crf_index >= 0)
		crf->release(instance->crf_index);
}

static void mse_work_packetize(struct work_struct *work)
{
	struct mse_instance *instance;
	int ret;

	instance = container_of(work, struct mse_instance, wk_packetize);

	mse_debug("trans size=%zu\n", instance->media_buffer_size);

	if (instance->f_stopping)
		return;

	if (instance->media->type == MSE_TYPE_ADAPTER_AUDIO) {
		/* make AVTP packet with timestamps */
		ret = mse_packet_ctrl_make_packet(
			instance->index_packetizer,
			instance->media_buffer,
			instance->media_buffer_size,
			instance->ptp_clock,
			&instance->avtp_timestamps_current,
			instance->avtp_timestamps_size,
			instance->avtp_timestamps,
			instance->packet_buffer,
			instance->packetizer,
			&instance->work_length);
	} else {
		/* make AVTP packet with one timesamp */
		ret = mse_packet_ctrl_make_packet(
			instance->index_packetizer,
			instance->media_buffer,
			instance->media_buffer_size,
			-1,
			NULL,
			1,
			&instance->timestamp,
			instance->packet_buffer,
			instance->packetizer,
			&instance->work_length);
	}

	if (ret < 0) {
		mse_err("error=%d\n", ret);
		return;
	}
	mse_debug("packetized=%d len=%zu\n",
		  ret, instance->work_length);
	/* start workqueue for streaming */
	instance->f_continue =
		(instance->work_length < instance->media_buffer_size);

	if (!instance->f_continue &&
	    (instance->use_temp_video_buffer_mjpeg ||
	     instance->use_temp_video_buffer_mpeg2ts))
		instance->f_temp_video_buffer_rewind = true;

	/* start workqueue for streaming */
	if (!instance->f_streaming) {
		instance->f_streaming = true;
		queue_work(instance->wq_stream, &instance->wk_stream);
	}
}

static int tstamps_store_avtp_timestamp(struct mse_instance *instance,
					unsigned long std_time,
					unsigned int timestamp)
{
	unsigned long flags;

	/* save avtp timestamp */
	spin_lock_irqsave(&instance->lock_ques, flags);
	tstamps_enq_avtp(&instance->avtp_que, std_time, timestamp);
	spin_unlock_irqrestore(&instance->lock_ques, flags);

	return 0;
}

static bool check_presentation_time(struct mse_instance *instance)
{
	struct ptp_clock_time now;
	unsigned int t = 0, t_d = 0;

	if (instance->f_present)
		return true;

	if (!instance->media_clock_recovery) {
		if (instance->temp_w < MSE_DECODE_BUFFER_NUM_START_MIN)
			return false;
	} else {
		mse_ptp_get_time(instance->ptp_index,
				 instance->ptp_dev_id, &now);
		t = (unsigned long)now.sec * NSEC_SCALE + now.nsec;
		t_d = instance->timestamp - t;
		if (t_d > UINT_MAX / 2 &&
		    instance->temp_w < MSE_DECODE_BUFFER_NUM_START_MAX)
			return false;
		mse_debug("start present avtp %u ptp %u q %d\n",
			  instance->timestamp, t, instance->temp_w);
	}
	instance->f_present = true;
	instance->start_present_time = instance->timestamp;
	mse_debug("start present listener avtp %u ptp %u q %d\n",
		  instance->timestamp, t, instance->temp_w);
	return true;
}

static void mse_work_depacketize(struct work_struct *work)
{
	struct mse_instance *instance;
	int received, ret;
	unsigned int timestamps[128];
	int t_stored, i;
	unsigned int d_t;
	struct mse_audio_config *audio;
	unsigned long flags;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_depacketize);
	if (instance->f_stopping) {
		mse_err("depaketize stopping\n");
		instance->f_depacketizing = false;
		return;
	}

	received = mse_packet_ctrl_check_packet_remain(
						instance->packet_buffer);

	switch (instance->media->type) {
	case MSE_TYPE_ADAPTER_AUDIO:
		/* get AVTP packet payload */
		audio = &instance->media_config.audio;
		while (!instance->f_stopping) {
			/* get AVTP packet payload */
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

			if (ret != -EAGAIN && ret < 0) {
				instance->f_trans_start = false;
				instance->mse_completion(
						instance->private_data, ret);
				break;
			}

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
			if (instance->work_length >=
			    instance->media_buffer_size) {
				instance->temp_w = (instance->temp_w + 1) %
					MSE_DECODE_BUFFER_NUM;
				instance->work_length = 0;
				continue;
			}
			/* Not loop */
			break;
		}
		mse_debug("received=%d depacketized=%zu ret=%d\n",
			  received, instance->work_length, ret);
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
	case MSE_TYPE_ADAPTER_MPEG2TS:
		if (!instance->f_trans_start) {
			instance->f_depacketizing = false;
			return;
		}

		/* get AVTP packet payload */
		ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						instance->media_buffer,
						instance->media_buffer_size,
						timestamps,
						ARRAY_SIZE(timestamps),
						&t_stored,
						instance->packet_buffer,
						instance->packetizer,
						&instance->work_length);

		mse_debug("media_buffer=%p received=%d depacketized=%d\n",
			  instance->media_buffer, received, ret);

		if (instance->f_stopping)
			break;

		/* complete callback */
		if (ret >= 0) {
			instance->f_trans_start = false;
			instance->mse_completion(instance->private_data, ret);
			instance->work_length = 0;
		} else if (ret == -EAGAIN) {
			spin_lock_irqsave(&instance->lock_timer, flags);
			instance->timer_cnt++;
			spin_unlock_irqrestore(&instance->lock_timer, flags);
			if (!instance->f_streaming) {
				instance->f_streaming = true;
				queue_work(instance->wq_stream,
					   &instance->wk_stream);
			}
		} else {
			instance->f_trans_start = false;
			instance->mse_completion(
					instance->private_data, ret);
			instance->work_length = 0;
		}
		break;

	default:
		mse_err("unknown type=0x%08x\n", instance->media->type);
		break;
	}
	instance->f_depacketizing = false;
}

static void mse_work_callback(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	unsigned long flags;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_callback);
	if (instance->f_stopping)
		return;

	adapter = instance->media;

	if (instance->ptp_clock == 0)
		tstamps_store_ptp_timestamp(instance);

	if (instance->use_temp_video_buffer_mpeg2ts) {
		mse_debug("mpeg2ts_clock_90k time=%llu pcr=%llu\n",
			  instance->mpeg2ts_clock_90k,
			  instance->mpeg2ts_pcr_90k);

		if (compare_pcr(instance->mpeg2ts_pcr_90k,
				instance->mpeg2ts_clock_90k)) {
			spin_lock_irqsave(&instance->lock_timer, flags);
			instance->timer_cnt++;
			spin_unlock_irqrestore(&instance->lock_timer, flags);
			return;
		}
	}

	if (!instance->tx) {
		if (IS_MSE_TYPE_AUDIO(adapter->type)) {
			if (instance->temp_w != instance->temp_r &&
			    check_presentation_time(instance)) {
				memcpy(instance->media_buffer,
				       instance->temp_buffer[instance->temp_r],
				       instance->media_buffer_size);
				instance->temp_r = (instance->temp_r + 1) %
					MSE_DECODE_BUFFER_NUM;
			}
		} else {
			return;
		}
	}

	/* complete callback anytime */
	instance->f_trans_start = false;
	instance->mse_completion(instance->private_data, 0);
}

static void mse_work_stop(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mch_ops *m_ops;
	enum MSE_CRF_TYPE crf_type;
	int ret;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_stop);

	instance->state = MSE_STATE_OPEN;
	adapter = instance->media;
	crf_type = instance->crf_type;

	ret = instance->network->cancel(instance->index_network);
	if (ret)
		mse_err("failed cancel() ret=%d\n", ret);

	if (crf_type == MSE_CRF_TYPE_RX) {
		ret = instance->network->cancel(instance->crf_index_network);
		if (ret)
			mse_err("failed cancel() ret=%d\n", ret);

		flush_work(&instance->wk_crf_receive);
	} else if (crf_type == MSE_CRF_TYPE_TX && instance->f_crf_sending) {
		flush_work(&instance->wk_crf_send);
	}

	/* timer stop */
	ret = hrtimer_try_to_cancel(&instance->timer);
	if (ret)
		mse_err("The timer was still in use...\n");

	/* timestamp timer, crf timer stop */
	if (IS_MSE_TYPE_AUDIO(adapter->type)) {
		ret = hrtimer_try_to_cancel(&instance->tstamp_timer);
		if (ret)
			mse_err("The tstamp_timer was still in use...\n");

		ret = hrtimer_try_to_cancel(&instance->crf_timer);
		if (ret)
			mse_err("The crf_timer was still in use...\n");

		if (instance->mch_index >= 0) {
			m_ops = mse->mch_table[instance->mch_index];
			ret = m_ops->close(instance->mch_dev_id);
			if (ret < 0) {
				mse_err("mch close error(%d).\n", ret);
			}
		}
	}

	if (instance->f_work_timestamp)
		flush_work(&instance->wk_timestamp);

	if (instance->f_streaming)
		flush_work(&instance->wk_stream);
}

static enum hrtimer_restart mse_timer_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	ktime_t ktime;
	unsigned long flags;

	instance = container_of(arg, struct mse_instance, timer);
	adapter = instance->media;

	if (instance->f_stopping) {
		mse_err("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	if (instance->mpeg2ts_clock_90k != MPEG2TS_PCR90K_INVALID) {
		instance->mpeg2ts_clock_90k +=
			(instance->timer_delay * MPEG2TS_CLOCK_N) /
			MPEG2TS_CLOCK_D;
	}

	/* timer update */
	ktime = ktime_set(0, instance->timer_delay);
	hrtimer_forward(&instance->timer,
			hrtimer_get_expires(&instance->timer),
			ktime);

	spin_lock_irqsave(&instance->lock_timer, flags);
	if (!instance->timer_cnt) {
		spin_unlock_irqrestore(&instance->lock_timer, flags);
		mse_debug("timer_cnt error\n");
		return HRTIMER_RESTART;
	}
	instance->timer_cnt--;

	spin_unlock_irqrestore(&instance->lock_timer, flags);

	/* start workqueue for completion */
	queue_work(instance->wq_packet, &instance->wk_callback);

	return HRTIMER_RESTART;
}

static void mse_work_crf_send(struct work_struct *work)
{
	struct mse_instance *instance;
	int err, tsize, size;
	struct ptp_clock_time timestamps[6];
	unsigned long flags;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_crf_send);

	if (instance->f_stopping) {
		instance->f_crf_sending = false;
		return;
	}

	tsize = instance->ptp_clock == 0 ?
		CRF_PTP_TIMESTAMPS : CRF_AUDIO_TIMESTAMPS;
	spin_lock_irqsave(&instance->lock_ques, flags);
	size = tstamps_get_tstamps_size(&instance->tstamp_que);
	spin_unlock_irqrestore(&instance->lock_ques, flags);
	while (!instance->f_stopping && size >= tsize) {
		/* get Timestamps */
		mse_debug("size %d tsize %d\n", size, tsize);
		spin_lock_irqsave(&instance->lock_ques, flags);
		get_timestamps(&instance->tstamp_que, tsize, timestamps);
		spin_unlock_irqrestore(&instance->lock_ques, flags);

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

		spin_lock_irqsave(&instance->lock_ques, flags);
		size = tstamps_get_tstamps_size(&instance->tstamp_que);
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	}

	instance->f_crf_sending = false;
}

static void mse_work_crf_receive(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int ret, err, count, i;
	u64 ptimes[6];
	unsigned long flags;
	char *dev_name;

	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;

	instance = container_of(work, struct mse_instance, wk_crf_receive);
	adapter = instance->media;

	mse_debug("START\n");

	dev_name = adapter->config.network_device.device_name_rx_crf;
	ret = instance->network->open(dev_name);
	instance->crf_index_network = ret;
	if (ret < 0)
		mse_err("can not open %d\n", ret);

	/* setup network */
	instance->network->set_streamid(instance->crf_index_network,
					instance->crf_net_config.streamid);

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
			mse_err("receive error %d\n", err);
			break;
		}

		count = mse_packet_ctrl_take_out_packet_crf(
			instance->crf_index,
			ptimes,
			ARRAY_SIZE(ptimes),
			instance->crf_packet_buffer);

		mse_debug("crf receive %d timestamp\n", count);

		crf->get_audio_info(
			instance->crf_index,
			&instance->crf_audio_info);

		spin_lock_irqsave(&instance->lock_ques, flags);
		for (i = 0; i < count; i++) {
			tstamps_enq_crf(&instance->crf_que,
					&instance->std_time_crf,
					&ptimes[i]);
			instance->std_time_crf +=
				instance->crf_audio_info.frame_interval_time;
		}
		spin_unlock_irqrestore(&instance->lock_ques, flags);
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
		mse_err("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	/* timer update */
	ktime = ktime_set(0, instance->crf_timer_delay);
	hrtimer_forward(&instance->crf_timer,
			hrtimer_get_expires(&instance->crf_timer),
			ktime);

	/* start workqueue for send */
	if (!instance->f_crf_sending) {
		instance->f_crf_sending = true;
		queue_work(instance->wq_crf_packet, &instance->wk_crf_send);
	}

	return HRTIMER_RESTART;
}

static enum hrtimer_restart mse_timestamp_collect_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	ktime_t ktime;

	instance = container_of(arg, struct mse_instance, tstamp_timer);

	mse_debug("START\n");

	if (instance->f_stopping) {
		mse_err("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	/* timer update */
	ktime = ktime_set(0, instance->tstamp_timer_delay);
	hrtimer_forward(&instance->tstamp_timer,
			hrtimer_get_expires(&instance->tstamp_timer),
			ktime);

	if (instance->f_work_timestamp)
		return HRTIMER_RESTART;

	instance->f_work_timestamp = true;
	queue_work(instance->wq_tstamp, &instance->wk_timestamp);

	return HRTIMER_RESTART;
}

static void mse_work_start_streaming(struct work_struct *work)
{
	struct mse_instance *instance;
	ktime_t ktime;
	ktime_t ktime_ts;
	ktime_t ktime_crf;
	struct mse_adapter *adapter;
	int ret = 0;
	struct ptp_clock_time now;
	unsigned long flags;

	instance = container_of(work, struct mse_instance, wk_start_stream);

	adapter = instance->media;

	/* get timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index,
			 instance->ptp_dev_id, &now);
	instance->timestamp = (unsigned long)now.sec * NSEC_SCALE + now.nsec;
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

	/* start timer */
	if (instance->timer_delay) {
		ktime = ktime_set(0, instance->timer_delay);
		hrtimer_start(&instance->timer, ktime, HRTIMER_MODE_REL);
	}

	if (IS_MSE_TYPE_AUDIO(adapter->type)) {
		/* capture timestamps */
		if (instance->ptp_clock == 1) {
			int count, i;
			/* get timestamps */
			ret = mse_ptp_get_timestamps(instance->ptp_index,
						     instance->ptp_dev_id,
						     instance->ptp_clock_ch,
						     &count,
						     instance->timestamps);
			if (ret) {
				mse_debug("could not get timestamps ret=%d\n",
					  ret);
				count = 0;
			}

			/* store timestamps */
			spin_lock_irqsave(&instance->lock_ques, flags);
			for (i = 0; i < count; i++) {
				tstamps_enq_tstamps(&instance->tstamp_que,
						    instance->std_time_counter,
						    &instance->timestamps[i]);
				instance->std_time_counter +=
					instance->add_std_time;
			}
			spin_unlock_irqrestore(&instance->lock_ques, flags);
		}

		/* ptp_clock is capture or media_clock_recovery is enable */
		if (instance->ptp_clock == 1 ||
		    instance->media_clock_recovery == 1) {
			ktime_ts = ktime_set(0, instance->tstamp_timer_delay);
			hrtimer_start(&instance->tstamp_timer,
				      ktime_ts, HRTIMER_MODE_REL);
		}

		/* send clock using CRF */
		if (instance->crf_type == MSE_CRF_TYPE_TX) {
			ktime_crf = ktime_set(0, instance->crf_timer_delay);
			hrtimer_start(&instance->crf_timer,
				      ktime_crf,
				      HRTIMER_MODE_REL);
		}

		/* receive clcok using CRF */
		if (instance->crf_type == MSE_CRF_TYPE_RX) {
			queue_work(instance->wq_crf_packet,
				   &instance->wk_crf_receive);
		}
	}

	instance->f_streaming = false;
	instance->f_stopping = false;
	instance->work_length = 0;
	instance->f_trans_start = false;
	instance->f_completion = false;

	instance->temp_w = 0;
	instance->temp_r = 0;
}

static inline bool is_mpeg2ts_ts(u8 *tsp)
{
	if (tsp[0] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_TS_SIZE] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_TS_SIZE * 2] == MPEG2TS_SYNC) {
		return true;
	} else {
		return false;
	}
}

static inline bool is_mpeg2ts_m2ts(u8 *tsp)
{
	if (tsp[MPEG2TS_M2TS_OFFSET] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_M2TS_OFFSET + MPEG2TS_M2TS_SIZE] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_M2TS_OFFSET + MPEG2TS_M2TS_SIZE * 2] == MPEG2TS_SYNC) {
		return true;
	} else {
		return false;
	}
}

static bool check_mpeg2ts_pcr(struct mse_instance *instance)
{
	u8 *tsp;
	u8 afc, afc_len, pcr_flag;
	u16 pid;
	u16 pcr_pid = instance->media_config.mpeg2ts.pcr_pid;
	u64 pcr;
	int psize, offset;

	if (instance->media_config.mpeg2ts.mpeg2ts_type ==
	    MSE_MPEG2TS_TYPE_TS) {
		psize = MPEG2TS_TS_SIZE;
		offset = 0;
	} else {
		psize = MPEG2TS_M2TS_SIZE;
		offset = MPEG2TS_M2TS_OFFSET;
	}

	while (instance->parsed + psize < instance->stored) {
		tsp = instance->temp_video_buffer +
			instance->parsed + offset;
		instance->parsed += psize;

		pid = (((u16)tsp[1] << 8) + (u16)tsp[2]) & 0x1fff;
		afc = (tsp[3] & 0x30) >> 4;
		afc_len = tsp[4];
		pcr_flag = (tsp[5] & 0x10) >> 4;

		/* Adaptation Field Control = 0b10 or 0b11 and  */
		/* afc_len >= 7 and pcr_flag = 1 */
		if (!(afc & 0x2) || (afc_len < 7) || !pcr_flag)
			continue;    /* no PCR */

		mse_debug("find pcr %d (required %d)\n", pid, pcr_pid);
		/* PCR base: 90KHz, 33bits (32+1bits) */
		pcr = ((u64)tsp[6] << 25) |
			((u64)tsp[7] << 17) |
			((u64)tsp[8] << 9) |
			((u64)tsp[9] << 1) |
			(((u64)tsp[10] & 0x80) >> 7);

		if (pcr_pid == MPEG2TS_PCR_PID_IGNORE) {
			if (instance->mpeg2ts_pre_pcr_pid !=
			    MPEG2TS_PCR_PID_IGNORE &&
			    (instance->mpeg2ts_pre_pcr_pid != pid ||
			     compare_pcr(instance->mpeg2ts_pre_pcr_90k, pcr))) {
				mse_info("change pid(%d -> %d) or rewind\n",
					 instance->mpeg2ts_pre_pcr_pid,
					pid);
				instance->mpeg2ts_clock_90k = pcr;
				instance->mpeg2ts_pcr_90k = pcr;
				instance->mpeg2ts_pre_pcr_pid = pid;
				instance->mpeg2ts_pre_pcr_90k = pcr;
				return true;
			}
		} else if (pcr_pid != pid) {
			continue;
		}
		instance->mpeg2ts_pre_pcr_pid = pid;
		instance->mpeg2ts_pre_pcr_90k = pcr;
		/*
		 * PCR extension: 27MHz, 9bits (8+1bits)
		 * Note: MSE is ignore PCR extension.
		 *
		 * pcr_ext = ((tsp[10] & 0x1) << 9) | tsp[11];
		 */
		if (compare_pcr(pcr, instance->mpeg2ts_clock_90k)) {
			if (instance->mpeg2ts_clock_90k ==
			    MPEG2TS_PCR90K_INVALID)
				instance->mpeg2ts_clock_90k = pcr;
			instance->mpeg2ts_pcr_90k = pcr;

			return true;
		}
	}

	return false;
}

static bool check_mjpeg(struct mse_instance *instance); /* tobe move */
static void mse_work_start_transmission(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	void *buffer;
	size_t buffer_size;
	int (*mse_completion)(void *priv, int size);
	unsigned long flags;
	int ret;
	struct ptp_clock_time now;

	instance = container_of(work, struct mse_instance, wk_start_trans);

	adapter = instance->media;

	buffer = instance->start_buffer;
	buffer_size = instance->start_buffer_size;
	mse_completion = instance->start_mse_completion;

	/* update timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index,
			 instance->ptp_dev_id, &now);
	instance->timestamp = (unsigned long)now.sec * NSEC_SCALE + now.nsec;

	/* TODO: to be move v4l2 adapter */
	if (instance->use_temp_video_buffer_mjpeg) {
		if (instance->f_temp_video_buffer_rewind) {
			memcpy(instance->temp_video_buffer,
			       instance->temp_video_buffer +
			       instance->parsed + 2,
			       instance->stored - instance->parsed - 2);
			instance->stored -= instance->parsed + 2;
			instance->parsed = 0;
			instance->f_temp_video_buffer_rewind = false;
		}
		if (!instance->f_first_vframe) {
			memcpy(instance->temp_video_buffer + instance->stored,
			       instance->start_buffer,
			       instance->start_buffer_size);
			instance->stored += buffer_size;
		}
		instance->f_first_vframe = false;
		if (check_mjpeg(instance)) {
			instance->state = MSE_STATE_EXECUTE;
			instance->media_buffer = instance->temp_video_buffer;
			instance->media_buffer_size = instance->parsed + 2;
			instance->f_trans_start = true;
		} else {
			/* not EOI */
			(*mse_completion)(instance->private_data, 0);
			return;
		}
	} else if (instance->use_temp_video_buffer_mpeg2ts) {
		if (instance->f_temp_video_buffer_rewind) {
			memcpy(instance->temp_video_buffer,
			       instance->temp_video_buffer +
			       instance->parsed,
			       instance->stored - instance->parsed);
			instance->stored -= instance->parsed;
			instance->parsed = 0;
			instance->f_temp_video_buffer_rewind = false;
		}

		if (instance->f_first_vframe) {
			/* check sync byte */
			enum MSE_MPEG2TS_TYPE ts_type;

			if (is_mpeg2ts_ts(buffer))
				ts_type = MSE_MPEG2TS_TYPE_TS;
			else if (is_mpeg2ts_m2ts(buffer))
				ts_type = MSE_MPEG2TS_TYPE_M2TS;
			else
				return;

			mse_debug("mpeg2ts_type=%d\n", ts_type);
			instance->media_config.mpeg2ts.mpeg2ts_type = ts_type;

			instance->packetizer->set_mpeg2ts_config(
				instance->index_packetizer,
				&instance->media_config.mpeg2ts);
		} else if (instance->stored + buffer_size <=
			   sizeof(instance->temp_video_buffer)) {
			memcpy(instance->temp_video_buffer + instance->stored,
			       buffer, buffer_size);
			instance->stored += buffer_size;
			if (instance->stored + buffer_size >=
			    sizeof(instance->temp_video_buffer))
				instance->f_force_flush = true;
		} else {
			mse_err("temp buffer overrun %u  %zu\n",
				instance->stored, buffer_size);
			return;
		}

		instance->f_first_vframe = false;

		if (check_mpeg2ts_pcr(instance) || instance->f_force_flush) {
			instance->state = MSE_STATE_EXECUTE;
			instance->media_buffer = instance->temp_video_buffer;
			instance->media_buffer_size = instance->parsed;
			instance->f_trans_start = true;
			instance->f_temp_video_buffer_rewind = true;
		} else {
			(*mse_completion)(instance->private_data, 0);
			return;
		}
	} else {
		instance->state = MSE_STATE_EXECUTE;
		instance->media_buffer = (unsigned char *)buffer;
		instance->media_buffer_size = buffer_size;
		instance->f_trans_start = true;
	}
	instance->mse_completion = mse_completion;

	if (instance->tx) {
		instance->work_length = 0;
		/* start workqueue for packetize */
		instance->f_continue = false;

		if (IS_MSE_TYPE_AUDIO(adapter->type)) {
			ret = create_avtp_timestamps(instance);
			if (ret < 0) {
				mse_completion(instance->private_data, ret);
				return;
			}
		}

		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		memset(buffer, 0, buffer_size);

		/* start workqueue for streaming */
		if (!instance->f_streaming) {
			instance->f_streaming = true;
			queue_work(instance->wq_stream, &instance->wk_stream);
		}
	}

	if (!instance->f_force_flush) {
		spin_lock_irqsave(&instance->lock_timer, flags);
		instance->timer_cnt++;
		spin_unlock_irqrestore(&instance->lock_timer, flags);
	}
}

static inline
enum MSE_STREAM_TYPE mse_type_to_stream_type(enum MSE_TYPE type)
{
	if (IS_MSE_TYPE_AUDIO(type))
		return MSE_STREAM_TYPE_AUDIO;
	else if (IS_MSE_TYPE_VIDEO(type))
		return MSE_STREAM_TYPE_VIDEO;
	else if (IS_MSE_TYPE_MPEG2TS(type))
		return MSE_STREAM_TYPE_MPEG2TS;
	else
		return -1;
}

/* External function for configuration */
int mse_dev_to_index(struct device *dev)
{
	struct mse_adapter *adapter;

	adapter = container_of(dev, struct mse_adapter, device);

	return adapter->index;
}

struct mse_config *mse_get_dev_config(int index)
{
	return &mse->media_table[index].config;
}

bool mse_dev_is_busy(int index)
{
	return mse->media_table[index].ro_config_f;
}

/* External function */
int mse_register_adapter_media(enum MSE_TYPE type,
			       char *name,
			       char *device_name)
{
	int index;
	unsigned long flags;

	/* check argument */
	if (!name) {
		mse_err("invalid argument. name\n");
		return -EINVAL;
	}

	if (!device_name) {
		mse_err("invalid argument. device_name\n");
		return -EINVAL;
	}

	switch (type) {
	case MSE_TYPE_ADAPTER_AUDIO:
	case MSE_TYPE_ADAPTER_VIDEO:
	case MSE_TYPE_ADAPTER_MPEG2TS:
		break;
	default:
		mse_err("unknown type=%d\n", type);
		return -EINVAL;
	}

	mse_debug("type=%d name=%s device_name=%s\n", type, name, device_name);

	spin_lock_irqsave(&mse->lock_tables, flags);

	/* search unused index */
	for (index = 0; index < ARRAY_SIZE(mse->media_table) &&
	     mse->media_table[index].used_f; index++)
		;

	if (index >= ARRAY_SIZE(mse->media_table)) {
		spin_unlock_irqrestore(&mse->lock_tables, flags);
		mse_err("%s is not registered\n", name);

		return -EBUSY;
	}

	/* init table */
	mse->media_table[index].used_f = true;
	mse->media_table[index].index = index;
	mse->media_table[index].type = type;
	strncpy(mse->media_table[index].name, name, MSE_NAME_LEN_MAX);

	spin_unlock_irqrestore(&mse->lock_tables, flags);

	/* init config data */
	mse_config_init(&mse->media_table[index].config,
			mse_type_to_stream_type(type),
			device_name);

	/* create control device */
	if (mse_create_config_device(index) < 0) {
		mse->media_table[index].used_f = false;
		mse_err("%s is not registered\n", name);

		return -EPERM;
	}

	mse_debug("registered index=%d\n", index);

	return index;
}
EXPORT_SYMBOL(mse_register_adapter_media);

int mse_unregister_adapter_media(int index_media)
{
	int i;
	unsigned long flags;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		mse_err("invalid argument. index=%d\n", index_media);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index_media);

	if (!mse->media_table[index_media].used_f) {
		mse_err("%d is not registered\n", index_media);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].index_media == index_media) {
			mse_err("module is in use. instance=%d\n", i);
			return -EPERM;
		}
	}

	/* delete control device */
	mse_delete_config_device(index_media);

	/* table delete */
	spin_lock_irqsave(&mse->lock_tables, flags);
	memset(&mse->media_table[index_media], 0, sizeof(struct mse_adapter));
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	mse_debug("unregistered\n");

	return 0;
}
EXPORT_SYMBOL(mse_unregister_adapter_media);

int mse_register_adapter_network(struct mse_adapter_network_ops *ops)
{
	int index;
	unsigned long flags;
	char name[MSE_NAME_LEN_MAX + 1];

	/* check argument */
	if (!ops) {
		mse_err("invalid argument. ops\n");
		return -EINVAL;
	}
	if (!ops->name) {
		mse_err("empty data. ops->name\n");
		return -EINVAL;
	}

	if (!IS_MSE_TYPE_NETWORK(ops->type)) {
		mse_err("unknown type=%d\n", ops->type);
		return -EINVAL;
	}

	mse_name_strlcpy(name, ops->name);
	mse_debug("type=%d name=%s\n", ops->type, name);

	spin_lock_irqsave(&mse->lock_tables, flags);

	/* search unused index */
	for (index = 0; index < ARRAY_SIZE(mse->network_table) &&
	     mse->network_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->network_table)) {
		spin_unlock_irqrestore(&mse->lock_tables, flags);
		mse_err("%s is not registered\n", name);

		return -EBUSY;
	}

	/* register table */
	mse->network_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	return index;
}
EXPORT_SYMBOL(mse_register_adapter_network);

int mse_unregister_adapter_network(int index)
{
	int i;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_NETWORK_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);

	if (!mse->network_table[index]) {
		mse_err("%d is not registered\n", index);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].index_network == index) {
			mse_err("module is in use. instance=%d\n", i);
			return -EPERM;
		}
	}

	spin_lock_irqsave(&mse->lock_tables, flags);
	mse->network_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	mse_debug("unregistered\n");

	return 0;
}
EXPORT_SYMBOL(mse_unregister_adapter_network);

int mse_get_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}
	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	/* get config */
	memcpy(config, &instance->media_config.audio, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_audio_config);

int mse_set_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;
	u64 timer;
	struct mse_adapter *adapter;
	struct mse_media_audio_config *media_audio_config;
	struct mse_network_config *net_config;
	struct mse_packetizer_ops *packetizer;
	struct mse_adapter_network_ops *network;
	int index_packetizer, index_network;
	int ret;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);
	mse_info("  sample_rate=%d channels=%d\n"
		 "  period_size=%d bytes_per_sample=%d bit_depth=%d\n"
		 "  is_big_endian=%d\n",
		 config->sample_rate,  config->channels,
		 config->period_size, config->bytes_per_sample,
		 mse_get_bit_depth(config->sample_bit_depth),
		 config->is_big_endian);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	adapter = instance->media;
	media_audio_config = &adapter->config.media_audio_config;
	net_config = &instance->net_config;
	packetizer = instance->packetizer;
	index_packetizer = instance->index_packetizer;
	network = instance->network;
	index_network = instance->index_network;

	/* calc timer value */
	timer = NSEC_SCALE * (u64)config->period_size;
	do_div(timer, config->sample_rate);
	instance->timer_delay = timer;
	mse_info("timer_delay=%d\n", instance->timer_delay);

	/* set AVTP header info */
	ret = packetizer->set_network_config(index_packetizer, net_config);
	if (ret < 0)
		return ret;

	/* init packet header */
	config->samples_per_frame = media_audio_config->samples_per_frame;
	ret = packetizer->set_audio_config(index_packetizer, config);
	if (ret < 0)
		return ret;

	if (instance->tx) {
		struct mse_cbsparam cbs;

		ret = packetizer->calc_cbs(index_packetizer, &cbs);
		if (ret < 0)
			return ret;

		ret = network->set_cbs_param(index_network, &cbs);
		if (ret < 0)
			return ret;
	} else {
		ret = network->set_streamid(index_network,
					    net_config->streamid);
		if (ret < 0)
			return ret;
	}

	/* set config */
	memcpy(&instance->media_config.audio, config, sizeof(*config));

	/* use CRF packet */
	if (instance->crf_type != MSE_CRF_TYPE_NOT_USE) {
		ret = mse_initialize_crf_packetizer(instance);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mse_set_audio_config);

int mse_get_video_config(int index, struct mse_video_config *config)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	/* get config */
	memcpy(config, &instance->media_config.video, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_video_config);

int mse_set_video_config(int index, struct mse_video_config *config)
{
	struct mse_instance *instance;
	u64 framerate;
	struct mse_network_config *net_config;
	struct mse_packetizer_ops *packetizer;
	struct mse_adapter_network_ops *network;
	int index_packetizer, index_network;
	int ret;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);
	mse_info("  format=%d bitrate=%d fps=%d/%d\n"
		 "  bytes_per_frame=%d\n",
		 config->format, config->bitrate, config->fps.numerator,
		 config->fps.denominator, config->bytes_per_frame);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	switch (config->format) {
	case MSE_VIDEO_FORMAT_H264_BYTE_STREAM:
	case MSE_VIDEO_FORMAT_H264_AVC:
		if ((instance->media->config.packetizer.packetizer !=
		     MSE_PACKETIZER_CVF_H264) &&
		    (instance->media->config.packetizer.packetizer !=
		     MSE_PACKETIZER_CVF_H264_D13)) {
			mse_err("invalid format.\n");
			return -EINVAL;
		}
		break;
	case MSE_VIDEO_FORMAT_MJPEG:
		if (instance->media->config.packetizer.packetizer !=
		    MSE_PACKETIZER_CVF_MJPEG) {
			mse_err("invalid format.\n");
			return -EINVAL;
		}
		break;
	default:
		mse_err("invalid format.\n");
		return -EINVAL;
	}

	net_config = &instance->net_config;
	packetizer = instance->packetizer;
	index_packetizer = instance->index_packetizer;
	network = instance->network;
	index_network = instance->index_network;

	/* calc timer value */
	if (instance->tx &&
	    config->fps.denominator != 0 && config->fps.numerator != 0) {
		framerate = NSEC_SCALE * (u64)config->fps.denominator;
		do_div(framerate, config->fps.numerator);
		instance->timer_delay = framerate;
		mse_info("timer_delay=%d\n", instance->timer_delay);
	}

	/* set AVTP header info */
	ret = packetizer->set_network_config(index_packetizer, net_config);
	if (ret < 0)
		return ret;

	/* init packet header */
	ret = packetizer->set_video_config(index_packetizer, config);
	if (ret < 0)
		return ret;

	if (instance->tx) {
		struct mse_cbsparam cbs;

		ret = packetizer->calc_cbs(index_packetizer, &cbs);
		if (ret < 0)
			return ret;

		ret = network->set_cbs_param(index_network, &cbs);
		if (ret < 0)
			return ret;

		mse_debug("bandwidth fraction = %08x\n",
			  cbs.bandwidth_fraction);
	} else {
		ret = network->set_streamid(index_network,
					    net_config->streamid);
		if (ret < 0)
			return ret;
	}

	/* set config */
	memcpy(&instance->media_config.video, config, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_set_video_config);

int mse_get_mpeg2ts_config(int index, struct mse_mpeg2ts_config *config)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	/* get config */
	memcpy(config, &instance->media_config.mpeg2ts, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_mpeg2ts_config);

int mse_set_mpeg2ts_config(int index, struct mse_mpeg2ts_config *config)
{
	struct mse_instance *instance;
	struct mse_network_config *net_config;
	struct mse_packetizer_ops *packetizer;
	struct mse_adapter_network_ops *network;
	int index_packetizer, index_network;

	int ret;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	net_config = &instance->net_config;
	packetizer = instance->packetizer;
	index_packetizer = instance->index_packetizer;
	network = instance->network;
	index_network = instance->index_network;

	instance->timer_delay = MPEG2TS_TIMER_NS;
	mse_info("timer_delay=%d\n", instance->timer_delay);

	/* set AVTP header info */
	ret = packetizer->set_network_config(index_packetizer, net_config);
	if (ret < 0)
		return ret;

	/* init packet header */
	ret = packetizer->set_mpeg2ts_config(index_packetizer, config);
	if (ret < 0)
		return ret;

	if (instance->tx) {
		struct mse_cbsparam cbs;

		ret = packetizer->calc_cbs(index_packetizer, &cbs);
		if (ret < 0)
			return ret;

		ret = network->set_cbs_param(index_network, &cbs);
		if (ret < 0)
			return ret;

		mse_debug("bandwidth fraction = %08x\n",
			  cbs.bandwidth_fraction);
	} else {
		ret = network->set_streamid(index_network,
					    net_config->streamid);
		if (ret < 0)
			return ret;
	}

	/* set config */
	memcpy(&instance->media_config.mpeg2ts, config, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_set_mpeg2ts_config);

static int check_mch_config(struct mse_instance *instance)
{
	bool mch_enable = instance->media_clock_recovery;
	bool tx = instance->tx;
	enum MSE_CRF_TYPE crf_type = instance->crf_type;

	if (!mch_enable) {
		if (crf_type == MSE_CRF_TYPE_RX)
			return 1;
	} else {
		if (tx && crf_type != MSE_CRF_TYPE_RX)
			return 1;
		else if (!tx && crf_type == MSE_CRF_TYPE_TX)
			return 1;
	}

	return 0; /* Valid */
}

int mse_open(int index_media, bool tx)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mse_adapter_network_ops *network;
	struct mse_packetizer_ops *packetizer;
	int ret, i, index, err;
	long link_speed;
	struct mch_ops *m_ops = NULL;
	unsigned long flags;
	struct mse_network_device *network_device;
	enum MSE_PACKETIZER packetizer_id;
	char *dev_name;
	char name[MSE_NAME_LEN_MAX + 1];

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		mse_err("invalid argument. index=%d\n", index_media);
		return -EINVAL;
	}

	mse_debug("index=%d tx=%d\n", index_media, tx);

	adapter = &mse->media_table[index_media];
	mutex_lock(&mse->mutex_open);

	if (!adapter->used_f) {
		mse_err("undefined media adapter index=%d\n", index_media);
		mutex_unlock(&mse->mutex_open);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (!mse->instance_table[i].used_f)
			break;
	}
	index = i;

	if (ARRAY_SIZE(mse->instance_table) <= index) {
		mse_err("resister instance full!\n");
		mutex_unlock(&mse->mutex_open);
		return -EBUSY;
	}

	instance = &mse->instance_table[index];
	instance->used_f = true;
	adapter->ro_config_f = true;
	spin_lock_init(&instance->lock_timer);
	spin_lock_init(&instance->lock_ques);

	mutex_unlock(&mse->mutex_open);

	network_device = &adapter->config.network_device;

	/* search network adapter name for configuration value */
	for (i = 0; i < ARRAY_SIZE(mse->network_table); i++) {
		network = mse->network_table[i];
		if (!network)
			continue;

		if (!mse_compare_param_key(network_device->module_name,
					   network->name))
			break;
	}

	if (i >= ARRAY_SIZE(mse->network_table)) {
		mse_err("network adapter module is not loaded\n");
		err = -ENODEV;

		goto error_network_adapter_not_found;
	}

	mse_name_strlcpy(name, network->name);
	mse_debug("network adapter index=%d name=%s\n", i, name);

	/* get packetizer id */
	packetizer_id = adapter->config.packetizer.packetizer;
	if (!mse_packetizer_is_valid(packetizer_id)) {
		mse_err("packetizer is not valid\n");
		err = -EINVAL;

		goto error_packetizer_is_not_valid;
	}

	/* if mjpeg input */
	if (packetizer_id == MSE_PACKETIZER_CVF_MJPEG && tx) {
		mse_debug("use mjpeg\n");
		instance->use_temp_video_buffer_mjpeg = true;
		instance->f_first_vframe = true;
	}

	/* if mpeg2-ts input */
	if (packetizer_id == MSE_PACKETIZER_IEC61883_4 && tx) {
		mse_debug("use mpeg2ts\n");
		instance->use_temp_video_buffer_mpeg2ts = true;
		instance->f_first_vframe = true;
	}
	instance->f_force_flush = false;

	/* packetizer for configuration value */
	packetizer = mse_packetizer_get_ops(packetizer_id);

	mse_debug("packetizer id=%d\n", packetizer_id);

	/* ptp open */
	instance->ptp_index = mse_ptp_get_first_index();
	ret = mse_ptp_open(instance->ptp_index, &instance->ptp_dev_id);
	if (ret < 0) {
		mse_err("cannot mse_ptp_open()\n");
		err = ret;

		goto error_cannot_open_ptp;
	}

	/* open network adapter */
	if (tx)
		dev_name = network_device->device_name_tx;
	else
		dev_name = network_device->device_name_rx;

	ret = network->open(dev_name);
	if (ret < 0) {
		mse_err("cannot open network adapter ret=%d\n", ret);
		err = ret;

		goto error_cannot_open_network_adapter;
	}
	instance->index_network = ret;

	ret = network->set_option(instance->index_network);
	if (ret)
		mse_err("failed set_option() ret=%d\n", ret);

	/* get speed link */
	link_speed = network->get_link_speed(instance->index_network);
	if (link_speed <= 0) {
		mse_err("Link Down. ret=%ld\n", link_speed);
		err = -ENETDOWN;

		goto error_network_interface_is_link_down;
	}

	mse_debug("Link Speed=%ldMbps\n", link_speed);

	instance->net_config.port_transmit_rate = mbit_to_bit(link_speed);
	instance->crf_net_config.port_transmit_rate = mbit_to_bit(link_speed);

	/* open paketizer */
	ret = packetizer->open();
	if (ret < 0) {
		mse_err("cannot open packetizer ret=%d\n", ret);
		err = ret;

		goto error_cannot_open_packetizer;
	}
	instance->index_packetizer = ret;

	/* set selector */
	instance->tx = tx;
	instance->state = MSE_STATE_OPEN;
	instance->media = adapter;
	instance->packetizer = packetizer;
	instance->network = network;
	instance->index_media = index_media;
	instance->crf_index = MSE_INDEX_UNDEFINED;

	/* init work queue */
	INIT_WORK(&instance->wk_packetize, mse_work_packetize);
	INIT_WORK(&instance->wk_depacketize, mse_work_depacketize);
	INIT_WORK(&instance->wk_callback, mse_work_callback);
	INIT_WORK(&instance->wk_stream, mse_work_stream);
	INIT_WORK(&instance->wk_stop, mse_work_stop);
	INIT_WORK(&instance->wk_crf_send, mse_work_crf_send);
	INIT_WORK(&instance->wk_crf_receive, mse_work_crf_receive);
	INIT_WORK(&instance->wk_timestamp, mse_work_timestamp);
	INIT_WORK(&instance->wk_start_stream, mse_work_start_streaming);
	INIT_WORK(&instance->wk_start_trans, mse_work_start_transmission);

	instance->wq_stream = create_singlethread_workqueue("mse_streamq");
	instance->wq_packet = create_singlethread_workqueue("mse_packetq");
	hrtimer_init(&instance->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	instance->timer_delay = 0;
	instance->timer.function = &mse_timer_callback;

	/* for timestamp */
	instance->wq_tstamp = create_singlethread_workqueue("mse_tstampq");
	if (IS_MSE_TYPE_AUDIO(adapter->type)) {
		spin_lock_irqsave(&instance->lock_ques, flags);
		tstamps_clear_tstamps(&instance->tstamp_que);
		spin_unlock_irqrestore(&instance->lock_ques, flags);

		hrtimer_init(&instance->tstamp_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->tstamp_timer_delay = PTP_DELAY;
		instance->tstamp_timer.function =
					&mse_timestamp_collect_callback;

		/* for crf */
		instance->wq_crf_packet =
			create_singlethread_workqueue("mse_crfpacketq");
		hrtimer_init(&instance->crf_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->crf_timer_delay = CRF_DELAY;
		instance->crf_timer.function = &mse_crf_callback;
	}

	ret = mse_get_default_config(index_media, instance);
	if (ret < 0)
		mse_err("cannot get configurations\n");

	if (check_mch_config(instance)) {
		mse_err("media clock recovery config is invalid mch_config.enable=%d media_audio_config.crf_type=%d\n",
			instance->media_clock_recovery, instance->crf_type);
		goto error_mch_config_invalid;
	}

	if (instance->media_clock_recovery == 1) {
		for (i = 0; i < MSE_MCH_MAX; i++) {
			m_ops = mse->mch_table[i];
			if (m_ops) {
				instance->mch_index = i;
				break;
			}
		}
		if (instance->mch_index < 0) {
			mse_err("mch is not registered.\n");
			err = -EINVAL;

			goto error_mch_not_found;
		}

		ret = m_ops->open(&instance->mch_dev_id);
		if (ret < 0) {
			mse_err("mch open error(%d).\n", ret);
			instance->mch_index = MSE_INDEX_UNDEFINED;
			err = -EINVAL;

			goto error_mch_cannot_open;
		}
	}

	/* get packet memory */
	instance->packet_buffer = mse_packet_ctrl_alloc(
						&mse->pdev->dev,
						MSE_DMA_MAX_PACKET * 2,
						MSE_DMA_MAX_PACKET_SIZE);

	if (instance->tx)
		mse_packet_ctrl_send_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);
	else
		mse_packet_ctrl_receive_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);

	/* send clock using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_TX) {
		int ret;

		dev_name = network_device->device_name_tx_crf;
		ret = instance->network->open(dev_name);
		if (ret < 0) {
			err = -EINVAL;

			goto error_cannot_open_network_device_for_crf;
		}

		instance->crf_index_network = ret;

		/* get packet memory */
		instance->crf_packet_buffer = mse_packet_ctrl_alloc(
			&mse->pdev->dev,
			MSE_DMA_MAX_PACKET,
			MSE_DMA_MAX_PACKET_SIZE);
		if (!instance->crf_packet_buffer) {
			err = -EINVAL;

			goto error_cannot_alloc_crf_packet_buffer;
		}

		/* prepare for send */
		mse_packet_ctrl_send_prepare_packet(
			instance->crf_index_network,
			instance->crf_packet_buffer,
			instance->network);
	}

	/* init paketizer */
	instance->packetizer->init(instance->index_packetizer);

	return index;

error_cannot_alloc_crf_packet_buffer:
	network->release(instance->crf_index_network);

error_cannot_open_network_device_for_crf:
	mse_packet_ctrl_free(instance->packet_buffer);
	if (m_ops)
		m_ops->close(instance->mch_dev_id);

error_mch_cannot_open:
error_mch_not_found:
error_mch_config_invalid:
	if (instance->wq_crf_packet)
		destroy_workqueue(instance->wq_crf_packet);

	destroy_workqueue(instance->wq_tstamp);
	destroy_workqueue(instance->wq_packet);
	destroy_workqueue(instance->wq_stream);

	packetizer->release(instance->index_packetizer);

error_cannot_open_packetizer:
error_network_interface_is_link_down:
	network->release(instance->index_network);

error_cannot_open_network_adapter:
	mse_ptp_close(instance->ptp_index, instance->ptp_dev_id);

error_cannot_open_ptp:
error_network_adapter_not_found:
error_packetizer_is_not_valid:
	instance->used_f = false;
	adapter->ro_config_f = false;

	return err;
}
EXPORT_SYMBOL(mse_open);

int mse_close(int index)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int ret;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);

	instance = &mse->instance_table[index];
	adapter = instance->media;

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	/* wait stop */
	flush_work(&instance->wk_stop);

	/* free packet memory */
	if (instance->packet_buffer)
		mse_packet_ctrl_free(instance->packet_buffer);

	if (instance->crf_packet_buffer)
		mse_packet_ctrl_free(instance->crf_packet_buffer);

	/* destroy workqueue */
	destroy_workqueue(instance->wq_packet);
	destroy_workqueue(instance->wq_stream);

	destroy_workqueue(instance->wq_tstamp);
	if (IS_MSE_TYPE_AUDIO(adapter->type))
		destroy_workqueue(instance->wq_crf_packet);

	/* release network adapter */
	instance->network->release(instance->index_network);
	if (instance->crf_type == MSE_CRF_TYPE_TX)
		instance->network->release(instance->crf_index_network);

	/* release packetizer */
	instance->packetizer->release(instance->index_packetizer);
	mse_release_crf_packetizer(instance);

	ret = mse_ptp_close(instance->ptp_index, instance->ptp_dev_id);
	if (ret < 0)
		mse_err("cannot mse_ptp_close()\n");

	/* set table */
	memset(instance, 0, sizeof(*instance));
	instance->used_f = false;
	instance->state = MSE_STATE_CLOSE;
	instance->index_media = MSE_INDEX_UNDEFINED;
	instance->index_network = MSE_INDEX_UNDEFINED;
	instance->index_packetizer = MSE_INDEX_UNDEFINED;
	instance->mch_index = MSE_INDEX_UNDEFINED;
	instance->ptp_index = MSE_INDEX_UNDEFINED;
	adapter->ro_config_f = false;

	return 0;
}
EXPORT_SYMBOL(mse_close);

int mse_start_streaming(int index)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);
	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	queue_work(instance->wq_packet,
		   &instance->wk_start_stream);

	instance->mpeg2ts_clock_90k = MPEG2TS_PCR90K_INVALID;
	instance->mpeg2ts_pre_pcr_pid = MPEG2TS_PCR_PID_IGNORE;
	instance->mpeg2ts_pre_pcr_90k = 0;

	return 0;
}
EXPORT_SYMBOL(mse_start_streaming);

int mse_stop_streaming(int index)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_err("index=%d\n", index);
	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	instance->f_stopping = true;

	instance = &mse->instance_table[index];
	queue_work(instance->wq_packet,
		   &instance->wk_stop);

	return 0;
}
EXPORT_SYMBOL(mse_stop_streaming);

static bool check_mjpeg(struct mse_instance *instance)
{
	int i;

	if (instance->parsed > 0)
		return false;

	for (i = 0; i < instance->stored - 1; i++) {
		if (instance->temp_video_buffer[i] == 0xFF &&
		    instance->temp_video_buffer[i + 1] == 0xD9) {
			instance->parsed = i;
			return true;
		}
	}

	return false;
}

int mse_start_transmission(int index,
			   void *buffer,
			   size_t buffer_size,
			   void *priv,
			   int (*mse_completion)(void *priv, int size))
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d buffer=%p size=%zu\n", index, buffer, buffer_size);

	instance = &mse->instance_table[index];

	if (!instance->used_f) {
		mse_err("index=%d is not opened", index);
		return -EINVAL;
	}

	instance->start_buffer = buffer;
	instance->start_buffer_size = buffer_size;
	instance->private_data = priv;
	instance->start_mse_completion = mse_completion;

	queue_work(instance->wq_packet,
		   &instance->wk_start_trans);

	return 0;
}
EXPORT_SYMBOL(mse_start_transmission);

int mse_register_mch(struct mch_ops *ops)
{
	int index;
	unsigned long flags;

	if (!ops) {
		mse_err("invalid argument. ops\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&mse->lock_tables, flags);

	for (index = 0; index < ARRAY_SIZE(mse->mch_table) &&
	     mse->mch_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->mch_table)) {
		mse_err("ops is not registered\n");
		spin_unlock_irqrestore(&mse->lock_tables, flags);

		return -EBUSY;
	}

	/* init table */
	mse->mch_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	return index;
}
EXPORT_SYMBOL(mse_register_mch);

int mse_unregister_mch(int index)
{
	int i;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_MCH_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].mch_index == index) {
			mse_err("module is in use. instance=%d\n", i);
			return -EPERM;
		}
	}

	spin_lock_irqsave(&mse->lock_tables, flags);
	mse->mch_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	return 0;
}
EXPORT_SYMBOL(mse_unregister_mch);

int mse_register_ptp(struct mse_ptp_ops *ops)
{
	int index;
	unsigned long flags;

	if (!ops) {
		mse_err("invalid argument. ops\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&mse->lock_tables, flags);

	/* search unused index */
	for (index = 0; index < ARRAY_SIZE(mse->ptp_table) &&
	     mse->ptp_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->ptp_table)) {
		mse_err("ops is not registered\n");
		spin_unlock_irqrestore(&mse->lock_tables, flags);

		return -EBUSY;
	}

	/* register table */
	mse->ptp_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	return index;
}
EXPORT_SYMBOL(mse_register_ptp);

int mse_unregister_ptp(int index)
{
	int i;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_PTP_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);

	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i].ptp_index == index) {
			mse_err("module is in use. instance=%d\n", i);
			return -EPERM;
		}
	}

	spin_lock_irqsave(&mse->lock_tables, flags);
	mse->ptp_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_tables, flags);

	return 0;
}
EXPORT_SYMBOL(mse_unregister_ptp);

/*
 * initialize MSE API
 */
static int mse_probe(void)
{
	int i;

	/* allocate device data */
	mse = kzalloc(sizeof(*mse), GFP_KERNEL);
	if (!mse)
		return -ENOMEM;

	spin_lock_init(&mse->lock_tables);
	mutex_init(&mse->mutex_open);

	/* register platform device */
	mse->pdev = platform_device_register_simple("mse", -1, NULL, 0);
	if (IS_ERR(mse->pdev)) {
		mse_err("Failed to register platform device. ret=%p\n",
			mse->pdev);
		return -EINVAL;
	}

	/* W/A for cannot using DMA APIs */
	of_dma_configure(&mse->pdev->dev, NULL);

#if defined(CONFIG_MSE_SYSFS)
	/* create class */
	mse->class = class_create(THIS_MODULE, "ravb_mse");
	if (IS_ERR(mse->class)) {
		int err = PTR_RET(mse->class);
		mse_err("failed class_create() ret=%d\n", err);
		kfree(mse);
		return err;
	}
#endif

	/* init ioctl device */
	major = mse_ioctl_init(major, mse_instance_max);

	/* init table */
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		mse->instance_table[i].index_media = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].index_network = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].index_packetizer = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].mch_index = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].ptp_index = MSE_INDEX_UNDEFINED;
	}

	for (i = 0; i < ARRAY_SIZE(mse->media_table); i++)
		mse->media_table[i].index = MSE_INDEX_UNDEFINED;

	mse_debug("success\n");

	return 0;
}

/*
 * cleanup MSE API
 */
static int mse_remove(void)
{
	/* release ioctl device */
	mse_ioctl_exit(major, mse_instance_max);
	/* destroy class */
	if (mse->class)
		class_destroy(mse->class);
	/* unregister platform device */
	platform_device_unregister(mse->pdev);
	/* release device data */
	kfree(mse);

	mse_debug("success\n");

	return 0;
}

static int __init mse_module_init(void)
{
	mse_debug("START\n");
	return mse_probe();
}

static void __exit mse_module_exit(void)
{
	mse_debug("START\n");
	mse_remove();
}

module_init(mse_module_init);
module_exit(mse_module_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
