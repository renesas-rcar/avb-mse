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
#include <linux/completion.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/ptp_clock.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include "avtp.h"
#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "mse_config.h"
#include "mse_packet_ctrl.h"
#include "mse_sysfs.h"
#include "mse_ptp.h"
#include "mse_ioctl_local.h"

#define MSE_DEBUG_TSTAMPS (0)
#define MSE_DEBUG_STATE   (0)

#define BUF_SIZE                (32)

#define NANO_SCALE              (1000000000ul)

#define MSE_RADIX_HEXADECIMAL   (16)
#define MSE_DEFAULT_BITRATE     (50000000) /* 50Mbps */

/** @brief MCH table max */
#define MSE_MCH_MAX                (10)
/** @brief PTP table max */
#define MSE_PTP_MAX                (10)

/** @brief packet buffer related definitions */
#define MSE_PACKET_SIZE_MAX     (1526)
#define MSE_TX_PACKET_NUM_MAX   (128)
#define MSE_RX_PACKET_NUM_MAX   (128)
#define MSE_TX_PACKET_NUM       (MSE_TX_PACKET_NUM_MAX)
#define MSE_RX_PACKET_NUM       (64)
#define MSE_TX_RING_SIZE        (MSE_TX_PACKET_NUM_MAX * 3)
#define MSE_RX_RING_SIZE        (MSE_RX_PACKET_NUM_MAX * 2)
#define MSE_CRF_TX_RING_SIZE    (MSE_TX_PACKET_NUM_MAX)
#define MSE_CRF_RX_RING_SIZE    (MSE_RX_PACKET_NUM_MAX * 2)

#define q_next(pos, max)        (((pos) + 1) % max)

#define PTP_TIMESTAMPS_MAX   (512)
#define PTP_TIMER_INTERVAL   (20 * 1000000)  /* 1/300 sec * 6 = 20ms */

#define CRF_TIMESTAMPS_MAX   (512)
#define CRF_TIMER_INTERVAL   (20 * 1000000)  /* 20ms */
#define CRF_PTP_TIMESTAMPS   (1)     /* timestamps per CRF packet using ptp */
#define CRF_AUDIO_TIMESTAMPS (6)     /* audio timestamps per CRF packet */

#define AVTP_TIMESTAMPS_MAX  (512)
#define CREATE_AVTP_TIMESTAMPS_MAX   (4096)

#define MSE_DECODE_BUFFER_NUM (8)
#define MSE_DECODE_BUFFER_NUM_START_MIN (2)
#define MSE_DECODE_BUFFER_NUM_START_MAX (6)
#define MAX_DECODE_SIZE       (8192) /* ALSA Period byte size */

#define MSE_TRANS_BUF_NUM (3) /* size of transmission buffer array */
#define MSE_TRANS_BUF_ACCEPTABLE (MSE_TRANS_BUF_NUM - 1)

#define MSE_MPEG2TS_BUF_NUM  (MSE_TRANS_BUF_NUM)
#define MSE_MPEG2TS_BUF_SIZE (512U * 1024U)

#define mbit_to_bit(mbit)     (mbit * 1000000)

#define MPEG2TS_TIMER_NS        (10000000)           /* 10 msec */

/* (timer_ns * 90kHz) / 1e9 => (timer_ns * 9) / 1e5 */
#define MPEG2TS_CLOCK_INC       ((MPEG2TS_TIMER_NS * 9) / 100000)

#define MPEG2TS_TS_SIZE         (188)
#define MPEG2TS_SYNC            (0x47)
#define MPEG2TS_M2TS_OFFSET     (4)
#define MPEG2TS_M2TS_SIZE       (MPEG2TS_M2TS_OFFSET + MPEG2TS_TS_SIZE)
#define MPEG2TS_PCR90K_BITS     (33)
#define MPEG2TS_PCR90K_INVALID  (BIT(MPEG2TS_PCR90K_BITS))
#define MPEG2TS_PCR_PID_IGNORE  (MSE_CONFIG_PCR_PID_MAX)

#define atomic_dec_not_zero(v)  atomic_add_unless((v), -1, 0)

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
	MSE_STATE_UNDEFINED = 0x00000000,
	MSE_STATE_CLOSE     = 0x00000001,
	MSE_STATE_OPEN      = 0x00000002,
	MSE_STATE_IDLE      = 0x00000004,
	MSE_STATE_EXECUTE   = 0x00000008,
	MSE_STATE_STOPPING  = 0x00000010,

	MSE_STATE_RUNNABLE  = MSE_STATE_IDLE |
			      MSE_STATE_EXECUTE,

	MSE_STATE_STARTED   = MSE_STATE_IDLE    |
			      MSE_STATE_EXECUTE |
			      MSE_STATE_STOPPING,

	MSE_STATE_RUNNING   = MSE_STATE_EXECUTE |
			      MSE_STATE_STOPPING,
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

/** @brief transmission buffer */
struct mse_trans_buffer {
	/** @brief media buffer of adapter */
	void *media_buffer;
	/** @brief media buffer for core processing */
	u8 *buffer;
	/** @brief media buffer size */
	size_t buffer_size;
	/** @brief processed length of media buffer */
	size_t work_length;
	/** @brief private data of media adapter */
	void *private_data;
	/** @brief callback function to media adapter */
	int (*mse_completion)(void *priv, int size);

	struct list_head list;
};

/** @brief instance by related adapter */
struct mse_instance {
	/** @brief instance used flag */
	bool used_f;

	/** @brief wait for streaming stop */
	struct completion completion_stop;

	/** @brief instance direction */
	bool tx;
	/** @brief instance state */
	enum MSE_STATE state;
	/** @brief spin lock for state */
	rwlock_t lock_state;

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
	enum MSE_PACKETIZER packetizer_id;

	/** @brief streaming queue */
	struct work_struct wk_stream;
	/** @brief paketize queue */
	struct work_struct wk_packetize;
	/** @brief depaketize queue */
	struct work_struct wk_depacketize;
	/** @brief callback queue */
	struct work_struct wk_callback;
	/** @brief timestamp queue */
	struct work_struct wk_timestamp;
	/** @brief crf send queue */
	struct work_struct wk_crf_send;
	/** @brief crf receive queue */
	struct work_struct wk_crf_receive;
	/** @brief start transmission queue */
	struct work_struct wk_start_trans;
	/** @brief stop streaming queue */
	struct work_struct wk_stop_streaming;

	/** @brief stream workqueue */
	struct workqueue_struct *wq_stream;
	/** @brief packet workqueue */
	struct workqueue_struct *wq_packet;
	/** @brief timestamp workqueue */
	struct workqueue_struct *wq_tstamp;
	/** @brief crf packet workqueue */
	struct workqueue_struct *wq_crf_packet;

	/** @brief wait queue for streaming */
	wait_queue_head_t wait_wk_stream;

	/** @brief spin lock for buffer list */
	spinlock_t lock_buf_list;
	/** @brief array of transmission buffer */
	struct mse_trans_buffer trans_buffer[MSE_TRANS_BUF_NUM];
	/** @brief list of transmission buffer is not completed */
	struct list_head trans_buf_list;
	/** @brief list of transmission buffer for core processing */
	struct list_head proc_buf_list;
	/** brief index of transmission buffer array */
	int trans_idx;
	/** brief count of buffers is not completed */
	atomic_t trans_buf_cnt;
	/** brief count of buffers is completed */
	atomic_t done_buf_cnt;

	/** @brief timer handler */
	struct hrtimer timer;
	u64 timer_interval;

	/** @brief spin lock for timer count */
	spinlock_t lock_timer;

	/** @brief timestamp timer handler */
	struct hrtimer tstamp_timer;
	u64 tstamp_timer_interval;

	/** @brief crf timer handler */
	struct hrtimer crf_timer;
	u64 crf_timer_interval;

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
	struct timestamp_queue tstamp_que_crf;
	struct crf_queue crf_que;
	struct avtp_queue avtp_que;

	/** @brief timestamp(nsec) */
	unsigned int timestamp;

	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;

	/** @brief AVTP timestampes */
	unsigned int avtp_timestamps[CREATE_AVTP_TIMESTAMPS_MAX];
	int avtp_timestamps_size;
	int avtp_timestamps_current;
	/** @brief stopping streaming flag */
	bool f_stopping;
	/** @brief continue streaming flag */
	bool f_continue;
	bool f_depacketizing;
	bool f_completion;
	bool f_trans_start;
	bool f_work_timestamp;

	/** @brief streaming flag */
	bool f_streaming;
	/** @brief spin lock for streaming flag */
	rwlock_t lock_stream;

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
	bool f_get_first_packet;
	unsigned int first_avtp_timestamp;

	/** @brief packet buffer */
	int crf_index_network;
	struct mse_packet_ctrl *crf_packet_buffer;
	bool f_crf_sending;

	/** @brief media clcok recovery work */
	struct ptp_clock_time timestamps[PTP_TIMESTAMPS_MAX];
	unsigned int master_timestamps[AVTP_TIMESTAMPS_MAX];
	unsigned int device_timestamps[AVTP_TIMESTAMPS_MAX];

	/** @brief mpeg2ts buffer  */
	u64 mpeg2ts_pcr_90k;
	u64 mpeg2ts_clock_90k;
	int mpeg2ts_pre_pcr_pid;
	u64 mpeg2ts_pre_pcr_90k;
	bool f_force_flush;
	struct mse_trans_buffer mpeg2ts_buffer[MSE_MPEG2TS_BUF_NUM];
	u8 *mpeg2ts_buffer_base;
	int mpeg2ts_buffer_idx;

	/** @brief audio buffer  */
	int temp_w;
	int temp_r;
	unsigned char temp_buffer[MSE_DECODE_BUFFER_NUM][MAX_DECODE_SIZE];
	size_t temp_len[MSE_DECODE_BUFFER_NUM];

	/** @brief debug */
	size_t processed;
};

static int mse_instance_max = MSE_INSTANCE_MAX;
DEFINE_SPINLOCK(packetizer_crf_lock);

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

/*
 * state machine related functions
 */
#define mse_debug_state(instance) __mse_debug_state(__func__, instance)

#if (!MSE_DEBUG_STATE)
static inline void __mse_debug_state(const char *func,
				     struct mse_instance *instance) { ; }
#else
static inline char *mse_state_stringfy(enum MSE_STATE state)
{
	switch (state) {
	case MSE_STATE_CLOSE:
		return "MSE_STATE_CLOSE";

	case MSE_STATE_OPEN:
		return "MSE_STATE_OPEN";

	case MSE_STATE_IDLE:
		return "MSE_STATE_IDLE";

	case MSE_STATE_EXECUTE:
		return "MSE_STATE_EXECUTE";

	case MSE_STATE_STOPPING:
		return "MSE_STATE_STOPPING";

	default:
		return "undefined";
	}
}

static inline void __mse_debug_state(const char *func,
				     struct mse_instance *instance)
{
	pr_debug("%s: state=%s flags=[%d %d %d %d %d %d]\n",
		 func, mse_state_stringfy(instance->state),
		 instance->used_f, instance->f_trans_start,
		 instance->f_continue, instance->f_depacketizing,
		 instance->f_stopping, instance->f_completion);
}
#endif

/* @brief Get MSE state from instance without rwlock */
static inline int mse_state_get_nolock(struct mse_instance *instance)
{
	return instance->state;
}

/* @brief Get MSE state from instance with rwlock */
static inline int mse_state_get(struct mse_instance *instance)
{
	unsigned long flags;
	enum MSE_STATE state;

	read_lock_irqsave(&instance->lock_state, flags);

	state = instance->state;

	read_unlock_irqrestore(&instance->lock_state, flags);

	return state;
}

/* @brief Test MSE state was contained from 'state' without rwlock */
static inline bool mse_state_test_nolock(struct mse_instance *instance,
					 enum MSE_STATE state)
{
	return !!(mse_state_get_nolock(instance) & state);
}

/* @brief Test MSE state was contained from 'state' with rwlock */
static inline bool mse_state_test(struct mse_instance *instance,
				  enum MSE_STATE state)
{
	return !!(mse_state_get(instance) & state);
}

#define mse_state_change(instance, next) \
	__mse_state_change(__func__, instance, next)

static int __mse_state_change(const char *func,
			      struct mse_instance *instance,
			      enum MSE_STATE next)
{
	int err = 0;
	int index_media = instance->index_media;
	enum MSE_STATE state;

	/**
	 *           | CLOSE  OPEN   IDLE    EXECUTE STOPPING
	 * ----------|----------------------------------------
	 *  CLOSE    | Y      Y      EPERM   EPERM   EPERM
	 *  OPEN     | Y      Y      Y       EPERM   EPERM
	 *  IDLE     | EBUSY  Y      Y       Y       Y
	 *  EXECUTE  | EBUSY  EBUSY  Y       Y       Y
	 *  STOPPING | EBUSY  Y      EBUSY   EBUSY   Y
	 */
	state = mse_state_get_nolock(instance);
	switch (state) {
	case MSE_STATE_CLOSE:
		if (next & (MSE_STATE_CLOSE |
			    MSE_STATE_OPEN)) {
			; /* do nothing */
		} else {
			err = -EPERM;
		}
		break;

	case MSE_STATE_OPEN:
		if (next & (MSE_STATE_CLOSE |
			    MSE_STATE_OPEN  |
			    MSE_STATE_IDLE)) {
			; /* do nothing */
		} else {
			err = -EPERM;
		}
		break;

	case MSE_STATE_IDLE:
		if (next & (MSE_STATE_OPEN    |
			    MSE_STATE_IDLE    |
			    MSE_STATE_EXECUTE |
			    MSE_STATE_STOPPING)) {
			; /* do nothing */
		} else if (next & MSE_STATE_CLOSE) {
			err = -EBUSY;
		} else {
			err = -EPERM;
		}
		break;

	case MSE_STATE_EXECUTE:
		if (next & (MSE_STATE_IDLE    |
			    MSE_STATE_EXECUTE |
			    MSE_STATE_STOPPING)) {
			; /* do nothing */
		} else if (next & (MSE_STATE_CLOSE |
				   MSE_STATE_OPEN)) {
			err = -EBUSY;
		} else {
			err = -EPERM;
		}
		break;

	case MSE_STATE_STOPPING:
		if (next & (MSE_STATE_OPEN |
			    MSE_STATE_STOPPING)) {
			; /* do nothing */
		} else if (next & (MSE_STATE_CLOSE |
				   MSE_STATE_IDLE  |
				   MSE_STATE_EXECUTE)) {
			err = -EBUSY;
		} else {
			err = -EPERM;
		}
		break;

	default:
		err = -EPERM;
		break;
	}

	if (!err)
		instance->state = next;
	else if (err == -EPERM)
		pr_err("%s: index=%d: operation is not permitted\n",
		       func, index_media);
	else if (err == -EBUSY)
		pr_err("%s: index=%d: instance is busy\n", func, index_media);
	else
		pr_err("%s: index=%d: unknown error\n", func, index_media);

#if (MSE_DEBUG_STATE)
	pr_debug("%s: index=%d state=%s->%s %s\n",
		 func,
		 index_media,
		 (err) ? "failure" : "success",
		 mse_state_stringfy(state),
		 mse_state_stringfy(next));
#endif

	WARN_ON(err);

	return err;
}

#define mse_state_change_if(instance, next, test_state) \
	__mse_state_change_if(__func__, instance, next, test_state)

static int __mse_state_change_if(const char *func,
				 struct mse_instance *instance,
				 enum MSE_STATE next,
				 int test_state)
{
	enum MSE_STATE state;

	state = mse_state_get_nolock(instance);
	if (!(state & test_state))
		return 0;

	return __mse_state_change(func, instance, next);
}

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
		err = mse_config_get_delay_time(index,
						&delay_time);
		instance->max_transit_time = delay_time.max_transit_time_ns;
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
		err = mse_config_get_delay_time(index,
						&delay_time);
		instance->max_transit_time = delay_time.max_transit_time_ns;
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

static void callback_completion(struct mse_trans_buffer *buf, int size)
{
	mse_debug("buf=%p media_buffer=%p buffer=%p buffer_size=%zu callback=%p private=%p size=%d\n",
		  buf, buf->media_buffer, buf->buffer, buf->buffer_size,
		  buf->mse_completion, buf->private_data, size);

	list_del(&buf->list);

	if (buf->mse_completion)
		buf->mse_completion(buf->private_data, size);

	buf->buffer_size = 0;
	buf->work_length = 0;
	buf->media_buffer = NULL;
	buf->private_data = NULL;
	buf->mse_completion = NULL;
}

static void mse_trans_complete(struct mse_instance *instance, int size)
{
	struct mse_trans_buffer *buf;

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);
	if (buf) {
		if (size > 0)
			instance->processed += buf->work_length;

		mse_debug("total processed=%zu\n", instance->processed);
		atomic_dec(&instance->trans_buf_cnt);
		callback_completion(buf, size);
	}
}

static void mse_free_all_trans_buffers(struct mse_instance *instance, int size)
{
	struct mse_trans_buffer *buf, *buf1;

	/* free all buf from DONE buf list */
	list_for_each_entry_safe(buf, buf1, &instance->proc_buf_list, list)
		callback_completion(buf, size);
	atomic_set(&instance->done_buf_cnt, 0);

	/* free all buf from TRANS buf list */
	list_for_each_entry_safe(buf, buf1, &instance->trans_buf_list, list)
		callback_completion(buf, size);
	atomic_set(&instance->trans_buf_cnt, 0);
}

static void mse_work_stream(struct work_struct *work)
{
	struct mse_instance *instance;
	int index_network;
	struct mse_packet_ctrl *dma;
	struct mse_adapter_network_ops *network;
	int err = 0;
	unsigned long flags;

	instance = container_of(work, struct mse_instance, wk_stream);

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED))
		return; /* skip work */

	mse_debug("START\n");
	mse_debug_state(instance);

	write_lock_irqsave(&instance->lock_stream, flags);
	instance->f_streaming = true;
	write_unlock_irqrestore(&instance->lock_stream, flags);

	index_network = instance->index_network;
	dma = instance->packet_buffer;
	network = instance->network;

	if (instance->tx) {
		/* while data is remained */
		do {
			/* request send packet */
			err = mse_packet_ctrl_send_packet(index_network, dma,
							  network);

			if (err < 0) {
				mse_err("send error %d\n", err);
				break;
			}

			wake_up_interruptible(&instance->wait_wk_stream);
		} while (mse_packet_ctrl_check_packet_remain(dma));
	} else {
		/* while state is RUNNABLE */
		while (mse_state_test(instance, MSE_STATE_RUNNABLE)) {
			/* request receive packet */
			err = mse_packet_ctrl_receive_packet(
				index_network,
				MSE_RX_PACKET_NUM,
				dma, network);

			if (err < 0) {
				mse_err("receive error %d\n", err);
				break;
			}

			/* if NOT work queued, then queue it */
			if (!work_busy(&instance->wk_depacketize))
				queue_work(instance->wq_packet,
					   &instance->wk_depacketize);
		}
	}

	write_lock_irqsave(&instance->lock_stream, flags);
	instance->f_streaming = false;
	mse_debug_state(instance);
	write_unlock_irqrestore(&instance->lock_stream, flags);

	mse_debug("END\n");
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
		t_d = t - avtp_time;
		if (t_d < UINT_MAX / 2)
			break;
	}

	if (p == que->tail || p == que->head)
		return -1;
	*std_time = que->std_times[p];
	if (t_d < NSEC_SCALE)
		*std_time -= t_d;
	else
		mse_info("not precision offset capture %u", t_d);

#if (MSE_DEBUG_TSTAMPS)
	mse_debug("found %lu t= %u avtp= %u\n", *std_time, t, avtp_time);
#endif

	return 0;
}

static int tstamps_enq_tstamps(struct timestamp_queue *que,
			       unsigned long std_time,
			       struct ptp_clock_time *clock_time)
{
#if (MSE_DEBUG_TSTAMPS)
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);
#endif

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
#if (MSE_DEBUG_TSTAMPS)
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);
#endif

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
#if (MSE_DEBUG_TSTAMPS)
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);
#endif

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

static int media_clock_recovery_avtp(struct mse_instance *instance,
				     unsigned int d_t,
				     int *count_out)
{
	unsigned long flags;
	int i, count, out;
	unsigned int avtp_time;
	unsigned int search_time;
	unsigned long device_time;
	unsigned long std_time;
	int ret;

	spin_lock_irqsave(&instance->lock_ques, flags);

	out = 0;
	count = tstamps_get_avtp_size(&instance->avtp_que);
	for (i = 0; i < count; i++) {
		ret = tstamps_deq_avtp(&instance->avtp_que, &std_time,
				       &avtp_time);
		if (ret < 0)
			continue;

		if (instance->f_match_ptp_clock) {
			instance->mch_std_time += d_t;
		} else {
			search_time = avtp_time;

			if (instance->ptp_clock)
				search_time -= instance->listener_delay_time;

			ret = tstamps_search_tstamp(
				&instance->tstamp_que,
				&instance->mch_std_time,
				search_time);
			if (ret < 0)
				continue;

			instance->f_match_ptp_clock = true;
		}

		ret = tstamps_calc_tstamp(&instance->tstamp_que,
					  instance->mch_std_time,
					  &device_time);
		if (ret < 0)
			continue;

		if (instance->ptp_clock)
			device_time += instance->listener_delay_time;

		instance->master_timestamps[out] = avtp_time;
		instance->device_timestamps[out] = device_time;
		out++;

		if (out >= ARRAY_SIZE(instance->master_timestamps))
			break;
	}

	spin_unlock_irqrestore(&instance->lock_ques, flags);

	*count_out = out;

	return ret;
}

static int media_clock_recovery_crf(struct mse_instance *instance,
				    unsigned int d_t,
				    int *count_out)
{
	unsigned long flags;
	int i, count, out;
	unsigned int search_time;
	unsigned long device_time;
	unsigned long std_time;
	u64 crf_time;
	int ret;

	spin_lock_irqsave(&instance->lock_ques, flags);

	out = 0;
	count = tstamps_get_crf_size(&instance->crf_que);
	for (i = 0; i < count; i++) {
		ret = tstamps_deq_crf(&instance->crf_que, &std_time,
				      &crf_time);
		if (ret < 0)
			continue;

		if (instance->f_match_ptp_clock) {
			instance->mch_std_time += d_t;
		} else {
			search_time = crf_time - instance->max_transit_time;

			if (instance->ptp_clock)
				search_time -= instance->talker_delay_time;

			ret = tstamps_search_tstamp(
				&instance->tstamp_que,
				&instance->mch_std_time,
				search_time);
			if (ret < 0)
				continue;

			instance->f_match_ptp_clock = true;
		}

		ret = tstamps_calc_tstamp(&instance->tstamp_que,
					  instance->mch_std_time,
					  &device_time);
		if (ret < 0)
			continue;

		device_time += instance->max_transit_time;
		if (instance->ptp_clock)
			device_time += instance->talker_delay_time;

		instance->master_timestamps[out] = (unsigned int)crf_time;
		instance->device_timestamps[out] = device_time;
		out++;

		if (out >= ARRAY_SIZE(instance->master_timestamps))
			break;
	}

	spin_unlock_irqrestore(&instance->lock_ques, flags);

	*count_out = out;

	return ret;
}

static int media_clock_recovery(struct mse_instance *instance)
{
	struct mch_ops *m_ops;
	int ret, out;
	unsigned int d_t;

	if (instance->crf_type != MSE_CRF_TYPE_RX) {
		d_t = instance->audio_info.frame_interval_time;
		ret = media_clock_recovery_avtp(instance, d_t, &out);
	} else {
		d_t = instance->crf_audio_info.frame_interval_time;
		ret = media_clock_recovery_crf(instance, d_t, &out);
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

	/* state is NOT RUNNABLE */
	if (!mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		mse_debug_state(instance);
		instance->f_work_timestamp = false;
		return;
	}

	/* capture timestamps */
	if (instance->ptp_clock == 1) {
		unsigned long flags;

		/* get timestamps */
		count = ARRAY_SIZE(instance->timestamps);
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
			tstamps_enq_tstamps(&instance->tstamp_que_crf,
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

#if (MSE_DEBUG_TSTAMPS)
	mse_debug("total=%d\n", count);
#endif

	for (i = 0; i < count; i++) {
		tstamps_deq_tstamps(que, &std_time, &clock_time);
		timestamps[i] = clock_time;
	}

	return 0;
}

static int tstamps_store_ptp_timestamp(struct mse_instance *instance,
				       struct ptp_clock_time *now)
{
	unsigned long flags;

	spin_lock_irqsave(&instance->lock_ques, flags);

	tstamps_enq_tstamps(&instance->tstamp_que,
			    instance->std_time_counter, now);
	tstamps_enq_tstamps(&instance->tstamp_que_crf,
			    instance->std_time_counter, now);

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

	offset = instance->max_transit_time;
	if (instance->ptp_clock)
		offset += instance->talker_delay_time;

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
	unsigned long flags;

	spin_lock_irqsave(&packetizer_crf_lock, flags);
	ret = instance->crf_index = crf->open();
	spin_unlock_irqrestore(&packetizer_crf_lock, flags);
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

	if (instance->crf_type == MSE_CRF_TYPE_RX) {
		ret = instance->network->set_streamid(
					instance->crf_index_network,
					instance->crf_net_config.streamid);
		if (ret < 0)
			goto error_set_streamid_fail;
	}

	return 0;

error_set_streamid_fail:
error_set_cbs_param_fail:
error_calc_cbs_fail:
error_set_audio_config_fail:
	spin_lock_irqsave(&packetizer_crf_lock, flags);
	crf->release(instance->crf_index);
	instance->crf_index = MSE_INDEX_UNDEFINED;
	spin_unlock_irqrestore(&packetizer_crf_lock, flags);

	return ret;
}

static void mse_release_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;
	unsigned long flags;

	if (instance->crf_index >= 0) {
		spin_lock_irqsave(&packetizer_crf_lock, flags);
		crf->release(instance->crf_index);
		spin_unlock_irqrestore(&packetizer_crf_lock, flags);
	}
}

static bool check_packet_remain(struct mse_instance *instance)
{
	int wait_count = MSE_TX_PACKET_NUM / 2;
	int rem = mse_packet_ctrl_check_packet_remain(instance->packet_buffer);

	return wait_count > rem;
}

static void mse_work_packetize(struct work_struct *work)
{
	struct mse_instance *instance;
	int ret = 0;
	int trans_size;
	unsigned long flags;
	struct mse_trans_buffer *buf;

	instance = container_of(work, struct mse_instance, wk_packetize);
	mse_debug_state(instance);

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return;

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);

	if (!buf)
		return; /* skip work */

	trans_size = buf->buffer_size - buf->work_length;
	mse_debug("trans size=%d buffer=%p buffer_size=%zu\n",
		  trans_size, buf->buffer, buf->buffer_size);

	if (!(trans_size > 0)) {
		/* no data to process */
		instance->f_force_flush = false;
		atomic_inc(&instance->done_buf_cnt);

		if (!instance->timer_interval)
			queue_work(instance->wq_packet, &instance->wk_callback);

		return;
	}

	/* make AVTP packet with one timestamp */
	if (!IS_MSE_TYPE_AUDIO(instance->media->type)) {
		instance->avtp_timestamps_current = 0;
		instance->avtp_timestamps_size = 1;
		instance->avtp_timestamps[0] =
			instance->timestamp + instance->max_transit_time;
	}

	while (buf->work_length < buf->buffer_size) {
		/* state is EXECUTE */
		if (mse_state_test(instance, MSE_STATE_EXECUTE))
			/* wait for packet buffer processed */
			wait_event_interruptible(instance->wait_wk_stream,
						 check_packet_remain(instance));

		ret = mse_packet_ctrl_make_packet(
					instance->index_packetizer,
					buf->buffer,
					buf->buffer_size,
					instance->ptp_clock,
					&instance->avtp_timestamps_current,
					instance->avtp_timestamps_size,
					instance->avtp_timestamps,
					instance->packet_buffer,
					instance->packetizer,
					&buf->work_length);

		if (ret < 0)
			break;

		/* start workqueue for streaming */
		read_lock_irqsave(&instance->lock_stream, flags);
		if (!instance->f_streaming && ret > 0)
			queue_work(instance->wq_stream, &instance->wk_stream);
		read_unlock_irqrestore(&instance->lock_stream, flags);
	}

	mse_debug("packetized(ret)=%d len=%zu\n", ret, buf->work_length);

	instance->f_continue = buf->work_length < buf->buffer_size;

	if (ret == -EAGAIN) {
		instance->f_continue = false;
		if (!atomic_read(&instance->trans_buf_cnt)) {
			instance->f_force_flush = false;

			/* state is STOPPING */
			if (mse_state_test(instance, MSE_STATE_STOPPING)) {
				mse_debug("discard %zu byte before stopping\n",
					  buf->buffer_size - buf->work_length);

				instance->f_trans_start = false;
				queue_work(instance->wq_packet,
					   &instance->wk_stop_streaming);
			} else {
				mse_err("short of data\n");

				write_lock_irqsave(&instance->lock_state,
						   flags);
				/* if state is EXECUTE, change to IDLE */
				mse_state_change_if(instance, MSE_STATE_IDLE,
						    MSE_STATE_EXECUTE);
				write_unlock_irqrestore(&instance->lock_state,
							flags);
			}

			mse_trans_complete(instance, ret);

			return;
		}
	} else if (ret < 0) {
		mse_err("error=%d buffer may be corrupted\n", ret);
		instance->f_trans_start = false;
		instance->f_continue = false;
		instance->f_force_flush = false;

		mse_trans_complete(instance, ret);

		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
		} else {
			write_lock_irqsave(&instance->lock_state, flags);
			/* if state is EXECUTE, change to IDLE */
			mse_state_change_if(instance, MSE_STATE_IDLE,
					    MSE_STATE_EXECUTE);
			write_unlock_irqrestore(&instance->lock_state, flags);
		}

		return;
	}

	if (instance->f_continue) {
		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		instance->f_force_flush = false;
		if (ret != -EAGAIN)
			atomic_inc(&instance->done_buf_cnt);

		if (!instance->timer_interval)
			queue_work(instance->wq_packet, &instance->wk_callback);
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
		t_d = instance->first_avtp_timestamp - t;
		if (t_d < UINT_MAX / 2 &&
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
	int received, ret = 0;
	unsigned int timestamps[128];
	int t_stored, i;
	unsigned int d_t;
	struct mse_audio_config *audio;
	struct mse_packet_ctrl *dma;
	unsigned long flags;
	struct mse_trans_buffer *buf;
	int buf_cnt;

	instance = container_of(work, struct mse_instance, wk_depacketize);

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED))
		return; /* skip work */

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);

	/* no buffer to write data */
	if (!buf) {
		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);

		return;
	}

	read_lock_irqsave(&instance->lock_stream, flags);
	if (!instance->f_streaming)
		queue_work(instance->wq_stream, &instance->wk_stream);
	read_unlock_irqrestore(&instance->lock_stream, flags);
	instance->f_depacketizing = true;

	dma = instance->packet_buffer;
	received = mse_packet_ctrl_check_packet_remain(dma);

	switch (instance->media->type) {
	case MSE_TYPE_ADAPTER_AUDIO:
		/* get AVTP packet payload */
		audio = &instance->media_config.audio;
		while (received) {
			/* get AVTP packet payload */
			ret = mse_packet_ctrl_take_out_packet(
				instance->index_packetizer,
				instance->temp_buffer[instance->temp_w],
				buf->buffer_size,
				timestamps,
				ARRAY_SIZE(timestamps),
				&t_stored,
				dma,
				instance->packetizer,
				&instance->temp_len[instance->temp_w]);

			if (ret < 0) {
				if (ret != -EAGAIN) {
					instance->f_trans_start = false;
					mse_trans_complete(instance, ret);
					break;
				}
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
			if (!instance->f_get_first_packet && t_stored > 0) {
				instance->first_avtp_timestamp = timestamps[0];
				instance->f_get_first_packet = true;
				instance->processed = 0;
			}

			if (instance->temp_len[instance->temp_w] >=
			    buf->buffer_size) {
				mse_debug("buffer=%p depacketized=%zu ret=%d\n",
					  buf->buffer,
					  instance->temp_len[instance->temp_w],
					  ret);

				instance->temp_w = (instance->temp_w + 1) %
					MSE_DECODE_BUFFER_NUM;
				atomic_inc(&instance->done_buf_cnt);
				memset(instance->temp_buffer[instance->temp_w],
				       0, buf->buffer_size);
				instance->temp_len[instance->temp_w] = 0;

				mse_debug("temp_r=%d temp_w=%d\n",
					  instance->temp_r, instance->temp_w);
			}

			/* update received count */
			received = mse_packet_ctrl_check_packet_remain(dma);

			/* state is STOPPING */
			if (!received &&
			    mse_state_test(instance, MSE_STATE_STOPPING))
				break;
		}
		break;

	case MSE_TYPE_ADAPTER_VIDEO:
	case MSE_TYPE_ADAPTER_MPEG2TS:
		/* get AVTP packet payload */
		ret = mse_packet_ctrl_take_out_packet(
						instance->index_packetizer,
						buf->buffer,
						buf->buffer_size,
						timestamps,
						ARRAY_SIZE(timestamps),
						&t_stored,
						dma,
						instance->packetizer,
						&buf->work_length);

		/* complete callback */
		if (ret > 0) {
			instance->f_trans_start = false;
			buf_cnt = atomic_inc_return(&instance->done_buf_cnt);

			/* if timer enabled, done_buf_cnt keep in one */
			if (instance->timer_interval && buf_cnt != 1)
				atomic_dec(&instance->done_buf_cnt);

			mse_debug("buffer=%p depacketized=%zu ret=%d\n",
				  buf->buffer,
				  buf->work_length, ret);
		} else if (ret != -EAGAIN) {
			instance->f_trans_start = false;
			mse_trans_complete(instance, ret);
		}

		break;

	default:
		mse_err("unknown type=0x%08x\n", instance->media->type);
		break;
	}

	if (!instance->timer_interval)
		queue_work(instance->wq_packet, &instance->wk_callback);

	/* state is STOPPING */
	if (mse_state_test(instance, MSE_STATE_STOPPING))
		if (atomic_inc_return(&instance->done_buf_cnt) != 1)
			atomic_dec(&instance->done_buf_cnt);

	/* state is NOT EXECUTE */
	if (!mse_state_test(instance, MSE_STATE_EXECUTE))
		instance->f_depacketizing = false;
}

static void mse_work_callback(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int size;
	unsigned long flags;
	int temp_r;
	struct mse_trans_buffer *buf;

	instance = container_of(work, struct mse_instance, wk_callback);

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	adapter = instance->media;

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);

	/* no buffer to callback */
	if (!buf)
		return; /* skip work */

	/* for capture with timer_interval */
	if (instance->timer_interval && !instance->tx)
		if (atomic_inc_return(&instance->done_buf_cnt) != 1)
			atomic_dec(&instance->done_buf_cnt);

	/* no buffer is processed */
	if (!atomic_read(&instance->done_buf_cnt))
		return;

	mse_debug("START\n");
	mse_debug_state(instance);

	size = buf->work_length;

	if (instance->tx) {
		if (IS_MSE_TYPE_MPEG2TS(adapter->type)) {
			u64 clock_90k = instance->mpeg2ts_clock_90k;
			u64 pcr_90k = instance->mpeg2ts_pcr_90k;

			if (instance->f_force_flush) {
				queue_work(instance->wq_packet,
					&instance->wk_packetize);
				return;
			}

			/* state is NOT STOPPING */
			if (!mse_state_test(instance, MSE_STATE_STOPPING)) {
				if (clock_90k != MPEG2TS_PCR90K_INVALID) {
					mse_debug("mpeg2ts_clock_90k time=%llu pcr=%llu\n",
						  clock_90k, pcr_90k);

					if (compare_pcr(pcr_90k, clock_90k))
						return;
				}
			}
		}
	} else {
		if (!IS_MSE_TYPE_AUDIO(adapter->type)) {
			size = buf->work_length;
		} else {
			size = buf->buffer_size;
			temp_r = instance->temp_r;
			if (buf->media_buffer &&
			    check_presentation_time(instance)) {
				memcpy(buf->media_buffer,
				       instance->temp_buffer[temp_r],
				       size);
				if (instance->temp_w != temp_r) {
					instance->temp_r = (temp_r + 1) %
						MSE_DECODE_BUFFER_NUM;
				} else {
					memset(instance->temp_buffer[temp_r],
					       0, instance->temp_len[temp_r]);
					instance->temp_len[temp_r] = 0;
				}

				mse_debug("temp_r=%d temp_w=%d\n",
					  instance->temp_r,
					  instance->temp_w);
			}
		}

		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
	}

	/* complete callback */
	instance->f_trans_start = false;
	if (instance->tx || size)
		if (atomic_dec_not_zero(&instance->done_buf_cnt))
			mse_trans_complete(instance, size);

	if (!atomic_read(&instance->trans_buf_cnt)) {
		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
		} else {
			write_lock_irqsave(&instance->lock_state, flags);
			/* if state is EXECUTE, change to IDLE */
			mse_state_change_if(instance, MSE_STATE_IDLE,
					    MSE_STATE_EXECUTE);
			write_unlock_irqrestore(&instance->lock_state, flags);
		}
	} else {
		/* if NOT work queued, then queue it */
		if (instance->tx && !work_busy(&instance->wk_packetize))
			queue_work(instance->wq_packet,
				   &instance->wk_packetize);

		if (!instance->tx && !work_busy(&instance->wk_depacketize))
			queue_work(instance->wq_packet,
				   &instance->wk_depacketize);
	}
}

static void mse_stop_streaming_audio(struct mse_instance *instance)
{
	int ret;
	struct mch_ops *m_ops;
	enum MSE_CRF_TYPE crf_type = instance->crf_type;

	/* cancel timestamp timer */
	hrtimer_cancel(&instance->tstamp_timer);

	/* cancel crf timer */
	hrtimer_cancel(&instance->crf_timer);

	if (instance->mch_index >= 0) {
		m_ops = mse->mch_table[instance->mch_index];
		ret = m_ops->close(instance->mch_dev_id);
		if (ret < 0)
			mse_err("mch close error(%d).\n", ret);
	}

	if (crf_type == MSE_CRF_TYPE_RX) {
		ret = instance->network->cancel(
			instance->crf_index_network);
		if (ret)
			mse_err("failed cancel() ret=%d\n", ret);

		flush_work(&instance->wk_crf_receive);
	} else if (crf_type == MSE_CRF_TYPE_TX &&
		   instance->f_crf_sending) {
		flush_work(&instance->wk_crf_send);
	}

	if (instance->f_work_timestamp)
		flush_work(&instance->wk_timestamp);
}

static void mse_stop_streaming_common(struct mse_instance *instance)
{
	int ret;
	unsigned long flags;

	write_lock_irqsave(&instance->lock_state, flags);
	ret = mse_state_change(instance, MSE_STATE_OPEN);
	write_unlock_irqrestore(&instance->lock_state, flags);
	if (ret)
		return;

	if (!instance->tx) {
		ret = instance->network->cancel(instance->index_network);
		if (ret)
			mse_err("failed network adapter cancel() ret=%d\n",
				ret);
	}

	/* cancel timer */
	hrtimer_cancel(&instance->timer);

	/* return callback to all transmission request */
	mse_free_all_trans_buffers(instance, 0);

	/* timestamp timer, crf timer stop */
	if (IS_MSE_TYPE_AUDIO(instance->media->type))
		mse_stop_streaming_audio(instance);

	instance->f_completion = true;
	instance->f_stopping = false;

	complete(&instance->completion_stop);
}

static void mse_work_stop_streaming(struct work_struct *work)
{
	int ret;
	struct mse_instance *instance;
	struct mse_adapter_network_ops *network;
	unsigned long flags;

	instance = container_of(work, struct mse_instance, wk_stop_streaming);
	network = instance->network;

	mse_debug_state(instance);

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED))
		return; /* skip work */

	instance->f_stopping = true;

	/* state is NOT EXECUTE */
	if (!mse_state_test(instance, MSE_STATE_EXECUTE)) {
		mse_stop_streaming_common(instance);
		return;
	}

	/*
	 * If state is EXECUTE, state change to STOPPING.
	 * then wait complete streaming process.
	 */
	write_lock_irqsave(&instance->lock_state, flags);
	mse_state_change(instance, MSE_STATE_STOPPING);
	write_unlock_irqrestore(&instance->lock_state, flags);

	if (instance->tx) {
		queue_work(instance->wq_packet, &instance->wk_start_trans);
	} else {
		ret = network->cancel(instance->index_network);
		if (ret)
			mse_err("failed network adapter cancel() => %d\n", ret);

		queue_work(instance->wq_packet, &instance->wk_depacketize);
	}
}

static enum hrtimer_restart mse_timer_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;

	instance = container_of(arg, struct mse_instance, timer);
	adapter = instance->media;

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	if (instance->mpeg2ts_clock_90k != MPEG2TS_PCR90K_INVALID)
		instance->mpeg2ts_clock_90k += MPEG2TS_CLOCK_INC;

	/* timer update */
	hrtimer_add_expires_ns(&instance->timer, instance->timer_interval);

	/* start workqueue for completion */
	queue_work(instance->wq_packet, &instance->wk_callback);

	return HRTIMER_RESTART;
}

static void mse_work_crf_send(struct work_struct *work)
{
	struct mse_instance *instance;
	int err, tsize, size, i;
	struct ptp_clock_time timestamps[6];
	unsigned long flags;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_crf_send);

	/* state is NOT RUNNABLE */
	if (!mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		instance->f_crf_sending = false;
		return;
	}

	tsize = instance->ptp_clock == 0 ?
		CRF_PTP_TIMESTAMPS : CRF_AUDIO_TIMESTAMPS;
	spin_lock_irqsave(&instance->lock_ques, flags);
	size = tstamps_get_tstamps_size(&instance->tstamp_que_crf);
	spin_unlock_irqrestore(&instance->lock_ques, flags);

	while (size >= tsize) {
		/* get Timestamps */
		mse_debug("size %d tsize %d\n", size, tsize);
		spin_lock_irqsave(&instance->lock_ques, flags);
		get_timestamps(&instance->tstamp_que_crf, tsize, timestamps);
		spin_unlock_irqrestore(&instance->lock_ques, flags);
		for (i = 0; i < tsize; i++) {
			u64 t;

			t = timestamps[i].sec * NSEC_SCALE + timestamps[i].nsec;
			if (instance->tx)
				t += instance->max_transit_time;
			if (instance->ptp_clock) {
				if (instance->tx)
					t += instance->talker_delay_time;
				else
					t += instance->listener_delay_time;
			}
			timestamps[i].sec = div_s64_rem(t, NSEC_SCALE,
							&timestamps[i].nsec);
		}

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

		/* state is NOT RUNNABLE */
		if (!mse_state_test(instance, MSE_STATE_RUNNABLE))
			break;

		spin_lock_irqsave(&instance->lock_ques, flags);
		size = tstamps_get_tstamps_size(&instance->tstamp_que_crf);
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	}

	instance->f_crf_sending = false;
}

static void mse_work_crf_receive(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int err, count, i;
	u64 ptimes[6];
	unsigned long flags;

	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;

	instance = container_of(work, struct mse_instance, wk_crf_receive);
	adapter = instance->media;

	mse_debug("START\n");

	do {
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

	/* while state is RUNNABLE */
	} while (mse_state_test(instance, MSE_STATE_RUNNABLE));
}

static enum hrtimer_restart mse_crf_callback(struct hrtimer *arg)
{
	struct mse_instance *instance;

	instance = container_of(arg, struct mse_instance, crf_timer);

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	/* timer update */
	hrtimer_add_expires_ns(&instance->crf_timer,
			       instance->crf_timer_interval);

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

	instance = container_of(arg, struct mse_instance, tstamp_timer);

	mse_debug("START\n");

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		return HRTIMER_NORESTART;
	}

	/* timer update */
	hrtimer_add_expires_ns(&instance->tstamp_timer,
			       instance->tstamp_timer_interval);

	if (instance->f_work_timestamp)
		return HRTIMER_RESTART;

	instance->f_work_timestamp = true;
	queue_work(instance->wq_tstamp, &instance->wk_timestamp);

	return HRTIMER_RESTART;
}

static s64 calc_diff_ptp_clock_time(struct ptp_clock_time *a,
				    struct ptp_clock_time *b)
{
	s64 a_nsec, b_nsec;

	a_nsec = a->sec * NSEC_SCALE + a->nsec;
	b_nsec = b->sec * NSEC_SCALE + b->nsec;

	return a_nsec - b_nsec;
}

static void mse_get_capture_timestamp_first(struct mse_instance *instance,
					    struct ptp_clock_time *now)
{
	int ret = 0;
	unsigned long flags;
	int count, i;
	s64 delay, diff;

	/* get timestamps */
	count = ARRAY_SIZE(instance->timestamps);
	ret = mse_ptp_get_timestamps(instance->ptp_index,
				     instance->ptp_dev_id,
				     instance->ptp_clock_ch,
				     &count,
				     instance->timestamps);
	if (ret) {
		mse_warn("could not get timestamps ret=%d\n", ret);
		return;
	}

	/* store timestamps */
	if (instance->tx)
		delay = instance->talker_delay_time;
	else
		delay = instance->listener_delay_time;

	spin_lock_irqsave(&instance->lock_ques, flags);

	diff = 0;
	/* skip older timestamp */
	for (i = 0; i < count; i++) {
		diff = calc_diff_ptp_clock_time(now, &instance->timestamps[i]);
		diff -= delay;
		if (diff <= 0)
			break;
		instance->std_time_avtp = diff;
	}
	if (i != 0) {
		i--;
	} else {
		/* not enough timestamps */
		if (instance->tx)
			instance->talker_delay_time += diff;
		else
			instance->listener_delay_time += diff;
	}

	for (; i < count; i++) {
		tstamps_enq_tstamps(&instance->tstamp_que,
				    instance->std_time_counter,
				    &instance->timestamps[i]);
		tstamps_enq_tstamps(&instance->tstamp_que_crf,
				    instance->std_time_counter,
				    &instance->timestamps[i]);
		instance->std_time_counter += instance->add_std_time;
	}

	spin_unlock_irqrestore(&instance->lock_ques, flags);
}

static void mse_start_streaming_audio(struct mse_instance *instance,
				      struct ptp_clock_time *now)
{
	int i;

	if (!instance->tx) {
		instance->temp_w = 0;
		instance->temp_r = 0;
		for (i = 0; i < MSE_DECODE_BUFFER_NUM; i++)
			instance->temp_len[i] = 0;
	}

	/* ptp_clock is capture, capture timestamps */
	if (instance->ptp_clock == 1)
		mse_get_capture_timestamp_first(instance, now);

	/* ptp_clock is capture or media_clock_recovery is enable */
	if (instance->ptp_clock == 1 ||
	    instance->media_clock_recovery == 1) {
		hrtimer_start(&instance->tstamp_timer,
			      ns_to_ktime(instance->tstamp_timer_interval),
			      HRTIMER_MODE_REL);
	}

	/* send clock using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_TX) {
		hrtimer_start(&instance->crf_timer,
			      ns_to_ktime(instance->crf_timer_interval),
			      HRTIMER_MODE_REL);
	}

	/* receive clcok using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_RX) {
		queue_work(instance->wq_crf_packet,
			   &instance->wk_crf_receive);
	}
}

static void mse_tstamp_init(struct mse_instance *instance,
			    struct ptp_clock_time *now)
{
	instance->timestamp = (unsigned long)now->sec * NSEC_SCALE + now->nsec;

	/* prepare std clock time */
	if (instance->ptp_clock) {
		instance->add_std_time =
			NSEC_SCALE / instance->ptp_capture_freq;
		instance->add_crf_std_time =
			NSEC_SCALE / instance->ptp_capture_freq;
	} else {
		instance->add_std_time = instance->timer_interval;
		instance->add_crf_std_time = instance->timer_interval;
	}

	instance->std_time_counter = 0;
	instance->std_time_avtp = 0;
	instance->std_time_crf = 0;

	instance->f_present = false;
	instance->f_get_first_packet = false;
	instance->remain = 0;
}

static void mse_start_streaming_common(struct mse_instance *instance)
{
	int i;

	reinit_completion(&instance->completion_stop);
	instance->f_streaming = false;
	instance->f_continue = false;
	instance->f_stopping = false;
	instance->f_trans_start = false;
	instance->f_completion = false;
	instance->f_force_flush = false;
	instance->mpeg2ts_clock_90k = MPEG2TS_PCR90K_INVALID;
	instance->mpeg2ts_pre_pcr_pid = MPEG2TS_PCR_PID_IGNORE;
	instance->mpeg2ts_pcr_90k = MPEG2TS_PCR90K_INVALID;
	instance->mpeg2ts_pre_pcr_90k = MPEG2TS_PCR90K_INVALID;
	instance->f_depacketizing = false;
	instance->processed = 0;

	if (instance->tx && IS_MSE_TYPE_MPEG2TS(instance->media->type)) {
		for (i = 0; i < MSE_MPEG2TS_BUF_NUM; i++) {
			instance->mpeg2ts_buffer[i].buffer_size = 0;
			instance->mpeg2ts_buffer[i].work_length = 0;
		}
	}

	/* start timer */
	if (instance->timer_interval) {
		hrtimer_start(&instance->timer,
			      ns_to_ktime(instance->timer_interval),
			      HRTIMER_MODE_REL);
	}
}

static void mpeg2ts_buffer_free(struct mse_instance *instance)
{
	int i;
	struct mse_trans_buffer *mpeg2ts;

	/* NOT allocated, skip */
	if (!instance->mpeg2ts_buffer_base)
		return;

	for (i = 0; i < MSE_MPEG2TS_BUF_NUM; i++) {
		mpeg2ts = &instance->mpeg2ts_buffer[i];
		mpeg2ts->buffer = NULL;
	}

	kfree(instance->mpeg2ts_buffer_base);
	instance->mpeg2ts_buffer_base = NULL;
}

static int mpeg2ts_buffer_alloc(struct mse_instance *instance)
{
	int i;
	unsigned char *buf;
	struct mse_trans_buffer *mpeg2ts;

	buf = kmalloc_array(MSE_MPEG2TS_BUF_SIZE,
			    MSE_MPEG2TS_BUF_NUM, GFP_KERNEL);

	if (!buf)
		return -ENOMEM;

	instance->mpeg2ts_buffer_base = buf;
	instance->mpeg2ts_buffer_idx = 0;
	for (i = 0; i < MSE_MPEG2TS_BUF_NUM; i++) {
		mpeg2ts = &instance->mpeg2ts_buffer[i];
		mpeg2ts->buffer = buf + MSE_MPEG2TS_BUF_SIZE * i;
		mpeg2ts->media_buffer = NULL;
		mpeg2ts->buffer_size = 0;
		mpeg2ts->work_length = 0;
		mpeg2ts->private_data = NULL;
		mpeg2ts->mse_completion = NULL;
	}

	return 0;
}

static int mpeg2ts_packet_size(struct mse_instance *instance)
{
	if (instance->media_config.mpeg2ts.mpeg2ts_type == MSE_MPEG2TS_TYPE_TS)
		return MPEG2TS_TS_SIZE;
	else
		return MPEG2TS_M2TS_SIZE;
}

static bool check_mpeg2ts_pcr(struct mse_instance *instance,
			      struct mse_trans_buffer *buf)
{
	u8 *tsp;
	u8 afc, afc_len, pcr_flag;
	u16 pid;
	u16 pcr_pid = instance->media_config.mpeg2ts.pcr_pid;
	u64 pcr;
	int psize, offset;
	int skip = 0;
	bool ret = false;

	psize = mpeg2ts_packet_size(instance);
	offset = psize - MPEG2TS_TS_SIZE;

	while (buf->work_length + psize < buf->buffer_size) {
		tsp = buf->buffer + buf->work_length + offset;

		/* check sync byte */
		if (buf->work_length + psize < buf->buffer_size &&
		    tsp[0] != MPEG2TS_SYNC) {
			buf->work_length++;
			skip++;
			continue;
		}

		if (skip) {
			mse_debug("check sync byte. skip %d byte\n",
				  skip);
			skip = 0;
		}

		buf->work_length += psize;

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

				ret = true;
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

			ret = true;
		}
	}

	return ret;
}

static void mpeg2ts_buffer_flush(struct mse_instance *instance)
{
	struct mse_trans_buffer *mpeg2ts;
	int idx;

	idx = instance->mpeg2ts_buffer_idx;
	mpeg2ts = &instance->mpeg2ts_buffer[idx];

	if (!mpeg2ts->buffer_size)
		return; /* no data */

#ifdef DEBUG
	if (instance->f_force_flush)
		mse_debug("flush %zu bytes, before stopping.\n",
			  mpeg2ts->buffer_size);
#endif

	/* add mpeg2ts buffer to proc buffer list*/
	mpeg2ts->work_length = 0;
	list_add_tail(&mpeg2ts->list, &instance->proc_buf_list);
	atomic_inc(&instance->trans_buf_cnt);
	instance->mpeg2ts_buffer_idx = (idx + 1) % MSE_MPEG2TS_BUF_NUM;
	instance->f_force_flush = true;
}

static int mpeg2ts_buffer_write(struct mse_instance *instance,
				struct mse_trans_buffer *buf)
{
	bool trans_start;
	unsigned char *buffer = buf->media_buffer;
	size_t buffer_size = buf->buffer_size;
	struct mse_trans_buffer *mpeg2ts;
	int idx;
	size_t request_size;

	idx = instance->mpeg2ts_buffer_idx;
	mpeg2ts = &instance->mpeg2ts_buffer[idx];

	mpeg2ts->media_buffer = buffer;
	mpeg2ts->private_data = buf->private_data;
	mpeg2ts->mse_completion = buf->mse_completion;

	request_size = mpeg2ts->buffer_size + buffer_size;
	if (request_size > MSE_MPEG2TS_BUF_SIZE) {
		mse_err("mpeg2ts buffer overrun %zu/%u\n",
			request_size, MSE_MPEG2TS_BUF_SIZE);
		mse_trans_complete(instance, -EIO);

		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);

		return -1;
	}

	mse_debug("to=%p from=%p size=%zu, buffer_size=%zu\n",
		  mpeg2ts->buffer + mpeg2ts->buffer_size,
		  buffer, buffer_size,
		  mpeg2ts->buffer_size + buffer_size);

	memcpy(mpeg2ts->buffer + mpeg2ts->buffer_size,
	       buffer, buffer_size);
	mpeg2ts->buffer_size += buffer_size;

	if (request_size + buffer_size >= MSE_MPEG2TS_BUF_SIZE)
		instance->f_force_flush = true;

#ifdef DEBUG
	if (instance->f_force_flush)
		mse_debug("flush %zu bytes.\n", mpeg2ts->buffer_size);
#endif

	trans_start = check_mpeg2ts_pcr(instance, mpeg2ts);
	if (trans_start)
		instance->f_force_flush = false;
	else
		trans_start = instance->f_force_flush;

	if (!trans_start) {
		/* Not enough data, request next buffer */
		mse_trans_complete(instance, buffer_size);

		return -1;
	}

	instance->f_trans_start = true;

	/* replace proc buffer */
	list_del(&buf->list);
	buf->media_buffer = NULL;
	buf->buffer_size = 0;
	buf->private_data = NULL;
	buf->mse_completion = NULL;

	mpeg2ts->work_length = 0;
	list_add_tail(&mpeg2ts->list, &instance->proc_buf_list);
	instance->mpeg2ts_buffer_idx = (idx + 1) % MSE_MPEG2TS_BUF_NUM;

	return 0;
}

static void mse_work_start_transmission(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mse_trans_buffer *buf;
	int ret;
	struct ptp_clock_time now;
	unsigned long flags;

	instance = container_of(work, struct mse_instance, wk_start_trans);

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return;

	spin_lock_irqsave(&instance->lock_buf_list, flags);
	buf = list_first_entry_or_null(&instance->trans_buf_list,
				       struct mse_trans_buffer, list);

	/* no transmission buffer */
	if (!buf) {
		spin_unlock_irqrestore(&instance->lock_buf_list, flags);
		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			/* if using mpeg2ts buffer, flush last data */
			if (instance->mpeg2ts_buffer_base) {
				mpeg2ts_buffer_flush(instance);
				queue_work(instance->wq_packet,
					   &instance->wk_packetize);
			}
		}

		return;
	}

	list_move_tail(&buf->list, &instance->proc_buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	mse_debug("index=%d buffer=%p buffer_size=%zu\n",
		  instance->index_media, buf->media_buffer,
		  buf->buffer_size);

	/* update timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index,
			 instance->ptp_dev_id, &now);
	instance->timestamp = (unsigned long)now.sec * NSEC_SCALE + now.nsec;

	if (instance->ptp_clock == 0)
		tstamps_store_ptp_timestamp(instance, &now);

	buf->buffer = buf->media_buffer;

	adapter = instance->media;
	if (instance->tx) {
		if (IS_MSE_TYPE_MPEG2TS(adapter->type))
			if (mpeg2ts_buffer_write(instance, buf))
				return;

		if (IS_MSE_TYPE_AUDIO(adapter->type)) {
			ret = create_avtp_timestamps(instance);
			if (ret < 0) {
				write_lock_irqsave(&instance->lock_state,
						   flags);
				/* if state is EXECUTE, change to IDLE */
				mse_state_change_if(instance, MSE_STATE_IDLE,
						    MSE_STATE_EXECUTE);
				write_unlock_irqrestore(&instance->lock_state,
							flags);

				mse_trans_complete(instance, ret);
				return;
			}
		}

		/* start workqueue for packetize */
		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		if (buf->buffer)
			memset(buf->buffer, 0, buf->buffer_size);

		/* start workqueue for depacketize */
		queue_work(instance->wq_packet, &instance->wk_depacketize);
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

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* get config */
	memcpy(config, &instance->media_config.audio, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_audio_config);

int mse_set_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;
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

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
	}

	adapter = instance->media;
	media_audio_config = &adapter->config.media_audio_config;
	net_config = &instance->net_config;
	packetizer = instance->packetizer;
	index_packetizer = instance->index_packetizer;
	network = instance->network;
	index_network = instance->index_network;

	/* calc timer value */
	instance->timer_interval = div64_u64(
		NSEC_SCALE * (u64)config->period_size,
		(u64)config->sample_rate);
	mse_info("timer_interval=%llu\n", instance->timer_interval);

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

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* get config */
	memcpy(config, &instance->media_config.video, sizeof(*config));

	return 0;
}
EXPORT_SYMBOL(mse_get_video_config);

int mse_set_video_config(int index, struct mse_video_config *config)
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

	mse_debug("index=%d data=%p\n", index, config);
	mse_info("  format=%d bitrate=%d fps=%d/%d\n"
		 "  bytes_per_frame=%d\n",
		 config->format, config->bitrate, config->fps.numerator,
		 config->fps.denominator, config->bytes_per_frame);

	instance = &mse->instance_table[index];

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
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
		instance->timer_interval = div64_u64(
			NSEC_SCALE * (u64)config->fps.denominator,
			config->fps.numerator);
		mse_info("timer_interval=%llu\n", instance->timer_interval);
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

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
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

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
	}

	mse_debug("mpeg2ts_type=%d\n", config->mpeg2ts_type);

	net_config = &instance->net_config;
	packetizer = instance->packetizer;
	index_packetizer = instance->index_packetizer;
	network = instance->network;
	index_network = instance->index_network;

	instance->timer_interval = MPEG2TS_TIMER_NS;
	mse_info("timer_interval=%llu\n", instance->timer_interval);

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

/* free packet buffer */
static void packet_buffer_free(struct mse_instance *instance)
{
	mse_packet_ctrl_free(instance->packet_buffer);
	instance->packet_buffer = NULL;
}

/* allocate packet buffer */
static int packet_buffer_alloc(struct mse_instance *instance)
{
	int ring_size;
	struct mse_packet_ctrl *packet_buffer;

	ring_size = instance->tx ? MSE_TX_RING_SIZE : MSE_RX_RING_SIZE;

	packet_buffer = mse_packet_ctrl_alloc(&mse->pdev->dev,
					      ring_size,
					      MSE_PACKET_SIZE_MAX);

	if (!packet_buffer)
		return -ENOMEM;

	instance->packet_buffer = packet_buffer;

	return 0;
}

/* associate packet buffer with network adapter */
static int packet_buffer_prepare(struct mse_instance *instance)
{
	if (instance->tx)
		return mse_packet_ctrl_send_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);
	else
		return mse_packet_ctrl_receive_prepare_packet(
						instance->index_network,
						instance->packet_buffer,
						instance->network);
}

static void crf_network_cleanup(struct mse_instance *instance)
{
	if (instance->crf_index_network < 0)
		return;

	instance->network->release(instance->crf_index_network);
	instance->crf_index_network = MSE_INDEX_UNDEFINED;

	mse_packet_ctrl_free(instance->crf_packet_buffer);
	instance->crf_packet_buffer = NULL;
}

static int crf_tx_network_setup(struct mse_instance *instance)
{
	int ret;
	int index_network;
	struct mse_adapter_network_ops *network;
	struct mse_packet_ctrl *packet_buffer;
	char *dev_name;

	network = instance->network;
	dev_name =
		instance->media->config.network_device.device_name_tx_crf;

	index_network = network->open(dev_name);
	if (index_network < 0)
		return index_network;

	/* allocate packet buffer */
	packet_buffer = mse_packet_ctrl_alloc(&mse->pdev->dev,
					      MSE_CRF_TX_RING_SIZE,
					      MSE_PACKET_SIZE_MAX);

	if (!packet_buffer) {
		network->release(index_network);

		return -ENOMEM;
	}

	/* associate crf packet buffer with network adapter */
	ret = mse_packet_ctrl_send_prepare_packet(index_network,
						  packet_buffer,
						  instance->network);

	if (ret) {
		mse_packet_ctrl_free(packet_buffer);
		network->release(index_network);

		return ret;
	}

	instance->crf_index_network = index_network;
	instance->crf_packet_buffer = packet_buffer;

	return 0;
}

static int crf_rx_network_setup(struct mse_instance *instance)
{
	int ret;
	int index_network;
	struct mse_adapter_network_ops *network;
	struct mse_packet_ctrl *packet_buffer;
	char *dev_name;

	network = instance->network;
	dev_name =
		instance->media->config.network_device.device_name_rx_crf;

	index_network = network->open(dev_name);
	if (index_network < 0)
		return index_network;

	/* allocate packet buffer */
	packet_buffer = mse_packet_ctrl_alloc(&mse->pdev->dev,
					      MSE_CRF_RX_RING_SIZE,
					      MSE_PACKET_SIZE_MAX);

	if (!packet_buffer) {
		network->release(index_network);

		return -ENOMEM;
	}

	/* associate crf packet buffer with network adapter */
	ret = mse_packet_ctrl_receive_prepare_packet(index_network,
						     packet_buffer,
						     network);

	if (ret) {
		mse_packet_ctrl_free(packet_buffer);
		network->release(index_network);

		return ret;
	}

	instance->crf_index_network = index_network;
	instance->crf_packet_buffer = packet_buffer;

	return 0;
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
		if (mse->instance_table[i].state == MSE_STATE_CLOSE)
			break;
	}
	index = i;

	if (ARRAY_SIZE(mse->instance_table) <= index) {
		mse_err("resister instance full!\n");
		mutex_unlock(&mse->mutex_open);
		return -EBUSY;
	}

	instance = &mse->instance_table[index];
	mse_debug_state(instance);
	instance->state = MSE_STATE_OPEN;
	instance->used_f = true;
	instance->trans_idx = 0;
	init_completion(&instance->completion_stop);
	complete(&instance->completion_stop);
	atomic_set(&instance->trans_buf_cnt, 0);
	atomic_set(&instance->done_buf_cnt, 0);
	init_waitqueue_head(&instance->wait_wk_stream);
	INIT_LIST_HEAD(&instance->trans_buf_list);
	INIT_LIST_HEAD(&instance->proc_buf_list);
	adapter->ro_config_f = true;
	rwlock_init(&instance->lock_state);
	rwlock_init(&instance->lock_stream);
	spin_lock_init(&instance->lock_timer);
	spin_lock_init(&instance->lock_ques);
	spin_lock_init(&instance->lock_buf_list);

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

	/* Get packetizer */
	packetizer_id = adapter->config.packetizer.packetizer;
	mse_debug("packetizer id=%d\n", packetizer_id);
	packetizer = mse_packetizer_get_ops(packetizer_id);
	if (!packetizer) {
		mse_err("packetizer is not valid\n");
		err = -EINVAL;

		goto error_packetizer_is_not_valid;
	}

	/* if mpeg2ts tx */
	if (tx && IS_MSE_TYPE_MPEG2TS(adapter->type)) {
		err = mpeg2ts_buffer_alloc(instance);
		if (err) {
			mse_err("cannot allocate mpeg2ts_buffer\n");
			goto error_cannot_alloc_mpeg2ts_buffer;
		}
	}

	instance->f_force_flush = false;

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
	ret = mse_packetizer_open(packetizer_id);
	if (ret < 0) {
		mse_err("cannot open packetizer ret=%d\n", ret);
		err = ret;

		goto error_cannot_open_packetizer;
	}
	instance->index_packetizer = ret;

	/* set selector */
	instance->tx = tx;
	instance->media = adapter;
	instance->packetizer = packetizer;
	instance->packetizer_id = packetizer_id;
	instance->network = network;
	instance->index_media = index_media;
	instance->crf_index = MSE_INDEX_UNDEFINED;

	/* init work queue */
	INIT_WORK(&instance->wk_packetize, mse_work_packetize);
	INIT_WORK(&instance->wk_depacketize, mse_work_depacketize);
	INIT_WORK(&instance->wk_callback, mse_work_callback);
	INIT_WORK(&instance->wk_stream, mse_work_stream);
	INIT_WORK(&instance->wk_crf_send, mse_work_crf_send);
	INIT_WORK(&instance->wk_crf_receive, mse_work_crf_receive);
	INIT_WORK(&instance->wk_timestamp, mse_work_timestamp);
	INIT_WORK(&instance->wk_start_trans, mse_work_start_transmission);
	INIT_WORK(&instance->wk_stop_streaming, mse_work_stop_streaming);

	instance->wq_stream = create_singlethread_workqueue("mse_streamq");
	instance->wq_packet = create_singlethread_workqueue("mse_packetq");
	hrtimer_init(&instance->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	instance->timer_interval = 0;
	instance->timer.function = &mse_timer_callback;

	/* for timestamp */
	instance->wq_tstamp = create_singlethread_workqueue("mse_tstampq");
	if (IS_MSE_TYPE_AUDIO(adapter->type)) {
		spin_lock_irqsave(&instance->lock_ques, flags);
		tstamps_clear_tstamps(&instance->tstamp_que);
		tstamps_clear_tstamps(&instance->tstamp_que_crf);
		spin_unlock_irqrestore(&instance->lock_ques, flags);

		hrtimer_init(&instance->tstamp_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->tstamp_timer_interval = PTP_TIMER_INTERVAL;
		instance->tstamp_timer.function =
					&mse_timestamp_collect_callback;

		/* for crf */
		instance->wq_crf_packet =
			create_singlethread_workqueue("mse_crfpacketq");
		hrtimer_init(&instance->crf_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->crf_timer_interval = CRF_TIMER_INTERVAL;
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

	/* allocate packet buffer */
	err = packet_buffer_alloc(instance);
	if (err)
		goto error_packet_buffer_alloc;

	/* associate packet buffer with network adapter */
	err = packet_buffer_prepare(instance);
	if (err)
		goto error_packet_buffer_prepare;

	/* send clock using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_TX)
		err = crf_tx_network_setup(instance);

	/* receive clock using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_RX)
		err = crf_rx_network_setup(instance);

	if (err)
		goto error_cannot_crf_network_setup;

	/* init paketizer */
	err = instance->packetizer->init(instance->index_packetizer);

	if (err)
		goto error_packetizer_init;

	return index;

error_packetizer_init:
	crf_network_cleanup(instance);

error_cannot_crf_network_setup:
error_packet_buffer_prepare:
	packet_buffer_free(instance);

error_packet_buffer_alloc:
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

	mse_packetizer_release(packetizer_id, instance->index_packetizer);

error_cannot_open_packetizer:
error_network_interface_is_link_down:
	network->release(instance->index_network);

error_cannot_open_network_adapter:
	mse_ptp_close(instance->ptp_index, instance->ptp_dev_id);

error_cannot_open_ptp:
	mpeg2ts_buffer_free(instance);

error_cannot_alloc_mpeg2ts_buffer:
error_packetizer_is_not_valid:
error_network_adapter_not_found:
	instance->state = MSE_STATE_CLOSE;
	instance->used_f = false;
	adapter->ro_config_f = false;

	return err;
}
EXPORT_SYMBOL(mse_open);

int mse_close(int index)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int err = -EINVAL;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return err;
	}

	mse_debug("index=%d\n", index);

	instance = &mse->instance_table[index];
	adapter = instance->media;

	write_lock_irqsave(&instance->lock_state, flags);
	mse_debug_state(instance);

	/* state is STARTED */
	if (mse_state_test_nolock(instance, MSE_STATE_STARTED)) {
		mse_debug("wait for completion\n");
		write_unlock_irqrestore(&instance->lock_state, flags);
		wait_for_completion(&instance->completion_stop);

		write_lock_irqsave(&instance->lock_state, flags);
		mse_debug_state(instance);

		/* state is STARTED */
		if (mse_state_test_nolock(instance, MSE_STATE_STARTED)) {
			write_unlock_irqrestore(&instance->lock_state, flags);
			mse_err("instance is busy. index=%d\n", index);
			return -EBUSY;
		}
	}

	/* state is CLOSE */
	if (mse_state_test_nolock(instance, MSE_STATE_CLOSE)) {
		write_unlock_irqrestore(&instance->lock_state, flags);
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	err = mse_state_change(instance, MSE_STATE_CLOSE);
	write_unlock_irqrestore(&instance->lock_state, flags);
	if (err)
		return err;

	mutex_lock(&mse->mutex_open);

	/* flush workqueue */
	flush_workqueue(instance->wq_packet);
	flush_workqueue(instance->wq_stream);

	/* destroy workqueue */
	destroy_workqueue(instance->wq_packet);
	destroy_workqueue(instance->wq_stream);

	destroy_workqueue(instance->wq_tstamp);
	if (IS_MSE_TYPE_AUDIO(adapter->type))
		destroy_workqueue(instance->wq_crf_packet);

	/* release packetizer */
	mse_packetizer_release(instance->packetizer_id,
			       instance->index_packetizer);

	/* release network adapter */
	instance->network->release(instance->index_network);

	/* free packet buffer */
	packet_buffer_free(instance);

	mse_release_crf_packetizer(instance);

	/* cleanup crf tx*/
	crf_network_cleanup(instance);

	err = mse_ptp_close(instance->ptp_index, instance->ptp_dev_id);
	if (err < 0)
		mse_err("cannot mse_ptp_close()\n");

	mpeg2ts_buffer_free(instance);

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

	mutex_unlock(&mse->mutex_open);

	return 0;
}
EXPORT_SYMBOL(mse_close);

int mse_start_streaming(int index)
{
	int err = -EINVAL;
	struct mse_instance *instance;
	struct ptp_clock_time now;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return err;
	}

	mse_debug("index=%d\n", index);

	instance = &mse->instance_table[index];

	mse_debug_state(instance);

	/* state is RUNNABLE */
	if (mse_state_test(instance, MSE_STATE_RUNNABLE))
		return 0;

	write_lock_irqsave(&instance->lock_state, flags);
	err = mse_state_change(instance, MSE_STATE_IDLE);
	write_unlock_irqrestore(&instance->lock_state, flags);
	if (err)
		return err;

	/* get timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index,
			 instance->ptp_dev_id, &now);
	mse_tstamp_init(instance, &now);

	if (IS_MSE_TYPE_AUDIO(instance->media->type))
		mse_start_streaming_audio(instance, &now);

	mse_start_streaming_common(instance);

	return err;
}
EXPORT_SYMBOL(mse_start_streaming);

int mse_stop_streaming(int index)
{
	struct mse_instance *instance;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);
	instance = &mse->instance_table[index];

	mse_debug_state(instance);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is RUNNABLE */
	if (mse_state_test(instance, MSE_STATE_RUNNABLE))
		queue_work(instance->wq_packet, &instance->wk_stop_streaming);

	return 0;
}
EXPORT_SYMBOL(mse_stop_streaming);

int mse_start_transmission(int index,
			   void *buffer,
			   size_t buffer_size,
			   void *priv,
			   int (*mse_completion)(void *priv, int size))
{
	int err = -EINVAL;
	struct mse_instance *instance;
	struct mse_trans_buffer *buf;
	int buf_cnt;
	int idx;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return err;
	}

	if (!buffer) {
		mse_err("invalid argument. buffer is NULL\n");
		return err;
	}

	if (!buffer_size) {
		mse_err("invalid argument. buffer_size is zero\n");
		return err;
	}

	mse_debug("index=%d buffer=%p size=%zu\n", index, buffer, buffer_size);

	instance = &mse->instance_table[index];

	write_lock_irqsave(&instance->lock_state, flags);
	mse_debug_state(instance);

	/* state is STOPPING */
	if (mse_state_test_nolock(instance, MSE_STATE_STOPPING)) {
		write_unlock_irqrestore(&instance->lock_state, flags);
		mse_err("instance is busy. index=%d\n", index);

		return -EBUSY;
	}

	/* state is NOT RUNNABLE */
	if (!mse_state_test_nolock(instance, MSE_STATE_RUNNABLE)) {
		write_unlock_irqrestore(&instance->lock_state, flags);
		mse_err("operation is not permitted. index=%d\n", index);

		return -EPERM;
	}

	err = mse_state_change(instance, MSE_STATE_EXECUTE);
	write_unlock_irqrestore(&instance->lock_state, flags);

	if (!err) {
		buf_cnt = atomic_read(&instance->trans_buf_cnt);
		if (buf_cnt >= MSE_TRANS_BUF_ACCEPTABLE)
			return -EAGAIN;

		spin_lock_irqsave(&instance->lock_buf_list, flags);
		idx = instance->trans_idx;
		buf = &instance->trans_buffer[idx];
		buf->media_buffer = buffer;
		buf->buffer = NULL;
		buf->buffer_size = buffer_size;
		buf->work_length = 0;
		buf->private_data = priv;
		buf->mse_completion = mse_completion;
		instance->trans_idx = (idx + 1) % MSE_TRANS_BUF_NUM;

		list_add_tail(&buf->list, &instance->trans_buf_list);
		atomic_inc(&instance->trans_buf_cnt);

		spin_unlock_irqrestore(&instance->lock_buf_list, flags);
		queue_work(instance->wq_packet, &instance->wk_start_trans);
	}

	return err;
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
	if (IS_ENABLED(CONFIG_OF))
		of_dma_configure(&mse->pdev->dev, NULL);
	else
		/*
		 * MSE has no dependency to OF, but w/o CONFIG_OF set the
		 * above function does nothing while at least initializing
		 * dma_mask, coherent_dma_mask is mandatory. Limitation to
		 * 32bit is needed, as struct eavb_entryvec relies on 32bit
		 * addresses.
		 */
		dma_coerce_mask_and_coherent(&mse->pdev->dev, DMA_BIT_MASK(32));

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
		mse->instance_table[i].state = MSE_STATE_CLOSE;
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
