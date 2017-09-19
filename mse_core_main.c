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
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include "avtp.h"
#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "mse_config.h"
#include "mse_packet_ctrl.h"
#include "mse_sysfs.h"
#include "mse_ptp.h"
#include "mse_ioctl_local.h"

#define MSE_DEBUG_TSTAMPS  (0)
#define MSE_DEBUG_TSTAMPS2 (0) /* very noisy */
#define MSE_DEBUG_STATE    (0)

#define MSE_TIMEOUT_CLOSE       (msecs_to_jiffies(5000)) /* 5secs */
#define MSE_TIMEOUT_PACKETIZE   (msecs_to_jiffies(16))   /* 16 msecs */

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

#define q_next(que, pos)        (((pos) + 1) % (que)->len)
#define q_prev(que, pos)        (((pos) - 1 + (que)->len) % (que)->len)
#define q_empty(que)            ((que)->head == (que)->tail)

#define PTP_TIMESTAMPS_MAX   (512)
#define PTP_TIMER_INTERVAL   (20 * 1000000)  /* 1/300 sec * 6 = 20ms */

/* judge error 5% */
#define PTP_TIMER_ERROR_THRESHOLD(x)  div64_u64((x) * 5, 100)

/* judge error 2 periods */
#define PTP_START_ERROR_THRESHOLD(x)  ((x) * 2)

/* judge error 10% capture syncronized */
#define PTP_SYNC_ERROR_THRESHOLD(x)  div64_u64((x) * 10, 100)

/* judge error 10% valid to mch */
#define MCH_ERROR_THRESHOLD(x)  div64_u64((x) * 10, 100)

#define PTP_SYNC_LOCK_THRESHOLD (3)

#define CRF_TIMER_INTERVAL   (20 * 1000000)  /* 20ms */
#define CRF_PTP_TIMESTAMPS   (1)     /* timestamps per CRF packet using ptp */
#define CRF_AUDIO_TIMESTAMPS (6)     /* audio timestamps per CRF packet */

#define CREATE_AVTP_TIMESTAMPS_MAX   (4096)

#define MSE_DECODE_BUFFER_NUM (8)
#define MSE_DECODE_BUFFER_NUM_START_MIN (2)
#define MSE_DECODE_BUFFER_NUM_START_MAX (6)
#define MAX_DECODE_SIZE       (8192) /* ALSA Period byte size */

#define MSE_TRANS_BUF_NUM (3) /* size of transmission buffer array */
#define MSE_TRANS_BUF_ACCEPTABLE (MSE_TRANS_BUF_NUM - 1)

#define MSE_MPEG2TS_BUF_NUM  (MSE_TRANS_BUF_NUM)
#define MSE_MPEG2TS_BUF_SIZE (512U * 1024U)
#define MSE_MPEG2TS_BUF_THRESH (188U * 192U * 14U)

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

struct timestamp_set {
	u64 std;
	u64 real;
};

struct timestamp_queue {
	const char *name;
	bool f_init;
	bool f_sync;
	bool f_32bit;
	bool f_discont;
	u32 std_interval;
	u64 std_counter;
	int sync_count;
	int head;
	int tail;
	int len;
	struct timestamp_set timestamps[PTP_TIMESTAMPS_MAX];
};

struct timestamp_reader {
	const char *name;
	bool f_out_ok;
	bool f_32bit;
	u64 out_time;
	u64 out_std;
	u64 out_offset;
	struct timestamp_queue *que;
};

struct mse_timing_ctrl {
	u64 start_time;
	u64 std_start_time;
	u32 start_time_count;
	u32 send_count;
	u64 first_send_time;
	u64 std_first_send_time;
	u32 captured_timestamps;
	u64 last_complete;
	bool f_ok;
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
	/** @brief semaphore for stopping process */
	struct semaphore sem_stopping;

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
	int crf_discont;

	void *ptp_handle;
	void *ptp_timer_handle;
	void *mch_handle;
	int ptp_index;
	int mch_index;

	/* @brief timestamp ptp|capture */
	spinlock_t lock_ques;
	struct timestamp_queue tstamp_que;
	struct timestamp_queue tstamp_que_crf;
	struct timestamp_queue crf_que;
	struct timestamp_queue avtp_que;
	struct timestamp_reader reader_create_avtp;
	struct timestamp_reader reader_mch;
	struct timestamp_reader reader_ptp_start_time;

	/** @brief timestamp(nsec) */
	u32 timestamp;

	/** @brief start timing using ptp_timer */
	u64 ptp_timer_start;

	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;

	/** @brief AVTP timestampes */
	u32 avtp_timestamps[CREATE_AVTP_TIMESTAMPS_MAX];
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
	bool f_wait_start_transmission;

	/** @brief streaming flag */
	bool f_streaming;
	/** @brief spin lock for streaming flag */
	rwlock_t lock_stream;

	/** @brief timing control */
	struct mse_timing_ctrl timing_ctrl;

	/** @brief network configuration */
	struct mse_network_config net_config;
	struct mse_network_config crf_net_config;

	/** @brief media specific configuration */
	struct mse_media_config {
		struct mse_audio_config audio;
		struct mse_video_config video;
		struct mse_mpeg2ts_config mpeg2ts;
	} media_config;

	/** @brief MCH & CRF Settings **/
	int f_ptp_capture;
	int f_mch_enable;
	int ptp_clock_device;
	int ptp_clock_ch;
	int ptp_capture_freq;
	int media_capture_freq;
	enum MSE_CRF_TYPE crf_type;
	u32 max_transit_time_ns;
	u32 delay_time_ns;
	int remain;

	bool f_present;
	bool f_get_first_packet;
	u32 first_avtp_timestamp;

	/** @brief packet buffer */
	int crf_index_network;
	struct mse_packet_ctrl *crf_packet_buffer;
	bool f_crf_sending;

	/** @brief media clock recovery work */
	u64 timestamps[PTP_TIMESTAMPS_MAX];
	struct mch_timestamp ts[PTP_TIMESTAMPS_MAX];

	/** @brief mpeg2ts buffer  */
	u64 mpeg2ts_pcr_90k;
	u64 mpeg2ts_clock_90k;
	int mpeg2ts_pre_pcr_pid;
	u64 mpeg2ts_pre_pcr_90k;
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
 * function prototypes
 */
static int mse_get_capture_timestamp(struct mse_instance *instance,
				     u64 now,
				     bool is_first);
static int tstamps_get_last_timestamp(struct timestamp_queue *que,
				      struct timestamp_set *ts_set);

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
	pr_debug("%s: state=%s flags=[%d %d %d %d %d %d %d]\n",
		 func, mse_state_stringfy(instance->state),
		 instance->used_f, instance->f_trans_start,
		 instance->f_continue, instance->f_depacketizing,
		 instance->f_stopping, instance->f_completion,
		 instance->f_wait_start_transmission);
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
		instance->max_transit_time_ns = delay_time.max_transit_time_ns;
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
		instance->max_transit_time_ns = delay_time.max_transit_time_ns;
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
		instance->f_ptp_capture =
			(ptp_config.type == MSE_PTP_TYPE_CAPTURE);
		instance->ptp_clock_ch = ptp_config.capture_ch;
		instance->ptp_capture_freq = ptp_config.capture_freq;

		err = mse_config_get_mch_config(index,
						&mch_config);
		instance->f_mch_enable = mch_config.enable;
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
		instance->max_transit_time_ns = delay_time.max_transit_time_ns;
		if (instance->tx)
			instance->delay_time_ns = delay_time.tx_delay_time_ns;
		else
			instance->delay_time_ns = delay_time.rx_delay_time_ns;
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
	.get_time = mse_ptp_get_time_dummy,
	.open = mse_ptp_open_dummy,
	.close = mse_ptp_close_dummy,
	.capture_start = mse_ptp_capture_start_dummy,
	.capture_stop = mse_ptp_capture_stop_dummy,
	.get_timestamps = mse_ptp_get_timestamps_dummy,
	.timer_open = mse_ptp_timer_open_dummy,
	.timer_close = mse_ptp_timer_close_dummy,
	.timer_start = mse_ptp_timer_start_dummy,
	.timer_cancel = mse_ptp_timer_cancel_dummy,
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

static int mse_ptp_get_time(int index, u64 *ns)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->get_time(ns);
}

static void *mse_ptp_open(int index)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->open();
}

static int mse_ptp_close(int index, void *ptp_handle)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->close(ptp_handle);
}

static int mse_ptp_capture_start(int index, void *ptp_handle, int ch,
				 int max_count)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->capture_start(ptp_handle, ch, max_count);
}

static int mse_ptp_capture_stop(int index, void *ptp_handle)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->capture_stop(ptp_handle);
}

static int mse_ptp_get_timestamps(int index,
				  void *ptp_handle,
				  int req_count,
				  u64 *timestamps)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->get_timestamps(ptp_handle,
				     req_count,
				     timestamps);
}

static void *mse_ptp_timer_open(int index, u32 (*handler)(void *priv),
				void *priv)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->timer_open(handler, priv);
}

static int mse_ptp_timer_close(int index, void *timer_handle)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->timer_close(timer_handle);
}

static int mse_ptp_timer_start(int index, void *timer_handle, u32 start)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->timer_start(timer_handle, start);
}

static int mse_ptp_timer_cancel(int index, void *timer_handle)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	return p_ops->timer_cancel(timer_handle);
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
	struct mse_packet_ctrl *packet_buffer;
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
	packet_buffer = instance->packet_buffer;
	network = instance->network;

	if (instance->tx) {
		/* while data is remained */
		do {
			/* request send packet */
			err = mse_packet_ctrl_send_packet(index_network,
							  packet_buffer,
							  network);

			if (err < 0) {
				mse_err("send error %d\n", err);
				break;
			}

			wake_up_interruptible(&instance->wait_wk_stream);
		} while (mse_packet_ctrl_check_packet_remain(packet_buffer));
	} else {
		/* while state is RUNNABLE */
		while (mse_state_test(instance, MSE_STATE_RUNNABLE)) {
			/* request receive packet */
			err = mse_packet_ctrl_receive_packet(
				index_network,
				MSE_RX_PACKET_NUM,
				packet_buffer,
				network);

			if (err < 0) {
				mse_err("receive error %d\n", err);
				break;
			}

			/* if NOT work queued, then queue it */
			if (!work_busy(&instance->wk_depacketize))
				queue_work(instance->wq_packet,
					   &instance->wk_depacketize);
		}

		/* if NOT work queued, then queue it to process last data */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			if (!work_busy(&instance->wk_depacketize))
				queue_work(instance->wq_packet,
					   &instance->wk_depacketize);
	}

	write_lock_irqsave(&instance->lock_stream, flags);
	instance->f_streaming = false;
	mse_debug_state(instance);
	write_unlock_irqrestore(&instance->lock_stream, flags);

	mse_debug("END\n");
}

/*
 * Timestamps related functions
 */
#if (MSE_DEBUG_TSTAMPS)
#define mse_debug_tstamps(fmt, ...) mse_info(fmt, ## __VA_ARGS__)
#else
#define mse_debug_tstamps(fmt, ...) no_printk(fmt, ## __VA_ARGS__)
#endif

#if (MSE_DEBUG_TSTAMPS2)
#define mse_debug_tstamps2(fmt, ...) mse_debug(fmt, ## __VA_ARGS__)
#else
#define mse_debug_tstamps2(fmt, ...) no_printk(fmt, ## __VA_ARGS__)
#endif

static void tstamps_init(struct timestamp_queue *que,
			 const char *name,
			 bool f_32bit,
			 u32 std_interval)
{
	que->name = name;
	que->f_init = true;
	que->head = 0;
	que->tail = 0;
	que->f_32bit = f_32bit;
	que->std_interval = std_interval;
	que->sync_count = 0;
	que->f_sync = false;
	que->f_discont = false;
	que->len = ARRAY_SIZE(que->timestamps);

	mse_debug_tstamps2("%s: interval %u\n", que->name, que->std_interval);
}

static void tstamps_reader_init(struct timestamp_reader *reader,
				struct timestamp_queue *que,
				const char *name,
				bool f_32bit)
{
	reader->name = name;
	reader->f_out_ok = false;
	reader->f_32bit = f_32bit;
	reader->que = que;
	reader->out_time = 0;
	reader->out_std = 0;
	reader->out_offset = 0;

	mse_debug_tstamps2("%s\n", reader->name);
}

/* calculate timestamp from std_time */
static int tstamps_calc_tstamp(struct timestamp_reader *reader,
			       u64 now,
			       u32 offset,
			       u64 interval,
			       u64 *timestamp)
{
	struct timestamp_queue *que = reader->que;
	struct timestamp_set ts_last = { 0 };
	struct timestamp_set ts1, ts2;
	int pos1, pos2;
	u64 t;
	bool f_first = false;

	if (!que)
		return -1;

	if (que->f_discont)
		reader->f_out_ok = false;

	/* first & not received */
	if (!que->f_sync) {
		if (reader->out_time == 0)
			reader->out_time = now;
		else
			reader->out_time += interval;

		*timestamp = reader->out_time;
		return -1;
	}

	/* first sync output */
	if (!reader->f_out_ok) {
		tstamps_get_last_timestamp(que, &ts_last);
		reader->out_std = ts_last.std + now - ts_last.real - offset;
		reader->out_offset = offset;
		if (que->timestamps[que->head].std > reader->out_std) {
			reader->out_std = que->timestamps[que->head].std;
			reader->out_offset = now -
				que->timestamps[que->head].real;
		}

		reader->f_out_ok = true;
		f_first = true;
	} else {
		reader->out_std += interval;
	}

	pos1 = que->head;
	for (pos2 = que->head;
	     q_next(que, pos2) != que->tail;
	     pos2 = q_next(que, pos2)) {
		if (que->timestamps[pos2].std > reader->out_std)
			break;

		pos1 = pos2;
	}

	if (pos2 == que->head) {
		mse_debug_tstamps("ERROR %s std_time %llu(+%llu) is over adjust range %llu - %llu\n",
				  reader->name,
				  reader->out_std,
				  reader->out_offset,
				  que->timestamps[que->head].std,
				  que->timestamps[q_prev(que, que->tail)].std);

		reader->f_out_ok = false;
		reader->out_time += interval;
		*timestamp = reader->out_time;

		return -1;
	}

	ts1 = que->timestamps[pos1];
	ts2 = que->timestamps[pos2];

	t = (ts2.real - ts1.real) * (reader->out_std - ts1.std);
	*timestamp = div64_u64(t, ts2.std - ts1.std) +
			ts1.real + reader->out_offset;

	if (f_first) {
		mse_debug_tstamps2("%s out start %llu ofsset %llu now %llu out %llu prev %llu\n",
				   reader->name,
				   reader->out_std,
				   reader->out_offset,
				   now,
				   *timestamp,
				   reader->out_time);
	}

	reader->out_time = *timestamp;

	return 0;
}

static int tstamps_get_last_timestamp(struct timestamp_queue *que,
				      struct timestamp_set *ts_set)
{
	if (q_empty(que)) {
		mse_debug("time stamp queue is not updated\n");
		return -1;
	}

	if (!que->f_sync)
		return -1;

	*ts_set = que->timestamps[q_prev(que, que->tail)];

	return 0;
}

static int tstamps_get_timestamp_diff_average(struct timestamp_queue *que,
					      u64 *diff,
					      int max)
{
	struct timestamp_set ts_set;
	int pos, count = 0;
	u64 prev, now, diff_sum = 0;

	if (tstamps_get_last_timestamp(que, &ts_set) < 0)
		return -1;

	prev = ts_set.real;
	pos = q_prev(que, que->tail);

	/* for each queue reverse */
	for (pos = q_prev(que, pos);
	     pos != que->head;
	     pos = q_prev(que, pos)) {
		if (count >= max)
			break;

		now = que->timestamps[pos].real;
		diff_sum += prev - now;
		prev = now;
		count++;
	}

	*diff = div64_u64(diff_sum, count);

	return 0;
}

static int tstamps_search_tstamp32(struct timestamp_reader *reader,
				   u32 avtp_time,
				   u32 offset,
				   u32 interval)
{
	struct timestamp_queue *que = reader->que;
	struct timestamp_set ts_set;
	int pos;
	u32 delta_ts;

	if (reader->f_out_ok && que->f_sync)
		return 0;

	if (q_empty(que)) {
		mse_debug("time stamp queue is not updated\n");
		return -1;
	}

	if (!que->f_sync)
		return -1;

	avtp_time -= offset;

	/* for each queue forward */
	for (pos = que->head;
	     pos != que->tail;
	     pos = q_next(que, pos)) {
		ts_set = que->timestamps[pos];
		delta_ts = (u32)ts_set.real - avtp_time;
		if (delta_ts < U32_MAX / 2)
			break;
	}

	if (pos == que->tail || pos == que->head)
		return -1;

	reader->out_std = ts_set.std;

	if (delta_ts < NSEC_SCALE)
		reader->out_std -= delta_ts;
	else
		mse_info("not precision offset capture %u", delta_ts);

	reader->out_std -= interval;
	reader->out_offset = offset;
	reader->f_out_ok = true;

	mse_debug_tstamps2("found %llu(%llu) avtp=%u offset=%u out_std %llu\n",
			   ts_set.real,
			   ts_set.std,
			   avtp_time,
			   offset,
			   reader->out_std);

	return 0;
}

static bool tstamps_continuity_check(struct timestamp_queue *que,
				     u64 timestamp)
{
	struct timestamp_set ts_set;
	s64 diff;

	ts_set = que->timestamps[q_prev(que, que->tail)];

	diff = timestamp - ts_set.real - que->std_interval;
	if (que->f_32bit)
		diff = (s64)(s32)diff;

	if (abs(diff) < PTP_SYNC_ERROR_THRESHOLD(que->std_interval)) {
		if (!que->f_sync) {
			que->sync_count++;
			if (que->sync_count >= PTP_SYNC_LOCK_THRESHOLD) {
				que->f_sync = true;
				mse_debug_tstamps("OK: %s sync\n", que->name);
			}
		}

		return true;
	}

	if (que->f_sync) {
		mse_debug_tstamps("NG: %s discontinuous %llu %llu std %u diff %llu\n",
				  que->name,
				  ts_set.real,
				  timestamp,
				  que->std_interval,
				  diff);
		que->f_discont = true;
	}

	que->f_sync = false;
	que->sync_count = 0;
	que->head = 0;
	que->tail = 0;

	return false;
}

static int tstamps_enq_tstamp(struct timestamp_queue *que,
			      u64 timestamp)
{
	struct timestamp_set timestamp_set;

	mse_debug_tstamps2("START head=%d, tail=%d\n", que->head, que->tail);

	if (!que->f_init)
		return -1;

	if (!q_empty(que))
		if (!tstamps_continuity_check(que, timestamp))
			return 0;

	timestamp_set.real = timestamp;
	timestamp_set.std = que->std_counter;

	que->std_counter += que->std_interval;
	que->timestamps[que->tail] = timestamp_set;
	que->tail = q_next(que, que->tail);

	/* if tail equal head, push out head */
	if (que->tail == que->head)
		que->head = q_next(que, que->head);

	return 0;
}

static int tstamps_deq_tstamp(struct timestamp_queue *que,
			      u64 *timestamp)
{
	if (q_empty(que))
		return -1;

	if (q_next(que, que->head) == que->tail)
		return -1;

	if (!que->f_sync)
		return -1;

	*timestamp = que->timestamps[que->head].real;

	que->head = q_next(que, que->head);

	return 0;
}

static int tstamps_get_tstamps_size(struct timestamp_queue *que)
{
	if (q_empty(que))
		return 0;

	if (que->tail >= que->head)
		return que->tail - que->head - 1;
	else
		return (que->tail + PTP_TIMESTAMPS_MAX) - que->head - 1;
}

static bool media_clock_recovery_calc_ts(struct timestamp_reader *reader,
					 struct timestamp_queue *que,
					 u64 timestamp,
					 u32 interval,
					 u32 offset,
					 struct mch_timestamp *output_ts)
{
	u64 device_time;
	s32 diff;
	bool f_out_ok_pre;

	f_out_ok_pre = reader->f_out_ok;

	if (tstamps_search_tstamp32(reader,
				    (u32)timestamp,
				    offset,
				    interval) < 0) {
		mse_debug_tstamps2("NG: tstamps_search_tstamp32\n");
		reader->f_out_ok = false;
		return false;
	}

	if (tstamps_calc_tstamp(reader,
				0,
				offset,
				interval,
				&device_time) < 0) {
		mse_debug_tstamps("NG: tstamps_calc_tstamp\n");
		reader->f_out_ok = false;
		return false;
	}

	diff = (s32)((u32)timestamp - (u32)device_time);
	if (abs(diff) > MCH_ERROR_THRESHOLD(interval)) {
		mse_debug_tstamps("NG: mch diff error m=%u d=%u (%d) interval=%u\n",
				  (u32)timestamp, (u32)device_time, diff,
				  interval);

		reader->f_out_ok = false;
		return false;
	}

	output_ts->master = timestamp;
	output_ts->device = device_time;

	if (!f_out_ok_pre && reader->f_out_ok)
		mse_debug_tstamps("OK: media clock recovery m %u d %u (%d)",
				  (u32)timestamp, (u32)device_time, diff);

	return true;
}

static int media_clock_recovery(struct mse_instance *instance)
{
	struct mch_ops *m_ops;
	struct timestamp_queue *que;
	struct mse_audio_info audio_info;
	struct mse_packetizer_ops *packetizer;
	struct timestamp_reader *reader_mch = &instance->reader_mch;
	struct mch_timestamp ts;
	int index_packetizer;
	u64 timestamp;
	u32 delta_ts, offset;
	int out = 0;
	int recovery_value;
	u64 recovery_capture_freq;
	bool f_out_ok_pre;
	unsigned long flags;
	int calc_error = 0;

	if (instance->crf_type != MSE_CRF_TYPE_RX) {
		packetizer = instance->packetizer;
		index_packetizer = instance->index_packetizer;
		que = &instance->avtp_que;
		offset = 0;
	} else {
		packetizer = &mse_packetizer_crf_tstamp_audio_ops;
		index_packetizer = instance->crf_index;
		que = &instance->crf_que;
		offset = instance->max_transit_time_ns;
	}

	packetizer->get_audio_info(index_packetizer, &audio_info);
	delta_ts = audio_info.frame_interval_time;
	if (instance->f_ptp_capture)
		offset += instance->delay_time_ns;

	/* if timestamps queue continuity broken */
	if (!que->f_sync || !instance->tstamp_que.f_sync)
		return 0;

	f_out_ok_pre = reader_mch->f_out_ok;

	/* fill master/device timestamps */
	spin_lock_irqsave(&instance->lock_ques, flags);
	while (out < ARRAY_SIZE(instance->ts)) {
		if (tstamps_deq_tstamp(que, &timestamp) < 0)
			break;

		if (!media_clock_recovery_calc_ts(reader_mch,
						  que,
						  timestamp,
						  delta_ts,
						  offset,
						  &ts)) {
			calc_error++;
		} else {
			instance->ts[out] = ts;
			out++;
		}
	}
	/* could not get master timestamps */
	spin_unlock_irqrestore(&instance->lock_ques, flags);
	if (out <= 0) {
		/* if CRF Rx, accept for one period which not get timestamp */
		if (instance->crf_type == MSE_CRF_TYPE_RX &&
		    instance->crf_discont < 1)
			instance->crf_discont++;
		else
			reader_mch->f_out_ok = false;
	} else {
		/* reset CRF discontinuity counter */
		instance->crf_discont = 0;

		/* get mch ops */
		m_ops = mse->mch_table[instance->mch_index];
		m_ops->set_interval(instance->mch_handle, delta_ts);
		m_ops->send_timestamps(instance->mch_handle, instance->ts, out);

		if (instance->media_capture_freq && instance->f_ptp_capture) {
			m_ops->get_recovery_value(instance->mch_handle,
						  &recovery_value);

			recovery_capture_freq = instance->ptp_capture_freq *
				(u64)(NSEC_SCALE + recovery_value);

			instance->tstamp_que.std_interval =
				div64_u64((u64)NSEC_SCALE * (u64)NSEC_SCALE,
					  recovery_capture_freq);

			mse_debug("recover %u\n",
				  instance->tstamp_que.std_interval);
		}
	}

	if (f_out_ok_pre != reader_mch->f_out_ok) {
		if (!f_out_ok_pre)
			mse_debug("Good! m %u d %u\n", ts.master, ts.device);
		else
			mse_debug("Bad! %d calc error %d\n", out, calc_error);
	}

	return out;
}

static void mse_work_timestamp(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_timing_ctrl *timing_ctrl;
	u64 now;
	int captured;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_timestamp);
	timing_ctrl = &instance->timing_ctrl;

	/* state is NOT RUNNABLE */
	if (!mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		mse_debug_state(instance);
		instance->f_work_timestamp = false;
		return;
	}

	/* capture timestamps */
	if (instance->f_ptp_capture) {
		mse_ptp_get_time(instance->ptp_index, &now);
		captured = mse_get_capture_timestamp(
				instance,
				now,
				!timing_ctrl->captured_timestamps);
		timing_ctrl->captured_timestamps += captured;
	}

	if (instance->f_mch_enable) {
		/* mch */
		media_clock_recovery(instance);
	}

	instance->f_work_timestamp = false;
}

static int create_avtp_timestamps(struct mse_instance *instance)
{
	int i;
	int create_size;
	struct mse_audio_config *audio = &instance->media_config.audio;
	struct mse_audio_info audio_info;
	u64 now = 0;
	u64 avtp_timestamp = 0;
	u32 delta_ts;
	u32 offset;
	unsigned long flags;

	instance->packetizer->get_audio_info(
		instance->index_packetizer,
		&audio_info);

	instance->avtp_timestamps_current = 0;

	offset = instance->f_ptp_capture ? instance->delay_time_ns : 0;

	create_size = audio->period_size / audio_info.sample_per_packet;

	instance->remain +=
		audio->period_size -
		create_size * audio_info.sample_per_packet;

	if (instance->remain >= audio_info.sample_per_packet) {
		instance->remain -= audio_info.sample_per_packet;
		create_size++;
	}

	if (create_size > CREATE_AVTP_TIMESTAMPS_MAX) {
		mse_err("too much packet, cannot create %d timestamps\n",
			create_size);
		return -EPERM;
	}

	delta_ts = audio_info.frame_interval_time;

	/* get timestamps from private table */
	mse_ptp_get_time(instance->ptp_index, &now);
	spin_lock_irqsave(&instance->lock_ques, flags);
	for (i = 0; i < create_size; i++) {
		tstamps_calc_tstamp(&instance->reader_create_avtp,
				    now,
				    offset,
				    delta_ts,
				    &avtp_timestamp);

		instance->avtp_timestamps[i] =
			avtp_timestamp + instance->max_transit_time_ns;
	}
	instance->avtp_timestamps_size = create_size;
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
	if (!instance->f_ptp_capture) {
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
	int wait_count = MSE_TX_RING_SIZE - MSE_TX_PACKET_NUM;
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
			instance->timestamp + instance->max_transit_time_ns;
	}

	while (buf->work_length < buf->buffer_size) {
		/* state is EXECUTE */
		if (mse_state_test(instance, MSE_STATE_EXECUTE)) {
			/* wait for packet buffer processed */
			if (!check_packet_remain(instance)) {
				read_lock_irqsave(&instance->lock_stream,
						  flags);
				if (!instance->f_streaming)
					queue_work(instance->wq_stream,
						   &instance->wk_stream);
				read_unlock_irqrestore(&instance->lock_stream,
						       flags);

				if (!wait_event_interruptible_timeout(
						instance->wait_wk_stream,
						check_packet_remain(instance),
						MSE_TIMEOUT_PACKETIZE))
					mse_debug("wait event timeouted\n");
			}
		}

		ret = mse_packet_ctrl_make_packet(
					instance->index_packetizer,
					buf->buffer,
					buf->buffer_size,
					instance->f_ptp_capture,
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
		if (ret != -EAGAIN)
			atomic_inc(&instance->done_buf_cnt);

		if (!instance->timer_interval)
			queue_work(instance->wq_packet, &instance->wk_callback);
	}
}

static void mse_inc_send_count(struct mse_timing_ctrl *timing_ctrl)
{
	if (timing_ctrl->start_time_count == timing_ctrl->send_count)
		timing_ctrl->send_count++;
}

static void mse_update_first_send_time(struct mse_instance *instance)
{
	struct mse_timing_ctrl *timing_ctrl = &instance->timing_ctrl;
	struct timestamp_set ts_last = { 0 };
	u64 now;
	unsigned long flags;
	int ret;

	mse_ptp_get_time(instance->ptp_index, &now);

	spin_lock_irqsave(&instance->lock_ques, flags);
	ret = tstamps_get_last_timestamp(&instance->tstamp_que,
					 &ts_last);
	spin_unlock_irqrestore(&instance->lock_ques, flags);

	if (ret < 0) {
		mse_err("timestamp not ready\n");
		instance->timing_ctrl.std_first_send_time = 0;
		return;
	}

	timing_ctrl->first_send_time = now;
	timing_ctrl->std_first_send_time = ts_last.std + now - ts_last.real;
}

static void mse_set_start_time_recovery(struct mse_instance *instance)
{
	struct mse_timing_ctrl *timing_ctrl = &instance->timing_ctrl;
	struct mse_packetizer_ops *packetizer = instance->packetizer;
	struct mse_start_time start_time;
	u64 now;

	timing_ctrl->f_ok = false;

	start_time.start_time = timing_ctrl->last_complete;
	start_time.capture_diff = NSEC_SCALE / instance->ptp_capture_freq;
	start_time.capture_freq = instance->ptp_capture_freq;

	packetizer->set_start_time(instance->index_packetizer,
				   &start_time);

	mse_ptp_get_time(instance->ptp_index, &now);
	mse_debug_tstamps("NG: use start time recover mode %u now %u\n",
			  start_time.start_time, (u32)now);
}

static void mse_set_start_time(struct mse_instance *instance)
{
	struct mse_timing_ctrl *timing_ctrl = &instance->timing_ctrl;
	struct mse_packetizer_ops *packetizer = instance->packetizer;
	struct mse_start_time start_time;
	struct timestamp_set ts_last = { 0 };
	u64 std_start, diff = 0;
	u64 offset_time;
	u64 now = 0;
	unsigned long flags;

	/* invalid timestamp, do reset start time */
	if (!timing_ctrl->std_first_send_time ||
	    instance->tstamp_que.f_discont) {
		mse_set_start_time_recovery(instance);
		return;
	}

	/* start normal sequence */
	timing_ctrl->start_time_count = timing_ctrl->send_count;

	std_start = timing_ctrl->std_first_send_time +
		(timing_ctrl->start_time_count - 1) * instance->timer_interval;

	spin_lock_irqsave(&instance->lock_ques, flags);
	tstamps_get_last_timestamp(&instance->tstamp_que, &ts_last);
	tstamps_get_timestamp_diff_average(&instance->tstamp_que, &diff, 10);
	spin_unlock_irqrestore(&instance->lock_ques, flags);

	offset_time = abs(std_start - ts_last.std) * diff *
		instance->ptp_capture_freq;
	offset_time = div64_u64(offset_time, NSEC_SCALE);
	if (std_start > ts_last.std)
		timing_ctrl->start_time = ts_last.real + offset_time;
	else
		timing_ctrl->start_time = ts_last.real - offset_time;

	if (!instance->f_get_first_packet) {
		mse_ptp_get_time(instance->ptp_index, &now);
		if (abs(timing_ctrl->start_time - now) >
		    PTP_START_ERROR_THRESHOLD(instance->timer_interval)) {
			if (timing_ctrl->f_ok) {
				mse_err("NG: diff error now %llu calc %llu\n",
					now, timing_ctrl->start_time);
			}
			timing_ctrl->start_time = now;
			timing_ctrl->f_ok = false;
		} else {
			if (!timing_ctrl->f_ok) {
				mse_info("OK: diff now %llu calc %llu\n",
					 now, timing_ctrl->start_time);
			}
			timing_ctrl->f_ok = true;
		}
	}

	start_time.start_time = timing_ctrl->start_time;
	start_time.capture_diff = diff;
	start_time.capture_freq = instance->ptp_capture_freq;

	packetizer->set_start_time(instance->index_packetizer, &start_time);

	mse_ptp_get_time(instance->ptp_index, &now);
	mse_debug_tstamps2("set start time %u now %u diff %lld count %d\n",
			   (u32)timing_ctrl->start_time,
			   (u32)now,
			   (s64)timing_ctrl->start_time - now,
			   timing_ctrl->start_time_count);
}

static void mse_work_depacketize(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_packet_ctrl *packet_buffer;
	struct mse_audio_info audio_info;
	struct mse_trans_buffer *buf;
	int received, ret = 0;
	u32 timestamps[128];
	int t_stored, i;
	int buf_cnt;
	unsigned long flags;

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

	packet_buffer = instance->packet_buffer;
	received = mse_packet_ctrl_check_packet_remain(packet_buffer);

	switch (instance->media->type) {
	case MSE_TYPE_ADAPTER_AUDIO:
		/* get AVTP packet payload */
		while (received) {
			/* set start time of period in media clock recovery */
			if (instance->ptp_timer_handle && !buf->work_length)
				mse_set_start_time(instance);

			/* get AVTP packet payload */
			ret = mse_packet_ctrl_take_out_packet(
				instance->index_packetizer,
				instance->temp_buffer[instance->temp_w],
				buf->buffer_size,
				timestamps,
				ARRAY_SIZE(timestamps),
				&t_stored,
				packet_buffer,
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
				&audio_info);

			/* store avtp timestamp */
			spin_lock_irqsave(&instance->lock_ques, flags);
			if (!instance->avtp_que.f_init && t_stored > 0) {
				tstamps_init(&instance->avtp_que,
					     "AVTP",
					     true,
					     audio_info.frame_interval_time);
			}
			for (i = 0; i < t_stored; i++) {
				tstamps_enq_tstamp(&instance->avtp_que,
						   timestamps[i]);
			}
			spin_unlock_irqrestore(&instance->lock_ques, flags);

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
				       0,
				       buf->buffer_size);
				instance->temp_len[instance->temp_w] = 0;

				mse_debug("temp_r=%d temp_w=%d\n",
					  instance->temp_r, instance->temp_w);

				mse_inc_send_count(&instance->timing_ctrl);
			}

			/* update received count */
			received = mse_packet_ctrl_check_packet_remain(
				packet_buffer);

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
						packet_buffer,
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
	if (mse_state_test(instance, MSE_STATE_STOPPING)) {
		if (instance->ptp_timer_handle ||
		    !atomic_read(&instance->trans_buf_cnt))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);

		if (atomic_inc_return(&instance->done_buf_cnt) != 1)
			atomic_dec(&instance->done_buf_cnt);
	}

	/* state is NOT EXECUTE */
	if (!mse_state_test(instance, MSE_STATE_EXECUTE))
		instance->f_depacketizing = false;
}

static void mse_work_callback(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mse_trans_buffer *buf;
	struct mse_timing_ctrl *timing_ctrl;
	bool has_valid_data;
	int size;
	int temp_r;
	unsigned long flags;

	instance = container_of(work, struct mse_instance, wk_callback);

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	adapter = instance->media;
	timing_ctrl = &instance->timing_ctrl;

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

	mse_debug_state(instance);

	size = buf->work_length;

	if (instance->tx) {
		if (IS_MSE_TYPE_MPEG2TS(adapter->type)) {
			u64 clock_90k = instance->mpeg2ts_clock_90k;
			u64 pcr_90k = instance->mpeg2ts_pcr_90k;

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

			if (instance->ptp_timer_handle) {
				if (!timing_ctrl->send_count)
					mse_update_first_send_time(instance);

				if (!timing_ctrl->start_time_count)
					timing_ctrl->send_count++;
			}

			has_valid_data = false;
			if (buf->media_buffer &&
			    instance->temp_w != temp_r) {
				if (instance->ptp_timer_handle) {
					has_valid_data = true;
				} else {
					if (instance->f_present ||
					    instance->temp_w >= MSE_DECODE_BUFFER_NUM_START_MIN) {
						instance->f_present = true;
						has_valid_data = true;
					}
				}
			}

			if (has_valid_data) {
				memcpy(buf->media_buffer,
				       instance->temp_buffer[temp_r],
				       size);

				if (instance->temp_w != temp_r) {
					instance->temp_r = (temp_r + 1) %
						MSE_DECODE_BUFFER_NUM;
				} else {
					/* clear temp buffer to use next timing */
					memset(instance->temp_buffer[temp_r],
					       0,
					       instance->temp_len[temp_r]);
					instance->temp_len[temp_r] = 0;
				}
			}
		}

		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
	}

	/* complete callback */
	instance->f_trans_start = false;
	if (instance->tx || size) {
		if (atomic_dec_not_zero(&instance->done_buf_cnt)) {
			mse_ptp_get_time(instance->ptp_index,
					 &timing_ctrl->last_complete);
			mse_trans_complete(instance, size);
		}
	}

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
	enum MSE_CRF_TYPE crf_type = instance->crf_type;

	/* cancel timestamp timer */
	hrtimer_cancel(&instance->tstamp_timer);

	/* cancel crf timer */
	hrtimer_cancel(&instance->crf_timer);

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
	if (instance->ptp_timer_handle) {
		if (mse_ptp_timer_cancel(instance->ptp_index,
					 instance->ptp_timer_handle) < 0)
			mse_err("The timer was still in use...\n");
	} else {
		hrtimer_cancel(&instance->timer);
	}

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
		up(&instance->sem_stopping);
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

	instance = container_of(arg, struct mse_instance, timer);

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

static u32 mse_ptp_timer_callback(void *arg)
{
	struct mse_instance *instance = arg;

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		return 0;
	}

	queue_work(instance->wq_packet, &instance->wk_callback);

	return 0;
}

static void mse_work_crf_send(struct work_struct *work)
{
	struct mse_instance *instance;
	int err, tsize, size, i;
	u64 timestamp;
	u64 timestamps[6];
	unsigned long flags;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_crf_send);

	/* state is NOT RUNNABLE */
	if (!mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		instance->f_crf_sending = false;
		return;
	}

	tsize = (!instance->f_ptp_capture) ?
		CRF_PTP_TIMESTAMPS : CRF_AUDIO_TIMESTAMPS;
	timestamp = 0;

	do {
		spin_lock_irqsave(&instance->lock_ques, flags);
		size = tstamps_get_tstamps_size(&instance->tstamp_que_crf);

		/* get Timestamps */
		mse_debug("size %d tsize %d\n", size, tsize);

		if (size < tsize) {
			spin_unlock_irqrestore(&instance->lock_ques, flags);
			break;
		}

		for (i = 0; i < tsize; i++) {
			tstamps_deq_tstamp(&instance->tstamp_que_crf,
					   &timestamp);

			if (instance->tx)
				timestamp += instance->max_transit_time_ns;

			if (instance->f_ptp_capture)
				timestamp += instance->delay_time_ns;

			timestamps[i] = timestamp;
		}
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

		/* state is RUNNABLE */
	} while (mse_state_test(instance, MSE_STATE_RUNNABLE));

	instance->f_crf_sending = false;
}

static void mse_work_crf_receive(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_audio_info audio_info;
	int err, count, i;
	u64 ptimes[6];
	unsigned long flags;

	struct mse_packetizer_ops *crf = &mse_packetizer_crf_tstamp_audio_ops;

	instance = container_of(work, struct mse_instance, wk_crf_receive);

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

		crf->get_audio_info(instance->crf_index, &audio_info);

		spin_lock_irqsave(&instance->lock_ques, flags);

		if (!instance->crf_que.f_init)
			tstamps_init(&instance->crf_que,
				     "CRF",
				     true,
				     audio_info.frame_interval_time);

		for (i = 0; i < count; i++)
			tstamps_enq_tstamp(&instance->crf_que, ptimes[i]);

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

	if (instance->f_wait_start_transmission) {
		queue_work(instance->wq_packet, &instance->wk_start_trans);
		instance->f_wait_start_transmission = false;
	}

	return HRTIMER_RESTART;
}

static int mse_get_capture_timestamp(struct mse_instance *instance,
				     u64 now,
				     bool is_first)
{
	int ret;
	int count, i;
	s64 diff;
	unsigned long flags;

	/* get timestamps */
	ret = mse_ptp_get_timestamps(instance->ptp_index,
				     instance->ptp_handle,
				     ARRAY_SIZE(instance->timestamps),
				     instance->timestamps);
	if (ret <= 0) {
		mse_warn("could not get timestamps ret=%d\n", ret);
		return 0;
	}

	count = ret;
	mse_debug("mse_ptp_get_timestamps %d %llu - %llu\n",
		  count,
		  instance->timestamps[0],
		  instance->timestamps[count - 1]);

	if (is_first) {
		/* skip older timestamp */
		for (i = 0; i < count; i++) {
			diff = (s64)(now - instance->timestamps[i]) -
				(s64)instance->delay_time_ns;
			if (diff <= 0)
				break;
		}

		if (i) {
			i--;
		} else {
			/* not enough timestamps */
			instance->delay_time_ns += diff;
		}
	} else {
		i = 0;
	}

	/* store timestamps */
	spin_lock_irqsave(&instance->lock_ques, flags);

	for (; i < count; i++) {
		tstamps_enq_tstamp(&instance->tstamp_que,
				   instance->timestamps[i]);
		tstamps_enq_tstamp(&instance->tstamp_que_crf,
				   instance->timestamps[i]);
	}

	spin_unlock_irqrestore(&instance->lock_ques, flags);

	return count;
}

static void mse_start_streaming_audio(struct mse_instance *instance, u64 now)
{
	struct mse_timing_ctrl *timing_ctrl = &instance->timing_ctrl;
	int i;
	int captured;
	struct mse_delay_time delay_time;

	if (!instance->tx) {
		instance->temp_w = 0;
		instance->temp_r = 0;
		for (i = 0; i < MSE_DECODE_BUFFER_NUM; i++)
			instance->temp_len[i] = 0;
	}

	/* Initialize delay time */
	mse_config_get_delay_time(instance->media->index, &delay_time);
	if (instance->tx)
		instance->delay_time_ns = delay_time.tx_delay_time_ns;
	else
		instance->delay_time_ns = delay_time.rx_delay_time_ns;

	/* Initialize timing control parameters */
	memset(timing_ctrl, 0, sizeof(*timing_ctrl));
	instance->f_wait_start_transmission = false;

	/* f_ptp_capture is true, capture timestamps */
	if (instance->f_ptp_capture) {
		captured = mse_get_capture_timestamp(
				instance,
				now,
				true);
		timing_ctrl->captured_timestamps += captured;
	}

	/* initialize timestamp readers */
	if (instance->ptp_timer_handle)
		tstamps_reader_init(&instance->reader_ptp_start_time,
				    &instance->tstamp_que,
				    "PERIOD",
				    false);

	if (instance->f_mch_enable)
		tstamps_reader_init(&instance->reader_mch,
				    &instance->tstamp_que,
				    "MCH",
				    true);

	tstamps_reader_init(&instance->reader_create_avtp,
			    &instance->tstamp_que,
			    "CREATE_AVTP",
			    true);

	/* f_ptp_capture is true or f_mch_enable is enable */
	if (instance->f_ptp_capture || instance->f_mch_enable) {
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

	/* receive clock using CRF */
	if (instance->crf_type == MSE_CRF_TYPE_RX) {
		queue_work(instance->wq_crf_packet,
			   &instance->wk_crf_receive);
	}
}

static void mse_tstamp_init(struct mse_instance *instance, u64 now)
{
	instance->timestamp = now;

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
	if (instance->ptp_timer_handle) {
		instance->ptp_timer_start = 0;
	} else {
		if (instance->timer_interval)
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

static u64 calc_pcr_progress_by_bitrate(u64 bitrate, size_t size)
{
	if (!bitrate)
		bitrate = MSE_DEFAULT_BITRATE;

	/* (size * 8 * 90kHz) / bitrate */
	return div64_u64((u64)(size * 8 * 90000), bitrate);
}

static void mpeg2ts_adjust_pcr(struct mse_instance *instance,
			       size_t size)
{
	u64 bitrate = instance->media_config.mpeg2ts.bitrate;
	u64 progress;

	if (instance->mpeg2ts_clock_90k == MPEG2TS_PCR90K_INVALID)
		instance->mpeg2ts_clock_90k = 0;

	if (instance->mpeg2ts_pcr_90k == MPEG2TS_PCR90K_INVALID)
		instance->mpeg2ts_pcr_90k = 0;

	instance->mpeg2ts_pre_pcr_90k = instance->mpeg2ts_pcr_90k;

	progress = calc_pcr_progress_by_bitrate(bitrate, size);

	instance->mpeg2ts_pcr_90k += progress;

	/* recovery */
	if (compare_pcr(instance->mpeg2ts_pcr_90k,
			instance->mpeg2ts_clock_90k))
		instance->mpeg2ts_pcr_90k =
			instance->mpeg2ts_clock_90k + progress;

	mse_debug("clock_90k=%llu pcr=%llu\n",
		  instance->mpeg2ts_clock_90k, instance->mpeg2ts_pcr_90k);
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
	mse_debug("flush %zu bytes, before stopping.\n", mpeg2ts->buffer_size);
#endif

	/* add mpeg2ts buffer to proc buffer list*/
	mpeg2ts->work_length = 0;
	list_add_tail(&mpeg2ts->list, &instance->proc_buf_list);
	atomic_inc(&instance->trans_buf_cnt);
	instance->mpeg2ts_buffer_idx = (idx + 1) % MSE_MPEG2TS_BUF_NUM;

	mpeg2ts_adjust_pcr(instance, mpeg2ts->buffer_size);
}

static int mpeg2ts_buffer_write(struct mse_instance *instance,
				struct mse_trans_buffer *buf)
{
	bool trans_start;
	bool force_flush = false;
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

	if (request_size + buffer_size >= MSE_MPEG2TS_BUF_THRESH)
		force_flush = true;

#ifdef DEBUG
	if (force_flush)
		mse_debug("flush %zu bytes.\n", mpeg2ts->buffer_size);
#endif

	trans_start = check_mpeg2ts_pcr(instance, mpeg2ts);

	if (!trans_start && !force_flush) {
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

	if (!force_flush)
		return 0;

	if (instance->mpeg2ts_clock_90k != MPEG2TS_PCR90K_INVALID)
		mpeg2ts_adjust_pcr(instance, mpeg2ts->buffer_size);

	return 0;
}

static u64 ptp_timer_update_start_timing(struct mse_instance *instance, u64 now)
{
	u64 ptp_timer_start, next_time;
	s64 diff, update;
	u64 error_thresh;
	u32 interval, offset;
	int ret;
	unsigned long flags;

	ptp_timer_start = instance->ptp_timer_start;
	offset = instance->delay_time_ns;
	interval = instance->timer_interval;
	error_thresh = PTP_TIMER_ERROR_THRESHOLD(interval);

	/* if NOT set ptp_timer_start, then init ptp_timer_start */
	if (!ptp_timer_start)
		ptp_timer_start = now;

	spin_lock_irqsave(&instance->lock_ques, flags);
	ret = tstamps_calc_tstamp(&instance->reader_ptp_start_time,
				  now,
				  offset,
				  interval,
				  &next_time);
	spin_unlock_irqrestore(&instance->lock_ques, flags);

	update = now - next_time;
	diff = ptp_timer_start - next_time;

	/* check 'update' and 'diff' within error threshold */
	if ((abs(update) < error_thresh) && (abs(diff) < error_thresh))
		ptp_timer_start = next_time;

	ptp_timer_start += interval;

	mse_debug_tstamps2("start %u next %llu now %llu update %llu diff %lld\n",
			   (u32)ptp_timer_start, next_time, now, update, diff);

	instance->ptp_timer_start = ptp_timer_start;

	return ptp_timer_start;
}

static void mse_work_start_transmission(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	struct mse_trans_buffer *buf;
	struct mse_timing_ctrl *timing_ctrl;
	int ret;
	u64 now, ptp_timer_start;
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

			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
		}

		return;
	} else if (instance->ptp_timer_handle) {
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			spin_unlock_irqrestore(&instance->lock_buf_list,
					       flags);
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
			return;
		}
	}

	timing_ctrl = &instance->timing_ctrl;
	if (instance->ptp_timer_handle &&
	    timing_ctrl->captured_timestamps < PTP_SYNC_LOCK_THRESHOLD) {
		spin_unlock_irqrestore(&instance->lock_buf_list, flags);
		instance->f_wait_start_transmission = true;

		return;
	}

	list_move_tail(&buf->list, &instance->proc_buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	mse_debug("index=%d buffer=%p buffer_size=%zu\n",
		  instance->index_media, buf->media_buffer,
		  buf->buffer_size);

	/* update timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index, &now);
	instance->timestamp = now;

	if (!instance->f_ptp_capture) {
		spin_lock_irqsave(&instance->lock_ques, flags);
		/* store ptp timestamp to AVTP and CRF */
		tstamps_enq_tstamp(&instance->tstamp_que, now);
		tstamps_enq_tstamp(&instance->tstamp_que_crf, now);
		spin_unlock_irqrestore(&instance->lock_ques, flags);
	} else if (instance->ptp_timer_handle) {
		ptp_timer_start = ptp_timer_update_start_timing(instance, now);
		mse_debug("mse_ptp_timer_start %u now %u\n",
			  (u32)ptp_timer_start,
			  (u32)now);
		if (mse_ptp_timer_start(instance->ptp_index,
					instance->ptp_timer_handle,
					(u32)ptp_timer_start) < 0) {
			mse_err("mse_ptp_timer_start error\n");
			return;
		}
	}

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

	down(&instance->sem_stopping);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		up(&instance->sem_stopping);
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		up(&instance->sem_stopping);
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
	}

	up(&instance->sem_stopping);

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

	down(&instance->sem_stopping);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		up(&instance->sem_stopping);
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		up(&instance->sem_stopping);
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
	}

	up(&instance->sem_stopping);

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

	down(&instance->sem_stopping);

	/* state is CLOSE */
	if (mse_state_test(instance, MSE_STATE_CLOSE)) {
		up(&instance->sem_stopping);
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		up(&instance->sem_stopping);
		mse_err("instance is busy. index=%d\n", index);
		return -EBUSY;
	}

	up(&instance->sem_stopping);

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
	bool tx = instance->tx;
	enum MSE_CRF_TYPE crf_type = instance->crf_type;

	if (!instance->f_mch_enable) {
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
	dev_name = (char *)
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
	dev_name = (char *)
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
	void *ptp_timer_handle;
	int ret, i, index, err = 0;
	long link_speed;
	struct mch_ops *m_ops = NULL;
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
	sema_init(&instance->sem_stopping, 1);

	mutex_unlock(&mse->mutex_open);

	network_device = &adapter->config.network_device;

	/* search network adapter name for configuration value */
	for (i = 0; i < ARRAY_SIZE(mse->network_table); i++) {
		network = mse->network_table[i];
		if (!network)
			continue;

		if (!mse_compare_param_key((char *)network_device->module_name,
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

	/* open network adapter */
	if (tx)
		dev_name = (char *)network_device->device_name_tx;
	else
		dev_name = (char *)network_device->device_name_rx;

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

	/* open packetizer */
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
	/* for timestamp */
	instance->wq_tstamp = create_singlethread_workqueue("mse_tstampq");

	hrtimer_init(&instance->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	instance->timer_interval = 0;
	instance->timer.function = &mse_timer_callback;

	if (IS_MSE_TYPE_AUDIO(adapter->type)) {
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
	if (ret < 0) {
		mse_err("cannot get configurations\n");
		err = ret;

		goto error_cannot_get_default_config;
	}

	instance->ptp_index = mse_ptp_get_first_index();

	/* open PTP capture and PTP Timer */
	if (instance->f_ptp_capture) {
		/* ptp open */
		instance->ptp_handle = mse_ptp_open(instance->ptp_index);
		if (!instance->ptp_handle) {
			mse_err("cannot mse_ptp_open()\n");
			err = -ENODEV;

			goto error_cannot_open_ptp;
		}

		mse_info("mse_ptp_capture_start %d %p %d %zu\n",
			 instance->ptp_index,
			 instance->ptp_handle,
			 instance->ptp_clock_ch,
			 ARRAY_SIZE(instance->timestamps));

		ret = mse_ptp_capture_start(instance->ptp_index,
					    instance->ptp_handle,
					    instance->ptp_clock_ch,
					    ARRAY_SIZE(instance->timestamps));
		if (ret < 0) {
			mse_err("cannot mse_ptp_capture_start()\n");
			err = ret;

			goto error_cannot_ptp_capture_start;
		}

		ptp_timer_handle = mse_ptp_timer_open(instance->ptp_index,
						      mse_ptp_timer_callback,
						      instance);
		if (!ptp_timer_handle) {
			mse_warn("cannot open ptp_timer, fallback using hires timer\n");
		} else {
			tstamps_reader_init(&instance->reader_ptp_start_time,
					    &instance->tstamp_que,
					    "PERIOD",
					    false);
		}

		instance->ptp_timer_handle = ptp_timer_handle;
	}

	if (check_mch_config(instance)) {
		mse_err("media clock recovery config is invalid mch_config.enable=%d media_audio_config.crf_type=%d\n",
			instance->f_mch_enable, instance->crf_type);
		err = -EINVAL;
		goto error_mch_config_invalid;
	}

	if (instance->f_mch_enable) {
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

		instance->mch_handle = m_ops->open();
		if (!instance->mch_handle) {
			mse_err("mch open error.\n");
			err = -EINVAL;

			goto error_mch_cannot_open;
		}

		tstamps_reader_init(&instance->reader_mch,
				    &instance->tstamp_que,
				    "MCH",
				    true);
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

	/* init packetizer */
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
		m_ops->close(instance->mch_handle);

error_mch_cannot_open:
error_mch_not_found:
error_mch_config_invalid:
	if (instance->ptp_timer_handle)
		mse_ptp_timer_close(instance->ptp_index,
				    instance->ptp_timer_handle);
	instance->ptp_timer_handle = NULL;

	if (instance->ptp_handle)
		mse_ptp_capture_stop(instance->ptp_index,
				     instance->ptp_handle);

error_cannot_ptp_capture_start:
	if (instance->ptp_handle)
		mse_ptp_close(instance->ptp_index, instance->ptp_handle);
	instance->ptp_handle = NULL;

error_cannot_open_ptp:
error_cannot_get_default_config:
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
	struct mch_ops *m_ops;
	int err = -EINVAL;
	int ret;
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
		wait_for_completion_timeout(&instance->completion_stop,
					    MSE_TIMEOUT_CLOSE);

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

	if (instance->ptp_timer_handle) {
		if (mse_ptp_timer_close(instance->ptp_index,
					instance->ptp_timer_handle) < 0)
			mse_err("cannot mse_ptp_timer_close()\n");

		instance->ptp_timer_handle = NULL;
	}

	if (instance->ptp_handle) {
		if (mse_ptp_capture_stop(instance->ptp_index,
					 instance->ptp_handle) < 0)
			mse_err("cannot mse_ptp_capture_stop()\n");

		if (mse_ptp_close(instance->ptp_index,
				  instance->ptp_handle) < 0)
			mse_err("cannot mse_ptp_close()\n");

		instance->ptp_handle = NULL;
	}

	if (instance->mch_index >= 0) {
		m_ops = mse->mch_table[instance->mch_index];
		ret = m_ops->close(instance->mch_handle);
		if (ret < 0)
			mse_err("mch close error(%d).\n", ret);
	}

	mpeg2ts_buffer_free(instance);

	/* set table */
	memset(instance, 0, sizeof(*instance));
	instance->used_f = false;
	instance->state = MSE_STATE_CLOSE;
	instance->index_media = MSE_INDEX_UNDEFINED;
	instance->index_network = MSE_INDEX_UNDEFINED;
	instance->crf_index_network = MSE_INDEX_UNDEFINED;
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
	u64 now = 0;
	unsigned long flags;
	u32 std;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return err;
	}

	mse_debug("index=%d\n", index);

	instance = &mse->instance_table[index];

	mse_debug_state(instance);

	down(&instance->sem_stopping);

	/* state is RUNNABLE */
	if (mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		up(&instance->sem_stopping);
		return 0;
	}

	/* state is NOT OPEN */
	if (!mse_state_test(instance, MSE_STATE_OPEN)) {
		up(&instance->sem_stopping);
		return -EPERM;
	}

	write_lock_irqsave(&instance->lock_state, flags);
	err = mse_state_change(instance, MSE_STATE_IDLE);
	write_unlock_irqrestore(&instance->lock_state, flags);
	if (err) {
		up(&instance->sem_stopping);
		return err;
	}

	/* get timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index, &now);
	mse_tstamp_init(instance, now);

	/* initialize timestamp queues */
	if (instance->f_ptp_capture)
		std = NSEC_SCALE / instance->ptp_capture_freq;
	else
		std = instance->timer_interval;

	tstamps_init(&instance->tstamp_que, "TSTAMP_AVTP", false, std);
	tstamps_init(&instance->tstamp_que_crf, "TSTAMP_CRF", false, std);

	if (IS_MSE_TYPE_AUDIO(instance->media->type))
		mse_start_streaming_audio(instance, now);

	mse_start_streaming_common(instance);

	up(&instance->sem_stopping);

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

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	/* state is RUNNABLE */
	if (mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		down(&instance->sem_stopping);
		queue_work(instance->wq_packet, &instance->wk_stop_streaming);
	}

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
	int err;

	/* allocate device data */
	mse = kzalloc(sizeof(*mse), GFP_KERNEL);
	if (!mse)
		return -ENOMEM;

	spin_lock_init(&mse->lock_tables);
	mutex_init(&mse->mutex_open);

	/* register platform device */
	mse->pdev = platform_device_register_simple("mse", -1, NULL, 0);
	if (IS_ERR(mse->pdev)) {
		err = PTR_ERR(mse->pdev);
		mse->pdev = NULL;
		mse_err("Failed to register platform device. ret=%d\n", err);
		goto error;
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
		err = PTR_ERR(mse->class);
		mse->class = NULL;
		mse_err("failed class_create() ret=%d\n", err);
		goto error;
	}
#endif

	/* init ioctl device */
	major = mse_ioctl_init(major, mse_instance_max);
	if (major < 0) {
		err = major;
		goto error;
	}

	/* init table */
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		mse->instance_table[i].state = MSE_STATE_CLOSE;
		mse->instance_table[i].index_media = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].index_network = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].crf_index_network = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].index_packetizer = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].mch_index = MSE_INDEX_UNDEFINED;
		mse->instance_table[i].ptp_index = MSE_INDEX_UNDEFINED;
	}

	for (i = 0; i < ARRAY_SIZE(mse->media_table); i++)
		mse->media_table[i].index = MSE_INDEX_UNDEFINED;

	mse_debug("success\n");

	return 0;

error:
	if (mse) {
		if (mse->class)
			class_destroy(mse->class);

		if (mse->pdev)
			platform_device_unregister(mse->pdev);

		kfree(mse);
	}

	return err;
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
