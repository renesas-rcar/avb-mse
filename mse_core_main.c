/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2018,2021 Renesas Electronics Corporation

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

#define MSE_TIMEOUT_CLOSE             (msecs_to_jiffies(5000)) /* 5secs */
#define MSE_TIMEOUT_PACKETIZE         (msecs_to_jiffies(16))   /* 16msecs */
#define MSE_TIMEOUT_PACKETIZE_MPEG2TS (msecs_to_jiffies(2000)) /* 2secs */

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

#define PTP_TIMESTAMP_INVALID   ((u64)-1)
#define PTP_TIME_DIFF_S32(a, b) ((s32)((u32)(a) - (u32)(b)))

/* judge error 5% */
#define PTP_TIMER_ERROR_THRESHOLD(x)  div64_u64((x) * 5, 100)

/* judge error 2 periods */
#define PTP_START_ERROR_THRESHOLD(x)  ((x) * 2)

/* judge error 10% capture syncronized */
#define PTP_SYNC_ERROR_THRESHOLD(x)  div64_u64((x) * 10, 100)

/* judge error 10% valid to mch */
#define MCH_ERROR_THRESHOLD(x)  div64_u64((x) * 10, 100)

#define PTP_SYNC_LOCK_THRESHOLD (3)

/* judge start 500us */
#define PTP_TIMER_START_THRESHOLD (500000)

#define CRF_TIMER_INTERVAL   (20 * 1000000)  /* 20ms */
#define CRF_PTP_TIMESTAMPS   (1)     /* timestamps per CRF packet using ptp */
#define CRF_AUDIO_TIMESTAMPS (6)     /* audio timestamps per CRF packet */

#define CREATE_AVTP_TIMESTAMPS_MAX   (4096)

#define MSE_DECODE_BUFFER_NUM (8)
#define MAX_DECODE_SIZE       (8192) /* ALSA Period byte size */

#define MSE_TRANS_BUF_NUM (5) /* size of transmission buffer array */
#define MSE_TRANS_BUF_ACCEPTABLE (MSE_TRANS_BUF_NUM - 1)

#define MSE_MPEG2TS_BUF_NUM  (MSE_TRANS_BUF_NUM)
#define MSE_MPEG2TS_BUF_SIZE (512U * 1024U)
#define MSE_MPEG2TS_BUF_THRESH (188U * 192U * 14U)

#define mbit_to_bit(mbit)     (mbit * 1000000)

#define MPEG2TS_TS_SIZE           (188)
#define MPEG2TS_SYNC              (0x47)
#define MPEG2TS_M2TS_OFFSET       (4)
#define MPEG2TS_M2TS_SIZE         (MPEG2TS_M2TS_OFFSET + MPEG2TS_TS_SIZE)
#define MPEG2TS_M2TS_FREQ         (27000000)    /* 27MHz */
#define MPEG2TS_M2TS_DIFF(a, b)   (((a) - (b)) & 0x3fffffff)
#define MPEG2TS_PCR_PID_IGNORE    (MSE_CONFIG_PCR_PID_MAX)
#define MPEG2TS_PCR27M_BITS       (42)
#define MPEG2TS_PCR27M_INVALID    (BIT_ULL(MPEG2TS_PCR27M_BITS))
#define MPEG2TS_PCR27M_THRESHOLD  (27000000 / 100) /* 10ms */

#define MPEG2TS_PCR27M_DIFF(a, b) (((a) - (b)) & (BIT_ULL(MPEG2TS_PCR27M_BITS) - 1))
#define MPEG2TS_PCR27M_TO_NS(pcr) ((pcr) * (NSEC_SCALE / 1000000) / 27)

#define MPEG2TS_NS_TO_M2TS(ns)     (MPEG2TS_M2TS_FREQ / 1000000 * (ns) / 1000)
#define MPEG2TS_WAIT_INTERVAL      MPEG2TS_NS_TO_M2TS(10000000) /* 10ms */
#define MPEG2TS_INTERVAL_THRESHOLD MPEG2TS_NS_TO_M2TS(PTP_TIMER_START_THRESHOLD * 2) /* 1ms */
#define MPEG2TS_MAX_WAIT_TIME      (1500000000) /* 1.5secs */

#define atomic_dec_not_zero(v)  atomic_add_unless((v), -1, 0)

/**
 * @brief main data for Adapter
 */
struct mse_adapter {
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
	rwlock_t rwlock;
	const char *name;
	bool f_init;
	bool f_sync;
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
	u64 out_time;
	u64 out_std;
	u64 out_offset;
	struct timestamp_queue *que;
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
	/** @brief AVTP timestamp for launch timer */
	u64 launch_avtp_timestamp;
	/** @brief private data of media adapter */
	void *private_data;
	/** @brief callback function to media adapter */
	int (*mse_completion)(void *priv, int size);

	struct list_head list;
};

/** @brief information for adjust transmission time of packet */
struct mse_wait_packet {
	/** @brief AVTP timestamp for launch timer */
	u64 launch_avtp_timestamp;
	/** @brief release position of packet buffer */
	int release_p;

	struct list_head list;
};

/** @brief instance by related adapter */
struct mse_instance {
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
	/** @brief list of transmission buffer for adjust output timing */
	struct list_head wait_buf_list;
	/** @brief list of transmission buffer for processing done */
	struct list_head done_buf_list;
	/** @brief index of transmission buffer array */
	int trans_idx;
	/** @brief count of buffers is not completed */
	atomic_t trans_buf_cnt;
	/** @brief count of buffers is completed */
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
	struct timestamp_queue tstamp_que;
	struct timestamp_queue tstamp_que_crf;
	struct timestamp_queue crf_que;
	struct timestamp_queue avtp_que;
	struct timestamp_reader reader_create_avtp;
	struct timestamp_reader reader_mch;
	struct timestamp_reader reader_ptp_start_time;

	/** @brief timestamp(nsec) */
	u32 timestamp;
	/** @brief start streaming flag */
	bool f_start_stream;

	/** @brief start timing using ptp_timer */
	u64 ptp_timer_start;

	/** @brief packet buffer */
	struct mse_packet_ctrl *packet_buffer;
	/** @brief array of wait packet */
	struct mse_wait_packet *wait_packet;
	/** @brief index of wait packet array */
	int wait_packet_idx;
	/** @brief list of wait packet */
	struct list_head wait_packet_list;

	/** @brief AVTP timestampes */
	u64 avtp_timestamps[CREATE_AVTP_TIMESTAMPS_MAX];
	int avtp_timestamps_size;
	int avtp_timestamps_current;
	/** @brief stopping streaming flag */
	bool f_stopping;
	/** @brief continue streaming flag */
	bool f_continue;
	bool f_depacketizing;
	bool f_completion;
	bool f_work_timestamp;
	bool f_wait_start_transmission;

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

	/** @brief MCH & CRF Settings **/
	int f_ptp_capture;
	int f_ptp_timer;
	int f_mch_enable;
	int ptp_clock_device;
	int ptp_clock_ch;
	int ptp_capture_freq;
	int media_capture_freq;
	enum MSE_CRF_TYPE crf_type;
	u32 max_transit_time_ns;
	u32 delay_time_ns;
	u32 capture_delay_time_ns;
	int remain;
	enum MSE_TRANSMIT_MODE transmit_mode;

	bool f_present;
	bool f_timer_started;
	int captured_timestamps;
	bool f_get_first_packet;

	/** @brief packet buffer */
	int crf_index_network;
	struct mse_packet_ctrl *crf_packet_buffer;
	bool f_crf_sending;

	/** @brief media clock recovery work */
	u64 temp_ts[PTP_TIMESTAMPS_MAX];
	u64 timestamps[PTP_TIMESTAMPS_MAX];
	struct mch_timestamp ts[PTP_TIMESTAMPS_MAX];

	/** @brief mpeg2ts buffer  */
	bool f_first_pcr;
	int mpeg2ts_pre_pcr_pid;
	u64 mpeg2ts_pcr_27m;
	u64 mpeg2ts_pre_pcr_27m;
	u32 mpeg2ts_timestamp_base;
	u32 mpeg2ts_m2ts_base;

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

	/** @brief lock for resource tables */
	spinlock_t lock_network_table;  /* lock for network adapter table */
	spinlock_t lock_media_table;    /* lock for media adapter table */
	spinlock_t lock_instance_table; /* lock for core instance table */
	spinlock_t lock_ptp_table;      /* lock for ptp table */
	spinlock_t lock_mch_table;      /* lock for mch table */

	DECLARE_BITMAP(mse_instance_map, MSE_INSTANCE_MAX);
	DECLARE_BITMAP(mse_media_map, MSE_ADAPTER_MEDIA_MAX);

	/* @brief device class */
	struct class *class;

	struct mse_adapter_network_ops *network_table[MSE_ADAPTER_NETWORK_MAX];
	struct mse_adapter *media_table[MSE_ADAPTER_MEDIA_MAX];
	struct mse_instance *instance_table[MSE_INSTANCE_MAX];
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
static bool mse_check_ptp_timer_threshold(struct mse_instance *instance,
					  u32 timestamp);
static int mse_get_capture_timestamp(struct mse_instance *instance,
				     u64 now,
				     bool is_first);
static u64 ptp_timer_update_start_timing(struct mse_instance *instance,
					 u64 now);

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
	pr_debug("%s: state=%s flags=[%d %d %d %d %d]\n",
		 func, mse_state_stringfy(instance->state),
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
	int index_media = instance->media->index;
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

static inline bool mse_is_buffer_empty(struct mse_instance *instance)
{
	return !atomic_read(&instance->trans_buf_cnt);
}

static bool compare_pcr(u64 a, u64 b)
{
	u64 diff;

	diff = MPEG2TS_PCR27M_DIFF(a, b);
	if (diff < MPEG2TS_PCR27M_THRESHOLD) {
		mse_debug("PCR is within threshold. %llu - %llu\n",
			  a, b);
		return false;
	}

	return (diff < BIT_ULL(MPEG2TS_PCR27M_BITS - 1));
}

#if !(defined(CONFIG_MSE_SYSFS) || defined(CONFIG_MSE_IOCTL))
static inline int mse_create_config_device(struct mse_adapter *adapter) { return 0; }
static inline void mse_delete_config_device(struct mse_adapter *adapter) { return; }
#else
static void mse_device_release(struct device *dev)
{
	mse_debug("do noting\n");
}

static int mse_create_config_device(struct mse_adapter *adapter)
{
	struct device *device;
	int index_media = adapter->index;
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

static void mse_delete_config_device(struct mse_adapter *adapter)
{
	mse_debug("START\n");

	mse_ioctl_unregister(adapter->index);
	device_unregister(&adapter->device);
}
#endif

static void mse_get_default_config(struct mse_instance *instance,
				   struct mse_adapter *media,
				   bool tx)
{
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

	avtp_tx_param = media->config.avtp_tx_param;

	memcpy(network->dest_addr, avtp_tx_param.dst_mac,
	       sizeof(network->dest_addr));
	memcpy(network->source_addr, avtp_tx_param.src_mac,
	       sizeof(network->source_addr));
	network->priority = avtp_tx_param.priority;
	network->vlanid = avtp_tx_param.vlan;
	network->uniqueid = avtp_tx_param.uniqueid;

	avtp_rx_param = media->config.avtp_rx_param;

	memcpy(network->streamid, avtp_rx_param.streamid,
	       sizeof(network->streamid));

	delay_time = media->config.delay_time;

	instance->max_transit_time_ns = delay_time.max_transit_time_ns;
	if (tx)
		instance->delay_time_ns = delay_time.tx_delay_time_ns;
	else
		instance->delay_time_ns = delay_time.rx_delay_time_ns;

	switch (media->type) {
	case MSE_TYPE_ADAPTER_VIDEO:
		video_config = media->config.media_video_config;

		video->fps.numerator = video_config.fps_numerator;
		video->fps.denominator = video_config.fps_denominator;
		video->bitrate = video_config.bitrate;
		video->bytes_per_frame = video_config.bytes_per_frame;

		if (!tx && delay_time.rx_delay_time_ns != 0)
			instance->f_ptp_timer = true;
		else
			instance->f_ptp_timer = false;

		break;

	case MSE_TYPE_ADAPTER_MPEG2TS:
		mpeg2ts_config = media->config.media_mpeg2ts_config;

		mpeg2ts->tspackets_per_frame =
			mpeg2ts_config.tspackets_per_frame;
		mpeg2ts->bitrate = mpeg2ts_config.bitrate;
		mpeg2ts->pcr_pid = mpeg2ts_config.pcr_pid;
		mpeg2ts->transmit_mode = mpeg2ts_config.transmit_mode;

		if (tx && mpeg2ts->transmit_mode != MSE_TRANSMIT_MODE_BITRATE)
			instance->f_ptp_timer = true;
		else
			instance->f_ptp_timer = false;

		break;

	case MSE_TYPE_ADAPTER_AUDIO:
		audio_config = media->config.media_audio_config;

		audio->samples_per_frame = audio_config.samples_per_frame;

		ptp_config = media->config.ptp_config;

		instance->ptp_clock_device = 0;
		instance->f_ptp_capture =
			(ptp_config.type == MSE_PTP_TYPE_CAPTURE);
		instance->f_ptp_timer = instance->f_ptp_capture;
		instance->ptp_clock_ch = ptp_config.capture_ch;
		instance->ptp_capture_freq = ptp_config.capture_freq;

		mch_config = media->config.mch_config;

		instance->f_mch_enable = mch_config.enable;
		instance->crf_type = audio_config.crf_type;
		instance->media_capture_freq =
			(ptp_config.recovery_capture_freq ==
			 MSE_RECOVERY_CAPTURE_FREQ_NOT_FIXED);

		crf_tx = media->config.avtp_tx_param_crf;

		memcpy(crf_network->dest_addr, crf_tx.dst_mac,
		       sizeof(crf_network->dest_addr));
		memcpy(crf_network->source_addr, crf_tx.src_mac,
		       sizeof(crf_network->source_addr));
		crf_network->priority = crf_tx.priority;
		crf_network->vlanid = crf_tx.vlan;
		crf_network->uniqueid = crf_tx.uniqueid;

		crf_rx = media->config.avtp_rx_param_crf;

		memcpy(crf_network->streamid, crf_rx.streamid,
		       sizeof(crf_network->streamid));

		break;

	default:
		/* fall through */
		break;
	}

	return;
}

/*
 * PTP related functions
 */
static struct mse_ptp_ops mse_ptp_ops_dummy = {
	.owner = NULL,
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

static struct mse_ptp_ops *mse_ptp_find_ops(int index)
{
	if ((index < 0) || (index >= MSE_PTP_MAX))
		return &mse_ptp_ops_dummy;
	else
		return mse->ptp_table[index];
}

static void mse_ptp_put_index(int index)
{
	struct mse_ptp_ops *p_ops = mse_ptp_find_ops(index);

	module_put(p_ops->owner);
}

static int mse_ptp_get_first_index(void)
{
	int i;
	struct mse_ptp_ops *p_ops;

	for (i = 0; i < MSE_PTP_MAX; i++) {
		if (mse->ptp_table[i]) {
			p_ops = mse_ptp_find_ops(i);
			if (!try_module_get(p_ops->owner)) {
				mse_err("try_module_get() fail\n");

				return MSE_INDEX_UNDEFINED;
			}

			return i;
		}
	}

	return MSE_INDEX_UNDEFINED; /* not found, use dummy ops */
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

static void mse_trans_complete(struct mse_instance *instance,
			       struct list_head *buf_list,
			       int size)
{
	struct mse_trans_buffer *buf;

	buf = list_first_entry_or_null(buf_list, struct mse_trans_buffer, list);
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
	list_for_each_entry_safe(buf, buf1, &instance->done_buf_list, list)
		callback_completion(buf, size);
	list_for_each_entry_safe(buf, buf1, &instance->wait_buf_list, list)
		callback_completion(buf, size);
	list_for_each_entry_safe(buf, buf1, &instance->proc_buf_list, list)
		callback_completion(buf, size);
	atomic_set(&instance->done_buf_cnt, 0);

	/* free all buf from TRANS buf list */
	list_for_each_entry_safe(buf, buf1, &instance->trans_buf_list, list)
		callback_completion(buf, size);
	atomic_set(&instance->trans_buf_cnt, 0);
}

static void mse_work_stream_common(struct mse_instance *instance)
{
	int index_network;
	struct mse_packet_ctrl *packet_buffer;
	struct mse_adapter_network_ops *network;
	int err = 0;
	unsigned long flags;

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

static void mse_work_stream_mpeg2ts_tx(struct mse_instance *instance)
{
	struct mse_adapter_network_ops *network;
	struct mse_packet_ctrl *packet_buffer;
	unsigned long flags;
	int index_network;
	int err = 0;

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

	/* while data is remained */
	do {
		/* request send packet */
		err = mse_packet_ctrl_send_packet_wait(index_network,
						       packet_buffer,
						       network);
		if (err < 0) {
			mse_err("send error %d\n", err);
			break;
		}

		wake_up_interruptible(&instance->wait_wk_stream);
	} while (mse_packet_ctrl_check_packet_remain_wait(packet_buffer));

	write_lock_irqsave(&instance->lock_stream, flags);
	instance->f_streaming = false;
	mse_debug_state(instance);
	write_unlock_irqrestore(&instance->lock_stream, flags);

	mse_debug("END\n");
}

static void mse_work_stream(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_stream);

	if (IS_MSE_TYPE_MPEG2TS(instance->media->type) && instance->tx)
		mse_work_stream_mpeg2ts_tx(instance);
	else
		mse_work_stream_common(instance);
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
	rwlock_init(&que->rwlock);
	que->name = name;
	que->f_init = true;
	que->head = 0;
	que->tail = 0;
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
	reader->que = que;
	reader->out_time = 0;
	reader->out_std = 0;
	reader->out_offset = 0;

	mse_debug_tstamps2("%s\n", reader->name);
}

static int tstamps_get_last_timestamp_nolock(struct timestamp_queue *que,
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

/* calculate timestamp from std_time */
static int tstamps_calc_tstamp_nolock(struct timestamp_reader *reader,
				      u64 base,
				      u32 offset,
				      u64 interval,
				      u64 *timestamp)
{
	struct timestamp_queue *que = reader->que;
	struct timestamp_set ts_last = { 0 };
	struct timestamp_set ts1, ts2;
	int pos1, pos2;
	u64 t;
	s64 ts21_diff;
	s64 interval_thresh;
	bool f_first = false;

	if (!que)
		return -1;

	if (que->f_discont)
		reader->f_out_ok = false;

	/* first & not received */
	if (!que->f_sync) {
		if (reader->out_time == 0)
			reader->out_time = base;
		else
			reader->out_time += interval;

		*timestamp = reader->out_time;
		return -1;
	}

	/* first sync output */
	if (!reader->f_out_ok) {
		tstamps_get_last_timestamp_nolock(que, &ts_last);
		reader->out_std = ts_last.std + base - ts_last.real - offset;
		reader->out_offset = offset;
		if (que->timestamps[que->head].std > reader->out_std) {
			reader->out_std = que->timestamps[que->head].std;
			reader->out_offset = base -
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

	ts21_diff = (ts2.real - ts1.real) - (ts2.std - ts1.std);

	/* half of the interval */
	interval_thresh = interval >> 1;

	/**
	 * If ts2 to ts1 differ greatly from the nominal time period, do
	 * replace ts2 value by interpolate value.
	 */
	if (ts21_diff < -interval_thresh || interval_thresh < ts21_diff) {
		/* interpolate by the nominal time period */
		ts2.real = ts1.real + (ts2.std - ts1.std);
		que->timestamps[pos2].real = ts2.real;
	}

	t = (ts2.real - ts1.real) * (reader->out_std - ts1.std);
	*timestamp = div64_u64(t, ts2.std - ts1.std) +
			ts1.real + reader->out_offset;

	if (f_first) {
		mse_debug_tstamps2("%s out start %llu ofsset %llu now %llu out %llu prev %llu\n",
				   reader->name,
				   reader->out_std,
				   reader->out_offset,
				   base,
				   *timestamp,
				   reader->out_time);
	}

	reader->out_time = *timestamp;

	return 0;
}

static int tstamps_calc_tstamp(struct timestamp_reader *reader,
			       u64 base,
			       u32 offset,
			       u64 interval,
			       u64 *timestamp)
{
	int ret;
	unsigned long flags;

	read_lock_irqsave(&reader->que->rwlock, flags);
	ret = tstamps_calc_tstamp_nolock(reader,
					 base,
					 offset,
					 interval,
					 timestamp);
	read_unlock_irqrestore(&reader->que->rwlock, flags);

	return ret;
}

static int tstamps_calc_tstamps(struct timestamp_reader *reader,
				u64 base,
				u32 offset,
				u64 interval,
				u64 *timestamps,
				int size)
{
	int i;
	u64 timestamp;
	unsigned long flags;
	int ret = -1;

	read_lock_irqsave(&reader->que->rwlock, flags);
	for (i = 0; i < size; i++) {
		ret = tstamps_calc_tstamp_nolock(reader,
						 base,
						 offset,
						 interval,
						 &timestamp);
		if (ret < 0)
			break;

		timestamps[i] = timestamp;
	}
	read_unlock_irqrestore(&reader->que->rwlock, flags);

	return ret;
}

static int tstamps_search_tstamp32(struct timestamp_reader *reader,
				   u32 avtp_time,
				   u32 offset,
				   u32 interval)
{
	struct timestamp_queue *que = reader->que;
	struct timestamp_set ts_set;
	unsigned long flags;
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

	read_lock_irqsave(&que->rwlock, flags);
	/* for each queue forward */
	for (pos = que->head;
	     pos != que->tail;
	     pos = q_next(que, pos)) {
		ts_set = que->timestamps[pos];
		delta_ts = (u32)ts_set.real - avtp_time;
		if (delta_ts < U32_MAX / 2)
			break;
	}

	/* not found search timestamp */
	if (pos == que->tail || pos == que->head) {
		read_unlock_irqrestore(&que->rwlock, flags);
		return -1;
	}

	read_unlock_irqrestore(&que->rwlock, flags);

	reader->out_std = ts_set.std;

	if (delta_ts < NSEC_SCALE)
		reader->out_std -= delta_ts;
	else
		mse_info("not precision offset capture %u\n", delta_ts);

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

static bool tstamps_continuity_check_nolock(struct timestamp_queue *que,
					    u64 timestamp)
{
	struct timestamp_set ts_set;
	s64 diff;

	ts_set = que->timestamps[q_prev(que, que->tail)];

	diff = timestamp - ts_set.real - que->std_interval;
	/* compare lower 32bit only */
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

static void tstamps_enq_tstamps(struct timestamp_queue *que,
				u64 *timestamps,
				int n)
{
	struct timestamp_set timestamp_set;
	u64 timestamp;
	int i;
	unsigned long flags;

	if (!que->f_init)
		return;

	write_lock_irqsave(&que->rwlock, flags);
	for (i = 0; i < n; i++) {
		timestamp = timestamps[i];

		if (!q_empty(que))
			if (!tstamps_continuity_check_nolock(que, timestamp))
				break;

		timestamp_set.real = timestamp;
		timestamp_set.std = que->std_counter;

		que->std_counter += que->std_interval;
		que->timestamps[que->tail] = timestamp_set;
		que->tail = q_next(que, que->tail);

		/* if tail equal head, push out head */
		if (que->tail == que->head)
			que->head = q_next(que, que->head);

		mse_debug_tstamps2("%s head=%d, tail=%d timestamp=%llu\n",
				   que->name, que->head, que->tail, timestamp);
	}

	write_unlock_irqrestore(&que->rwlock, flags);
}

static void tstamps_enq_tstamp(struct timestamp_queue *que,
			       u64 timestamp)
{
	tstamps_enq_tstamps(que, &timestamp, 1);
}

static int tstamps_get_tstamps_size_nolock(struct timestamp_queue *que)
{
	if (q_empty(que))
		return 0;

	if (!que->f_sync)
		return 0;

	if (que->tail >= que->head)
		return que->tail - que->head - 1;
	else
		return (que->tail + PTP_TIMESTAMPS_MAX) - que->head - 1;
}

static int tstamps_deq_tstamp_nolock(struct timestamp_queue *que,
				     u64 *timestamp)
{
	if (q_empty(que))
		return -1;

	if (!que->f_sync)
		return -1;

	*timestamp = que->timestamps[que->head].real;

	que->head = q_next(que, que->head);

	mse_debug_tstamps2("%s head=%d, tail=%d timestamp=%llu\n",
			   que->name, que->head, que->tail, *timestamp);

	return 0;
}

static int tstamps_deq_tstamps(struct timestamp_queue *que,
			       u64 *timestamps,
			       int req_num_min,
			       int req_num_max)
{
	int num, i;
	unsigned long flags;

	if (q_empty(que))
		return 0;

	if (!que->f_sync)
		return 0;

	write_lock_irqsave(&que->rwlock, flags);

	num = tstamps_get_tstamps_size_nolock(que);

	if (num < req_num_min)
		num = 0;
	else
		num = min(req_num_max, num);

	for (i = 0; i < num; i++)
		tstamps_deq_tstamp_nolock(que, &timestamps[i]);

	write_unlock_irqrestore(&que->rwlock, flags);

	mse_debug_tstamps2("%s head=%d, tail=%d timestamp=%llu\n",
			   que->name, que->head, que->tail, timestamps[0]);

	return num;
}

static bool media_clock_recovery_calc_ts(struct timestamp_reader *reader,
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

	diff = PTP_TIME_DIFF_S32(timestamp, device_time);
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
	u32 delta_ts, offset;
	int ts_num;
	int i = 0;
	int out = 0;
	int recovery_value;
	u64 recovery_capture_freq;
	bool f_out_ok_pre;
	int calc_error = 0;

	if (instance->crf_type != MSE_CRF_TYPE_RX) {
		packetizer = instance->packetizer;
		index_packetizer = instance->index_packetizer;
		que = &instance->avtp_que;
		offset = 0;
	} else {
		packetizer = &mse_packetizer_crf_timestamp_audio_ops;
		index_packetizer = instance->crf_index;
		que = &instance->crf_que;
		offset = instance->max_transit_time_ns;
	}

	packetizer->get_audio_info(index_packetizer, &audio_info);
	delta_ts = audio_info.frame_interval_time;
	if (instance->f_ptp_capture)
		offset += instance->capture_delay_time_ns;

	/* if timestamps queue continuity broken */
	if (!que->f_sync || !instance->tstamp_que.f_sync)
		return 0;

	f_out_ok_pre = reader_mch->f_out_ok;

	/* fill master/device timestamps */
	ts_num = tstamps_deq_tstamps(que,
				     instance->temp_ts,
				     0,
				     ARRAY_SIZE(instance->ts));
	for (i = 0; i < ts_num; i++) {
		if (!media_clock_recovery_calc_ts(reader_mch,
						  instance->temp_ts[i],
						  delta_ts,
						  offset,
						  &ts)) {
			calc_error++;
		} else {
			instance->ts[out] = ts;
			out++;
		}
	}

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
	int captured;
	u64 now;

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_timestamp);

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
				!instance->captured_timestamps);
		instance->captured_timestamps += captured;
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
	u32 delta_ts;
	u32 offset;
	int ret;

	instance->packetizer->get_audio_info(
		instance->index_packetizer,
		&audio_info);

	instance->avtp_timestamps_current = 0;

	offset = instance->f_ptp_capture ? instance->capture_delay_time_ns : 0;

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

	ret = tstamps_calc_tstamps(&instance->reader_create_avtp,
				   now,
				   offset,
				   delta_ts,
				   instance->avtp_timestamps,
				   create_size);
	if (ret >= 0) {
		for (i = 0; i < create_size; i++) {
			instance->avtp_timestamps[i] +=
				instance->max_transit_time_ns;
		}
	} else {
		instance->avtp_timestamps[0] =
			now + instance->max_transit_time_ns;
		for (i = 1; i < create_size; i++) {
			instance->avtp_timestamps[i] =
				instance->avtp_timestamps[i - 1] + delta_ts;
		}
	}

	instance->avtp_timestamps_size = create_size;

	return 0;
}

static int mse_initialize_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf =
		&mse_packetizer_crf_timestamp_audio_ops;
	struct mse_audio_config *audio = &instance->media_config.audio;
	struct mse_audio_config config;
	struct mse_cbsparam cbs;
	int ret;

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
		return ret;

	if (instance->crf_type == MSE_CRF_TYPE_TX) {
		ret = crf->calc_cbs(instance->crf_index, &cbs);
		if (ret < 0)
			return ret;

		ret = instance->network->set_cbs_param(
			instance->crf_index_network, &cbs);
		if (ret < 0)
			return ret;
	}

	if (instance->crf_type == MSE_CRF_TYPE_RX) {
		ret = instance->network->set_streamid(
					instance->crf_index_network,
					instance->crf_net_config.streamid);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void mse_release_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf =
		&mse_packetizer_crf_timestamp_audio_ops;
	unsigned long flags;

	if (instance->crf_index >= 0) {
		spin_lock_irqsave(&packetizer_crf_lock, flags);
		crf->release(instance->crf_index);
		spin_unlock_irqrestore(&packetizer_crf_lock, flags);

		instance->crf_index = MSE_INDEX_UNDEFINED;
	}
}

static int mse_open_crf_packetizer(struct mse_instance *instance)
{
	struct mse_packetizer_ops *crf =
		&mse_packetizer_crf_timestamp_audio_ops;
	int ret;
	unsigned long flags;

	if (instance->crf_type == MSE_CRF_TYPE_NOT_USE)
		return 0;

	spin_lock_irqsave(&packetizer_crf_lock, flags);
	ret = crf->open();
	spin_unlock_irqrestore(&packetizer_crf_lock, flags);
	if (ret < 0) {
		mse_err("cannot open packetizer ret=%d\n", ret);
		return ret;
	}

	instance->crf_index = ret;

	ret = crf->init(instance->crf_index);
	if (ret < 0) {
		mse_err("cannot init packetizer ret=%d\n", ret);
		crf->release(instance->crf_index);
		instance->crf_index = MSE_INDEX_UNDEFINED;

		return ret;
	}

	return 0;
}

static bool check_packet_remain(struct mse_instance *instance)
{
	int wait_count = MSE_TX_RING_SIZE - MSE_TX_PACKET_NUM;
	int rem = mse_packet_ctrl_check_packet_remain(instance->packet_buffer);
	struct mse_wait_packet *wp0, *wp1;
	u32 accum_wait_time;

	/**
	 * Check accumulate wait time of wait packets.
	 *
	 * This check is effect only on MPEG2TS usecase. On the other usecases
	 * does not effect because wait_packet_list is always empty.
	 */
	if (list_empty(&instance->wait_packet_list)) {
		accum_wait_time = 0;
	} else {
		wp0 = list_first_entry(&instance->wait_packet_list,
				       struct mse_wait_packet,
				       list);
		wp1 = list_last_entry(&instance->wait_packet_list,
				      struct mse_wait_packet,
				      list);

		accum_wait_time = (u32)PTP_TIME_DIFF_S32(wp1->launch_avtp_timestamp,
							 wp0->launch_avtp_timestamp);
	}

	if (accum_wait_time > MPEG2TS_MAX_WAIT_TIME)
		return false;
	else
		return wait_count > rem;
}

static void mse_work_packetize_common(struct mse_instance *instance)
{
	int ret = 0;
	int trans_size;
	unsigned long flags;
	struct mse_trans_buffer *buf;

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

	if (trans_size <= 0) /* no data to process */
		return;

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
		if (mse_is_buffer_empty(instance)) {
			/* state is STOPPING */
			if (mse_state_test(instance, MSE_STATE_STOPPING)) {
				mse_debug("discard %zu byte before stopping\n",
					  buf->buffer_size - buf->work_length);

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

			mse_trans_complete(instance,
					   &instance->proc_buf_list,
					   ret);

			return;
		}
	} else if (ret < 0) {
		mse_err("error=%d buffer may be corrupted\n", ret);
		instance->f_continue = false;

		mse_trans_complete(instance, &instance->proc_buf_list, ret);

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
		if (!instance->timer_interval)
			atomic_inc(&instance->done_buf_cnt);

		if (atomic_read(&instance->done_buf_cnt) > 0)
			queue_work(instance->wq_packet, &instance->wk_callback);
	}
}

static u32 m2ts_timestamp_to_nsec(u32 host_header)
{
	u64 ts_nsec = (u64)(host_header & 0x3fffffff) * NSEC_SCALE;

	return (u32)div64_u64(ts_nsec, MPEG2TS_M2TS_FREQ);
}

static bool mse_search_mpeg2ts_pcr(struct mse_instance *instance,
				   struct mse_trans_buffer *buf,
				   size_t start_position,
				   bool update_pcr,
				   size_t *found_pos)
{
	struct mse_mpeg2ts_config *mpeg2ts = &instance->media_config.mpeg2ts;
	u16 pcr_pid = mpeg2ts->pcr_pid;
	u8 afc, afc_len, pcr_flag;
	size_t work_length;
	int psize, offset;
	bool ret = false;
	int skip = 0;
	u64 pcr_ext;
	u8 *tsp;
	u16 pid;
	u64 pcr;

	if (mpeg2ts->mpeg2ts_type == MSE_MPEG2TS_TYPE_TS)
		psize = MPEG2TS_TS_SIZE;
	else
		psize = MPEG2TS_M2TS_SIZE;

	offset = psize - MPEG2TS_TS_SIZE;
	work_length = start_position;

	while (work_length + psize <= buf->buffer_size) {
		tsp = buf->buffer + work_length + offset;

		/* check sync byte */
		if (tsp[0] != MPEG2TS_SYNC) {
			work_length++;
			skip++;
			continue;
		}

		if (skip) {
			mse_debug("check sync byte. skip %d byte\n",
				  skip);
			skip = 0;
		}

		work_length += psize;

		pid = (((u16)tsp[1] << 8) + (u16)tsp[2]) & 0x1fff;
		afc = (tsp[3] & 0x30) >> 4;
		afc_len = tsp[4];
		pcr_flag = (tsp[5] & 0x10) >> 4;

		/* Adaptation Field Control = 0b10 or 0b11 and  */
		/* afc_len >= 7 and pcr_flag = 1 */
		if (!(afc & 0x2) || afc_len < 7 || !pcr_flag)
			continue;    /* no PCR */

		/* PCR base: 90KHz, 33bits (32+1bits) */
		pcr = ((u64)tsp[6] << 25) |
			((u64)tsp[7] << 17) |
			((u64)tsp[8] << 9) |
			((u64)tsp[9] << 1) |
			(((u64)tsp[10] & 0x80) >> 7);

		/* PCR extension: 27MHz, 9bits (8+1bits) */
		pcr_ext = ((tsp[10] & 0x1) << 8) | tsp[11];
		pcr = (pcr * 300) + (pcr_ext % 300);

		mse_debug("find pcr %d (required %d)\n", pid, pcr_pid);

		if (pcr_pid == MPEG2TS_PCR_PID_IGNORE) {
			if (instance->mpeg2ts_pre_pcr_pid != MPEG2TS_PCR_PID_IGNORE &&
			    pid != instance->mpeg2ts_pre_pcr_pid) {
				continue;
			}
		} else if (pcr_pid != pid) {
			continue;
		}

		if (instance->mpeg2ts_pre_pcr_pid == MPEG2TS_PCR_PID_IGNORE) {
			if (update_pcr) {
				/* Find first PCR. */
				instance->f_first_pcr = true;
				instance->mpeg2ts_pcr_27m = pcr;
				instance->mpeg2ts_pre_pcr_pid = pid;
				instance->mpeg2ts_pre_pcr_27m = pcr;
			}
			ret = true;
			break;
		}

		if (pcr == instance->mpeg2ts_pcr_27m) {
			ret = true;
			break;
		}

		if (compare_pcr(pcr, instance->mpeg2ts_pre_pcr_27m)) {
			if (update_pcr) {
				instance->mpeg2ts_pre_pcr_27m = instance->mpeg2ts_pcr_27m;
				instance->mpeg2ts_pcr_27m = pcr;
			}
			ret = true;
			break;
		}
	}

	if (ret)
		*found_pos = work_length - psize;

	return ret;
}

static bool mse_search_mpeg2ts_m2ts_interval(struct mse_instance *instance,
					     struct mse_trans_buffer *buf,
					     size_t start_position,
					     size_t data_size,
					     u32 base_m2ts_timestamp,
					     size_t *found_pos)
{
	size_t work_length;
	u32 m2ts_timestamp;
	u32 diff;

	work_length = start_position;

	while (work_length + MPEG2TS_M2TS_SIZE <= data_size) {
		m2ts_timestamp = ntohl(*(u32 *)(buf->buffer + work_length));
		diff = MPEG2TS_M2TS_DIFF(m2ts_timestamp, base_m2ts_timestamp);
		if (diff >= MPEG2TS_WAIT_INTERVAL) {
			*found_pos = work_length;
			return true;
		}

		work_length += MPEG2TS_M2TS_SIZE;
	}

	return false;
}

static void mse_init_mpeg2ts_base_timestamp(struct mse_instance *instance,
					    struct mse_trans_buffer *buf)
{
	struct mse_mpeg2ts_config *mpeg2ts = &instance->media_config.mpeg2ts;
	enum MSE_TRANSMIT_MODE transmit_mode;

	transmit_mode = mpeg2ts->transmit_mode;

	if (transmit_mode == MSE_TRANSMIT_MODE_TIMESTAMP) {
		if (mpeg2ts->mpeg2ts_type == MSE_MPEG2TS_TYPE_M2TS) {
			instance->mpeg2ts_m2ts_base = ntohl(*(u32 *)buf->buffer);
		} else {
			mse_info("Unsupported type. Change to PCR mode.\n");
			transmit_mode = MSE_TRANSMIT_MODE_PCR;
		}
	}

	instance->timestamp = buf->launch_avtp_timestamp;
	instance->transmit_mode = transmit_mode;
	instance->mpeg2ts_timestamp_base = buf->launch_avtp_timestamp;
}

static void mse_update_mpeg2ts_base_timestamp(struct mse_instance *instance,
					      struct mse_trans_buffer *buf)
{
	u32 m2ts_timestamp_base;
	u32 m2ts_timestamp;
	u64 diff;

	m2ts_timestamp_base = instance->mpeg2ts_timestamp_base;

	if (instance->transmit_mode == MSE_TRANSMIT_MODE_PCR) {
		/* Add diff of PCR */
		diff = MPEG2TS_PCR27M_DIFF(instance->mpeg2ts_pcr_27m,
					   instance->mpeg2ts_pre_pcr_27m);

		m2ts_timestamp_base += (u32)MPEG2TS_PCR27M_TO_NS(diff);
	} else if (instance->transmit_mode == MSE_TRANSMIT_MODE_TIMESTAMP) {
		/* Add diff of m2ts timestamp */
		m2ts_timestamp = ntohl(*(u32 *)(buf->buffer + buf->work_length));
		diff = MPEG2TS_M2TS_DIFF(m2ts_timestamp, instance->mpeg2ts_m2ts_base);

		m2ts_timestamp_base += m2ts_timestamp_to_nsec(diff);
		instance->mpeg2ts_m2ts_base = m2ts_timestamp;
	} else { /* MSE_TRANSMIT_MODE_BITRATE */
		/* nop */
	}

	if (instance->f_first_pcr) {
		instance->f_first_pcr = false;
		/* Check elapsed time */
		if (m2ts_timestamp_base - instance->timestamp > instance->delay_time_ns)
			mse_info("delay_time insufficient. Elapsed time %u.\n",
				 m2ts_timestamp_base - instance->timestamp);
		else
			m2ts_timestamp_base = instance->timestamp + instance->delay_time_ns;
	}

	instance->mpeg2ts_timestamp_base = m2ts_timestamp_base;
}

static bool mse_check_wait_packet(struct mse_instance *instance,
				  struct mse_trans_buffer *buf,
				  size_t *size)
{
	struct mse_mpeg2ts_config *mpeg2ts = &instance->media_config.mpeg2ts;
	u32 m2ts_timestamp_base;
	u32 m2ts_timestamp;
	size_t packet_size;
	size_t trans_size;
	size_t found_pos;
	bool ret = false;
	u32 diff;

	trans_size = buf->buffer_size;

	if (instance->transmit_mode == MSE_TRANSMIT_MODE_BITRATE) {
		*size = trans_size;
		return false;
	}

	if (mpeg2ts->mpeg2ts_type == MSE_MPEG2TS_TYPE_TS)
		packet_size = MPEG2TS_TS_SIZE;
	else
		packet_size = MPEG2TS_M2TS_SIZE;

	/* Search PCR in trans data */
	if (mse_search_mpeg2ts_pcr(instance,
				   buf,
				   buf->work_length,
				   true,
				   &found_pos)) {
		/* PCR exist */
		if (found_pos == buf->work_length) {
			/* PCR is top */
			ret = true;

			/* Search next PCR */
			if (mse_search_mpeg2ts_pcr(instance,
						   buf,
						   buf->work_length + packet_size,
						   false,
						   &found_pos)) {
				/* Size up to next PCR */
				trans_size = found_pos;
			}
		} else {
			/* Size up to PCR */
			trans_size = found_pos;
		}
	}

	if (instance->transmit_mode == MSE_TRANSMIT_MODE_PCR) {
		*size = trans_size;
		return ret;
	}

	/* MSE_TRANSMIT_MODE_TIMESTAMP */

	if (ret)
		m2ts_timestamp_base = ntohl(*(u32 *)buf->buffer);
	else
		m2ts_timestamp_base = instance->mpeg2ts_m2ts_base;

	if (buf->buffer_size != trans_size) {
		/* Check difference between interval packet and next PCR */
		m2ts_timestamp = ntohl(*(u32 *)(buf->buffer + trans_size));
		diff = MPEG2TS_M2TS_DIFF(m2ts_timestamp, m2ts_timestamp_base);
		if (diff <= MPEG2TS_WAIT_INTERVAL + MPEG2TS_INTERVAL_THRESHOLD) {
			/* Difference is within interval plus threshold */
			*size = trans_size;
			return ret;
		}
	}

	/* Search interval packet in trans data */
	if (mse_search_mpeg2ts_m2ts_interval(instance,
					     buf,
					     buf->work_length,
					     trans_size,
					     m2ts_timestamp_base,
					     &found_pos)) {
		/* Interval packet exist */
		if (found_pos == buf->work_length) {
			/* Interval packet is top */
			ret = true;

			/* Search next interval packet */
			m2ts_timestamp_base = ntohl(*(u32 *)buf->buffer);
			if (mse_search_mpeg2ts_m2ts_interval(instance,
							     buf,
							     buf->work_length + packet_size,
							     trans_size,
							     m2ts_timestamp_base,
							     &found_pos)) {
				/* Size up to next interval packet */
				trans_size = found_pos;
			}
		} else {
			/* Size up to interval packet */
			trans_size = found_pos;
		}
	}

	*size = trans_size;
	return ret;
}

static void mse_update_wait_packet(struct mse_instance *instance,
				   u32 timestamp)
{
	struct mse_packet_ctrl *dma;
	struct mse_wait_packet *wp;

	dma = instance->packet_buffer;

	if (!list_empty(&instance->wait_packet_list)) {
		wp = list_last_entry(&instance->wait_packet_list,
				     struct mse_wait_packet,
				     list);
		if (wp->launch_avtp_timestamp == timestamp) {
			/* Update last entry */
			wp->release_p = dma->write_p;
			return;
		}
	}

	/* Add entry */
	wp = &instance->wait_packet[instance->wait_packet_idx];
	wp->release_p = dma->write_p;
	wp->launch_avtp_timestamp = timestamp;

	list_add_tail(&wp->list, &instance->wait_packet_list);
	instance->wait_packet_idx = (instance->wait_packet_idx + 1) % MSE_TX_RING_SIZE;
}

static void mse_control_wait_packet(struct mse_instance *instance,
				    bool f_wait)
{
	unsigned long flags;
	u64 timestamp;
	int err;

	timestamp = instance->avtp_timestamps[0] - instance->max_transit_time_ns;

	if (!instance->ptp_timer_handle) {
		/* Output immediately. Skip wait_packet_list */
		mse_packet_ctrl_release_all_wait(instance->packet_buffer);

		read_lock_irqsave(&instance->lock_stream, flags);
		/* start workqueue for streaming */
		if (!instance->f_streaming)
			queue_work(instance->wq_stream, &instance->wk_stream);
		read_unlock_irqrestore(&instance->lock_stream, flags);

		return;
	}

	read_lock_irqsave(&instance->lock_stream, flags);

	if (instance->f_timer_started) {
		mse_update_wait_packet(instance, timestamp);
	} else if (!f_wait ||
		   !mse_check_ptp_timer_threshold(instance, timestamp)) {
		/* Output immediately. Skip wait_packet_list */
		mse_packet_ctrl_release_all_wait(instance->packet_buffer);

		/* start workqueue for streaming */
		if (!instance->f_streaming)
			queue_work(instance->wq_stream, &instance->wk_stream);
	} else {
		/* Update wait packet queue when start timer */
		mse_update_wait_packet(instance, timestamp);

		/* Set ptp timer */
		instance->ptp_timer_start = timestamp;

		mse_debug("Timer start. %u\n", (u32)instance->ptp_timer_start);

		err = mse_ptp_timer_start(instance->ptp_index,
					  instance->ptp_timer_handle,
					  (u32)instance->ptp_timer_start);
		if (err < 0) {
			mse_err("mse_ptp_timer_start error=%d\n", err);

			/* Output immediately. Skip wait_packet_list */
			INIT_LIST_HEAD(&instance->wait_packet_list);
			mse_packet_ctrl_release_all_wait(instance->packet_buffer);
			if (!instance->f_streaming)
				queue_work(instance->wq_stream, &instance->wk_stream);

		} else {
			instance->f_timer_started = true;
		}
	}

	read_unlock_irqrestore(&instance->lock_stream, flags);
}

static int do_packetize_mpeg2ts_tx(struct mse_instance *instance,
				   void *data,
				   size_t size,
				   bool f_wait,
				   size_t *processed)
{
	int ret = 0;

	ret = mse_packet_ctrl_make_packet(
				instance->index_packetizer,
				data,
				size,
				instance->f_ptp_capture,
				&instance->avtp_timestamps_current,
				instance->avtp_timestamps_size,
				instance->avtp_timestamps,
				instance->packet_buffer,
				instance->packetizer,
				processed);
	if (ret < 0) {
		mse_err("error=%d buffer may be corrupted\n", ret);
		return ret;
	}

	mse_control_wait_packet(instance, f_wait);

	return ret;
}

static void mse_work_packetize_mpeg2ts_tx(struct mse_instance *instance)
{
	struct mse_trans_buffer *buf;
	size_t piece_length = 0;
	unsigned long flags;
	size_t buffer_size;
	int trans_size;
	int ret = 0;
	bool f_wait;

	mse_debug_state(instance);

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return;

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);
	if (!buf) {
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			/* Flush packet piece */
			ret = do_packetize_mpeg2ts_tx(instance,
						      NULL,
						      0,
						      false,
						      &piece_length);

			queue_work(instance->wq_packet, &instance->wk_stop_streaming);
		}
		return; /* skip work */
	}

	trans_size = buf->buffer_size - buf->work_length;
	mse_debug("trans size=%d buffer=%p buffer_size=%zu\n",
		  trans_size, buf->buffer, buf->buffer_size);

	if (trans_size <= 0) {
		/* no data to process */
		list_move_tail(&buf->list, &instance->done_buf_list);
		queue_work(instance->wq_packet, &instance->wk_callback);
		return;
	}

	if (!instance->f_start_stream) {
		/* First stream */
		instance->f_start_stream = true;

		mse_init_mpeg2ts_base_timestamp(instance, buf);
	}

	/* Check wait packet. Limit packetize size up to next wait packet */
	f_wait = mse_check_wait_packet(instance, buf, &buffer_size);
	if (f_wait)
		mse_update_mpeg2ts_base_timestamp(instance, buf);

	/* make AVTP packet with one timestamp */
	instance->avtp_timestamps_current = 0;
	instance->avtp_timestamps_size = 1;
	instance->avtp_timestamps[0] = instance->mpeg2ts_timestamp_base;

	while (buf->work_length < buffer_size) {
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
						MSE_TIMEOUT_PACKETIZE_MPEG2TS)) {
					mse_debug("wait event timeouted\n");
					break;
				}
			}
		}

		ret = do_packetize_mpeg2ts_tx(instance,
					      buf->buffer,
					      buffer_size,
					      f_wait,
					      &buf->work_length);
		if (ret < 0)
			break;

		f_wait = false;
	}

	mse_debug("packetized(ret)=%d len=%zu\n", ret, buf->work_length);

	if (ret < 0) {
		buf->work_length = ret;
		instance->f_continue = false;
	} else {
		instance->f_continue = buf->work_length < buf->buffer_size;
	}

	if (instance->f_continue) {
		queue_work(instance->wq_packet, &instance->wk_packetize);
	} else {
		list_move_tail(&buf->list, &instance->done_buf_list);

		queue_work(instance->wq_packet, &instance->wk_callback);

		if (!list_empty(&instance->proc_buf_list) ||
		    mse_state_test(instance, MSE_STATE_STOPPING)) {
			queue_work(instance->wq_packet, &instance->wk_packetize);
		}
	}
}

static void mse_work_packetize(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_packetize);

	if (IS_MSE_TYPE_MPEG2TS(instance->media->type) && instance->tx)
		mse_work_packetize_mpeg2ts_tx(instance);
	else
		mse_work_packetize_common(instance);
}

static void mse_set_start_time(struct mse_instance *instance)
{
	struct mse_packetizer_ops *packetizer = instance->packetizer;
	u32 start_time;
	u64 now;
	int delay_period;

	delay_period = (instance->temp_w - instance->temp_r +
			MSE_DECODE_BUFFER_NUM) % MSE_DECODE_BUFFER_NUM;

	start_time = instance->ptp_timer_start +
		(delay_period - 1) * instance->timer_interval;

	packetizer->set_start_time(instance->index_packetizer, start_time);

	mse_ptp_get_time(instance->ptp_index, &now);
	mse_debug_tstamps2("set start time %u now %u diff %lld buffer %d\n",
			   (u32)start_time,
			   (u32)now,
			   (s64)start_time - now,
			   delay_period);
}

static void mse_work_depacketize_common(struct mse_instance *instance)
{
	struct mse_packet_ctrl *packet_buffer;
	struct mse_audio_info audio_info;
	struct mse_trans_buffer *buf;
	int received, ret = 0;
	u64 timestamps[128];
	int timestamps_stored;
	int temp_r, temp_w, temp_w_next;
	bool has_valid_data = false;
	unsigned long flags;

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
			temp_r = instance->temp_r;
			temp_w = instance->temp_w;
			temp_w_next = (temp_w + 1) % MSE_DECODE_BUFFER_NUM;

			if (temp_w_next == temp_r &&
			    instance->temp_len[temp_w] >= buf->buffer_size) {
				mse_packet_ctrl_discard_packet(packet_buffer);
				break;
			}

			/* set start time of period in media clock recovery */
			if (instance->ptp_timer_handle)
				mse_set_start_time(instance);

			/* get AVTP packet payload */
			ret = mse_packet_ctrl_take_out_packet(
				instance->index_packetizer,
				instance->temp_buffer[temp_w],
				buf->buffer_size,
				timestamps,
				ARRAY_SIZE(timestamps),
				&timestamps_stored,
				packet_buffer,
				instance->packetizer,
				&instance->temp_len[temp_w]);
			if (ret < 0 && ret != -EAGAIN) {
				mse_trans_complete(instance,
						   &instance->proc_buf_list,
						   ret);
				break;
			}

			/* samples per packet */
			instance->packetizer->get_audio_info(
				instance->index_packetizer,
				&audio_info);

			/* store avtp timestamp */
			if (timestamps_stored > 0) {
				if (!instance->avtp_que.f_init)  {
					tstamps_init(
						&instance->avtp_que,
						"AVTP",
						true,
						audio_info.frame_interval_time);
				}

				tstamps_enq_tstamps(&instance->avtp_que,
						    timestamps,
						    timestamps_stored);

				if (!instance->f_get_first_packet) {
					instance->f_get_first_packet = true;
					instance->processed = 0;
				}
			}

			if (instance->temp_len[temp_w] >= buf->buffer_size &&
			    temp_w_next != temp_r) {
				has_valid_data = true;
				memset(instance->temp_buffer[temp_w_next],
				       0,
				       buf->buffer_size);

				instance->temp_len[temp_w_next] = 0;
				instance->temp_w = temp_w_next;

				mse_debug("buffer=%p temp_r=%d temp_w_next=%d depacketized=%zu ret=%d\n",
					  buf->buffer,
					  temp_r,
					  temp_w_next,
					  instance->temp_len[temp_w],
					  ret);
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
						&timestamps_stored,
						packet_buffer,
						instance->packetizer,
						&buf->work_length);

		/* complete callback */
		if (ret > 0) {
			has_valid_data = true;
			atomic_inc(&instance->done_buf_cnt);

			mse_debug("buffer=%p depacketized=%zu ret=%d\n",
				  buf->buffer,
				  buf->work_length, ret);
		} else if (ret != -EAGAIN) {
			mse_trans_complete(instance,
					   &instance->proc_buf_list,
					   ret);
		}

		break;

	default:
		mse_err("unknown type=0x%08x\n", instance->media->type);
		break;
	}

	if (atomic_read(&instance->done_buf_cnt) > 0 &&
	    has_valid_data)
		queue_work(instance->wq_packet, &instance->wk_callback);

	/* state is STOPPING */
	if (mse_state_test(instance, MSE_STATE_STOPPING)) {
		if (instance->ptp_timer_handle ||
		    !atomic_read(&instance->done_buf_cnt))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);

		if (atomic_inc_return(&instance->done_buf_cnt) != 1)
			atomic_dec(&instance->done_buf_cnt);
	}

	/* state is NOT EXECUTE */
	if (!mse_state_test(instance, MSE_STATE_EXECUTE))
		instance->f_depacketizing = false;
}

static bool mse_check_buffer_starvation(struct mse_instance *instance)
{
	int remain;

	/* Stopped supply of buffer from upper */
	if (mse_state_test(instance, MSE_STATE_STOPPING))
		return false;

	/* Exists buffer in preparation */
	if (!list_empty(&instance->trans_buf_list) ||
	    !list_empty(&instance->proc_buf_list) ||
	    !list_empty(&instance->done_buf_list))
		return false;

	/* No exist buffer of extrusion target */
	if (list_empty(&instance->wait_buf_list))
		return false;

	/* Exists still available space in receive buffer */
	remain = mse_packet_ctrl_check_packet_remain(instance->packet_buffer);
	if (remain > MSE_RX_PACKET_NUM_MAX)
		return false;

	mse_debug("Insufficient free buffer.\n");

	return true;
}

static bool mse_check_ptp_timer_threshold(struct mse_instance *instance,
					  u32 timestamp)
{
	u64 now;

	mse_ptp_get_time(instance->ptp_index, &now);

	mse_debug("timestamp = %u, now = %u\n", timestamp, (u32)now);

	/* Check difference between now time and current timestamp */
	if (PTP_TIME_DIFF_S32(timestamp, now) < PTP_TIMER_START_THRESHOLD) {
		mse_debug("timestamp is within threshold.\n");

		return false;
	}

	return true;
}

static bool mse_check_ptp_timer_threshold_video_rx(struct mse_instance *instance,
						   u64 timestamp)
{
	struct mse_trans_buffer *wait_buf;

	if (timestamp == PTP_TIMESTAMP_INVALID)
		return false;

	/* Check difference between now time and current timestamp */
	if (!mse_check_ptp_timer_threshold(instance, (u32)timestamp))
		return false;

	if (!list_empty(&instance->wait_buf_list)) {
		/*
		 * Check difference between previous timestamp for wait and
		 * current timestamp
		 */
		wait_buf = list_last_entry(&instance->wait_buf_list,
					   struct mse_trans_buffer,
					   list);

		if (wait_buf->launch_avtp_timestamp != PTP_TIMESTAMP_INVALID &&
		    PTP_TIME_DIFF_S32(timestamp, wait_buf->launch_avtp_timestamp) < PTP_TIMER_START_THRESHOLD)
			return false;
	}

	return true;
}

static bool do_depacketize_video_rx(struct mse_instance *instance, struct mse_trans_buffer *buf)
{
	u64 launch_avtp_timestamp = PTP_TIMESTAMP_INVALID;
	int timestamps_stored;
	u64 timestamps[128];
	unsigned long flags;
	int ret = 0;

	/* get AVTP packet payload */
	ret = mse_packet_ctrl_take_out_packet(instance->index_packetizer,
					      buf->buffer,
					      buf->buffer_size,
					      timestamps,
					      ARRAY_SIZE(timestamps),
					      &timestamps_stored,
					      instance->packet_buffer,
					      instance->packetizer,
					      &buf->work_length);

	/* complete callback */
	if (ret > 0) {
		mse_debug("buffer=%p depacketized=%zu ret=%d\n",
			  buf->buffer,
			  buf->work_length,
			  ret);

		/*
		 * Should use the timestamp of picture first packet.
		 * But, all packets have same timestamp. Therefore,
		 * Here we use the output timestamp.
		 */
		launch_avtp_timestamp = (u32)(timestamps[0] + instance->delay_time_ns);
	} else if (ret == -EAGAIN && !mse_state_test(instance, MSE_STATE_STOPPING)) {
		/* no process data, wait next receive */
		return false;
	} else {
		buf->work_length = ret;
	}

	/* Move list of current buffer */
	spin_lock_irqsave(&instance->lock_buf_list, flags);

	if (!instance->ptp_timer_handle) {
		/* Output immediately. Skip wait_buf_list */
		list_move_tail(&buf->list, &instance->done_buf_list);
	} else {
		/* If timestamp is within threshold, output immediately */
		if (mse_check_ptp_timer_threshold_video_rx(instance, launch_avtp_timestamp))
			buf->launch_avtp_timestamp = launch_avtp_timestamp;

		if (instance->f_timer_started) {
			/* Update timer when launch timer */
			list_move_tail(&buf->list, &instance->wait_buf_list);
		} else {
			if (buf->launch_avtp_timestamp == PTP_TIMESTAMP_INVALID) {
				/* Output immediately. Skip wait_buf_list */
				list_move_tail(&buf->list, &instance->done_buf_list);
			} else {
				/* Set timer of current timestamp */
				list_move_tail(&buf->list, &instance->wait_buf_list);

				/* Set ptp timer */
				mse_debug("Timer start.\n");

				instance->ptp_timer_start = buf->launch_avtp_timestamp;
				if (mse_ptp_timer_start(instance->ptp_index,
							instance->ptp_timer_handle,
							(u32)(instance->ptp_timer_start)) < 0) {
					mse_err("mse_ptp_timer_start error\n");

					/* Output immediately. Skip wait_buf_list */
					buf->launch_avtp_timestamp = PTP_TIMESTAMP_INVALID;
					list_move_tail(&buf->list, &instance->done_buf_list);
				} else {
					instance->f_timer_started = true;
				}
			}
		}
	}

	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	return true;
}

static void mse_work_depacketize_video_rx(struct mse_instance *instance)
{
	struct mse_trans_buffer *buf, *buf1;
	struct mse_trans_buffer *wait_buf;
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	/* no buffer to write data */
	if (list_empty(&instance->proc_buf_list)) {
		spin_lock_irqsave(&instance->lock_buf_list, flags);

		if (mse_check_buffer_starvation(instance)) {
			/* Wait buffer extrusion */
			wait_buf = list_first_entry_or_null(&instance->wait_buf_list,
							    struct mse_trans_buffer,
							    list);
			if (wait_buf) {
				/* Move from wait_buf_list to done_buf_list */
				list_move_tail(&wait_buf->list, &instance->done_buf_list);
				queue_work(instance->wq_packet, &instance->wk_callback);
			}
		}

		spin_unlock_irqrestore(&instance->lock_buf_list, flags);

		return;
	}

	read_lock_irqsave(&instance->lock_stream, flags);
	if (!instance->f_streaming)
		queue_work(instance->wq_stream, &instance->wk_stream);
	read_unlock_irqrestore(&instance->lock_stream, flags);

	instance->f_depacketizing = true;

	list_for_each_entry_safe(buf, buf1, &instance->proc_buf_list, list)
		if (!do_depacketize_video_rx(instance, buf))
			break;

	/*
	 * If added to done_buf_list by do_depacketize_video_rx. Call
	 * wk_callback
	 */
	if (!list_empty(&instance->done_buf_list))
		queue_work(instance->wq_packet, &instance->wk_callback);

	/* state is NOT EXECUTE */
	if (!mse_state_test(instance, MSE_STATE_EXECUTE))
		instance->f_depacketizing = false;
}

static void mse_work_depacketize(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_depacketize);

	if (IS_MSE_TYPE_VIDEO(instance->media->type) && !instance->tx)
		mse_work_depacketize_video_rx(instance);
	else
		mse_work_depacketize_common(instance);
}

static void mse_work_callback_common(struct mse_instance *instance)
{
	struct mse_adapter *adapter;
	struct mse_trans_buffer *buf;
	int work_length;
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	adapter = instance->media;

	buf = list_first_entry_or_null(&instance->proc_buf_list,
				       struct mse_trans_buffer, list);

	/* no buffer to callback */
	if (!buf) {
		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);

		return; /* skip work */
	}

	if (!buf->media_buffer)
		return;

	/* no buffer is processed */
	if (!instance->tx && !atomic_read(&instance->done_buf_cnt))
		return;

	mse_debug_state(instance);

	work_length = buf->work_length;

	if (instance->tx) {
		if (work_length < buf->buffer_size)
			return;
	} else {
		if (IS_MSE_TYPE_AUDIO(adapter->type)) {
			int out_cnt, i, temp_r, temp_w, temp_len;
			bool has_valid_data, has_writing_data, f_present;
			struct mse_packetizer_ops *packetizer;

			temp_r = instance->temp_r;
			temp_w = instance->temp_w;
			temp_len = instance->temp_len[temp_w];
			packetizer = instance->packetizer;

			out_cnt = atomic_read(&instance->done_buf_cnt);
			has_valid_data = (temp_w != temp_r);

			if (!instance->ptp_timer_handle &&
			    !instance->f_present)
				has_writing_data = false;
			else
				has_writing_data = (temp_len > 0);

			f_present = instance->f_present ||
				has_valid_data || has_writing_data;

			mse_debug("instance->f_present=%d f_present=%d out_cnt=%d has_valid_data=%d has_writing_data=%d\n",
				  instance->f_present,
				  f_present,
				  out_cnt,
				  has_valid_data,
				  has_writing_data);

			if (!f_present) {
				/*
				 * it is before presenting.
				 * it has no valid data, output silent data.
				 */
				;
			} else if (has_valid_data) {
				/*
				 * it is presenting.
				 * it has valid data, output valid data
				 */
				;
			} else if (out_cnt < 2) {
				/*
				 * it is presenting but it has no valid data.
				 * wait for valid data until next period.
				 */
				out_cnt = 0;
			} else {
				/*
				 * it is presenting and waiting for valid data.
				 * since waiting already 2 period,
				 * output all waiting periods and reset status.
				 */
				instance->f_present = false;
				if (instance->ptp_timer_handle)
					packetizer->set_need_calc_offset(
						instance->index_packetizer);
			}

			/* if it has some received data, copy to buffer */
			if (out_cnt > 0 &&
			    (has_valid_data || has_writing_data)) {
				memcpy(buf->media_buffer,
				       instance->temp_buffer[temp_r],
				       instance->temp_len[temp_r]);
				buf->work_length = instance->temp_len[temp_r];
				instance->f_present = true;
				if (has_valid_data)  {
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

			/* output out_cnt buffers to adapter */
			for (i = 0; i < out_cnt && buf; i++) {
				mse_trans_complete(instance,
						   &instance->proc_buf_list,
						   buf->buffer_size);

				atomic_dec(&instance->done_buf_cnt);
				buf = list_first_entry_or_null(
					&instance->proc_buf_list,
					struct mse_trans_buffer, list);
			}
		}

		/* state is STOPPING */
		if (mse_state_test(instance, MSE_STATE_STOPPING))
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
	}

	/* complete callback */
	if (instance->tx || work_length)
		if (atomic_dec_not_zero(&instance->done_buf_cnt))
			mse_trans_complete(instance,
					   &instance->proc_buf_list,
					   work_length);

	write_lock_irqsave(&instance->lock_state, flags);

	if (mse_is_buffer_empty(instance)) {
		/* state is STOPPING */
		if (mse_state_test_nolock(instance, MSE_STATE_STOPPING)) {
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
		} else {
			/* if state is EXECUTE, change to IDLE */
			mse_state_change_if(instance, MSE_STATE_IDLE,
					    MSE_STATE_EXECUTE);
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

	write_unlock_irqrestore(&instance->lock_state, flags);
}

static void mse_work_callback_video_rx(struct mse_instance *instance)
{
	struct mse_trans_buffer *buf, *buf1;
	struct list_head buf_list;
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	mse_debug_state(instance);

	INIT_LIST_HEAD(&buf_list);

	spin_lock_irqsave(&instance->lock_buf_list, flags);
	/* Move all buffer to temp list, for lock done_buf_list. */
	list_splice_init(&instance->done_buf_list, &buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	/* complete callback */
	list_for_each_entry_safe(buf, buf1, &buf_list, list)
		mse_trans_complete(instance, &buf_list, buf->work_length);

	write_lock_irqsave(&instance->lock_state, flags);

	/* buffer is NOT empty, so wait next callback queued */
	if (!mse_is_buffer_empty(instance)) {
		write_unlock_irqrestore(&instance->lock_state, flags);
		return;
	}

	/* state is STOPPING */
	if (mse_state_test_nolock(instance, MSE_STATE_STOPPING)) {
		queue_work(instance->wq_packet, &instance->wk_stop_streaming);
	} else {
		/* if state is EXECUTE, change to IDLE */
		mse_state_change_if(instance, MSE_STATE_IDLE, MSE_STATE_EXECUTE);
	}

	write_unlock_irqrestore(&instance->lock_state, flags);
}

static void mse_work_callback_mpeg2ts_tx(struct mse_instance *instance)
{
	struct mse_trans_buffer *buf, *buf1;
	struct list_head buf_list;
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return; /* skip work */

	mse_debug_state(instance);

	INIT_LIST_HEAD(&buf_list);

	spin_lock_irqsave(&instance->lock_buf_list, flags);
	/* Move all buffer to temp list, for lock done_buf_list. */
	list_splice_init(&instance->done_buf_list, &buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	/* complete callback */
	list_for_each_entry_safe(buf, buf1, &buf_list, list)
		mse_trans_complete(instance, &buf_list, buf->work_length);

	write_lock_irqsave(&instance->lock_state, flags);

	/* buffer is NOT empty, so wait next callback queued */
	if (!mse_is_buffer_empty(instance)) {
		queue_work(instance->wq_packet, &instance->wk_packetize);
		write_unlock_irqrestore(&instance->lock_state, flags);
		return;
	}

	/* state is STOPPING */
	if (mse_state_test_nolock(instance, MSE_STATE_STOPPING)) {
		queue_work(instance->wq_packet, &instance->wk_stop_streaming);
	} else {
		/* if state is EXECUTE, change to IDLE */
		mse_state_change_if(instance, MSE_STATE_IDLE, MSE_STATE_EXECUTE);
	}

	write_unlock_irqrestore(&instance->lock_state, flags);
}

static void mse_work_callback(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_callback);

	if (instance->tx)
		if (IS_MSE_TYPE_MPEG2TS(instance->media->type))
			mse_work_callback_mpeg2ts_tx(instance);
		else
			mse_work_callback_common(instance);
	else
		if (IS_MSE_TYPE_VIDEO(instance->media->type))
			mse_work_callback_video_rx(instance);
		else
			mse_work_callback_common(instance);
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

static void mse_work_stop_streaming_common(struct mse_instance *instance)
{
	int ret;
	struct mse_adapter_network_ops *network;
	unsigned long flags;

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

static bool mse_check_streaming_complete(struct mse_instance *instance)
{
	if (list_empty(&instance->wait_packet_list) &&
	    !mse_packet_ctrl_check_packet_remain(instance->packet_buffer))
		return true;

	return false;
}

static void mse_work_stop_streaming_mpeg2ts_tx(struct mse_instance *instance)
{
	unsigned long flags;

	mse_debug_state(instance);

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED))
		return; /* skip work */

	instance->f_stopping = true;

	/* state is NOT EXECUTE */
	if (mse_state_test(instance, MSE_STATE_STOPPING)) {
		/* Wait transmission complete. */
		if (!wait_event_interruptible_timeout(
				instance->wait_wk_stream,
				mse_check_streaming_complete(instance),
				MSE_TIMEOUT_PACKETIZE_MPEG2TS)) {
			mse_debug("wait event timeouted\n");
		}

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

	queue_work(instance->wq_packet, &instance->wk_packetize);
}

static void mse_work_stop_streaming(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_stop_streaming);

	if (instance->tx)
		if (IS_MSE_TYPE_MPEG2TS(instance->media->type))
			mse_work_stop_streaming_mpeg2ts_tx(instance);
		else
			mse_work_stop_streaming_common(instance);
	else
		mse_work_stop_streaming_common(instance);
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

	if (IS_MSE_TYPE_AUDIO(instance->media->type))
		atomic_inc(&instance->done_buf_cnt);
	else if (IS_MSE_TYPE_VIDEO(instance->media->type) && instance->tx)
		atomic_inc(&instance->done_buf_cnt);

	/* timer update */
	hrtimer_add_expires_ns(&instance->timer, instance->timer_interval);

	/* start workqueue for completion */
	queue_work(instance->wq_packet, &instance->wk_callback);

	return HRTIMER_RESTART;
}

static u32 mse_ptp_timer_callback_common(struct mse_instance *instance)
{
	u64 expire_next, expire_prev;

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		return 0;
	}

	expire_prev = instance->ptp_timer_start;
	expire_next = ptp_timer_update_start_timing(instance, expire_prev);

	if (IS_MSE_TYPE_AUDIO(instance->media->type))
		atomic_inc(&instance->done_buf_cnt);

	queue_work(instance->wq_packet, &instance->wk_callback);

	return (u32)(expire_next - expire_prev);
}

static u32 mse_ptp_timer_callback_video_rx(struct mse_instance *instance)
{
	struct mse_trans_buffer *buf, *buf1;
	u64 expire_next, expire_prev;
	u64 launch_avtp_timestamp;
	unsigned long flags;

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		instance->f_timer_started = false;
		return 0;
	}

	spin_lock_irqsave(&instance->lock_buf_list, flags);

	expire_prev = instance->ptp_timer_start;
	expire_next = instance->ptp_timer_start;

	/* Search valid next timestamp */
	list_for_each_entry_safe(buf, buf1, &instance->wait_buf_list, list) {
		launch_avtp_timestamp = buf->launch_avtp_timestamp;

		/* If equal to previous timestamp, current output buffer */
		if (launch_avtp_timestamp != expire_prev) {
			/* If valid, schedule next expire time */
			if (launch_avtp_timestamp != PTP_TIMESTAMP_INVALID) {
				expire_next = launch_avtp_timestamp;
				instance->ptp_timer_start = expire_next;
				break;
			}

			/* If invalid, Output at same time as current buffer */
		}

		/* Output buffer */
		list_move_tail(&buf->list, &instance->done_buf_list);
		queue_work(instance->wq_packet, &instance->wk_callback);
	}

	/* If not found next timestamp, timer stop */
	if (expire_next == expire_prev)
		instance->f_timer_started = false;

	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	mse_debug("ret = %u\n", PTP_TIME_DIFF_S32(expire_next, expire_prev));

	return (u32)PTP_TIME_DIFF_S32(expire_next, expire_prev);
}

static u32 mse_ptp_timer_callback_mpeg2ts_tx(struct mse_instance *instance)
{
	struct mse_wait_packet *wp, *wp1;
	u64 expire_next, expire_prev;
	u64 launch_avtp_timestamp;
	unsigned long flags;

	/* state is NOT STARTED */
	if (!mse_state_test(instance, MSE_STATE_STARTED)) {
		mse_debug("stopping ...\n");
		instance->f_timer_started = false;
		return 0;
	}

	read_lock_irqsave(&instance->lock_stream, flags);

	expire_prev = instance->ptp_timer_start;
	expire_next = instance->ptp_timer_start;

	/* Search valid next timestamp */
	list_for_each_entry_safe(wp, wp1, &instance->wait_packet_list, list) {
		launch_avtp_timestamp = wp->launch_avtp_timestamp;

		/* If equal to previous timestamp, current output buffer */
		if (PTP_TIME_DIFF_S32(launch_avtp_timestamp, expire_prev) <= 0) {
			/* Release limit */
			instance->packet_buffer->wait_p = wp->release_p;
			list_del(&wp->list);
		} else {
			/* Schedule next expire time */
			expire_next = launch_avtp_timestamp;
			instance->ptp_timer_start = expire_next;
			break;
		}
	}

	/* start workqueue for streaming */
	if (!instance->f_streaming)
		queue_work(instance->wq_stream, &instance->wk_stream);

	/* If not found next timestamp, timer stop */
	if (expire_next == expire_prev)
		instance->f_timer_started = false;

	read_unlock_irqrestore(&instance->lock_stream, flags);

	mse_debug("ret = %u\n", PTP_TIME_DIFF_S32(expire_next, expire_prev));

	return (u32)PTP_TIME_DIFF_S32(expire_next, expire_prev);
}

static u32 mse_ptp_timer_callback(void *arg)
{
	struct mse_instance *instance = arg;

	if (instance->tx)
		if (IS_MSE_TYPE_MPEG2TS(instance->media->type))
			return mse_ptp_timer_callback_mpeg2ts_tx(instance);
		else
			return mse_ptp_timer_callback_common(instance);
	else
		if (IS_MSE_TYPE_VIDEO(instance->media->type))
			return mse_ptp_timer_callback_video_rx(instance);
		else
			return mse_ptp_timer_callback_common(instance);
}

static void mse_work_crf_send(struct work_struct *work)
{
	struct mse_instance *instance;
	int err, tsize, size, i;
	u64 timestamps[CRF_AUDIO_TIMESTAMPS];

	mse_debug("START\n");

	instance = container_of(work, struct mse_instance, wk_crf_send);

	/* state is NOT RUNNABLE */
	if (!mse_state_test(instance, MSE_STATE_RUNNABLE)) {
		instance->f_crf_sending = false;
		return;
	}

	tsize = (!instance->f_ptp_capture) ?
		CRF_PTP_TIMESTAMPS : CRF_AUDIO_TIMESTAMPS;

	do {
		size = tstamps_deq_tstamps(&instance->tstamp_que_crf,
					   timestamps,
					   tsize,
					   tsize);
		if (size != tsize)
			break;

		for (i = 0; i < tsize; i++) {
			if (instance->tx)
				timestamps[i] += instance->max_transit_time_ns;

			if (instance->f_ptp_capture)
				timestamps[i] +=
					instance->capture_delay_time_ns;
		}

		/* create CRF packets */
		err = mse_packet_ctrl_make_packet_crf(
			instance->crf_index,
			timestamps,
			tsize,
			instance->crf_packet_buffer);

		if (err < 0)
			break;

		/* send packets */
		err = mse_packet_ctrl_send_packet(
			instance->crf_index_network,
			instance->crf_packet_buffer,
			instance->network);

		if (err < 0) {
			mse_err("send error %d\n", err);
			break;
		}

		/* state is RUNNABLE */
	} while (mse_state_test(instance, MSE_STATE_RUNNABLE));

	instance->f_crf_sending = false;
}

static void mse_work_crf_receive(struct work_struct *work)
{
	struct mse_instance *instance;
	struct mse_audio_info audio_info;
	int err, count;
	u64 ptimes[6];
	struct mse_packetizer_ops *crf =
		&mse_packetizer_crf_timestamp_audio_ops;

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

		if (!instance->crf_que.f_init)
			tstamps_init(&instance->crf_que,
				     "CRF",
				     true,
				     audio_info.frame_interval_time);

		tstamps_enq_tstamps(&instance->crf_que, ptimes, count);

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

		instance->capture_delay_time_ns = instance->delay_time_ns;
		if (i) {
			i--;
		} else {
			/* not enough timestamps */
			instance->capture_delay_time_ns += diff;
		}
	} else {
		i = 0;
	}

	/* store timestamps */
	tstamps_enq_tstamps(&instance->tstamp_que,
			    instance->timestamps, count);
	tstamps_enq_tstamps(&instance->tstamp_que_crf,
			    instance->timestamps, count);

	return count;
}

static void mse_start_streaming_audio(struct mse_instance *instance, u64 now)
{
	int i;
	int captured;

	if (!instance->tx) {
		instance->temp_w = 0;
		instance->temp_r = 0;
		for (i = 0; i < MSE_DECODE_BUFFER_NUM; i++)
			instance->temp_len[i] = 0;
	}

	instance->f_wait_start_transmission = false;

	/* f_ptp_capture is true, capture timestamps */
	if (instance->f_ptp_capture) {
		captured = mse_get_capture_timestamp(
				instance,
				now,
				true);
		instance->captured_timestamps += captured;
	}

	if (instance->ptp_timer_handle) {
		/* initialize timestamp readers */
		tstamps_reader_init(&instance->reader_ptp_start_time,
				    &instance->tstamp_que,
				    "PERIOD",
				    false);

		/* request calc offset */
		instance->packetizer->set_need_calc_offset(
			instance->index_packetizer);
	}

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
	instance->f_timer_started = false;
	instance->captured_timestamps = 0;

	instance->f_get_first_packet = false;
	instance->remain = 0;
}

static void mse_start_streaming_common(struct mse_instance *instance)
{
	reinit_completion(&instance->completion_stop);
	instance->f_streaming = false;
	instance->f_continue = false;
	instance->f_stopping = false;
	instance->f_completion = false;
	instance->f_start_stream = false;
	instance->f_depacketizing = false;
	instance->f_first_pcr = false;
	instance->mpeg2ts_pre_pcr_pid = MPEG2TS_PCR_PID_IGNORE;
	instance->mpeg2ts_pcr_27m = MPEG2TS_PCR27M_INVALID;
	instance->mpeg2ts_pre_pcr_27m = MPEG2TS_PCR27M_INVALID;
	instance->processed = 0;

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

static u64 ptp_timer_update_start_timing(struct mse_instance *instance, u64 now)
{
	u64 ptp_timer_start, next_time;
	s64 diff, update;
	u64 error_thresh;
	u32 interval, offset;
	int ret;

	ptp_timer_start = instance->ptp_timer_start;
	offset = instance->delay_time_ns;
	interval = instance->timer_interval;
	error_thresh = PTP_TIMER_ERROR_THRESHOLD(interval);

	/* if NOT set ptp_timer_start, then init ptp_timer_start */
	if (!ptp_timer_start)
		ptp_timer_start = now;

	ret = tstamps_calc_tstamp(&instance->reader_ptp_start_time,
				  now,
				  offset,
				  interval,
				  &next_time);

	update = now - next_time;
	diff = ptp_timer_start - next_time;

	/* check 'update' and 'diff' within error threshold */
	if (ret >= 0 &&
	    ((abs(update) < error_thresh) && (abs(diff) < error_thresh)))
		ptp_timer_start = next_time;

	ptp_timer_start += interval;

	mse_debug_tstamps2("start %u next %llu now %llu update %llu diff %lld\n",
			   (u32)ptp_timer_start, next_time, now, update, diff);

	instance->ptp_timer_start = ptp_timer_start;

	return ptp_timer_start;
}

static void mse_work_start_transmission_common(struct mse_instance *instance)
{
	struct mse_adapter *adapter;
	struct mse_trans_buffer *buf;
	int ret;
	u64 now, ptp_timer_start;
	unsigned long flags;

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
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
		}

		return;
	} else if (IS_MSE_TYPE_AUDIO(instance->media->type)) {
		if (mse_state_test(instance, MSE_STATE_STOPPING)) {
			spin_unlock_irqrestore(&instance->lock_buf_list,
					       flags);
			queue_work(instance->wq_packet,
				   &instance->wk_stop_streaming);
			return;
		}
	}

	if (instance->f_ptp_capture &&
	    instance->ptp_timer_handle &&
	    instance->captured_timestamps < PTP_SYNC_LOCK_THRESHOLD) {
		spin_unlock_irqrestore(&instance->lock_buf_list, flags);
		instance->f_wait_start_transmission = true;

		return;
	}

	list_move_tail(&buf->list, &instance->proc_buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	mse_debug("index=%d buffer=%p buffer_size=%zu\n",
		  instance->media->index, buf->media_buffer,
		  buf->buffer_size);

	/* update timestamp(nsec) */
	mse_ptp_get_time(instance->ptp_index, &now);
	instance->timestamp = now;

	if (!instance->f_ptp_capture) {
		/* store ptp timestamp to AVTP and CRF */
		tstamps_enq_tstamp(&instance->tstamp_que, now);
		tstamps_enq_tstamp(&instance->tstamp_que_crf, now);
	} else if (instance->ptp_timer_handle &&
		   !instance->f_timer_started) {
		ptp_timer_start = ptp_timer_update_start_timing(instance, now);
		if (mse_ptp_timer_start(instance->ptp_index,
					instance->ptp_timer_handle,
					(u32)ptp_timer_start) < 0) {
			mse_err("mse_ptp_timer_start error\n");
			return;
		}
		instance->f_timer_started = true;
	}

	adapter = instance->media;
	if (instance->tx) {
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

				mse_trans_complete(instance,
						   &instance->proc_buf_list,
						   ret);

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

static void mse_work_start_transmission_video_rx(struct mse_instance *instance)
{
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return;

	spin_lock_irqsave(&instance->lock_buf_list, flags);
	list_splice_tail_init(&instance->trans_buf_list,
			      &instance->proc_buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	/* start workqueue for depacketize */
	queue_work(instance->wq_packet, &instance->wk_depacketize);
}

static void mse_work_start_transmission_mpeg2ts_tx(struct mse_instance *instance)
{
	unsigned long flags;

	/* state is NOT RUNNING */
	if (!mse_state_test(instance, MSE_STATE_RUNNING))
		return;

	spin_lock_irqsave(&instance->lock_buf_list, flags);
	list_splice_tail_init(&instance->trans_buf_list,
			      &instance->proc_buf_list);
	spin_unlock_irqrestore(&instance->lock_buf_list, flags);

	/* start workqueue for packetize */
	queue_work(instance->wq_packet, &instance->wk_packetize);
}

static void mse_work_start_transmission(struct work_struct *work)
{
	struct mse_instance *instance;

	instance = container_of(work, struct mse_instance, wk_start_trans);

	if (instance->tx)
		if (IS_MSE_TYPE_MPEG2TS(instance->media->type))
			mse_work_start_transmission_mpeg2ts_tx(instance);
		else
			mse_work_start_transmission_common(instance);
	else
		if (IS_MSE_TYPE_VIDEO(instance->media->type))
			mse_work_start_transmission_video_rx(instance);
		else
			mse_work_start_transmission_common(instance);
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
	return &mse->media_table[index]->config;
}

bool mse_dev_is_busy(int index)
{
	return mse->media_table[index]->ro_config_f;
}

/* External function */
int mse_register_adapter_media(enum MSE_TYPE type,
			       char *name,
			       char *device_name)
{
	struct mse_adapter *media;
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

	media = kzalloc(sizeof(*media), GFP_KERNEL);
	if (!media)
		return -ENOMEM;

	/* search unused index */
	spin_lock_irqsave(&mse->lock_media_table, flags);
	index = find_first_zero_bit(mse->mse_media_map, MSE_ADAPTER_MEDIA_MAX);
	if (ARRAY_SIZE(mse->media_table) <= index) {
		mse_err("%s is not registered\n", name);
		spin_unlock_irqrestore(&mse->lock_media_table, flags);
		kfree(media);

		return -EBUSY;
	}
	set_bit(index, mse->mse_media_map);
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	/* init table */
	media->index = index;
	media->type = type;
	strncpy(media->name, name, MSE_NAME_LEN_MAX);

	/* init config data */
	mse_config_init(&media->config,
			mse_type_to_stream_type(type),
			device_name);

	/* create control device */
	if (mse_create_config_device(media) < 0) {
		spin_lock_irqsave(&mse->lock_media_table, flags);
		clear_bit(index, mse->mse_media_map);
		spin_unlock_irqrestore(&mse->lock_media_table, flags);
		kfree(media);
		mse_err("%s is not registered\n", name);

		return -EPERM;
	}

	spin_lock_irqsave(&mse->lock_media_table, flags);
	mse->media_table[index] = media;
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	mse_debug("registered index=%d\n", index);

	return index;
}
EXPORT_SYMBOL(mse_register_adapter_media);

int mse_unregister_adapter_media(int index_media)
{
	struct mse_adapter *media;
	unsigned long flags;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		mse_err("invalid argument. index=%d\n", index_media);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index_media);

	spin_lock_irqsave(&mse->lock_media_table, flags);
	media = mse->media_table[index_media];
	if (!media) {
		spin_unlock_irqrestore(&mse->lock_media_table, flags);
		mse_err("%d is not registered\n", index_media);

		return -EINVAL;
	}

	if (media->ro_config_f) {
		mse_err("%s module is using in mse_core\n", media->name);
		spin_unlock_irqrestore(&mse->lock_media_table, flags);

		return -EPERM;
	}
	mse->media_table[index_media] = NULL;
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	/* delete control device */
	mse_delete_config_device(media);

	spin_lock_irqsave(&mse->lock_media_table, flags);
	clear_bit(index_media, mse->mse_media_map);
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	kfree(media);

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

	spin_lock_irqsave(&mse->lock_network_table, flags);

	/* search unused index */
	for (index = 0; index < ARRAY_SIZE(mse->network_table) &&
	     mse->network_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->network_table)) {
		spin_unlock_irqrestore(&mse->lock_network_table, flags);
		mse_err("%s is not registered\n", name);

		return -EBUSY;
	}

	/* register table */
	mse->network_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_network_table, flags);

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

	spin_lock_irqsave(&mse->lock_network_table, flags);
	if (!mse->network_table[index]) {
		spin_unlock_irqrestore(&mse->lock_network_table, flags);
		mse_err("%d is not registered\n", index);

		return -EINVAL;
	}
	spin_unlock_irqrestore(&mse->lock_network_table, flags);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i] &&
		    mse->instance_table[i]->index_network == index) {
			spin_unlock_irqrestore(&mse->lock_instance_table,
					       flags);
			mse_err("module is in use. instance=%d\n", i);

			return -EPERM;
		}
	}
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	spin_lock_irqsave(&mse->lock_network_table, flags);
	mse->network_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_network_table, flags);

	mse_debug("unregistered\n");

	return 0;
}
EXPORT_SYMBOL(mse_unregister_adapter_network);

int mse_get_audio_config(int index, struct mse_audio_config *config)
{
	struct mse_instance *instance;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}
	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	mse_debug_state(instance);

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
	unsigned long flags;

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	mse_debug_state(instance);

	down(&instance->sem_stopping);

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
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	mse_debug_state(instance);

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
	unsigned long flags;

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	mse_debug_state(instance);

	down(&instance->sem_stopping);

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
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	mse_debug("index=%d data=%p\n", index, config);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

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
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	if (!config) {
		mse_err("invalid argument. config\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	mse_debug_state(instance);

	down(&instance->sem_stopping);

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

static void mse_cleanup_crf_network_interface(struct mse_instance *instance)
{
	struct mse_adapter_network_ops *network = instance->network;

	if (instance->crf_index_network < 0)
		return;

	instance->network->release(instance->crf_index_network);
	instance->crf_index_network = MSE_INDEX_UNDEFINED;
	module_put(network->owner);

	mse_packet_ctrl_free(instance->crf_packet_buffer);
	instance->crf_packet_buffer = NULL;
}

static int mse_setup_crf_network_interface(struct mse_instance *instance)
{
	int ret;
	int index_network;
	struct mse_adapter_network_ops *network;
	struct mse_network_device *network_device;
	struct mse_packet_ctrl *packet_buffer;
	char *dev_name;
	int ring_size;
	bool tx;

	network_device = &instance->media->config.network_device;
	if (instance->crf_type == MSE_CRF_TYPE_TX) {
		dev_name = (char *)network_device->device_name_tx_crf;
		ring_size = MSE_CRF_TX_RING_SIZE;
		tx = true;
	} else if (instance->crf_type == MSE_CRF_TYPE_RX) {
		dev_name = (char *)network_device->device_name_rx_crf;
		ring_size = MSE_CRF_RX_RING_SIZE;
		tx = false;
	} else {
		return 0;
	}

	network = instance->network;

	if (!try_module_get(network->owner)) {
		mse_err("try_module_get() fail\n");

		return -EBUSY;
	}

	index_network = network->open(dev_name);
	if (index_network < 0) {
		module_put(network->owner);

		return index_network;
	}

	instance->crf_net_config.port_transmit_rate =
		instance->net_config.port_transmit_rate;

	/* allocate packet buffer */
	packet_buffer = mse_packet_ctrl_alloc(&mse->pdev->dev,
					      ring_size,
					      MSE_PACKET_SIZE_MAX);
	if (!packet_buffer) {
		network->release(index_network);
		module_put(network->owner);

		return -ENOMEM;
	}

	/* associate crf packet buffer with network adapter */
	if (tx)
		ret = mse_packet_ctrl_send_prepare_packet(index_network,
							  packet_buffer,
							  network);
	else
		ret = mse_packet_ctrl_receive_prepare_packet(index_network,
							     packet_buffer,
							     network);
	if (ret) {
		mse_packet_ctrl_free(packet_buffer);
		network->release(index_network);
		module_put(network->owner);

		return ret;
	}

	instance->crf_index_network = index_network;
	instance->crf_packet_buffer = packet_buffer;

	return 0;
}

static void mse_cleanup_ptp(struct mse_instance *instance)
{

	if (instance->ptp_timer_handle) {
		mse_ptp_timer_close(instance->ptp_index,
				    instance->ptp_timer_handle);

		instance->ptp_timer_handle = NULL;
	}

	if (instance->ptp_handle) {
		mse_ptp_capture_stop(instance->ptp_index,
				     instance->ptp_handle);

		mse_ptp_close(instance->ptp_index,
			      instance->ptp_handle);

		instance->ptp_handle = NULL;
	}
}

static int mse_setup_ptp(struct mse_instance *instance)
{
	int err = 0;
	void *ptp_handle;

	/* ptp open */
	ptp_handle = mse_ptp_open(instance->ptp_index);
	if (!ptp_handle) {
		mse_err("cannot mse_ptp_open()\n");

		return -ENODEV;
	}

	mse_info("mse_ptp_capture_start %d %p %d %zu\n",
		 instance->ptp_index,
		 ptp_handle,
		 instance->ptp_clock_ch,
		 ARRAY_SIZE(instance->timestamps));

	err = mse_ptp_capture_start(instance->ptp_index,
				    ptp_handle,
				    instance->ptp_clock_ch,
				    ARRAY_SIZE(instance->timestamps));

	if (err < 0) {
		mse_err("cannot mse_ptp_capture_start()\n");
		mse_ptp_close(instance->ptp_index, ptp_handle);

		return err;
	}

	instance->ptp_handle = ptp_handle;

	return 0;
}

static int mse_setup_ptp_timer(struct mse_instance *instance)
{
	void *ptp_timer_handle;

	ptp_timer_handle = mse_ptp_timer_open(instance->ptp_index,
					      mse_ptp_timer_callback,
					      instance);
	if (!ptp_timer_handle)
		mse_warn("cannot open ptp_timer, fallback using hires timer\n");

	instance->ptp_timer_handle = ptp_timer_handle;

	return 0;
}

static void mse_cleanup_mch(struct mse_instance *instance)
{
	struct mch_ops *m_ops = NULL;

	if (instance->mch_index < 0)
		return;

	m_ops = mse->mch_table[instance->mch_index];
	m_ops->close(instance->mch_handle);

	instance->mch_index = MSE_INDEX_UNDEFINED;
	module_put(m_ops->owner);
}

static int mse_setup_mch(struct mse_instance *instance)
{
	int i;
	struct mch_ops *m_ops = NULL;
	void *mch_handle;

	for (i = 0; i < ARRAY_SIZE(mse->mch_table); i++) {
		m_ops = mse->mch_table[i];
		if (m_ops)
			break;
	}
	if (i >= ARRAY_SIZE(mse->mch_table)) {
		mse_err("mch is not registered.\n");

		return -EINVAL;
	}

	if (!try_module_get(m_ops->owner)) {
		mse_err("try_module_get() fail\n");

		return -EBUSY;
	}

	mch_handle = m_ops->open();
	if (!mch_handle) {
		mse_err("mch open error.\n");
		module_put(m_ops->owner);

		return -EINVAL;
	}

	tstamps_reader_init(&instance->reader_mch,
			    &instance->tstamp_que,
			    "MCH",
			    true);

	instance->mch_index = i;
	instance->mch_handle = mch_handle;

	return 0;
}

static void mse_exit_kernel_resource(struct mse_instance *instance,
				     struct mse_adapter *adapter)

{
	/* flush workqueue */
	if (instance->wq_crf_packet)
		flush_workqueue(instance->wq_crf_packet);

	if (instance->wq_tstamp)
		flush_workqueue(instance->wq_tstamp);

	if (instance->wq_packet)
		flush_workqueue(instance->wq_packet);

	if (instance->wq_stream)
		flush_workqueue(instance->wq_stream);

	/* destroy workqueue */
	if (instance->wq_crf_packet)
		destroy_workqueue(instance->wq_crf_packet);

	if (instance->wq_tstamp)
		destroy_workqueue(instance->wq_tstamp);

	if (instance->wq_packet)
		destroy_workqueue(instance->wq_packet);

	if (instance->wq_stream)
		destroy_workqueue(instance->wq_stream);
}

static int mse_init_kernel_resource(struct mse_instance *instance,
				    struct mse_adapter *adapter)
{
	struct workqueue_struct *wq_work;

	init_completion(&instance->completion_stop);
	complete(&instance->completion_stop);
	atomic_set(&instance->trans_buf_cnt, 0);
	atomic_set(&instance->done_buf_cnt, 0);
	init_waitqueue_head(&instance->wait_wk_stream);
	INIT_LIST_HEAD(&instance->trans_buf_list);
	INIT_LIST_HEAD(&instance->proc_buf_list);
	INIT_LIST_HEAD(&instance->wait_buf_list);
	INIT_LIST_HEAD(&instance->done_buf_list);
	INIT_LIST_HEAD(&instance->wait_packet_list);
	rwlock_init(&instance->lock_state);
	rwlock_init(&instance->lock_stream);
	spin_lock_init(&instance->lock_timer);
	spin_lock_init(&instance->lock_buf_list);
	sema_init(&instance->sem_stopping, 1);

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

	wq_work = create_singlethread_workqueue("mse_streamq");
	if (!wq_work)
		goto error_create_singlethread_wq;

	instance->wq_stream = wq_work;

	wq_work = create_singlethread_workqueue("mse_packetq");
	if (!wq_work)
		goto error_create_singlethread_wq;

	instance->wq_packet = wq_work;

	/* for timestamp */
	wq_work = create_singlethread_workqueue("mse_tstampq");
	if (!wq_work)
		goto error_create_singlethread_wq;

	instance->wq_tstamp = wq_work;

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
		wq_work = create_singlethread_workqueue("mse_crfpacketq");
		if (!wq_work)
			goto error_create_singlethread_wq;

		instance->wq_crf_packet = wq_work;

		hrtimer_init(&instance->crf_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		instance->crf_timer_interval = CRF_TIMER_INTERVAL;
		instance->crf_timer.function = &mse_crf_callback;
	}

	return 0;

error_create_singlethread_wq:
	mse_exit_kernel_resource(instance, adapter);

	return -ENOMEM;
}

static void mse_cleanup_network_interface(struct mse_instance *instance)
{
	struct mse_adapter_network_ops *network = instance->network;

	if (instance->index_network < 0)
		return;

	instance->network->release(instance->index_network);
	instance->index_network = MSE_INDEX_UNDEFINED;
	module_put(network->owner);

	if (IS_MSE_TYPE_MPEG2TS(instance->media->type) && instance->tx) {
		kfree(instance->wait_packet);
		instance->wait_packet = NULL;
	}

	mse_packet_ctrl_free(instance->packet_buffer);
	instance->packet_buffer = NULL;
}

static int mse_setup_network_interface(struct mse_instance *instance,
				       struct mse_adapter *media,
				       bool tx)
{
	int index_network;
	struct mse_adapter_network_ops *network;
	struct mse_network_device *network_device;
	struct mse_packet_ctrl *packet_buffer;
	struct mse_wait_packet *wait_packet;
	char *dev_name;
	char name[MSE_NAME_LEN_MAX + 1];
	long link_speed;
	int i, ret, ring_size;
	unsigned long flags;

	network_device = &media->config.network_device;

	/* search network adapter name for configuration value */
	spin_lock_irqsave(&mse->lock_network_table, flags);
	for (i = 0; i < ARRAY_SIZE(mse->network_table); i++) {
		network = mse->network_table[i];
		if (!network)
			continue;

		if (!mse_compare_param_key((char *)network_device->module_name,
					   network->name))
			break;
	}
	spin_unlock_irqrestore(&mse->lock_network_table, flags);

	if (i >= ARRAY_SIZE(mse->network_table)) {
		mse_err("network adapter module is not loaded\n");

		return -ENODEV;
	}

	mse_name_strlcpy(name, network->name);
	mse_debug("network adapter index=%d name=%s\n", i, name);

	if (tx) {
		dev_name = (char *)network_device->device_name_tx;
		ring_size = MSE_TX_RING_SIZE;
	} else {
		dev_name = (char *)network_device->device_name_rx;
		ring_size = MSE_RX_RING_SIZE;
	}

	if (!try_module_get(network->owner)) {
		mse_err("try_module_get() fail\n");

		return -EBUSY;
	}

	/* open network adapter */
	ret = network->open(dev_name);
	if (ret < 0) {
		mse_err("cannot open network adapter ret=%d\n", ret);
		module_put(network->owner);

		return ret;
	}
	index_network = ret;

	/* get speed link */
	link_speed = network->get_link_speed(index_network);
	if (link_speed <= 0) {
		mse_err("Link Down. ret=%ld\n", link_speed);
		network->release(index_network);
		module_put(network->owner);

		return -ENETDOWN;
	}

	mse_debug("Link Speed=%ldMbps\n", link_speed);

	instance->net_config.port_transmit_rate = mbit_to_bit(link_speed);

	/* allocate packet buffer */
	packet_buffer = mse_packet_ctrl_alloc(&mse->pdev->dev,
					      ring_size,
					      MSE_PACKET_SIZE_MAX);
	if (!packet_buffer) {
		network->release(index_network);
		module_put(network->owner);

		return -ENOMEM;
	}

	/* associate packet buffer with network adapter */
	if (tx)
		ret = mse_packet_ctrl_send_prepare_packet(index_network,
							  packet_buffer,
							  network);
	else
		ret = mse_packet_ctrl_receive_prepare_packet(index_network,
							     packet_buffer,
							     network);
	if (ret) {
		mse_packet_ctrl_free(packet_buffer);
		network->release(index_network);
		module_put(network->owner);

		return ret;
	}

	if (IS_MSE_TYPE_MPEG2TS(instance->media->type) && instance->tx) {
		wait_packet = kmalloc_array(MSE_TX_RING_SIZE,
					    sizeof(struct mse_wait_packet),
					    GFP_KERNEL);
		if (!wait_packet) {
			mse_packet_ctrl_free(packet_buffer);
			network->release(index_network);
			module_put(network->owner);

			return -ENOMEM;
		}

		instance->wait_packet = wait_packet;
		instance->wait_packet_idx = 0;
	}

	instance->network = network;
	instance->index_network = index_network;
	instance->packet_buffer = packet_buffer;

	return 0;
}

static void mse_release_packetizer(struct mse_instance *instance)
{
	if (instance->index_packetizer < 0)
		return;

	instance->packetizer->release(instance->index_packetizer);
	instance->index_packetizer = MSE_INDEX_UNDEFINED;
}

static int mse_open_packetizer(struct mse_instance *instance,
			       enum MSE_PACKETIZER packetizer_id)
{
	int ret;
	struct mse_packetizer_ops *packetizer;

	/* Get packetizer */
	mse_debug("packetizer id=%d\n", packetizer_id);
	packetizer = mse_packetizer_get_ops(packetizer_id);
	if (!packetizer) {
		mse_err("packetizer is not valid\n");

		return -EINVAL;
	}

	/* open packetizer */
	ret = mse_packetizer_open(packetizer_id);
	if (ret < 0) {
		mse_err("cannot open packetizer ret=%d\n", ret);

		return ret;
	}

	instance->index_packetizer = ret;
	instance->packetizer = packetizer;
	instance->packetizer_id = packetizer_id;

	/* init packetizer */
	ret = instance->packetizer->init(instance->index_packetizer);
	if (ret < 0)
		return ret;

	return 0;
}

static void mse_resource_release(struct mse_instance *instance)
{
	mse_cleanup_mch(instance);

	mse_cleanup_ptp(instance);

	mse_ptp_put_index(instance->ptp_index);
	instance->ptp_index = MSE_INDEX_UNDEFINED;

	mse_release_crf_packetizer(instance);

	mse_cleanup_crf_network_interface(instance);

	mse_release_packetizer(instance);

	mse_cleanup_network_interface(instance);
}

int mse_open(int index_media, bool tx)
{
	struct mse_instance *instance;
	struct mse_adapter *adapter;
	int index, err = 0;
	unsigned long flags;

	if ((index_media < 0) || (index_media >= MSE_ADAPTER_MEDIA_MAX)) {
		mse_err("invalid argument. index=%d\n", index_media);

		return -EINVAL;
	}

	mse_debug("index=%d tx=%d\n", index_media, tx);

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (!instance)
		return -ENOMEM;

	instance->tx = tx;
	instance->state = MSE_STATE_CLOSE;
	instance->crf_index = MSE_INDEX_UNDEFINED;
	instance->index_network = MSE_INDEX_UNDEFINED;
	instance->crf_index_network = MSE_INDEX_UNDEFINED;
	instance->index_packetizer = MSE_INDEX_UNDEFINED;
	instance->mch_index = MSE_INDEX_UNDEFINED;
	instance->ptp_index = MSE_INDEX_UNDEFINED;

	spin_lock_irqsave(&mse->lock_media_table, flags);
	adapter = mse->media_table[index_media];

	if (!adapter) {
		spin_unlock_irqrestore(&mse->lock_media_table, flags);
		mse_err("undefined media adapter index=%d\n", index_media);
		kfree(instance);

		return -ENODEV;
	}
	adapter->ro_config_f = true;
	instance->media = adapter;
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	index = find_first_zero_bit(mse->mse_instance_map, MSE_INSTANCE_MAX);
	if (ARRAY_SIZE(mse->instance_table) <= index) {
		spin_unlock_irqrestore(&mse->lock_instance_table, flags);
		mse_err("resister instance full!\n");
		err = -EBUSY;

		goto error_mse_resource_release;
	}
	set_bit(index, mse->mse_instance_map);
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	mse_debug_state(instance);

	mse_get_default_config(instance, adapter, tx);

	if (check_mch_config(instance)) {
		mse_err("media clock recovery config is invalid mch_config.enable=%d media_audio_config.crf_type=%d\n",
			instance->f_mch_enable, instance->crf_type);
		err = -EINVAL;

		goto error_mse_resource_release;
	}

	/* setup network interface & open packetizer */
	err = mse_setup_network_interface(instance, adapter, tx);
	if (err < 0)
		goto error_mse_resource_release;

	err = mse_open_packetizer(instance,
				  adapter->config.packetizer.packetizer);
	if (err < 0)
		goto error_mse_resource_release;

	err = mse_setup_crf_network_interface(instance);
	if (err < 0)
		goto error_mse_resource_release;

	err = mse_open_crf_packetizer(instance);
	if (err < 0)
		goto error_mse_resource_release;

	instance->ptp_index = mse_ptp_get_first_index();

	/* open PTP capture and PTP Timer */
	if (instance->f_ptp_capture) {
		err = mse_setup_ptp(instance);
		if (err < 0)
			goto error_mse_resource_release;
	}

	/* open PTP Timer */
	if (instance->f_ptp_timer) {
		err = mse_setup_ptp_timer(instance);
		if (err < 0)
			goto error_mse_resource_release;
	}

	if (instance->f_mch_enable) {
		err = mse_setup_mch(instance);
		if (err < 0)
			goto error_mse_resource_release;
	}

	err = mse_init_kernel_resource(instance, adapter);
	if (err < 0)
		goto error_mse_resource_release;

	mse_state_change(instance, MSE_STATE_OPEN);
	spin_lock_irqsave(&mse->lock_instance_table, flags);
	mse->instance_table[index] = instance;
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	return index;

error_mse_resource_release:
	mse_resource_release(instance);

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	clear_bit(index, mse->mse_instance_map);
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	spin_lock_irqsave(&mse->lock_media_table, flags);
	adapter->ro_config_f = false;
	spin_unlock_irqrestore(&mse->lock_media_table, flags);

	kfree(instance);

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

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

	err = mse_state_change(instance, MSE_STATE_CLOSE);
	write_unlock_irqrestore(&instance->lock_state, flags);
	if (err)
		return err;

	mse_exit_kernel_resource(instance, adapter);

	/* remove instance from resource table */
	spin_lock_irqsave(&mse->lock_instance_table, flags);
	mse->instance_table[index] = NULL;
	clear_bit(index, mse->mse_instance_map);
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	mse_resource_release(instance);
	spin_lock_irqsave(&mse->lock_media_table, flags);
	adapter->ro_config_f = false;
	spin_unlock_irqrestore(&mse->lock_media_table, flags);
	kfree(instance);

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

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
	unsigned long flags;

	if ((index < 0) || (index >= MSE_INSTANCE_MAX)) {
		mse_err("invalid argument. index=%d\n", index);
		return -EINVAL;
	}

	mse_debug("index=%d\n", index);
	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

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
	u64 now;
	int idx;
	unsigned long flags, flags2;

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	instance = mse->instance_table[index];
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	if (!instance) {
		mse_err("operation is not permitted. index=%d\n", index);
		return -EPERM;
	}

	write_lock_irqsave(&instance->lock_state, flags2);
	mse_debug_state(instance);

	/* state is STOPPING */
	if (mse_state_test_nolock(instance, MSE_STATE_STOPPING)) {
		write_unlock_irqrestore(&instance->lock_state, flags2);
		mse_err("instance is busy. index=%d\n", index);

		return -EBUSY;
	}

	/* state is NOT RUNNABLE */
	if (!mse_state_test_nolock(instance, MSE_STATE_RUNNABLE)) {
		write_unlock_irqrestore(&instance->lock_state, flags2);
		mse_err("operation is not permitted. index=%d\n", index);

		return -EPERM;
	}

	err = mse_state_change(instance, MSE_STATE_EXECUTE);
	if (!err) {
		buf_cnt = atomic_read(&instance->trans_buf_cnt);
		if (buf_cnt >= MSE_TRANS_BUF_ACCEPTABLE) {
			write_unlock_irqrestore(&instance->lock_state, flags2);
			return -EAGAIN;
		}

		spin_lock_irqsave(&instance->lock_buf_list, flags);
		idx = instance->trans_idx;
		buf = &instance->trans_buffer[idx];
		buf->media_buffer = buffer;
		buf->buffer = buffer;
		buf->buffer_size = buffer_size;
		buf->work_length = 0;
		buf->private_data = priv;
		buf->mse_completion = mse_completion;

		if (instance->tx &&
		    IS_MSE_TYPE_MPEG2TS(instance->media->type)) {
			/* update timestamp(nsec) */
			mse_ptp_get_time(instance->ptp_index, &now);

			buf->launch_avtp_timestamp = (u32)(now + instance->max_transit_time_ns);
		} else {
			buf->launch_avtp_timestamp = PTP_TIMESTAMP_INVALID;
		}

		instance->trans_idx = (idx + 1) % MSE_TRANS_BUF_NUM;
		list_add_tail(&buf->list, &instance->trans_buf_list);
		atomic_inc(&instance->trans_buf_cnt);

		spin_unlock_irqrestore(&instance->lock_buf_list, flags);
		queue_work(instance->wq_packet, &instance->wk_start_trans);
	}

	write_unlock_irqrestore(&instance->lock_state, flags2);

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

	spin_lock_irqsave(&mse->lock_mch_table, flags);

	for (index = 0; index < ARRAY_SIZE(mse->mch_table) &&
	     mse->mch_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->mch_table)) {
		mse_err("ops is not registered\n");
		spin_unlock_irqrestore(&mse->lock_mch_table, flags);

		return -EBUSY;
	}

	/* init table */
	mse->mch_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_mch_table, flags);

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i] &&
		    mse->instance_table[i]->mch_index == index) {
			spin_unlock_irqrestore(&mse->lock_instance_table,
					       flags);
			mse_err("module is in use. instance=%d\n", i);

			return -EPERM;
		}
	}
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	spin_lock_irqsave(&mse->lock_mch_table, flags);
	mse->mch_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_mch_table, flags);

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

	spin_lock_irqsave(&mse->lock_ptp_table, flags);

	/* search unused index */
	for (index = 0; index < ARRAY_SIZE(mse->ptp_table) &&
	     mse->ptp_table[index]; index++)
		;

	if (index >= ARRAY_SIZE(mse->ptp_table)) {
		mse_err("ops is not registered\n");
		spin_unlock_irqrestore(&mse->lock_ptp_table, flags);

		return -EBUSY;
	}

	/* register table */
	mse->ptp_table[index] = ops;
	mse_debug("registered index=%d\n", index);
	spin_unlock_irqrestore(&mse->lock_ptp_table, flags);

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

	spin_lock_irqsave(&mse->lock_instance_table, flags);
	for (i = 0; i < ARRAY_SIZE(mse->instance_table); i++) {
		if (mse->instance_table[i] &&
		    mse->instance_table[i]->ptp_index == index) {
			spin_unlock_irqrestore(&mse->lock_instance_table,
					       flags);
			mse_err("module is in use. instance=%d\n", i);

			return -EPERM;
		}
	}
	spin_unlock_irqrestore(&mse->lock_instance_table, flags);

	spin_lock_irqsave(&mse->lock_ptp_table, flags);
	mse->ptp_table[index] = NULL;
	spin_unlock_irqrestore(&mse->lock_ptp_table, flags);

	return 0;
}
EXPORT_SYMBOL(mse_unregister_ptp);

/*
 * initialize MSE API
 */
static int mse_probe(void)
{
	int err;

	/* allocate device data */
	mse = kzalloc(sizeof(*mse), GFP_KERNEL);
	if (!mse)
		return -ENOMEM;

	spin_lock_init(&mse->lock_network_table);
	spin_lock_init(&mse->lock_media_table);
	spin_lock_init(&mse->lock_instance_table);
	spin_lock_init(&mse->lock_ptp_table);
	spin_lock_init(&mse->lock_mch_table);

	/* register platform device */
	mse->pdev = platform_device_register_simple("mse", -1, NULL, 0);
	if (IS_ERR(mse->pdev)) {
		err = PTR_ERR(mse->pdev);
		mse->pdev = NULL;
		mse_err("Failed to register platform device. ret=%d\n", err);
		goto error;
	}

	/* W/A for cannot using DMA APIs */
	if (IS_ENABLED(CONFIG_OF)) {
		of_dma_configure(&mse->pdev->dev, NULL, false);
	} else {
		/*
		 * MSE has no dependency to OF, but w/o CONFIG_OF set the
		 * above function does nothing while at least initializing
		 * dma_mask, coherent_dma_mask is mandatory. Limitation to
		 * 32bit is needed, as struct eavb_entryvec relies on 32bit
		 * addresses.
		 */
		err = dma_coerce_mask_and_coherent(&mse->pdev->dev,
						   DMA_BIT_MASK(32));
		if (err) {
			mse_err("Failed to configure DMA masks. ret=%d\n", err);
			goto error;
		}
	}

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
static void mse_remove(void)
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
