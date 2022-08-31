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

#ifndef __RAVB_MSE_KERNEL_H__
#define __RAVB_MSE_KERNEL_H__
#ifdef __KERNEL__

#include <linux/ptp_clock.h>
#include "ravb_mse.h"
#include "ravb_mch.h"

/**
 * @brief Message function for MSE
 */
#define mse_err(fmt, ...)   pr_err("%s: " fmt, __func__, ## __VA_ARGS__)
#define mse_warn(fmt, ...)  pr_warn("%s: " fmt, __func__, ## __VA_ARGS__)
#define mse_info(fmt, ...)  pr_info("%s: " fmt, __func__, ## __VA_ARGS__)
#define mse_debug(fmt, ...) pr_debug("%s: " fmt, __func__, ## __VA_ARGS__)

/**
 * @brief Compare param function for MSE
 *
 */
#define mse_compare_param_key(val, key) strncmp(val, key, MSE_NAME_LEN_MAX)

/**
 * @brief Copy name function for MSE
 */
#define mse_name_strlcpy(dst, src) strlcpy(dst, src, MSE_NAME_LEN_MAX + 1)

/**
 * @brief Unresistered device index
 */
#define MSE_INDEX_UNDEFINED	(-1)

/**
 * @brief Check Type of MSE
 */
#define IS_MSE_TYPE_AUDIO(type) ((type) == MSE_TYPE_ADAPTER_AUDIO)
#define IS_MSE_TYPE_VIDEO(type) ((type) == MSE_TYPE_ADAPTER_VIDEO)
#define IS_MSE_TYPE_MPEG2TS(type) ((type) == MSE_TYPE_ADAPTER_MPEG2TS)
#define IS_MSE_TYPE_NETWORK(type) ((type) == MSE_TYPE_ADAPTER_NETWORK)

/**
 * @brief type of MSE
 */
enum MSE_TYPE {
	/** @brief Audio Adapter */
	MSE_TYPE_ADAPTER_AUDIO,
	/** @brief Video Adapter */
	MSE_TYPE_ADAPTER_VIDEO,
	/** @brief Adapter type for MPEG-2 TS */
	MSE_TYPE_ADAPTER_MPEG2TS,
	/** @brief Network Adapter */
	MSE_TYPE_ADAPTER_NETWORK,
};

/**
 * @brief network configuration
 */
struct mse_network_config {
	/** @brief destination MAC address */
	char dest_addr[MSE_MAC_LEN_MAX];
	/** @brief source MAC address */
	char source_addr[MSE_MAC_LEN_MAX];
	/** @brief priority */
	int priority;
	/** @brief VLAN ID */
	int vlanid;
	/** @brief unique ID */
	int uniqueid;
	/** @brief AVTP timestamp offset */
	unsigned long ts_offset;
	/** @brief port transmit rate */
	unsigned long port_transmit_rate;
	/** @brief streamid */
	u8 streamid[8];
	/* if need, add more parameters */
};

/**
 * @brief audio stream configuration
 */
enum MSE_AUDIO_BIT {
	MSE_AUDIO_BIT_INVALID,
	MSE_AUDIO_BIT_16,
	MSE_AUDIO_BIT_18,
	MSE_AUDIO_BIT_20,
	MSE_AUDIO_BIT_24,
	MSE_AUDIO_BIT_32
};

struct mse_audio_config {
	/** @brief sampling rate */
	int sample_rate;
	/** @brief channels */
	int channels;
	/** @brief period_size */
	int period_size;
	/** @brief samples per frame */
	int bytes_per_sample;
	/** @brief sample bit depth  */
	enum MSE_AUDIO_BIT sample_bit_depth;
	/** @brief sample endian */
	bool is_big_endian;
	/** @brief samples per frame */
	int samples_per_frame;
	/* if need, add more parameters */
};

/**
 * @brief video stream configuration
 */
enum MSE_VIDEO_FORMAT_TYPE {
	MSE_VIDEO_FORMAT_H264_BYTE_STREAM,
	MSE_VIDEO_FORMAT_H264_AVC,
	MSE_VIDEO_FORMAT_MJPEG
};

struct mse_video_config {
	/** @brief video format */
	enum MSE_VIDEO_FORMAT_TYPE format;
	/** @brief bitrate [bps] */
	int bitrate;
	/** @brief framerate */
	struct {
		/** @brief numerator */
		int numerator;
		/** @brief denominotor  */
		int denominator;
	} fps;
	/** @brief bytes per frame is data size in 1 ether frame */
	int bytes_per_frame;
	/** @brief count of class interval frames */
	int class_interval_frames;
	/** @brief maximum frames per class interval */
	int max_interval_frames;
};

/**
 * @brief mpeg2ts stream configuration
 */
enum MSE_MPEG2TS_TYPE {
	/** @brief TS */
	MSE_MPEG2TS_TYPE_TS,
	/** @brief M2TS */
	MSE_MPEG2TS_TYPE_M2TS,
};

struct mse_mpeg2ts_config {
	/** @brief bitrate [Mbps] */
	int bitrate;
	/** @brief bytes per frame is data size in 1 ether frame */
	int tspackets_per_frame;
	/** @brief pid of pcr */
	int pcr_pid;
	/** @brief mpeg2ts type */
	enum MSE_MPEG2TS_TYPE mpeg2ts_type;
	/** @brief transmit mode */
	enum MSE_TRANSMIT_MODE transmit_mode;
	/** @brief count of class interval frames */
	int class_interval_frames;
	/** @brief maximum frames per class interval */
	int max_interval_frames;
};

/**
 * @brief DMA buffer for Adapter
 */
struct mse_packet {
	/** @brief packet size */
	unsigned int len;
	/** @brief physical address for DMA */
	dma_addr_t paddr;
	/** @brief virtual address for driver */
	void *vaddr;
};

/**
 * @brief CBS parameters
 */
struct mse_cbsparam {
	uint32_t bandwidth_fraction;
	uint32_t idle_slope;
	uint32_t send_slope;
	uint32_t hi_credit;
	uint32_t lo_credit;
};

/**
 * @brief registered operations for network adapter
 */
struct mse_adapter_network_ops {
	/** @brief name */
	char *name;
	/** @brief type */
	enum MSE_TYPE type;
	/** @brief owner info */
	struct module *owner;
	/** @brief open function pointer */
	int (*open)(char *name);
	/** @brief release function pointer */
	int (*release)(int index);
	/** @brief set CBS config function pointer */
	int (*set_cbs_param)(int index, struct mse_cbsparam *cbs);
	/** @brief set Stream ID config function pointer */
	int (*set_streamid)(int index, u8 streamid[8]);
	/** @brief send prepare function pointer */
	int (*send_prepare)(int index,
			    struct mse_packet *packets,
			    int num_packets);
	/** @brief send function pointer */
	int (*send)(int index,
		    struct mse_packet *packets,
		    int num_packets);
	/** @brief receive function pointer */
	int (*receive_prepare)(int index,
			       struct mse_packet *packets,
			       int num_packets);
	/** @brief receive function pointer */
	int (*receive)(int index, int num_packets);
	/** @brief cancel function pointer */
	int (*cancel)(int index);
	/** @brief get link speed function pointer */
	int (*get_link_speed)(int index);
};

/**
 * @brief registered operations for mch
 */
struct mch_ops {
	struct module *owner;
	void *(*open)(void);
	int (*close)(void *mch);
	int (*set_interval)(void *mch, u32 ns);
	int (*send_timestamps)(void *mch,
			       struct mch_timestamp *ts,
			       int count);
	int (*get_recovery_value)(void *mch,
				  int *value);
};

/**
 * @brief registered operations for external ptp
 */
struct mse_ptp_ops {
	/* owner info */
	struct module *owner;

	/* PTP Time API */
	int (*get_time)(u64 *ns);

	/* PTP Capture API */
	void *(*open)(void);
	int (*close)(void *ptp_handle);
	int (*capture_start)(void *ptp_handle, int ch, int max_count);
	int (*capture_stop)(void *ptp_handle);
	int (*get_timestamps)(void *ptp_handle, int req_count, u64 *timestamps);

	/* PTP Timer API */
	void *(*timer_open)(u32 (*handler)(void *), void *priv);
	int (*timer_close)(void *timer_handle);
	int (*timer_start)(void *timer_handle, u32 start);
	int (*timer_cancel)(void *timer_handle);
};

/**
 * @brief register media adapter to MSE
 *
 * @param[in] type type of adapter
 * @param[in] name of adapter
 * @param[in] device_name device name
 *
 * @retval >=0 MSE adapter ID
 * @retval <0 Error
 */
int mse_register_adapter_media(enum MSE_TYPE type,
			       char *name,
			       char *device_name);

/**
 * @brief unregister media adapter from MSE
 *
 * @param[in] index_media MSE adapter ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_unregister_adapter_media(int index_media);

/**
 * @brief register network adapter to MSE
 *
 * @param[in] ops adapter operations
 *
 * @retval 0 MSE instance ID
 * @retval <0 Error
 */
int mse_register_adapter_network(struct mse_adapter_network_ops *ops);

/**
 * @brief unregister network adapter from MSE
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_unregister_adapter_network(int index);

/**
 * @brief get audio configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config audio configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_get_audio_config(int index, struct mse_audio_config *config);

/**
 * @brief set audio configuration
 *
 * @param[in] index MSE instance ID
 * @param[in] config audio configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_set_audio_config(int index, struct mse_audio_config *config);

/**
 * @brief get video configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config video configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_get_video_config(int index, struct mse_video_config *config);

/**
 * @brief set video configuration
 *
 * @param[in] index MSE instance ID
 * @param[in] config video configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_set_video_config(int index, struct mse_video_config *config);

/**
 * @brief get mpeg2ts configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config mpeg2ts configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_get_mpeg2ts_config(int index,
			   struct mse_mpeg2ts_config *config);

/**
 * @brief set mpeg2ts configuration
 *
 * @param[in] index MSE instance ID
 * @param[in] config mpeg2ts configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_set_mpeg2ts_config(int index,
			   struct mse_mpeg2ts_config *config);

/**
 * @brief MSE open
 *
 * @param[in] index_media MSE adapter ID
 * @param[in] type direction of adapter
 *
 * @retval >=0 instance ID of MSE
 * @retval <0 Error
 */
int mse_open(int index_media, bool tx);

/**
 * @brief MSE close
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_close(int index);

/**
 * @brief MSE streaming on
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_start_streaming(int index);

/**
 * @brief MSE streaming off
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_stop_streaming(int index);

/**
 * @brief MSE start transmission
 *
 * @param[in] index MSE instance ID
 * @param[in] buffer send data
 * @param[in] buffer_size buffer size
 * @param[out] priv private data
 * @param[in] mse_completion callback function pointer
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_start_transmission(int index,
			   void *buffer,
			   size_t buffer_size,
			   void *priv,
			   int (*mse_completion)(void *priv, int size));

/**
 * @brief register MCH to MSE
 *
 * @param[in] ops
 *
 * @retval 0 MCH table ID
 * @retval <0 Error
 */
int mse_register_mch(struct mch_ops *ops);

/**
 * @brief unregister MCH to MSE
 *
 * @param[in] index
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_unregister_mch(int index);

/**
 * @brief register external PTP to MSE
 *
 * @param[in] ops
 *
 * @retval 0 PTP table ID
 * @retval <0 Error
 */
int mse_register_ptp(struct mse_ptp_ops *ops);

/**
 * @brief unregister external PTP to MSE
 *
 * @param[in] index
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_unregister_ptp(int index);

#endif /* __KERNEL__ */
#endif /* __RAVB_MSE_KERNEL_H__ */
