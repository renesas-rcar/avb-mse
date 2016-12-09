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

#ifndef __RAVB_MSE_KERNEL_H__
#define __RAVB_MSE_KERNEL_H__
#ifdef __KERNEL__

#include "ravb_eavb.h"

/**
 * @brief Resistered adapter name length
 */
#define MSE_NAME_LEN_MAX	(64)
/**
 * @brief MAC address length
 */
#define MSE_MAC_LEN_MAX		(6)

/**
 * @brief Unresistered device index
 */
#define MSE_INDEX_UNDEFINED	(-1)

/**
 * @brief Get kind of MSE
 */
#define MSE_TYPE_KIND_GET(type) ((type) & 0xFFFF0000)
#define IS_MSE_TYPE_KIND_AUDIO(type) \
	(((type) & 0xFFFF0000) == MSE_TYPE_ADAPTER_AUDIO)
#define IS_MSE_TYPE_KIND_VIDEO(type) \
	(((type) & 0xFFFF0000) == MSE_TYPE_ADAPTER_VIDEO)
#define IS_MSE_TYPE_KIND_NETWORK(type) \
	(((type) & 0xFFFF0000) == MSE_TYPE_ADAPTER_NETWORK)
#define IS_MSE_TYPE_KIND_PACKETIZER(type) \
	(((type) & 0xFFFF0000) == MSE_TYPE_PACKETIZER)

/**
 * @brief type of MSE
 */
enum MSE_TYPE {
	/** @brief Audio Adapter */
	MSE_TYPE_ADAPTER_AUDIO = 0x00000000,
	/** @brief Adapter type for PCM */
	MSE_TYPE_ADAPTER_AUDIO_PCM,
	/** @brief Video Adapter */
	MSE_TYPE_ADAPTER_VIDEO = 0x00010000,
	/** @brief Adapter type for H.264 */
	MSE_TYPE_ADAPTER_VIDEO_H264,
	/** @brief Network Adapter */
	MSE_TYPE_ADAPTER_NETWORK = 0x000A0000,
	/** @brief Special type for EAVB Adapter */
	MSE_TYPE_ADAPTER_NETWORK_EAVB,
	/** @brief Packetizer */
	MSE_TYPE_PACKETIZER = 0x000B0000,
	/** @brief Packetizer for Audio PCM */
	MSE_TYPE_PACKETIZER_AUDIO_PCM,
	/** @brief Packetizer for Video H.264 */
	MSE_TYPE_PACKETIZER_VIDEO_H264,
	/** @brief Packetizer for Video MJPEG */
	MSE_TYPE_PACKETIZER_VIDEO_MJPEG,
	/** @brief Packetizer for System MPEG2TS */
	MSE_TYPE_PACKETIZER_SYSTEM_MPEG2TS,
	/** @brief Packetizer for control protocol CRF  */
	MSE_TYPE_PACKETIZER_CTRL_CRF,
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
	/* if need, add more parameters */
};

enum MSE_AUDIO_BIT {
	MSE_AUDIO_BIT_INVALID,
	MSE_AUDIO_BIT_16,
	MSE_AUDIO_BIT_18,
	MSE_AUDIO_BIT_20,
	MSE_AUDIO_BIT_24,
	MSE_AUDIO_BIT_32
};

static inline int mse_get_bit_depth(enum MSE_AUDIO_BIT bit_depth)
{
	switch (bit_depth) {
	case MSE_AUDIO_BIT_16:
		return 16;
	case MSE_AUDIO_BIT_18:
		return 18;
	case MSE_AUDIO_BIT_20:
		return 20;
	case MSE_AUDIO_BIT_24:
		return 24;
	case MSE_AUDIO_BIT_32:
		return 32;
	default:
		return 0;
	}
}

/**
 * @brief aduio stream configuration
 */
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
	/** @brief bytes per frame */
	int bytes_per_frame;
	/* if need, add more parameters */
};

/**
 * @brief aduio stream information
 */
struct mse_audio_info {
	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;
};

enum MSE_MPEG2TS_TYPE {
	/** @brief TS */
	MSE_MPEG2TS_TYPE_TS,
	/** @brief M2TS */
	MSE_MPEG2TS_TYPE_M2TS,
};

enum MSE_VIDEO_FORMAT_TYPE {
	MSE_VIDEO_FORMAT_H264_BYTE_STREAM,
	MSE_VIDEO_FORMAT_H264_AVC,
	MSE_VIDEO_FORMAT_MJPEG
};

/**
 * @brief video stream configuration
 */
struct mse_video_config {
	/** @brief video format */
	enum MSE_VIDEO_FORMAT_TYPE format;
	/** @brief bitrate [Mbps] */
	int bitrate;
	/** @brief framerate */
	struct {
		/** @brief seconds [sec]  */
		int n;
		/** @brief frame number */
		int m;
	} fps;
	/** @brief height of the frame */
	int height;
	/** @brief width of the frame */
	int width;
	/** @brief color_space */
	int color_space;
	/** @brief interlaced */
	int interlaced;
	/** @brief bytes per frame is data size in 1 ether frame */
	int bytes_per_frame;
};

/**
 * @brief video stream configuration
 */
struct mse_mpeg2ts_config {
	/** @brief bitrate [Mbps] */
	int bitrate;
	/** @brief bytes per frame is data size in 1 ether frame */
	int bytes_per_frame;
	/** @brief mpeg2ts type */
	enum MSE_MPEG2TS_TYPE mpeg2ts_type;
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
 * @brief main data for Adapter
 */
struct mse_adapter {
	/** @brief instance used flag */
	bool used_f;
	/** @brief adapter name */
	char name[MSE_NAME_LEN_MAX];
	/** @brief type of Adapter */
	enum MSE_TYPE type;
	/** @brief sysfs index */
	int index_sysfs;
	/** @brief sysfs data */
	struct mse_sysfs_config *sysfs_config;
};

/**
 * @brief registered operations for network adapter
 */
struct mse_adapter_network_ops {
	/** @brief name */
	char *name;
	/** @brief type */
	enum MSE_TYPE type;
	/** @brief private data */
	void *priv;
	/** @brief open function pointer */
	int (*open)(char *name);
	/** @brief release function pointer */
	int (*release)(int index);
	/** @brief set_option function pointer */
	int (*set_option)(int index);
	/** @brief set CBS config function pointer */
	int (*set_cbs_param)(int index, struct eavb_cbsparam *cbs);
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
	/** @brief check function pointer */
	int (*check_receive)(int index);
	/** @brief cancel function pointer */
	int (*cancel)(int index);
	/** @brief get link speed function pointer */
	int (*get_link_speed)(int index);
};

/**
 * @brief packetizer status
 */
enum MSE_PACKETIZE_STATUS  {
	MSE_PACKETIZE_STATUS_CONTINUE,
	MSE_PACKETIZE_STATUS_COMPLETE,
	MSE_PACKETIZE_STATUS_MAY_COMPLETE,
	MSE_PACKETIZE_STATUS_NOT_ENOUGH,
};

/**
 * @brief registered operations for packetizer
 */
struct mse_packetizer_ops {
	/** @brief name */
	char *name;
	/** @brief type */
	enum MSE_TYPE type;
	/** @brief private data */
	void *priv;
	/** @brief open function pointer */
	int (*open)(void);
	/** @brief release function pointer */
	int (*release)(int index);
	/** @brief init function pointer */
	int (*init)(int index);
	/** @brief set network config function pointer */
	int (*set_network_config)(int index,
				  struct mse_network_config *config);
	/** @brief set audio config function pointer */
	int (*set_audio_config)(int index, struct mse_audio_config *config);
	/** @brief set video config function pointer */
	int (*set_video_config)(int index, struct mse_video_config *config);
	/** @brief set mpeg2ts config function pointer */
	int (*set_mpeg2ts_config)(int index,
				  struct mse_mpeg2ts_config *config);
	/** @brief get audio info function pointer */
	int (*get_audio_info)(int index, struct mse_audio_info *info);

	/** @brief calc_cbs function pointer */
	int (*calc_cbs)(int index, struct eavb_cbsparam *cbs);

	/** @brief packetize function pointer */
	int (*packetize)(int index,
			 void *packet,
			 size_t *packet_size,
			 void *buffer,
			 size_t buffer_size,
			 size_t *buffer_processed,
			 unsigned int *timestamp);
	/** @brief depacketize function pointer */
	int (*depacketize)(int index,
			   void *buffer,
			   size_t buffer_size,
			   size_t *buffer_processed,
			   unsigned int *timestamp,
			   void *packet,
			   size_t packet_size);
};

/**
 * @brief registered operations for mch
 */
struct mch_ops {
	int (*open)(int *dev_id);
	int (*close)(int dev_id);
	int (*send_timestamps)(int dev_id,
			       int time_rate_ns,
			       int master_count,
			       unsigned int master_timestamps[],
			       int device_count,
			       unsigned int device_timestamps[]);
	int (*get_recovery_value)(int dev_id,
				  int *value);
};

static inline void mse_make_streamid(u8 *streamid, char *mac, int uid)
{
	streamid[0] = mac[0];
	streamid[1] = mac[1];
	streamid[2] = mac[2];
	streamid[3] = mac[3];
	streamid[4] = mac[4];
	streamid[5] = mac[5];
	streamid[6] = (u8)((uid & 0xff00) >> 8);
	streamid[7] = (u8)((uid & 0x00ff));
}

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
extern int mse_register_adapter_media(enum MSE_TYPE type,
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
extern int mse_unregister_adapter_media(int index_media);

/**
 * @brief register network adapter to MSE
 *
 * @param[in] ops adapter operations
 *
 * @retval 0 MSE instance ID
 * @retval <0 Error
 */
extern int mse_register_adapter_network(struct mse_adapter_network_ops *ops);

/**
 * @brief unregister network adapter from MSE
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_unregister_adapter_network(int index);

/**
 * @brief register packetizer to MSE
 *
 * @param[in] ops packetizer operations
 *
 * @retval 0 MSE adapter ID
 * @retval <0 Error
 */
extern int mse_register_packetizer(struct mse_packetizer_ops *ops);

/**
 * @brief unregister packetizer from MSE
 *
 * @param[in] index MSE adapter ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_unregister_packetizer(int index);

/**
 * @brief get audio configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config audio configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_get_audio_config(int index, struct mse_audio_config *config);

/**
 * @brief set audio configuration
 *
 * @param[in] index MSE instance ID
 * @param[in] config audio configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_set_audio_config(int index, struct mse_audio_config *config);

/**
 * @brief get video configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config video configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_get_video_config(int index, struct mse_video_config *config);

/**
 * @brief set video configuration
 *
 * @param[in] index MSE instance ID
 * @param[in] config video configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_set_video_config(int index, struct mse_video_config *config);

/**
 * @brief get mpeg2ts configuration
 *
 * @param[in] index MSE instance ID
 * @param[out] config mpeg2ts configuration
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_get_mpeg2ts_config(int index,
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
extern int mse_set_mpeg2ts_config(int index,
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
extern int mse_open(int index_media, bool tx);

/**
 * @brief MSE close
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_close(int index);

/**
 * @brief MSE streaming on
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_start_streaming(int index);

/**
 * @brief MSE streaming off
 *
 * @param[in] index MSE instance ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_stop_streaming(int index);

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
extern int mse_start_transmission(int index,
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
extern int mse_register_mch(struct mch_ops *ops);

/**
 * @brief unregister MCH to MSE
 *
 * @param[in] index
 *
 * @retval 0 Success
 * @retval <0 Error
 */
extern int mse_unregister_mch(int index);

#endif /* __KERNEL__ */
#endif /* __RAVB_MSE_KERNEL_H__ */
