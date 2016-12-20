/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016 Renesas Electronics Corporation

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

#ifndef __RAVB_MSE_H__
#define __RAVB_MSE_H__

/** @brief MSE's media adapter max */
#define MSE_ADAPTER_MEDIA_MAX   (10)
/** @brief MSE's network adapter max */
#define MSE_ADAPTER_NETWORK_MAX (10)
/** @brief MSE's instance max */
#define MSE_INSTANCE_MAX        (10)

enum MSE_STREAM_TYPE {
	MSE_STREAM_TYPE_AUDIO,
	MSE_STREAM_TYPE_VIDEO,
	MSE_STREAM_TYPE_MPEG2TS,
	MSE_STREAM_TYPE_MAX,
};

struct mse_info {
	uint8_t              device[32];
	enum MSE_STREAM_TYPE type;
};

#define MSE_CONFIG_DEFAULT_MODULE_NAME    "ravb"
#define MSE_CONFIG_DEFAULT_DEVICE_NAME_TX "avb_tx0"
#define MSE_CONFIG_DEFAULT_DEVICE_NAME_RX "avb_rx0"

struct mse_network_device {
	uint8_t module_name[32];
	uint8_t device_name_tx[32];
	uint8_t device_name_rx[32];
	uint8_t device_name_tx_crf[32];
	uint8_t device_name_rx_crf[32];
};

enum MSE_PACKETIZER {
	MSE_PACKETIZER_AAF_PCM,
	MSE_PACKETIZER_IEC61883_6,
	MSE_PACKETIZER_CVF_H264,
	MSE_PACKETIZER_CVF_H264_D13,
	MSE_PACKETIZER_CVF_MJPEG,
	MSE_PACKETIZER_IEC61883_4,
	MSE_PACKETIZER_MAX,
};

struct mse_packetizer {
	enum MSE_PACKETIZER packetizer;
};

#define MSE_CONFIG_VLAN_MAX     (4094)
#define MSE_CONFIG_PRIORITY_MAX (7)
#define MSE_CONFIG_UNIQUEID_MAX (65535)

struct mse_avtp_tx_param {
	uint8_t  dst_mac[6];
	uint8_t  src_mac[6];
	uint16_t vlan;
	uint16_t priority;
	uint32_t uniqueid;
};

struct mse_avtp_rx_param {
	uint8_t streamid[8];
};

#define MSE_CONFIG_SAMPLE_PER_FRAME_MAX (740)

enum MSE_CRF_TYPE {
	MSE_CRF_TYPE_NOT_USE,
	MSE_CRF_TYPE_RX,
	MSE_CRF_TYPE_TX,
	MSE_CRF_TYPE_MAX,
};

struct mse_media_audio_config {
	uint32_t          samples_per_frame;
	enum MSE_CRF_TYPE crf_type;
};

#define MSE_CONFIG_BYTES_PER_FRAME_MAX (1480)
#define MSE_CONFIG_FPS_DENOMINATOR_MAX (60000)
#define MSE_CONFIG_FPS_NUMERATOR_MAX   (1001)
#define MSE_CONFIG_BITRATE_MIN         (1)

struct mse_media_video_config {
	uint32_t bytes_per_frame;
	uint32_t fps_denominator;
	uint32_t fps_numerator;
	uint32_t bitrate;
};

#define MSE_CONFIG_TSPACKET_PER_FRAME_MIN (1)
#define MSE_CONFIG_TSPACKET_PER_FRAME_MAX (7)
#define MSE_CONFIG_PCR_PID_MAX            (8192)

struct mse_media_mpeg2ts_config {
	uint32_t tspackets_per_frame;
	uint32_t bitrate;
	uint32_t pcr_pid;
};

enum MSE_PTP_TYPE {
	MSE_PTP_TYPE_CURRENT_TIME,
	MSE_PTP_TYPE_CAPTURE,
	MSE_PTP_TYPE_MAX,
};

#define MSE_CONFIG_CAPTURE_CH_MAX   (16)

enum MSE_RECOVERY_CAPTURE_FREQ {
	MSE_RECOVERY_CAPTURE_FREQ_FIXED,
	MSE_RECOVERY_CAPTURE_FREQ_NOT_FIXED,
	MSE_RECOVERY_CAPTURE_FREQ_MAX,
};

struct mse_ptp_config {
	enum MSE_PTP_TYPE              type;
	uint32_t                       deviceid;
	uint32_t                       capture_ch;
	uint32_t                       capture_freq;
	enum MSE_RECOVERY_CAPTURE_FREQ recovery_capture_freq;
};

struct mse_mch_config {
	bool enable;
};

struct mse_delay_time {
	uint32_t max_transit_time_ns;
	uint32_t tx_delay_time_ns;
	uint32_t rx_delay_time_ns;
};

#define MSE_MAGIC               (0x21)

#define MSE_G_INFO              _IOR(MSE_MAGIC, 1, struct mse_info)
#define MSE_S_NETWORK_DEVICE    _IOW(MSE_MAGIC, 2, struct mse_network_device)
#define MSE_G_NETWORK_DEVICE    _IOR(MSE_MAGIC, 3, struct mse_network_device)
#define MSE_S_PACKETIZER        _IOW(MSE_MAGIC, 4, struct mse_packetizer)
#define MSE_G_PACKETIZER        _IOR(MSE_MAGIC, 5, struct mse_packetizer)
#define MSE_S_AVTP_TX_PARAM     _IOW(MSE_MAGIC, 6, struct mse_avtp_tx_param)
#define MSE_G_AVTP_TX_PARAM     _IOR(MSE_MAGIC, 7, struct mse_avtp_tx_param)
#define MSE_S_AVTP_RX_PARAM     _IOW(MSE_MAGIC, 8, struct mse_avtp_rx_param)
#define MSE_G_AVTP_RX_PARAM     _IOR(MSE_MAGIC, 9, struct mse_avtp_rx_param)
#define MSE_S_MEDIA_AUDIO_CONFIG \
			_IOW(MSE_MAGIC, 10, struct mse_media_audio_config)
#define MSE_G_MEDIA_AUDIO_CONFIG \
			_IOR(MSE_MAGIC, 11, struct mse_media_audio_config)
#define MSE_S_MEDIA_VIDEO_CONFIG \
			_IOW(MSE_MAGIC, 12, struct mse_media_video_config)
#define MSE_G_MEDIA_VIDEO_CONFIG \
			_IOR(MSE_MAGIC, 13, struct mse_media_video_config)
#define MSE_S_MEDIA_MPEG2TS_CONFIG \
			_IOW(MSE_MAGIC, 14, struct mse_media_mpeg2ts_config)
#define MSE_G_MEDIA_MPEG2TS_CONFIG \
			_IOR(MSE_MAGIC, 15, struct mse_media_mpeg2ts_config)
#define MSE_S_PTP_CONFIG        _IOW(MSE_MAGIC, 16, struct mse_ptp_config)
#define MSE_G_PTP_CONFIG        _IOR(MSE_MAGIC, 17, struct mse_ptp_config)
#define MSE_S_MCH_CONFIG        _IOW(MSE_MAGIC, 18, struct mse_mch_config)
#define MSE_G_MCH_CONFIG        _IOR(MSE_MAGIC, 19, struct mse_mch_config)
#define MSE_S_AVTP_TX_PARAM_CRF _IOW(MSE_MAGIC, 20, struct mse_avtp_tx_param)
#define MSE_G_AVTP_TX_PARAM_CRF _IOR(MSE_MAGIC, 21, struct mse_avtp_tx_param)
#define MSE_S_AVTP_RX_PARAM_CRF _IOW(MSE_MAGIC, 22, struct mse_avtp_rx_param)
#define MSE_G_AVTP_RX_PARAM_CRF _IOR(MSE_MAGIC, 23, struct mse_avtp_rx_param)
#define MSE_S_DELAY_TIME        _IOW(MSE_MAGIC, 24, struct mse_delay_time)
#define MSE_G_DELAY_TIME        _IOR(MSE_MAGIC, 25, struct mse_delay_time)

#endif /* __RAVB_MSE_H__ */
