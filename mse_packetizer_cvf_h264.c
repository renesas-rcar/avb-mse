/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2017,2021 Renesas Electronics Corporation

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
#include <linux/kernel.h>
#include <uapi/linux/if_ether.h>

#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "avtp.h"

#define MBIT_ADDR (0x28)
#define MBIT_SET  (0x10)

#define FU_HEADER_LEN     (2)
#define FU_ADDR_INDICATOR (0)
#define FU_ADDR_HEADER    (1)

#define FU_I_F_MASK       (0x80)
#define FU_I_NRI_MASK     (0x60)
#define FU_I_F_NRI_MASK   (FU_I_F_MASK | FU_I_NRI_MASK)
#define FU_H_S_BIT        (0x80)
#define FU_H_E_BIT        (0x40)
#define NALU_TYPE_MASK    (0x1F)

#define START_CODE        (0x00000001)

enum NALU_TYPE {
	NALU_TYPE_UNSPECIFIED0  = 0,
	NALU_TYPE_VCL_NON_IDR   = 1,
	NALU_TYPE_VCL_PART_A    = 2,
	NALU_TYPE_VCL_PART_B    = 3,
	NALU_TYPE_VCL_PART_C    = 4,
	NALU_TYPE_VCL_IDR_PIC   = 5,
	NALU_TYPE_SEI           = 6,
	NALU_TYPE_SPS           = 7,
	NALU_TYPE_PPS           = 8,
	NALU_TYPE_AUD           = 9,
	NALU_TYPE_END_OF_SEQ    = 10,
	NALU_TYPE_END_OF_STREAM = 11,
	NALU_TYPE_FILLER        = 12,
	NALU_TYPE_SPS_EXT       = 13,
	NALU_TYPE_PREFIX_NALU   = 14,
	NALU_TYPE_SUBSET_SPS    = 15,
	NALU_TYPE_RESERVED16    = 16,
	NALU_TYPE_RESERVED17    = 17,
	NALU_TYPE_RESERVED18    = 18,
	NALU_TYPE_AUX           = 19,
	NALU_TYPE_EXT           = 20,
	NALU_TYPE_RESERVED21    = 21,
	NALU_TYPE_RESERVED22    = 22,
	NALU_TYPE_RESERVED23    = 23,
	NALU_TYPE_STAP_A        = 24,
	NALU_TYPE_STAP_B        = 25,
	NALU_TYPE_MTAP16        = 26,
	NALU_TYPE_MTAP24        = 27,
	NALU_TYPE_FU_A          = 28,
	NALU_TYPE_FU_B          = 29,
	NALU_TYPE_UNSPECIFIED30 = 30,
	NALU_TYPE_UNSPECIFIED31 = 31,
};

struct avtp_cvf_h264_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct cvf_h264_packetizer {
	bool used_f;
	bool f_start_code;

	int send_seq_num;
	int header_size;             /* whole header size defined IEEE1722 */
	int additional_header_size;  /* additional .. defined IEEE1722 H.264 */
	int data_len_max;
	int payload_max;

	unsigned char *vcl_start;
	unsigned char *next_nal;
	unsigned char fu_indicator;
	unsigned char fu_header;
	size_t nal_header_offset;
	unsigned char packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_video_config video_config;
	struct mse_packetizer_stats stats;
};

struct cvf_h264_packetizer cvf_h264_packetizer_table[MSE_INSTANCE_MAX];

static int mse_packetizer_cvf_h264_open(void)
{
	struct cvf_h264_packetizer *h264;
	int index;

	for (index = 0; cvf_h264_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(cvf_h264_packetizer_table); index++)
		;
	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	h264 = &cvf_h264_packetizer_table[index];

	h264->used_f = true;
	h264->send_seq_num = 0;
	h264->header_size = AVTP_CVF_H264_PAYLOAD_OFFSET;
	h264->additional_header_size =
		AVTP_CVF_H264_PAYLOAD_OFFSET - AVTP_PAYLOAD_OFFSET;

	mse_packetizer_stats_init(&h264->stats);

	mse_debug("index=%d\n", index);
	return index;
}

static int mse_packetizer_cvf_h264_d13_open(void)
{
	struct cvf_h264_packetizer *h264;
	int index;

	for (index = 0; cvf_h264_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(cvf_h264_packetizer_table); index++)
		;
	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	h264 = &cvf_h264_packetizer_table[index];

	h264->used_f = true;
	h264->send_seq_num = 0;
	h264->header_size = AVTP_CVF_H264_D13_PAYLOAD_OFFSET;
	h264->additional_header_size =
		AVTP_CVF_H264_D13_PAYLOAD_OFFSET - AVTP_PAYLOAD_OFFSET;

	mse_packetizer_stats_init(&h264->stats);

	mse_debug("index=%d\n", index);
	return index;
}

static int mse_packetizer_cvf_h264_release(int index)
{
	struct cvf_h264_packetizer *h264;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	h264 = &cvf_h264_packetizer_table[index];
	mse_debug("index=%d\n", index);

	mse_packetizer_stats_report(&h264->stats);

	memset(h264, 0, sizeof(*h264));
	return 0;
}

static int mse_packetizer_cvf_h264_packet_init(int index)
{
	struct cvf_h264_packetizer *h264;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	h264 = &cvf_h264_packetizer_table[index];

	h264->send_seq_num = 0;

	mse_packetizer_stats_init(&h264->stats);

	return 0;
}

static int mse_packetizer_cvf_h264_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct cvf_h264_packetizer *h264;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	h264 = &cvf_h264_packetizer_table[index];
	h264->net_config = *config;
	return 0;
}

static int mse_packetizer_cvf_h264_header_build(
					void *dst,
					struct avtp_cvf_h264_param *param)
{
	u8 cfi;
	u8 streamid[AVTP_STREAMID_SIZE];

	memset(dst, 0, ETHFRAMELEN_MAX);   /* clear MAC frame buffer */

	/* Ethernet frame header */
	set_ieee8021q_dest(dst, (u8 *)param->dest_addr);
	set_ieee8021q_source(dst, (u8 *)param->source_addr);

	/* IEEE802.1Q Q-tag */
	cfi = 0;

	set_ieee8021q_tpid(dst, ETH_P_8021Q);
	/* pcp:3bit, cfi:1bit, vid:12bit */
	set_ieee8021q_tci(dst,
			  (param->priority << 13) | (cfi << 12) | param->vid);
	set_ieee8021q_ethtype(dst, ETH_P_1722);

	/* 1722 header update + payload */
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_cvf_h264_template(dst);

	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, param->payload_size);

	return 0;
}

static int mse_packetizer_cvf_h264_set_video_config(
					int index,
					struct mse_video_config *config)
{
	struct cvf_h264_packetizer *h264;
	struct avtp_cvf_h264_param param;
	int bytes_per_frame;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	h264 = &cvf_h264_packetizer_table[index];
	h264->video_config = *config;

	switch (config->format) {
	case MSE_VIDEO_FORMAT_H264_BYTE_STREAM:
		h264->f_start_code = true;
		break;
	case MSE_VIDEO_FORMAT_H264_AVC:
		h264->f_start_code = false;
		break;
	default:
		mse_err("unknown format=%08x\n", config->format);
		return -EPERM;
	}

	bytes_per_frame = h264->video_config.bytes_per_frame;
	if (bytes_per_frame > AVTP_PAYLOAD_MAX) {
		mse_err("bytes_per_frame too big %d\n", bytes_per_frame);
		return -EINVAL;
	}

	if (bytes_per_frame == 0) {
		h264->payload_max = AVTP_PAYLOAD_MAX;
	} else {
		h264->payload_max = bytes_per_frame;
	}

	h264->data_len_max = h264->payload_max - FU_HEADER_LEN -
		h264->additional_header_size;

	memcpy(param.dest_addr, h264->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, h264->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = h264->net_config.uniqueid;
	param.priority = h264->net_config.priority;
	param.vid = h264->net_config.vlanid;
	param.payload_size = 0;

	mse_packetizer_cvf_h264_header_build(h264->packet_template, &param);

	return 0;
}

static int mse_packetizer_cvf_h264_calc_cbs(int index,
					    struct mse_cbsparam *cbs)
{
	struct cvf_h264_packetizer *h264;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	h264 = &cvf_h264_packetizer_table[index];

	if (h264->video_config.bitrate)
		return mse_packetizer_calc_cbs_by_bitrate(
				h264->net_config.port_transmit_rate,
				h264->payload_max + AVTP_PAYLOAD_OFFSET,
				h264->video_config.bitrate,
				h264->payload_max,
				cbs);

	return mse_packetizer_calc_cbs_by_frames(
			h264->net_config.port_transmit_rate,
			h264->payload_max + AVTP_PAYLOAD_OFFSET,
			h264->video_config.class_interval_frames *
			h264->video_config.max_interval_frames,
			CBS_ADJUSTMENT_FACTOR,
			cbs);
}

static inline bool is_single_nal(u8 fu_indicator)
{
	u8 nalu_type = fu_indicator & NALU_TYPE_MASK;

	return nalu_type > NALU_TYPE_UNSPECIFIED0 &&
	       nalu_type < NALU_TYPE_STAP_A;
}

static int mse_packetizer_cvf_h264_packetize(int index,
					     void *packet,
					     size_t *packet_size,
					     void *buffer,
					     size_t buffer_size,
					     size_t *buffer_processed,
					     unsigned int *timestamp)
{
	struct cvf_h264_packetizer *h264;
	int data_len;
	u32 data_offset;
	u32 nal_size, nal_header;
	u32 start_code = htonl(START_CODE);
	unsigned char *buf = (unsigned char *)buffer;
	unsigned char *cur_nal;
	unsigned char *payload;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	h264 = &cvf_h264_packetizer_table[index];
	mse_debug("index=%d seqnum=%d process=%zu/%zu t=%u\n",
		  index, h264->send_seq_num, *buffer_processed,
		  buffer_size, *timestamp);

	/* search NAL */
	cur_nal = buf + *buffer_processed;
	if (!h264->next_nal) {            /* nal first  */
		if (h264->f_start_code) {
			for (h264->next_nal = cur_nal + 1;
			     h264->next_nal < buf + buffer_size - sizeof(u32);
			     h264->next_nal++) {
				if (!memcmp(h264->next_nal, &start_code,
					    sizeof(u32)))
					break;
			}
			if (h264->next_nal >= buf + buffer_size - sizeof(u32))
				h264->next_nal = buf + buffer_size;
			cur_nal += sizeof(u32);
			nal_size = h264->next_nal - cur_nal;
		} else {
			memcpy(&nal_header, cur_nal, sizeof(nal_header));
			nal_size = ntohl(nal_header);
			cur_nal += sizeof(u32);
			h264->next_nal = cur_nal + nal_size;
		}
		mse_debug("seqnum=%d process=%zu/%zu t=%u nal=%d\n",
			  h264->send_seq_num, *buffer_processed,
			  buffer_size, *timestamp, nal_size);

		switch (*cur_nal & NALU_TYPE_MASK) {
		case NALU_TYPE_UNSPECIFIED0:
		case NALU_TYPE_UNSPECIFIED30:
		case NALU_TYPE_UNSPECIFIED31:
			/* invalid nal type, continue */
			mse_err("NAL format error\n");
			return MSE_PACKETIZE_STATUS_CONTINUE;

		default:
			break;
		}

		h264->fu_indicator =
				(*cur_nal & FU_I_F_NRI_MASK) | NALU_TYPE_FU_A;
		h264->fu_header = FU_H_S_BIT | (*cur_nal & NALU_TYPE_MASK);

#if defined(CONFIG_MSE_PACKETIZER_CVF_H264_SINGLE_NAL)
		if (h264->next_nal - cur_nal < h264->data_len_max) {
			h264->fu_indicator =
				*cur_nal & (FU_I_F_NRI_MASK | NALU_TYPE_MASK);
		}
#endif

		cur_nal++;
		(*buffer_processed) += sizeof(u32) + 1;
	} else {
		h264->fu_header &= ~FU_H_S_BIT;  /* remove start */
	}

	data_len = h264->next_nal - cur_nal;
	if (data_len > h264->data_len_max) {
		data_len = h264->data_len_max;
	} else if (is_single_nal(h264->fu_indicator) ||
		   !(h264->fu_header & FU_H_S_BIT)) {
		h264->fu_header |= FU_H_E_BIT; /* end bit*/
		h264->next_nal = NULL;
	}

	/* header */
	memcpy(packet, h264->packet_template, h264->header_size);

	/* variable header */
	data_offset = is_single_nal(h264->fu_indicator) ? 1 : FU_HEADER_LEN;
	avtp_set_sequence_num(packet, h264->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_stream_data_length(packet,
				    data_len + data_offset +
				    h264->additional_header_size);

	payload = packet + h264->header_size;
	payload[FU_ADDR_INDICATOR] = h264->fu_indicator;
	if (data_offset == FU_HEADER_LEN)
		payload[FU_ADDR_HEADER] = h264->fu_header;

	memcpy(payload + data_offset, cur_nal, data_len);
	*packet_size = h264->header_size + data_offset + data_len;
	(*buffer_processed) += data_len;

	if (*buffer_processed >= buffer_size &&
	    h264->fu_header & FU_H_E_BIT) {
		/* set M bit */
		((unsigned char *)packet)[MBIT_ADDR] |= MBIT_SET;

		return MSE_PACKETIZE_STATUS_COMPLETE;
	}

	/* remove M bit */
	((unsigned char *)packet)[MBIT_ADDR] &= ~MBIT_SET;

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

static void set_nal_header(struct cvf_h264_packetizer *h264,
			   unsigned char *buf,
			   size_t data_len)
{
	u32 nal_header;

	if (h264->f_start_code)
		nal_header = htonl(START_CODE);
	else
		nal_header = htonl(data_len - h264->nal_header_offset -
				   sizeof(u32));

	memcpy(buf + h264->nal_header_offset, &nal_header, sizeof(u32));
}

static bool check_pic_end(struct cvf_h264_packetizer *h264,
			  unsigned char *buf,
			  u8 nalu_type)
{
	switch (nalu_type) {
	case NALU_TYPE_VCL_NON_IDR:
	case NALU_TYPE_VCL_PART_A:
	case NALU_TYPE_VCL_PART_B:
	case NALU_TYPE_VCL_PART_C:
	case NALU_TYPE_VCL_IDR_PIC:
		h264->vcl_start = buf + h264->nal_header_offset;
		break;

	case NALU_TYPE_AUD:
		if (h264->vcl_start)
			return true;
		break;

	default:
		break;
	}

	return false;
}

static int mse_packetizer_cvf_h264_depacketize(int index,
					       void *buffer,
					       size_t buffer_size,
					       size_t *buffer_processed,
					       unsigned int *timestamp,
					       void *packet,
					       size_t packet_size)
{
	struct cvf_h264_packetizer *h264;
	u32 data_offset;
	size_t data_len = *buffer_processed;
	u8 nalu_nri;
	u8 nalu_type;
	int payload_size;
	int fu_size;
	unsigned char *buf = (unsigned char *)buffer;
	unsigned char *payload;
	unsigned char fu_indicator, fu_header;
	bool pic_end = false;

	if (index >= ARRAY_SIZE(cvf_h264_packetizer_table))
		return -EPERM;

	h264 = &cvf_h264_packetizer_table[index];
	mse_debug("index=%d\n", index);
	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_CVF) {
		mse_err("error subtype=%d\n", avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (avtp_get_cvf_format(packet) != AVTP_CVF_FORMAT_RFC) {
		mse_err("error cvf_format=%d\n", avtp_get_cvf_format(packet));
		return -EINVAL;
	}

	if (avtp_get_cvf_format_subtype(packet) != AVTP_CVF_RFC_FORMAT_H264) {
		mse_err("error cvf_format_subtype=%d\n",
			avtp_get_cvf_format_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	mse_packetizer_stats_seqnum(&h264->stats,
				    avtp_get_sequence_num(packet));

	payload_size = avtp_get_stream_data_length(packet) -
		h264->additional_header_size;
	payload = (unsigned char *)packet + h264->header_size;

	fu_indicator = payload[FU_ADDR_INDICATOR];
	nalu_nri = fu_indicator & FU_I_F_NRI_MASK;
	if ((fu_indicator & NALU_TYPE_MASK) == NALU_TYPE_FU_A) {
		data_offset = FU_HEADER_LEN;
		fu_size = payload_size - data_offset;
		fu_header = payload[FU_ADDR_HEADER];
		nalu_type = fu_header & NALU_TYPE_MASK;
		if (fu_header & FU_H_S_BIT) { /* start */
			mse_debug("start size=%d\n", payload_size);

			h264->nal_header_offset = data_len;

			/* Increase data_len by 4 bytes for nal_header */
			data_len += sizeof(u32);

			*(buf + data_len) = nalu_nri | nalu_type;
			data_len += sizeof(u8);
		}
		if (fu_size > 0) {
			if (data_len + fu_size >= buffer_size) {
				mse_err("buffer overrun %zu/%zu\n",
					data_len, buffer_size);

				set_nal_header(h264, buf, data_len);
				(*buffer_processed) = data_len;
				h264->vcl_start = NULL;

				return MSE_PACKETIZE_STATUS_COMPLETE;
			}

			memcpy(buf + data_len, payload + data_offset, fu_size);
			data_len += fu_size;
		}
		if (fu_header & FU_H_E_BIT) { /* end */
			set_nal_header(h264, buf, data_len);
			pic_end = check_pic_end(h264, buf, nalu_type);
		}
	} else if (is_single_nal(fu_indicator)) {
		mse_debug("single nal %02x\n", fu_indicator);

		data_offset = sizeof(fu_indicator);
		fu_size = payload_size - data_offset;
		nalu_type = fu_indicator & NALU_TYPE_MASK;
		h264->nal_header_offset = data_len;

		/* Increase data_len by 4 bytes for nal_header */
		data_len += sizeof(u32);

		*(buf + data_len) = nalu_nri | nalu_type;
		data_len += sizeof(u8);
		memcpy(buf + data_len, payload + data_offset, fu_size);
		data_len += fu_size;

		pic_end = check_pic_end(h264, buf, nalu_type);
		set_nal_header(h264, buf, data_len);
	} else {
		mse_err("unkonwon nal unit = %02x\n",
			fu_indicator & NALU_TYPE_MASK);
		return -EPERM;
	}

	*buffer_processed = data_len;
	*timestamp = avtp_get_timestamp(packet);

	avtp_set_sequence_num(packet, 0); /* for debug */

	if (((unsigned char *)packet)[MBIT_ADDR] & MBIT_SET) /* M bit */
		pic_end = true;

	if (!pic_end)
		return MSE_PACKETIZE_STATUS_CONTINUE;

	h264->vcl_start = NULL;

	mse_debug("size = %zu\n", *buffer_processed);

	return MSE_PACKETIZE_STATUS_COMPLETE;
}

struct mse_packetizer_ops mse_packetizer_cvf_h264_d13_ops = {
	.open = mse_packetizer_cvf_h264_d13_open,
	.release = mse_packetizer_cvf_h264_release,
	.init = mse_packetizer_cvf_h264_packet_init,
	.set_network_config = mse_packetizer_cvf_h264_set_network_config,
	.set_video_config = mse_packetizer_cvf_h264_set_video_config,
	.calc_cbs = mse_packetizer_cvf_h264_calc_cbs,
	.packetize = mse_packetizer_cvf_h264_packetize,
	.depacketize = mse_packetizer_cvf_h264_depacketize,
};

struct mse_packetizer_ops mse_packetizer_cvf_h264_ops = {
	.open = mse_packetizer_cvf_h264_open,
	.release = mse_packetizer_cvf_h264_release,
	.init = mse_packetizer_cvf_h264_packet_init,
	.set_network_config = mse_packetizer_cvf_h264_set_network_config,
	.set_video_config = mse_packetizer_cvf_h264_set_video_config,
	.calc_cbs = mse_packetizer_cvf_h264_calc_cbs,
	.packetize = mse_packetizer_cvf_h264_packetize,
	.depacketize = mse_packetizer_cvf_h264_depacketize,
};
