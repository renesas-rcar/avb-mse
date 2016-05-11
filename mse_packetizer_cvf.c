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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <uapi/linux/if_ether.h>

#include "mse_core.h"
#include "mse_packetizer.h"
#include "avtp.h"

#define NSEC                    (1000000000L)
#define SEQNUM_INIT             (-1)

#define ETHFRAMELEN_MAX_IPG     (ETHFRAMELEN_MAX + 12)
#define AVTP_PAYLOAD_MAX        (ETHFRAMELEN_MAX - AVTP_PAYLOAD_OFFSET)
#define AVTP_PAYLOAD_MIN        (60 - AVTP_PAYLOAD_OFFSET)

#define TRANSMIT_RATE_BASE      (1000000)
#define PORT_TRANSMIT_RATE      (100000000) /* 100M [bit/sec] */

#define MSE_PACKETIZER_MAX      (10)

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

#define START_CODE        {0x00, 0x00, 0x00, 0x01}

enum NALU_TYPE {
	NALU_TYPE_UNSPECIFIED    = 0,
	NALU_TYPE_VCL_NON_IDR    = 1,
	NALU_TYPE_VCL_PART_A     = 2,
	NALU_TYPE_VCL_PART_B     = 3,
	NALU_TYPE_VCL_PART_C     = 4,
	NALU_TYPE_VCL_IDR_PIC    = 5,
	NALU_TYPE_FU_A           = 28,
	NALU_TYPE_UNSPECIFIED31  = 31,
};

struct avtp_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct cvf_packetizer {
	bool used_f;
	bool is_vcl;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	unsigned char *next_nal;
	unsigned char fu_indicator;
	unsigned char fu_header;
	unsigned char packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_video_config cvf_config;
};

struct cvf_packetizer cvf_packetizer_table[MSE_PACKETIZER_MAX];

static int mse_packetizer_video_cvf_open(void)
{
	struct cvf_packetizer *cvf;
	int index;

	for (index = 0; cvf_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(cvf_packetizer_table); index++)
		;
	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	cvf = &cvf_packetizer_table[index];

	cvf->used_f = true;
	cvf->send_seq_num = 0;
	cvf->old_seq_num = SEQNUM_INIT;
	cvf->seq_num_err = SEQNUM_INIT;

	pr_debug("[%s] index=%d\n", __func__, index);
	return index;
}

static int mse_packetizer_video_cvf_release(int index)
{
	struct cvf_packetizer *cvf;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	cvf = &cvf_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);

	memset(cvf, 0, sizeof(*cvf));
	return 0;
}

static int mse_packetizer_video_cvf_packet_init(int index)
{
	struct cvf_packetizer *cvf;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	cvf = &cvf_packetizer_table[index];

	cvf->send_seq_num = 0;
	cvf->old_seq_num = SEQNUM_INIT;
	cvf->seq_num_err = SEQNUM_INIT;
	return 0;
}

static int mse_packetizer_video_cvf_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct cvf_packetizer *cvf;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	cvf = &cvf_packetizer_table[index];
	cvf->net_config = *config;
	return 0;
}

static int mse_packetizer_video_cvf_header_build(void *dst,
						 struct avtp_param *param)
{
	int hlen, len;
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

	hlen = AVTP_CVF_H264_D13_PAYLOAD_OFFSET;
	len = param->payload_size;

	/* 1722 header update + payload */
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_cvf_h264_d13_template(dst);

	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);

	return hlen + len;
}

static int mse_packetizer_video_cvf_set_video_config(
					int index,
					struct mse_video_config *config)
{
	struct cvf_packetizer *cvf;
	struct avtp_param param;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	cvf = &cvf_packetizer_table[index];
	cvf->cvf_config = *config;

	memcpy(param.dest_addr, cvf->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, cvf->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = cvf->net_config.uniqueid;
	param.priority = cvf->net_config.priority;
	param.vid = cvf->net_config.vlanid;
	param.payload_size = 0;

	mse_packetizer_video_cvf_header_build(cvf->packet_template, &param);
	return 0;
}

static int mse_packetizer_video_cvf_calc_cbs(int index,
					     struct eavb_cbsparam *cbs)
{
	struct cvf_packetizer *cvf;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	cvf = &cvf_packetizer_table[index];

	bandwidth_fraction_denominator =
		(u64)PORT_TRANSMIT_RATE / TRANSMIT_RATE_BASE;
	bandwidth_fraction_numerator = (u64)cvf->cvf_config.bitrate *
						(u64)ETHFRAMELEN_MAX_IPG;
	do_div(bandwidth_fraction_numerator, TRANSMIT_RATE_BASE);
	value = (u64)UINT_MAX * bandwidth_fraction_numerator;
	do_div(value, bandwidth_fraction_denominator);
	do_div(value, AVTP_PAYLOAD_MAX);
	if (value > UINT_MAX) {
		pr_err("[%s] cbs error value=0x%016llx\n", __func__, value);
		return -EPERM;
	}
	cbs->bandwidthFraction = (u32)value;

	value = (u64)USHRT_MAX * bandwidth_fraction_numerator;
	do_div(value, bandwidth_fraction_denominator);
	do_div(value, AVTP_PAYLOAD_MAX);
	cbs->sendSlope = (u32)value;

	value = (u64)USHRT_MAX * (bandwidth_fraction_denominator *
			(u64)AVTP_PAYLOAD_MAX - bandwidth_fraction_numerator);
	do_div(value, bandwidth_fraction_denominator);
	do_div(value, AVTP_PAYLOAD_MAX);
	cbs->idleSlope = (u32)value;

	return 0;
}

static int mse_packetizer_video_cvf_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct cvf_packetizer *cvf;
	int data_len;
	static u8 start_code[] = START_CODE;
	unsigned char *cur_nal;
	unsigned char *payload;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	cvf = &cvf_packetizer_table[index];
	pr_debug("[%s] index=%d seqnum=%d process=%zu/%zu t=%d\n",
		 __func__, index, cvf->send_seq_num, *buffer_processed,
		 buffer_size, *timestamp);

	/* search NAL */
	cur_nal = buffer + *buffer_processed;
	if (!cvf->next_nal) {
		for (cvf->next_nal = cur_nal + 1;
		     (cvf->next_nal < (unsigned char *)buffer + buffer_size -
		     sizeof(start_code)); cvf->next_nal++) {
			if (!memcmp(cvf->next_nal, start_code,
				    sizeof(start_code)))
				break;
		}
		if (cvf->next_nal >= (unsigned char *)buffer + buffer_size -
		    sizeof(start_code))
			cvf->next_nal = (unsigned char *)buffer + buffer_size;
	}

	/* if first packet */
	if (!memcmp(cur_nal, start_code, sizeof(start_code))) {
		cur_nal += sizeof(start_code);
		switch (*cur_nal & NALU_TYPE_MASK) {
		case NALU_TYPE_UNSPECIFIED:
		case NALU_TYPE_UNSPECIFIED31:
			pr_err("NAL format error\n");
			return 0; /* invalid nal type, continue */
		/* VCL */
		case NALU_TYPE_VCL_NON_IDR:
		case NALU_TYPE_VCL_PART_A:
		case NALU_TYPE_VCL_PART_B:
		case NALU_TYPE_VCL_PART_C:
		case NALU_TYPE_VCL_IDR_PIC:
			cvf->is_vcl = true;
			break;
		/* non VCL */
		default:
			cvf->is_vcl = false;
			break;
		}
		cvf->fu_indicator = (*cur_nal & FU_I_F_NRI_MASK) |
								NALU_TYPE_FU_A;
		cvf->fu_header = FU_H_S_BIT | (*cur_nal & NALU_TYPE_MASK);
		cur_nal++;
		(*buffer_processed) += sizeof(start_code) + 1;
	} else {
		cvf->fu_header &= ~FU_H_S_BIT;  /* remove start */
	}

	data_len = cvf->next_nal - cur_nal;
	if (data_len > AVTP_PAYLOAD_MAX - FU_HEADER_LEN) {
		data_len = AVTP_PAYLOAD_MAX - FU_HEADER_LEN;
		((unsigned char *)packet)[MBIT_ADDR] &= ~MBIT_SET;
	} else {
		cvf->fu_header |= FU_H_E_BIT; /* end bit*/
		cvf->next_nal = NULL;
	}

	/* header */
	memcpy(packet, cvf->packet_template, AVTP_CVF_H264_D13_PAYLOAD_OFFSET);

	/* variable header */
	avtp_set_sequence_num(packet, cvf->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_stream_data_length(packet, data_len + FU_HEADER_LEN);

	if (cvf->is_vcl && (cvf->fu_header & FU_H_E_BIT))
		/* set M bit */
		((unsigned char *)packet)[MBIT_ADDR] |= MBIT_SET;
	else
		/* remove M bit */
		((unsigned char *)packet)[MBIT_ADDR] &= ~MBIT_SET;

	payload = packet + AVTP_PAYLOAD_OFFSET;
	payload[FU_ADDR_INDICATOR] = cvf->fu_indicator;
	payload[FU_ADDR_HEADER] = cvf->fu_header;
	if (data_len > 0)
		memcpy(payload + FU_HEADER_LEN, cur_nal, data_len);

	if (data_len + FU_HEADER_LEN < AVTP_PAYLOAD_MIN)
		*packet_size = AVTP_PAYLOAD_OFFSET + AVTP_PAYLOAD_MIN;
	else
		*packet_size = AVTP_PAYLOAD_OFFSET + data_len + FU_HEADER_LEN;

	(*buffer_processed) += data_len;

	/* TODO buffer over check */
	if (*buffer_processed >= buffer_size)
		return 1; /* end of buffer */
	else
		return 0; /* continue */
}

static int mse_packetizer_video_cvf_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct cvf_packetizer *cvf;
	static u8 start_code[] = START_CODE;
	int seq_num;
	int payload_size;
	unsigned char *payload;
	unsigned char fu_indicator, fu_header;

	if (index >= ARRAY_SIZE(cvf_packetizer_table))
		return -EPERM;

	cvf = &cvf_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);
	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_CVF) {
		pr_err("[%s] error subtype=%d\n",
		       __func__, avtp_get_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (cvf->old_seq_num != seq_num && cvf->old_seq_num != SEQNUM_INIT) {
		if (cvf->seq_num_err == SEQNUM_INIT) {
			pr_err("sequence number discontinuity %d->%d=%d\n",
			       cvf->old_seq_num, seq_num, (seq_num + 1 +
			       AVTP_SEQUENCE_NUM_MAX - cvf->old_seq_num) %
			       (AVTP_SEQUENCE_NUM_MAX + 1));
			cvf->seq_num_err = 1;
		} else {
			cvf->seq_num_err++;
		}
	} else {
		if (cvf->seq_num_err != SEQNUM_INIT) {
			pr_err("sequence number recovery %d count=%d\n",
			       seq_num, cvf->seq_num_err);
			cvf->seq_num_err = SEQNUM_INIT;
		}
	}
	cvf->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1))
						% (AVTP_SEQUENCE_NUM_MAX + 1);

	payload_size = avtp_get_stream_data_length(packet);
	payload = (unsigned char *)packet + AVTP_PAYLOAD_OFFSET;

	fu_indicator = payload[FU_ADDR_INDICATOR];
	fu_header = payload[FU_ADDR_HEADER];

	if (fu_header & FU_H_S_BIT) { /* start */
		pr_debug("[%s] start size=%d\n", __func__, payload_size);
		/* start code & nal */
		memcpy((unsigned char *)buffer + *buffer_processed,
		       start_code, sizeof(start_code));
		(*buffer_processed) += sizeof(start_code);
		*((unsigned char *)buffer + *buffer_processed)
					= (fu_indicator & FU_I_F_NRI_MASK) |
					  (fu_header & NALU_TYPE_MASK);
		(*buffer_processed)++;
	}

	if (payload_size > FU_HEADER_LEN) {
		int fu_size = payload_size - FU_HEADER_LEN;

		if (*buffer_processed + fu_size >= buffer_size) {
			pr_err("[%s] buffer overrun\n", __func__);
			return -EPERM; /* error */
		}
		memcpy((unsigned char *)buffer + *buffer_processed,
		       payload + FU_HEADER_LEN, fu_size);

		(*buffer_processed) += fu_size;
	}

	*timestamp = avtp_get_timestamp(packet);

	/* TODO buffer over check */
	if (!(((unsigned char *)packet)[MBIT_ADDR] & MBIT_SET)) /* M bit */
		return 0; /* continue */

	pr_debug("[%s] M bit enable\n", __func__);

	return 1; /* end of picture frame */
}

struct mse_packetizer_ops mse_packetizer_video_cvf_ops = {
	.name = MSE_PACKETIZER_NAME_STR_CVF_H264_D13,
	.priv = NULL,
	.type = MSE_TYPE_PACKETIZER_VIDEO_H264,
	.open = mse_packetizer_video_cvf_open,
	.release = mse_packetizer_video_cvf_release,
	.init = mse_packetizer_video_cvf_packet_init,
	.set_network_config = mse_packetizer_video_cvf_set_network_config,
	.set_video_config = mse_packetizer_video_cvf_set_video_config,
	.calc_cbs = mse_packetizer_video_cvf_calc_cbs,
	.packetize = mse_packetizer_video_cvf_packetize,
	.depacketize = mse_packetizer_video_cvf_depacketize,
};
