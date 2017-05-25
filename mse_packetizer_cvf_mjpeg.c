/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016-2017 Renesas Electronics Corporation

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
#include "jpeg.h"

struct avtp_cvf_mjpeg_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct jpeg_info {
	u8 max_comp;

	size_t eoi_offset;

	struct mjpeg_quant_table qtable[JPEG_QUANT_NUM];
	struct mjpeg_component comp[JPEG_COMP_NUM];
	struct mjpeg_restart_header rheader;

	bool header_f;
	bool sos_f;
	bool dqt_f;
	bool sof_f;
	bool dri_f;
	bool eoi_f;
};

struct cvf_mjpeg_packetizer {
	bool used_f;

	struct jpeg_info jpeg;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;
	int payload_max;

	enum MJPEG_TYPE type;
	int width;
	int height;
	u8 quant;
	size_t jpeg_offset;

	size_t piece_data_len;
	u8 piece_data[ETHFRAMELEN_MAX];

	u8 packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_video_config video_config;
};

struct cvf_mjpeg_packetizer cvf_mjpeg_packetizer_table[MSE_INSTANCE_MAX];

static int mse_packetizer_cvf_mjpeg_open(void)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;
	int index;

	for (index = 0; cvf_mjpeg_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(cvf_mjpeg_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];

	cvf_mjpeg->used_f = true;
	cvf_mjpeg->send_seq_num = 0;
	cvf_mjpeg->old_seq_num = SEQNUM_INIT;
	cvf_mjpeg->seq_num_err = SEQNUM_INIT;

	mse_debug("index=%d\n", index);

	return index;
}

static int mse_packetizer_cvf_mjpeg_release(int index)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];
	memset(cvf_mjpeg, 0, sizeof(*cvf_mjpeg));

	mse_debug("index=%d\n", index);

	return 0;
}

static void mse_packetizer_cvf_mjpeg_flag_init(
					struct cvf_mjpeg_packetizer *cvf_mjpeg)
{
	memset(&cvf_mjpeg->jpeg, 0, sizeof(cvf_mjpeg->jpeg));
	cvf_mjpeg->jpeg_offset = 0;
	cvf_mjpeg->piece_data_len = 0;
}

static int mse_packetizer_cvf_mjpeg_packet_init(int index)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];

	cvf_mjpeg->send_seq_num = 0;
	cvf_mjpeg->old_seq_num = SEQNUM_INIT;
	cvf_mjpeg->seq_num_err = SEQNUM_INIT;

	cvf_mjpeg->quant = MJPEG_QUANT_DYNAMIC;
	cvf_mjpeg->type = MJPEG_TYPE_420;

	mse_packetizer_cvf_mjpeg_flag_init(cvf_mjpeg);

	return 0;
}

static int mse_packetizer_cvf_mjpeg_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];
	cvf_mjpeg->net_config = *config;

	return 0;
}

static int mse_packetizer_cvf_mjpeg_header_build(
					void *dst,
					struct avtp_cvf_mjpeg_param *param)
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

	hlen = AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	len = param->payload_size;

	/* 1722 header update + payload */
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_cvf_mjpeg_template(dst);

	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);

	return hlen + len;
}

static int mse_packetizer_cvf_mjpeg_set_video_config(
					int index,
					struct mse_video_config *config)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;
	struct avtp_cvf_mjpeg_param param;
	struct mse_network_config *net_config;
	int bytes_per_frame;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];
	cvf_mjpeg->video_config = *config;
	net_config = &cvf_mjpeg->net_config;

	bytes_per_frame = cvf_mjpeg->video_config.bytes_per_frame;
	if (bytes_per_frame > AVTP_PAYLOAD_MAX) {
		mse_err("bytes_per_frame too big %d\n", bytes_per_frame);
		return -EINVAL;
	}

	if (bytes_per_frame == 0 || bytes_per_frame >= JPEG_PAYLOAD_MAX) {
		cvf_mjpeg->payload_max = AVTP_PAYLOAD_MAX;
	} else {
		cvf_mjpeg->payload_max = bytes_per_frame +
					 AVTP_JPEG_HEADER_SIZE;
	}

	memcpy(param.dest_addr, net_config->dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, net_config->source_addr, MSE_MAC_LEN_MAX);

	param.uniqueid = net_config->uniqueid;
	param.priority = net_config->priority;
	param.vid = net_config->vlanid;
	param.payload_size = 0;

	mse_packetizer_cvf_mjpeg_header_build(cvf_mjpeg->packet_template,
					      &param);

	return 0;
}

static int mse_packetizer_cvf_mjpeg_calc_cbs(int index,
					     struct mse_cbsparam *cbs)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];

	return mse_packetizer_calc_cbs_by_bitrate(
			cvf_mjpeg->net_config.port_transmit_rate,
			cvf_mjpeg->payload_max + AVTP_PAYLOAD_OFFSET,
			cvf_mjpeg->video_config.bitrate,
			cvf_mjpeg->payload_max,
			cbs);
}

static int parse_jpeg_headers(struct cvf_mjpeg_packetizer *cvf_mjpeg,
			      u8 *buf,
			      size_t data_len)
{
	struct jpeg_info *jpeg = &cvf_mjpeg->jpeg;
	size_t offset = 0;
	u8 mk, qid, qlen;
	int header_len = 0;
	int ret;
	int i;

	while (offset < data_len && !jpeg->sos_f) {
		mk = jpeg_get_marker(buf, data_len, &offset);
		switch (mk) {
		case JPEG_MARKER_KIND_NIL:
			break;

		case JPEG_MARKER_KIND_SOI:
			mse_packetizer_cvf_mjpeg_flag_init(cvf_mjpeg);
			break;

		case JPEG_MARKER_KIND_SOF0:
			ret = jpeg_read_sof(buf,
					    data_len,
					    &offset,
					    &cvf_mjpeg->type,
					    &jpeg->max_comp,
					    jpeg->comp,
					    jpeg->qtable,
					    ARRAY_SIZE(jpeg->qtable),
					    &cvf_mjpeg->height,
					    &cvf_mjpeg->width);
			if (ret) {
				mse_err("invalid SOF0\n");
				return ret;
			}

			jpeg->sof_f = true;
			break;

		case JPEG_MARKER_KIND_EOI:
			jpeg->eoi_offset = offset;
			jpeg->eoi_f = true;
			break;

		case JPEG_MARKER_KIND_SOS:
			header_len = offset + JPEG_GET_HEADER_SIZE(buf, offset);
			jpeg->sos_f = true;
			break;

		case JPEG_MARKER_KIND_DQT:
			ret = jpeg_read_dqt(buf, data_len, &offset,
					    jpeg->qtable);
			if (ret) {
				mse_err("invalid DQT\n");
				return ret;
			}

			jpeg->dqt_f = true;
			break;

		case JPEG_MARKER_KIND_DRI:
			ret = jpeg_read_dri(buf, data_len, &offset,
					    &jpeg->rheader);
			if (ret < 0) {
				mse_err("invalid DRI\n");
				return ret;
			}

			if (&jpeg->rheader.restart_interval) {
				jpeg->dri_f = true;
				cvf_mjpeg->type |= MJPEG_TYPE_RESTART_BIT;
			}
			break;

		default:
			mse_debug("skip marker 0x%X\n", mk);
			offset += JPEG_GET_HEADER_SIZE(buf, offset);
			break;
		}
	}

	/* Found SOS, check JPEG syntax */
	if (!(jpeg->sof_f && jpeg->dqt_f)) {
		mse_err("Not support JPEG data, sof=%d dqt=%d\n",
			jpeg->sof_f, jpeg->dqt_f);
		return -EINVAL;
	}

	/* check Q-Table */
	for (i = 0; i <= jpeg->max_comp; i++) {
		qid = jpeg->comp[i].qt;
		if (qid >= ARRAY_SIZE(jpeg->qtable)) {
			mse_err("Invalid qid=%d\n", qid);
			return -EINVAL;
		}

		qlen = jpeg->qtable[qid].size;
		if (!qlen) {
			mse_err("Invalid qlen=0\n");
			return -EINVAL;
		}

		mse_debug("component %d id=%d len=%d\n", i, qid, qlen);
	}

	jpeg->header_f = true;

	return header_len;
}

static int mse_packetizer_cvf_mjpeg_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;
	struct jpeg_info *jpeg;
	struct mjpeg_quant_table *q;
	struct mjpeg_quant_header qheader;
	size_t qlen;
	size_t offset;
	u8 mk;
	u8 *buf, *payload;
	size_t data_len, end_len;
	size_t payload_size;
	u32 header_len = 0;
	int i;
	bool pic_end = false;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];
	jpeg = &cvf_mjpeg->jpeg;

	mse_debug("index=%d seqnum=%d process=%zu/%zu t=%u\n",
		  index, cvf_mjpeg->send_seq_num, *buffer_processed,
		  buffer_size, *timestamp);

	if (!*buffer_processed)
		jpeg->eoi_f = false;

	buf = (u8 *)(buffer + *buffer_processed);
	data_len = buffer_size - *buffer_processed;

	/* JPEG Header parse */
	if (!jpeg->header_f) {
		offset = parse_jpeg_headers(cvf_mjpeg, buf, data_len);
		if (offset < 0) {
			mse_packetizer_cvf_mjpeg_flag_init(cvf_mjpeg);
			return offset;
		}
		header_len = offset;
	} else {
		offset = *buffer_processed;
	}

	/* Search EOI */
	while (offset < data_len && !jpeg->eoi_f) {
		mk = jpeg_get_marker(buf, data_len, &offset);
		if (mk == JPEG_MARKER_KIND_EOI) {
			jpeg->eoi_offset = offset;
			jpeg->eoi_f = true;
			mse_debug("Found EOI offset=%zu\n", jpeg->eoi_offset);
		}
	}

	payload = packet + AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	payload_size = cvf_mjpeg->payload_max;
	data_len = payload_size - AVTP_JPEG_HEADER_SIZE;

	/* Copy restart header */
	if (jpeg->dri_f) {
		memcpy(payload, &jpeg->rheader, sizeof(jpeg->rheader));
		payload += sizeof(jpeg->rheader);
		data_len -= sizeof(jpeg->rheader);
	}

	/* If first packet, copy Q header */
	if ((!cvf_mjpeg->jpeg_offset) &&
	    (cvf_mjpeg->quant >= MJPEG_QUANT_QTABLE_BIT) &&
	    (jpeg->max_comp)) {
		qlen = 0;
		qheader.precision = 0;

		/* make Q header */
		for (i = 0; i <= jpeg->max_comp; i++) {
			q = &jpeg->qtable[jpeg->comp[i].qt];
			qheader.precision |= (q->precision << i);
			qlen += q->size;
		}
		qheader.length = htons(qlen);
		qlen += sizeof(qheader);

		memcpy(payload, &qheader, sizeof(qheader));
		payload += sizeof(qheader);

		/* copy Q Table data */
		for (i = 0; i <= jpeg->max_comp; i++) {
			q = &jpeg->qtable[jpeg->comp[i].qt];
			memcpy(payload, q->data, q->size);
			payload += q->size;
		}

		/* adjust paylod size */
		if (qlen + data_len > JPEG_PAYLOAD_MAX) {
			data_len = JPEG_PAYLOAD_MAX - qlen;
			if (jpeg->dri_f)
				data_len -= sizeof(jpeg->rheader);
		} else {
			payload_size += qlen;
		}
	}

	/* set avtp header */
	memcpy(packet, cvf_mjpeg->packet_template,
	       AVTP_CVF_MJPEG_PAYLOAD_OFFSET);
	avtp_set_sequence_num(packet, cvf_mjpeg->send_seq_num);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_cvf_mjpeg_tspec(packet, 0);
	avtp_set_cvf_mjpeg_offset(packet, cvf_mjpeg->jpeg_offset);
	avtp_set_cvf_mjpeg_type(packet, cvf_mjpeg->type);
	avtp_set_cvf_mjpeg_q(packet, cvf_mjpeg->quant);
	avtp_set_cvf_mjpeg_width(packet, cvf_mjpeg->width);
	avtp_set_cvf_mjpeg_height(packet, cvf_mjpeg->height);

	/* restore piece data */
	if (cvf_mjpeg->piece_data_len) {
		mse_debug("restore a piece of data len=%zu\n",
			  cvf_mjpeg->piece_data_len);
		memcpy(payload, cvf_mjpeg->piece_data,
		       cvf_mjpeg->piece_data_len);
		payload += cvf_mjpeg->piece_data_len;
		cvf_mjpeg->jpeg_offset += cvf_mjpeg->piece_data_len;
		data_len -= cvf_mjpeg->piece_data_len;
		cvf_mjpeg->piece_data_len = 0;
	}

	buf += header_len;
	*buffer_processed += header_len;

	if (jpeg->eoi_f)
		/* length of EOI marker */
		end_len = jpeg->eoi_offset - *buffer_processed;
	else
		/* length of buffer */
		end_len = buffer_size - *buffer_processed;

	/* adjustment end packet */
	if (data_len >= end_len) {
		payload_size -= data_len - end_len;
		data_len = end_len;
		if (jpeg->eoi_f) {
			/* M bit */
			avtp_set_cvf_m(packet, true);
			pic_end = true;
		} else {
			/* save piece data */
			memcpy(cvf_mjpeg->piece_data, buf, data_len);
			cvf_mjpeg->piece_data_len = data_len;
		}
	}

	/* processed buffer length */
	*buffer_processed += data_len;

	if (cvf_mjpeg->piece_data_len) {
		mse_debug("save a piece of data len=%zu\n",
			  cvf_mjpeg->piece_data_len);
		return MSE_PACKETIZE_STATUS_NOT_ENOUGH;
	}

	avtp_set_stream_data_length(packet, payload_size);

	/* set jpeg data */
	memcpy(payload, buf, data_len);

	/* set packet length */
	*packet_size = AVTP_CVF_MJPEG_PAYLOAD_OFFSET - AVTP_JPEG_HEADER_SIZE +
		avtp_get_stream_data_length(packet);

	cvf_mjpeg->jpeg_offset += data_len;

	if (pic_end) {
		/* jpeg frame data end */
		mse_debug("last frame seq=%d process=%zu/%zu t=%u\n",
			  cvf_mjpeg->send_seq_num, *buffer_processed,
			  buffer_size, *timestamp);

		mse_packetizer_cvf_mjpeg_flag_init(cvf_mjpeg);
	}

	cvf_mjpeg->send_seq_num++;

	/* check buffer end */
	if (*buffer_processed < buffer_size)
		return MSE_PACKETIZE_STATUS_CONTINUE;
	else
		return MSE_PACKETIZE_STATUS_COMPLETE;
}

static int mse_packetizer_cvf_mjpeg_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct cvf_mjpeg_packetizer *cvf_mjpeg;
	struct mjpeg_restart_header *rheader;
	struct mjpeg_quant_header qheader;
	u8 *data, *qt, tspec;
	int seq_num;
	size_t data_len;
	u16 dri = 0;
	u32 offset, width, height;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);

	cvf_mjpeg = &cvf_mjpeg_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_CVF) {
		mse_err("error subtype=%d\n", avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (avtp_get_cvf_format(packet) != AVTP_CVF_FORMAT_RFC) {
		mse_err("error cvf_format=%d\n", avtp_get_cvf_format(packet));
		return -EINVAL;
	}

	if (avtp_get_cvf_format_subtype(packet) != AVTP_CVF_RFC_FORMAT_MJPEG) {
		mse_err("error cvf_format_subtype=%d\n",
			avtp_get_cvf_format_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (cvf_mjpeg->old_seq_num != seq_num &&
	    cvf_mjpeg->old_seq_num != SEQNUM_INIT) {
		if (cvf_mjpeg->seq_num_err == SEQNUM_INIT) {
			mse_err("sequence number discontinuity %d->%d=%d\n",
				cvf_mjpeg->old_seq_num, seq_num,
				(seq_num + 1 + AVTP_SEQUENCE_NUM_MAX -
				 cvf_mjpeg->old_seq_num) %
				(AVTP_SEQUENCE_NUM_MAX + 1));
			cvf_mjpeg->seq_num_err = 1;
		} else {
			cvf_mjpeg->seq_num_err++;
		}
	} else {
		if (cvf_mjpeg->seq_num_err != SEQNUM_INIT) {
			mse_err("sequence number recovery %d count=%d\n",
				seq_num, cvf_mjpeg->seq_num_err);
			cvf_mjpeg->seq_num_err = SEQNUM_INIT;
		}
	}
	cvf_mjpeg->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1)) %
				 (AVTP_SEQUENCE_NUM_MAX + 1);

	data = (u8 *)packet + AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	data_len = avtp_get_stream_data_length(packet) - AVTP_JPEG_HEADER_SIZE;
	*timestamp = avtp_get_timestamp(packet);

	tspec = avtp_get_cvf_mjpeg_tspec(packet);
	offset = avtp_get_cvf_mjpeg_offset(packet);
	cvf_mjpeg->type = avtp_get_cvf_mjpeg_type(packet);
	cvf_mjpeg->quant = avtp_get_cvf_mjpeg_q(packet);
	/* convert from blocks to pixels */
	width = avtp_get_cvf_mjpeg_width(packet) * PIXEL_DIV_NUM;
	height = avtp_get_cvf_mjpeg_height(packet) * PIXEL_DIV_NUM;
	if (!width || !height) {
		mse_err("error widthxheight=%ux%u\n", width, height);
		return -EPERM;
	}

	mse_debug("tspec=%u, offset=%u, type=%u, quant=%u, pixel=%ux%u\n",
		  tspec, offset, cvf_mjpeg->type, cvf_mjpeg->quant,
		  width, height);

	if (cvf_mjpeg->type >= MJPEG_TYPE_RESTART_BIT) {
		rheader = (struct mjpeg_restart_header *)data;
		dri = ntohs(rheader->restart_interval);
		mse_debug("restart interval=%d\n", dri);
		data += sizeof(struct mjpeg_restart_header);
		data_len -= sizeof(struct mjpeg_restart_header);
	}

	if ((cvf_mjpeg->quant & MJPEG_QUANT_QTABLE_BIT) && !offset) {
		memcpy(&qheader, data, sizeof(qheader));
		if (cvf_mjpeg->quant == MJPEG_QUANT_DYNAMIC &&
		    !ntohs(qheader.length))
			return -EPERM;

		data += sizeof(struct mjpeg_quant_header);
		data_len -= sizeof(struct mjpeg_quant_header);

		qt = data;
		data += ntohs(qheader.length);
		data_len -= ntohs(qheader.length);
	} else {
		memset(&qheader, 0, sizeof(qheader));
		qt = NULL;
	}

	/* make header for first data */
	if (!offset) {
		u8 header[1024];
		u32 len;

		memset(header, 0, sizeof(header));
		len = jpeg_make_header(cvf_mjpeg->type,
				       cvf_mjpeg->quant,
				       header,
				       width,
				       height,
				       qt,
				       &qheader,
				       dri);

		if (*buffer_processed + len >= buffer_size) {
			mse_err("buffer overrun header\n");
			return -EPERM;
		}

		memcpy(buffer + *buffer_processed, header, len);
		*buffer_processed += len;
	}

	if (*buffer_processed + data_len >= buffer_size) {
		mse_err("buffer overrun data\n");
		return -EPERM;
	}

	/* data copy */
	memcpy(buffer + *buffer_processed, data, data_len);
	*buffer_processed += data_len;

	mse_debug("data_len=%zu processed=%zu\n", data_len, *buffer_processed);

	/* for debug */
	avtp_set_sequence_num(packet, 0);

	/* TODO buffer over check */
	if (!avtp_get_cvf_m(packet))
		return MSE_PACKETIZE_STATUS_CONTINUE;

	mse_debug("M bit enable seq=%d size=%zu/%zu\n",
		  cvf_mjpeg->old_seq_num - 1, *buffer_processed, buffer_size);

	return MSE_PACKETIZE_STATUS_COMPLETE;
}

struct mse_packetizer_ops mse_packetizer_cvf_mjpeg_ops = {
	.open = mse_packetizer_cvf_mjpeg_open,
	.release = mse_packetizer_cvf_mjpeg_release,
	.init = mse_packetizer_cvf_mjpeg_packet_init,
	.set_network_config = mse_packetizer_cvf_mjpeg_set_network_config,
	.set_video_config = mse_packetizer_cvf_mjpeg_set_video_config,
	.calc_cbs = mse_packetizer_cvf_mjpeg_calc_cbs,
	.packetize = mse_packetizer_cvf_mjpeg_packetize,
	.depacketize = mse_packetizer_cvf_mjpeg_depacketize,
};
