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

#define NSEC                    (1000000000L)
#define SEQNUM_INIT             (-1)

#define ETHFRAMELEN_MAX_IPG     (ETHFRAMELEN_MAX + 12)
#define AVTP_PAYLOAD_MAX        (ETHFRAMELEN_MAX - AVTP_PAYLOAD_OFFSET)

#define TRANSMIT_RATE_BASE      (1000000)

#define MSE_PACKETIZER_MAX      (10)

struct avtp_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct cvf_mjpeg_packetizer {
	bool used_f;
	bool header_f;
	bool sos_f;
	bool dqt_f;
	bool sof_f;
	bool dri_f;
	bool eoi_f;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	enum MJPEG_TYPE type;
	u8 quant;
	u8 max_comp;

	size_t eoi_offset;
	size_t jpeg_offset;

	u8 packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_video_config mjpeg_config;
};

struct cvf_mjpeg_packetizer cvf_mjpeg_packetizer_table[MSE_PACKETIZER_MAX];

static int mse_packetizer_cvf_mjpeg_open(void)
{
	struct cvf_mjpeg_packetizer *mjpg;
	int index;

	for (index = 0; cvf_mjpeg_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(cvf_mjpeg_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mjpg = &cvf_mjpeg_packetizer_table[index];

	mjpg->used_f = true;
	mjpg->send_seq_num = 0;
	mjpg->old_seq_num = SEQNUM_INIT;
	mjpg->seq_num_err = SEQNUM_INIT;

	pr_debug("[%s] index=%d\n", __func__, index);

	return index;
}

static int mse_packetizer_cvf_mjpeg_release(int index)
{
	struct cvf_mjpeg_packetizer *mjpg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mjpg = &cvf_mjpeg_packetizer_table[index];
	memset(mjpg, 0, sizeof(*mjpg));

	pr_debug("[%s] index=%d\n", __func__, index);

	return 0;
}

static void mse_packetizer_cvf_mjpeg_flag_init(
					struct cvf_mjpeg_packetizer *mjpg)
{
	mjpg->header_f = true;
	mjpg->sos_f = false;
	mjpg->dqt_f = false;
	mjpg->sof_f = false;
	mjpg->dri_f = false;
	mjpg->eoi_f = false;
	mjpg->max_comp = 0;
	mjpg->eoi_offset = 0;
	mjpg->jpeg_offset = 0;
}

static int mse_packetizer_cvf_mjpeg_packet_init(int index)
{
	struct cvf_mjpeg_packetizer *mjpg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	mjpg = &cvf_mjpeg_packetizer_table[index];

	mjpg->send_seq_num = 0;
	mjpg->old_seq_num = SEQNUM_INIT;
	mjpg->seq_num_err = SEQNUM_INIT;

	mjpg->quant = MJPEG_QUANT_DYNAMIC;
	mjpg->type = MJPEG_TYPE_420;

	mse_packetizer_cvf_mjpeg_flag_init(mjpg);

	return 0;
}

static int mse_packetizer_cvf_mjpeg_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct cvf_mjpeg_packetizer *mjpg;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	mjpg = &cvf_mjpeg_packetizer_table[index];
	mjpg->net_config = *config;

	return 0;
}

static int mse_packetizer_cvf_mjpeg_header_build(void *dst,
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

	hlen = AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	len = param->payload_size;

	/* 1722 header update + payload */
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_cvf_mjpeg_template(dst);

	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);

	return hlen + len;
}

static int mse_packetizer_cvf_mjpeg_set_video_config(
					int index,
					struct mse_video_config *config)
{
	struct cvf_mjpeg_packetizer *mjpg;
	struct avtp_param param;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	mjpg = &cvf_mjpeg_packetizer_table[index];
	mjpg->mjpeg_config = *config;

	memcpy(param.dest_addr, mjpg->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, mjpg->net_config.source_addr,
	       MSE_MAC_LEN_MAX);

	param.uniqueid = mjpg->net_config.uniqueid;
	param.priority = mjpg->net_config.priority;
	param.vid = mjpg->net_config.vlanid;
	param.payload_size = 0;

	mse_packetizer_cvf_mjpeg_header_build(mjpg->packet_template, &param);

	return 0;
}

static int mse_packetizer_cvf_mjpeg_calc_cbs(int index,
					     struct eavb_cbsparam *cbs)
{
	struct cvf_mjpeg_packetizer *mjpg;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	mjpg = &cvf_mjpeg_packetizer_table[index];

	bandwidth_fraction_denominator =
		(u64)mjpg->net_config.port_transmit_rate / TRANSMIT_RATE_BASE;
	bandwidth_fraction_numerator = (u64)mjpg->mjpeg_config.bitrate *
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

static int mse_packetizer_cvf_mjpeg_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct cvf_mjpeg_packetizer *mjpg;
	struct mse_video_config *cnf;
	struct mjpeg_restart_header rheader;
	struct mjpeg_quant_header qheader;
	struct mjpeg_quant_table qtable[JPEG_QUANT_NUM];
	struct mjpeg_component comp[JPEG_COMP_NUM];
	u8 *buf, *payload;
	size_t offset = 0, data_len, end_len = 0;
	size_t payload_size;
	u32 header_len = 0, quant_len = 0;
	int i, ret;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	mjpg = &cvf_mjpeg_packetizer_table[index];

	pr_debug("[%s] index=%d seqnum=%d process=%zu/%zu t=%d\n",
		 __func__, index, mjpg->send_seq_num, *buffer_processed,
		 buffer_size, *timestamp);

	cnf = &mjpg->mjpeg_config;
	buf = (u8 *)(buffer + *buffer_processed);
	data_len = buffer_size - *buffer_processed;
	memset(qtable, 0, sizeof(qtable));
	memset(comp, 0, sizeof(comp));

	if (*buffer_processed == 0)
		mjpg->header_f = true;

	while (offset < data_len && mjpg->header_f &&
	       !mjpg->eoi_f && !mjpg->sos_f) {
		u8 mk;

		mk = jpeg_get_marker(buf, data_len, &offset);
		switch (mk) {
		case JPEG_MARKER_KIND_NIL:
		case JPEG_MARKER_KIND_SOI:
			break;

		case JPEG_MARKER_KIND_SOF0:
			ret = jpeg_read_sof(buf,
					    data_len,
					    &offset,
					    &mjpg->type,
					    &mjpg->max_comp,
					    comp,
					    qtable,
					    ARRAY_SIZE(qtable),
					    &cnf->height,
					    &cnf->width);
			if (ret) {
				pr_err("[%s] invalid SOF0\n", __func__);
				goto header_error;
			}
			mjpg->sof_f = true;
			break;

		case JPEG_MARKER_KIND_EOI:
			mjpg->eoi_offset = offset;
			mjpg->eoi_f = true;
			break;

		case JPEG_MARKER_KIND_SOS:
			header_len = offset +
				     JPEG_GET_HEADER_SIZE(buf, offset);
			mjpg->sos_f = true;
			break;

		case JPEG_MARKER_KIND_DQT:
			ret = jpeg_read_dqt(buf, data_len, &offset, qtable);
			if (ret) {
				pr_err("[%s] invalid DQT\n", __func__);
				goto header_error;
			}
			mjpg->dqt_f = true;
			break;

		case JPEG_MARKER_KIND_DRI:
			ret = jpeg_read_dri(buf, data_len, &offset, &rheader);
			if (ret < 0) {
				pr_err("[%s] invalid DRI\n", __func__);
				goto header_error;
			} else if (ret) {
				mjpg->dri_f = true;
				mjpg->type |= MJPEG_TYPE_RESTART_BIT;
			}
			break;

		default:
			pr_debug("[%s] skip marker 0x%X\n", __func__, mk);
			offset += JPEG_GET_HEADER_SIZE(buf, offset);
			break;
		}
	}

	if (!mjpg->dqt_f || !mjpg->sof_f) {
		pr_err("[%s] Not support JPEG format sof=%d dqt=%d\n",
		       __func__, mjpg->dqt_f, mjpg->sof_f);
		goto header_error;
	}

	/* Search EOI */
	if (!mjpg->eoi_f) {
		mjpg->eoi_f = true;
		mjpg->eoi_offset = data_len;
		for (i = offset; i < data_len; i++) {
			if (buf[i] == 0xFF &&
			    buf[i + 1] == JPEG_MARKER_KIND_EOI) {
				mjpg->eoi_offset = i + 2;
			}
		}
	}

	payload_size = ETH_FRAME_LEN - AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	data_len = payload_size;

	if (mjpg->quant >= MJPEG_QUANT_QTABLE_BIT && !mjpg->jpeg_offset) {
		memset(&qheader, 0, sizeof(qheader));

		for (i = 0; i <= mjpg->max_comp; i++) {
			u8 qlen, qid;

			qid = comp[i].qt;
			if (qid >= ARRAY_SIZE(qtable)) {
				pr_err("[%s] Invalid qid=%d\n", __func__, qid);
				goto header_error;
			}

			qlen = qtable[qid].size;
			if (!qlen) {
				pr_err("[%s] Invalid qlen=0\n", __func__);
				goto header_error;
			}

			qheader.precision |= (qlen == JPEG_DQT_QUANT_SIZE8) ?
					     0 : (1 << i);
			quant_len += qlen;
		}
		qheader.length = htons(quant_len);
	}

	/* set header */
	memcpy(packet, mjpg->packet_template, AVTP_CVF_MJPEG_PAYLOAD_OFFSET);
	avtp_set_sequence_num(packet, mjpg->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_cvf_mjpeg_tspec(packet, 0);
	avtp_set_cvf_mjpeg_offset(packet, mjpg->jpeg_offset);
	avtp_set_cvf_mjpeg_type(packet, mjpg->type);
	avtp_set_cvf_mjpeg_q(packet, mjpg->quant);
	avtp_set_cvf_mjpeg_width(packet, cnf->width);
	avtp_set_cvf_mjpeg_height(packet, cnf->height);

	payload = packet + AVTP_CVF_MJPEG_PAYLOAD_OFFSET;

	if (mjpg->dri_f) {
		memcpy(payload, &rheader, sizeof(rheader));
		payload += sizeof(rheader);
		data_len -= sizeof(rheader);
	}

	/* only first packet */
	if (quant_len > 0) {
		memcpy(payload, &qheader, sizeof(qheader));
		payload += sizeof(qheader);
		data_len -= sizeof(qheader);

		for (i = 0; i <= mjpg->max_comp; i++) {
			u8 qlen, qid;

			qid = comp[i].qt;
			qlen = qtable[qid].size;
			memcpy(payload, qtable[qid].data, qlen);

			pr_debug("[%s] component %d id=%d len=%d\n",
				 __func__, i, qid, qlen);

			payload += qlen;
		}
		data_len -= quant_len;
	}

	buf += header_len;
	*buffer_processed += header_len;

	if (mjpg->eoi_f)
		/* length of EOI marker */
		end_len = mjpg->eoi_offset - *buffer_processed;
	else
		/* length of buffer */
		end_len = buffer_size - *buffer_processed;

	/* adjustment end packet */
	if (data_len >= end_len) {
		if (mjpg->eoi_f) {
			pr_debug("[%s] last frame seq=%d\n",
				 __func__, mjpg->send_seq_num - 1);
			/* M bit */
			avtp_set_cvf_m(packet, true);
		}
		avtp_set_stream_data_length(
					packet,
					payload_size - (data_len - end_len));
		data_len = end_len;
	} else {
		avtp_set_stream_data_length(packet, payload_size);
	}

	/* set jpeg data */
	memcpy(payload, buf, data_len);

	/* set packet length */
	*packet_size = AVTP_CVF_MJPEG_PAYLOAD_OFFSET +
		avtp_get_stream_data_length(packet);

	/* read buffer length */
	*buffer_processed += data_len;

	if (mjpg->eoi_f && !(mjpg->eoi_offset - *buffer_processed))
		/* jpeg data end */
		mse_packetizer_cvf_mjpeg_flag_init(mjpg);
	else
		mjpg->jpeg_offset += data_len;

	/* buffer end */
	if (*buffer_processed == buffer_size)
		return MSE_PACKETIZE_STATUS_COMPLETE;
	else
		return MSE_PACKETIZE_STATUS_CONTINUE;

header_error:
	/* find next header */
	mse_packetizer_cvf_mjpeg_flag_init(mjpg);

	return -EPERM;
}

static int mse_packetizer_cvf_mjpeg_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct cvf_mjpeg_packetizer *mjpg;
	struct mjpeg_restart_header *rheader;
	struct mjpeg_quant_header qheader;
	u8 *data, *qt, tspec;
	int seq_num;
	size_t data_len;
	u16 dri = 0;
	u32 offset, width, height;

	if (index >= ARRAY_SIZE(cvf_mjpeg_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	mjpg = &cvf_mjpeg_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_CVF) {
		pr_err("[%s] error subtype=%d\n",
		       __func__, avtp_get_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (mjpg->old_seq_num != seq_num && mjpg->old_seq_num != SEQNUM_INIT) {
		if (mjpg->seq_num_err == SEQNUM_INIT) {
			pr_err("sequence number discontinuity %d->%d=%d\n",
			       mjpg->old_seq_num, seq_num, (seq_num + 1 +
			       AVTP_SEQUENCE_NUM_MAX - mjpg->old_seq_num) %
			       (AVTP_SEQUENCE_NUM_MAX + 1));
			mjpg->seq_num_err = 1;
		} else {
			mjpg->seq_num_err++;
		}
	} else {
		if (mjpg->seq_num_err != SEQNUM_INIT) {
			pr_err("sequence number recovery %d count=%d\n",
			       seq_num, mjpg->seq_num_err);
			mjpg->seq_num_err = SEQNUM_INIT;
		}
	}
	mjpg->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1))
						% (AVTP_SEQUENCE_NUM_MAX + 1);

	data = (u8 *)packet + AVTP_CVF_MJPEG_PAYLOAD_OFFSET;
	data_len = avtp_get_stream_data_length(packet);
	*timestamp = avtp_get_timestamp(packet);

	tspec = avtp_get_cvf_mjpeg_tspec(packet);
	offset = avtp_get_cvf_mjpeg_offset(packet);
	mjpg->type = avtp_get_cvf_mjpeg_type(packet);
	mjpg->quant = avtp_get_cvf_mjpeg_q(packet);
	/* convert from blocks to pixels */
	width = avtp_get_cvf_mjpeg_width(packet) * PIXEL_DIV_NUM;
	height = avtp_get_cvf_mjpeg_height(packet) * PIXEL_DIV_NUM;
	if (!width || !height) {
		pr_err("[%s] error widthxheight=%ux%u\n",
		       __func__, width, height);
		return -EPERM;
	}

	pr_debug("[%s] tspec=%u, offset=%u, type=%u, quant=%u, pixel=%ux%u\n",
		 __func__, tspec, offset, mjpg->type, mjpg->quant,
		 width, height);

	if (mjpg->type >= MJPEG_TYPE_RESTART_BIT) {
		rheader = (struct mjpeg_restart_header *)data;
		dri = ntohs(rheader->restart_interval);
		pr_debug("[%s] restart interval=%d\n", __func__, dri);
		data += sizeof(struct mjpeg_restart_header);
		data_len -= sizeof(struct mjpeg_restart_header);
	}

	if ((mjpg->quant & MJPEG_QUANT_QTABLE_BIT) && !offset) {
		memcpy(&qheader, data, sizeof(qheader));
		if (mjpg->quant == MJPEG_QUANT_DYNAMIC &&
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
		len = jpeg_make_header(mjpg->type,
				       mjpg->quant,
				       header,
				       width,
				       height,
				       qt,
				       &qheader,
				       dri);

		if (*buffer_processed + len >= buffer_size) {
			pr_err("[%s] buffer overrun header\n", __func__);
			return -EPERM;
		}

		memcpy(buffer + *buffer_processed, header, len);
		*buffer_processed += len;
	}

	if (*buffer_processed + data_len >= buffer_size) {
		pr_err("[%s] buffer overrun data\n", __func__);
		return -EPERM;
	}

	/* data copy */
	memcpy(buffer + *buffer_processed, data, data_len);
	*buffer_processed += data_len;

	pr_debug("[%s] data_len=%zu processed=%zu\n",
		 __func__, data_len, *buffer_processed);

	/* for debug */
	avtp_set_sequence_num(packet, 0);

	/* TODO buffer over check */
	if (!avtp_get_cvf_m(packet))
		return MSE_PACKETIZE_STATUS_CONTINUE;

	pr_info("[%s] M bit enable seq=%d size=%zu/%zu\n", __func__,
		mjpg->old_seq_num - 1, *buffer_processed, buffer_size);

	return MSE_PACKETIZE_STATUS_COMPLETE;
}

struct mse_packetizer_ops mse_packetizer_video_cvf_mjpeg_ops = {
	.name = MSE_PACKETIZER_NAME_STR_CVF_MJPEG,
	.priv = NULL,
	.open = mse_packetizer_cvf_mjpeg_open,
	.release = mse_packetizer_cvf_mjpeg_release,
	.init = mse_packetizer_cvf_mjpeg_packet_init,
	.set_network_config = mse_packetizer_cvf_mjpeg_set_network_config,
	.set_video_config = mse_packetizer_cvf_mjpeg_set_video_config,
	.calc_cbs = mse_packetizer_cvf_mjpeg_calc_cbs,
	.packetize = mse_packetizer_cvf_mjpeg_packetize,
	.depacketize = mse_packetizer_cvf_mjpeg_depacketize,
};
