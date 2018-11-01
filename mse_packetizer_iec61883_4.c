/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016-2018 Renesas Electronics Corporation

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
#include <linux/math64.h>

#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "avtp.h"

#define AVTP_PAYLOAD_MIN        (AVTP_FRAME_SIZE_MIN - AVTP_IEC61883_4_PAYLOAD_OFFSET)
#define AVTP_SOURCE_PACKET_SIZE (4 + 188) /* timestamp + TSP */
#define MSE_TS_PACKET_SIZE      (188)
#define MSE_TIMESTAMP_SIZE      (4)
#define MSE_M2TS_PACKET_SIZE    (MSE_TIMESTAMP_SIZE + MSE_TS_PACKET_SIZE)
#define M2TS_FREQ               (27000000)    /* 27MHz */
#define M2TS_DIFF(a, b)         (((a) - (b)) & 0x3fffffff)
#define DEFAULT_DIFF_TIMESTAMP  (NSEC_SCALE / DEFAULT_INTERVAL_FRAMES)
#define PACKET_PIECE_SIZE_MAX   (MSE_M2TS_PACKET_SIZE * MSE_CONFIG_TSPACKET_PER_FRAME_MAX)

#define BYTE_TO_BIT(byte)        ((byte) * 8)
#define TIMESTAMP_DIFF_S32(a, b) ((s32)((u32)(a) - (u32)(b)))

struct avtp_iec61883_4_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct iec61883_4_packetizer {
	bool used_f;
	bool start_f;

	int send_seq_num;
	int payload_max;
	int packet_size;

	u8 dbc;

	u32 prev_timestamp;
	u32 curr_timestamp;
	u32 diff_timestamp;
	u32 base_timestamp;
	u32 base_timestamp_m2ts;
	u32 piece_data_len;

	unsigned char packet_template[ETHFRAMELEN_MAX];
	unsigned char packet_piece[PACKET_PIECE_SIZE_MAX];

	struct mse_network_config net_config;
	struct mse_mpeg2ts_config mpeg2ts_config;
	struct mse_packetizer_stats stats;
};

struct iec61883_4_packetizer iec61883_4_packetizer_table[MSE_INSTANCE_MAX];

static int mse_packetizer_iec61883_4_open(void)
{
	struct iec61883_4_packetizer *iec61883_4;
	int index;

	for (index = 0; iec61883_4_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(iec61883_4_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	iec61883_4 = &iec61883_4_packetizer_table[index];

	iec61883_4->used_f = true;
	iec61883_4->send_seq_num = 0;
	iec61883_4->dbc = 0;
	iec61883_4->piece_data_len = 0;

	mse_packetizer_stats_init(&iec61883_4->stats);

	mse_debug("index=%d\n", index);
	return index;
}

static int mse_packetizer_iec61883_4_release(int index)
{
	struct iec61883_4_packetizer *iec61883_4;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	iec61883_4 = &iec61883_4_packetizer_table[index];
	mse_debug("index=%d\n", index);

	mse_packetizer_stats_report(&iec61883_4->stats);

	memset(iec61883_4, 0, sizeof(*iec61883_4));

	return 0;
}

static int mse_packetizer_iec61883_4_packet_init(int index)
{
	struct iec61883_4_packetizer *iec61883_4;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_4 = &iec61883_4_packetizer_table[index];

	iec61883_4->start_f = false;
	iec61883_4->send_seq_num = 0;
	iec61883_4->dbc = 0;
	iec61883_4->diff_timestamp = 0;
	iec61883_4->piece_data_len = 0;

	mse_packetizer_stats_init(&iec61883_4->stats);

	return 0;
}

static int mse_packetizer_iec61883_4_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct iec61883_4_packetizer *iec61883_4;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);

	iec61883_4 = &iec61883_4_packetizer_table[index];
	iec61883_4->net_config = *config;

	return 0;
}

static int mse_packetizer_iec61883_4_header_build(
					void *dst,
					struct avtp_iec61883_4_param *param)
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

	hlen = AVTP_IEC61883_4_PAYLOAD_OFFSET;
	len = param->payload_size;

	/* 1722 header update + payload */
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_iec61883_4_template(dst);

	avtp_set_stream_id(dst, streamid);

	return hlen + len;
}

static int mse_packetizer_iec61883_4_set_mpeg2ts_config(
					int index,
					struct mse_mpeg2ts_config *config)
{
	struct iec61883_4_packetizer *iec61883_4;
	struct avtp_iec61883_4_param param;
	struct mse_network_config *net_config;
	int tspackets_per_frame;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_4 = &iec61883_4_packetizer_table[index];
	iec61883_4->mpeg2ts_config = *config;
	net_config = &iec61883_4->net_config;

	tspackets_per_frame = iec61883_4->mpeg2ts_config.tspackets_per_frame;
	iec61883_4->payload_max = tspackets_per_frame * MSE_TS_PACKET_SIZE;
	iec61883_4->packet_size = AVTP_IEC61883_4_PAYLOAD_OFFSET +
		tspackets_per_frame * AVTP_SOURCE_PACKET_SIZE;
	/* Calculate time(ns) per TS packet */
	iec61883_4->diff_timestamp =
		(u32)div64_u64(MSE_TS_PACKET_SIZE * 8 * NSEC_SCALE,
			       iec61883_4->mpeg2ts_config.bitrate);

	memcpy(param.dest_addr, net_config->dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, net_config->source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = net_config->uniqueid;
	param.priority = net_config->priority;
	param.vid = net_config->vlanid;
	param.payload_size = 0;

	mse_packetizer_iec61883_4_header_build(iec61883_4->packet_template,
					       &param);

	return 0;
}

static int mse_packetizer_iec61883_4_calc_cbs(int index,
					      struct mse_cbsparam *cbs)
{
	struct iec61883_4_packetizer *iec61883_4;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_4 = &iec61883_4_packetizer_table[index];

	return mse_packetizer_calc_cbs_by_bitrate(
			iec61883_4->net_config.port_transmit_rate,
			iec61883_4->packet_size,
			iec61883_4->mpeg2ts_config.bitrate,
			iec61883_4->payload_max,
			cbs);
}

static u32 m2ts_timestamp_to_nsec(u32 host_header)
{
	u64 ts_nsec = (u64)(host_header & 0x3fffffff) * NSEC_SCALE;

	return (u32)div64_u64(ts_nsec, M2TS_FREQ);
}

static void copy_payload(struct iec61883_4_packetizer *iec61883_4,
			 unsigned char *data,
			 size_t data_len,
			 int src_packet_size,
			 unsigned char *payload,
			 u32 *timestamp_top)
{
	u32 timestamp_m2ts;
	int payload_num;
	u32 timestamp;
	u32 diff;
	int i;

	payload_num = data_len / src_packet_size;
	for (i = 0; i < payload_num; i++) {
		/* calculate timestamp */
		if (iec61883_4->mpeg2ts_config.transmit_mode == MSE_TRANSMIT_MODE_TIMESTAMP &&
		    src_packet_size == MSE_M2TS_PACKET_SIZE) {
			/* M2TS timestamp */
			timestamp = iec61883_4->base_timestamp;

			timestamp_m2ts = ntohl(*(u32 *)data);

			/* timestamp adjust by m2ts timestamp */
			diff = M2TS_DIFF(timestamp_m2ts, iec61883_4->base_timestamp_m2ts);

			timestamp += m2ts_timestamp_to_nsec(diff);
			iec61883_4->curr_timestamp = timestamp;
		} else {
			/* calculate timestamp by bitrate */
			timestamp = iec61883_4->curr_timestamp;
			iec61883_4->curr_timestamp += iec61883_4->diff_timestamp;
		}

		/* offset adjust by m2ts host_header size */
		if (src_packet_size == MSE_M2TS_PACKET_SIZE)
			data += MSE_TIMESTAMP_SIZE;

		if (i == 0)
			*timestamp_top = timestamp;

		/* copy to source_header_timestamp */
		*(__be32 *)payload = cpu_to_be32(timestamp);

		/* copy to iec61883_4 packet */
		memcpy(payload + sizeof(__be32), data, MSE_TS_PACKET_SIZE);

		/* increment payload and data offset */
		payload += AVTP_SOURCE_PACKET_SIZE;
		data += MSE_TS_PACKET_SIZE;
	}
}

static void update_base_timestamp(struct iec61883_4_packetizer *iec61883_4,
				  u32 timestamp,
				  u8 *data,
				  bool is_ts,
				  unsigned int piece_num)
{
	u32 timestamp_m2ts_piece;
	u32 timestamp_m2ts;
	u32 diff;

	if (!is_ts)
		timestamp_m2ts = ntohl(*(u32 *)data);
	else
		timestamp_m2ts = 0;

	if (!iec61883_4->start_f) {
		iec61883_4->start_f = true;

		/* Init base timestamp */
		iec61883_4->prev_timestamp = timestamp;
		iec61883_4->curr_timestamp = timestamp;
		iec61883_4->base_timestamp = timestamp;
		iec61883_4->base_timestamp_m2ts = timestamp_m2ts;

		return;
	}

	if (timestamp == iec61883_4->prev_timestamp)
		return; /* No change */

	/* Update base timestamp */
	iec61883_4->prev_timestamp = timestamp;

	if (TIMESTAMP_DIFF_S32(timestamp, iec61883_4->curr_timestamp) < 0) {
		mse_debug("Base time[%u] is old, so use current time[%u].\n",
			  timestamp, iec61883_4->curr_timestamp);
		return;
	}

	if (piece_num != 0) {
		/* Back timestamp for piece packet */
		if (iec61883_4->mpeg2ts_config.transmit_mode == MSE_TRANSMIT_MODE_TIMESTAMP &&
		    !is_ts) {
			timestamp_m2ts_piece = ntohl(*(u32 *)iec61883_4->packet_piece);

			/* timestamp adjust by m2ts timestamp */
			diff = M2TS_DIFF(timestamp_m2ts, timestamp_m2ts_piece);
			timestamp -= m2ts_timestamp_to_nsec(diff);

			timestamp_m2ts = timestamp_m2ts_piece;
		} else {
			timestamp -= iec61883_4->diff_timestamp * piece_num;
		}
	}

	iec61883_4->curr_timestamp = timestamp;
	iec61883_4->base_timestamp = timestamp;
	iec61883_4->base_timestamp_m2ts = timestamp_m2ts;
}

static int mse_packetizer_iec61883_4_packetize(int index,
					       void *packet,
					       size_t *packet_size,
					       void *buffer,
					       size_t buffer_size,
					       size_t *buffer_processed,
					       unsigned int *timestamp)
{
	struct iec61883_4_packetizer *iec61883_4;
	int data_len;
	unsigned char *data;
	unsigned char *payload;
	int is_ts;
	int src_packet_size;
	int payload_num;
	u32 avtp_timestamp = 0;
	u32 avtp_timestamp_piece = 0;
	unsigned int piece_num;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	iec61883_4 = &iec61883_4_packetizer_table[index];
	mse_debug("index=%d seqnum=%d process=%zu/%zu t=%d\n",
		  index, iec61883_4->send_seq_num, *buffer_processed,
		  buffer_size, *timestamp);

	/* get mpeg2ts type */
	switch (iec61883_4->mpeg2ts_config.mpeg2ts_type) {
	case MSE_MPEG2TS_TYPE_TS:
		is_ts = 1;
		src_packet_size = MSE_TS_PACKET_SIZE;
		break;
	case MSE_MPEG2TS_TYPE_M2TS:
		is_ts = 0;
		src_packet_size = MSE_M2TS_PACKET_SIZE;
		break;
	default:
		mse_err("error mpeg2ts type=%d\n",
			iec61883_4->mpeg2ts_config.mpeg2ts_type);
		return -EINVAL;
	}

	/* Flush packet piece */
	if (!buffer) {
		/* Packet piece is none */
		*packet_size = 0;

		if (!iec61883_4->piece_data_len)
			return MSE_PACKETIZE_STATUS_NOT_ENOUGH;

		data = iec61883_4->packet_piece;
		data_len = iec61883_4->piece_data_len;
		payload_num = iec61883_4->piece_data_len / src_packet_size;

		iec61883_4->piece_data_len = 0;
		piece_num = 0;
	} else {
		/* data */
		data = buffer + *buffer_processed;
		data_len = buffer_size - *buffer_processed;

		piece_num = iec61883_4->piece_data_len / src_packet_size;
		payload_num = piece_num + (data_len / src_packet_size);

		if (payload_num >= iec61883_4->mpeg2ts_config.tspackets_per_frame) {
			payload_num = iec61883_4->mpeg2ts_config.tspackets_per_frame;
			data_len = (payload_num - piece_num) * src_packet_size;
		} else {
			update_base_timestamp(iec61883_4,
					      *timestamp,
					      data,
					      is_ts,
					      piece_num);

			/* Store packet piece */
			memcpy(iec61883_4->packet_piece + iec61883_4->piece_data_len,
			       data,
			       data_len);
			iec61883_4->piece_data_len += data_len;
			*packet_size = 0;
			*buffer_processed += data_len;

			return MSE_PACKETIZE_STATUS_NOT_ENOUGH;
		}
	}

	update_base_timestamp(iec61883_4,
			      *timestamp,
			      data,
			      is_ts,
			      piece_num);

	/* Copy header template */
	memcpy(packet,
	       iec61883_4->packet_template,
	       AVTP_IEC61883_4_PAYLOAD_OFFSET);

	/* Update dynamic field of header */
	avtp_set_sequence_num(packet, iec61883_4->send_seq_num++);
	avtp_set_stream_data_length(packet,
				    (payload_num * AVTP_SOURCE_PACKET_SIZE) + AVTP_CIP_HEADER_SIZE);
	avtp_set_iec61883_dbc(packet, iec61883_4->dbc);
	iec61883_4->dbc += payload_num;

	/* Set offset address of payload */
	payload = packet + AVTP_IEC61883_4_PAYLOAD_OFFSET;

	/* If packet piece exitst, copy payload from packet piece */
	if (piece_num != 0) {
		copy_payload(iec61883_4,
			     iec61883_4->packet_piece,
			     iec61883_4->piece_data_len,
			     src_packet_size,
			     payload,
			     &avtp_timestamp_piece);

		iec61883_4->piece_data_len = 0;
		payload += piece_num * AVTP_SOURCE_PACKET_SIZE;
	}

	/* Copy payload */
	copy_payload(iec61883_4,
		     data,
		     data_len,
		     src_packet_size,
		     payload,
		     &avtp_timestamp);

	/* If packet piece exits, use avtp_timestamp of packet piece */
	if (piece_num != 0)
		avtp_set_timestamp(packet, avtp_timestamp_piece);
	else
		avtp_set_timestamp(packet, avtp_timestamp);

	/* Set output values */
	*packet_size = AVTP_IEC61883_4_PAYLOAD_OFFSET +
		max(AVTP_SOURCE_PACKET_SIZE * payload_num, AVTP_PAYLOAD_MIN);
	*buffer_processed += data_len;

	mse_debug("bp=%zu/%zu, data_len=%d\n",
		  *buffer_processed, buffer_size, data_len);

	if (*buffer_processed >= buffer_size) {
		iec61883_4->prev_timestamp = *timestamp;

		return MSE_PACKETIZE_STATUS_COMPLETE;
	}

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

static int mse_packetizer_iec61883_4_depacketize(int index,
						 void *buffer,
						 size_t buffer_size,
						 size_t *buffer_processed,
						 unsigned int *timestamp,
						 void *packet,
						 size_t packet_size)
{
	struct iec61883_4_packetizer *iec61883_4;
	int payload_size;
	int offset;
	unsigned char *payload;

	if (index >= ARRAY_SIZE(iec61883_4_packetizer_table))
		return -EPERM;

	iec61883_4 = &iec61883_4_packetizer_table[index];
	mse_debug("index=%d\n", index);

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_61883_IIDC) {
		mse_err("error subtype=%d\n", avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (avtp_get_iec61883_fmt(packet) != AVTP_IEC61883_FMT_MPEG2TS) {
		mse_err("error iec61883_fmt=%d\n",
			avtp_get_iec61883_fmt(packet));
		return -EINVAL;
	}

	/* seq_num check */
	mse_packetizer_stats_seqnum(&iec61883_4->stats,
				    avtp_get_sequence_num(packet));

	payload_size =
		avtp_get_stream_data_length(packet) - AVTP_CIP_HEADER_SIZE;
	payload = (unsigned char *)packet + AVTP_IEC61883_4_PAYLOAD_OFFSET;

	mse_debug("start size=%d\n", payload_size);

	/* check size */
	if (*buffer_processed + payload_size >= buffer_size) {
		mse_err("buffer overrun\n");
		return -EPERM;
	}

	for (offset = sizeof(__be32);
	     offset < payload_size;
	     offset += AVTP_SOURCE_PACKET_SIZE) {
		mse_debug("packet %02x %02x %02x %02x\n",
			  *(payload + offset),
			  *(payload + offset + 1),
			  *(payload + offset + 2),
			  *(payload + offset + 3));
		memcpy((unsigned char *)buffer + *buffer_processed,
		       payload + offset, MSE_TS_PACKET_SIZE);
		*buffer_processed += MSE_TS_PACKET_SIZE;
	}
	*timestamp = avtp_get_timestamp(packet);

	return MSE_PACKETIZE_STATUS_MAY_COMPLETE;
}

struct mse_packetizer_ops mse_packetizer_iec61883_4_ops = {
	.open = mse_packetizer_iec61883_4_open,
	.release = mse_packetizer_iec61883_4_release,
	.init = mse_packetizer_iec61883_4_packet_init,
	.set_network_config = mse_packetizer_iec61883_4_set_network_config,
	.set_mpeg2ts_config = mse_packetizer_iec61883_4_set_mpeg2ts_config,
	.calc_cbs = mse_packetizer_iec61883_4_calc_cbs,
	.packetize = mse_packetizer_iec61883_4_packetize,
	.depacketize = mse_packetizer_iec61883_4_depacketize,
};
