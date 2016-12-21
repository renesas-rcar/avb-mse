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
#include <linux/math64.h>

#include "ravb_mse_kernel.h"
#include "mse_core.h"
#include "mse_packetizer.h"
#include "avtp.h"

#define SEQNUM_INIT             (-1)

#define ETH_IPG                 (12)
#define AVTP_PAYLOAD_MIN        (AVTP_FRAME_SIZE_MIN - AVTP_IEC61883_4_PAYLOAD_OFFSET)

#define AVTP_SOURCE_PACKET_SIZE (4 + 188) /* timestamp + TSP */
#define TRANSMIT_RATE_BASE      (1000000)

#define MSE_PACKETIZER_MAX      (10)

#define MSE_TS_PACKET_SIZE      (188)
#define MSE_TIMESTAMP_SIZE      (4)
#define MSE_M2TS_PACKET_SIZE    (MSE_TIMESTAMP_SIZE + MSE_TS_PACKET_SIZE)
#define M2TS_FREQ               (27000000)    /* 27MHz */
#define NSEC                    (1000000000L)
#define DEFAULT_DIFF_TIMESTAMP  (NSEC / 8000)

struct avtp_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int payload_size;
	int uniqueid;
	int priority;
	int vid;
};

struct mpeg2ts_packetizer {
	bool used_f;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	u8 dbc;

	u32 prev_timestamp;
	u32 curr_timestamp;
	u32 diff_timestamp;
	u32 m2ts_start;

	unsigned char packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_mpeg2ts_config mpeg2ts_config;
};

struct mpeg2ts_packetizer mpeg2ts_packetizer_table[MSE_PACKETIZER_MAX];

static int mse_packetizer_video_iec_open(void)
{
	struct mpeg2ts_packetizer *mpeg2ts;
	int index;

	for (index = 0; mpeg2ts_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(mpeg2ts_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	mpeg2ts = &mpeg2ts_packetizer_table[index];

	mpeg2ts->used_f = true;
	mpeg2ts->send_seq_num = 0;
	mpeg2ts->old_seq_num = SEQNUM_INIT;
	mpeg2ts->seq_num_err = SEQNUM_INIT;
	mpeg2ts->dbc = 0;

	pr_debug("[%s] index=%d\n", __func__, index);
	return index;
}

static int mse_packetizer_video_iec_release(int index)
{
	struct mpeg2ts_packetizer *mpeg2ts;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	mpeg2ts = &mpeg2ts_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);

	memset(mpeg2ts, 0, sizeof(*mpeg2ts));

	return 0;
}

static int mse_packetizer_video_iec_packet_init(int index)
{
	struct mpeg2ts_packetizer *mpeg2ts;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	mpeg2ts = &mpeg2ts_packetizer_table[index];

	mpeg2ts->send_seq_num = 0;
	mpeg2ts->old_seq_num = SEQNUM_INIT;
	mpeg2ts->seq_num_err = SEQNUM_INIT;
	mpeg2ts->dbc = 0;

	return 0;
}

static int mse_packetizer_video_iec_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct mpeg2ts_packetizer *mpeg2ts;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	mpeg2ts = &mpeg2ts_packetizer_table[index];
	mpeg2ts->net_config = *config;

	return 0;
}

static int mse_packetizer_video_iec_header_build(void *dst,
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

	hlen = AVTP_IEC61883_4_PAYLOAD_OFFSET;
	len = param->payload_size;

	/* 1722 header update + payload */
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_iec61883_4_template(dst);

	avtp_set_stream_id(dst, streamid);

	return hlen + len;
}

static int mse_packetizer_video_iec_set_mpeg2ts_config(
					int index,
					struct mse_mpeg2ts_config *config)
{
	struct mpeg2ts_packetizer *mpeg2ts;
	struct avtp_param param;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	mpeg2ts = &mpeg2ts_packetizer_table[index];
	mpeg2ts->mpeg2ts_config = *config;

	memcpy(param.dest_addr, mpeg2ts->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, mpeg2ts->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = mpeg2ts->net_config.uniqueid;
	param.priority = mpeg2ts->net_config.priority;
	param.vid = mpeg2ts->net_config.vlanid;
	param.payload_size = 0;

	mse_packetizer_video_iec_header_build(mpeg2ts->packet_template,
					      &param);

	return 0;
}

static int mse_packetizer_video_iec_calc_cbs(int index,
					     struct mse_cbsparam *cbs)
{
	struct mpeg2ts_packetizer *mpeg2ts;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;
	int payload_size, packet_size;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	pr_err("[%s] index=%d\n", __func__, index);
	mpeg2ts = &mpeg2ts_packetizer_table[index];
	payload_size = mpeg2ts->mpeg2ts_config.tspackets_per_frame *
		MSE_TS_PACKET_SIZE;
	packet_size =
		ETH_IPG + AVTP_IEC61883_4_PAYLOAD_OFFSET +
		mpeg2ts->mpeg2ts_config.tspackets_per_frame *
		AVTP_SOURCE_PACKET_SIZE;
	pr_err("[%s] payload_size=%d packet_size=%d\n",
	       __func__, payload_size, packet_size);

	bandwidth_fraction_denominator =
		(u64)mpeg2ts->net_config.port_transmit_rate * (u64)payload_size;

	if (!bandwidth_fraction_denominator) {
		pr_err("[%s] Link speed %lu bps is not support\n",
		       __func__, mpeg2ts->net_config.port_transmit_rate);
		return -EPERM;
	}

	bandwidth_fraction_numerator =
		(u64)mpeg2ts->mpeg2ts_config.bitrate * (u64)packet_size;
	bandwidth_fraction_denominator >>= 8;
	bandwidth_fraction_numerator >>= 8;
	value = (u64)UINT_MAX * bandwidth_fraction_numerator;
	value = div64_u64(value, bandwidth_fraction_denominator);
	if (value > UINT_MAX) {
		pr_err("[%s] cbs error value=0x%016llx\n", __func__, value);
		return -EPERM;
	}

	cbs->bandwidth_fraction = (u32)value;

	cbs->send_slope = value >> 16;
	cbs->idle_slope = USHRT_MAX - cbs->send_slope;

	return 0;
}

static u64 m2ts_timestamp_to_nsec(u32 host_header)
{
	u64 ts_nsec = (host_header & 0x3fffffff) * NSEC;

	return div64_u64(ts_nsec, M2TS_FREQ);
}

static int mse_packetizer_video_iec_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct mpeg2ts_packetizer *mpeg2ts;
	int data_len;
	unsigned char *data;
	unsigned char *payload;
	int is_ts;
	int src_packet_size;
	int payloads;
	int i;
	u64 timestamp_ns;
	unsigned int avtp_timestamp;
	unsigned int num = 0, diff;
	bool is_top_on_buffer;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	mpeg2ts = &mpeg2ts_packetizer_table[index];
	pr_debug("[%s] index=%d seqnum=%d process=%zu/%zu t=%d\n",
		 __func__, index, mpeg2ts->send_seq_num, *buffer_processed,
		 buffer_size, *timestamp);

	/* get mpeg2ts type */
	switch (mpeg2ts->mpeg2ts_config.mpeg2ts_type) {
	case MSE_MPEG2TS_TYPE_TS:
		is_ts = 1;
		src_packet_size = MSE_TS_PACKET_SIZE;
		break;
	case MSE_MPEG2TS_TYPE_M2TS:
		is_ts = 0;
		src_packet_size = MSE_M2TS_PACKET_SIZE;
		break;
	default:
		pr_err("[%s] error mpeg2ts type=%d\n",
		       __func__, mpeg2ts->mpeg2ts_config.mpeg2ts_type);
		return -EINVAL;
	}

	is_top_on_buffer = (*buffer_processed == 0) ? true : false;

	if (is_top_on_buffer) {
		mpeg2ts->curr_timestamp = *timestamp;
		if (mpeg2ts->diff_timestamp == 0) {
			mpeg2ts->diff_timestamp = DEFAULT_DIFF_TIMESTAMP;
		} else {
			num = buffer_size / src_packet_size;
			diff = mpeg2ts->curr_timestamp -
				mpeg2ts->prev_timestamp;
			mpeg2ts->diff_timestamp = diff / num;
		}
		pr_debug("[%s] timestamp curr %u prev %u diff %u num %u\n",
			 __func__,
			 mpeg2ts->curr_timestamp,
			 mpeg2ts->prev_timestamp,
			 mpeg2ts->diff_timestamp,
			 num);
	}

	/* data */
	data = buffer + *buffer_processed;

	data_len = buffer_size - *buffer_processed;
	/* check data length */
	if (data_len % src_packet_size != 0) {
		pr_err("[%s] invalid data length %d\n", __func__, data_len);
		return -EINVAL;
	}
	payloads = data_len / src_packet_size;
	if (payloads > mpeg2ts->mpeg2ts_config.tspackets_per_frame)
		payloads = mpeg2ts->mpeg2ts_config.tspackets_per_frame;

	/* header */
	memcpy(packet,
	       mpeg2ts->packet_template,
	       AVTP_IEC61883_4_PAYLOAD_OFFSET);

	/* variable header */
	avtp_set_sequence_num(packet, mpeg2ts->send_seq_num++);
	avtp_set_stream_data_length(
		packet,
		payloads * AVTP_SOURCE_PACKET_SIZE + AVTP_CIP_HEADER_SIZE);
	avtp_set_iec61883_dbc(packet, mpeg2ts->dbc);
	mpeg2ts->dbc += payloads;

	payload = packet + AVTP_IEC61883_4_PAYLOAD_OFFSET;
	for (i = 0; i < payloads; i++) {
		if (is_ts) {                    /* TS */
			avtp_timestamp = mpeg2ts->curr_timestamp;
			mpeg2ts->curr_timestamp += mpeg2ts->diff_timestamp;
		} else {                        /* M2TS */
			avtp_timestamp = mpeg2ts->curr_timestamp;

			timestamp_ns = m2ts_timestamp_to_nsec(ntohl(*(unsigned long *)data));
			if (is_top_on_buffer) {
				mpeg2ts->m2ts_start = (u32)timestamp_ns;
			} else {
				/* timestamp adjust by m2ts timestamp */
				avtp_timestamp += ((u32)timestamp_ns - mpeg2ts->m2ts_start);
			}

			/* offset adjust by m2ts host_header size */
			data += MSE_TIMESTAMP_SIZE;
		}

		if (i == 0)
			avtp_set_timestamp(packet, avtp_timestamp);

		/* copy to source_header_timestamp */
		*(__be32 *)payload = cpu_to_be32(avtp_timestamp);
		/* copy to MPEG2TS packet */
		memcpy(payload + sizeof(__be32), data, MSE_TS_PACKET_SIZE);

		/* increment payload and data offset */
		payload += AVTP_SOURCE_PACKET_SIZE;
		data += MSE_TS_PACKET_SIZE;
	}

	*packet_size = AVTP_IEC61883_4_PAYLOAD_OFFSET +
		max(AVTP_SOURCE_PACKET_SIZE * payloads, AVTP_PAYLOAD_MIN);
	*buffer_processed += src_packet_size * payloads;

	pr_debug("[%s] bp=%zu/%zu, data_len=%d\n",
		 __func__, *buffer_processed, buffer_size,
		 src_packet_size * payloads);

	if (*buffer_processed >= buffer_size) {
		mpeg2ts->prev_timestamp = *timestamp;

		return MSE_PACKETIZE_STATUS_COMPLETE;
	}

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

static int mse_packetizer_video_iec_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct mpeg2ts_packetizer *mpeg2ts;
	int seq_num;
	int payload_size;
	int offset;
	unsigned char *payload;

	if (index >= ARRAY_SIZE(mpeg2ts_packetizer_table))
		return -EPERM;

	mpeg2ts = &mpeg2ts_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_61883_IIDC) {
		pr_err("[%s] error subtype=%d\n",
		       __func__, avtp_get_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (mpeg2ts->old_seq_num != seq_num &&
	    mpeg2ts->old_seq_num != SEQNUM_INIT) {
		if (mpeg2ts->seq_num_err == SEQNUM_INIT) {
			pr_err("sequence number discontinuity %d->%d=%d\n",
			       mpeg2ts->old_seq_num, seq_num, (seq_num + 1 +
			       AVTP_SEQUENCE_NUM_MAX - mpeg2ts->old_seq_num) %
			       (AVTP_SEQUENCE_NUM_MAX + 1));
			mpeg2ts->seq_num_err = 1;
		} else {
			mpeg2ts->seq_num_err++;
		}
	} else {
		if (mpeg2ts->seq_num_err != SEQNUM_INIT) {
			pr_err("sequence number recovery %d count=%d\n",
			       seq_num, mpeg2ts->seq_num_err);
			mpeg2ts->seq_num_err = SEQNUM_INIT;
		}
	}
	mpeg2ts->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1))
						% (AVTP_SEQUENCE_NUM_MAX + 1);

	payload_size =
		avtp_get_stream_data_length(packet) - AVTP_CIP_HEADER_SIZE;
	payload = (unsigned char *)packet + AVTP_IEC61883_4_PAYLOAD_OFFSET;

	pr_debug("[%s] start size=%d\n", __func__, payload_size);

	/* check size */
	if (*buffer_processed + payload_size >= buffer_size) {
		pr_err("[%s] buffer overrun\n", __func__);
		return -EPERM;
	}

	for (offset = sizeof(__be32);
	     offset < payload_size;
	     offset += AVTP_SOURCE_PACKET_SIZE) {
		pr_debug("[%s] packet %02x %02x %02x %02x\n",
			 __func__,
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

struct mse_packetizer_ops mse_packetizer_video_iec61883_4_ops = {
	.id = MSE_PACKETIZER_IEC61883_4,
	.open = mse_packetizer_video_iec_open,
	.release = mse_packetizer_video_iec_release,
	.init = mse_packetizer_video_iec_packet_init,
	.set_network_config = mse_packetizer_video_iec_set_network_config,
	.set_mpeg2ts_config = mse_packetizer_video_iec_set_mpeg2ts_config,
	.calc_cbs = mse_packetizer_video_iec_calc_cbs,
	.packetize = mse_packetizer_video_iec_packetize,
	.depacketize = mse_packetizer_video_iec_depacketize,
};
