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

#define NSEC            (1000000000L)
#define SEQNUM_INIT     (-1)
#define BYTE_TO_BIT     (8)

#define CBS_ADJUSTMENT_NUMERATOR        (103)
#define CBS_ADJUSTMENT_DENOMINATOR      (100)

/* preamble + FCS + IGP */
#define ETHERNET_SPECIAL        (8 + 4 + 12)

#define PORT_TRANSMIT_RATE      (100000000) /* 100M [bit/sec] */
#define CLASS_INTERVAL_FRAMES   (1000) /* class C */
#define INTERVAL_FRAMES         (1)
#define AM824_DATA_SIZE         (sizeof(u32))

#define MSE_PACKETIZER_MAX      (10)

struct avtp_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int uniqueid;
	int priority;
	int vid;
	int samples_per_frame;
	int channels;
	int sample_rate;
};

struct iec_packetizer {
	bool used_f;
	bool piece_f;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;
	int data_bytes_per_ch;
	unsigned int local_total_samples;
	int piece_data_len;

	unsigned char packet_template[ETHFRAMELEN_MAX];
	unsigned char packet_piece[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_audio_config iec_config;
};

struct iec_packetizer iec_packetizer_table[MSE_PACKETIZER_MAX];

static int mse_packetizer_audio_iec_open(void)
{
	struct iec_packetizer *iec;
	int index;

	for (index = 0; iec_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(iec_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	iec = &iec_packetizer_table[index];

	iec->used_f = true;
	iec->piece_f = false;
	iec->send_seq_num = 0;
	iec->old_seq_num = SEQNUM_INIT;
	iec->seq_num_err = SEQNUM_INIT;
	iec->data_bytes_per_ch = sizeof(unsigned short); /* 1ch 16bit */
	iec->local_total_samples = 0;
	iec->piece_data_len = 0;

	pr_debug("[%s] index=%d\n", __func__, index);

	return index;
}

static int mse_packetizer_audio_iec_release(int index)
{
	struct iec_packetizer *iec;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	iec = &iec_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);

	memset(iec, 0, sizeof(*iec));

	return 0;
}

static int mse_packetizer_audio_iec_packet_init(int index)
{
	struct iec_packetizer *iec;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	iec = &iec_packetizer_table[index];

	iec->piece_f = false;
	iec->send_seq_num = 0;
	iec->old_seq_num = SEQNUM_INIT;
	iec->seq_num_err = SEQNUM_INIT;
	iec->local_total_samples = 0;
	iec->piece_data_len = 0;

	return 0;
}

static int mse_packetizer_audio_iec_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct iec_packetizer *iec;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	iec = &iec_packetizer_table[index];
	iec->net_config = *config;

	return 0;
}

static int mse_packetizer_audio_iec_header_build(void *dst,
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

	hlen = AVTP_IEC61883_6_PAYLOAD_OFFSET;
	len = (param->samples_per_frame * param->channels * sizeof(u32));

	/* 1722 header update + payload */
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_iec61883_6_template(dst);
	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);
	avtp_set_iec61883_dbs(dst, param->channels);
	avtp_set_iec61883_fdf(dst,
			      avtp_sample_rate_to_fdf(param->sample_rate));

	return hlen + len;
}

static int class_interval_frames;

static int mse_packetizer_audio_iec_set_audio_config(
					int index,
					struct mse_audio_config *config)
{
	struct iec_packetizer *iec;
	struct avtp_param param;
	int payload_size;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d rate=%d channels=%d bytes_per_frame=%d\n",
		 __func__, index, config->sample_rate, config->channels,
		 config->bytes_per_frame);
	iec = &iec_packetizer_table[index];
	iec->iec_config = *config;

	/* when bytes_per_frame is not set */
	if (!iec->iec_config.bytes_per_frame) {
		class_interval_frames = CLASS_INTERVAL_FRAMES;
		iec->sample_per_packet = iec->iec_config.sample_rate /
				(class_interval_frames * INTERVAL_FRAMES);
		iec->frame_interval_time = NSEC / CLASS_INTERVAL_FRAMES;
	} else {
		iec->sample_per_packet = iec->iec_config.bytes_per_frame /
			(iec->iec_config.channels * AM824_DATA_SIZE);
		if (iec->sample_per_packet < 2)
			iec->sample_per_packet = 2;
		else if (iec->sample_per_packet > 128)
			iec->sample_per_packet = 128;
		class_interval_frames = iec->iec_config.sample_rate /
					iec->sample_per_packet;
		iec->frame_interval_time = NSEC / class_interval_frames;
	}

	payload_size = iec->sample_per_packet * iec->iec_config.channels *
						AM824_DATA_SIZE;
	iec->avtp_packet_size = AVTP_IEC61883_6_PAYLOAD_OFFSET + payload_size;
	if (iec->avtp_packet_size < ETHFRAMELEN_MIN)
		iec->avtp_packet_size = ETHFRAMELEN_MIN;

	memcpy(param.dest_addr, iec->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, iec->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = iec->net_config.uniqueid;
	param.priority = iec->net_config.priority;
	param.vid = iec->net_config.vlanid;
	param.samples_per_frame = iec->sample_per_packet;
	param.channels = iec->iec_config.channels;
	param.sample_rate = iec->iec_config.sample_rate;

	mse_packetizer_audio_iec_header_build(iec->packet_template, &param);

	return 0;
}

static int mse_packetizer_audio_iec_get_audio_info(
	int index,
	struct mse_audio_info *info)
{
	struct iec_packetizer *iec;

	iec = &iec_packetizer_table[index];
	info->avtp_packet_size = iec->avtp_packet_size;
	info->sample_per_packet = iec->sample_per_packet;
	info->frame_interval_time = iec->frame_interval_time;

	return 0;
}

static int mse_packetizer_audio_iec_calc_cbs(int index,
					     struct eavb_cbsparam *cbs)
{
	struct iec_packetizer *iec;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	iec = &iec_packetizer_table[index];

	bandwidth_fraction_denominator = (u64)PORT_TRANSMIT_RATE *
					 (u64)CBS_ADJUSTMENT_DENOMINATOR;
	if (!bandwidth_fraction_denominator) {
		pr_err("[%s] cbs error(null)\n", __func__);
		return -EPERM;
	}

	bandwidth_fraction_numerator =
		(ETHERNET_SPECIAL + iec->avtp_packet_size) * BYTE_TO_BIT *
		class_interval_frames * INTERVAL_FRAMES *
		CBS_ADJUSTMENT_NUMERATOR;

	value = (u64)UINT_MAX * bandwidth_fraction_numerator;
	/* divide denominator into 2 */
	do_div(value, PORT_TRANSMIT_RATE);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	if (value > UINT_MAX) {
		pr_err("[%s] cbs error(too big)\n", __func__);
		return -EPERM;
	}
	cbs->bandwidthFraction = value;

	value = USHRT_MAX * bandwidth_fraction_numerator;
	/* divide denominator into 2 */
	do_div(value, PORT_TRANSMIT_RATE);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	cbs->sendSlope = value;

	value = USHRT_MAX * (bandwidth_fraction_denominator -
					 bandwidth_fraction_numerator);
	/* divide denominator into 2 */
	do_div(value, PORT_TRANSMIT_RATE);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	cbs->idleSlope = value;

	return 0;
}

#define SET_AM824_MBLA_16BIT(_data) htonl(0x42000000 | (_data << 8))

static int mse_packetizer_audio_iec_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct iec_packetizer *iec;
	int i, payload_len, data_size;
	u32 *sample;
	u16 *data;
	int piece_size = 0, piece_len = 0;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	iec = &iec_packetizer_table[index];
	pr_debug("[%s] index=%d seqnum=%d process=%zu/%zu t=%d\n",
		 __func__, index, iec->send_seq_num, *buffer_processed,
		 buffer_size, *timestamp);

	/* header */
	if (iec->piece_f) {
		piece_len = iec->piece_data_len;
		piece_size = piece_len / AM824_DATA_SIZE *
							iec->data_bytes_per_ch;
		iec->piece_f = false;
		iec->piece_data_len = 0;
		memcpy(packet, iec->packet_piece, iec->avtp_packet_size);
	} else {
		memcpy(packet, iec->packet_template, iec->avtp_packet_size);
	}

	/* payload */
	sample = (u32 *)(packet + AVTP_IEC61883_6_PAYLOAD_OFFSET + piece_len);
	data = (u16 *)(buffer + *buffer_processed);

	/* size check */
	data_size = iec->sample_per_packet * iec->iec_config.channels *
							iec->data_bytes_per_ch;
	payload_len = iec->sample_per_packet * iec->iec_config.channels *
							AM824_DATA_SIZE;

	if (data_size - piece_size > buffer_size - *buffer_processed) {
		iec->piece_f = true;
		data_size = buffer_size - *buffer_processed;
		payload_len = data_size / iec->data_bytes_per_ch *
							AM824_DATA_SIZE;
		*packet_size = AVTP_IEC61883_6_PAYLOAD_OFFSET +
			       payload_len + piece_len;
	} else {
		*packet_size = iec->avtp_packet_size;
	}

	/* 16 bits integer */
	for (i = 0; i < (data_size - piece_size); i++)
		*(sample + i) = SET_AM824_MBLA_16BIT(*(data + i));

	*buffer_processed += data_size - piece_size;

	/* keep piece of data */
	if (iec->piece_f) {
		iec->piece_data_len = payload_len + piece_len;
		memcpy(iec->packet_piece, packet, *packet_size);
		return -1;
	}

	/* variable header */
	avtp_set_sequence_num(packet, iec->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_stream_data_length(packet, payload_len);
	avtp_set_iec61883_dbc(packet, 1 + iec->local_total_samples);

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return 1; /* end of buffer */
	else
		return 0; /* continue */
}

#define GET_AM824_MBLA_16BIT(_data) ((ntohl(_data) & 0x00ffffff) >> 8)

static void mse_packetizer_audio_iec_data_convert(u16 *dst, void *packet)
{
	u32 *payload;
	int payload_size;
	int samples_per_frame;
	int channels;
	int i;

	payload_size = avtp_get_stream_data_length(packet);
	channels = avtp_get_iec61883_dbs(packet);
	samples_per_frame = payload_size / (AM824_DATA_SIZE * channels);
	payload = packet + AVTP_IEC61883_6_PAYLOAD_OFFSET;

	if (channels == 1) {
		/* mono channel to stereo channels */
		for (i = 0; i < samples_per_frame; i++, payload++) {
			*dst++ = GET_AM824_MBLA_16BIT(*payload);
			*dst++ = GET_AM824_MBLA_16BIT(*payload);
		}
	} else {
		/* multi channels to stereo channels */
		for (i = 0; i < samples_per_frame; i++, payload += channels) {
			*dst++ = GET_AM824_MBLA_16BIT(*payload);
			*dst++ = GET_AM824_MBLA_16BIT(*(payload + 1));
		}
	}
}

static int mse_packetizer_audio_iec_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct iec_packetizer *iec;
	int seq_num;
	int payload_size, piece_size = 0;
	int data_size;
	char *buf, tmp_buffer[ETHFRAMEMTU_MAX] = {0};
	unsigned long value;

	if (index >= ARRAY_SIZE(iec_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	iec = &iec_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_61883_IIDC) {
		pr_err("[%s] error subtype=%d\n",
		       __func__, avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (iec->piece_f) {
		iec->piece_f = false;
		memcpy(buffer, iec->packet_piece, iec->piece_data_len);
		*buffer_processed += iec->piece_data_len;
		iec->piece_data_len = 0;
	}

	payload_size = avtp_get_stream_data_length(packet);
	data_size = payload_size / AM824_DATA_SIZE *
					iec->data_bytes_per_ch;
	/* buffer over check */
	if (*buffer_processed + data_size > buffer_size)
		buf = tmp_buffer;
	else
		buf = buffer + *buffer_processed;

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (iec->old_seq_num != seq_num && iec->old_seq_num != SEQNUM_INIT) {
		if (iec->seq_num_err == SEQNUM_INIT) {
			pr_err("sequence number discontinuity %d->%d=%d\n",
			       iec->old_seq_num, seq_num,
			       (seq_num + 1 + AVTP_SEQUENCE_NUM_MAX -
			       iec->old_seq_num) %
			       (AVTP_SEQUENCE_NUM_MAX + 1));
			iec->seq_num_err = 1;
		} else {
			iec->seq_num_err++;
		}
	} else {
		if (iec->seq_num_err != SEQNUM_INIT) {
			pr_err("sequence number recovery %d count=%d\n",
			       seq_num, iec->seq_num_err);
			iec->seq_num_err = SEQNUM_INIT;
		}
	}
	iec->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1)) %
						(AVTP_SEQUENCE_NUM_MAX + 1);

	mse_packetizer_audio_iec_data_convert((u16 *)buf, packet);

	if (*buffer_processed + data_size > buffer_size) {
		iec->piece_f = true;
		piece_size = buffer_size - *buffer_processed;
		iec->piece_data_len = data_size - piece_size;
		memcpy(buffer + *buffer_processed, buf, piece_size);
		memcpy(iec->packet_piece, buf + piece_size,
		       iec->piece_data_len);

		*buffer_processed += (buffer_size - *buffer_processed);
	} else {
		*buffer_processed += data_size;
	}

	*timestamp = avtp_get_timestamp(packet);

	iec->sample_per_packet =
		avtp_get_stream_data_length(packet) /
		(AM824_DATA_SIZE * avtp_get_iec61883_dbs(packet));
	value = NSEC * iec->sample_per_packet;
	do_div(value,
	       avtp_fdf_to_sample_rate(avtp_get_iec61883_fdf(packet)));
	iec->frame_interval_time = value;

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return 1; /* end of buffer */

	return 0; /* continue */
}

struct mse_packetizer_ops mse_packetizer_audio_iec61883_6_ops = {
	.name = MSE_PACKETIZER_NAME_STR_IEC61883_6,
	.priv = NULL,
	.type = MSE_TYPE_PACKETIZER_AUDIO_PCM,
	.open = mse_packetizer_audio_iec_open,
	.release = mse_packetizer_audio_iec_release,
	.init = mse_packetizer_audio_iec_packet_init,
	.set_network_config = mse_packetizer_audio_iec_set_network_config,
	.set_audio_config = mse_packetizer_audio_iec_set_audio_config,
	.get_audio_info = mse_packetizer_audio_iec_get_audio_info,
	.calc_cbs = mse_packetizer_audio_iec_calc_cbs,
	.packetize = mse_packetizer_audio_iec_packetize,
	.depacketize = mse_packetizer_audio_iec_depacketize,
};
