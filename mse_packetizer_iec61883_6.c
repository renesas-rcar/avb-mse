/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2017 Renesas Electronics Corporation

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

#define NSEC            (1000000000L)
#define SEQNUM_INIT     (-1)
#define BYTE_TO_BIT     (8)

#define CBS_ADJUSTMENT_NUMERATOR        (103)
#define CBS_ADJUSTMENT_DENOMINATOR      (100)

/* preamble + FCS */
#define ETHERNET_SPECIAL        (8 + 4)

#define CLASS_INTERVAL_FRAMES   (8000) /* class A */
#define INTERVAL_FRAMES         (1)
#define AM824_DATA_SIZE         (sizeof(u32))

struct avtp_iec61883_6_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int uniqueid;
	int priority;
	int vid;
	int samples_per_frame;
	int channels;
	int sample_rate;
};

struct iec61883_6_packetizer {
	bool used_f;
	bool piece_f;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;

	int class_interval_frames;

	u8 local_total_samples;
	int piece_data_len;
	bool f_warned;

	unsigned char packet_template[ETHFRAMELEN_MAX];
	unsigned char packet_piece[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_audio_config audio_config;
};

struct iec61883_6_packetizer iec61883_6_packetizer_table[MSE_INSTANCE_MAX];

static int check_receive_packet(int index, int channels, void *packet)
{
	int sample_rate;
	struct iec61883_6_packetizer *iec61883_6;
	struct mse_audio_config *audio_config;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	audio_config = &iec61883_6->audio_config;

	if (channels != audio_config->channels) {
		mse_err("packet's ch=%d != cfg ch=%d\n",
			channels, audio_config->channels);
		return -EINVAL;
	}

	sample_rate = avtp_fdf_to_sample_rate(avtp_get_iec61883_fdf(packet));
	if (sample_rate != audio_config->sample_rate) {
		mse_err("packet's sample_rate=%d != cfg sample_rate=%d\n",
			sample_rate, audio_config->sample_rate);
		return -EINVAL;
	}

	return 0;
}

static int check_packet_format(int index)
{
	struct iec61883_6_packetizer *iec61883_6;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;
	iec61883_6 = &iec61883_6_packetizer_table[index];

	switch (iec61883_6->audio_config.sample_bit_depth) {
	case MSE_AUDIO_BIT_16:
		if (iec61883_6->audio_config.bytes_per_sample != 2)
			goto format_err;
		break;
	case MSE_AUDIO_BIT_18:
	case MSE_AUDIO_BIT_20:
		if (iec61883_6->audio_config.bytes_per_sample != 3)
			goto format_err;
		break;
	case MSE_AUDIO_BIT_24:
		if (iec61883_6->audio_config.bytes_per_sample != 3 &&
		    iec61883_6->audio_config.bytes_per_sample != 4)
			goto format_err;
		break;
	default:
		goto format_err;
	}

	if (avtp_sample_rate_to_fdf(iec61883_6->audio_config.sample_rate) ==
	    (AVTP_IEC61883_6_FDF_EVT_AM824 |
	     AVTP_IEC61883_6_FDF_SFC_RESERVED)) {
		mse_err("invalid sample rata %d\n",
			iec61883_6->audio_config.sample_rate);

		return -EINVAL;
	}

	if (iec61883_6->audio_config.channels < 1 ||
	    iec61883_6->audio_config.channels > 24) {
		mse_err("invalid channel number %d\n",
			iec61883_6->audio_config.channels);

		return -EINVAL;
	}

	return 0;

format_err:
	mse_err("invalid format bit_depth %d bytes_per_sample=%d\n",
		mse_get_bit_depth(iec61883_6->audio_config.sample_bit_depth),
		iec61883_6->audio_config.bytes_per_sample);

	return -EINVAL;
}

static int mse_packetizer_iec61883_6_open(void)
{
	struct iec61883_6_packetizer *iec61883_6;
	int index;

	for (index = 0; iec61883_6_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(iec61883_6_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	iec61883_6 = &iec61883_6_packetizer_table[index];

	iec61883_6->used_f = true;
	iec61883_6->piece_f = false;
	iec61883_6->send_seq_num = 0;
	iec61883_6->old_seq_num = SEQNUM_INIT;
	iec61883_6->seq_num_err = SEQNUM_INIT;
	iec61883_6->local_total_samples = 0;
	iec61883_6->piece_data_len = 0;

	mse_debug("index=%d\n", index);

	return index;
}

static int mse_packetizer_iec61883_6_release(int index)
{
	struct iec61883_6_packetizer *iec61883_6;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	mse_debug("index=%d\n", index);

	memset(iec61883_6, 0, sizeof(*iec61883_6));

	return 0;
}

static int mse_packetizer_iec61883_6_packet_init(int index)
{
	struct iec61883_6_packetizer *iec61883_6;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_6 = &iec61883_6_packetizer_table[index];

	iec61883_6->piece_f = false;
	iec61883_6->send_seq_num = 0;
	iec61883_6->old_seq_num = SEQNUM_INIT;
	iec61883_6->seq_num_err = SEQNUM_INIT;
	iec61883_6->local_total_samples = 0;
	iec61883_6->piece_data_len = 0;

	return 0;
}

static int mse_packetizer_iec61883_6_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct iec61883_6_packetizer *iec61883_6;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_6 = &iec61883_6_packetizer_table[index];
	iec61883_6->net_config = *config;

	return 0;
}

static int mse_packetizer_iec61883_6_header_build(
					void *dst,
					struct avtp_iec61883_6_param *param)
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
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_iec61883_6_template(dst);
	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len + AVTP_CIP_HEADER_SIZE);
	avtp_set_iec61883_dbs(dst, param->channels);
	avtp_set_iec61883_fdf(dst,
			      avtp_sample_rate_to_fdf(param->sample_rate));

	return hlen + len;
}

static int mse_packetizer_iec61883_6_set_audio_config(
					int index,
					struct mse_audio_config *config)
{
	struct iec61883_6_packetizer *iec61883_6;
	struct avtp_iec61883_6_param param;
	struct mse_audio_config *audio_config;
	struct mse_network_config *net_config;
	int payload_size;
	int ret;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	mse_debug("index=%d rate=%d channels=%d samples_per_frame=%d\n",
		  index, config->sample_rate, config->channels,
		  config->samples_per_frame);
	iec61883_6 = &iec61883_6_packetizer_table[index];
	iec61883_6->audio_config = *config;

	ret = check_packet_format(index);
	if (ret < 0)
		return ret;

	audio_config = &iec61883_6->audio_config;
	/* when samples_per_frame is not set */
	if (!audio_config->samples_per_frame) {
		iec61883_6->class_interval_frames = CLASS_INTERVAL_FRAMES;
		iec61883_6->sample_per_packet = DIV_ROUND_UP(
			audio_config->sample_rate,
			iec61883_6->class_interval_frames * INTERVAL_FRAMES);
		iec61883_6->frame_interval_time = NSEC / CLASS_INTERVAL_FRAMES;
	} else {
		iec61883_6->sample_per_packet = audio_config->samples_per_frame;
		if (iec61883_6->sample_per_packet < 2)
			iec61883_6->sample_per_packet = 2;
		else if (iec61883_6->sample_per_packet > 128)
			iec61883_6->sample_per_packet = 128;

		iec61883_6->class_interval_frames = DIV_ROUND_UP(
			audio_config->sample_rate,
			iec61883_6->sample_per_packet);
		iec61883_6->frame_interval_time =
				NSEC / iec61883_6->class_interval_frames;
	}

	payload_size = iec61883_6->sample_per_packet * audio_config->channels *
		       AM824_DATA_SIZE;
	iec61883_6->avtp_packet_size = AVTP_IEC61883_6_PAYLOAD_OFFSET +
				       payload_size;
	if (iec61883_6->avtp_packet_size < ETHFRAMELEN_MIN)
		iec61883_6->avtp_packet_size = ETHFRAMELEN_MIN;

	net_config = &iec61883_6->net_config;
	memcpy(param.dest_addr, net_config->dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, net_config->source_addr, MSE_MAC_LEN_MAX);
	param.uniqueid = net_config->uniqueid;
	param.priority = net_config->priority;
	param.vid = net_config->vlanid;
	param.samples_per_frame = iec61883_6->sample_per_packet;
	param.channels = audio_config->channels;
	param.sample_rate = audio_config->sample_rate;

	mse_packetizer_iec61883_6_header_build(iec61883_6->packet_template,
					       &param);

	return 0;
}

static int mse_packetizer_iec61883_6_get_audio_info(
					int index,
					struct mse_audio_info *info)
{
	struct iec61883_6_packetizer *iec61883_6;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	info->avtp_packet_size = iec61883_6->avtp_packet_size;
	info->sample_per_packet = iec61883_6->sample_per_packet;
	info->frame_interval_time = iec61883_6->frame_interval_time;

	return 0;
}

static int mse_packetizer_iec61883_6_calc_cbs(int index,
					      struct mse_cbsparam *cbs)
{
	struct iec61883_6_packetizer *iec61883_6;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_6 = &iec61883_6_packetizer_table[index];

	bandwidth_fraction_denominator =
				(u64)iec61883_6->net_config.port_transmit_rate *
				(u64)CBS_ADJUSTMENT_DENOMINATOR;
	if (!bandwidth_fraction_denominator) {
		mse_err("cbs error(null)\n");
		return -EPERM;
	}

	bandwidth_fraction_numerator =
		(ETHERNET_SPECIAL + iec61883_6->avtp_packet_size) *
		BYTE_TO_BIT * iec61883_6->class_interval_frames *
		INTERVAL_FRAMES * CBS_ADJUSTMENT_NUMERATOR;

	value = (u64)UINT_MAX * bandwidth_fraction_numerator;
	/* divide denominator into 2 */
	do_div(value, iec61883_6->net_config.port_transmit_rate);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	if (value > UINT_MAX) {
		mse_err("cbs error(too big)\n");
		return -EPERM;
	}
	cbs->bandwidth_fraction = value;

	value = USHRT_MAX * bandwidth_fraction_numerator;
	/* divide denominator into 2 */
	do_div(value, iec61883_6->net_config.port_transmit_rate);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	cbs->send_slope = value;

	value = USHRT_MAX * (bandwidth_fraction_denominator -
					 bandwidth_fraction_numerator);
	/* divide denominator into 2 */
	do_div(value, iec61883_6->net_config.port_transmit_rate);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	cbs->idle_slope = value;

	return 0;
}

#define SET_AM824_MBLA_24BIT(_data) \
	htonl(0x40000000 | ((_data) & 0xFFFFFF))
#define SET_AM824_MBLA_20BIT(_data) \
	htonl(0x41000000 | (((_data) << 4) & 0xFFFFFF))
#define SET_AM824_MBLA_18BIT(_data) \
	htonl(0x41000000 | (((_data) << 6) & 0xFFFFFF))
#define SET_AM824_MBLA_16BIT(_data) \
	htonl(0x42000000 | (((_data) << 8) & 0xFFFFFF))

#define SET_AM824_MBLA_24BIT_BE(_data) \
	(0x00000040 | ((_data) << 8))
#define SET_AM824_MBLA_16BIT_BE(_data) \
	(0x00000042 | (((_data) << 8) & 0xFFFF00))

static int mse_packetizer_iec61883_6_set_payload(int index,
						 int data_num,
						 u32 *sample,
						 void *buffer,
						 size_t buffer_processed)
{
	struct iec61883_6_packetizer *iec61883_6;
	int i, j = 0, count;
	u32 tmp;
	union {
		u8 *d8;
		u16 *d16;
		u32 *d32;
	} data;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	data.d16 = (u16 *)(buffer + buffer_processed);
	count = data_num / iec61883_6->audio_config.bytes_per_sample;

	if (iec61883_6->audio_config.is_big_endian) {
		if (iec61883_6->audio_config.bytes_per_sample == 4) {
			for (i = 0; i < count; i++) {
				*(sample + i) = SET_AM824_MBLA_24BIT_BE(
					*(data.d32 + i));
			}
			return 0;
		}
		switch (iec61883_6->audio_config.sample_bit_depth) {
		case MSE_AUDIO_BIT_16:
			for (i = 0; i < count; i++) {
				*(sample + i) = SET_AM824_MBLA_16BIT_BE(
					*(data.d16 + i));
			}
			return 0;
		case MSE_AUDIO_BIT_18:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i) << 16
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2);
				*(sample + j++) = SET_AM824_MBLA_18BIT(tmp);
			}
			return 0;
		case MSE_AUDIO_BIT_20:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i) << 16
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2);
				*(sample + j++) = SET_AM824_MBLA_20BIT(tmp);
			}
			return 0;
		case MSE_AUDIO_BIT_24:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i) << 16
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2);
				*(sample + j++) = SET_AM824_MBLA_24BIT(tmp);
			}
			return 0;
		default:
			return -EPERM;
		}
	} else {
		if (iec61883_6->audio_config.bytes_per_sample == 4) {
			for (i = 0; i < count; i++) {
				*(sample + i) =
					SET_AM824_MBLA_24BIT(*(data.d32 + i));
			}
			return 0;
		}
		switch (iec61883_6->audio_config.sample_bit_depth) {
		case MSE_AUDIO_BIT_16:
			for (i = 0; i < count; i++) {
				*(sample + i) =
					SET_AM824_MBLA_16BIT(*(data.d16 + i));
			}
			return 0;
		case MSE_AUDIO_BIT_18:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i)
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2) << 16;
				*(sample + j++) = SET_AM824_MBLA_18BIT(tmp);
			}
			return 0;
		case MSE_AUDIO_BIT_20:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i)
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2) << 16;
				*(sample + j++) = SET_AM824_MBLA_20BIT(tmp);
			}
			return 0;
		case MSE_AUDIO_BIT_24:
			for (i = 0; i < data_num; i += 3) {
				tmp = *(data.d8 + i)
					| *(data.d8 + i + 1) << 8
					| *(data.d8 + i + 2) << 16;
				*(sample + j++) = SET_AM824_MBLA_24BIT(tmp);
			}
			return 0;
		default:
			return -EPERM;
		}
	}
	return -EPERM;
}

static int mse_packetizer_iec61883_6_packetize(int index,
					       void *packet,
					       size_t *packet_size,
					       void *buffer,
					       size_t buffer_size,
					       size_t *buffer_processed,
					       unsigned int *timestamp)
{
	struct iec61883_6_packetizer *iec61883_6;
	struct mse_audio_config *audio_config;
	int ret, payload_len, data_size;
	u32 *sample;
	int piece_size = 0, piece_len = 0;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	mse_debug("index=%d seqnum=%d process=%zu/%zu t=%d\n",
		  index, iec61883_6->send_seq_num, *buffer_processed,
		  buffer_size, *timestamp);

	audio_config = &iec61883_6->audio_config;
	/* header */
	if (iec61883_6->piece_f) {
		piece_len = iec61883_6->piece_data_len;
		piece_size = piece_len / AM824_DATA_SIZE *
			     audio_config->bytes_per_sample;
		iec61883_6->piece_f = false;
		iec61883_6->piece_data_len = 0;
		memcpy(packet, iec61883_6->packet_piece,
		       iec61883_6->avtp_packet_size);
	} else {
		memcpy(packet, iec61883_6->packet_template,
		       iec61883_6->avtp_packet_size);
	}

	/* payload */
	sample = (u32 *)(packet + AVTP_IEC61883_6_PAYLOAD_OFFSET + piece_len);

	/* size check */
	data_size = iec61883_6->sample_per_packet * audio_config->channels *
		    audio_config->bytes_per_sample;
	payload_len = iec61883_6->sample_per_packet * audio_config->channels *
		      AM824_DATA_SIZE;

	if (data_size - piece_size > buffer_size - *buffer_processed) {
		iec61883_6->piece_f = true;
		data_size = buffer_size - *buffer_processed;
		payload_len = data_size / audio_config->bytes_per_sample *
			      AM824_DATA_SIZE;
		*packet_size = AVTP_IEC61883_6_PAYLOAD_OFFSET +
			       payload_len + piece_len;
	} else {
		*packet_size = iec61883_6->avtp_packet_size;
	}

	ret = mse_packetizer_iec61883_6_set_payload(index,
						    data_size - piece_size,
						    sample,
						    buffer,
						    *buffer_processed);

	*buffer_processed += data_size - piece_size;

	/* keep piece of data */
	if (iec61883_6->piece_f) {
		iec61883_6->piece_data_len = payload_len + piece_len;
		memcpy(iec61883_6->packet_piece, packet, *packet_size);
		return MSE_PACKETIZE_STATUS_NOT_ENOUGH;
	}

	/* variable header */
	avtp_set_sequence_num(packet, iec61883_6->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_iec61883_dbc(packet, iec61883_6->local_total_samples);
	iec61883_6->local_total_samples += iec61883_6->sample_per_packet;

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return MSE_PACKETIZE_STATUS_COMPLETE;
	else
		return MSE_PACKETIZE_STATUS_CONTINUE;
}

#define GET_AM824_MBLA_VBL(_data) \
	(((_data) & 0x03000000) >> 24)

static int get_am824_mbla_value(u32 *data, int *bit_depth)
{
	u32 value = ntohl(*data);

	switch (GET_AM824_MBLA_VBL(value)) {
	case 0:                               /* 24 bit */
		*bit_depth = 24;
		return value & 0x00ffffff;
	case 1:                               /* 20 bit */
		*bit_depth = 20;
		return (value & 0x00ffffff) >> 4;
	case 2:                               /* 16 bit */
		*bit_depth = 16;
		return (value & 0x00ffffff) >> 8;
	default:
		*bit_depth = -1;
		return -1;
	}
}

static int mse_packetizer_iec61883_6_data_convert(int index,
						  int data_num,
						  char *buf,
						  void *packet)
{
	u32 *payload;
	struct iec61883_6_packetizer *iec61883_6;
	struct mse_audio_config *audio_config;
	int i;
	int buf_bit_depth;

	iec61883_6 = &iec61883_6_packetizer_table[index];
	audio_config = &iec61883_6->audio_config;
	payload = packet + AVTP_IEC61883_6_PAYLOAD_OFFSET;
	buf_bit_depth = mse_get_bit_depth(audio_config->sample_bit_depth);

	for (i = 0; i < (data_num / audio_config->bytes_per_sample); i++) {
		int bit_depth, shift;
		int value = get_am824_mbla_value(payload++, &bit_depth);

		if (value < 0) {
			mse_err("am824 format error\n");
			return -EINVAL;
		}

		if (bit_depth != buf_bit_depth &&
		    !iec61883_6->f_warned) {
			mse_warn("packet's bit_depth=%d != cfg bit_depth=%d\n",
				 bit_depth, buf_bit_depth);
			iec61883_6->f_warned = true;
		}

		shift = bit_depth - buf_bit_depth;
		if (shift > 0)
			value >>= shift;
		else
			value <<= -shift;

		if (audio_config->is_big_endian) {
			value = htonl(value);
			memcpy(buf, ((unsigned char *)&value) +
			       4 - audio_config->bytes_per_sample,
			       audio_config->bytes_per_sample);
		} else {
			memcpy(buf, &value, audio_config->bytes_per_sample);
		}

		buf += audio_config->bytes_per_sample;
	}

	return 0;
}

static int mse_packetizer_iec61883_6_depacketize(int index,
						 void *buffer,
						 size_t buffer_size,
						 size_t *buffer_processed,
						 unsigned int *timestamp,
						 void *packet,
						 size_t packet_size)
{
	struct iec61883_6_packetizer *iec61883_6;
	int seq_num;
	int payload_size, piece_size = 0;
	int channels;
	int data_size;
	char *buf, tmp_buffer[ETHFRAMEMTU_MAX] = {0};
	unsigned long value;
	int ret;

	if (index >= ARRAY_SIZE(iec61883_6_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	iec61883_6 = &iec61883_6_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_61883_IIDC) {
		mse_err("error subtype=%d\n", avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (iec61883_6->piece_f) {
		iec61883_6->piece_f = false;
		memcpy(buffer, iec61883_6->packet_piece,
		       iec61883_6->piece_data_len);
		*buffer_processed += iec61883_6->piece_data_len;
		iec61883_6->piece_data_len = 0;
	}

	payload_size = avtp_get_stream_data_length(packet)
		- AVTP_CIP_HEADER_SIZE;
	channels = avtp_get_iec61883_dbs(packet);

	data_size = payload_size / AM824_DATA_SIZE *
		iec61883_6->audio_config.bytes_per_sample;
	/* buffer over check */
	if (*buffer_processed + data_size > buffer_size)
		buf = tmp_buffer;
	else
		buf = buffer + *buffer_processed;

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (iec61883_6->old_seq_num != seq_num &&
	    iec61883_6->old_seq_num != SEQNUM_INIT) {
		if (iec61883_6->seq_num_err == SEQNUM_INIT) {
			mse_err("sequence number discontinuity %d->%d=%d\n",
				iec61883_6->old_seq_num, seq_num,
				(seq_num + 1 + AVTP_SEQUENCE_NUM_MAX -
				 iec61883_6->old_seq_num) %
				(AVTP_SEQUENCE_NUM_MAX + 1));
			iec61883_6->seq_num_err = 1;
		} else {
			iec61883_6->seq_num_err++;
		}
	} else {
		if (iec61883_6->seq_num_err != SEQNUM_INIT) {
			mse_err("sequence number recovery %d count=%d\n",
				seq_num, iec61883_6->seq_num_err);
			iec61883_6->seq_num_err = SEQNUM_INIT;
		}
	}
	iec61883_6->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1)) %
						(AVTP_SEQUENCE_NUM_MAX + 1);

	ret = check_receive_packet(index, avtp_get_iec61883_dbs(packet),
				   packet);
	if (ret < 0)
		return ret;

	mse_packetizer_iec61883_6_data_convert(index,
					       data_size - piece_size,
					       buf,
					       packet);

	if (*buffer_processed + data_size > buffer_size) {
		iec61883_6->piece_f = true;
		piece_size = buffer_size - *buffer_processed;
		iec61883_6->piece_data_len = data_size - piece_size;
		memcpy(buffer + *buffer_processed, buf, piece_size);
		memcpy(iec61883_6->packet_piece, buf + piece_size,
		       iec61883_6->piece_data_len);

		*buffer_processed += (buffer_size - *buffer_processed);
	} else {
		*buffer_processed += data_size;
	}

	*timestamp = avtp_get_timestamp(packet);

	iec61883_6->sample_per_packet =
				payload_size / (AM824_DATA_SIZE * channels);
	value = NSEC * iec61883_6->sample_per_packet;
	do_div(value,
	       avtp_fdf_to_sample_rate(avtp_get_iec61883_fdf(packet)));
	iec61883_6->frame_interval_time = value;

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return MSE_PACKETIZE_STATUS_COMPLETE;

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

struct mse_packetizer_ops mse_packetizer_iec61883_6_ops = {
	.open = mse_packetizer_iec61883_6_open,
	.release = mse_packetizer_iec61883_6_release,
	.init = mse_packetizer_iec61883_6_packet_init,
	.set_network_config = mse_packetizer_iec61883_6_set_network_config,
	.set_audio_config = mse_packetizer_iec61883_6_set_audio_config,
	.get_audio_info = mse_packetizer_iec61883_6_get_audio_info,
	.calc_cbs = mse_packetizer_iec61883_6_calc_cbs,
	.packetize = mse_packetizer_iec61883_6_packetize,
	.depacketize = mse_packetizer_iec61883_6_depacketize,
};
