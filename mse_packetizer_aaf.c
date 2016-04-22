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

#define PORT_TRANSMIT_RATE      (100000000) /* 100M [bit/sec] */
#define CLASS_INTERVAL_FRAMES   (1000) /* class A */
#define INTERVAL_FRAMES         (1)

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

struct aaf_packetizer {
	bool used_f;

	int send_seq_num;
	int old_seq_num;
	int seq_num_err;

	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;
	int data_bytes_per_ch;

	unsigned char packet_template[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_audio_config aaf_config;
};

struct aaf_packetizer aaf_packetizer_table[MSE_PACKETIZER_MAX];

static int mse_packetizer_audio_aaf_open(void)
{
	struct aaf_packetizer *aaf;
	int index;

	for (index = 0; aaf_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(aaf_packetizer_table); index++)
		;
	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	aaf = &aaf_packetizer_table[index];

	aaf->used_f = true;
	aaf->send_seq_num = 0;
	aaf->old_seq_num = SEQNUM_INIT;
	aaf->seq_num_err = SEQNUM_INIT;
	aaf->data_bytes_per_ch = sizeof(unsigned short); /* 1ch 16bit */

	pr_debug("[%s] index=%d\n", __func__, index);
	return index;
}

static int mse_packetizer_audio_aaf_release(int index)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	aaf = &aaf_packetizer_table[index];
	pr_debug("[%s] index=%d\n", __func__, index);

	memset(aaf, 0, sizeof(*aaf));
	return 0;
}

static int mse_packetizer_audio_aaf_packet_init(int index)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	aaf = &aaf_packetizer_table[index];

	aaf->send_seq_num = 0;
	aaf->old_seq_num = SEQNUM_INIT;
	aaf->seq_num_err = SEQNUM_INIT;
	return 0;
}

static int mse_packetizer_audio_aaf_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	aaf = &aaf_packetizer_table[index];
	aaf->net_config = *config;
	return 0;
}

static int mse_packetizer_audio_aaf_header_build(void *dst,
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

	hlen = AVTP_AAF_PAYLOAD_OFFSET;
	len = (param->samples_per_frame * param->channels * sizeof(u16));

	/* 1722 header update + payload */
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_aaf_pcm_template(dst);
	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);
	avtp_set_aaf_nsr(dst, avtp_sample_rate_to_nsr(param->sample_rate));
	avtp_set_aaf_channels_per_frame(dst, param->channels);

	return hlen + len;
}

static int mse_packetizer_audio_aaf_set_audio_config(
					int index,
					struct mse_audio_config *config)
{
	struct aaf_packetizer *aaf;
	struct avtp_param param;
	int payload_size;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d rate=%d channels=%d\n",
		 __func__, index, config->sample_rate, config->channels);
	aaf = &aaf_packetizer_table[index];
	aaf->aaf_config = *config;

	aaf->sample_per_packet = aaf->aaf_config.sample_rate /
				(CLASS_INTERVAL_FRAMES * INTERVAL_FRAMES);
	aaf->frame_interval_time = NSEC / CLASS_INTERVAL_FRAMES;
	payload_size = aaf->sample_per_packet * aaf->aaf_config.channels *
						aaf->data_bytes_per_ch;
	aaf->avtp_packet_size = AVTP_AAF_PAYLOAD_OFFSET + payload_size;
	if (aaf->avtp_packet_size < ETHFRAMELEN_MIN)
		aaf->avtp_packet_size = ETHFRAMELEN_MIN;

	memcpy(param.dest_addr, aaf->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, aaf->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = aaf->net_config.uniqueid;
	param.priority = aaf->net_config.priority;
	param.vid = aaf->net_config.vlanid;
	param.samples_per_frame = aaf->sample_per_packet;
	param.channels = aaf->aaf_config.channels;
	param.sample_rate = aaf->aaf_config.sample_rate;

	mse_packetizer_audio_aaf_header_build(aaf->packet_template, &param);

	return 0;
}

static int mse_packetizer_audio_aaf_calc_cbs(int index,
					     struct eavb_cbsparam *cbs)
{
	struct aaf_packetizer *aaf;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	aaf = &aaf_packetizer_table[index];

	bandwidth_fraction_denominator = (u64)PORT_TRANSMIT_RATE *
					 (u64)CBS_ADJUSTMENT_DENOMINATOR;
	if (!bandwidth_fraction_denominator) {
		pr_err("[%s] cbs error(null)\n", __func__);
		return -EPERM;
	}

	bandwidth_fraction_numerator =
		(ETHOVERHEAD_REAL + aaf->avtp_packet_size) * BYTE_TO_BIT *
		CLASS_INTERVAL_FRAMES * INTERVAL_FRAMES *
		CBS_ADJUSTMENT_NUMERATOR;

	value = (u64)UINT_MAX * bandwidth_fraction_numerator;
	do_div(value, bandwidth_fraction_denominator);
	if (value > UINT_MAX) {
		pr_err("[%s] cbs error(too big)\n", __func__);
		return -EPERM;
	}
	cbs->bandwidthFraction = value;

	value = USHRT_MAX * bandwidth_fraction_numerator;
	do_div(value, bandwidth_fraction_denominator);
	cbs->sendSlope = value;

	value = USHRT_MAX * (bandwidth_fraction_denominator
					 - bandwidth_fraction_numerator);
	do_div(value, bandwidth_fraction_denominator);
	cbs->idleSlope = value;

	return 0;
}

static int mse_packetizer_audio_aaf_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct aaf_packetizer *aaf;
	int i, data_len, data_size;
	u16 *sample, *data;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	aaf = &aaf_packetizer_table[index];
	pr_debug("[%s] index=%d seqnum=%d process=%zu/%zu t=%d\n",
		 __func__, index, aaf->send_seq_num, *buffer_processed,
		 buffer_size, *timestamp);

	/* header */
	memcpy(packet, aaf->packet_template, aaf->avtp_packet_size);

	/* payload */
	sample = (u16 *)(packet + AVTP_AAF_PAYLOAD_OFFSET);
	data = (u16 *)(buffer + *buffer_processed);

	/* size check */
	data_size = aaf->sample_per_packet * aaf->aaf_config.channels;
	data_len = data_size * aaf->data_bytes_per_ch;
	if (data_len + *buffer_processed > buffer_size) {
		data_len = buffer_size - *buffer_processed;
		data_size = data_len / aaf->data_bytes_per_ch;
		*packet_size = AVTP_AAF_PAYLOAD_OFFSET + data_len;
	} else {
		*packet_size = aaf->avtp_packet_size;
	}

	/* variable header */
	avtp_set_sequence_num(packet, aaf->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_stream_data_length(packet, data_len);

	/* 16 bits integer */
	for (i = 0; i < data_size; i++)
		*(sample + i) = htons(*(data + i));

	*buffer_processed += data_len;
	*timestamp += aaf->frame_interval_time;

	/* TODO buffer over check */
	if (*buffer_processed >= buffer_size)
		return 1; /* end of buffer */
	else
		return 0; /* continue */
}

#define AAF_32BIT_OFFSET (16)

static void mse_packetizer_audio_aaf_data_convert32(u16 *dst,
						    u32 *src,
						    int channels,
						    int samples_per_frame)
{
	int i;

	if (channels == 1) {
		/* mono channel to stereo channels */
		for (i = 0; i < samples_per_frame; i++, src++) {
			*dst++ = ntohl(src[0]) >> AAF_32BIT_OFFSET;
			*dst++ = ntohl(src[0]) >> AAF_32BIT_OFFSET;
		}
	} else {
		/* multi channels to stereo channels */
		for (i = 0; i < samples_per_frame; i++, src += channels) {
			*dst++ = ntohl(src[0]) >> AAF_32BIT_OFFSET;
			*dst++ = ntohl(src[1]) >> AAF_32BIT_OFFSET;
		}
	}
}

static void mse_packetizer_audio_aaf_data_convert24(u16 *dst,
						    u8 *src,
						    int channels,
						    int samples_per_frame)
{
	/* TODO */
}

static void mse_packetizer_audio_aaf_data_convert16(u16 *dst,
						    u16 *src,
						    int channels,
						    int samples_per_frame)
{
	int i;

	if (channels == 1) {
		/* mono channel to stereo channels */
		for (i = 0; i < samples_per_frame; i++, src++) {
			*dst++ = ntohs(src[0]);
			*dst++ = ntohs(src[0]);
		}
	} else {
		/* multi channels to stereo channels */
		for (i = 0; i < samples_per_frame; i++, src += channels) {
			*dst++ = ntohs(src[0]);
			*dst++ = ntohs(src[1]);
		}
	}
}

static int mse_packetizer_audio_aaf_data_convert(u16 *dst, void *packet)
{
	void *payload;
	int payload_size;
	int samples_per_frame;
	int aaf_format;
	int channels;

	payload_size = avtp_get_stream_data_length(packet);
	channels = avtp_get_aaf_channels_per_frame(packet);
	aaf_format = avtp_get_aaf_format(packet);
	samples_per_frame = payload_size
			/ (avtp_aaf_format_to_bytes(aaf_format) * channels);
	payload = packet + AVTP_AAF_PAYLOAD_OFFSET;

	switch (aaf_format) {
	case AVTP_AAF_FORMAT_INT_32BIT:
		mse_packetizer_audio_aaf_data_convert32(dst,
							payload,
							channels,
							samples_per_frame);
		break;
	case AVTP_AAF_FORMAT_INT_24BIT:
		mse_packetizer_audio_aaf_data_convert24(dst,
							payload,
							channels,
							samples_per_frame);
		break;
	case AVTP_AAF_FORMAT_INT_16BIT:
		mse_packetizer_audio_aaf_data_convert16(dst,
							payload,
							channels,
							samples_per_frame);
		break;
	default:
		return -EPERM;
	}

	return 0;
}

static int mse_packetizer_audio_aaf_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	struct aaf_packetizer *aaf;
	int seq_num;
	int err;
	int payload_size;

	pr_debug("[%s] index=%d\n", __func__, index);
	aaf = &aaf_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_AAF) {
		pr_err("[%s] error subtype=%d\n",
		       __func__, avtp_get_subtype(packet));
		return -EINVAL;
	}

	/* seq_num check */
	seq_num = avtp_get_sequence_num(packet);
	if (aaf->old_seq_num != seq_num && aaf->old_seq_num != SEQNUM_INIT) {
		if (aaf->seq_num_err == SEQNUM_INIT) {
			pr_err("sequence number discontinuity %d->%d=%d\n",
			       aaf->old_seq_num, seq_num,
			       (seq_num + 1 + AVTP_SEQUENCE_NUM_MAX -
			       aaf->old_seq_num) %
			       (AVTP_SEQUENCE_NUM_MAX + 1));
			aaf->seq_num_err = 1;
		} else {
			aaf->seq_num_err++;
		}
	} else {
		if (aaf->seq_num_err != SEQNUM_INIT) {
			pr_err("sequence number recovery %d count=%d\n",
			       seq_num, aaf->seq_num_err);
			aaf->seq_num_err = SEQNUM_INIT;
		}
	}
	aaf->old_seq_num = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1)) %
						(AVTP_SEQUENCE_NUM_MAX + 1);

	payload_size = avtp_get_stream_data_length(packet);
	err = mse_packetizer_audio_aaf_data_convert(
					(u16 *)(buffer + *buffer_processed),
					packet);
	if (err) {
		pr_err("[%s] error convert\n", __func__);
		return -EPERM;
	}

	*buffer_processed += payload_size;
	*timestamp = avtp_get_timestamp(packet);

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return 1; /* end of buffer */

	return 0; /* continue */
}

struct mse_packetizer_ops mse_packetizer_audio_aaf_ops = {
	.name = MSE_PACKETIZER_NAME_STR_AAF_PCM,
	.priv = NULL,
	.type = MSE_TYPE_PACKETIZER_AUDIO_PCM,
	.open = mse_packetizer_audio_aaf_open,
	.release = mse_packetizer_audio_aaf_release,
	.init = mse_packetizer_audio_aaf_packet_init,
	.set_network_config = mse_packetizer_audio_aaf_set_network_config,
	.set_audio_config = mse_packetizer_audio_aaf_set_audio_config,
	.calc_cbs = mse_packetizer_audio_aaf_calc_cbs,
	.packetize = mse_packetizer_audio_aaf_packetize,
	.depacketize = mse_packetizer_audio_aaf_depacketize,
};
