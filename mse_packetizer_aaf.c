/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2018,2021 Renesas Electronics Corporation

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
#include <linux/of_device.h>

#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "avtp.h"

struct avtp_aaf_param {
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
	bool piece_f;

	int send_seq_num;

	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;
	bool has_valid_avtp_timestamp;
	u32 last_avtp_timestamp;
	enum AVTP_AAF_FORMAT avtp_format;
	int avtp_bytes_per_ch;
	int shift;

	int class_interval_frames;

	int piece_data_len;
	bool f_warned;
	bool f_need_calc_offset;
	u32 start_time;

	unsigned char packet_template[ETHFRAMELEN_MAX];
	unsigned char packet_piece[ETHFRAMELEN_MAX];

	struct mse_network_config net_config;
	struct mse_audio_config audio_config;
	struct mse_packetizer_stats stats;
};

struct aaf_packetizer aaf_packetizer_table[MSE_INSTANCE_MAX];

static enum AVTP_AAF_FORMAT get_aaf_format(enum MSE_AUDIO_BIT bit_depth)
{
	switch (bit_depth) {
	case MSE_AUDIO_BIT_16:
		return AVTP_AAF_FORMAT_INT_16BIT;
	case MSE_AUDIO_BIT_18:
	case MSE_AUDIO_BIT_20:
	case MSE_AUDIO_BIT_24:
		return AVTP_AAF_FORMAT_INT_24BIT;
	case MSE_AUDIO_BIT_32:
		return AVTP_AAF_FORMAT_INT_32BIT;
	default:
		return AVTP_AAF_FORMAT_RESERVED;
	}
}

static int get_bit_shift(enum MSE_AUDIO_BIT bit_depth)
{
	switch (bit_depth) {
	case MSE_AUDIO_BIT_18:
		return 6;
	case MSE_AUDIO_BIT_20:
		return 4;
	default:
		return 0;
	}
}

static int get_aaf_format_size(int aaf_format)
{
	switch (aaf_format) {
	case AVTP_AAF_FORMAT_INT_16BIT:
		return 2;
	case AVTP_AAF_FORMAT_INT_24BIT:
		return 3;
	case AVTP_AAF_FORMAT_INT_32BIT:
		return 4;
	default:
		return 0;
	}
}

static int check_receive_packet(int index, int channels,
				int sample_rate, int bit_depth)
{
	struct aaf_packetizer *aaf = &aaf_packetizer_table[index];
	struct mse_audio_config *audio_config = &aaf->audio_config;

	if (channels != audio_config->channels) {
		mse_err("packet's ch=%d != cfg ch=%d\n",
			channels, audio_config->channels);
		return -EINVAL;
	}

	if (sample_rate != audio_config->sample_rate) {
		mse_err("packet's sample_rate=%d != cfg sample_rate=%d\n",
			sample_rate, audio_config->sample_rate);
		return -EINVAL;
	}

	if (!aaf->f_warned &&
	    bit_depth != mse_get_bit_depth(audio_config->sample_bit_depth)) {
		mse_warn("packet's bit_depth=%d != cfg bit_depth=%d\n",
			 bit_depth,
			 mse_get_bit_depth(audio_config->sample_bit_depth));
		aaf->f_warned = true;
	}

	return 0;
}

static int check_packet_format(int index)
{
	int err;
	struct aaf_packetizer *aaf;
	struct mse_audio_config *audio_config;
	enum AVTP_AAF_FORMAT avtp_format;
	enum MSE_AUDIO_BIT sample_bit_depth;
	int bytes_per_sample, bit_depth, sample_rate, channels;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];
	audio_config = &aaf->audio_config;

	sample_bit_depth = audio_config->sample_bit_depth;
	bytes_per_sample = audio_config->bytes_per_sample;
	bit_depth = mse_get_bit_depth(sample_bit_depth);
	sample_rate = audio_config->sample_rate;
	channels = audio_config->channels;

	avtp_format = get_aaf_format(sample_bit_depth);
	if (avtp_format == AVTP_AAF_FORMAT_RESERVED) {
		mse_err("invalid aaf format by bit_depth %d\n", bit_depth);

		return -EINVAL;
	}

	err = 0;
	switch (sample_bit_depth) {
	case MSE_AUDIO_BIT_16:
		if (bytes_per_sample != 2)
			err = 1;
		break;
	case MSE_AUDIO_BIT_18:
	case MSE_AUDIO_BIT_20:
		if (bytes_per_sample != 3)
			err = 1;
		break;
	case MSE_AUDIO_BIT_24:
		if (bytes_per_sample != 3 &&
		    bytes_per_sample != 4)
			err = 1;
		break;
	case MSE_AUDIO_BIT_32:
		if (bytes_per_sample != 4)
			err = 1;
		break;
	default:
		err = 1;
		break;
	}
	if (err) {
		mse_err("invalid format bit_depth=%d bytes_per_sample=%d\n",
			bit_depth, bytes_per_sample);

		return -EINVAL;
	}

	if (avtp_sample_rate_to_nsr(sample_rate) == AVTP_AAF_NSR_USER) {
		mse_err("invalid sample rate %d\n", sample_rate);

		return -EINVAL;
	}

	if (channels < 1 || channels > 24) {
		mse_err("invalid channel number %d\n", channels);

		return -EINVAL;
	}

	return 0;
}

static int mse_packetizer_aaf_open(void)
{
	struct aaf_packetizer *aaf;
	int index;

	for (index = 0; aaf_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(aaf_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];

	aaf->used_f = true;
	aaf->piece_f = false;
	aaf->send_seq_num = 0;
	aaf->piece_data_len = 0;
	aaf->start_time = 0;
	aaf->f_need_calc_offset = false;
	aaf->has_valid_avtp_timestamp = false;
	aaf->last_avtp_timestamp = 0;

	mse_packetizer_stats_init(&aaf->stats);

	mse_debug("index=%d\n", index);

	return index;
}

static int mse_packetizer_aaf_release(int index)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];
	mse_debug("index=%d\n", index);

	mse_packetizer_stats_report(&aaf->stats);

	memset(aaf, 0, sizeof(*aaf));

	return 0;
}

static int mse_packetizer_aaf_packet_init(int index)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d\n", index);
	aaf = &aaf_packetizer_table[index];

	aaf->piece_f = false;
	aaf->send_seq_num = 0;
	aaf->piece_data_len = 0;
	aaf->start_time = 0;
	aaf->f_need_calc_offset = false;
	aaf->has_valid_avtp_timestamp = false;
	aaf->last_avtp_timestamp = 0;

	mse_packetizer_stats_init(&aaf->stats);

	return 0;
}

static int mse_packetizer_aaf_set_network_config(
					int index,
					struct mse_network_config *config)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d\n", index);
	aaf = &aaf_packetizer_table[index];
	aaf->net_config = *config;

	return 0;
}

static int mse_packetizer_aaf_header_build(void *dst,
					   struct avtp_aaf_param *param)
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
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_aaf_pcm_template(dst);
	avtp_set_stream_id(dst, streamid);
	avtp_set_stream_data_length(dst, len);
	avtp_set_aaf_nsr(dst, avtp_sample_rate_to_nsr(param->sample_rate));
	avtp_set_aaf_channels_per_frame(dst, param->channels);

	return hlen + len;
}

static int mse_packetizer_aaf_set_audio_config(int index,
					       struct mse_audio_config *config)
{
	struct aaf_packetizer *aaf;
	struct avtp_aaf_param param;
	int payload_size;
	int ret;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d rate=%d channels=%d samples_per_frame=%d\n",
		  index, config->sample_rate, config->channels,
		  config->samples_per_frame);
	aaf = &aaf_packetizer_table[index];
	aaf->audio_config = *config;

	ret = check_packet_format(index);
	if (ret < 0)
		return ret;

	aaf->avtp_format = get_aaf_format(aaf->audio_config.sample_bit_depth);
	aaf->avtp_bytes_per_ch = get_aaf_format_size(aaf->avtp_format);
	aaf->shift = get_bit_shift(aaf->audio_config.sample_bit_depth);

	/* when samples_per_frame is not set */
	if (!aaf->audio_config.samples_per_frame) {
		aaf->class_interval_frames = DEFAULT_INTERVAL_FRAMES;
		aaf->sample_per_packet = DIV_ROUND_UP(
			aaf->audio_config.sample_rate,
			aaf->class_interval_frames);
	} else {
		aaf->sample_per_packet = aaf->audio_config.samples_per_frame;
		aaf->class_interval_frames = DIV_ROUND_UP(
			aaf->audio_config.sample_rate,
			aaf->sample_per_packet);
	}
	aaf->frame_interval_time = NSEC_SCALE / aaf->class_interval_frames;

	payload_size = aaf->sample_per_packet * aaf->audio_config.channels *
						aaf->avtp_bytes_per_ch;
	aaf->avtp_packet_size = AVTP_AAF_PAYLOAD_OFFSET + payload_size;
	if (aaf->avtp_packet_size < ETHFRAMELEN_MIN)
		aaf->avtp_packet_size = ETHFRAMELEN_MIN;
	if (aaf->avtp_packet_size > ETHFRAMELEN_MAX) {
		mse_err("insufficient payload size\n");
		return -EPERM;
	}

	memcpy(param.dest_addr, aaf->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, aaf->net_config.source_addr,
	       MSE_MAC_LEN_MAX);
	param.uniqueid = aaf->net_config.uniqueid;
	param.priority = aaf->net_config.priority;
	param.vid = aaf->net_config.vlanid;
	param.samples_per_frame = aaf->sample_per_packet;
	param.channels = aaf->audio_config.channels;
	param.sample_rate = aaf->audio_config.sample_rate;

	mse_packetizer_aaf_header_build(aaf->packet_template, &param);

	return 0;
}

static int mse_packetizer_aaf_get_audio_info(int index,
					     struct mse_audio_info *info)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d\n", index);
	aaf = &aaf_packetizer_table[index];

	info->avtp_packet_size = aaf->avtp_packet_size;
	info->sample_per_packet = aaf->sample_per_packet;
	info->frame_interval_time = aaf->frame_interval_time;

	return 0;
}

static int mse_packetizer_aaf_calc_cbs(int index,
				       struct mse_cbsparam *cbs)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d\n", index);
	aaf = &aaf_packetizer_table[index];

	return mse_packetizer_calc_cbs_by_frames(
			aaf->net_config.port_transmit_rate,
			aaf->avtp_packet_size,
			aaf->class_interval_frames,
			CBS_ADJUSTMENT_FACTOR,
			cbs);
}

static void copy_bit_to_payload(unsigned char *dest, int dest_type,
				unsigned char *src, int src_byte, int shift,
				bool big_endian)
{
	unsigned int value;
	int i;

	if (big_endian) {
		for (i = 0, value = 0; i < src_byte ; i++) {
			value <<= 8;
			value |= src[i];
		}
	} else {
		for (i = 0, value = 0; i < src_byte ; i++)
			value |= src[i] << (8 * i);
	}

	value <<= shift;
	switch (dest_type) {
	case AVTP_AAF_FORMAT_INT_16BIT:
		dest[0] = (value & 0xFF00) >> 8;
		dest[1] = (value & 0x00FF);
		break;
	case AVTP_AAF_FORMAT_INT_24BIT:
		dest[0] = (value & 0xFF0000) >> 16;
		dest[1] = (value & 0x00FF00) >> 8;
		dest[2] = (value & 0x0000FF);
		break;
	case AVTP_AAF_FORMAT_INT_32BIT:
		dest[0] = (value & 0xFF000000) >> 24;
		dest[1] = (value & 0x00FF0000) >> 16;
		dest[2] = (value & 0x0000FF00) >> 8;
		dest[3] = (value & 0x000000FF);
		break;
	default:
		break;
	}
}

static void copy_payload(unsigned char *payload,
			 int *payload_stored,
			 unsigned char *buffer,
			 int *buffer_stored,
			 struct aaf_packetizer *aaf,
			 int count)
{
	int i;

	*payload_stored = 0;
	*buffer_stored = 0;

	for (i = 0; i < count; i++) {
		copy_bit_to_payload(payload,
				    aaf->avtp_format,
				    buffer,
				    aaf->audio_config.bytes_per_sample,
				    aaf->shift,
				    aaf->audio_config.is_big_endian);

		payload += aaf->avtp_bytes_per_ch;
		buffer += aaf->audio_config.bytes_per_sample;
		*payload_stored += aaf->avtp_bytes_per_ch;
		*buffer_stored += aaf->audio_config.bytes_per_sample;
	}
}

static void copy_bit_to_buffer(unsigned char *dest, int dest_byte,
			       unsigned char *src, int src_byte, int shift,
			       bool big_endian)
{
	unsigned int value;

	switch (src_byte) {
	case 2:
		value = src[0] << 8 | src[1];
		break;
	case 3:
		value = src[0] << 16 | src[1] << 8 | src[2];
		break;
	case 4:
		value = src[0] << 24 | src[1] << 16 | src[2] << 8 | src[3];
		break;
	default:
		break;
	}

	if (shift > 0)
		value >>= shift;
	else
		value <<= -shift;

	/* sign extend only S24_{LE,BE} */
	if (dest_byte == 4 && src_byte == 3) {
		if (value & 0x800000)
			value |= 0xFF000000;
	}

	if (big_endian) {
		value = htonl(value);
		memcpy(dest, ((unsigned char *)&value) + 4 - dest_byte,
		       dest_byte);
	} else {
		memcpy(dest, &value, dest_byte);
	}
}

static void copy_buffer(unsigned char *buffer,
			int *buffer_stored,
			unsigned char *payload,
			int aaf_byte_per_ch,
			struct aaf_packetizer *aaf,
			int count)
{
	int i;
	int shift;

	*buffer_stored = 0;
	if (aaf->audio_config.bytes_per_sample == 4 &&
	    aaf->audio_config.sample_bit_depth == MSE_AUDIO_BIT_24)  {
		shift = (aaf_byte_per_ch - 3) * 8 + aaf->shift;
	} else {
		shift = (aaf_byte_per_ch - aaf->audio_config.bytes_per_sample)
			* 8 + aaf->shift;
	}

	for (i = 0; i < count; i++) {
		copy_bit_to_buffer(buffer,
				   aaf->audio_config.bytes_per_sample,
				   payload,
				   aaf_byte_per_ch,
				   shift,
				   aaf->audio_config.is_big_endian);
		buffer += aaf->audio_config.bytes_per_sample;
		*buffer_stored += aaf->audio_config.bytes_per_sample;
		payload += aaf_byte_per_ch;
	}
}

static int mse_packetizer_aaf_packetize(int index,
					void *packet,
					size_t *packet_size,
					void *buffer,
					size_t buffer_size,
					size_t *buffer_processed,
					unsigned int *timestamp)
{
	struct aaf_packetizer *aaf;
	int data_len, data_size;
	unsigned char  *payload, *data;
	int piece_count = 0, piece_len = 0;
	int count, dest_byte, readed_byte;
	struct mse_audio_config *config;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];
	config = &aaf->audio_config;
	mse_debug("index=%d seqnum=%d process=%zu/%zu t=%d\n",
		  index, aaf->send_seq_num, *buffer_processed,
		  buffer_size, *timestamp);

	/* header */
	if (aaf->piece_f) {
		piece_len = aaf->piece_data_len;
		piece_count = piece_len / aaf->avtp_bytes_per_ch;
		aaf->piece_f = false;
		aaf->piece_data_len = 0;
		memcpy(packet, aaf->packet_piece, aaf->avtp_packet_size);
	} else {
		memcpy(packet, aaf->packet_template, aaf->avtp_packet_size);
	}

	payload = (unsigned char *)
		(packet + AVTP_AAF_PAYLOAD_OFFSET + piece_len);
	data = (unsigned char *)(buffer + *buffer_processed);

	/* size check */
	data_size = aaf->sample_per_packet * config->channels;
	data_len = (data_size - piece_count) * config->bytes_per_sample;
	if (data_len > buffer_size - *buffer_processed) {
		aaf->piece_f = true;
		data_len = buffer_size - *buffer_processed;
		count = data_len / config->bytes_per_sample;
		*packet_size = AVTP_AAF_PAYLOAD_OFFSET +
			count * config->bytes_per_sample;
	} else {
		*packet_size = aaf->avtp_packet_size;
		count = aaf->sample_per_packet * config->channels -
			piece_count;
	}

	copy_payload(payload, &dest_byte, data, &readed_byte,
		     aaf, count);
	*buffer_processed += readed_byte;

	/* keep piece of data */
	if (aaf->piece_f) {
		aaf->piece_data_len = dest_byte;
		memcpy(aaf->packet_piece, packet, *packet_size);
		return MSE_PACKETIZE_STATUS_NOT_ENOUGH;
	}

	/* variable header */
	avtp_set_sequence_num(packet, aaf->send_seq_num++);
	avtp_set_timestamp(packet, (u32)*timestamp);
	avtp_set_stream_data_length(packet,
				    data_size * aaf->avtp_bytes_per_ch);
	avtp_set_aaf_format(packet, aaf->avtp_format);
	avtp_set_aaf_bit_depth(packet,
			       mse_get_bit_depth(config->sample_bit_depth));

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return MSE_PACKETIZE_STATUS_COMPLETE;
	else
		return MSE_PACKETIZE_STATUS_CONTINUE;
}

static int mse_packetizer_aaf_depacketize(int index,
					  void *buffer,
					  size_t buffer_size,
					  size_t *buffer_processed,
					  unsigned int *timestamp,
					  void *packet,
					  size_t packet_size)
{
	struct aaf_packetizer *aaf;
	int payload_size, piece_size = 0;
	u32 offset;
	unsigned char *buf, tmp_buffer[ETHFRAMEMTU_MAX] = {0};
	int aaf_format;
	int aaf_bit_depth;
	int aaf_byte_per_ch;
	int aaf_sample_rate;
	int channels;
	int count, stored;
	int ret;
	bool tv;
	u32 avtp_timestamp;

	if (index >= ARRAY_SIZE(aaf_packetizer_table))
		return -EPERM;

	mse_debug("index=%d\n", index);
	aaf = &aaf_packetizer_table[index];

	if (avtp_get_subtype(packet) != AVTP_SUBTYPE_AAF) {
		mse_err("error subtype=%d\n", avtp_get_subtype(packet));
		return -EINVAL;
	}

	if (aaf->piece_f) {
		aaf->piece_f = false;
		memcpy(buffer + *buffer_processed,
		       aaf->packet_piece, aaf->piece_data_len);
		*buffer_processed += aaf->piece_data_len;
		aaf->piece_data_len = 0;
	}

	payload_size = avtp_get_stream_data_length(packet);
	channels = avtp_get_aaf_channels_per_frame(packet);
	aaf_format = avtp_get_aaf_format(packet);
	aaf_bit_depth = avtp_get_aaf_bit_depth(packet);
	aaf_sample_rate = avtp_aaf_nsr_to_sample_rate(
		avtp_get_aaf_nsr(packet));
	aaf_byte_per_ch = get_aaf_format_size(aaf_format);
	if (!aaf_byte_per_ch) {
		mse_err("packet's format=%d can not accecpted\n", aaf_format);
		return -EINVAL;
	}

	ret = check_receive_packet(index, channels, aaf_sample_rate,
				   aaf_bit_depth);
	if (ret < 0)
		return ret;

	aaf->sample_per_packet = payload_size / (channels * aaf_byte_per_ch);
	aaf->frame_interval_time = div_u64(NSEC_SCALE * aaf->sample_per_packet,
					   aaf_sample_rate);

	tv = avtp_get_tv(packet);
	avtp_timestamp = avtp_get_timestamp(packet);

	/* check timestamp valid, if false interpolate value */
	if (tv) {
		aaf->has_valid_avtp_timestamp = true;
	} else if (aaf->has_valid_avtp_timestamp) {
		avtp_timestamp = aaf->last_avtp_timestamp +
			aaf->frame_interval_time;
	} else if (aaf->f_need_calc_offset) {
		return MSE_PACKETIZE_STATUS_DISCARD;
	}
	aaf->last_avtp_timestamp = avtp_timestamp;

	if (aaf->f_need_calc_offset) {
		ret = mse_packetizer_calc_audio_offset(
			avtp_timestamp,
			aaf->start_time,
			aaf_sample_rate,
			aaf->audio_config.bytes_per_sample,
			channels,
			buffer_size,
			&offset);
		if (ret == MSE_PACKETIZE_STATUS_SKIP) {
			*buffer_processed = buffer_size;

			return MSE_PACKETIZE_STATUS_SKIP;
		} else if (ret == MSE_PACKETIZE_STATUS_DISCARD) {
			return MSE_PACKETIZE_STATUS_DISCARD;
		}

		aaf->f_need_calc_offset = false;
		*buffer_processed += offset;
	}

	/* buffer over check */
	count = payload_size / aaf_byte_per_ch;
	if (*buffer_processed + count * aaf->audio_config.bytes_per_sample >
	    buffer_size)
		buf = tmp_buffer;
	else
		buf = buffer + *buffer_processed;

	/* seq_num check */
	mse_packetizer_stats_seqnum(&aaf->stats, avtp_get_sequence_num(packet));

	copy_buffer(buf, &stored,
		    packet + AVTP_AAF_PAYLOAD_OFFSET,
		    aaf_byte_per_ch, aaf,
		    count);

	if (*buffer_processed + count * aaf->audio_config.bytes_per_sample >
	    buffer_size) {
		aaf->piece_f = true;
		piece_size = buffer_size - *buffer_processed;
		aaf->piece_data_len = stored - piece_size;
		memcpy(buffer + *buffer_processed, buf, piece_size);
		memcpy(aaf->packet_piece, buf + piece_size,
		       aaf->piece_data_len);
		mse_debug("piece %d - %02x %02x %02x %02x\n",
			  aaf->piece_data_len,
			  aaf->packet_piece[0], aaf->packet_piece[1],
			  aaf->packet_piece[2], aaf->packet_piece[3]);
	}

	if (*buffer_processed + stored > buffer_size)
		*buffer_processed = buffer_size;
	else
		*buffer_processed += stored;

	*timestamp = avtp_timestamp;

	/* buffer over check */
	if (*buffer_processed >= buffer_size)
		return MSE_PACKETIZE_STATUS_COMPLETE;

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

static int mse_packetizer_aaf_set_start_time(int index, u32 start_time)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];
	aaf->start_time = start_time;

	return 0;
}

static int mse_packetizer_aaf_set_need_calc_offset(int index)
{
	struct aaf_packetizer *aaf;

	if (index >= ARRAY_SIZE(aaf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	aaf = &aaf_packetizer_table[index];
	aaf->f_need_calc_offset = true;
	aaf->has_valid_avtp_timestamp = false;
	aaf->last_avtp_timestamp = 0;

	return 0;
}

struct mse_packetizer_ops mse_packetizer_aaf_ops = {
	.open = mse_packetizer_aaf_open,
	.release = mse_packetizer_aaf_release,
	.init = mse_packetizer_aaf_packet_init,
	.set_network_config = mse_packetizer_aaf_set_network_config,
	.set_audio_config = mse_packetizer_aaf_set_audio_config,
	.get_audio_info = mse_packetizer_aaf_get_audio_info,
	.set_start_time = mse_packetizer_aaf_set_start_time,
	.set_need_calc_offset = mse_packetizer_aaf_set_need_calc_offset,
	.calc_cbs = mse_packetizer_aaf_calc_cbs,
	.packetize = mse_packetizer_aaf_packetize,
	.depacketize = mse_packetizer_aaf_depacketize,
};
