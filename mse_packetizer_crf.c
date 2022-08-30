/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016-2018,2021 Renesas Electronics Corporation

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

#define MSE_CRFDATA_MAX         (6)

struct avtp_crf_param {
	char dest_addr[MSE_MAC_LEN_MAX];
	char source_addr[MSE_MAC_LEN_MAX];
	int uniqueid;
	int priority;
	int vid;
	int base_frequency;
	int timestamp_interval;
};

struct crf_packetizer {
	bool used_f;
	int send_seq_num;
	unsigned char packet_template[ETHFRAMELEN_MAX];

	int crf_packet_size;
	int frame_interval_time;

	struct mse_network_config net_config;
	struct mse_audio_config   crf_audio_config;
};

struct crf_packetizer crf_packetizer_table[MSE_INSTANCE_MAX];

static int mse_packetizer_crf_audio_open(void)
{
	struct crf_packetizer *crf;
	int index;

	for (index = 0; crf_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(crf_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	crf->used_f = true;
	crf->send_seq_num = 0;

	return index;
}

static int mse_packetizer_crf_audio_release(int index)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	memset(crf, 0, sizeof(*crf));

	return 0;
}

static int mse_packetizer_crf_audio_packet_init(int index)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	crf->send_seq_num = 0;

	return 0;
}

static int mse_packetizer_crf_audio_set_network_config(
	int index,
	struct mse_network_config *config)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	if (!config)
		return -EPERM;

	crf = &crf_packetizer_table[index];
	crf->net_config = *config;

	return 0;
}

static int mse_packetizer_crf_audio_header_build(void *dst,
						 struct avtp_crf_param *param)
{
	u8 dei;
	u8 streamid[AVTP_STREAMID_SIZE];

	memset(dst, 0, ETHFRAMELEN_MAX);   /* clear MAC frame buffer */

	/* Ethernet frame header */
	set_ieee8021q_dest(dst, (u8 *)param->dest_addr);
	set_ieee8021q_source(dst, (u8 *)param->source_addr);

	/* IEEE802.1Q Q-tag */
	dei = 0;
	set_ieee8021q_tpid(dst, ETH_P_8021Q);

	/* pcp:3bit, dei:1bit, vid:12bit */
	set_ieee8021q_tci(dst,
			  (param->priority << 13) | (dei << 12) | param->vid);
	set_ieee8021q_ethtype(dst, ETH_P_1722);

	/* 1722 header update + payload */
	avtp_make_streamid(streamid, param->source_addr, param->uniqueid);

	avtp_copy_crf_template(dst);

	avtp_set_stream_id(dst, streamid);

	avtp_set_crf_base_frequency(dst, (param->base_frequency & 0x1FFFFFFF));
	avtp_set_crf_timestamp_interval(dst, param->timestamp_interval);
	avtp_set_crf_data_length(dst, 0);

	return AVTP_CRF_PAYLOAD_OFFSET;
}

static int mse_packetizer_crf_audio_set_audio_config(
	int index,
	struct mse_audio_config *config)
{
	struct crf_packetizer *crf;
	struct avtp_crf_param param;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	if (!config)
		return -EPERM;

	crf = &crf_packetizer_table[index];
	crf->crf_audio_config = *config;

	crf->crf_packet_size = AVTP_CRF_PAYLOAD_OFFSET +
			       (sizeof(u64) * MSE_CRFDATA_MAX);

	memcpy(param.dest_addr, crf->net_config.dest_addr, MSE_MAC_LEN_MAX);
	memcpy(param.source_addr, crf->net_config.source_addr, MSE_MAC_LEN_MAX);

	param.uniqueid              = crf->net_config.uniqueid;
	param.priority              = crf->net_config.priority;
	param.vid                   = crf->net_config.vlanid;
	param.base_frequency        = crf->crf_audio_config.sample_rate;
	param.timestamp_interval    = crf->crf_audio_config.samples_per_frame;

	mse_packetizer_crf_audio_header_build(crf->packet_template, &param);

	return 0;
}

static int mse_packetizer_crf_audio_get_audio_info(
	int index,
	struct mse_audio_info *info)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	info->frame_interval_time = crf->frame_interval_time;

	return 0;
}

static int mse_packetizer_crf_audio_calc_cbs(int index,
					     struct mse_cbsparam *cbs)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	mse_debug("index=%d\n", index);
	crf = &crf_packetizer_table[index];

	return mse_packetizer_calc_cbs_by_frames(
			crf->net_config.port_transmit_rate,
			crf->crf_packet_size,
			CRF_INTERVAL_FRAMES,
			CBS_ADJUSTMENT_FACTOR,
			cbs);
}

static int mse_packetizer_crf_audio_packetize(int index,
					      void *packet,
					      size_t *packet_size,
					      void *buffer,
					      size_t buffer_size,
					      size_t *buffer_processed,
					      unsigned int *timestamp)
{
	struct crf_packetizer *crf;
	u64 *ptptimes;
	u64 *sample;
	int i, data_len;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	memcpy(packet, crf->packet_template, AVTP_CRF_PAYLOAD_OFFSET);
	sample = (u64 *)(packet + AVTP_CRF_PAYLOAD_OFFSET);
	ptptimes = buffer;

	data_len = 0;
	for (i = 0; i < buffer_size / sizeof(*ptptimes); i++) {
		*sample++ = cpu_to_be64(ptptimes[i]);
		data_len += sizeof(*sample);
	}

	/* variable header */
	avtp_set_sequence_num(packet, crf->send_seq_num++);
	avtp_set_crf_data_length(packet, data_len);
	*packet_size = (size_t)data_len + AVTP_CRF_PAYLOAD_OFFSET;

	return MSE_PACKETIZE_STATUS_COMPLETE;
}

static int mse_packetizer_crf_audio_depacketize(int index,
						void *buffer,
						size_t buffer_size,
						size_t *buffer_processed,
						unsigned int *timestamp,
						void *packet,
						size_t packet_size)
{
	u64 *crf_data, *dest;
	int size, i;
	unsigned long value;
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table)) {
		mse_err("wrong index: %d\n", index);
		return -EPERM;
	}

	crf = &crf_packetizer_table[index];

	size = avtp_get_crf_data_length(packet);

	if (size > buffer_size) {
		mse_err("packet too small, size=%d buffer_size=%lu\n", size, buffer_size);
		return -ENOMEM;
	}

	value = NSEC_SCALE * avtp_get_crf_timestamp_interval(packet);
	do_div(value, avtp_get_crf_base_frequency(packet));
	crf->frame_interval_time = value;
	crf_data = (u64 *)((char *)packet + AVTP_CRF_PAYLOAD_OFFSET);
	dest = buffer;

	for (i = 0; i < size / sizeof(u64); i++)
		*(dest + i) = be64_to_cpu(*(crf_data + i));
	*buffer_processed = size;

	return MSE_PACKETIZE_STATUS_COMPLETE;
}

struct mse_packetizer_ops mse_packetizer_crf_timestamp_audio_ops = {
	.open = mse_packetizer_crf_audio_open,
	.release = mse_packetizer_crf_audio_release,
	.init = mse_packetizer_crf_audio_packet_init,
	.set_network_config = mse_packetizer_crf_audio_set_network_config,
	.set_audio_config = mse_packetizer_crf_audio_set_audio_config,
	.get_audio_info = mse_packetizer_crf_audio_get_audio_info,
	.calc_cbs = mse_packetizer_crf_audio_calc_cbs,
	.packetize = mse_packetizer_crf_audio_packetize,
	.depacketize = mse_packetizer_crf_audio_depacketize,
};
