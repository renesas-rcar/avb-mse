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
#include <linux/ptp_clock.h>

#include "mse_core.h"
#include "mse_packetizer.h"
#include "avtp.h"

#define SEQNUM_INIT             (-1)

#define MSE_PACKETIZER_MAX      (10)

#define NSEC_SCALE              (1000000000ul)
#define BYTE_TO_BIT             (8)

#define CBS_ADJUSTMENT_NUMERATOR        (103)
#define CBS_ADJUSTMENT_DENOMINATOR      (100)
/* preamble + FCS + IGP */
#define ETHERNET_SPECIAL        (8 + 4 + 12)

#define PORT_TRANSMIT_RATE      (100000000) /* 100M [bit/sec] */
#define CLASS_INTERVAL_FRAMES   (8000) /* class A */
#define INTERVAL_FRAMES         (1)

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

struct crf_packetizer crf_packetizer_table[MSE_PACKETIZER_MAX];

static int class_interval_frames;

static int mse_packetizer_crf_audio_open(void)
{
	struct crf_packetizer *crf;
	int index;

	for (index = 0; crf_packetizer_table[index].used_f &&
	     index < ARRAY_SIZE(crf_packetizer_table); index++)
		;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

	crf = &crf_packetizer_table[index];

	crf->used_f = true;
	crf->send_seq_num = 0;

	return index;
}

static int mse_packetizer_crf_audio_release(int index)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

	crf = &crf_packetizer_table[index];

	memset(crf, 0, sizeof(*crf));

	return 0;
}

static int mse_packetizer_crf_audio_packet_init(int index)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

	crf = &crf_packetizer_table[index];

	crf->send_seq_num = 0;

	return 0;
}

static int mse_packetizer_crf_audio_set_network_config(
	int index,
	struct mse_network_config *config)
{
	struct crf_packetizer *crf;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

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
	mse_make_streamid(streamid, param->source_addr, param->uniqueid);

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

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

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
	param.timestamp_interval    = crf->crf_audio_config.bytes_per_frame;

	mse_packetizer_crf_audio_header_build(crf->packet_template, &param);

	return 0;
}

static int mse_packetizer_crf_audio_get_audio_info(
	int index,
	struct mse_audio_info *info)
{
	struct crf_packetizer *crf;

	crf = &crf_packetizer_table[index];

	info->frame_interval_time = crf->frame_interval_time;
	return 0;
}

static int mse_packetizer_crf_audio_calc_cbs(int index,
					     struct eavb_cbsparam *cbs)
{
	struct crf_packetizer *crf;
	u64 value;
	u64 bandwidth_fraction_denominator, bandwidth_fraction_numerator;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);
	crf = &crf_packetizer_table[index];

	bandwidth_fraction_denominator = (u64)PORT_TRANSMIT_RATE *
					 (u64)CBS_ADJUSTMENT_DENOMINATOR;
	if (!bandwidth_fraction_denominator) {
		pr_err("[%s] cbs error(null)\n", __func__);
		return -EPERM;
	}

	bandwidth_fraction_numerator =
		(ETHERNET_SPECIAL + crf->crf_packet_size) * BYTE_TO_BIT *
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

	value = USHRT_MAX * (bandwidth_fraction_denominator
					 - bandwidth_fraction_numerator);
	/* divide denominator into 2 */
	do_div(value, PORT_TRANSMIT_RATE);
	do_div(value, CBS_ADJUSTMENT_DENOMINATOR);
	cbs->idleSlope = value;

	return 0;
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
	struct ptp_clock_time *ptptimes;
	u64 *sample;
	int i, data_len;

	if (index >= ARRAY_SIZE(crf_packetizer_table))
		return -EPERM;

	crf = &crf_packetizer_table[index];

	memcpy(packet, crf->packet_template, AVTP_CRF_PAYLOAD_OFFSET);
	sample = (u64 *)(packet + AVTP_CRF_PAYLOAD_OFFSET);
	ptptimes = (struct ptp_clock_time *)buffer;

	data_len = 0;
	for (i = 0; i < buffer_size / sizeof(struct ptp_clock_time); i++) {
		*sample++ = cpu_to_be64(ptptimes[i].sec * NSEC_SCALE +
			ptptimes[i].nsec);
		data_len += sizeof(u64);
	}

	/* variable header */
	avtp_set_sequence_num(packet, crf->send_seq_num++);
	avtp_set_crf_data_length(packet, data_len);
	*packet_size = (size_t)data_len + AVTP_CRF_PAYLOAD_OFFSET;

	return 0;
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

	crf = &crf_packetizer_table[index];

	size = avtp_get_crf_data_length(packet);

	if (size > buffer_size) {
		pr_err("[%s] error\n", __func__);
		return -ENOMEM;
	}

	value = NSEC_SCALE * avtp_get_crf_timestamp_interval(packet);
	do_div(value, avtp_get_crf_base_frequency(packet));
	crf->frame_interval_time = value;
	crf_data = (u64 *)((char *)packet + AVTP_CRF_PAYLOAD_OFFSET);
	dest = buffer;

	for (i = 0; i < size / sizeof(u64); i++)
		*dest++ = be64_to_cpu(*crf_data++);
	/* *buffer_processed = size;*/

	return size; /* continue */
}

struct mse_packetizer_ops mse_packetizer_crf_tstamp_audio_ops = {
	.name = MSE_PACKETIZER_NAME_STR_CRF_TIMESTAMP,
	.priv = NULL,
	.type = MSE_TYPE_PACKETIZER_CTRL_CRF,
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
