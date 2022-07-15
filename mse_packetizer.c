/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2018 Renesas Electronics Corporation

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

#include <linux/module.h>
#include <linux/kernel.h>

#include "ravb_mse_kernel.h"
#include "mse_packetizer.h"
#include "avtp.h"

/* preamble (8) + FCS (4) */
#define ETHERNET_OVERHEAD             (8 + 4)
#define CBS_ADJUST_RATIO_BASE_PERCENT (100)
#define BIT_TO_BYTE                   (8)
#define TRANSMIT_RATE_BASE            (1000)

/* MSE packetizer function table */
struct mse_packetizer_ops_table {
	/** @brief stream type of packetizer */
	enum MSE_STREAM_TYPE type;
	/** @brief function array of packetizer */
	struct mse_packetizer_ops *ops;
};

DEFINE_SPINLOCK(packetizer_lock);

static const struct mse_packetizer_ops_table
packetizer_table[MSE_PACKETIZER_MAX] = {
#if defined(CONFIG_MSE_PACKETIZER_AAF)
	[MSE_PACKETIZER_AAF_PCM] = {
		MSE_STREAM_TYPE_AUDIO,
		&mse_packetizer_aaf_ops
	},
#endif
#if defined(CONFIG_MSE_PACKETIZER_IEC61883_6)
	[MSE_PACKETIZER_IEC61883_6] = {
		MSE_STREAM_TYPE_AUDIO,
		&mse_packetizer_iec61883_6_ops
	},
#endif
#if defined(CONFIG_MSE_PACKETIZER_CVF_H264)
	[MSE_PACKETIZER_CVF_H264] = {
		MSE_STREAM_TYPE_VIDEO,
		&mse_packetizer_cvf_h264_ops
	},
	[MSE_PACKETIZER_CVF_H264_D13] = {
		MSE_STREAM_TYPE_VIDEO,
		&mse_packetizer_cvf_h264_d13_ops
	},
#endif
#if defined(CONFIG_MSE_PACKETIZER_CVF_MJPEG)
	[MSE_PACKETIZER_CVF_MJPEG] = {
		MSE_STREAM_TYPE_VIDEO,
		&mse_packetizer_cvf_mjpeg_ops
	},
#endif
#if defined(CONFIG_MSE_PACKETIZER_IEC61883_4)
	[MSE_PACKETIZER_IEC61883_4] = {
		MSE_STREAM_TYPE_MPEG2TS,
		&mse_packetizer_iec61883_4_ops
	},
#endif
};

enum MSE_STREAM_TYPE mse_packetizer_get_type(enum MSE_PACKETIZER id)
{
	enum MSE_STREAM_TYPE type = -1;

	if (id < MSE_PACKETIZER_MAX)
		type = packetizer_table[id].type;
	else
		mse_err("failed to get type, id=%d\n", id);

	return type;
}

struct mse_packetizer_ops *mse_packetizer_get_ops(enum MSE_PACKETIZER id)
{
	struct mse_packetizer_ops *ops = NULL;

	if (id < MSE_PACKETIZER_MAX)
		ops = packetizer_table[id].ops;
	else
		mse_err("failed to get ops, id=%d\n", id);

	return ops;
}

bool mse_packetizer_is_valid(enum MSE_PACKETIZER id)
{
	return mse_packetizer_get_ops(id) ? true : false;
}

int mse_packetizer_calc_cbs(u64 bw_num,
			    u64 bw_denom,
			    struct mse_cbsparam *cbs)
{
	u64 bw_frac;

	mse_debug("bw_frac = %llu/%llu (0x%016llx/0x%016llx)\n",
		  bw_num, bw_denom, bw_num, bw_denom);

	if (!bw_denom) {
		mse_err("error zero divide\n");
		return -EINVAL;
	}

	if (bw_num > UINT_MAX) {
		mse_err("error bw numerator overflow\n");
		return -EPERM;
	}

	if (bw_num >= bw_denom) {
		mse_err("error insufficient bandwidth, (bw_num >= bw_denom)\n");
		return -ENOSPC;
	}

	bw_frac = div64_u64((u64)UINT_MAX * bw_num, bw_denom);
	if (bw_frac > (UINT_MAX - (1 << 15))) {
		mse_err("error insufficient bandwidth, (bw_frac > 1.0)\n");
		return -ENOSPC;
	}

	bw_frac += 1 << 15;
	cbs->bandwidth_fraction = (u32)bw_frac;
	cbs->idle_slope = bw_frac >> 16;
	cbs->send_slope = (USHRT_MAX - cbs->idle_slope) * -1;

	mse_debug("bw_frac = %llu (0x%016llx)\n", bw_frac, bw_frac);

	return 0;
}

int mse_packetizer_calc_cbs_by_frames(u32 port_transmit_rate,
				      u32 avtp_packet_size,
				      u32 class_interval_frames,
				      u32 cbs_adjust_ratio_percent,
				      struct mse_cbsparam *cbs)
{
	u32 ether_size;
	u64 bw_num, bw_denom;

	/*
	 * (1)
	 *                           bandwidth
	 * bandwidth_fraction = --------------------
	 *                       port_transmit_rate
	 *
	 * (2)
	 * ether_size = ether_overhead + avtp_packet_size
	 *
	 * (3)
	 * bandwidth = ether_size x 8 x class_interval_frames
	 *
	 * (4)
	 *                     cbs_adjust_ratio_percent
	 * cbs_adjust_ratio = --------------------------
	 *                               100
	 *
	 * (1')
	 *                            bandwidth
	 * bandwidth fraction = -------------------- x cbs_adjust_ratio
	 *                       port_transmit_rate
	 */
	ether_size = ETHERNET_OVERHEAD + avtp_packet_size;
	bw_num = (u64)ether_size * (u64)class_interval_frames *
		(u64)cbs_adjust_ratio_percent;
	bw_denom = (u64)(port_transmit_rate / BIT_TO_BYTE) *
		CBS_ADJUST_RATIO_BASE_PERCENT;

	mse_debug("%u %u %u %u\n",
		  port_transmit_rate, avtp_packet_size,
		  class_interval_frames, cbs_adjust_ratio_percent);

	return mse_packetizer_calc_cbs(bw_num, bw_denom, cbs);
}

int mse_packetizer_calc_cbs_by_bitrate(u32 port_transmit_rate,
				       u32 ether_size,
				       u32 payload_bitrate,
				       u32 payload_size,
				       struct mse_cbsparam *cbs)
{
	u64 bw_num, bw_denom;

	/*
	 * (1)
	 *                           bandwidth
	 * bandwidth_fraction = --------------------
	 *                       port_transmit_rate
	 *
	 * (2)
	 *   ether_size         bandwidth
	 * -------------- = -----------------
	 *  payload_size     payload_bitrate
	 *
	 * (2')
	 *                        payload_bitrate        ether_size
	 * bandwidth_fraction = -------------------- x --------------
	 *                       port_transmit_rate     payload_size
	 *
	 */
	bw_num = (u64)(payload_bitrate / TRANSMIT_RATE_BASE) *
		(u64)(ether_size + ETHERNET_OVERHEAD);
	bw_denom = (u64)(port_transmit_rate / TRANSMIT_RATE_BASE) *
		(u64)payload_size;

	mse_debug("%u %u %u %u\n",
		  payload_bitrate, port_transmit_rate,
		  payload_size, ether_size);

	return mse_packetizer_calc_cbs(bw_num, bw_denom, cbs);
}

int mse_packetizer_calc_audio_offset(
	u32 avtp_timestamp,
	u32 start_time,
	int sample_rate,
	int sample_byte,
	int channels,
	size_t buffer_size,
	u32 *offset)
{
	u64 diff;
	u32 sample_offset;
	u64 calc_time, period_time, sample_time;
	int byte_per_ch;

	byte_per_ch = sample_byte * channels;

	/*  Calculate offset by avtp timestamp on first packet */
	diff = avtp_timestamp - start_time;

	sample_offset = div64_u64(diff * sample_rate, NSEC_SCALE) * byte_per_ch;

	if (diff >= BIT(31))
		return MSE_PACKETIZE_STATUS_DISCARD;

	if (sample_offset > buffer_size)
		return MSE_PACKETIZE_STATUS_SKIP;

	calc_time = div64_u64((u64)(sample_offset / byte_per_ch) * NSEC_SCALE,
			      sample_rate);
	period_time = div64_u64((u64)(buffer_size / byte_per_ch) * NSEC_SCALE,
				sample_rate);
	sample_time = NSEC_SCALE / sample_rate;

	mse_info("start %u avtp %u calc %u offset %d (i=%lld) diff=%d(+%u)\n",
		 start_time,
		 avtp_timestamp,
		 start_time + (u32)calc_time + (u32)period_time,
		 sample_offset,
		 sample_time,
		 (s32)(start_time + calc_time - avtp_timestamp),
		 (u32)period_time);

	*offset = sample_offset;

	return MSE_PACKETIZE_STATUS_CONTINUE;
}

void mse_packetizer_stats_init(struct mse_packetizer_stats *stats)
{
	stats->seq_num_next = SEQNUM_INIT;
	stats->seq_num_err = SEQNUM_INIT;
	stats->seq_num_err_total = 0;
}

int mse_packetizer_stats_seqnum(struct mse_packetizer_stats *stats, u8 seq_num)
{
	int ret = 0;

	if (stats->seq_num_next != seq_num &&
	    stats->seq_num_next != SEQNUM_INIT) {
		if (stats->seq_num_err == SEQNUM_INIT) {
			mse_debug("sequence number discontinuity %u->%u=%u\n",
				  stats->seq_num_next,
				  seq_num,
				  (seq_num + 1 + AVTP_SEQUENCE_NUM_MAX -
				   stats->seq_num_next) %
				  (AVTP_SEQUENCE_NUM_MAX + 1));
			stats->seq_num_err = 1;
		} else {
			stats->seq_num_err++;
		}
		stats->seq_num_err_total++;
		ret = 1;
	} else {
		if (stats->seq_num_err != SEQNUM_INIT) {
			mse_debug("sequence number recovery %u count=%u\n",
				  seq_num, stats->seq_num_err);
			stats->seq_num_err = SEQNUM_INIT;
		}
	}

	stats->seq_num_next = (seq_num + 1 + (AVTP_SEQUENCE_NUM_MAX + 1)) %
			(AVTP_SEQUENCE_NUM_MAX + 1);

	return ret;
}

void mse_packetizer_stats_report(struct mse_packetizer_stats *stats)
{
	if (stats->seq_num_err_total)
		mse_err("sequence number discontinuity total=%llu\n",
			stats->seq_num_err_total);
}

int mse_packetizer_open(enum MSE_PACKETIZER id)
{
	struct mse_packetizer_ops *ops;
	int ret;
	unsigned long flags;

	ops = mse_packetizer_get_ops(id);
	if (!ops)
		return -EPERM;

	spin_lock_irqsave(&packetizer_lock, flags);
	ret = ops->open();
	spin_unlock_irqrestore(&packetizer_lock, flags);

	return ret;
}

int mse_packetizer_release(enum MSE_PACKETIZER id, int index)
{
	struct mse_packetizer_ops *ops;
	int ret;
	unsigned long flags;

	ops = mse_packetizer_get_ops(id);
	if (!ops)
		return -EPERM;

	spin_lock_irqsave(&packetizer_lock, flags);
	ret = ops->release(index);
	spin_unlock_irqrestore(&packetizer_lock, flags);

	return ret;
}
