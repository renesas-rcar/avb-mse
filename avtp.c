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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>

#include "avtp.h"

/* P1722/D16 6.4.3 IEC 61883 (SPH=1) encapsulation */
struct avtp_iec61883_sph1_hdr {
	u8 subtype;
	u8 sv_version_mr_gv_tv; /* sv:1, version:3, mr:1, r:1, gv:1, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	__be32 gateway_info;
	__be16 stream_data_length;
	u8 tag_channel; /* tag:2, channel:6 */
	u8 tcode_sy; /* tcode:4, sy:4 */
	struct {
		u8 qi1_sid; /* qi_1:2, sid:6 */
		u8 dbs;
		u8 fn_qpc_sph; /* fn:2, qpc:3, sph:1, rsv:2 */
		u8 dbc;
	} cip1;
	struct {
		__be32 qi2_fmt_fdf; /* qi_2:2, fmt:6, fdf:24 */
	} cip2;
	u8 payload[0];
} __packed;

/* AVTP IEC61883-4 header */
static const struct avtp_iec61883_sph1_hdr avtp_iec61883_4_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_61883_IIDC,
	.sv_version_mr_gv_tv    = 0x81, /* sv=1, version=0, mr=0, gv=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0x01,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.gateway_info           = cpu_to_be32(0),
	.stream_data_length     = cpu_to_be16(0),
	.tag_channel            = 0x5f, /* tag=1, channel=0x1f  */
	.tcode_sy               = 0xa0, /* tcode=0xa, sy=0 */
	.cip1.qi1_sid           = 0x3f, /* qi_1=0, sid=0x3f */
	.cip1.dbs               = 6,    /* 6 quadlets (see std IEC 61883-4 5.1) */
	.cip1.fn_qpc_sph        = 3 << 2, /* fn=3, qpc=0, sph=1 */
	.cip1.dbc               = 0,
	.cip2.qi2_fmt_fdf      = (2 << 30 | AVTP_IEC61883_FMT_MPEG2TS << 24) /* qi_2=2, fmt=0x20, fdf=0 */
};

void avtp_copy_iec61883_4_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_iec61883_4_hdr_tmpl,
	       sizeof(avtp_iec61883_4_hdr_tmpl));
}

/* P1722/D16 6.4.3 IEC 61883 (SPH=0) encapsulation */
struct avtp_iec61883_sph0_hdr {
	u8 subtype;
	u8 sv_version_mr_gv_tv; /* sv:1, version:3, mr:1, r:1, gv:1, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	__be32 gateway_info;
	__be16 stream_data_length;
	u8 tag_channel; /* tag:2, channel:6 */
	u8 tcode_sy; /* tcode:4, sy:4 */
	struct {
		u8 qi1_sid; /* qi_1:2, sid:6 */
		u8 dbs;
		u8 fn_qpc_sph; /* fn:2, qpc:3, sph:1, rsv:2 */
		u8 dbc;
	} cip1;
	struct {
		u8 qi2_fmt; /* qi_2:2, fmt:6 */
		u8 fdf;
		__be16 syt;
	} cip2;
	u8 payload[0];
} __packed;

/* AVTP IEC61883-6 header */
static const struct avtp_iec61883_sph0_hdr avtp_iec61883_6_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_61883_IIDC,
	.sv_version_mr_gv_tv    = 0x81, /* sv=1, version=0, mr=0, gv=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0x01,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.gateway_info           = cpu_to_be32(0),
	.stream_data_length     = cpu_to_be16(0),
	.tag_channel            = 0x5f, /* tag=1, channel=0x1f  */
	.tcode_sy               = 0xa0, /* tcode=0xa, sy=0 */
	.cip1.qi1_sid           = 0x3f, /* qi_1=0, sid=0x3f */
	.cip1.dbs               = 0,
	.cip1.fn_qpc_sph        = AVTP_IEC61883_FN_NODIV << 6, /* fn=0, qpc=0, sph=0 */
	.cip1.dbc               = 0,
	.cip2.qi2_fmt           = (2 << 6 | AVTP_IEC61883_FMT_AUDIO), /* qi_2=2, fmt=0x10 */
	.cip2.fdf               = 0,
	.cip2.syt               = cpu_to_be16(0xFFFF),
};

void avtp_copy_iec61883_6_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_iec61883_6_hdr_tmpl,
	       sizeof(avtp_iec61883_6_hdr_tmpl));
}

/* P1722/D16 8.3 AAF PCM stream data encapsulation */
struct avtp_aaf_pcm_hdr {
	u8 subtype;
	u8 sv_version_mr_tv; /* sv:1, version:3, mr:1, rsv:2, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	u8 format;
	__be16 nsr_channels_per_frame; /* nsr:4, rsv:2, channels_per_frame:10 */
	u8 bit_depth;
	__be16 stream_data_length;
	u8 sp_evt; /* rsv:3, sp:1, evt:4 */
	u8 reserved;
	u8 payload[0];
} __packed;

/* AVTP AAF PCM header */
static const struct avtp_aaf_pcm_hdr avtp_aaf_pcm_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_AAF,
	.sv_version_mr_tv       = 0x81, /* sv=1, version=0, mr=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.format                 = AVTP_AAF_FORMAT_INT_16BIT,
	.nsr_channels_per_frame = 0,
	.bit_depth              = 16,
	.stream_data_length     = cpu_to_be16(0),
	.sp_evt                 = 0,
	.reserved               = 0,
};

void avtp_copy_aaf_pcm_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_aaf_pcm_hdr_tmpl,
	       sizeof(avtp_aaf_pcm_hdr_tmpl));
}

/* P1722/D13 9.5 H.264 Video Format */
struct avtp_cvf_h264_d13_hdr {
	u8 subtype;
	u8 sv_version_mr_tv; /* sv:1, version:3, mr:1, rsv:2, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	u8 format;
	u8 format_subtype;
	__be16 reserved0;
	__be16 stream_data_length;
	u8 M_evt; /* rsv:3, M:1, evt:4 */
	u8 reserved1;
	u8 payload[0];
} __packed;

/* AVTP CVF H.264 without h264_timestamp field header */
static const struct avtp_cvf_h264_d13_hdr avtp_cvf_h264_d13_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_CVF,
	.sv_version_mr_tv       = 0x81, /* sv=1, version=0, mr=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.format                 = AVTP_CVF_FORMAT_RFC,
	.format_subtype         = AVTP_CVF_RFC_FORMAT_H264,
	.reserved0              = cpu_to_be16(0),
	.stream_data_length     = cpu_to_be16(0),
	.M_evt                  = 0,
	.reserved1              = 0,
};

void avtp_copy_cvf_h264_d13_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_cvf_h264_d13_hdr_tmpl,
	       sizeof(avtp_cvf_h264_d13_hdr_tmpl));
}

/* P1722/D16 9.5 H.264 Video Format */
struct avtp_cvf_h264_hdr {
	u8 subtype;
	u8 sv_version_mr_tv; /* sv:1, version:3, mr:1, rsv:2, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	u8 format;
	u8 format_subtype;
	__be16 reserved0;
	__be16 stream_data_length;
	u8 ptv_M_evt; /* rsv:2, ptv:1, M:1, evt:4 */
	u8 reserved1;
	__be32 h264_timestamp;
	u8 payload[0];
} __packed;

/* AVTP CVF H.264 with h264_timestamp field header */
static const struct avtp_cvf_h264_hdr avtp_cvf_h264_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_CVF,
	.sv_version_mr_tv       = 0x81, /* sv=1, version=0, mr=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.format                 = AVTP_CVF_FORMAT_RFC,
	.format_subtype         = AVTP_CVF_RFC_FORMAT_H264,
	.reserved0              = cpu_to_be16(0),
	.stream_data_length     = cpu_to_be16(0),
	.ptv_M_evt                  = 0,
	.reserved1              = 0,
	.h264_timestamp         = 0,
};

void avtp_copy_cvf_h264_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_cvf_h264_hdr_tmpl,
	       sizeof(avtp_cvf_h264_hdr_tmpl));
}

/* P1722-rev1/D16 9.4 MJPEG Video Format */
struct avtp_cvf_mjpeg_hdr {
	u8 subtype;
	u8 sv_version_mr_tv; /* sv:1, version:3, mr:1, rsv:2, tv:1 */
	u8 sequence_num;
	u8 tu; /* reserved:7, tu:1 */
	__be64 stream_id;
	__be32 avtp_timestamp;
	u8 format;
	u8 format_subtype;
	__be16 reserved0;
	__be16 stream_data_length;
	u8 M_evt; /* rsv:3, M:1, evt:4 */
	u8 reserved1;
	struct {
		__be32 type_specific_fragment_offset; /* tspec:8 offset:24 */
		u8 type;
		u8 Q;
		u8 width;
		u8 height;
	} jpg;
	u8 payload[0];
} __packed;

/* AVTP CVF MJPEG field header */
static const struct avtp_cvf_mjpeg_hdr avtp_cvf_mjpeg_hdr_tmpl = {
	.subtype                = AVTP_SUBTYPE_CVF,
	.sv_version_mr_tv       = 0x81, /* sv=1, version=0, mr=0, tv=1 */
	.sequence_num           = 0,
	.tu                     = 0,
	.stream_id              = cpu_to_be64(0),
	.avtp_timestamp         = cpu_to_be32(0),
	.format                 = AVTP_CVF_FORMAT_RFC,
	.format_subtype         = AVTP_CVF_RFC_FORMAT_MJPEG,
	.reserved0              = cpu_to_be16(0),
	.stream_data_length     = cpu_to_be16(0),
	.M_evt                  = 0,
	.reserved1              = 0,
	.jpg.type_specific_fragment_offset = 0,
	.jpg.type                          = 0,
	.jpg.Q                             = 0,
	.jpg.width                         = 0,
	.jpg.height                        = 0,
};

void avtp_copy_cvf_mjpeg_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_cvf_mjpeg_hdr_tmpl,
	       sizeof(avtp_cvf_mjpeg_hdr_tmpl));
}

/* CRF */
struct avtp_crf_hdr {
	u8 subtype;
	u8 sv_version_mr_gv_tv; /* sv:1, version:3, mr:1, r:1, fs:1, tu:1 */
	u8 sequence_num;
	u8 type;
	__be64 stream_id;
	__be32 packet_info;     /* pull:3, base fequency:29 */
	__be16 crf_data_length;
	__be16 timestamp_interval;
} __packed;

/* AVTP CRF header */
static const struct avtp_crf_hdr avtp_crf_tmpl = {
	.subtype                = AVTP_SUBTYPE_CRF,
	.sv_version_mr_gv_tv    = 0x80, /* sv=1, version=0, mr=0, gv=0, tv=0 */
	.sequence_num           = 0,
	.type                   = AVTP_CRF_TYPE_AUDIO_SAMPLE,
	.stream_id              = cpu_to_be64(0),
	.packet_info            = cpu_to_be32(0),
	.crf_data_length        = cpu_to_be16(0),
	.timestamp_interval     = cpu_to_be16(0),
};

void avtp_copy_crf_template(void *data)
{
	memcpy(data + AVTP_OFFSET, &avtp_crf_tmpl, sizeof(struct avtp_crf_hdr));
}
