/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016-2017 Renesas Electronics Corporation

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

#include "ravb_mse_kernel.h"
#include "jpeg.h"

static const u32 jpeg_luma_quantizer[QTABLE_SIZE] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99,
};

static const u32 jpeg_chroma_quantizer[QTABLE_SIZE] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
};

const u32 *quantizer[] = {
	jpeg_luma_quantizer,
	jpeg_chroma_quantizer,
};

static u8 lum_dc_codelens[] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
};

static u8 lum_dc_symbols[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static u8 lum_ac_codelens[] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
};

static u8 lum_ac_symbols[] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static u8 chm_dc_codelens[] = {
	0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
};

static u8 chm_dc_symbols[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static u8 chm_ac_codelens[] = {
	0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
};

static u8 chm_ac_symbols[] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

u8 jpeg_get_marker(const u8 *buf, size_t len, size_t *offset)
{
	u8 marker;

	while ((buf[(*offset)++] != JPEG_MARKER) && (*offset < len))
		;

	if (*offset >= len)
		return JPEG_MARKER_KIND_NIL;

	marker = buf[(*offset)++];

	mse_debug("found marker=0x%02X offset=%zu\n", marker, *offset - 1);

	return marker;
}

int jpeg_read_sof(const u8 *buf,
		  size_t len,
		  size_t *offset,
		  enum MJPEG_TYPE *type,
		  u8 *max_comp,
		  struct mjpeg_component comp[],
		  struct mjpeg_quant_table qtable[],
		  size_t qtable_num,
		  s32 *height,
		  s32 *width)
{
	struct mjpeg_component comp_work;
	size_t header_len;
	size_t offset_work = *offset;
	s32 width_work, height_work;
	int i, j;
	u8 sample_bit, comp_num;

	if (offset_work + JPEG_SOF_LENGTH > len) {
		mse_debug("invalid header length, request next buffer.\n");
		return -EAGAIN;
	}

	header_len = JPEG_GET_HEADER_SIZE(buf, offset_work);
	if (header_len < JPEG_SOF_LENGTH) {
		mse_debug("invalid length request next buffer. hlen=%zu\n",
			  header_len);
		return -EAGAIN;
	}

	*offset += header_len;
	offset_work += JPEG_MARKER_SIZE_LENGTH;

	sample_bit = buf[offset_work++];
	if (sample_bit != JPEG_SOF_SAMPLE_PREC) {
		mse_warn("invalid value. sample_bit=0x%x\n", sample_bit);
		return -EPERM;
	}

	height_work = buf[offset_work] << 8 | buf[offset_work + 1];
	offset_work += JPEG_SOF_PIXEL_LEN;
	width_work = buf[offset_work] << 8 | buf[offset_work + 1];
	offset_work += JPEG_SOF_PIXEL_LEN;

	if (height_work == 0 || width_work == 0 ||
	    height_work > JPEG_SOF_PIXEL_MAX ||
	    width_work > JPEG_SOF_PIXEL_MAX) {
		mse_warn("invalid display size. %ux%u\n",
			 width_work, height_work);
		return -EPERM;
	}

	*height = height_work / PIXEL_DIV_NUM +
		  (height_work % PIXEL_DIV_NUM ? 1 : 0);
	*width = width_work / PIXEL_DIV_NUM +
		 (width_work % PIXEL_DIV_NUM ? 1 : 0);

	comp_num = buf[offset_work++];
	if (comp_num != JPEG_COMP_NUM) {
		mse_warn("invalid value. comp_num=%u\n", comp_num);
		return -EPERM;
	}

	/* get component */
	for (i = 0; i < comp_num; i++) {
		comp_work.id = buf[offset_work++];
		comp_work.samp = buf[offset_work++];
		comp_work.qt = buf[offset_work++];
		mse_debug("comp %d samp=%02x, qt=%d\n",
			  comp_work.id, comp_work.samp, comp_work.qt);
		for (j = i; j > 0; j--) {
			if (comp[j - 1].id < comp_work.id)
				break;
			comp[j] = comp[j - 1];
		}
		comp[j] = comp_work;
		if (*max_comp < comp_work.qt)
			*max_comp = comp_work.qt;
	}

	if (comp[JPEG_SOF_COMP_ID_Y].samp == JPEG_SOF_COMP_SAMPLE_2X1) {
		*type = MJPEG_TYPE_422;
	} else if (comp[JPEG_SOF_COMP_ID_Y].samp == JPEG_SOF_COMP_SAMPLE_2X2) {
		*type = MJPEG_TYPE_420;
	} else {
		mse_warn("comp[0].samp=%u\n", comp[JPEG_SOF_COMP_ID_Y].samp);
		return -EPERM;
	}

	if (comp[JPEG_SOF_COMP_ID_U].samp != JPEG_SOF_COMP_SAMPLE_1X1 ||
	    comp[JPEG_SOF_COMP_ID_V].samp != JPEG_SOF_COMP_SAMPLE_1X1) {
		mse_warn("comp[1].samp=%u comp[2].samp=%u\n",
			 comp[JPEG_SOF_COMP_ID_U].samp,
			 comp[JPEG_SOF_COMP_ID_V].samp);
		return -EPERM;
	}

	if (comp[JPEG_SOF_COMP_ID_U].qt != comp[JPEG_SOF_COMP_ID_V].qt) {
		u8 u = comp[JPEG_SOF_COMP_ID_U].qt;
		u8 v = comp[JPEG_SOF_COMP_ID_V].qt;

		if ((u >= qtable_num) || (v >= qtable_num) ||
		    (qtable[u].size == 0) ||
		    (qtable[u].size != qtable[v].size) ||
		    (memcmp(qtable[u].data,
			    qtable[v].data,
			    qtable[u].size) != 0)) {
			mse_warn("Invalid qtable.\n");
			return -EPERM;
		}
	}

	return 0;
}

int jpeg_read_dqt(const u8 *buf,
		  size_t len,
		  size_t *offset,
		  struct mjpeg_quant_table qtable[])
{
	size_t header_len, quant_len;
	u8 qid, precision;

	if (*offset + JPEG_MARKER_SIZE_LENGTH > len) {
		mse_debug("invalid header length, request next buffer.\n");
		return -EAGAIN;
	}

	header_len = JPEG_GET_HEADER_SIZE(buf, *offset);
	if (header_len < JPEG_MARKER_SIZE_LENGTH) {
		mse_debug("invalid length, request next buffer. hlen=%zu\n",
			  header_len);
		return -EAGAIN;
	}

	if (*offset + header_len > len) {
		mse_debug("header short, request next buffer. hlen=%zu size=%zu+%zu\n",
			  header_len, len, *offset);
		return -EAGAIN;
	}

	*offset += JPEG_MARKER_SIZE_LENGTH;
	header_len -= JPEG_MARKER_SIZE_LENGTH;

	while (header_len > 0) {
		qid = JPEG_GET_DQT_QID(buf[*offset]);
		if (qid > JPEG_QUANT_NUM) {
			mse_warn("invalid id. qid=%d\n", qid);
			return -EPERM;
		}

		precision = JPEG_GET_DQT_PREC(buf[*offset]);
		if (precision)
			quant_len = JPEG_DQT_QUANT_SIZE16;
		else
			quant_len = JPEG_DQT_QUANT_SIZE8;

		header_len--;
		(*offset)++;

		if (header_len < quant_len) {
			mse_warn("invalid length. hlen=%zu qlen=%zu\n",
				 header_len, quant_len);
			return -EPERM;
		}

		mse_debug("DQT Tq=%d Pq=%d Size=%zu",
			  qid, precision, quant_len);

		qtable[qid].precision = precision;
		qtable[qid].size = quant_len;
		memcpy(qtable[qid].data, &buf[*offset], quant_len);
		header_len -= quant_len;
		*offset += quant_len;
	}

	return 0;
}

int jpeg_read_dri(const u8 *buf,
		  size_t len,
		  size_t *offset,
		  struct mjpeg_restart_header *rheader)
{
	size_t header_len;
	size_t offset_work = *offset;

	if (*offset + JPEG_DRI_LENGTH > len) {
		mse_debug("not enough data for DRI, request next buffer");
		*offset += len;
		return -EAGAIN;
	}

	header_len = JPEG_GET_HEADER_SIZE(buf, *offset);
	if (header_len < JPEG_DRI_LENGTH) {
		mse_debug("invalid length, request next buffer. hlen=%zu\n",
			  header_len);
		*offset += header_len;
		return -EAGAIN;
	}

	*offset += header_len;
	offset_work += JPEG_MARKER_SIZE_LENGTH;

	rheader->restart_interval = (buf[offset_work] << 8) |
				    (buf[offset_work + 1]);
	rheader->f_l_restart_count = JPEG_DRI_F_DEFAULT |
				     JPEG_DRI_L_DEFAULT |
				     JPEG_DRI_RCOUNT_DEFAULT;

	return 0;
}

static u8 *jpeg_make_quantizer(u8 *p, u32 id, u8 q)
{
	u32 i;
	u32 factor = q;

	if (q < 1)
		factor = 1;
	else if (q > 99)
		factor = 99;

	if (factor < 50)
		factor = 5000 / factor;
	else
		factor = 200 - factor * 2;

	for (i = 0; i < QTABLE_SIZE; i++) {
		u32 value = (quantizer[id][i] * factor + 50) / 100;

		/* Limit the quantizers to 1 <= factor <= 255 */
		if (value < 1)
			value = 1;
		else if (value > 255)
			value = 255;
		*p++ = (u8)value;
	}

	return p;
}

static u8 *jpeg_make_dqt(u8 *p, u8 *qt, u32 id, u8 q, u32 len)
{
	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_DQT;
	*p++ = 0;
	*p++ = JPEG_MARKER_SIZE_LENGTH + JPEG_DQT_ID_LEN + len;
	*p++ = id;

	if (q) {
		memcpy(p, qt, len);
		p += len;
	} else {
		p = jpeg_make_quantizer(p, JPEG_GET_DQT_QID(id), q);
	}

	return p;
}

static u8 *jpeg_make_dht(u8 *p, u32 i)
{
	struct {
		u8 class;
		u8 id;
		u8 *codelens;
		u8 ncodes;
		u8 *symbols;
		u8 nsymbols;
	} htable[] = {
		{
			0, 0, lum_dc_codelens, ARRAY_SIZE(lum_dc_codelens),
			lum_dc_symbols, ARRAY_SIZE(lum_dc_symbols),
		},
		{
			1, 0, lum_ac_codelens, ARRAY_SIZE(lum_ac_codelens),
			lum_ac_symbols, ARRAY_SIZE(lum_ac_symbols),
		},
		{
			0, 1, chm_dc_codelens, ARRAY_SIZE(chm_dc_codelens),
			chm_dc_symbols, ARRAY_SIZE(chm_dc_symbols),
		},
		{
			1, 1, chm_ac_codelens, ARRAY_SIZE(chm_ac_codelens),
			chm_ac_symbols, ARRAY_SIZE(chm_ac_symbols),
		},
	};

	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_DHT;	/* DHT */
	*p++ = 0;					  /* length msb */
	*p++ = 3 + htable[i].ncodes + htable[i].nsymbols; /* length lsb */
	*p++ = (htable[i].class << 4) | htable[i].id;
	memcpy(p, htable[i].codelens, htable[i].ncodes);
	p += htable[i].ncodes;
	memcpy(p, htable[i].symbols, htable[i].nsymbols);
	p += htable[i].nsymbols;

	return p;
}

static u8 *jpeg_make_dri(u8 *p, u16 dri)
{
	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_DRI;	/* DRI */
	*p++ = 0;			/* length msb */
	*p++ = JPEG_DRI_LENGTH;		/* length lsb */
	*p++ = dri >> 8;		/* dri msb */
	*p++ = dri & 0xff;		/* dri lsb */
	return p;
}

u32 jpeg_make_header(enum MJPEG_TYPE type,
		     u8 quant,
		     u8 *p,
		     u32 w,
		     u32 h,
		     u8 *qt,
		     struct mjpeg_quant_header *qheader,
		     u16 dri)
{
	u8 *start = p;
	u16 left_len;
	u32 i;

	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_SOI;
	/* ToDo: trail marker start */
	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_JFIF;
	*p++ = 0;
	*p++ = 16;
	*p++ = 0x4A;
	*p++ = 0x46;
	*p++ = 0x49;
	*p++ = 0x46;
	*p++ = 0x00;
	*p++ = 0x01;
	*p++ = 0x01;
	*p++ = 0;
	*p++ = 0;
	*p++ = 1;
	*p++ = 0;
	*p++ = 1;
	*p++ = 0;
	*p++ = 0;
	/* ToDo: trail marker end */

	left_len = ntohs(qheader->length);
	for (i = 0; i < JPEG_QUANT_NUM && left_len; i++) {
		u32 len;
		u8 id;

		if (qheader->precision & (1 << i)) {
			id = JPEG_SET_DQT_ID(1, i);
			len = JPEG_DQT_QUANT_SIZE16;
		} else {
			id = i;
			len = JPEG_DQT_QUANT_SIZE8;
		}
		p = jpeg_make_dqt(p, qt, id, quant, len);
		qt += len;
		left_len -= len;
		if (left_len <= 0)
			break;
	}

	if (dri != 0)
		p = jpeg_make_dri(p, dri);

	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_SOF0;
	*p++ = 0;			/* length msb */
	*p++ = JPEG_SOF_LENGTH;		/* length lsb */
	*p++ = JPEG_SOF_SAMPLE_PREC;	/* 8-bit precision */
	*p++ = h >> 8;			/* height msb */
	*p++ = h;			/* height lsb */
	*p++ = w >> 8;			/* width msb */
	*p++ = w;			/* wudth lsb */
	*p++ = JPEG_COMP_NUM;		/* number of components */
	*p++ = JPEG_SOF_COMP_ID_Y + 1;		/* comp 0 */
	if (type == MJPEG_TYPE_422)
		*p++ = JPEG_SOF_COMP_SAMPLE_2X1; /* hsamp = 2, vsamp = 1 */
	else
		*p++ = JPEG_SOF_COMP_SAMPLE_2X2; /* hsamp = 2, vsamp = 2 */
	*p++ = 0;				/* quant table 0 */
	*p++ = JPEG_SOF_COMP_ID_U + 1;		/* comp 1 */
	*p++ = JPEG_SOF_COMP_SAMPLE_1X1;	/* hsamp = 1, vsamp = 1 */
	*p++ = 1;				/* quant table 1 */
	*p++ = JPEG_SOF_COMP_ID_V + 1;		/* comp 2 */
	*p++ = JPEG_SOF_COMP_SAMPLE_1X1;	/* hsamp = 1, vsamp = 1 */
	*p++ = 1;				/* quant table 1 */

	for (i = 0; i < JPEG_DHT_TABLE_NUM; i++)
		p = jpeg_make_dht(p, i);

	*p++ = JPEG_MARKER;
	*p++ = JPEG_MARKER_KIND_SOS;
	*p++ = 0;			/* length msb */
	*p++ = JPEG_SOS_ID_LEN;		/* length lsb */
	*p++ = JPEG_COMP_NUM;		/* 3 components */
	*p++ = 1;			/* comp 1 */
	*p++ = 0;			/* huffman table 0 */
	*p++ = 2;			/* comp 2 */
	*p++ = 0x11;			/* huffman table 1 */
	*p++ = 3;			/* comp 3 */
	*p++ = 0x11;			/* huffman table 1 */
	*p++ = 0;			/* first DCT coeff */
	*p++ = 63;			/* last DCT coeff */
	*p++ = 0;			/* sucessive approx. */

	mse_debug("header size=%u\n", (u32)(p - start));

	return p - start;
};
