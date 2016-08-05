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

#ifndef __JPEG_H__
#define __JPEG_H__

#define JPEG_MARKER               (0xFF)
#define JPEG_MARKER_SIZE_LENGTH   (2)

#define JPEG_GET_HEADER_SIZE(__data, __offset) \
		(((__data)[(__offset)] << 8) | ((__data)[(__offset) + 1]))

#define JPEG_SOF_LENGTH           (17)
#define JPEG_SOF_SAMPLE_PREC      (8)
#define JPEG_SOF_PIXEL_LEN        (2)
#define JPEG_SOF_PIXEL_MAX        (2040)
#define JPEG_SOF_COMP_ID_Y        (0)
#define JPEG_SOF_COMP_ID_U        (1)
#define JPEG_SOF_COMP_ID_V        (2)
#define JPEG_SOF_COMP_SAMPLE_2X1  (0x02 << 4 | 0x01)
#define JPEG_SOF_COMP_SAMPLE_2X2  (0x02 << 4 | 0x02)
#define JPEG_SOF_COMP_SAMPLE_1X1  (0x01 << 4 | 0x01)
#define JPEG_DHT_TABLE_NUM        (4)
#define JPEG_SOS_ID_LEN           (12)
#define JPEG_DQT_ID_LEN           (1)
#define JPEG_DQT_QUANT_SIZE8      (64)
#define JPEG_DQT_QUANT_SIZE16     (128)
#define JPEG_DRI_LENGTH           (4)
#define JPEG_DRI_F_DEFAULT        (0x80)
#define JPEG_DRI_L_DEFAULT        (0x40)
#define JPEG_DRI_RCOUNT_DEFAULT   (0x3FFF)

#define JPEG_GET_DQT_PREC(__data) (((__data) & 0xF0) >> 4)
#define JPEG_GET_DQT_QID(__data)  ((__data) & 0x0F)
#define JPEG_SET_DQT_ID(__prec, __qid) \
				  ((((__prec) << 4) & 0xF0) + ((__qid) & 0x0F))

#define JPEG_QUANT_NUM (4)
#define JPEG_COMP_NUM  (3)
#define PIXEL_DIV_NUM  (8)
#define QTABLE_SIZE    (64)

enum MJPEG_TYPE {
	MJPEG_TYPE_422 = 0x00,
	MJPEG_TYPE_420 = 0x01,
};

#define MJPEG_TYPE_RESTART_BIT (0x40)

#define MJPEG_QUANT_QTABLE_BIT (0x80)
#define MJPEG_QUANT_DYNAMIC    (0xFF)

enum JPEG_MARKER_KIND {
	JPEG_MARKER_KIND_NIL  = 0x00,
	JPEG_MARKER_KIND_SOF0 = 0xC0,
	JPEG_MARKER_KIND_DHT  = 0xC4,
	JPEG_MARKER_KIND_SOI  = 0xD8,
	JPEG_MARKER_KIND_EOI  = 0xD9,
	JPEG_MARKER_KIND_SOS  = 0xDA,
	JPEG_MARKER_KIND_DQT  = 0xDB,
	JPEG_MARKER_KIND_DRI  = 0xDD,
	JPEG_MARKER_KIND_JFIF = 0xE0,
	JPEG_MARKER_KIND_COM  = 0xFE,
};

struct mjpeg_restart_header {
	u16 restart_interval;
	u16 f_l_restart_count; /* f:1, l:1, restart_count:13 */
};

struct mjpeg_quant_header {
	u8 mbz;
	u8 precision;
	u16 length;
};

struct mjpeg_quant_table {
	u8 size;
	u8 data[JPEG_DQT_QUANT_SIZE16];
};

struct mjpeg_component {
	u8 id;
	u8 samp;
	u8 qt;
};

extern u8 jpeg_get_marker(const u8 *buf, size_t len, size_t *offset);
extern int jpeg_read_sof(const u8 *buf,
			 size_t len,
			 size_t *offset,
			 enum MJPEG_TYPE *type,
			 u8 *max_comp,
			 struct mjpeg_component comp[],
			 struct mjpeg_quant_table qtable[],
			 size_t qtable_num,
			 s32 *height,
			 s32 *width);
extern int jpeg_read_dqt(const u8 *buf,
			 size_t len,
			 size_t *offset,
			 struct mjpeg_quant_table qtable[]);
extern int jpeg_read_dri(const u8 *buf,
			 size_t len,
			 size_t *offset,
			 struct mjpeg_restart_header *rheader);
extern u32 jpeg_make_header(enum MJPEG_TYPE type,
			    u8 quant,
			    u8 *p,
			    u32 w,
			    u32 h,
			    u8 *qt,
			    struct mjpeg_quant_header *qheader,
			    u16 dri);

#endif /* __JPEG_H__ */
