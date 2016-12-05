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

#ifndef __AVTP_H__
#define __AVTP_H__

/* ether header(6+6+2=14), Q-Tag(4) */
#define ETHOVERHEAD       (ETH_HLEN + 4)
/* preamble(8), ETHOVERHEAD, CRC(4) */
#define ETHOVERHEAD_REAL  (8 + ETHOVERHEAD + ETH_FCS_LEN)
/* ETHOVERHEAD_REAL, IFG(12) */
#define ETHOVERHEAD_IFG   (ETHOVERHEAD_REAL + 12)

#define ETHFRAMEMTU_MIN   (ETH_ZLEN - ETH_HLEN)
#define ETHFRAMEMTU_MAX   ETH_DATA_LEN
#define ETHFRAMELEN_MIN   (ETHFRAMEMTU_MIN + ETHOVERHEAD)
#define ETHFRAMELEN_MAX   (ETHFRAMEMTU_MAX + ETHOVERHEAD)

#define ETH_P_1722 ETH_P_TSN

#ifndef AVTP_OFFSET
/* Ethernet frame header length (DA + SA + Qtag + EthType) */
#define AVTP_OFFSET (18)
#endif

#define AVTP_CIP_HEADER_SIZE             (8)
#define AVTP_FRAME_SIZE_MIN              (60)
#define AVTP_PAYLOAD_OFFSET              (24 + AVTP_OFFSET)
#define AVTP_IEC61883_4_PAYLOAD_OFFSET \
	(AVTP_CIP_HEADER_SIZE + AVTP_PAYLOAD_OFFSET)
#define AVTP_IEC61883_6_PAYLOAD_OFFSET \
	(AVTP_CIP_HEADER_SIZE + AVTP_PAYLOAD_OFFSET)
#define AVTP_AAF_PAYLOAD_OFFSET          (AVTP_PAYLOAD_OFFSET)
#define AVTP_CVF_H264_D13_PAYLOAD_OFFSET (AVTP_PAYLOAD_OFFSET)
#define AVTP_CVF_H264_PAYLOAD_OFFSET     (4 + AVTP_PAYLOAD_OFFSET)
#define AVTP_CVF_MJPEG_PAYLOAD_OFFSET    (8 + AVTP_PAYLOAD_OFFSET)
#define AVTP_CRF_PAYLOAD_OFFSET          (20 + AVTP_OFFSET)

#define AVTP_MAC_ADDR_SIZE ETH_ALEN
#define AVTP_STREAMID_SIZE (8)

#define AVTP_SEQUENCE_NUM_MAX (255)

#define DEF_ACCESSER_UINT16(name, offset) \
	static inline u16 get_##name(void *data) \
	{ \
		return htons(*((u16 *)(data + offset))); \
	} \
	static inline void set_##name(void *data, u16 value) \
	{ \
		*((u16 *)(data + offset)) = htons(value); \
	}

#define DEF_AVTP_ACCESSER_UINT8(name, offset) \
	static inline u8 avtp_get_##name(void *data) \
	{ \
		return *((u8 *)(data + offset + AVTP_OFFSET)); \
	} \
	static inline void avtp_set_##name(void *data, u8 value) \
	{ \
		*((u8 *)(data + offset + AVTP_OFFSET)) = value; \
	}

#define DEF_AVTP_ACCESSER_UINT16(name, offset) \
	static inline u16 avtp_get_##name(void *data) \
	{ \
		return htons(*((u16 *)(data + offset + AVTP_OFFSET))); \
	} \
	static inline void avtp_set_##name(void *data, u16 value) \
	{ \
		*((u16 *)(data + offset + AVTP_OFFSET)) = htons(value); \
	}

#define DEF_AVTP_ACCESSER_UINT32(name, offset) \
	static inline u32 avtp_get_##name(void *data) \
	{ \
		return htonl(*((u32 *)(data + offset + AVTP_OFFSET))); \
	} \
	static inline void avtp_set_##name(void *data, u32 value) \
	{ \
		*((u32 *)(data + offset + AVTP_OFFSET)) = htonl(value); \
	}

/* P1722/D16 Table 6 - AVTP stream data subtype values */
enum AVTP_SUBTYPE {
	AVTP_SUBTYPE_61883_IIDC     = 0x00, /* IEC 61883/IIDC Format */
	AVTP_SUBTYPE_MMA_STREAM     = 0x01, /* MMA Streams */
	AVTP_SUBTYPE_AAF            = 0x02, /* AVTP Audio Format */
	AVTP_SUBTYPE_CVF            = 0x03, /* Compressed Video Format */
	AVTP_SUBTYPE_CRF            = 0x04, /* Clock Reference Format */
	AVTP_SUBTYPE_TSCF           = 0x05, /* Time Synchronous Control Format */
	AVTP_SUBTYPE_SVF            = 0x06, /* SDI Video Format */
	AVTP_SUBTYPE_RVF            = 0x07, /* Raw Video Format */
	/* 0x08-0x6D Reserved */
	AVTP_SUBTYPE_AEF_CONTINUOUS = 0x6E, /* AES Encrypted Format Stream */
	AVTP_SUBTYPE_VSF_STREAM     = 0x6F, /* Vender Specific Foramt Stream */
	/* 0x70-0x7E Reserved */
	AVTP_SUBTYPE_EF_STREAM      = 0x7F, /* Experimental Foramt Stream */
	/* 0x80-0x81 Reserved */
	AVTP_SUBTYPE_NTSCF          = 0x82, /* Non Time Synchronous Control Format */
	/* 0x83-0xED Reserved */
	AVTP_SUBTYPE_ESCF           = 0xEC, /* ECC Signed Control Format */
	AVTP_SUBTYPE_EECF           = 0xED, /* ECC Encrypted Control Format */
	AVTP_SUBTYPE_AEF_DISCRETE   = 0xEE, /* AES Encrypted Format Discrete */
	/* 0xEF-0xF9 Reserved */
	AVTP_SUBTYPE_ADP            = 0xFA, /* AVDECC Discovery Protocol */
	AVTP_SUBTYPE_AECP           = 0xFB, /* AVDECC Enumeration and Control Protocol */
	AVTP_SUBTYPE_ACMP           = 0xFC, /* AVDECC Connection Management Protocol */
	/* 0xFD Reserved */
	AVTP_SUBTYPE_MAAP           = 0xFE, /* MAAP Protocol */
	AVTP_SUBTYPE_EF_CONTROL     = 0xFF, /* Experimental Format Control */
};

/* IEC61883-1 Table 2 - Code allocation of FN */
enum AVTP_IEC61883_FN {
	AVTP_IEC61883_FN_NODIV = 0x0, /* 00'b Not divided */
	AVTP_IEC61883_FN_DIV_2 = 0x1, /* 01'b Divided into two data blocks */
	AVTP_IEC61883_FN_DIV_4 = 0x2, /* 10'b Divided into four data blocks */
	AVTP_IEC61883_FN_DIV_8 = 0x3, /* 11'b Divided into eight data blocks */
};

/* IEC61883-1 Table 5 - Code allocation of FMT */
enum AVTP_IEC61883_FMT {
	AVTP_IEC61883_FMT_DVCR    = 0x00, /* 00_0000'b DVCR */
	AVTP_IEC61883_FMT_601     = 0x01, /* 00_0001'b 601 over 1394 */
	AVTP_IEC61883_FMT_AUDIO   = 0x10, /* 01_0000'b Audio and music */
	AVTP_IEC61883_FMT_VENDOR0 = 0x1e, /* 01_1110'b Free (vendor unique) */
	AVTP_IEC61883_FMT_MPEG2TS = 0x20, /* 02_0000'b MPEG2-TS */
	AVTP_IEC61883_FMT_1294B   = 0x21, /* 02_0001'b ITU-R B0.1294 System B */
	AVTP_IEC61883_FMT_VENDOR1 = 0x3e, /* 03_1110'b Free (vendor unique) */
	AVTP_IEC61883_FMT_NODATA  = 0x3f, /* 03_1111'b No data */
};

/* IEC61883-6 Table 19 - Event type (EVT) code definition */
enum AVTP_IEC61883_6_FDF_EVT {
	AVTP_IEC61883_6_FDF_EVT_AM824       = 0 << 4, /* AM824 data */
	AVTP_IEC61883_6_FDF_EVT_24BIT_4PACK = 1 << 4, /* 24-bit * 4 audio Pack */
	AVTP_IEC61883_6_FDF_EVT_32BIT_FLOAT = 2 << 4, /* 32-bit floating-point data */
	AVTP_IEC61883_6_FDF_EVT_RESERVED    = 3 << 4, /* reserved */
};

/* IEC61883-6 Table 20 - Default SFC table */
enum AVTP_IEC61883_6_FDF_SFC {
	AVTP_IEC61883_6_FDF_SFC_32KHZ       = 0,
	AVTP_IEC61883_6_FDF_SFC_44_1KHZ     = 1,
	AVTP_IEC61883_6_FDF_SFC_48KHZ       = 2,
	AVTP_IEC61883_6_FDF_SFC_88_2KHZ     = 3,
	AVTP_IEC61883_6_FDF_SFC_96KHZ       = 4,
	AVTP_IEC61883_6_FDF_SFC_176_4KHZ    = 5,
	AVTP_IEC61883_6_FDF_SFC_192KHZ      = 6,
	AVTP_IEC61883_6_FDF_SFC_RESERVED    = 7,
};

enum AVTP_IEC61883_6_FDF_AM824 {
	AVTP_IEC61883_6_FDF_AM824_32KHZ    = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_32KHZ,
	AVTP_IEC61883_6_FDF_AM824_44_1KHZ  = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_44_1KHZ,
	AVTP_IEC61883_6_FDF_AM824_48KHZ    = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_48KHZ,
	AVTP_IEC61883_6_FDF_AM824_88_2KHZ  = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_88_2KHZ,
	AVTP_IEC61883_6_FDF_AM824_96KHZ    = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_96KHZ,
	AVTP_IEC61883_6_FDF_AM824_176_4KHZ = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_176_4KHZ,
	AVTP_IEC61883_6_FDF_AM824_192KHZ   = AVTP_IEC61883_6_FDF_EVT_AM824 | AVTP_IEC61883_6_FDF_SFC_192KHZ,
};

/* P1722/D16 Table 9 - AAF Field Values */
enum AVTP_AAF_FORMAT {
	AVTP_AAF_FORMAT_USER        = 0x00, /* User specified */
	AVTP_AAF_FORMAT_FLOAT_32BIT = 0x01, /* 32 bit floating point. See 8.2.2.1. */
	AVTP_AAF_FORMAT_INT_32BIT   = 0x02, /* 32 bit integer */
	AVTP_AAF_FORMAT_INT_24BIT   = 0x03, /* 24 bit integer */
	AVTP_AAF_FORMAT_INT_16BIT   = 0x04, /* 16 bit integer */
	AVTP_AAF_FORMAT_AES3_32BIT  = 0x05, /* 32 bit AES3 format. See 8.4 and Annex K. */
	/* 0x06-0xFF Reserved */
};

/* P1722/D16 Table 11 - nsr field values */
enum AVTP_AAF_NSR {
	AVTP_AAF_NSR_USER     = 0x0, /* User specified */
	AVTP_AAF_NSR_8KHZ     = 0x1,
	AVTP_AAF_NSR_16KHZ    = 0x2,
	AVTP_AAF_NSR_32KHZ    = 0x3,
	AVTP_AAF_NSR_44_1KHZ  = 0x4,
	AVTP_AAF_NSR_48KHZ    = 0x5,
	AVTP_AAF_NSR_88_2KHZ  = 0x6,
	AVTP_AAF_NSR_96KHZ    = 0x7,
	AVTP_AAF_NSR_176_4KHZ = 0x8,
	AVTP_AAF_NSR_192KHZ   = 0x9,
	AVTP_AAF_NSR_24KHZ    = 0xA,
	/* 0xB-0xF Reserved */
};

/* P1722/D16 Table 19 - Compressed Video format field */
enum AVTP_CVF_FORMAT {
	/* 0x00-0x01 Reserved */
	AVTP_CVF_FORMAT_RFC = 0x02, /* RFC Payload type */
	/* 0x03-0xFF Reserved */
};

/* P1722/D16 Table 20 - format_subtype field for RFC format */
enum AVTP_CVF_RFC_FORMAT {
	AVTP_CVF_RFC_FORMAT_MJPEG    = 0x0, /* MJPEG format (RFC 2435) */
	AVTP_CVF_RFC_FORMAT_H264     = 0x1, /* H.264 format (RFC 6184) */
	AVTP_CVF_RFC_FORMAT_JPEG2000 = 0x2, /* JPEG 2000 Video (RFC 5371) */
	/* 0x03-0xFF Reserved */
};

/* P1722/D16 Table 26 CRF Types */
enum AVTP_CRF_TYPE {
	AVTP_CRF_TYPE_USER          = 0x1, /* User Specified               */
	AVTP_CRF_TYPE_AUDIO_SAMPLE  = 0x2, /* Audio Sample Timestamp       */
	AVTP_CRF_TYPE_VIDEO_FRAME   = 0x3, /* Video Frame Sync Timestamp   */
	AVTP_CRF_TYPE_VIDEO_LINE    = 0x4, /* Video Line Sync Timestamp    */
	AVTP_CRF_TYPE_MACHINE_CYCLE = 0x5, /* Machine Cycle Timestamp      */
};

/* P1722/D16 Table 27 - pull field values */
enum AVTP_CRF_BASE_FREQ {
	AVTP_CRF_BASE_FREQ_0 = 0x0, /* Multiply base_frequency field by 1.0     */
	AVTP_CRF_BASE_FREQ_1 = 0x1, /* Multiply base_frequency field by 1/1.001 */
	AVTP_CRF_BASE_FREQ_2 = 0x2, /* Multiply base_frequency field by 1.001   */
	AVTP_CRF_BASE_FREQ_3 = 0x3, /* Multiply base_frequency field by 24/25   */
	AVTP_CRF_BASE_FREQ_4 = 0x4, /* Multiply base_frequency field by 25/24   */
	AVTP_CRF_BASE_FREQ_5 = 0x5, /* Multiply base_frequency field by 1/8     */
};

/**
 * Accessor - IEEE802.1Q
 */
DEF_ACCESSER_UINT16(ieee8021q_tpid, 12);
DEF_ACCESSER_UINT16(ieee8021q_tci, 14);
DEF_ACCESSER_UINT16(ieee8021q_ethtype, 16);

static inline void get_ieee8021q_dest(void *data, u8 value[6])
{
	value[0] = *((u8 *)(data + 0));
	value[1] = *((u8 *)(data + 1));
	value[2] = *((u8 *)(data + 2));
	value[3] = *((u8 *)(data + 3));
	value[4] = *((u8 *)(data + 4));
	value[5] = *((u8 *)(data + 5));
}

static inline void set_ieee8021q_dest(void *data, u8 value[6])
{
	*((u8 *)(data + 0)) = value[0];
	*((u8 *)(data + 1)) = value[1];
	*((u8 *)(data + 2)) = value[2];
	*((u8 *)(data + 3)) = value[3];
	*((u8 *)(data + 4)) = value[4];
	*((u8 *)(data + 5)) = value[5];
}

static inline void get_ieee8021q_source(void *data, u8 value[6])
{
	value[0] = *((u8 *)(data + 6));
	value[1] = *((u8 *)(data + 7));
	value[2] = *((u8 *)(data + 8));
	value[3] = *((u8 *)(data + 9));
	value[4] = *((u8 *)(data + 10));
	value[5] = *((u8 *)(data + 11));
}

static inline void set_ieee8021q_source(void *data, u8 value[6])
{
	*((u8 *)(data + 6))  = value[0];
	*((u8 *)(data + 7))  = value[1];
	*((u8 *)(data + 8))  = value[2];
	*((u8 *)(data + 9))  = value[3];
	*((u8 *)(data + 10)) = value[4];
	*((u8 *)(data + 11)) = value[5];
}

/**
 * Accessor - IEEE1722
 */
DEF_AVTP_ACCESSER_UINT8(subtype, 0)
DEF_AVTP_ACCESSER_UINT8(sequence_num, 2)
DEF_AVTP_ACCESSER_UINT32(timestamp, 12)
DEF_AVTP_ACCESSER_UINT16(stream_data_length, 20)

static inline void avtp_get_stream_id(void *data, u8 value[8])
{
	value[0] = *((u8 *)(data + 4 + AVTP_OFFSET));
	value[1] = *((u8 *)(data + 5 + AVTP_OFFSET));
	value[2] = *((u8 *)(data + 6 + AVTP_OFFSET));
	value[3] = *((u8 *)(data + 7 + AVTP_OFFSET));
	value[4] = *((u8 *)(data + 8 + AVTP_OFFSET));
	value[5] = *((u8 *)(data + 9 + AVTP_OFFSET));
	value[6] = *((u8 *)(data + 10 + AVTP_OFFSET));
	value[7] = *((u8 *)(data + 11 + AVTP_OFFSET));
}

static inline void avtp_set_stream_id(void *data, u8 value[8])
{
	*((u8 *)(data + 4 + AVTP_OFFSET))  = value[0];
	*((u8 *)(data + 5 + AVTP_OFFSET))  = value[1];
	*((u8 *)(data + 6 + AVTP_OFFSET))  = value[2];
	*((u8 *)(data + 7 + AVTP_OFFSET))  = value[3];
	*((u8 *)(data + 8 + AVTP_OFFSET))  = value[4];
	*((u8 *)(data + 9 + AVTP_OFFSET))  = value[5];
	*((u8 *)(data + 10 + AVTP_OFFSET)) = value[6];
	*((u8 *)(data + 11 + AVTP_OFFSET)) = value[7];
}

/* IEC61883 */
DEF_AVTP_ACCESSER_UINT8(iec61883_gateway_info, 16)
DEF_AVTP_ACCESSER_UINT8(iec61883_dbs, 25)
DEF_AVTP_ACCESSER_UINT8(iec61883_dbc, 27)
DEF_AVTP_ACCESSER_UINT8(iec61883_fdf, 29)
DEF_AVTP_ACCESSER_UINT16(iec61883_syt, 30)

static inline u8 avtp_get_iec61883_sid(void *data)
{
	return *((u8 *)(data + 24 + AVTP_OFFSET)) & 0xc0;
}

static inline int avtp_sample_rate_to_fdf(int sample_rate)
{
	switch (sample_rate) {
	/* 0b0000_0xxx Basic format for AM824 */
	case  32000: return AVTP_IEC61883_6_FDF_AM824_32KHZ;
	case  44100: return AVTP_IEC61883_6_FDF_AM824_44_1KHZ;
	case  48000: return AVTP_IEC61883_6_FDF_AM824_48KHZ;
	case  88200: return AVTP_IEC61883_6_FDF_AM824_88_2KHZ;
	case  96000: return AVTP_IEC61883_6_FDF_AM824_96KHZ;
	case 176400: return AVTP_IEC61883_6_FDF_AM824_176_4KHZ;
	case 192000: return AVTP_IEC61883_6_FDF_AM824_192KHZ;
	/* other, not supported */
	default: return 0x07;
	}
}

static inline int avtp_fdf_to_sample_rate(int format_dependent_field)
{
	switch (format_dependent_field) {
	/* 0b0000_0xxx Basic format for AM824 */
	case AVTP_IEC61883_6_FDF_AM824_32KHZ:    return  32000;
	case AVTP_IEC61883_6_FDF_AM824_44_1KHZ:  return  44100;
	case AVTP_IEC61883_6_FDF_AM824_48KHZ:    return  48000;
	case AVTP_IEC61883_6_FDF_AM824_88_2KHZ:  return  88200;
	case AVTP_IEC61883_6_FDF_AM824_96KHZ:    return  96000;
	case AVTP_IEC61883_6_FDF_AM824_176_4KHZ: return 176400;
	case AVTP_IEC61883_6_FDF_AM824_192KHZ:   return 192000;
	/* other, not supported */
	default: return 0;
	}
}

/* AAF */
DEF_AVTP_ACCESSER_UINT8(aaf_format, 16)
DEF_AVTP_ACCESSER_UINT8(aaf_channels_per_frame, 18)
DEF_AVTP_ACCESSER_UINT8(aaf_bit_depth, 19)

static inline u8 avtp_get_aaf_nsr(void *data)
{
	u8 tmp = *((u8 *)(data + 17 + AVTP_OFFSET));

	return (tmp & 0xf0) >> 4;
}

static inline void avtp_set_aaf_nsr(void *data, u8 value)
{
	u8 tmp = *((u8 *)(data + 17 + AVTP_OFFSET));

	*((u8 *)(data + 17 + AVTP_OFFSET)) = (tmp & 0x0f) | (value << 4);
}

static inline int avtp_sample_rate_to_nsr(int sample_rate)
{
	switch (sample_rate) {
	case   8000: return AVTP_AAF_NSR_8KHZ;
	case  16000: return AVTP_AAF_NSR_16KHZ;
	case  24000: return AVTP_AAF_NSR_24KHZ;
	case  32000: return AVTP_AAF_NSR_32KHZ;
	case  44100: return AVTP_AAF_NSR_44_1KHZ;
	case  48000: return AVTP_AAF_NSR_48KHZ;
	case  88200: return AVTP_AAF_NSR_88_2KHZ;
	case  96000: return AVTP_AAF_NSR_96KHZ;
	case 176400: return AVTP_AAF_NSR_176_4KHZ;
	case 192000: return AVTP_AAF_NSR_192KHZ;
	default: return AVTP_AAF_NSR_USER; /* User defined */
	}
}

static inline int avtp_aaf_nsr_to_sample_rate(int nsr)
{
	switch (nsr) {
	case AVTP_AAF_NSR_8KHZ:     return   8000;
	case AVTP_AAF_NSR_16KHZ:    return  16000;
	case AVTP_AAF_NSR_24KHZ:    return  24000;
	case AVTP_AAF_NSR_32KHZ:    return  32000;
	case AVTP_AAF_NSR_44_1KHZ:  return  44100;
	case AVTP_AAF_NSR_48KHZ:    return  48000;
	case AVTP_AAF_NSR_88_2KHZ:  return  88200;
	case AVTP_AAF_NSR_96KHZ:    return  96000;
	case AVTP_AAF_NSR_176_4KHZ: return 176400;
	case AVTP_AAF_NSR_192KHZ:   return 192000;
	/* other, not supported */
	default: return 0x00;
	}
}

static inline int avtp_aaf_format_to_bytes(int format)
{
	switch (format) {
	case AVTP_AAF_FORMAT_FLOAT_32BIT: return 4;
	case AVTP_AAF_FORMAT_INT_32BIT:   return 4;
	case AVTP_AAF_FORMAT_INT_24BIT:   return 3;
	case AVTP_AAF_FORMAT_INT_16BIT:   return 2;
	case AVTP_AAF_FORMAT_AES3_32BIT:  return 4;
	/* other, not supported */
	case AVTP_AAF_FORMAT_USER:
	default: return 0;
	}
}

/* CVF */
DEF_AVTP_ACCESSER_UINT8(cvf_format, 16)
DEF_AVTP_ACCESSER_UINT8(cvf_format_subtype, 17)
DEF_AVTP_ACCESSER_UINT32(cvf_h264_timestamp, 24)

static inline u32 avtp_get_cvf_m(void *data)
{
	u8 tmp = *((u8 *)(data + 22 + AVTP_OFFSET));

	return (tmp & 0x10) >> 4;
}

static inline void avtp_set_cvf_m(void *data, bool value)
{
	u8 tmp = *((u8 *)(data + 22 + AVTP_OFFSET));

	*((u8 *)(data + 22 + AVTP_OFFSET)) = (tmp & 0xef) | (value << 4);
}

static inline u32 avtp_get_cvf_h264_ptv(void *data)
{
	u8 tmp = *((u8 *)(data + 22 + AVTP_OFFSET));

	return (tmp & 0x20) >> 4;
}

static inline void avtp_set_cvf_h264_ptv(void *data, bool value)
{
	u8 tmp = *((u8 *)(data + 22 + AVTP_OFFSET));

	*((u8 *)(data + 22 + AVTP_OFFSET)) = (tmp & 0xDf) | (value << 5);
}

/* MJPEG */
DEF_AVTP_ACCESSER_UINT8(cvf_mjpeg_tspec, 24)
DEF_AVTP_ACCESSER_UINT8(cvf_mjpeg_type, 28)
DEF_AVTP_ACCESSER_UINT8(cvf_mjpeg_q, 29)
DEF_AVTP_ACCESSER_UINT8(cvf_mjpeg_width, 30)
DEF_AVTP_ACCESSER_UINT8(cvf_mjpeg_height, 31)

static inline u32 avtp_get_cvf_mjpeg_offset(void *data)
{
	u32 tmp;

	tmp = *((u8 *)(data + 25 + AVTP_OFFSET)) << 16;
	tmp |= *((u8 *)(data + 26 + AVTP_OFFSET)) << 8;
	tmp |= *((u8 *)(data + 27 + AVTP_OFFSET));

	return tmp;
}

static inline void avtp_set_cvf_mjpeg_offset(void *data, u32 value)
{
	*((u8 *)(data + 25 + AVTP_OFFSET)) = value >> 16;
	*((u8 *)(data + 26 + AVTP_OFFSET)) = value >> 8;
	*((u8 *)(data + 27 + AVTP_OFFSET)) = value;
}

/* CRF */
DEF_AVTP_ACCESSER_UINT32(crf_base_frequency, 12)
DEF_AVTP_ACCESSER_UINT16(crf_data_length, 16)
DEF_AVTP_ACCESSER_UINT16(crf_timestamp_interval, 18)

/**
 * Template - IEEE1722
 */
extern void avtp_copy_iec61883_4_template(void *data);
extern void avtp_copy_iec61883_6_template(void *data);
extern void avtp_copy_aaf_pcm_template(void *data);
extern void avtp_copy_cvf_h264_template(void *data);
extern void avtp_copy_cvf_mjpeg_template(void *data);
extern void avtp_copy_crf_template(void *data);

#endif /* __AVTP_H__ */
