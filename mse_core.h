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

#ifndef __MSE_CORE_H__
#define __MSE_CORE_H__

/**
 * @brief audio stream information
 */
struct mse_audio_info {
	int avtp_packet_size;
	int sample_per_packet;
	int frame_interval_time;
};

/**
 * @brief packetizer status
 */
enum MSE_PACKETIZE_STATUS  {
	MSE_PACKETIZE_STATUS_CONTINUE,
	MSE_PACKETIZE_STATUS_COMPLETE,
	MSE_PACKETIZE_STATUS_MAY_COMPLETE,
	MSE_PACKETIZE_STATUS_NOT_ENOUGH,
};

/**
 * @brief registered operations for packetizer
 */
struct mse_packetizer_ops {
	/** @brief id */
	enum MSE_PACKETIZER id;
	/** @brief open function pointer */
	int (*open)(void);
	/** @brief release function pointer */
	int (*release)(int index);
	/** @brief init function pointer */
	int (*init)(int index);
	/** @brief set network config function pointer */
	int (*set_network_config)(int index,
				  struct mse_network_config *config);
	/** @brief set audio config function pointer */
	int (*set_audio_config)(int index, struct mse_audio_config *config);
	/** @brief set video config function pointer */
	int (*set_video_config)(int index, struct mse_video_config *config);
	/** @brief set mpeg2ts config function pointer */
	int (*set_mpeg2ts_config)(int index,
				  struct mse_mpeg2ts_config *config);
	/** @brief get audio info function pointer */
	int (*get_audio_info)(int index, struct mse_audio_info *info);

	/** @brief calc_cbs function pointer */
	int (*calc_cbs)(int index, struct mse_cbsparam *cbs);

	/** @brief packetize function pointer */
	int (*packetize)(int index,
			 void *packet,
			 size_t *packet_size,
			 void *buffer,
			 size_t buffer_size,
			 size_t *buffer_processed,
			 unsigned int *timestamp);
	/** @brief depacketize function pointer */
	int (*depacketize)(int index,
			   void *buffer,
			   size_t buffer_size,
			   size_t *buffer_processed,
			   unsigned int *timestamp,
			   void *packet,
			   size_t packet_size);
};

static inline int mse_get_bit_depth(enum MSE_AUDIO_BIT bit_depth)
{
	switch (bit_depth) {
	case MSE_AUDIO_BIT_16:
		return 16;
	case MSE_AUDIO_BIT_18:
		return 18;
	case MSE_AUDIO_BIT_20:
		return 20;
	case MSE_AUDIO_BIT_24:
		return 24;
	case MSE_AUDIO_BIT_32:
		return 32;
	default:
		return 0;
	}
}

static inline void mse_make_streamid(u8 *streamid, char *mac, int uid)
{
	streamid[0] = mac[0];
	streamid[1] = mac[1];
	streamid[2] = mac[2];
	streamid[3] = mac[3];
	streamid[4] = mac[4];
	streamid[5] = mac[5];
	streamid[6] = (u8)((uid & 0xff00) >> 8);
	streamid[7] = (u8)((uid & 0x00ff));
}

/**
 * @brief register packetizer to MSE
 *
 * @param[in] ops packetizer operations
 *
 * @retval 0 MSE adapter ID
 * @retval <0 Error
 */
int mse_register_packetizer(struct mse_packetizer_ops *ops);

/**
 * @brief unregister packetizer from MSE
 *
 * @param[in] index MSE adapter ID
 *
 * @retval 0 Success
 * @retval <0 Error
 */
int mse_unregister_packetizer(int index);

#endif /* __MSE_CORE_H__ */
