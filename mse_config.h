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

#ifndef __MSE_CONFIG_H__
#define __MSE_CONFIG_H__

/**
 * @brief configuration data
 */
struct mse_config {
	/** @brief spin lock for config access */
	spinlock_t lock;

	struct mse_info info;
	struct mse_network_device network_device;
	struct mse_packetizer packetizer;
	struct mse_avtp_tx_param avtp_tx_param;
	struct mse_avtp_rx_param avtp_rx_param;
	struct mse_media_audio_config media_audio_config;
	struct mse_media_video_config media_video_config;
	struct mse_media_mpeg2ts_config media_mpeg2ts_config;
	struct mse_ptp_config ptp_config;
	struct mse_mch_config mch_config;
	struct mse_avtp_tx_param avtp_tx_param_crf;
	struct mse_avtp_rx_param avtp_rx_param_crf;
	struct mse_delay_time delay_time;
};

int mse_dev_to_index(struct device *dev);
bool mse_dev_is_busy(int index);
struct mse_config *mse_get_dev_config(int index);
int mse_config_get_info(int index, struct mse_info *data);
int mse_config_set_network_device(int index,
				  struct mse_network_device *data);
int mse_config_get_network_device(int index,
				  struct mse_network_device *data);
int mse_config_set_packetizer(int index, struct mse_packetizer *data);
int mse_config_get_packetizer(int index, struct mse_packetizer *data);
int mse_config_set_avtp_tx_param(int index,
				 struct mse_avtp_tx_param *data);
int mse_config_get_avtp_tx_param(int index,
				 struct mse_avtp_tx_param *data);
int mse_config_set_avtp_rx_param(int index,
				 struct mse_avtp_rx_param *data);
int mse_config_get_avtp_rx_param(int index,
				 struct mse_avtp_rx_param *data);
int mse_config_set_media_audio_config(int index,
				      struct mse_media_audio_config *data);
int mse_config_get_media_audio_config(int index,
				      struct mse_media_audio_config *data);
int mse_config_set_media_video_config(int index,
				      struct mse_media_video_config *data);
int mse_config_get_media_video_config(int index,
				      struct mse_media_video_config *data);
int mse_config_set_media_mpeg2ts_config(int index,
					struct mse_media_mpeg2ts_config *data);
int mse_config_get_media_mpeg2ts_config(int index,
					struct mse_media_mpeg2ts_config *data);
int mse_config_set_ptp_config(int index, struct mse_ptp_config *data);
int mse_config_get_ptp_config(int index, struct mse_ptp_config *data);
int mse_config_set_mch_config(int index, struct mse_mch_config *data);
int mse_config_get_mch_config(int index, struct mse_mch_config *data);
int mse_config_set_avtp_tx_param_crf(int index,
				     struct mse_avtp_tx_param *data);
int mse_config_get_avtp_tx_param_crf(int index,
				     struct mse_avtp_tx_param *data);
int mse_config_set_avtp_rx_param_crf(int index,
				     struct mse_avtp_rx_param *data);
int mse_config_get_avtp_rx_param_crf(int index,
				     struct mse_avtp_rx_param *data);
int mse_config_set_delay_time(int index, struct mse_delay_time *data);
int mse_config_get_delay_time(int index, struct mse_delay_time *data);

#endif /* __MSE_CONFIG_H__ */
