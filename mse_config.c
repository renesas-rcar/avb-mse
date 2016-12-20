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

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME "/" fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include "ravb_mse_kernel.h"
#include "mse_config.h"
#include "mse_core.h"

int mse_config_get_info(int index, struct mse_info *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->info;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_network_device(int index, struct mse_network_device *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	config->network_device = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_get_network_device(int index, struct mse_network_device *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->network_device;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_packetizer(int index, struct mse_packetizer *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	switch (config->info.type) {
	case MSE_STREAM_TYPE_AUDIO:
		if ((data->packetizer != MSE_PACKETIZER_AAF_PCM) &&
		    (data->packetizer != MSE_PACKETIZER_IEC61883_6))
			goto wrong_combination;
		break;

	case MSE_STREAM_TYPE_VIDEO:
		if ((data->packetizer != MSE_PACKETIZER_CVF_H264) &&
		    (data->packetizer != MSE_PACKETIZER_CVF_H264_D13) &&
		    (data->packetizer != MSE_PACKETIZER_CVF_MJPEG))
			goto wrong_combination;
		break;

	case MSE_STREAM_TYPE_MPEG2TS:
		if (data->packetizer != MSE_PACKETIZER_IEC61883_4)
			goto wrong_combination;
		break;

	default:
		pr_err("[%s] invalid value. packetizer=%d\n",
		       __func__, data->packetizer);
		return -EINVAL;
	}

	spin_lock_irqsave(&config->lock, flags);
	config->packetizer = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_combination:
	pr_err("[%s] invalid combination. type=%d, packetizer=%d\n",
	       __func__, config->info.type, data->packetizer);
	return -EINVAL;
}

int mse_config_get_packetizer(int index, struct mse_packetizer *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->packetizer;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_avtp_tx_param(int index, struct mse_avtp_tx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if (data->vlan > MSE_CONFIG_VLAN_MAX)
		goto wrong_value;

	if (data->priority > MSE_CONFIG_PRIORITY_MAX)
		goto wrong_value;

	if (data->uniqueid > MSE_CONFIG_UNIQUEID_MAX)
		goto wrong_value;

	spin_lock_irqsave(&config->lock, flags);
	config->avtp_tx_param = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. vlan=%d, priority=%d, uniqueid=%d\n",
	       __func__, data->vlan, data->priority, data->uniqueid);

	return -EINVAL;
}

int mse_config_get_avtp_tx_param(int index, struct mse_avtp_tx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->avtp_tx_param;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_avtp_rx_param(int index, struct mse_avtp_rx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	spin_lock_irqsave(&config->lock, flags);
	config->avtp_rx_param = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_get_avtp_rx_param(int index, struct mse_avtp_rx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->avtp_rx_param;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_media_audio_config(int index,
				      struct mse_media_audio_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if (data->samples_per_frame > MSE_CONFIG_SAMPLE_PER_FRAME_MAX)
		goto wrong_value;

	if ((data->crf_type < 0) || (data->crf_type >= MSE_CRF_TYPE_MAX))
		goto wrong_value;

	if ((data->crf_type == MSE_CRF_TYPE_TX) && config->mch_config.enable) {
		pr_err("[%s] invalid combination. mch=%d, crf_type=%d\n",
		       __func__, config->mch_config.enable, data->crf_type);
		return -EINVAL;
	}

	spin_lock_irqsave(&config->lock, flags);
	config->media_audio_config = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. samples_per_frame=%d, crf_type=%d\n",
	       __func__, data->samples_per_frame, data->crf_type);
	return -EINVAL;
}

int mse_config_get_media_audio_config(int index,
				      struct mse_media_audio_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->media_audio_config;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_media_video_config(int index,
				      struct mse_media_video_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_VIDEO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if (data->bytes_per_frame > MSE_CONFIG_BYTES_PER_FRAME_MAX)
		goto wrong_value;

	if (data->fps_denominator > MSE_CONFIG_FPS_DENOMINATOR_MAX)
		goto wrong_value;

	if (data->fps_numerator > MSE_CONFIG_FPS_NUMERATOR_MAX)
		goto wrong_value;

	if (data->bitrate < MSE_CONFIG_BITRATE_MIN)
		goto wrong_value;

	spin_lock_irqsave(&config->lock, flags);
	config->media_video_config = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. bytes_per_frame=%d fps=%d/%d bitrate=%d\n",
	       __func__, data->bytes_per_frame, data->fps_numerator,
	       data->fps_denominator, data->bitrate);

	return -EINVAL;
}

int mse_config_get_media_video_config(int index,
				      struct mse_media_video_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_VIDEO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->media_video_config;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_media_mpeg2ts_config(int index,
					struct mse_media_mpeg2ts_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_MPEG2TS) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if ((data->tspackets_per_frame < MSE_CONFIG_TSPACKET_PER_FRAME_MIN) ||
	    (data->tspackets_per_frame > MSE_CONFIG_TSPACKET_PER_FRAME_MAX))
		goto wrong_value;

	if (data->bitrate < MSE_CONFIG_BITRATE_MIN)
		goto wrong_value;

	if (data->pcr_pid > MSE_CONFIG_PCR_PID_MAX)
		goto wrong_value;

	spin_lock_irqsave(&config->lock, flags);
	config->media_mpeg2ts_config = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. bitrate=%d pcr_pid=%d\n",
	       __func__, data->bitrate, data->pcr_pid);
	return -EINVAL;
}

int mse_config_get_media_mpeg2ts_config(int index,
					struct mse_media_mpeg2ts_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_MPEG2TS) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->media_mpeg2ts_config;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_ptp_config(int index, struct mse_ptp_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if ((data->type < 0) || (data->type >= MSE_PTP_TYPE_MAX))
		goto wrong_value;

	if (data->capture_ch > MSE_CONFIG_CAPTURE_CH_MAX)
		goto wrong_value;

	if ((data->recovery_capture_freq < 0) ||
	    (data->recovery_capture_freq >= MSE_RECOVERY_CAPTURE_FREQ_MAX))
		goto wrong_value;

	spin_lock_irqsave(&config->lock, flags);
	config->ptp_config = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. type=%d deviceid=%d capture_ch=%d capture_freq=%d recovery_capture_freq=%d\n",
	       __func__, data->type, data->deviceid, data->capture_ch,
	       data->capture_freq, data->recovery_capture_freq);
	return -EINVAL;
}

int mse_config_get_ptp_config(int index, struct mse_ptp_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->ptp_config;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_mch_config(int index, struct mse_mch_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if (data->enable &&
	    (config->media_audio_config.crf_type == MSE_CRF_TYPE_TX)) {
		pr_err("[%s] invalid combination. enable=%d PTP type=%d\n",
		       __func__, data->enable, config->ptp_config.type);
		return -EINVAL;
	}

	spin_lock_irqsave(&config->lock, flags);
	config->mch_config = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_get_mch_config(int index, struct mse_mch_config *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->mch_config;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_avtp_tx_param_crf(int index, struct mse_avtp_tx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	if (data->vlan > MSE_CONFIG_VLAN_MAX)
		goto wrong_value;

	if (data->priority > MSE_CONFIG_PRIORITY_MAX)
		goto wrong_value;

	if (data->uniqueid > MSE_CONFIG_UNIQUEID_MAX)
		goto wrong_value;

	spin_lock_irqsave(&config->lock, flags);
	config->avtp_tx_param_crf = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;

wrong_value:
	pr_err("[%s] invalid value. vlan=%d, priority=%d, uniqueid=%d\n",
	       __func__, data->vlan, data->priority, data->uniqueid);

	return -EINVAL;
}

int mse_config_get_avtp_tx_param_crf(int index, struct mse_avtp_tx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->avtp_tx_param_crf;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_avtp_rx_param_crf(int index, struct mse_avtp_rx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	config->avtp_rx_param_crf = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_get_avtp_rx_param_crf(int index, struct mse_avtp_rx_param *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (config->info.type != MSE_STREAM_TYPE_AUDIO) {
		pr_err("[%s] mse%d does not permit.\n", __func__, index);
		return -EPERM;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->avtp_rx_param_crf;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_set_delay_time(int index, struct mse_delay_time *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	if (mse_dev_is_busy(index)) {
		pr_err("[%s] mse%d is running.\n", __func__, index);
		return -EBUSY;
	}

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	config->delay_time = *data;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

int mse_config_get_delay_time(int index, struct mse_delay_time *data)
{
	struct mse_config *config;
	unsigned long flags;

	if ((index < 0) || (index >= MSE_ADAPTER_MEDIA_MAX)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, index);
		return -EINVAL;
	}
	config = mse_get_dev_config(index);

	pr_debug("[%s]\n", __func__);

	spin_lock_irqsave(&config->lock, flags);
	*data = config->delay_time;
	spin_unlock_irqrestore(&config->lock, flags);

	return 0;
}

