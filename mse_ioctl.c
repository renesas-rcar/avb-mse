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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/netdevice.h>
#include "ravb_mse_kernel.h"
#include "mse_config.h"
#include "mse_packetizer.h"

/* defines */
struct mse_ioctl_table {
	int index;
	bool used_f;
	bool open_f;
	struct cdev cdev;
};

/* Variables */
static dev_t devt;
static int ioctl_max;
static struct mse_ioctl_table *ioctl_table;

/* Functions */
static int mse_ioctl_open(struct inode *inode, struct file *file)
{
	pr_debug("[%s] minor=%d\n", __func__, iminor(inode));

	if (ioctl_table[iminor(inode)].open_f)
		return -EBUSY;

	ioctl_table[iminor(inode)].open_f = true;

	return 0;
}

static int mse_ioctl_release(struct inode *inode, struct file *file)
{
	pr_debug("[%s] minor=%d\n", __func__, iminor(inode));

	ioctl_table[iminor(inode)].open_f = false;

	return 0;
}

static long mse_ioctl_get_info(struct file *file, unsigned long param)
{
	struct mse_info data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_info(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_network_device(struct file *file,
					 unsigned long param)
{
	struct mse_network_device data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_network_device(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_network_device(struct file *file,
					 unsigned long param)
{
	struct mse_network_device data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_network_device(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_packetizer(struct file *file, unsigned long param)
{
	struct mse_packetizer data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_packetizer(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_packetizer(struct file *file, unsigned long param)
{
	struct mse_packetizer data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_packetizer(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_avtp_tx_param(struct file *file, unsigned long param)
{
	struct mse_avtp_tx_param data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_avtp_tx_param(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_avtp_tx_param(struct file *file, unsigned long param)
{
	struct mse_avtp_tx_param data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_avtp_tx_param(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_avtp_rx_param(struct file *file, unsigned long param)
{
	struct mse_avtp_rx_param data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_avtp_rx_param(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_avtp_rx_param(struct file *file, unsigned long param)
{
	struct mse_avtp_rx_param data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_avtp_rx_param(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_audio_config(struct file *file, unsigned long param)
{
	struct mse_media_audio_config data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_media_audio_config(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_audio_config(struct file *file, unsigned long param)
{
	struct mse_media_audio_config data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_media_audio_config(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_video_config(struct file *file, unsigned long param)
{
	struct mse_media_video_config data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_media_video_config(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_video_config(struct file *file, unsigned long param)
{
	struct mse_media_video_config data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_media_video_config(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_mpeg2ts_config(struct file *file,
					 unsigned long param)
{
	struct mse_media_mpeg2ts_config data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_media_mpeg2ts_config(iminor(file->f_inode),
						   &data);
}

static long mse_ioctl_get_mpeg2ts_config(struct file *file,
					 unsigned long param)
{
	struct mse_media_mpeg2ts_config data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_media_mpeg2ts_config(iminor(file->f_inode),
						  &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_ptp_config(struct file *file, unsigned long param)
{
	struct mse_ptp_config data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_ptp_config(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_ptp_config(struct file *file, unsigned long param)
{
	struct mse_ptp_config data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_ptp_config(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_mch_config(struct file *file, unsigned long param)
{
	struct mse_mch_config data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_mch_config(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_mch_config(struct file *file, unsigned long param)
{
	struct mse_mch_config data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_mch_config(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_avtp_tx_param_crf(struct file *file,
					    unsigned long param)
{
	struct mse_avtp_tx_param data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_avtp_tx_param_crf(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_avtp_tx_param_crf(struct file *file,
					    unsigned long param)
{
	struct mse_avtp_tx_param data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_avtp_tx_param_crf(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_avtp_rx_param_crf(struct file *file,
					    unsigned long param)
{
	struct mse_avtp_rx_param data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_avtp_rx_param_crf(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_avtp_rx_param_crf(struct file *file,
					    unsigned long param)
{
	struct mse_avtp_rx_param data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_avtp_rx_param_crf(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_set_delay_time(struct file *file, unsigned long param)
{
	struct mse_delay_time data;
	char __user *buf = (char __user *)param;

	pr_debug("[%s]\n", __func__);

	if (copy_from_user(&data, buf, sizeof(data)))
		return -EFAULT;

	return mse_config_set_delay_time(iminor(file->f_inode), &data);
}

static long mse_ioctl_get_delay_time(struct file *file, unsigned long param)
{
	struct mse_delay_time data;
	char __user *buf = (char __user *)param;
	int ret;

	pr_debug("[%s]\n", __func__);

	ret = mse_config_get_delay_time(iminor(file->f_inode), &data);
	if (ret)
		return ret;

	if (copy_to_user(buf, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long mse_ioctl_unlocked_ioctl(struct file *file,
				     unsigned int cmd,
				     unsigned long param)
{
	long ret;

	pr_debug("[%s] cmd=0x%08x\n", __func__, cmd);

	if (iminor(file->f_inode) > ioctl_max) {
		pr_err("[%s] illegal minor=0x%08x\n",
		       __func__, iminor(file->f_inode));
		return -EINVAL;
	}

	switch (cmd) {
	case MSE_G_INFO:
		ret = mse_ioctl_get_info(file, param);
		break;
	case MSE_S_NETWORK_DEVICE:
		ret = mse_ioctl_set_network_device(file, param);
		break;
	case MSE_G_NETWORK_DEVICE:
		ret = mse_ioctl_get_network_device(file, param);
		break;
	case MSE_S_PACKETIZER:
		ret = mse_ioctl_set_packetizer(file, param);
		break;
	case MSE_G_PACKETIZER:
		ret = mse_ioctl_get_packetizer(file, param);
		break;
	case MSE_S_AVTP_TX_PARAM:
		ret = mse_ioctl_set_avtp_tx_param(file, param);
		break;
	case MSE_G_AVTP_TX_PARAM:
		ret = mse_ioctl_get_avtp_tx_param(file, param);
		break;
	case MSE_S_AVTP_RX_PARAM:
		ret = mse_ioctl_set_avtp_rx_param(file, param);
		break;
	case MSE_G_AVTP_RX_PARAM:
		ret = mse_ioctl_get_avtp_rx_param(file, param);
		break;
	case MSE_S_MEDIA_AUDIO_CONFIG:
		ret = mse_ioctl_set_audio_config(file, param);
		break;
	case MSE_G_MEDIA_AUDIO_CONFIG:
		ret = mse_ioctl_get_audio_config(file, param);
		break;
	case MSE_S_MEDIA_VIDEO_CONFIG:
		ret = mse_ioctl_set_video_config(file, param);
		break;
	case MSE_G_MEDIA_VIDEO_CONFIG:
		ret = mse_ioctl_get_video_config(file, param);
		break;
	case MSE_S_MEDIA_MPEG2TS_CONFIG:
		ret = mse_ioctl_set_mpeg2ts_config(file, param);
		break;
	case MSE_G_MEDIA_MPEG2TS_CONFIG:
		ret = mse_ioctl_get_mpeg2ts_config(file, param);
		break;
	case MSE_S_PTP_CONFIG:
		ret = mse_ioctl_set_ptp_config(file, param);
		break;
	case MSE_G_PTP_CONFIG:
		ret = mse_ioctl_get_ptp_config(file, param);
		break;
	case MSE_S_MCH_CONFIG:
		ret = mse_ioctl_set_mch_config(file, param);
		break;
	case MSE_G_MCH_CONFIG:
		ret = mse_ioctl_get_mch_config(file, param);
		break;
	case MSE_S_AVTP_TX_PARAM_CRF:
		ret = mse_ioctl_set_avtp_tx_param_crf(file, param);
		break;
	case MSE_G_AVTP_TX_PARAM_CRF:
		ret = mse_ioctl_get_avtp_tx_param_crf(file, param);
		break;
	case MSE_S_AVTP_RX_PARAM_CRF:
		ret = mse_ioctl_set_avtp_rx_param_crf(file, param);
		break;
	case MSE_G_AVTP_RX_PARAM_CRF:
		ret = mse_ioctl_get_avtp_rx_param_crf(file, param);
		break;
	case MSE_S_DELAY_TIME:
		ret = mse_ioctl_set_delay_time(file, param);
		break;
	case MSE_G_DELAY_TIME:
		ret = mse_ioctl_get_delay_time(file, param);
		break;
	default:
		pr_err("[%s] illegal cmd=0x%08x\n", __func__, cmd);
		return -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mse_ioctl_compat_ioctl(struct file *file,
				   unsigned int cmd,
				   unsigned long param)
{
	pr_debug("[%s] cmd=0x%08x\n", __func__, cmd);

	return mse_ioctl_unlocked_ioctl(file, cmd, param);
}
#endif

static const struct file_operations ioctl_fops = {
	.owner		= THIS_MODULE,
	.open		= mse_ioctl_open,
	.release	= mse_ioctl_release,
	.unlocked_ioctl	= mse_ioctl_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= mse_ioctl_compat_ioctl,
#endif
};

/* External function */
int mse_ioctl_register(int index)
{
	struct cdev *cdev;
	int err;

	if (ioctl_table[index].used_f) {
		pr_err("[%s] failed /dev/mse%d\n", __func__, index);
		return -EINVAL;
	}

	pr_debug("[%s] resister /dev/mse%d\n", __func__, index);

	/* initialize cdev */
	cdev = &ioctl_table[index].cdev;
	cdev_init(cdev, &ioctl_fops);
	cdev->owner = THIS_MODULE;

	err = cdev_add(cdev, devt, ioctl_max);
	if (err) {
		pr_err("[%s] failed cdev_add()\n", __func__);
		return err;
	}

	pr_debug("[%s] success!\n", __func__);
	ioctl_table[index].index = index;
	ioctl_table[index].used_f = true;

	return index;
}

void mse_ioctl_unregister(int index)
{
	pr_debug("[%s]\n", __func__);

	cdev_del(&ioctl_table[index].cdev);
	ioctl_table[index].used_f = false;
}

int mse_ioctl_init(int major, int mse_instance_max)
{
	int err;

	pr_debug("[%s] start\n", __func__);

	ioctl_table = kcalloc(mse_instance_max, sizeof(*ioctl_table),
			      GFP_KERNEL);
	if (!ioctl_table)
		return -ENOMEM;

	/* register chrdev */
	if (major) {
		devt = MKDEV(major, 0);
		err = register_chrdev_region(devt, mse_instance_max, "mse");
	} else {
		err = alloc_chrdev_region(&devt, 0, mse_instance_max, "mse");
	}

	if (err < 0) {
		pr_err("[%s] failed _chrdev_region() ret=%d\n", __func__, err);
		return err;
	}

	ioctl_max = mse_instance_max;

	return MAJOR(devt);
}

void mse_ioctl_exit(int major, int mse_instance_max)
{
	pr_debug("[%s]\n", __func__);

	unregister_chrdev_region(MKDEV(major, 0), mse_instance_max);
	kfree(ioctl_table);
	ioctl_max = 0;
}
