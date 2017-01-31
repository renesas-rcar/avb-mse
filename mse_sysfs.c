/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2017 Renesas Electronics Corporation

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
#include <linux/of_device.h>

#include "ravb_mse_kernel.h"
#include "mse_sysfs.h"
#include "mse_config.h"

/* macro */
#define __MSE_ATTR(_name, _mode, _show, _store) { \
	.attr = { .name = __stringify(_name), .mode = _mode }, \
	.show = _show, \
	.store = _store, \
}

#define __MSE_ATTR_RO(_name, _group) { \
	.attr = { .name = __stringify(_name), .mode = 0444 }, \
	.show = mse_##_group##_##_name##_show, \
}

#define __MSE_ATTR_RW(_name, _group) { \
	.attr = { .name = __stringify(_name), .mode = 0644 }, \
	.show = mse_##_group##_##_name##_show, \
	.store = mse_##_group##_##_name##_store, \
}

#define MSE_DEVICE_ATTR(_name, _group, _mode, _show, _store) \
	struct device_attribute mse_dev_attr_##_group##_##_name = \
					__MSE_ATTR(_name, _mode, _show, _store)
#define MSE_DEVICE_ATTR_RW(_name, _group) \
	struct device_attribute mse_dev_attr_##_group##_##_name = \
						__MSE_ATTR_RW(_name, _group)
#define MSE_DEVICE_ATTR_RO(_name, _group) \
	struct device_attribute mse_dev_attr_##_group##_##_name = \
						__MSE_ATTR_RO(_name, _group)

/* define */
#define MSE_RADIX_HEXADECIMAL   (16)
#define MSE_NAME_LEN_MAX        (32)
#define MSE_MAC_STR_LEN_MAX     (12)

#define MSE_SYSFS_NAME_STR_MODULE_NAME               "module_name"
#define MSE_SYSFS_NAME_STR_DEVICE_NAME_TX            "device_name_tx"
#define MSE_SYSFS_NAME_STR_DEVICE_NAME_RX            "device_name_rx"
#define MSE_SYSFS_NAME_STR_DEVICE_NAME_TX_CRF        "device_name_tx_crf"
#define MSE_SYSFS_NAME_STR_DEVICE_NAME_RX_CRF        "device_name_rx_crf"
#define MSE_SYSFS_NAME_STR_VLAN                      "vlan"
#define MSE_SYSFS_NAME_STR_PRIORITY                  "priority"
#define MSE_SYSFS_NAME_STR_UNIQUEID                  "uniqueid"
#define MSE_SYSFS_NAME_STR_SAMPLES_PER_FRAME         "samples_per_frame"
#define MSE_SYSFS_NAME_STR_BYTES_PER_FRAME           "bytes_per_frame"
#define MSE_SYSFS_NAME_STR_FPS_DENOMINATOR           "fps_denominator"
#define MSE_SYSFS_NAME_STR_FPS_NUMERATOR             "fps_numerator"
#define MSE_SYSFS_NAME_STR_BITRATE                   "bitrate"
#define MSE_SYSFS_NAME_STR_TSPACKETS_PER_FRAME       "tspackets_per_frame"
#define MSE_SYSFS_NAME_STR_PCR_PID                   "pcr_pid"
#define MSE_SYSFS_NAME_STR_DEVICEID                  "deviceid"
#define MSE_SYSFS_NAME_STR_CAPTURE_CH                "capture_ch"
#define MSE_SYSFS_NAME_STR_CAPTURE_FREQ              "capture_freq"
#define MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ     "recovery_capture_freq"
#define MSE_SYSFS_NAME_STR_ENABLE                    "enable"
#define MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME_NS       "max_transit_time_ns"
#define MSE_SYSFS_NAME_STR_TX_DELAY_TIME_NS          "tx_delay_time_ns"
#define MSE_SYSFS_NAME_STR_RX_DELAY_TIME_NS          "rx_delay_time_ns"

struct convert_table {
	int id;
	char str[32];
};

/* variables */
static struct convert_table type_table[] = {
	{
		MSE_STREAM_TYPE_AUDIO,
		"audio",
	},
	{
		MSE_STREAM_TYPE_VIDEO,
		"video",
	},
	{
		MSE_STREAM_TYPE_MPEG2TS,
		"mpeg2ts",
	},
};

static struct convert_table packetizer_table[] = {
	{
		MSE_PACKETIZER_AAF_PCM,
		"aaf_pcm",
	},
	{
		MSE_PACKETIZER_IEC61883_6,
		"iec61883-6",
	},
	{
		MSE_PACKETIZER_CVF_H264_D13,
		"cvf_h264_d13",
	},
	{
		MSE_PACKETIZER_CVF_H264,
		"cvf_h264",
	},
	{
		MSE_PACKETIZER_CVF_MJPEG,
		"cvf_mjpeg",
	},
	{
		MSE_PACKETIZER_IEC61883_4,
		"iec61883-4",
	},
};

static struct convert_table crf_table[] = {
	{
		MSE_CRF_TYPE_NOT_USE,
		"not use",
	},
	{
		MSE_CRF_TYPE_RX,
		"rx",
	},
	{
		MSE_CRF_TYPE_TX,
		"tx",
	},
};

static struct convert_table ptp_type_table[] = {
	{
		MSE_PTP_TYPE_CURRENT_TIME,
		"ptp",
	},
	{
		MSE_PTP_TYPE_CAPTURE,
		"capture",
	},
};

/* function */
static int strtobin(unsigned char *dest, const char *in_str, int len)
{
	union {
		unsigned long long hex;
		unsigned char byte[sizeof(unsigned long long)];
	} decode;
	int err, i, j;

	err = kstrtoll(in_str, MSE_RADIX_HEXADECIMAL, &decode.hex);
	if (err < 0)
		return err;

	for (i = 0, j = len - 1; i < len; i++, j--)
		dest[i] = decode.byte[j];

	return err;
}

static ssize_t mse_sysfs_strncpy_from_user(char *dst,
					   const char *src,
					   ssize_t size)
{
	char **value = &dst;
	char *value2;

	strncpy(dst, src, size);
	if (*(dst + size - 1) != '\0' &&
	    *(dst + size - 1) != '\n')
		return -EINVAL;

	value2 = strsep(value, "\n");

	return strlen(value2);
}

static ssize_t mse_info_device_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct mse_info data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_info(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%s\n", data.device);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_info_type_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct mse_info data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_info(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_STREAM_TYPE_MAX; i++) {
		if (data.type == type_table[i].id)
			break;
	}
	if (i == MSE_STREAM_TYPE_MAX)
		return -EPERM;

	ret = sprintf(buf, "%s\n", type_table[i].str);

	mse_debug("END value=%s(%d) ret=%d\n", buf, data.type, ret);

	return ret;
}

static ssize_t mse_network_device_str_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct mse_network_device data;
	int index = mse_dev_to_index(dev);
	int ret;
	char *value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_network_device(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_MODULE_NAME,
		     strlen(attr->attr.name)))
		value = data.module_name;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICE_NAME_TX,
			  strlen(attr->attr.name)))
		value = data.device_name_tx;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICE_NAME_RX,
			  strlen(attr->attr.name)))
		value = data.device_name_rx;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_DEVICE_NAME_TX_CRF,
			  strlen(attr->attr.name)))
		value = data.device_name_tx_crf;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_DEVICE_NAME_RX_CRF,
			  strlen(attr->attr.name)))
		value = data.device_name_rx_crf;
	else
		return -EPERM;

	ret = sprintf(buf, "%s\n", value);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_network_device_str_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t len)
{
	struct mse_network_device data;
	int index = mse_dev_to_index(dev);
	int ret, cplen;
	char *value, *value_org;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = mse_config_get_network_device(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_MODULE_NAME,
		     strlen(attr->attr.name)))
		value = data.module_name;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICE_NAME_TX,
			  strlen(attr->attr.name)))
		value = data.device_name_tx;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICE_NAME_RX,
			  strlen(attr->attr.name)))
		value = data.device_name_rx;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_DEVICE_NAME_TX_CRF,
			  strlen(attr->attr.name)))
		value = data.device_name_tx_crf;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_DEVICE_NAME_RX_CRF,
			  strlen(attr->attr.name)))
		value = data.device_name_rx_crf;
	else
		return -EPERM;

	if (len > MSE_NAME_LEN_MAX)
		cplen = MSE_NAME_LEN_MAX;
	else
		cplen = strlen(buf);

	memset(value, 0, MSE_NAME_LEN_MAX);
	strncpy(value, buf, cplen);
	value_org = value;
	strsep(&value, "\n");

	ret = mse_config_set_network_device(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%s ret=%d\n", value_org, cplen);

	return len;
}

static ssize_t mse_packetizer_name_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mse_packetizer data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_packetizer(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_PACKETIZER_MAX; i++) {
		if (data.packetizer == packetizer_table[i].id)
			break;
	}
	if (i == MSE_PACKETIZER_MAX)
		return -EPERM;

	ret = sprintf(buf, "%s\n", packetizer_table[i].str);

	mse_debug("END value=%s(%d) ret=%d\n", buf, data.packetizer, ret);

	return ret;
}

static ssize_t mse_packetizer_name_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct mse_packetizer data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	for (i = 0; i < MSE_PACKETIZER_MAX; i++) {
		if (!strncmp(buf, packetizer_table[i].str,
			     strlen(packetizer_table[i].str))) {
			data.packetizer = packetizer_table[i].id;
			break;
		}
	}
	if (i == MSE_PACKETIZER_MAX)
		return -EINVAL;

	ret = mse_config_set_packetizer(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", data.packetizer, len);

	return len;
}

static ssize_t mse_avtp_tx_dst_mac_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x\n",
		      data.dst_mac[0], data.dst_mac[1], data.dst_mac[2],
		      data.dst_mac[3], data.dst_mac[4], data.dst_mac[5]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_dst_mac_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;
	char buf2[MSE_MAC_STR_LEN_MAX + 1];

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	if (len > sizeof(buf2))
		return -EINVAL;

	ret = mse_sysfs_strncpy_from_user(buf2, buf, sizeof(buf2));
	if (ret != MSE_MAC_STR_LEN_MAX)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	ret = strtobin(data.dst_mac, buf2, MSE_MAC_LEN_MAX);
	if (ret < 0)
		return -EINVAL;

	ret =  mse_config_set_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.dst_mac[0], data.dst_mac[1], data.dst_mac[2],
		  data.dst_mac[3], data.dst_mac[4], data.dst_mac[5], len);

	return len;
}

static ssize_t mse_avtp_tx_src_mac_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x\n",
		      data.src_mac[0], data.src_mac[1], data.src_mac[2],
		      data.src_mac[3], data.src_mac[4], data.src_mac[5]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_src_mac_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;
	char buf2[MSE_MAC_STR_LEN_MAX + 1];

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	if (len > sizeof(buf2))
		return -EINVAL;

	ret = mse_sysfs_strncpy_from_user(buf2, buf, sizeof(buf2));
	if (ret != MSE_MAC_STR_LEN_MAX)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	ret = strtobin(data.src_mac, buf2, MSE_MAC_LEN_MAX);
	if (ret < 0)
		return -EINVAL;

	ret =  mse_config_set_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.src_mac[0], data.src_mac[1], data.src_mac[2],
		  data.src_mac[3], data.src_mac[4], data.src_mac[5], len);

	return len;
}

static ssize_t mse_avtp_tx_int_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_VLAN,
		     strlen(attr->attr.name)))
		value = data.vlan;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PRIORITY,
			  strlen(attr->attr.name)))
		value = data.priority;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_UNIQUEID,
			  strlen(attr->attr.name)))
		value = data.uniqueid;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_int_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_VLAN,
		     strlen(attr->attr.name)))
		data.vlan = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PRIORITY,
			  strlen(attr->attr.name)))
		data.priority = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_UNIQUEID,
			  strlen(attr->attr.name)))
		data.uniqueid = value;
	else
		return -EPERM;

	ret =  mse_config_set_avtp_tx_param(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_avtp_rx_streamid_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct mse_avtp_rx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_rx_param(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		      data.streamid[0], data.streamid[1], data.streamid[2],
		      data.streamid[3], data.streamid[4], data.streamid[5],
		      data.streamid[6], data.streamid[7]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_rx_streamid_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	struct mse_avtp_rx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = mse_config_get_avtp_rx_param(index, &data);
	if (ret)
		return ret;

	strtobin(data.streamid, buf, sizeof(u64));

	ret =  mse_config_set_avtp_rx_param(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.streamid[0], data.streamid[1], data.streamid[2],
		  data.streamid[3], data.streamid[4], data.streamid[5],
		  data.streamid[6], data.streamid[7], len);

	return len;
}

static ssize_t mse_audio_config_int_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct mse_media_audio_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_media_audio_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_SAMPLES_PER_FRAME,
		     strlen(attr->attr.name)))
		value = data.samples_per_frame;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%d ret=%d\n", value, ret);

	return ret;
}

static ssize_t mse_audio_config_int_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	struct mse_media_audio_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_media_audio_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_SAMPLES_PER_FRAME,
		     strlen(attr->attr.name)))
		data.samples_per_frame = value;
	else
		return -EPERM;

	ret =  mse_config_set_media_audio_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_audio_config_crf_type_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mse_media_audio_config data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_media_audio_config(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_CRF_TYPE_MAX; i++) {
		if (data.crf_type == crf_table[i].id)
			break;
	}
	if (i == MSE_CRF_TYPE_MAX)
		return -EPERM;

	ret = sprintf(buf, "%s\n", crf_table[i].str);

	mse_debug("END value=%s(%d) ret=%d\n", buf, data.crf_type, ret);

	return ret;
}

static ssize_t mse_audio_config_crf_type_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf,
					       size_t len)
{
	struct mse_media_audio_config data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = mse_config_get_media_audio_config(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_CRF_TYPE_MAX; i++) {
		if (!strncmp(buf, crf_table[i].str,
			     strlen(crf_table[i].str))) {
			data.crf_type = crf_table[i].id;
			break;
		}
	}
	if (i == MSE_CRF_TYPE_MAX)
		return -EINVAL;

	ret = mse_config_set_media_audio_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%s(%d) ret=%zd\n", buf, data.crf_type, len);

	return len;
}

static ssize_t mse_video_config_int_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct mse_media_video_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_media_video_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BYTES_PER_FRAME,
		     strlen(attr->attr.name)))
		value = data.bytes_per_frame;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_FPS_DENOMINATOR,
			  strlen(attr->attr.name)))
		value = data.fps_denominator;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_FPS_NUMERATOR,
			  strlen(attr->attr.name)))
		value = data.fps_numerator;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BITRATE,
			  strlen(attr->attr.name)))
		value = data.bitrate;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END %d ret:%d\n", value, ret);

	return ret;
}

static ssize_t mse_video_config_int_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	struct mse_media_video_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_media_video_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BYTES_PER_FRAME,
		     strlen(attr->attr.name)))
		data.bytes_per_frame = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_FPS_DENOMINATOR,
			  strlen(attr->attr.name)))
		data.fps_denominator = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_FPS_NUMERATOR,
			  strlen(attr->attr.name)))
		data.fps_numerator = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BITRATE,
			  strlen(attr->attr.name)))
		data.bitrate = value;
	else
		return -EPERM;

	ret =  mse_config_set_media_video_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_mpeg2ts_config_int_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct mse_media_mpeg2ts_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_media_mpeg2ts_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_TSPACKETS_PER_FRAME,
		     strlen(attr->attr.name)))
		value = data.tspackets_per_frame;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BITRATE,
			  strlen(attr->attr.name)))
		value = data.bitrate;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PCR_PID,
			  strlen(attr->attr.name)))
		value = data.pcr_pid;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%d ret=%d\n", value, ret);

	return ret;
}

static ssize_t mse_mpeg2ts_config_int_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t len)
{
	struct mse_media_mpeg2ts_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_media_mpeg2ts_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_TSPACKETS_PER_FRAME,
		     strlen(attr->attr.name)))
		data.tspackets_per_frame = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_BITRATE,
			  strlen(attr->attr.name)))
		data.bitrate = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PCR_PID,
			  strlen(attr->attr.name)))
		data.pcr_pid = value;
	else
		return -EPERM;

	ret =  mse_config_set_media_mpeg2ts_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_ptp_config_type_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mse_ptp_config data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_ptp_config(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_PTP_TYPE_MAX; i++) {
		if (data.type == ptp_type_table[i].id)
			break;
	}
	if (i == MSE_PTP_TYPE_MAX)
		return -EPERM;

	ret = sprintf(buf, "%s\n", ptp_type_table[i].str);

	mse_debug("END value=%s(%d) ret=%d\n", buf, data.type, ret);

	return ret;
}

static ssize_t mse_ptp_config_type_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct mse_ptp_config data;
	int index = mse_dev_to_index(dev);
	int i, ret;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = mse_config_get_ptp_config(index, &data);
	if (ret)
		return ret;

	for (i = 0; i < MSE_PTP_TYPE_MAX; i++) {
		if (!strncmp(buf, ptp_type_table[i].str,
			     strlen(ptp_type_table[i].str))) {
			data.type = ptp_type_table[i].id;
			break;
		}
	}
	if (i == MSE_PTP_TYPE_MAX)
		return -EINVAL;

	ret = mse_config_set_ptp_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%s(%d) ret=%zd\n", buf, data.type, len);

	return len;
}

static ssize_t mse_ptp_config_int_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct mse_ptp_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_ptp_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICEID,
		     strlen(attr->attr.name)))
		value = data.deviceid;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_CAPTURE_CH,
			  strlen(attr->attr.name)))
		value = data.capture_ch;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_CAPTURE_FREQ,
			  strlen(attr->attr.name)))
		value = data.capture_freq;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ,
			  strlen(attr->attr.name)))
		value = data.recovery_capture_freq;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%d ret=%d\n", value, ret);

	return ret;
}

static ssize_t mse_ptp_config_int_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	struct mse_ptp_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_ptp_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_DEVICEID,
		     strlen(attr->attr.name)))
		data.deviceid = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_CAPTURE_CH,
			  strlen(attr->attr.name)))
		data.capture_ch = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_CAPTURE_FREQ,
			  strlen(attr->attr.name)))
		data.capture_freq = value;
	else if (!strncmp(attr->attr.name,
			  MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ,
			  strlen(attr->attr.name)))
		data.recovery_capture_freq = value;
	else
		return -EPERM;

	ret =  mse_config_set_ptp_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_mch_config_int_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct mse_mch_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_mch_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_ENABLE,
		     strlen(attr->attr.name)))
		value = data.enable;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%d ret=%d\n", value, ret);

	return ret;
}

static ssize_t mse_mch_config_int_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	struct mse_mch_config data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_mch_config(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_ENABLE,
		     strlen(attr->attr.name)))
		data.enable = value;
	else
		return -EPERM;

	ret =  mse_config_set_mch_config(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_avtp_tx_crf_dst_mac_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x\n",
		      data.dst_mac[0], data.dst_mac[1], data.dst_mac[2],
		      data.dst_mac[3], data.dst_mac[4], data.dst_mac[5]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_crf_dst_mac_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf,
					     size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;
	char buf2[MSE_MAC_STR_LEN_MAX + 1];

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	if (len > sizeof(buf2))
		return -EINVAL;

	ret = mse_sysfs_strncpy_from_user(buf2, buf, sizeof(buf2));
	if (ret != MSE_MAC_STR_LEN_MAX)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	ret = strtobin(data.dst_mac, buf2, MSE_MAC_LEN_MAX);
	if (ret < 0)
		return -EINVAL;

	ret =  mse_config_set_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.dst_mac[0], data.dst_mac[1], data.dst_mac[2],
		  data.dst_mac[3], data.dst_mac[4], data.dst_mac[5], len);

	return len;
}

static ssize_t mse_avtp_tx_crf_src_mac_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x\n",
		      data.src_mac[0], data.src_mac[1], data.src_mac[2],
		      data.src_mac[3], data.src_mac[4], data.src_mac[5]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_crf_src_mac_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf,
					     size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret;
	char buf2[MSE_MAC_STR_LEN_MAX + 1];

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	if (len > sizeof(buf2))
		return -EINVAL;

	ret = mse_sysfs_strncpy_from_user(buf2, buf, sizeof(buf2));
	if (ret != MSE_MAC_STR_LEN_MAX)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	ret = strtobin(data.src_mac, buf2, MSE_MAC_LEN_MAX);
	if (ret < 0)
		return -EINVAL;

	ret =  mse_config_set_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.src_mac[0], data.src_mac[1], data.src_mac[2],
		  data.src_mac[3], data.src_mac[4], data.src_mac[5], len);

	return len;
}

static ssize_t mse_avtp_tx_crf_int_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_VLAN,
		     strlen(attr->attr.name)))
		value = data.vlan;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PRIORITY,
			  strlen(attr->attr.name)))
		value = data.priority;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_UNIQUEID,
			  strlen(attr->attr.name)))
		value = data.uniqueid;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_tx_crf_int_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct mse_avtp_tx_param data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_VLAN,
		     strlen(attr->attr.name)))
		data.vlan = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_PRIORITY,
			  strlen(attr->attr.name)))
		data.priority = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_UNIQUEID,
			  strlen(attr->attr.name)))
		data.uniqueid = value;
	else
		return -EPERM;

	ret =  mse_config_set_avtp_tx_param_crf(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

static ssize_t mse_avtp_rx_crf_streamid_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mse_avtp_rx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_avtp_rx_param_crf(index, &data);
	if (ret)
		return ret;

	ret = sprintf(buf, "%02x%02x%02x%02x%02x%02x%02x%02x\n",
		      data.streamid[0], data.streamid[1], data.streamid[2],
		      data.streamid[3], data.streamid[4], data.streamid[5],
		      data.streamid[6], data.streamid[7]);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_avtp_rx_crf_streamid_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf,
					      size_t len)
{
	struct mse_avtp_rx_param data;
	int index = mse_dev_to_index(dev);
	int ret;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = mse_config_get_avtp_rx_param_crf(index, &data);
	if (ret)
		return ret;

	strtobin(data.streamid, buf, sizeof(u64));

	ret =  mse_config_set_avtp_rx_param_crf(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%02x%02x%02x%02x%02x%02x%02x%02x ret=%zd\n",
		  data.streamid[0], data.streamid[1], data.streamid[2],
		  data.streamid[3], data.streamid[4], data.streamid[5],
		  data.streamid[6], data.streamid[7], len);

	return len;
}

static ssize_t mse_delay_time_int_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct mse_delay_time data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s\n", attr->attr.name);

	ret = mse_config_get_delay_time(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME_NS,
		     strlen(attr->attr.name)))
		value = data.max_transit_time_ns;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_TX_DELAY_TIME_NS,
			  strlen(attr->attr.name)))
		value = data.tx_delay_time_ns;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_RX_DELAY_TIME_NS,
			  strlen(attr->attr.name)))
		value = data.rx_delay_time_ns;
	else
		return -EPERM;

	ret = sprintf(buf, "%d\n", value);

	mse_debug("END value=%s ret=%d\n", buf, ret);

	return ret;
}

static ssize_t mse_delay_time_int_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t len)
{
	struct mse_delay_time data;
	int index = mse_dev_to_index(dev);
	int ret, value;

	mse_debug("START %s(%zd) to %s\n", buf, len, attr->attr.name);

	ret = kstrtol(buf, 0, (long *)&value);
	if (ret)
		return -EINVAL;

	ret = mse_config_get_delay_time(index, &data);
	if (ret)
		return ret;

	if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME_NS,
		     strlen(attr->attr.name)))
		data.max_transit_time_ns = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_TX_DELAY_TIME_NS,
			  strlen(attr->attr.name)))
		data.tx_delay_time_ns = value;
	else if (!strncmp(attr->attr.name, MSE_SYSFS_NAME_STR_RX_DELAY_TIME_NS,
			  strlen(attr->attr.name)))
		data.rx_delay_time_ns = value;
	else
		return -EPERM;

	ret =  mse_config_set_delay_time(index, &data);
	if (ret)
		return ret;

	mse_debug("END value=%d ret=%zd\n", value, len);

	return len;
}

/* attribute variables */
static MSE_DEVICE_ATTR_RO(device, info);
static MSE_DEVICE_ATTR_RO(type, info);

static struct attribute *mse_attr_info[] = {
	&mse_dev_attr_info_device.attr,
	&mse_dev_attr_info_type.attr,
	NULL,
};

static struct attribute_group mse_attr_group_info = {
	.name = "info",
	.attrs = mse_attr_info,
};

static MSE_DEVICE_ATTR(module_name, network_device, 0644,
		       mse_network_device_str_show,
		       mse_network_device_str_store);
static MSE_DEVICE_ATTR(device_name_tx, network_device, 0644,
		       mse_network_device_str_show,
		       mse_network_device_str_store);
static MSE_DEVICE_ATTR(device_name_rx, network_device, 0644,
		       mse_network_device_str_show,
		       mse_network_device_str_store);
static MSE_DEVICE_ATTR(device_name_tx_crf, network_device, 0644,
		       mse_network_device_str_show,
		       mse_network_device_str_store);
static MSE_DEVICE_ATTR(device_name_rx_crf, network_device, 0644,
		       mse_network_device_str_show,
		       mse_network_device_str_store);

static struct attribute *mse_attr_network_device_audio[] = {
	&mse_dev_attr_network_device_module_name.attr,
	&mse_dev_attr_network_device_device_name_tx.attr,
	&mse_dev_attr_network_device_device_name_rx.attr,
	&mse_dev_attr_network_device_device_name_tx_crf.attr,
	&mse_dev_attr_network_device_device_name_rx_crf.attr,
	NULL,
};

static struct attribute *mse_attr_network_device_other[] = {
	&mse_dev_attr_network_device_module_name.attr,
	&mse_dev_attr_network_device_device_name_tx.attr,
	&mse_dev_attr_network_device_device_name_rx.attr,
	NULL,
};

static struct attribute_group mse_attr_group_network_device_audio = {
	.name = "network_device",
	.attrs = mse_attr_network_device_audio,
};

static struct attribute_group mse_attr_group_network_device_other = {
	.name = "network_device",
	.attrs = mse_attr_network_device_other,
};

static MSE_DEVICE_ATTR_RW(name, packetizer);

static struct attribute *mse_attr_packetizer[] = {
	&mse_dev_attr_packetizer_name.attr,
	NULL,
};

static struct attribute_group mse_attr_group_packetizer = {
	.name = "packetizer",
	.attrs = mse_attr_packetizer,
};

static MSE_DEVICE_ATTR_RW(dst_mac, avtp_tx);
static MSE_DEVICE_ATTR_RW(src_mac, avtp_tx);
static MSE_DEVICE_ATTR(vlan, avtp_tx, 0644,
		       mse_avtp_tx_int_show, mse_avtp_tx_int_store);
static MSE_DEVICE_ATTR(priority, avtp_tx, 0644,
		       mse_avtp_tx_int_show, mse_avtp_tx_int_store);
static MSE_DEVICE_ATTR(uniqueid, avtp_tx, 0644,
		       mse_avtp_tx_int_show, mse_avtp_tx_int_store);

static struct attribute *mse_attr_avtp_tx[] = {
	&mse_dev_attr_avtp_tx_dst_mac.attr,
	&mse_dev_attr_avtp_tx_src_mac.attr,
	&mse_dev_attr_avtp_tx_vlan.attr,
	&mse_dev_attr_avtp_tx_priority.attr,
	&mse_dev_attr_avtp_tx_uniqueid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_avtp_tx = {
	.name = "avtp_tx_param",
	.attrs = mse_attr_avtp_tx,
};

static MSE_DEVICE_ATTR_RW(streamid, avtp_rx);

static struct attribute *mse_attr_avtp_rx[] = {
	&mse_dev_attr_avtp_rx_streamid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_avtp_rx = {
	.name = "avtp_rx_param",
	.attrs = mse_attr_avtp_rx,
};

static MSE_DEVICE_ATTR(samples_per_frame, audio_config, 0644,
		       mse_audio_config_int_show, mse_audio_config_int_store);
static MSE_DEVICE_ATTR_RW(crf_type, audio_config);

static struct attribute *mse_attr_audio_config[] = {
	&mse_dev_attr_audio_config_samples_per_frame.attr,
	&mse_dev_attr_audio_config_crf_type.attr,
	NULL,
};

static struct attribute_group mse_attr_group_audio_config = {
	.name = "media_audio_config",
	.attrs = mse_attr_audio_config,
};

static MSE_DEVICE_ATTR(bytes_per_frame, video_config, 0644,
		       mse_video_config_int_show, mse_video_config_int_store);
static MSE_DEVICE_ATTR(fps_denominator, video_config, 0644,
		       mse_video_config_int_show, mse_video_config_int_store);
static MSE_DEVICE_ATTR(fps_numerator, video_config, 0644,
		       mse_video_config_int_show, mse_video_config_int_store);
static MSE_DEVICE_ATTR(bitrate, video_config, 0644,
		       mse_video_config_int_show, mse_video_config_int_store);

static struct attribute *mse_attr_video_config[] = {
	&mse_dev_attr_video_config_bytes_per_frame.attr,
	&mse_dev_attr_video_config_fps_denominator.attr,
	&mse_dev_attr_video_config_fps_numerator.attr,
	&mse_dev_attr_video_config_bitrate.attr,
	NULL,
};

static struct attribute_group mse_attr_group_video_config = {
	.name = "media_video_config",
	.attrs = mse_attr_video_config,
};

static MSE_DEVICE_ATTR(tspackets_per_frame, mpeg2ts_config, 0644,
		       mse_mpeg2ts_config_int_show,
		       mse_mpeg2ts_config_int_store);
static MSE_DEVICE_ATTR(bitrate, mpeg2ts_config, 0644,
		       mse_mpeg2ts_config_int_show,
		       mse_mpeg2ts_config_int_store);
static MSE_DEVICE_ATTR(pcr_pid, mpeg2ts_config, 0644,
		       mse_mpeg2ts_config_int_show,
		       mse_mpeg2ts_config_int_store);

static struct attribute *mse_attr_mpeg2ts_config[] = {
	&mse_dev_attr_mpeg2ts_config_tspackets_per_frame.attr,
	&mse_dev_attr_mpeg2ts_config_bitrate.attr,
	&mse_dev_attr_mpeg2ts_config_pcr_pid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_mpeg2ts_config = {
	.name = "media_mpeg2ts_config",
	.attrs = mse_attr_mpeg2ts_config,
};

static MSE_DEVICE_ATTR_RW(type, ptp_config);
static MSE_DEVICE_ATTR(deviceid, ptp_config, 0644,
		       mse_ptp_config_int_show, mse_ptp_config_int_store);
static MSE_DEVICE_ATTR(capture_ch, ptp_config, 0644,
		       mse_ptp_config_int_show, mse_ptp_config_int_store);
static MSE_DEVICE_ATTR(capture_freq, ptp_config, 0644,
		       mse_ptp_config_int_show, mse_ptp_config_int_store);
static MSE_DEVICE_ATTR(recovery_capture_freq, ptp_config, 0644,
		       mse_ptp_config_int_show, mse_ptp_config_int_store);

static struct attribute *mse_attr_ptp_config_audio[] = {
	&mse_dev_attr_ptp_config_type.attr,
	&mse_dev_attr_ptp_config_deviceid.attr,
	&mse_dev_attr_ptp_config_capture_ch.attr,
	&mse_dev_attr_ptp_config_capture_freq.attr,
	&mse_dev_attr_ptp_config_recovery_capture_freq.attr,
	NULL,
};

static struct attribute_group mse_attr_group_ptp_config_audio = {
	.name = "ptp_config",
	.attrs = mse_attr_ptp_config_audio,
};

static struct attribute *mse_attr_ptp_config_other[] = {
	&mse_dev_attr_ptp_config_type.attr,
	&mse_dev_attr_ptp_config_deviceid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_ptp_config_other = {
	.name = "ptp_config",
	.attrs = mse_attr_ptp_config_other,
};

static MSE_DEVICE_ATTR(enable, mch_config, 0644,
		       mse_mch_config_int_show, mse_mch_config_int_store);

static struct attribute *mse_attr_mch_config[] = {
	&mse_dev_attr_mch_config_enable.attr,
	NULL,
};

static struct attribute_group mse_attr_group_mch_config = {
	.name = "mch_config",
	.attrs = mse_attr_mch_config,
};

static MSE_DEVICE_ATTR_RW(dst_mac, avtp_tx_crf);
static MSE_DEVICE_ATTR_RW(src_mac, avtp_tx_crf);
static MSE_DEVICE_ATTR(vlan, avtp_tx_crf, 0644,
		       mse_avtp_tx_crf_int_show, mse_avtp_tx_crf_int_store);
static MSE_DEVICE_ATTR(priority, avtp_tx_crf, 0644,
		       mse_avtp_tx_crf_int_show, mse_avtp_tx_crf_int_store);
static MSE_DEVICE_ATTR(uniqueid, avtp_tx_crf, 0644,
		       mse_avtp_tx_crf_int_show, mse_avtp_tx_crf_int_store);

static struct attribute *mse_attr_avtp_tx_crf[] = {
	&mse_dev_attr_avtp_tx_crf_dst_mac.attr,
	&mse_dev_attr_avtp_tx_crf_src_mac.attr,
	&mse_dev_attr_avtp_tx_crf_vlan.attr,
	&mse_dev_attr_avtp_tx_crf_priority.attr,
	&mse_dev_attr_avtp_tx_crf_uniqueid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_avtp_tx_crf = {
	.name = "avtp_tx_param_crf",
	.attrs = mse_attr_avtp_tx_crf,
};

static MSE_DEVICE_ATTR_RW(streamid, avtp_rx_crf);

static struct attribute *mse_attr_avtp_rx_crf[] = {
	&mse_dev_attr_avtp_rx_crf_streamid.attr,
	NULL,
};

static struct attribute_group mse_attr_group_avtp_rx_crf = {
	.name = "avtp_rx_param_crf",
	.attrs = mse_attr_avtp_rx_crf,
};

static MSE_DEVICE_ATTR(max_transit_time_ns, delay_time, 0644,
		       mse_delay_time_int_show, mse_delay_time_int_store);
static MSE_DEVICE_ATTR(tx_delay_time_ns, delay_time, 0644,
		       mse_delay_time_int_show, mse_delay_time_int_store);
static MSE_DEVICE_ATTR(rx_delay_time_ns, delay_time, 0644,
		       mse_delay_time_int_show, mse_delay_time_int_store);

static struct attribute *mse_attr_delay_time[] = {
	&mse_dev_attr_delay_time_max_transit_time_ns.attr,
	&mse_dev_attr_delay_time_tx_delay_time_ns.attr,
	&mse_dev_attr_delay_time_rx_delay_time_ns.attr,
	NULL,
};

static struct attribute_group mse_attr_group_delay_time = {
	.name = "delay_time",
	.attrs = mse_attr_delay_time,
};

/* external variable */
const struct attribute_group *mse_attr_groups_audio[] = {
	&mse_attr_group_info,
	&mse_attr_group_network_device_audio,
	&mse_attr_group_packetizer,
	&mse_attr_group_avtp_tx,
	&mse_attr_group_avtp_rx,
	&mse_attr_group_audio_config,
	&mse_attr_group_ptp_config_audio,
	&mse_attr_group_mch_config,
	&mse_attr_group_avtp_tx_crf,
	&mse_attr_group_avtp_rx_crf,
	&mse_attr_group_delay_time,
	NULL,
};

const struct attribute_group *mse_attr_groups_video[] = {
	&mse_attr_group_info,
	&mse_attr_group_network_device_other,
	&mse_attr_group_packetizer,
	&mse_attr_group_avtp_tx,
	&mse_attr_group_avtp_rx,
	&mse_attr_group_video_config,
	&mse_attr_group_ptp_config_other,
	&mse_attr_group_delay_time,
	NULL,
};

const struct attribute_group *mse_attr_groups_mpeg2ts[] = {
	&mse_attr_group_info,
	&mse_attr_group_network_device_other,
	&mse_attr_group_packetizer,
	&mse_attr_group_avtp_tx,
	&mse_attr_group_avtp_rx,
	&mse_attr_group_mpeg2ts_config,
	&mse_attr_group_ptp_config_other,
	&mse_attr_group_delay_time,
	NULL,
};

void mse_sysfs_set_device_groups(struct device *dev, enum MSE_STREAM_TYPE type)
{
	if (type == MSE_STREAM_TYPE_AUDIO)
		dev->groups = mse_attr_groups_audio;
	else if (type == MSE_STREAM_TYPE_VIDEO)
		dev->groups = mse_attr_groups_video;
	else if (type == MSE_STREAM_TYPE_MPEG2TS)
		dev->groups = mse_attr_groups_mpeg2ts;
}
