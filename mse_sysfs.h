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

#ifndef __MSE_SYSFS_H__
#define __MSE_SYSFS_H__

#define MSE_SYSFS_NAME_STR_TYPE                      "type"
#define MSE_SYSFS_NAME_STR_KIND                      "kind"
#define MSE_SYSFS_NAME_STR_DST_MAC                   "dst_mac"
#define MSE_SYSFS_NAME_STR_SRC_MAC                   "src_mac"
#define MSE_SYSFS_NAME_STR_VLAN                      "vlan"
#define MSE_SYSFS_NAME_STR_PRIORITY                  "priority"
#define MSE_SYSFS_NAME_STR_UNIQUE_ID                 "unique_id"
#define MSE_SYSFS_NAME_STR_NETWORK_ADAPTER_NAME      "network_adapter_name"
#define MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_TX    "network_device_name_tx"
#define MSE_SYSFS_NAME_STR_NETWORK_DEVICE_NAME_RX    "network_device_name_rx"
#define MSE_SYSFS_NAME_STR_PACKETIZER_NAME           "packetizer_name"
#define MSE_SYSFS_NAME_STR_FPS_SECONDS               "fps_seconds"
#define MSE_SYSFS_NAME_STR_FPS_FRAMES                "fps_frames"
#define MSE_SYSFS_NAME_STR_BITRATE                   "bitrate"
#define MSE_SYSFS_NAME_STR_BYTES_PER_FRAME           "bytes_per_frame"
#define MSE_SYSFS_NAME_STR_DEVICE                    "device"
#define MSE_SYSFS_NAME_STR_PTP_CLOCK                 "ptp_clock"
#define MSE_SYSFS_NAME_STR_PTP_CLOCK_DEVICE          "ptp_clock_device"
#define MSE_SYSFS_NAME_STR_PTP_CAPTURE_CH            "ptp_capture_ch"
#define MSE_SYSFS_NAME_STR_PTP_CAPTURE_FREQ          "ptp_capture_freq"
#define MSE_SYSFS_NAME_STR_MEDIA_CLOCK_RECOVERY      "media_clock_recovery"
#define MSE_SYSFS_NAME_STR_MEDIA_CLOCK_TYPE          "media_clock_type"
#define MSE_SYSFS_NAME_STR_RECOVERY_CAPTURE_FREQ     "recovery_capture_freq"
#define MSE_SYSFS_NAME_STR_SEND_CLOCK                "send_clock"
#define MSE_SYSFS_NAME_STR_SEND_CLOCK_DST_MAC        "send_clock_dst_mac"
#define MSE_SYSFS_NAME_STR_SEND_CLOCK_SRC_MAC        "send_clock_src_mac"
#define MSE_SYSFS_NAME_STR_SEND_CLOCK_UNIQUE_ID      "send_clock_unique_id"
#define MSE_SYSFS_NAME_STR_MAX_TRANSIT_TIME          "max_transit_time"
#define MSE_SYSFS_NAME_STR_TALKER_DELAY_TIME         "talker_delay_time"
#define MSE_SYSFS_NAME_STR_LISTENER_DELAY_TIME       "listener_delay_time"

enum MSE_SYSFS_CONFIG_TYPE {
	MSE_TYPE_INT,
	MSE_TYPE_STR,
	MSE_TYPE_ENUM,
	MSE_TYPE_MEM,
};

struct mse_sysfs_config {
	char *name;
	enum MSE_SYSFS_CONFIG_TYPE type;
	int int_value;
	union {
		char str_value[255];
		char *enum_list[10];
	};
};

extern int mse_sysfs_init(struct mse_sysfs_config *config);
extern void mse_sysfs_exit(int index);
extern int mse_sysfs_get_config_int(int index, char *name, int *value);
extern int mse_sysfs_get_config_str(int index,
				    char *name,
				    char *buf,
				    int buf_size);
extern int mse_sysfs_set_config_int(int index, char *name, int value);
extern int mse_sysfs_set_config_str(int index,
				    char *name,
				    char *buf,
				    int buf_size);

#endif /* __MSE_SYSFS_H__ */
