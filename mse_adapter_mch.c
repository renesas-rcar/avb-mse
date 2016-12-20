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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include "ravb_mse_kernel.h"

/* TODO: move to header */
extern int mch_open(int *dev_id);
extern int mch_close(int dev_id);
extern int mch_send_timestamps(int dev_id,
			       int time_rate_ns,
			       int master_count,
			       unsigned int master_timestamps[],
			       int device_count,
			       unsigned int device_timestamps[]);
extern int mch_get_recovery_value(int dev_id, int *value);

extern int mch_ptp_open(int *dev_id);
extern int mch_ptp_close(int dev_id);
extern int mch_ptp_get_time(int dev_id,
			    struct ptp_clock_time *clock_time);
extern int mch_ptp_get_timestamps(int dev_id,
				  int ch,
				  int *count,
				  struct ptp_clock_time timestamps[]);

static struct mch_ops mch_mse_ops = {
	.open = mch_open,
	.close = mch_close,
	.send_timestamps = mch_send_timestamps,
	.get_recovery_value = mch_get_recovery_value,
};

static struct mse_ptp_ops ptp_mse_ops = {
	.open = mch_ptp_open,
	.close = mch_ptp_close,
	.get_time = mch_ptp_get_time,
	.get_timestamps = mch_ptp_get_timestamps,
};

static int mch_mse_if_instance_id;
static int ptp_mse_if_instance_id;

static int __init mse_adapter_mch_init(void)
{
	int inst_id;

	pr_debug("[%s]\n", __func__);

	inst_id = mse_register_mch(&mch_mse_ops);
	if (inst_id < 0)
		return inst_id;

	mch_mse_if_instance_id = inst_id;

	inst_id = mse_register_ptp(&ptp_mse_ops);
	if (inst_id < 0)
		return inst_id;

	ptp_mse_if_instance_id = inst_id;

	return 0;
}

static void __exit mse_adapter_mch_exit(void)
{
	pr_debug("[%s]\n", __func__);
	mse_unregister_mch(mch_mse_if_instance_id);
	mse_unregister_ptp(ptp_mse_if_instance_id);
}

module_init(mse_adapter_mch_init);
module_exit(mse_adapter_mch_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
