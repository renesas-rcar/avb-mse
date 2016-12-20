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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include "ravb_mse_kernel.h"
#include "ravb_eavb.h"

#define MSE_EAVB_ADAPTER_MAX (10)

#define MSE_EAVB_ADAPTER_PACKET_MAX (1024)
#define MSE_EAVB_ADAPTER_ENTRY_MAX (128)

#define MSE_EAVB_PACKET_LENGTH (1526)

#define avb_compare_param_key(val, key) \
	strncmp(val, key, strlen(key))

struct mse_adapter_eavb {
	bool used_f;
	struct eavb_entry *entry;
	int num_entry;
	int entried, unentry;
	int num_send;
	struct ravb_streaming_kernel_if ravb;
	struct eavb_entry read_entry[MSE_EAVB_ADAPTER_ENTRY_MAX];
};

static struct mse_adapter_eavb eavb_table[MSE_EAVB_ADAPTER_MAX];

static struct {
	const char *key;
	enum AVB_DEVNAME value;
} avb_devname_table[] = {
	{ "avb_rx15", AVB_DEVNAME_RX15 },
	{ "avb_rx14", AVB_DEVNAME_RX14 },
	{ "avb_rx13", AVB_DEVNAME_RX13 },
	{ "avb_rx12", AVB_DEVNAME_RX12 },
	{ "avb_rx11", AVB_DEVNAME_RX11 },
	{ "avb_rx10", AVB_DEVNAME_RX10 },
	{ "avb_rx9", AVB_DEVNAME_RX9 },
	{ "avb_rx8", AVB_DEVNAME_RX8 },
	{ "avb_rx7", AVB_DEVNAME_RX7 },
	{ "avb_rx6", AVB_DEVNAME_RX6 },
	{ "avb_rx5", AVB_DEVNAME_RX5 },
	{ "avb_rx4", AVB_DEVNAME_RX4 },
	{ "avb_rx3", AVB_DEVNAME_RX3 },
	{ "avb_rx2", AVB_DEVNAME_RX2 },
	{ "avb_rx1", AVB_DEVNAME_RX1 },
	{ "avb_rx0", AVB_DEVNAME_RX0 },
	{ "avb_tx1", AVB_DEVNAME_TX1 },
	{ "avb_tx0", AVB_DEVNAME_TX0 },
};

static enum AVB_DEVNAME mse_adapter_eavb_set_devname(const char *val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(avb_devname_table); i++)
		if (!avb_compare_param_key(val, avb_devname_table[i].key))
			return avb_devname_table[i].value;

	return -EINVAL;
}

static int mse_adapter_eavb_open(char *name)
{
	int err, index;
	enum AVB_DEVNAME id;
	struct mse_adapter_eavb *eavb;
	static struct eavb_option option = {
		.id = EAVB_OPTIONID_BLOCKMODE,
		.param = EAVB_BLOCK_WAITALL,
	};

	/* convert table string->id */
	id = mse_adapter_eavb_set_devname(name);
	if (id < 0) {
		pr_err("[%s] error unknown dev=%s\n", __func__, name);
		id = AVB_DEVNAME_TX0;
	}

	pr_debug("[%s] dev=%s(%d)\n", __func__, name, id);

	for (index = 0; index < ARRAY_SIZE(eavb_table) &&
	     eavb_table[index].used_f; index++)
		;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];
	eavb->used_f = true;

	err = ravb_streaming_open_stq_kernel(id, &eavb->ravb, 0);
	if (err) {
		pr_err("[%s] error open dev=%s code=%d\n",
		       __func__, name, err);
		return err;
	}

	err = eavb->ravb.set_option(eavb->ravb.handle, &option);
	if (err)
		pr_err("[%s] error option_kernel code=%d\n", __func__, err);

	eavb->entry = kcalloc(MSE_EAVB_ADAPTER_ENTRY_MAX * 2,
			      sizeof(struct eavb_entry),
			      GFP_KERNEL);
	if (!eavb->entry)
		return -ENOMEM;

	eavb->num_entry = 0;

	return index;
}

static int mse_adapter_eavb_release(int index)
{
	int err;
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	err = ravb_streaming_release_stq_kernel(eavb->ravb.handle);
	if (err)
		pr_err("[%s] error release code=%d\n", __func__, err);

	kfree(eavb->entry);
	eavb->used_f = false;

	return err;
}

static int mse_adapter_eavb_set_cbs_param(int index, struct eavb_cbsparam *cbs)
{
	long err;
	struct eavb_txparam txparam;
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	txparam.cbs = *cbs;
	pr_debug(" bandwidthFraction = %08x\n", txparam.cbs.bandwidthFraction);
	pr_debug(" idleSlope         = %08x\n", txparam.cbs.idleSlope);
	pr_debug(" sendSlope         = %08x\n", txparam.cbs.sendSlope);
	pr_debug(" hiCredit          = %08x\n", txparam.cbs.hiCredit);
	pr_debug(" loCredit          = %08x\n", txparam.cbs.loCredit);

	err = eavb->ravb.set_txparam(eavb->ravb.handle, &txparam);
	if (err)
		pr_err("[%s] error %ld\n", __func__, err);

	return 0;
}

static int mse_adapter_eavb_set_streamid(int index, u8 streamid[8])
{
	long err;
	struct eavb_rxparam rxparam;
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	memcpy(rxparam.streamid, streamid, sizeof(rxparam.streamid));
	pr_debug(" streamid=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
		 rxparam.streamid[0], rxparam.streamid[1],
		 rxparam.streamid[2], rxparam.streamid[3],
		 rxparam.streamid[4], rxparam.streamid[5],
		 rxparam.streamid[6], rxparam.streamid[7]);

	err = eavb->ravb.set_rxparam(eavb->ravb.handle, &rxparam);
	if (err)
		pr_err("[%s] error %ld\n", __func__, err);

	return 0;
}

static int mse_adapter_eavb_check_receive(int index)
{
	long err;
	struct eavb_entrynum en = {0, 0, 0};
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	err = eavb->ravb.get_entrynum(eavb->ravb.handle, &en);
	pr_debug("[%s] index=%d ret=%ld entry=%d wait=%d log=%d\n",
		 __func__, index, err,
		 en.accepted, en.processed, en.completed);

	return en.completed;
}

static int mse_adapter_eavb_send_prepare(int index,
					 struct mse_packet *packets,
					 int num_packets)
{
	int i;
	struct mse_adapter_eavb *eavb;

	pr_debug("[%s] index=%d addr=%p num=%d\n",
		 __func__, index, packets, num_packets);

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	if (num_packets <= 0)
		return -EPERM;

	if (num_packets > MSE_EAVB_ADAPTER_PACKET_MAX) {
		pr_err("[%s] too much packets %d\n", __func__, num_packets);
		return -EPERM;
	}

	for (i = 0; i < num_packets; i++) {
		(eavb->entry + i)->seq_no = i;
		(eavb->entry + i)->vec[0].base = packets[i].paddr;
		(eavb->entry + i)->vec[0].len = packets[i].len;
	}

	eavb->entried = 0;
	eavb->unentry = 0;
	eavb->num_send = 0;
	eavb->num_entry = num_packets;

	return 0;
}

static int mse_adapter_eavb_send(int index,
				 struct mse_packet *packets,
				 int num_packets)
{
	int num_dequeue, i, ofs;
	ssize_t wret, rret = 0, wret2;
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	if (num_packets <= 0)
		return -EPERM;

	pr_debug("[%s] index=%d num=%d\n",
		 __func__, index, num_packets);

	if (num_packets > MSE_EAVB_ADAPTER_ENTRY_MAX) {
		pr_err("[%s] too much packets\n", __func__);
		return -EPERM;
	}

	num_dequeue = eavb->num_send + num_packets -
		MSE_EAVB_ADAPTER_ENTRY_MAX;
	if (num_dequeue > 0) {
		/* dequeue before send */
		rret = eavb->ravb.read(eavb->ravb.handle, eavb->read_entry,
				       num_dequeue);
		if (rret != num_dequeue) {
			/* TODO: Error recover */
			mse_adapter_eavb_check_receive(index);
			if (rret < 0) {
				pr_err("[%s] read error %d\n",
				       __func__, (int)rret);
				return rret;
			}
			pr_err("[%s] read is short %d/%d\n",
			       __func__, (int)rret, num_dequeue);
		}
		eavb->entried = (eavb->entried + rret) % eavb->num_entry;
		eavb->num_send -= rret;
	}

	/* update packet size */
	for (i = 0; i < num_packets; i++) {
		ofs = (eavb->unentry + i) % eavb->num_entry;
		(eavb->entry + ofs)->vec[0].len = packets[ofs].len;
	}
	/* enqueue */
	if (eavb->unentry + num_packets <= eavb->num_entry) {
		wret = eavb->ravb.write(eavb->ravb.handle,
					eavb->entry + eavb->unentry,
					num_packets);
		if (wret < 0) {
			pr_err("[%s] write error %d\n", __func__, (int)wret);
			return wret;
		}
	} else {
		wret = eavb->ravb.write(eavb->ravb.handle,
					eavb->entry + eavb->unentry,
					eavb->num_entry - eavb->unentry);
		if (wret < 0) {
			pr_err("[%s] write error %d\n", __func__, (int)wret);
			return wret;
		}
		wret2 = eavb->ravb.write(
				eavb->ravb.handle,
				eavb->entry,
				num_packets - eavb->num_entry + eavb->unentry);
		if (wret2 < 0) {
			pr_err("[%s] write error %d\n", __func__, (int)wret);
			return wret2;
		}
		wret += wret2;
	}

	if (wret != num_packets) {
		/* TODO: Error recover */
		mse_adapter_eavb_check_receive(index);
		pr_err("[%s] write is short %d/%d\n", __func__,
		       (int)wret, num_packets);
	}
	eavb->unentry = (eavb->unentry + wret) % eavb->num_entry;
	eavb->num_send += wret;
	pr_debug("[%s] read %zd write %zd\n", __func__, rret, wret);

	return (int)wret;
}

static int mse_adapter_eavb_receive_prepare(int index,
					    struct mse_packet *packets,
					    int num_packets)
{
	struct mse_adapter_eavb *eavb;
	int i;
	ssize_t ret;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	if (num_packets <= 0)
		return -EPERM;

	pr_debug("[%s] index=%d addr=%p num=%d\n",
		 __func__, index, packets, num_packets);

	if (num_packets > MSE_EAVB_ADAPTER_PACKET_MAX) {
		pr_err("[%s] too much packets\n", __func__);
		return -EPERM;
	}

	for (i = 0; i < num_packets; i++) {
		(eavb->entry + i)->seq_no = i;
		(eavb->entry + i)->vec[0].base = packets[i].paddr;
		(eavb->entry + i)->vec[0].len = packets[i].len;
	}

	/* entry queue */
	eavb->entried = 0;
	ret = eavb->ravb.write(eavb->ravb.handle,
			       eavb->entry,
			       MSE_EAVB_ADAPTER_ENTRY_MAX);
	if (ret != MSE_EAVB_ADAPTER_ENTRY_MAX) {
		pr_err("[%s] cannot entry packet %zu/%d\n",
		       __func__, ret, MSE_EAVB_ADAPTER_ENTRY_MAX);

		mse_adapter_eavb_check_receive(index);
		if (ret < 0)
			pr_err("[%s] write error %d\n", __func__, (int)ret);
		else
			pr_err("[%s] write is short %d/%d\n", __func__,
			       (int)ret, MSE_EAVB_ADAPTER_ENTRY_MAX);
		return -EPERM;
	}
	eavb->unentry = MSE_EAVB_ADAPTER_ENTRY_MAX;
	eavb->num_entry = num_packets;

	return 0;
}

static int mse_adapter_eavb_receive(int index, int num_packets)
{
	int receive, i, ofs;
	ssize_t ret;
	struct mse_adapter_eavb *eavb;
	int read_packets;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	if (num_packets <= 0)
		return -EPERM;

	pr_debug("[%s] index=%d num=%d\n", __func__, index, num_packets);
	if (num_packets > MSE_EAVB_ADAPTER_ENTRY_MAX) {
		pr_err("[%s] too much packets\n", __func__);
		return -EPERM;
	}

	read_packets = mse_adapter_eavb_check_receive(index);
	if (read_packets == 0)
		read_packets = 1;
	if (read_packets > num_packets)
		read_packets = num_packets;

	/* dequeue */
	ret = eavb->ravb.read(eavb->ravb.handle,
			      eavb->read_entry,
			      read_packets);
	if (ret == -EINTR || ret == -EAGAIN) {
		pr_info("[%s] receive error %zd\n", __func__, ret);
		return ret;
	} else if (ret < 0) {
		pr_err("[%s] receive error %zd\n", __func__, ret);
		return ret;
	}
	if (ret == 0) {
		pr_debug("[%s] receive  %zd packet\n", __func__, ret);
		return 0;
	}
	receive = ret;
	eavb->entried = (eavb->entried + receive) % eavb->num_entry;

	/* update packet size */
	for (i = 0; i < receive; i++) {
		ofs = (eavb->unentry + i) % eavb->num_entry;
		(eavb->entry + ofs)->vec[0].len = MSE_EAVB_PACKET_LENGTH;
	}
	/* enqueue */
	if (eavb->unentry + receive <= eavb->num_entry) {
		ret = eavb->ravb.write(eavb->ravb.handle,
				       eavb->entry + eavb->unentry,
				       receive);
	} else {
		ret = eavb->ravb.write(eavb->ravb.handle,
				       eavb->entry + eavb->unentry,
				       eavb->num_entry - eavb->unentry);
		ret = eavb->ravb.write(
				eavb->ravb.handle,
				eavb->entry,
				receive - eavb->num_entry + eavb->unentry);
	}

	eavb->unentry = (eavb->unentry + receive) % eavb->num_entry;

	return receive;
}

static int mse_adapter_eavb_set_option(int index)
{
	static struct eavb_option option = {
		.id = EAVB_OPTIONID_BLOCKMODE,
		.param = EAVB_BLOCK_WAITALL,
	};
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	return eavb->ravb.set_option(eavb->ravb.handle, &option);
}

static int mse_adapter_eavb_cancel(int index)
{
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	return eavb->ravb.blocking_cancel(eavb->ravb.handle);
}

static int mse_adapter_eavb_get_link_speed(int index)
{
	long link_speed;
	struct mse_adapter_eavb *eavb;

	if (index >= ARRAY_SIZE(eavb_table))
		return -EPERM;

	eavb = &eavb_table[index];

	if (!eavb->used_f)
		return -EPERM;

	pr_debug("[%s] index=%d\n", __func__, index);

	link_speed = eavb->ravb.get_linkspeed(eavb->ravb.handle);
	if (link_speed < 0)
		pr_err("[%s] error get link speed code=%ld\n",
		       __func__,
		       link_speed);

	/* return speed as Mbps */
	return link_speed;
}

static struct mse_adapter_network_ops mse_adapter_eavb_ops = {
	.name = "ravb",
	.type = MSE_TYPE_ADAPTER_NETWORK,
	.open = mse_adapter_eavb_open,
	.release = mse_adapter_eavb_release,
	.set_option = mse_adapter_eavb_set_option,
	.set_cbs_param = mse_adapter_eavb_set_cbs_param,
	.set_streamid = mse_adapter_eavb_set_streamid,
	.send_prepare = mse_adapter_eavb_send_prepare,
	.send = mse_adapter_eavb_send,
	.receive_prepare = mse_adapter_eavb_receive_prepare,
	.receive = mse_adapter_eavb_receive,
	.check_receive = mse_adapter_eavb_check_receive,
	.cancel = mse_adapter_eavb_cancel,
	.get_link_speed = mse_adapter_eavb_get_link_speed,
};

static int adapter_index;

static int __init mse_adapter_eavb_init(void)
{
	pr_debug("[%s]\n", __func__);

	adapter_index = mse_register_adapter_network(&mse_adapter_eavb_ops);
	if (adapter_index < 0) {
		pr_err("[%s] cannot register\n", __func__);
		return -EPERM;
	}

	return 0;
}

static void __exit mse_adapter_eavb_exit(void)
{
	pr_debug("[%s]\n", __func__);
	mse_unregister_adapter_network(adapter_index);
}

module_init(mse_adapter_eavb_init);
module_exit(mse_adapter_eavb_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
