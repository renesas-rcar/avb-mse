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
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/ptp_clock.h>
#include "ravb_mse_kernel.h"
#include "mse_packet_ctrl.h"
#include "mse_packetizer.h"

#define MSE_DMA_BUF_RECEIVE_SIZE 10
#define MSE_DMA_BUF_SEND_SIZE 10

#define MSE_PACKET_SIZE_MIN 64

inline int mse_packet_ctrl_check_packet_remain(struct mse_packet_ctrl *dma)
{
	return dma->write_p >= dma->read_p ? dma->write_p - dma->read_p
				: dma->size + dma->write_p - dma->read_p;
}

struct mse_packet_ctrl *mse_packet_ctrl_alloc(struct device *dev,
					      int max_packet,
					      int max_packet_size)
{
	struct mse_packet_ctrl *dma;
	dma_addr_t paddr, pitch;
	int i;

	pr_debug("[%s] packets=%d size=%d",
		 __func__, max_packet, max_packet_size);

	dma = kmalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return NULL;

	dma->dma_vaddr = dma_alloc_coherent(dev,
					    max_packet_size * max_packet,
					    &dma->dma_handle,
					    GFP_KERNEL);
	if (!dma->dma_vaddr) {
		pr_err("[%s] cannot dma_alloc_coherent!\n", __func__);
		kfree(dma);
		return NULL;
	}

	dma->dev = dev;
	dma->size = max_packet;
	dma->write_p = 0;
	dma->read_p = 0;
	dma->max_packet_size = max_packet_size;
	dma->packet_table = kmalloc((sizeof(struct mse_packet) * dma->size),
				    GFP_KERNEL);

	paddr = dma->dma_handle;
	for (i = 0; i < dma->size; i++) {
		pitch = dma->max_packet_size * i;
		dma->packet_table[i].len = dma->max_packet_size;
		dma->packet_table[i].paddr = paddr + pitch;
		dma->packet_table[i].vaddr = dma->dma_vaddr + pitch;
	}

	return dma;
}

void mse_packet_ctrl_free(struct mse_packet_ctrl *dma)
{
	if (!dma)
		return;

	pr_debug("[%s]\n", __func__);

	kfree(dma->packet_table);
	dma_free_coherent(dma->dev,
			  dma->size * dma->max_packet_size,
			  dma->dma_vaddr,
			  dma->dma_handle);
	kfree(dma);
}

#if 0
static int calculate_timestamp(unsigned int *timestamp,
			       int ptp_clock,
			       int tstamp_size,
			       unsigned int *tstamp)
{
	/* TODO: calculate! */
	switch (ptp_clock) {
	case 0:
		/* ptp */
		*timestamp = tstamp[tstamp_size - 1] +
			    ((tstamp[tstamp_size - 1] +
			    tstamp[tstamp_size - 2]) / 2);
		break;
	case 1:
		/* timestamp */
		*timestamp = tstamp[tstamp_size - 1] +
			    ((tstamp[tstamp_size - 1] +
			    tstamp[tstamp_size - 2]) / 2);
		break;
	default:
		*timestamp = tstamp[tstamp_size - 1];
		break;
	}

	return 0;
}
#endif

int mse_packet_ctrl_make_packet(int index,
				void *data,
				size_t size,
				int ptp_clock,
				int tstamp_size,
				unsigned int *tstamp,
				struct mse_packet_ctrl *dma,
				struct mse_packetizer_ops *ops)
{
	int ret = 0;
	size_t processed = 0;
	size_t packet_size = 0;
	int new_write_p;
	int pcount = 0;
	unsigned int timestamp, t;
	static int packet_max;
	static int pno;

	if (!ops) {
		pr_err("[%s] no packetizer\n", __func__);
		return -EINVAL;
	}
	t  = 0;

	while (!ret) {
		new_write_p = (dma->write_p + 1) % dma->size;
		if (new_write_p == dma->read_p) {
			pr_err("make overrun r=%d w=%d nw=%d p=%zu/%zu\n",
			       dma->read_p, dma->write_p, new_write_p,
			       processed, size);
			return processed;
		}

		memset(dma->packet_table[dma->write_p].vaddr, 0,
		       dma->packet_table[dma->write_p].len);
		timestamp = tstamp[t++];

		ret = ops->packetize(index,
				     dma->packet_table[dma->write_p].vaddr,
				     &packet_size,
				     data,
				     size,
				     &processed,
				     &timestamp);

		if (ret >= 0) {
			pcount++;
			if (packet_size >= 64) {
				dma->packet_table[dma->write_p].len =
					packet_size;
			} else {
				dma->packet_table[dma->write_p].len = 64;
			}
			dma->write_p = new_write_p;
		} else {
			break;
		}
	}

	pr_debug("[%s] packet, %d, %d, %d, %zu\n",
		 __func__, pno++, pcount, packet_max, processed);

	pr_debug("[%s] dma buffer ok wp=%d, rp=%d, processed=%zu/%zu\n",
		 __func__, dma->write_p, dma->read_p, processed, size);

	return processed;
}

int mse_packet_ctrl_make_packet_crf(int index,
				    struct ptp_clock_time *timestamps,
				    int count,
				    struct mse_packet_ctrl *dma)
{
	int ret = 0;
	size_t packet_size = 0;
	int new_write_p;

	new_write_p = (dma->write_p + 1) % dma->size;
	if (new_write_p == dma->read_p) {
		pr_err("make overrun r=%d w=%d nw=%d\n",
		       dma->read_p, dma->write_p, new_write_p);
		return -1;
	}

	memset(dma->packet_table[dma->write_p].vaddr, 0, 64);

	/* CRF packetizer */
	mse_packetizer_crf_tstamp_audio_ops.packetize(
		index,
		dma->packet_table[dma->write_p].vaddr,
		&packet_size,
		timestamps,
		count * sizeof(struct ptp_clock_time),
		NULL,
		NULL);

	if (ret >= 0) {
		if (packet_size < MSE_PACKET_SIZE_MIN)
			packet_size = MSE_PACKET_SIZE_MIN;
		dma->packet_table[dma->write_p].len = packet_size;
		dma->write_p = new_write_p;
	}

	return 1;
}

int mse_packet_ctrl_send_packet(int index,
				struct mse_packet_ctrl *dma,
				struct mse_adapter_network_ops *ops)
{
	int new_read_p, now_write_p, ret;

	if (dma->write_p == dma->read_p) {
		pr_err("[%s] no data\n", __func__);
		return 0;
	}
	if (!ops) {
		pr_err("[%s] no network adapter\n", __func__);
		return -EINVAL;
	}

	while (dma->write_p != dma->read_p) {
		now_write_p = dma->write_p;
		if (now_write_p > dma->read_p) {
			/* no wrap */
			new_read_p = dma->read_p + MSE_DMA_BUF_SEND_SIZE;
			if (new_read_p > now_write_p)
				new_read_p = now_write_p;
		} else {
			/* wrap */
			new_read_p = dma->read_p + MSE_DMA_BUF_SEND_SIZE;
			if (new_read_p > dma->size)
				new_read_p = dma->size;
		}

		ret = ops->send(index,
				&dma->packet_table[dma->read_p],
				new_read_p - dma->read_p);
		if (ret < 0)
			return -EPERM;

		new_read_p = (dma->read_p + ret) % dma->size;

		pr_debug("[%s] %d packtets w=%d r=%d -> %d\n",
			 __func__, ret, now_write_p, dma->read_p, new_read_p);

		dma->read_p = new_read_p;
	}

	return 0;
}

int mse_packet_ctrl_receive_prepare_packet(
				int index,
				struct mse_packet_ctrl *dma,
				struct mse_adapter_network_ops *ops)
{
	return ops->receive_prepare(index,
				    dma->packet_table,
				    dma->size);
}

int mse_packet_ctrl_receive_packet(int index,
				   int max_size,
				   struct mse_packet_ctrl *dma,
				   struct mse_adapter_network_ops *ops)
{
	int new_write_p, ret;
	int size;

	/* TODO: receive insufficient size */
	if (!ops) {
		pr_err("[%s] no network adapter\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] network adapter=%s r=%d w=%d\n",
		 __func__, ops->name, dma->read_p, dma->write_p);

	while (1) {
		size = 1;
		if (dma->read_p > dma->write_p &&
		    dma->read_p < dma->write_p + size) {
			pr_info("receive overrun r=%d w=%d size=%d/%d\n",
				dma->read_p, dma->write_p, size, max_size);
			return -ENOSPC;
		}

		ret = ops->receive(index, size);
		if (ret < 0) {
			pr_err("[%s] receive error %d\n", __func__, ret);
			return -EPERM;
		}

		new_write_p = (dma->write_p + ret) % dma->size;

		pr_debug("[%s] %d packtets r=%d w=%d->%d\n", __func__,
			 ret, dma->read_p, dma->write_p, new_write_p);

		dma->write_p = new_write_p;

		/* for cancel */
		if (ret != size)
			return -EINTR;
	}

	return 0;
}


int mse_packet_ctrl_receive_packet_crf(int index,
				   int max_size,
				   struct mse_packet_ctrl *dma,
				   struct mse_adapter_network_ops *ops)
{
	int new_write_p, ret;
	int size;

	/* TODO: receive insufficient size */
	if (!ops) {
		pr_err("[%s] no network adapter\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] network adapter=%s r=%d w=%d\n",
		 __func__, ops->name, dma->read_p, dma->write_p);

	while (1) {
		size = max_size;
		if (dma->read_p > dma->write_p &&
		    dma->read_p < dma->write_p + size) {
			pr_info("receive overrun r=%d w=%d size=%d/%d\n",
				dma->read_p, dma->write_p, size, max_size);
			return -ENOSPC;
		}

		ret = ops->receive(index, size);
		if (ret < 0) {
			pr_err("[%s] receive error %d\n", __func__, ret);
			return -EPERM;
		}

		new_write_p = (dma->write_p + ret) % dma->size;

		pr_debug("[%s] %d packtets r=%d w=%d->%d\n", __func__,
			 ret, dma->read_p, dma->write_p, new_write_p);

		dma->write_p = new_write_p;

		/* for cancel */
		if (ret != size)
			return -EINTR;
		break;
	}

	return 0;
}


int mse_packet_ctrl_take_out_packet(int index,
				    void *data,
				    size_t size,
				    unsigned int *timestamps,
				    int t_size,
				    int *t_stored,
				    struct mse_packet_ctrl *dma,
				    struct mse_packetizer_ops *ops,
				    size_t *processed)
{
	int ret = 0;
	int new_read_p;
	unsigned int recv_time;
	static int pcount;

	if (!*processed)
		pcount = 0;

	if (!ops) {
		pr_err("[%s] no packetizer\n", __func__);
		return -EINVAL;
	}

	pr_debug("[%s] packetizer=%s r=%d w=%d s=%d v=%p\n",
		 __func__, ops->name, dma->read_p, dma->write_p, dma->size,
		 dma->packet_table[dma->read_p].vaddr);

	*t_stored = 0;

	while (dma->read_p != dma->write_p) {
		new_read_p = (dma->read_p + 1) % dma->size;
		ret = ops->depacketize(index,
				       data,
				       size,
				       processed,
				       &recv_time,
				       dma->packet_table[dma->read_p].vaddr,
				       dma->packet_table[dma->read_p].len);

		dma->read_p = new_read_p;
		pcount++;

		if (ret >= 0 && *t_stored < t_size) {
			*timestamps++ = recv_time;
			(*t_stored)++;
		}
		if (ret == 1)
			break;
	}

	if (ret != 1 && pcount > 0) {
		pr_debug("[%s] depacketize not enough. processed packet=%d(processed=%zu, ret=%d)\n",
			 __func__, pcount, *processed, ret);
		return -1;
	}

	return *processed;
}

int mse_packet_ctrl_take_out_packet_crf(
	int index,
	u64 *timestamp,
	int size,
	struct mse_packet_ctrl *dma)
{
	int ret;
	int new_read_p;
	int count;
	size_t crf_len;

	pr_debug("[%s] r=%d w=%d s=%d v=%p\n",
		 __func__,  dma->read_p, dma->write_p, dma->size,
		 dma->packet_table[dma->read_p].vaddr);

	if (dma->read_p == dma->write_p)
		return 0;

	new_read_p = (dma->read_p + 1) % dma->size;

	ret = mse_packetizer_crf_tstamp_audio_ops.depacketize(
		index, timestamp, size * sizeof(*timestamp),
		&crf_len,
		NULL,
		dma->packet_table[dma->read_p].vaddr,
		dma->packet_table[dma->read_p].len);

	if (ret > 0) {
		timestamp += ret / sizeof(u64);
		count = ret / sizeof(u64);
	} else {
		return ret;
	}

	dma->read_p = new_read_p;

	return count;
}
