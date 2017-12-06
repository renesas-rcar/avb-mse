/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2016-2017 Renesas Electronics Corporation

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
#include <linux/time.h>
#include <linux/hrtimer.h>

#include "ravb_mse_kernel.h"
#include "mse_ptp.h"

#define MAX_PTP_DEVICES    10
#define MAX_TIMESTAMPS     100

#define PTP_DUMMY_INTERVAL 3333333  /* 3.33..ms */

#define q_next(que, pos)        (((que)->pos + 1) % (que)->len)
#define q_empty(que)            ((que)->head == (que)->tail)

/* structs */
struct ptp_queue {
	int head;
	int tail;
	int len;
	u64 *timestamps;
};

struct ptp_device {
	int			index;
	int			ch;
	struct			hrtimer timer;
	int			timer_interval;
	struct ptp_queue	que;
	/* timestamp queue lock */
	spinlock_t		qlock;
};

/* total devices */
static struct ptp_device *g_ptp_devices[MAX_PTP_DEVICES];
DECLARE_BITMAP(ptp_dummy_device_map, MAX_PTP_DEVICES);
DEFINE_SPINLOCK(ptp_dummy_lock);

static void enqueue(struct ptp_queue *que, u64 ns)
{
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

	if (q_next(que, tail) == que->head)
		que->head = q_next(que, head);

	que->timestamps[que->tail] = ns;
	que->tail = q_next(que, tail);
}

static int dequeue(struct ptp_queue *que, u64 *ns)
{
	if (q_empty(que))
		return -1;

	*ns = que->timestamps[que->head];
	que->head = q_next(que, head);

	return 0;
}

static enum hrtimer_restart ptp_timestamp_callback(struct hrtimer *arg)
{
	struct ptp_device *dev;
	u64 ns;
	unsigned long flags;

	dev = container_of(arg, struct ptp_device, timer);

	if (!dev->timer_interval)
		return HRTIMER_NORESTART;

	hrtimer_add_expires_ns(&dev->timer, dev->timer_interval);

	/* Get time from system timer */
	ns = ktime_get_real_ns();

	spin_lock_irqsave(&dev->qlock, flags);

	/* add to queue */
	enqueue(&dev->que, ns);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return HRTIMER_RESTART;
}

static struct ptp_device *mse_ptp_get_dev(void *ptp_handle)
{
	struct ptp_device *dev = ptp_handle;

	if (!dev)
		return NULL;

	if (g_ptp_devices[dev->index] != dev)
		return NULL;

	return dev;
}

/*
 * Public functions
 */
int mse_ptp_get_time_dummy(u64 *ns)
{
	/* Get time from system timer */
	*ns = ktime_get_real_ns();

	return 0;
}

int mse_ptp_capture_stop_dummy(void *ptp_handle)
{
	struct ptp_device *dev;

	dev = mse_ptp_get_dev(ptp_handle);
	if (!dev)
		return -EINVAL;

	/* if already capture started, return success */
	if (!dev->timer_interval)
		return 0;

	/* timer stop */
	dev->timer_interval = 0;
	hrtimer_cancel(&dev->timer);

	/* unassing and free timestamp buffer */
	kfree(dev->que.timestamps);
	dev->que.timestamps = NULL;
	dev->que.len = 0;

	return 0;
}

int mse_ptp_capture_start_dummy(void *ptp_handle,
				int ch,
				int max_count)
{
	struct ptp_device *dev;
	void *timestamps;
	int ret;

	dev = mse_ptp_get_dev(ptp_handle);
	if (!dev)
		return -EINVAL;

	/* alloc timestamp buffer */
	timestamps = kcalloc(max_count + 1, sizeof(u64), GFP_KERNEL);
	if (!timestamps)
		return -ENOMEM;

	/* if already capture started, so stop it */
	if (dev->timer_interval) {
		ret = mse_ptp_capture_stop_dummy(ptp_handle);
		if (ret) {
			kfree(timestamps);
			return ret;
		}
	}

	/* assign timestamp buffer to queue */
	dev->que.timestamps = timestamps;
	dev->que.len = max_count + 1;

	/* start timer */
	dev->timer_interval = PTP_DUMMY_INTERVAL;
	hrtimer_start(&dev->timer,
		      ns_to_ktime(dev->timer_interval),
		      HRTIMER_MODE_REL);

	return 0;
}

int mse_ptp_get_timestamps_dummy(void *ptp_handle,
				 int req_count,
				 u64 *timestamps)
{
	struct ptp_device *dev;
	unsigned long flags;
	int i;

	dev = mse_ptp_get_dev(ptp_handle);
	if (!dev)
		return -EINVAL;

	/* if timer is NOT started */
	if (!dev->timer_interval)
		return -EPERM;

	spin_lock_irqsave(&dev->qlock, flags);

	for (i = 0; i < req_count; i++)
		if (dequeue(&dev->que, &timestamps[i]) < 0)
			break;

	spin_unlock_irqrestore(&dev->qlock, flags);

	return i;
}

void *mse_ptp_open_dummy(void)
{
	int index;
	struct ptp_device *dev;
	unsigned long flags;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	spin_lock_irqsave(&ptp_dummy_lock, flags);

	index = find_first_zero_bit(ptp_dummy_device_map, MAX_PTP_DEVICES);
	if (index >= ARRAY_SIZE(g_ptp_devices)) {
		spin_unlock_irqrestore(&ptp_dummy_lock, flags);
		mse_err("cannot register ptp dummy device\n");
		kfree(dev);
		return NULL;
	}

	set_bit(index, ptp_dummy_device_map);
	g_ptp_devices[index] = dev;
	dev->index = index;

	spin_unlock_irqrestore(&ptp_dummy_lock, flags);

	/* initialize ptp dummy device */
	spin_lock_init(&dev->qlock);

	/* init timer */
	hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timer_interval = 0;
	dev->timer.function = &ptp_timestamp_callback;

	mse_debug("ptp_handle=%p\n", dev);

	return dev;
}

int mse_ptp_close_dummy(void *ptp_handle)
{
	struct ptp_device *dev;
	unsigned long flags;
	int ret;

	dev = mse_ptp_get_dev(ptp_handle);
	if (!dev)
		return -EINVAL;

	/* if already capture started, so stop it */
	if (dev->timer_interval) {
		ret = mse_ptp_capture_stop_dummy(ptp_handle);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&ptp_dummy_lock, flags);

	g_ptp_devices[dev->index] = NULL;
	clear_bit(dev->index, ptp_dummy_device_map);

	spin_unlock_irqrestore(&ptp_dummy_lock, flags);

	kfree(dev);

	return 0;
}

void *mse_ptp_timer_open_dummy(u32 (*handler)(void *),
			       void *priv)
{
	return NULL;
}

int mse_ptp_timer_close_dummy(void *timer_handle)
{
	return -EPERM;
}

int mse_ptp_timer_start_dummy(void *timer_handle, u32 start)
{
	return -EPERM;
}

int mse_ptp_timer_cancel_dummy(void *timer_handle)
{
	return -EPERM;
}
