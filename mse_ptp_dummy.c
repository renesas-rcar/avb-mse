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
#include <linux/ptp_clock.h>

#include "ravb_mse_kernel.h"

#define MAX_PTP_DEVICES  10
#define DELAY            3333333  /* 3.33..ms */

#define MAX_TIMESTAMPS   100

#define q_next(n)        (((n) + 1) % MAX_TIMESTAMPS)

/* structs */
struct ptp_queue {
	int head;
	int tail;
	struct ptp_clock_time timestamps[MAX_TIMESTAMPS];
};

struct ptp_device {
	struct ptp_queue	que;
	struct			hrtimer timer;
	int			timer_delay;
	spinlock_t		qlock;
};

/* total devices */
static struct ptp_device *g_ptp_devices[MAX_PTP_DEVICES];
DECLARE_BITMAP(ptp_dummy_device_map, MAX_PTP_DEVICES);
DEFINE_SPINLOCK(ptp_dummy_lock);

static int enqueue(struct ptp_queue *que, struct ptp_clock_time *clock_time)
{
	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

	if (q_next(que->tail) == que->head)
		que->head = q_next(que->head);

	que->timestamps[que->tail] = *clock_time;
	que->tail = q_next(que->tail);

	return 0;
}

static int dequeue(struct ptp_queue *que, struct ptp_clock_time *clock_time)
{
	if (que->head == que->tail)
		return -1;

	*clock_time = que->timestamps[que->head];
	que->head = q_next(que->head);

	return 0;
}

static enum hrtimer_restart ptp_timestamp_callback(struct hrtimer *arg)
{
	struct ptp_device *dev;
	struct ptp_clock_time clock_time;
	struct timespec64 time;
	unsigned long flags;

	dev = container_of(arg, struct ptp_device, timer);

	if (!dev->timer_delay)
		return HRTIMER_NORESTART;

	hrtimer_forward(&dev->timer,
			hrtimer_get_expires(&dev->timer),
			ns_to_ktime(dev->timer_delay));

	/* Get time from system timer */
	getnstimeofday64(&time);
	clock_time.sec = time.tv_sec;
	clock_time.nsec = time.tv_nsec;

	spin_lock_irqsave(&dev->qlock, flags);

	/* add to queue */
	enqueue(&dev->que, &clock_time);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return HRTIMER_RESTART;
}

int mse_ptp_open_dummy(int *dev_id)
{
	int index;
	struct ptp_device *dev = NULL;
	unsigned long flags;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_irqsave(&ptp_dummy_lock, flags);

	index = find_first_zero_bit(ptp_dummy_device_map, MAX_PTP_DEVICES);
	if (index >= ARRAY_SIZE(g_ptp_devices)) {
		mse_err("cannot register ptp dummy device\n");
		spin_unlock_irqrestore(&ptp_dummy_lock, flags);
		kfree(dev);
		return -EBUSY;
	}

	g_ptp_devices[index] = dev;
	set_bit(index, ptp_dummy_device_map);

	spin_unlock_irqrestore(&ptp_dummy_lock, flags);

	/* initialize ptp dummy device */
	spin_lock_init(&dev->qlock);

	/* init timer */
	hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timer_delay = DELAY;
	dev->timer.function = &ptp_timestamp_callback;

	/* start timer */
	hrtimer_start(&dev->timer,
		      ns_to_ktime(dev->timer_delay),
		      HRTIMER_MODE_REL);

	*dev_id = index;

	mse_debug("index=%d\n", index);

	return 0;
}

int mse_ptp_close_dummy(int dev_id)
{
	struct ptp_device *dev;
	unsigned long flags;

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		mse_err("invalid argument. index=%d\n", dev_id);
		return -EINVAL;
	}

	mse_debug("index=%d\n", dev_id);

	spin_lock_irqsave(&ptp_dummy_lock, flags);

	dev = g_ptp_devices[dev_id];
	g_ptp_devices[dev_id] = NULL;
	clear_bit(dev_id, ptp_dummy_device_map);

	spin_unlock_irqrestore(&ptp_dummy_lock, flags);

	/* timer stop */
	dev->timer_delay = 0;
	hrtimer_cancel(&dev->timer);

	kfree(dev);

	return 0;
}

int mse_ptp_get_time_dummy(int dev_id, struct ptp_clock_time *clock_time)
{
	struct timespec64 time;

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		mse_err("invalid argument. index=%d\n", dev_id);
		return -EINVAL;
	}

	if (!g_ptp_devices[dev_id]) {
		mse_err("invalid dev_id=%d\n", dev_id);
		return -EINVAL;
	}

	/* Get time from system timer */
	getnstimeofday64(&time);
	clock_time->sec = time.tv_sec;
	clock_time->nsec = time.tv_nsec;

	return 0;
}

int mse_ptp_get_timestamps_dummy(int dev_id,
				 int ch,
				 int *count,
				 struct ptp_clock_time timestamps[])
{
	struct ptp_device *dev;
	struct ptp_queue *queue;
	struct ptp_clock_time clock_time;
	unsigned long flags;
	int i;

	mse_debug("START\n");

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		mse_err("invalid argument. index=%d\n", dev_id);
		return -EINVAL;
	}

	dev = g_ptp_devices[dev_id];
	if (!dev) {
		mse_err("invalid dev_id=%d\n", dev_id);
		return -EINVAL;
	}

	spin_lock_irqsave(&dev->qlock, flags);

	queue = &dev->que;

	for (i = 0; i < MAX_TIMESTAMPS; i++) {
		if (dequeue(queue, &clock_time) < 0)
			break;
		timestamps[i] = clock_time;
	}

	spin_unlock_irqrestore(&dev->qlock, flags);

	*count = i;

	mse_debug("END\n");

	return 0;
}
