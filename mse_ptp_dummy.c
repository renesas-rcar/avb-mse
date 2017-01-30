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

static int enqueue(struct ptp_queue *que, struct ptp_clock_time *clock_time)
{
	int is_tail_equal_head = 0;

	mse_debug("START head=%d, tail=%d\n", que->head, que->tail);

	if (q_next(que->tail) == que->head)
		is_tail_equal_head = 1;

	que->timestamps[que->tail] = *clock_time;
	que->tail = q_next(que->tail);

	if (is_tail_equal_head) {
		que->head = q_next(que->head);
		is_tail_equal_head = 0;
	}

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

static void get_clock_time(struct ptp_clock_time *clock_time)
{
	struct timespec time;

	/* TODO get time from ptp driver */
	getnstimeofday(&time);
	clock_time->sec = time.tv_sec;
	clock_time->nsec = time.tv_nsec;
}

static enum hrtimer_restart ptp_timestamp_callbak(struct hrtimer *arg)
{
	struct ptp_device *dev;
	ktime_t ktime;
	struct ptp_queue *queue;
	struct ptp_clock_time clock_time;
	unsigned long flags;

	dev = container_of(arg, struct ptp_device, timer);

	spin_lock_irqsave(&dev->qlock, flags);

	ktime = ktime_set(0, dev->timer_delay);
	hrtimer_forward(&dev->timer,
			hrtimer_get_expires(&dev->timer),
			ktime);

	get_clock_time(&clock_time);

	/* add to queue */
	queue = &dev->que;
	enqueue(queue, &clock_time);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return HRTIMER_RESTART;
}

static int init_ptp_device(struct ptp_device *dev)
{
	ktime_t ktime;

	mse_debug("START\n");

	dev->que.head = 0;
	dev->que.tail = 0;

	spin_lock_init(&dev->qlock);

	/* init timer */
	hrtimer_init(&dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timer_delay = DELAY;
	dev->timer.function = &ptp_timestamp_callbak;

	/* start timer */
	ktime = ktime_set(0, dev->timer_delay);
	hrtimer_start(&dev->timer, ktime, HRTIMER_MODE_REL);

	mse_debug("END\n");

	return 0;
}

int mse_ptp_open_dummy(int *dev_id)
{
	int i;
	int ret;
	struct ptp_device *dev = NULL;

	for (i = 0; i < ARRAY_SIZE(g_ptp_devices); i++) {
		if (g_ptp_devices[i])
			continue;

		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev)
			return -ENOMEM;

		/* change to mse_debug */
		g_ptp_devices[i] = dev;
		mse_info("register ptp device index=%d\n", i);
		*dev_id = i;

		ret = init_ptp_device(dev);

		return 0;
	}

	mse_err("cannot register ptp device\n");

	return -1;
}

int mse_ptp_close_dummy(int dev_id)
{
	int ret;
	struct ptp_device *dev;

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		mse_err("invalid argument. index=%d\n", dev_id);
		return -EINVAL;
	}

	mse_info("index=%d\n", dev_id);

	dev = g_ptp_devices[dev_id];

	/* timer stop */
	ret = hrtimer_try_to_cancel(&dev->timer);
	if (ret)
		mse_err("The timer was still in use...\n");

	kfree(dev);
	g_ptp_devices[dev_id] = NULL;

	return 0;
}

int mse_ptp_get_time_dummy(int dev_id, struct ptp_clock_time *clock_time)
{
	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		mse_err("invalid argument. index=%d\n", dev_id);
		return -EINVAL;
	}

	if (!g_ptp_devices[dev_id]) {
		mse_err("invalid dev_id=%d\n", dev_id);
		return -EINVAL;
	}

	get_clock_time(clock_time);

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

	spin_lock(&dev->qlock);

	queue = &dev->que;

	if (queue->tail > queue->head)
		*count = queue->tail - queue->head;
	else
		*count = (queue->tail + MAX_TIMESTAMPS) - queue->head;

	if (*count == 0) {
		spin_unlock(&dev->qlock);
		return 0;
	}

	mse_debug("total=%d\n", *count);
	for (i = 0; i < *count; i++) {
		dequeue(queue, &clock_time);
		timestamps[i] = clock_time;
	}

	spin_unlock(&dev->qlock);

	mse_debug("END\n");

	return 0;
}
