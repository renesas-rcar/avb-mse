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

	pr_debug("[%s] START head=%d, tail=%d\n",
		__func__, que->head, que->tail);

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

static int get_clock_time(struct ptp_clock_time *clock_time)
{
	struct timespec time;

	/* TODO get time from ptp driver */
	getnstimeofday(&time);
	clock_time->sec = time.tv_sec;
	clock_time->nsec = time.tv_nsec;

	return 0;
}

static enum hrtimer_restart ptp_timestamp_callbak(struct hrtimer *arg)
{
	struct ptp_device *dev;
	ktime_t ktime;
	struct ptp_queue *queue;
	struct ptp_clock_time *clock_time;
	unsigned long flags;
	int ret;

	dev = container_of(arg, struct ptp_device, timer);

	spin_lock_irqsave(&dev->qlock, flags);

	ktime = ktime_set(0, dev->timer_delay);
	hrtimer_forward(&dev->timer,
			hrtimer_get_expires(&dev->timer),
			ktime);

	clock_time = kzalloc(sizeof(*clock_time), GFP_ATOMIC);
	if (!clock_time) {
		pr_warn("[%s] failed allocate clock_time\n", __func__);
		spin_unlock(&dev->qlock);
		return HRTIMER_RESTART;
	}

	ret = get_clock_time(clock_time);
	if (ret) {
		pr_warn("[%s] failed get_clock_time\n", __func__);
		spin_unlock(&dev->qlock);
		return HRTIMER_RESTART;
	}

	/* add to queue */
	queue = &dev->que;
	enqueue(queue, clock_time);

	spin_unlock_irqrestore(&dev->qlock, flags);

	return HRTIMER_RESTART;
}

static int init_ptp_device(struct ptp_device *dev)
{
	ktime_t ktime;

	pr_debug("[%s] START\n", __func__);

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

	pr_debug("[%s] END\n", __func__);

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
		if (!dev) {
			pr_err("[%s] could not create ptp_device\n", __func__);
			return -ENOMEM;
		}

		/* change to pr_debug */
		g_ptp_devices[i] = dev;
		pr_info("[%s] registered ptp device index=%d\n",
			__func__, i);
		*dev_id = i;

		ret = init_ptp_device(dev);

		return 0;
	}

	pr_err("[%s] unregistered ptp device\n", __func__);

	return -1;
}

int mse_ptp_close_dummy(int dev_id)
{
	int ret;
	struct ptp_device *dev;

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, dev_id);
		return -EINVAL;
	}

	pr_info("[%s] index=%d\n", __func__, dev_id);

	dev = g_ptp_devices[dev_id];

	/* timer stop */
	ret = hrtimer_try_to_cancel(&dev->timer);
	if (ret)
		pr_err("[%s] The timer was still in use...\n", __func__);

	kfree(dev);
	g_ptp_devices[dev_id] = NULL;

	return 0;
}

int mse_ptp_get_time_dummy(int dev_id, struct ptp_clock_time *clock_time)
{
	int ret = 0;

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, dev_id);
		return -EINVAL;
	}

	if (!g_ptp_devices[dev_id]) {
		pr_err("[%s] invalid dev_id=%d\n", __func__, dev_id);
		return -EINVAL;
	}

	ret = get_clock_time(clock_time);

	return ret;
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

	pr_debug("[%s] START\n", __func__);

	if ((dev_id < 0) || (dev_id >= MAX_PTP_DEVICES)) {
		pr_err("[%s] invalid argument. index=%d\n", __func__, dev_id);
		return -EINVAL;
	}

	dev = g_ptp_devices[dev_id];
	if (!dev) {
		pr_err("[%s] invalid dev_id=%d\n", __func__, dev_id);
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

	pr_debug("[%s] total=%d\n", __func__, *count);
	for (i = 0; i < *count; i++) {
		dequeue(queue, &clock_time);
		timestamps[i] = clock_time;
	}

	spin_unlock(&dev->qlock);

	pr_debug("[%s] END\n", __func__);

	return 0;
}
