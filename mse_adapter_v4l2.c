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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-memops.h>

#include "ravb_mse_kernel.h"
#include "jpeg.h"

/*********************/
/* Name of driver    */
/*********************/
#define MSE_ADAPTER_V4L2_NAME_BASE   "ravb_mse"
#define MSE_ADAPTER_V4L2_DRIVER_NAME "ravb_mse.v4l2"

/*********************/
/* Number of devices */
/*********************/
#define MSE_ADAPTER_V4L2_DEVICE_MAX		MSE_ADAPTER_MEDIA_MAX
#define MSE_ADAPTER_V4L2_DEVICE_VIDEO_DEFAULT	2
#define MSE_ADAPTER_V4L2_DEVICE_MPEG2TS_DEFAULT	2

#define NUM_BUFFERS 2
#define NUM_PLANES  1

#define MSE_ADAPTER_V4L2_MIN_VIDEO_SIZEIMAGE    (512 * 1024)
#define MSE_ADAPTER_V4L2_MPEG2TS_BYTESPERLINE   (188 * 192)
#define MSE_ADAPTER_V4L2_MPEG2TS_SIZEIMAGE \
	(MSE_ADAPTER_V4L2_MPEG2TS_BYTESPERLINE * 14)
#define MSE_ADAPTER_V4L2_TEMP_BUF_MAXSIZE       (2048 * 1024)

/*************/
/* Structure */
/*************/
/* Buffer information */
struct v4l2_adapter_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct v4l2_adapter_temp_buffer {
	size_t bytesused;
	size_t length;
	unsigned char *buf;
	bool prepared;
};

/* File handle information */
struct v4l2_adapter_fh {
	struct v4l2_fh fh;
	struct v4l2_adapter_device *dev;
};

/* Device information */
struct v4l2_adapter_device {
	struct v4l2_adapter_fh	*owner;
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	/* mutex lock */
	struct mutex		mutex_vb2;        /* lock for vb2_queue */
	struct v4l2_pix_format	format;
	struct v4l2_fract	frameintervals;
	struct vb2_queue	q_cap;
	struct vb2_queue	q_out;
	/* spin lock */
	spinlock_t		lock_buf_list;    /* lock for buf_list */
	/* list of buffer queued from v4l2 core */
	struct list_head	buf_list;
	/* list of buffer prepared */
	struct list_head	prepared_buf_list;
	/* list of buffer mse processing */
	struct list_head	stream_buf_list;
	unsigned int		sequence;
	/* for buffer management debug */
	unsigned int		queued;
	unsigned int		dequeued;
	/* index for register */
	int			index_mse;
	enum MSE_TYPE		type;
	/* index for MSE instance */
	int			index_instance;
	bool			f_mse_open;
	/* previous trans buffer for checking buffer overwite */
	void			*prev;

	/* temp video buffer */
	bool			use_temp_buffer;
	struct v4l2_adapter_temp_buffer		temp_buf[NUM_BUFFERS];
	void *temp_buf_base;
	int temp_w;
	int temp_r;

	/* mpeg2ts stream type */
	enum MSE_MPEG2TS_TYPE	mpeg2ts_type;
};

/* Format information */
struct v4l2_adapter_fmt {
	u32			fourcc;
	unsigned int		min_width;
	unsigned int		max_width;
	unsigned int		step_width;
	unsigned int		min_height;
	unsigned int		max_height;
	unsigned int		step_height;
};

static const struct v4l2_fmtdesc g_formats_video[] = {
	{
		.description = "H264 with start codes",
		.pixelformat = V4L2_PIX_FMT_H264,
	},
	{
		.description = "H264 without start codes",
		.pixelformat = V4L2_PIX_FMT_H264_NO_SC,
	},
	{
		.description = "Motion-JPEG",
		.pixelformat = V4L2_PIX_FMT_MJPEG,
	},
};

static const struct v4l2_fmtdesc g_formats_mpeg[] = {
	{
		.description = "MPEG-1/2/4 Multiplexed",
		.pixelformat = V4L2_PIX_FMT_MPEG,
	},
};

/* Playback, capture video format sizes */
static const struct v4l2_adapter_fmt g_mse_adapter_v4l2_fmt_sizes_video[] = {
	/* limited H.264 picture size to range of
	 * R-Car Hardware video decoder (VCP4).
	 */
	{
		.fourcc		= V4L2_PIX_FMT_H264,
		.min_width	= 80,
		.max_width	= 3840,
		.step_width	= 2,
		.min_height	= 80,
		.max_height	= 2160,
		.step_height	= 2,
	},
	/* limited H.264 picture size to range of
	 * R-Car Hardware video decoder (VCP4).
	 */
	{
		.fourcc		= V4L2_PIX_FMT_H264_NO_SC,
		.min_width	= 80,
		.max_width	= 3840,
		.step_width	= 2,
		.min_height	= 80,
		.max_height	= 2160,
		.step_height	= 2,
	},
	/* limited MJPEG picture size to range of
	 * AVTP format(same as RTP), see RFC 2435 3.1.5,3.1.6
	 */
	{
		.fourcc		= V4L2_PIX_FMT_MJPEG,
		.min_width	= 8,
		.max_width	= 2040,
		.step_width	= 8,
		.min_height	= 8,
		.max_height	= 2040,
		.step_height	= 8,
	},
};

static const struct v4l2_adapter_fmt g_mse_adapter_v4l2_fmt_sizes_mpeg[] = {
	/* limited MPEG2-TS picture size to range of
	 * R-Car Hardware video decoder (VCP4).
	 */
	{
		.fourcc		= V4L2_PIX_FMT_MPEG,
		.min_width	= 80,
		.max_width	= 3840,
		.step_width	= 2,
		.min_height	= 80,
		.max_height	= 2160,
		.step_height	= 2,
	},
};

/*******************/
/* global variable */
/*******************/
static int v4l2_video_devices = MSE_ADAPTER_V4L2_DEVICE_VIDEO_DEFAULT;
module_param(v4l2_video_devices, int, 0440);
static int v4l2_mpeg2ts_devices = MSE_ADAPTER_V4L2_DEVICE_MPEG2TS_DEFAULT;
module_param(v4l2_mpeg2ts_devices, int, 0440);
static int v4l2_devices;

/************/
/* Function */
/************/
#define vadp_buffer_done(vadp_dev, vadp_buf, state) \
	do { \
		struct v4l2_adapter_device *__dev = (vadp_dev); \
		struct v4l2_adapter_buffer *__buf = (vadp_buf); \
		mse_debug("queued=%u dequeued=%u\n", \
			__dev->queued, ++(__dev->dequeued)); \
		list_del(&__buf->list); \
		vb2_buffer_done(&__buf->vb.vb2_buf, (state)); \
	} while (0)

#define v4l2_type_stringfy(type) \
		(V4L2_TYPE_IS_OUTPUT(type) ? "output" : "capture")

/*
 * temp_buffer related functions
 */
static void temp_buffer_free(struct v4l2_adapter_device *vadp_dev)
{
	int i;
	struct v4l2_adapter_temp_buffer *temp;

	/* NOT allocated, skip */
	if (!vadp_dev->temp_buf_base)
		return;

	for (i = 0; i < NUM_BUFFERS; i++) {
		temp = &vadp_dev->temp_buf[i];
		temp->buf = NULL;
		temp->length = 0;
	}

	kfree(vadp_dev->temp_buf_base);
	vadp_dev->temp_buf_base = NULL;
}

static int temp_buffer_alloc(struct v4l2_adapter_device *vadp_dev)
{
	int i;
	struct v4l2_adapter_temp_buffer *temp;
	u32 length;
	u8 *buf;

	/* already allocated, skip allocate memory */
	if (vadp_dev->temp_buf_base)
		return 0;

	length = min_t(u32, vadp_dev->format.sizeimage,
		       MSE_ADAPTER_V4L2_TEMP_BUF_MAXSIZE);

	buf = kmalloc_array((size_t)length, NUM_BUFFERS, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	vadp_dev->temp_buf_base = buf;
	vadp_dev->temp_w = 0;
	vadp_dev->temp_r = 0;

	for (i = 0; i < NUM_BUFFERS; i++) {
		temp = &vadp_dev->temp_buf[i];
		temp->length = (size_t)length;
		temp->buf = buf + (temp->length * i);
		temp->prepared = false;
		temp->bytesused = 0;
	}

	return 0;
}

static struct v4l2_adapter_temp_buffer *temp_buffer_get_prepared(
					struct v4l2_adapter_device *vadp_dev)
{
	struct v4l2_adapter_temp_buffer *temp;

	temp = &vadp_dev->temp_buf[vadp_dev->temp_r];
	if (!temp->prepared)
		return NULL;

	return temp;
}

#define MPEG2TS_TS_SIZE         (188)
#define MPEG2TS_SYNC            (0x47)
#define MPEG2TS_M2TS_OFFSET     (4)
#define MPEG2TS_M2TS_SIZE       (MPEG2TS_M2TS_OFFSET + MPEG2TS_TS_SIZE)
static bool is_mpeg2ts_ts(unsigned char *tsp)
{
	if (tsp[0] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_TS_SIZE] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_TS_SIZE * 2] == MPEG2TS_SYNC)
		return true;
	else
		return false;
}

static bool is_mpeg2ts_m2ts(unsigned char *tsp)
{
	if (tsp[MPEG2TS_M2TS_OFFSET] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_M2TS_OFFSET + MPEG2TS_M2TS_SIZE] == MPEG2TS_SYNC &&
	    tsp[MPEG2TS_M2TS_OFFSET + MPEG2TS_M2TS_SIZE * 2] == MPEG2TS_SYNC)
		return true;
	else
		return false;
}

static int get_mpeg2ts_format(unsigned char *buf)
{
	if (is_mpeg2ts_ts(buf))
		return MSE_MPEG2TS_TYPE_TS;
	else if (is_mpeg2ts_m2ts(buf))
		return MSE_MPEG2TS_TYPE_M2TS;
	else
		return -1;
}

static bool tspacket_size_is_valid(enum MSE_MPEG2TS_TYPE type,
				   size_t size)
{
	if (type == MSE_MPEG2TS_TYPE_M2TS)
		return !(size % MPEG2TS_M2TS_SIZE);
	else if (type == MSE_MPEG2TS_TYPE_TS)
		return !(size % MPEG2TS_TS_SIZE);
	else
		return false;
}

static bool jpeg_frame_is_valid(unsigned char *buf, size_t size)
{
	size_t offset = 0;

	if ((buf[offset] != JPEG_MARKER) ||
	    (buf[offset + 1] != JPEG_MARKER_KIND_SOI))
		return false;

	offset = size - JPEG_MARKER_SIZE_LENGTH;

	if ((buf[offset] != JPEG_MARKER) ||
	    (buf[offset + 1] != JPEG_MARKER_KIND_EOI))
		return false;

	return true;
}

static int temp_buffer_check_type(struct v4l2_adapter_device *vadp_dev,
				  unsigned char *buf,
				  size_t size)
{
	int ret;
	u8 mk;
	size_t offset = 0;

	switch (vadp_dev->format.pixelformat) {
	case V4L2_PIX_FMT_MPEG:
		ret = get_mpeg2ts_format(buf);
		vadp_dev->mpeg2ts_type = ret;
		vadp_dev->use_temp_buffer = !tspacket_size_is_valid(ret, size);
		break;

	case V4L2_PIX_FMT_MJPEG:
		mk = jpeg_get_marker(buf, size, &offset);
		ret = (mk == JPEG_MARKER_KIND_SOI) ? 0 : -1;
		vadp_dev->use_temp_buffer = !jpeg_frame_is_valid(buf, size);
		break;

	default:
		ret = 0;
		vadp_dev->use_temp_buffer = false;
		break;
	}

	return ret;
}

static int temp_buffer_write_mpeg2ts(struct v4l2_adapter_device *vadp_dev,
				     unsigned char *buf,
				     size_t size)
{
	struct v4l2_adapter_temp_buffer *temp;
	unsigned char *copy_from;
	size_t bytesused, length, pos;
	size_t copy_size;
	size_t psize;
	int temp_w = vadp_dev->temp_w;
	int temp_r = vadp_dev->temp_r;

	temp = &vadp_dev->temp_buf[temp_w];

	if (temp->prepared)
		return -EAGAIN;

	bytesused = temp->bytesused;
	length = temp->length;

	mse_debug("temp_w=%d bytesused=%zu buf=%p size=%zu\n",
		  temp_w, bytesused, buf, size);

	if (vadp_dev->mpeg2ts_type == MSE_MPEG2TS_TYPE_M2TS)
		psize = MPEG2TS_M2TS_SIZE;
	else
		psize = MPEG2TS_TS_SIZE;

	if (!((temp_w + NUM_BUFFERS - temp_r - 1) % NUM_BUFFERS > 0)) {
		if (bytesused + size % psize) {
			mse_debug("temp buffer has no enough area\n");
			return -EAGAIN;
		}
	}

	copy_from = buf;
	copy_size = size;

	if (bytesused + copy_size > length) {
		mse_debug("temp buffer overrun %zu/%zu\n",
			  bytesused + copy_size, temp->length);
		return -EAGAIN;
	}

	/* copy stream */
	memcpy(temp->buf + bytesused, copy_from, copy_size);
	bytesused += copy_size;

	/* adjust bytesused */
	pos = bytesused - bytesused % psize;
	temp->bytesused = pos;
	temp->prepared = true;
	vadp_dev->temp_w = (temp_w + 1) % NUM_BUFFERS;

	if (pos == bytesused)
		return 0;

	/* copy remain data */
	copy_from = temp->buf + pos;
	copy_size = bytesused - pos;

	temp = &vadp_dev->temp_buf[vadp_dev->temp_w];
	memcpy(temp->buf + temp->bytesused, copy_from, copy_size);
	temp->bytesused += copy_size;

	return 0;
}

static int temp_buffer_write_mjpeg(struct v4l2_adapter_device *vadp_dev,
				   unsigned char *buf,
				   size_t size)
{
	struct v4l2_adapter_temp_buffer *temp;
	unsigned char *copy_from;
	size_t bytesused, length, pos;
	size_t copy_size = size;
	int temp_w = vadp_dev->temp_w;
	int temp_r = vadp_dev->temp_r;

	temp = &vadp_dev->temp_buf[temp_w];

	if (temp->prepared)
		return -EAGAIN;

	bytesused = temp->bytesused;
	length = temp->length;

	mse_debug("temp_w=%d bytesused=%zu buf=%p size=%zu\n",
		  temp_w, bytesused, buf, size);

	pos = jpeg_search_eoi(buf, size, 0);
	if (!((temp_w + NUM_BUFFERS - temp_r - 1) % NUM_BUFFERS > 0)) {
		if (pos != size) {
			mse_debug("temp buffer has no enough area\n");
			return -EAGAIN;
		}
	}

	copy_from = buf;
	copy_size = pos;

	if (bytesused + copy_size > length) {
		mse_debug("temp buffer overrun %zu/%zu\n",
			  bytesused + copy_size, temp->length);
		return -EAGAIN;
	}

	/* copy stream until period position */
	memcpy(temp->buf + bytesused, copy_from, copy_size);
	temp->bytesused += copy_size;

	if (jpeg_frame_is_valid(temp->buf, temp->bytesused)) {
		temp->prepared = true;
		vadp_dev->temp_w = (temp_w + 1) % NUM_BUFFERS;
	}

	if (size == pos)
		return 0;

	/* copy remain data */
	copy_from = buf + pos;
	copy_size = size - pos;

	temp = &vadp_dev->temp_buf[vadp_dev->temp_w];
	memcpy(temp->buf + temp->bytesused, copy_from, copy_size);
	temp->bytesused += copy_size;

	return 0;
}

static int temp_buffer_write(struct v4l2_adapter_device *vadp_dev,
			     unsigned char *buf,
			     size_t size)
{
	if (vadp_dev->format.pixelformat == V4L2_PIX_FMT_MPEG)
		return temp_buffer_write_mpeg2ts(vadp_dev, buf, size);
	else if (vadp_dev->format.pixelformat == V4L2_PIX_FMT_MJPEG)
		return temp_buffer_write_mjpeg(vadp_dev, buf, size);
	else
		return 0;
}

static inline const char *convert_type_to_str(enum MSE_TYPE type)
{
	switch (type) {
	case MSE_TYPE_ADAPTER_VIDEO:
		return "video";

	case MSE_TYPE_ADAPTER_MPEG2TS:
		return "mpeg2ts";

	default:
		return "";
	}
}

static inline struct v4l2_adapter_fh *to_v4l2_adapter_fh(struct file *filp)
{
	return container_of(filp->private_data, struct v4l2_adapter_fh, fh);
}

static inline struct v4l2_adapter_buffer *to_v4l2_adapter_buffer(
						struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct v4l2_adapter_buffer, vb);
}

/* Get V4L2 Adapter device ownership */
static int vadp_owner_get(struct v4l2_adapter_fh *vadp_fh)
{
	unsigned long flags;
	int ret = 0;
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);

	if (!vadp_dev->owner)
		vadp_dev->owner = vadp_fh;
	else if (vadp_dev->owner != vadp_fh)
		ret = -EBUSY;

	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	return ret;
}

/* Put V4L2 Adapter device ownership */
static void vadp_owner_put(struct v4l2_adapter_fh *vadp_fh)
{
	unsigned long flags;
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);

	vadp_dev->owner = NULL;

	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);
}

/* Test V4L2 Adapter device ownership */
static int vadp_owner_is(struct v4l2_adapter_fh *vadp_fh)
{
	unsigned long flags;
	int ret;
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);

	ret = (vadp_dev->owner == vadp_fh);

	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	return ret;
}

static int try_mse_open(struct v4l2_adapter_device *vadp_dev,
			enum v4l2_buf_type i)
{
	bool tx = V4L2_TYPE_IS_OUTPUT(i);
	int index;

	if (vadp_dev->f_mse_open)
		return 0;

	/* probe is not finish yet */
	if (vadp_dev->index_mse == MSE_INDEX_UNDEFINED) {
		mse_info("probe is not finish yet\n");
		return 0;
	}

	index = mse_open(vadp_dev->index_mse, tx);
	if (index < 0)
		return index;

	vadp_dev->index_instance = index;
	vadp_dev->f_mse_open = true;

	return 0;
}

static int try_mse_close(struct v4l2_adapter_device *vadp_dev)
{
	int err;

	if (!vadp_dev->f_mse_open)
		return 0;

	/* probe is not finish yet */
	if (vadp_dev->index_mse == MSE_INDEX_UNDEFINED) {
		mse_info("probe is not finish yet\n");
		return 0;
	}

	if (vadp_dev->index_instance == MSE_INDEX_UNDEFINED) {
		mse_info("mse_start is not finish yet\n");
		return 0;
	}

	err = mse_close(vadp_dev->index_instance);
	if (err < 0)
		return err;

	vadp_dev->f_mse_open = false;

	return 0;
}

static int mse_adapter_v4l2_fop_open(struct file *filp)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_adapter_fh *vadp_fh;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	vadp_fh = kzalloc(sizeof(*vadp_fh), GFP_KERNEL);
	if (!vadp_fh)
		return -ENOMEM;

	v4l2_fh_init(&vadp_fh->fh, &vadp_dev->vdev);
	v4l2_fh_add(&vadp_fh->fh);

	vadp_fh->dev = vadp_dev;
	filp->private_data = &vadp_fh->fh;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_fop_release(struct file *filp)
{
	int err;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_requestbuffers req;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_owner_is(vadp_fh)) {
		if (vb2_is_busy(vadp_dev->vdev.queue)) {
			mse_debug("vb2 is busy, try queue free\n");

			vb2_ioctl_streamoff(filp, filp->private_data,
					    vadp_dev->vdev.queue->type);

			req.count = 0;
			req.type = vadp_dev->vdev.queue->type;
			req.memory = V4L2_MEMORY_MMAP;
			err = vb2_ioctl_reqbufs(filp,
						filp->private_data,
						&req);
			if (err < 0) {
				mse_err("Failed vb2_ioctl_reqbufs()\n");
				return err;
			}
		}

		if (vadp_dev->f_mse_open) {
			err = try_mse_close(vadp_dev);
			if (err < 0) {
				mse_err("Failed mse_close()\n");
				return err;
			}
		}
		vadp_owner_put(vadp_fh);
	}

	filp->private_data = NULL;
	v4l2_fh_del(&vadp_fh->fh);
	v4l2_fh_exit(&vadp_fh->fh);
	kfree(vadp_fh);

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_querycap(struct file *filp,
				     void *priv,
				     struct v4l2_capability *vcap)
{
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	strlcpy(vcap->driver, MSE_ADAPTER_V4L2_DRIVER_NAME,
		sizeof(vcap->driver));
	strlcpy(vcap->card, vadp_dev->vdev.name, sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:%s",
		 vadp_dev->v4l2_dev.name);
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT |
			    V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_enum_fmt_vid(struct file *filp,
					 void *priv,
					 struct v4l2_fmtdesc *fmt)
{
	unsigned int index;
	const struct v4l2_fmtdesc *fmtdesc;
	const struct v4l2_fmtdesc *fmtbase;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	enum v4l2_buf_type buf_type;
	int fmt_size;

	mse_debug("START fmt->index=%d\n", fmt->index);

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_dev->type == MSE_TYPE_ADAPTER_VIDEO) {
		fmtbase = g_formats_video;
		fmt_size = ARRAY_SIZE(g_formats_video);
	} else if (vadp_dev->type == MSE_TYPE_ADAPTER_MPEG2TS) {
		fmtbase = g_formats_mpeg;
		fmt_size = ARRAY_SIZE(g_formats_mpeg);
	} else {
		mse_err("Failed vdev type=%d\n", vadp_dev->type);
		return -EINVAL;
	}

	if (fmt->index >= fmt_size) {
		mse_debug("fmt->index(%d) is equal or bigger than %d\n",
			  fmt->index, fmt_size);
		return -EINVAL;
	}

	buf_type = fmt->type;

	index = fmt->index;
	memset(fmt, 0, sizeof(*fmt));

	fmtdesc = &fmtbase[index];

	fmt->index = index;
	fmt->type = buf_type;
	strcpy(fmt->description, fmtdesc->description);
	fmt->pixelformat = fmtdesc->pixelformat;

	mse_debug("END format: %s\n", fmtdesc->description);

	return 0;
}

static const struct v4l2_fmtdesc *get_fmtdesc(
					const struct v4l2_fmtdesc *fmtdescs,
					int arr_size,
					struct v4l2_format *fmt)
{
	int i;
	const struct v4l2_fmtdesc *fmtdesc;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (i = 0; i < arr_size; i++) {
		fmtdesc = &fmtdescs[i];
		if (fmtdesc->pixelformat == pix->pixelformat)
			return fmtdesc;
	}

	return NULL;
}

static void get_fmt_sizes(const struct v4l2_adapter_fmt *dflt_fmts,
			  int arr_size,
			  struct v4l2_format *fmt)
{
	int i;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (i = 0; i < arr_size; i++) {
		if (pix->pixelformat != dflt_fmts[i].fourcc)
			continue;

		if (pix->width > dflt_fmts[i].max_width)
			pix->width = dflt_fmts[i].max_width;
		else if (pix->width < dflt_fmts[i].min_width)
			pix->width = dflt_fmts[i].min_width;
		else
			pix->width -= (pix->width % dflt_fmts[i].step_width);

		if (pix->height > dflt_fmts[i].max_height)
			pix->height = dflt_fmts[i].max_height;
		else if (pix->height < dflt_fmts[i].min_height)
			pix->height = dflt_fmts[i].min_height;
		else
			pix->height -= (pix->height % dflt_fmts[i].step_height);
		break;
	}
}

static int mse_adapter_v4l2_try_fmt_vid(struct file *filp,
					void *priv,
					struct v4l2_format *fmt)
{
	const struct v4l2_fmtdesc *fmtdesc;
	const struct v4l2_fmtdesc *fmtbase;
	const struct v4l2_adapter_fmt *vadp_fmt;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct video_device *vdev;
	int fmt_size, vadp_fmt_size;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_owner_get(vadp_fh))
		return -EBUSY;

	vdev = &vadp_dev->vdev;

	if (vadp_dev->type == MSE_TYPE_ADAPTER_VIDEO) {
		fmtbase = g_formats_video;
		fmt_size = ARRAY_SIZE(g_formats_video);
		vadp_fmt = g_mse_adapter_v4l2_fmt_sizes_video;
		vadp_fmt_size = ARRAY_SIZE(g_mse_adapter_v4l2_fmt_sizes_video);
	} else if (vadp_dev->type == MSE_TYPE_ADAPTER_MPEG2TS) {
		fmtbase = g_formats_mpeg;
		fmt_size = ARRAY_SIZE(g_formats_mpeg);
		vadp_fmt = g_mse_adapter_v4l2_fmt_sizes_mpeg;
		vadp_fmt_size = ARRAY_SIZE(g_mse_adapter_v4l2_fmt_sizes_mpeg);
	} else {
		mse_err("Failed vdev type=%d\n", vadp_dev->type);
		return -EPERM;
	}

	fmtdesc = get_fmtdesc(fmtbase, fmt_size, fmt);
	if (!fmtdesc) {
		mse_info("Unknown fourcc format=(0x%08x)\n", pix->pixelformat);
		fmtdesc = &fmtbase[0];
		pix->pixelformat = fmtdesc->pixelformat;
	}

	if (pix->field != V4L2_FIELD_NONE &&
	    pix->field != V4L2_FIELD_INTERLACED)
		pix->field = V4L2_FIELD_NONE;

	get_fmt_sizes(vadp_fmt, vadp_fmt_size, fmt);

	if (vadp_dev->type == MSE_TYPE_ADAPTER_VIDEO) {
		pix->bytesperline = pix->width * 2;
		pix->sizeimage = pix->bytesperline * pix->height;
		if (pix->sizeimage < MSE_ADAPTER_V4L2_MIN_VIDEO_SIZEIMAGE) {
			pix->sizeimage = rounddown(
				MSE_ADAPTER_V4L2_MIN_VIDEO_SIZEIMAGE,
				pix->bytesperline);
		}
	} else {
		pix->bytesperline = MSE_ADAPTER_V4L2_MPEG2TS_BYTESPERLINE;
		pix->sizeimage = MSE_ADAPTER_V4L2_MPEG2TS_SIZEIMAGE;
	}

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_g_fmt_vid(struct file *filp,
				      void *priv,
				      struct v4l2_format *fmt)
{
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	*pix = vadp_dev->format;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_s_fmt_vid(struct file *filp,
				      void *priv,
				      struct v4l2_format *fmt)
{
	int err;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct vb2_queue *vq;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_owner_get(vadp_fh))
		return -EBUSY;

	if (V4L2_TYPE_IS_OUTPUT(fmt->type))
		vq = &vadp_dev->q_out;
	else
		vq = &vadp_dev->q_cap;

	if (vb2_is_busy(vq)) {
		mse_err("Failed vb2 is busy\n");
		return -EBUSY;
	}

	err = mse_adapter_v4l2_try_fmt_vid(filp, priv, fmt);
	if (err < 0) {
		mse_err("Failed capture_try_fmt_vid_cap()\n");
		return err;
	}

	vadp_dev->format = *pix;
	vadp_dev->vdev.queue = vq;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_streamon(struct file *filp,
				     void *priv,
				     enum v4l2_buf_type i)
{
	int err;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_owner_get(vadp_fh))
		return -EBUSY;

	if (i != vadp_dev->vdev.queue->type)
		return -EINVAL;

	err = try_mse_open(vadp_dev, i);
	if (err) {
		mse_err("Failed mse_open()\n");
		return err;
	}

	if (V4L2_TYPE_IS_OUTPUT(i)) {
		if ((vadp_dev->format.pixelformat == V4L2_PIX_FMT_MPEG) ||
		    (vadp_dev->format.pixelformat == V4L2_PIX_FMT_MJPEG)) {
			err = temp_buffer_alloc(vadp_dev);
			if (err < 0) {
				mse_err("cannnot allocate temp buffer\n");
				try_mse_close(vadp_dev);

				return err;
			}
		}
	}

	vadp_dev->sequence = 0;
	vadp_dev->queued = 0;
	vadp_dev->dequeued = 0;
	vadp_dev->prev = NULL;

	mse_debug("END\n");
	return vb2_ioctl_streamon(filp, priv, i);
}

static int mse_adapter_v4l2_g_parm(struct file *filp,
				   void *priv,
				   struct v4l2_streamparm *sp)
{
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_fract *fract;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_OUTPUT(sp->type)) {
		fract = &sp->parm.output.timeperframe;
		sp->parm.output.outputmode = 0;
	} else {
		fract = &sp->parm.capture.timeperframe;
		sp->parm.capture.capturemode = 0;
		sp->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	}

	*fract = vadp_dev->frameintervals;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_s_parm(struct file *filp,
				   void *priv,
				   struct v4l2_streamparm *sp)
{
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	struct v4l2_fract *fract;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_owner_get(vadp_fh))
		return -EBUSY;

	if (V4L2_TYPE_IS_OUTPUT(sp->type))
		fract = &sp->parm.output.timeperframe;
	else
		fract = &sp->parm.capture.timeperframe;

	vadp_dev->frameintervals = *fract;

	mse_debug("END\n");

	return 0;
}

static int mse_adapter_v4l2_enum_framesizes(
					struct file *filp,
					void *priv,
					struct v4l2_frmsizeenum *fsize)
{
	const struct v4l2_adapter_fmt *vadp_fmt = NULL;
	const struct v4l2_adapter_fmt *vadp_fmtbase;
	struct v4l2_adapter_fh *vadp_fh = to_v4l2_adapter_fh(filp);
	struct v4l2_adapter_device *vadp_dev = vadp_fh->dev;
	int i, vadp_fmt_size;

	mse_debug("START fsize->index=%d\n", fsize->index);

	if (!vadp_dev) {
		mse_err("Failed video_drvdata()\n");
		return -EINVAL;
	}

	if (vadp_dev->type == MSE_TYPE_ADAPTER_VIDEO) {
		vadp_fmtbase = g_mse_adapter_v4l2_fmt_sizes_video;
		vadp_fmt_size = ARRAY_SIZE(g_mse_adapter_v4l2_fmt_sizes_video);
	} else if (vadp_dev->type == MSE_TYPE_ADAPTER_MPEG2TS) {
		vadp_fmtbase = g_mse_adapter_v4l2_fmt_sizes_mpeg;
		vadp_fmt_size = ARRAY_SIZE(g_mse_adapter_v4l2_fmt_sizes_mpeg);
	} else {
		mse_err("Failed vdev type=%d\n", vadp_dev->type);
		return -EPERM;
	}

	/* get frame sizes */
	for (i = 0; i < vadp_fmt_size; i++) {
		if (vadp_fmtbase[i].fourcc == fsize->pixel_format) {
			vadp_fmt = &vadp_fmtbase[i];
			break;
		}
	}

	if (fsize->index > 0 || !vadp_fmt) {
		mse_debug("fsize->index(%d)\n", fsize->index);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = vadp_fmt->min_width;
	fsize->stepwise.max_width = vadp_fmt->max_width;
	fsize->stepwise.step_width = vadp_fmt->step_width;
	fsize->stepwise.min_height = vadp_fmt->min_height;
	fsize->stepwise.max_height = vadp_fmt->max_height;
	fsize->stepwise.step_height = vadp_fmt->step_height;

	mse_debug("END\n  format=%c%c%c%c, min_width=%d, min_height=%d\n"
		  "  max_width=%d, max_height=%d\n",
		  vadp_fmt->fourcc >> 0,
		  vadp_fmt->fourcc >> 8,
		  vadp_fmt->fourcc >> 16,
		  vadp_fmt->fourcc >> 24,
		  fsize->stepwise.min_width,
		  fsize->stepwise.min_height,
		  fsize->stepwise.max_width,
		  fsize->stepwise.max_height);

	return 0;
}

static int mse_adapter_v4l2_callback(void *priv, int size);

#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
static int mse_adapter_v4l2_queue_setup(struct vb2_queue *vq,
					unsigned int *nbuffers,
					unsigned int *nplanes,
					unsigned int sizes[],
					struct device *alloc_devs[])
#else
static int mse_adapter_v4l2_queue_setup(struct vb2_queue *vq,
					unsigned int *nbuffers,
					unsigned int *nplanes,
					unsigned int sizes[],
					void *alloc_ctxs[])
#endif
{
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return -EINVAL;
	}

	mse_debug("vq->num_buffers=%d, nbuffers=%d",
		  vq->num_buffers, *nbuffers);
	if (vq->num_buffers + *nbuffers < NUM_BUFFERS)
		*nbuffers = NUM_BUFFERS - vq->num_buffers;

	if (*nplanes && sizes[0] < vadp_dev->format.sizeimage) {
		mse_err("sizeimage too small (%d < %d)\n",
			sizes[0], vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	if (!*nplanes)
		sizes[0] = vadp_dev->format.sizeimage;
	*nplanes = NUM_PLANES;

	mse_debug("END nbuffers=%d sizeimage=%d\n",
		  *nbuffers, vadp_dev->format.sizeimage);

	return 0;
}

static int return_buffers(struct v4l2_adapter_device *vadp_dev,
			  struct list_head *buf_list,
			  enum vb2_buffer_state state)
{
	struct v4l2_adapter_buffer *buf, *tmp;
	int count = 0;

	list_for_each_entry_safe(buf, tmp, buf_list, list) {
		count++;
		vadp_buffer_done(vadp_dev, buf, state);
	}

	return count;
}

static void return_all_buffers(struct v4l2_adapter_device *vadp_dev,
			       enum vb2_buffer_state state)
{
	unsigned long flags;
	int count = 0;

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);
	count = return_buffers(vadp_dev, &vadp_dev->stream_buf_list, state);
	mse_debug("stream_buf_list count=%d\n", count);

	count = return_buffers(vadp_dev, &vadp_dev->prepared_buf_list, state);
	mse_debug("prepared_buf_list count=%d\n", count);

	count = return_buffers(vadp_dev, &vadp_dev->buf_list, state);
	mse_debug("buf_list count=%d\n", count);
	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);
}

static int mse_adapter_v4l2_buf_prepare(struct vb2_buffer *vb)
{
	unsigned long plane_size;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);

	mse_debug("START vb=%p\n", vb2_plane_vaddr(vb, 0));

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return -EINVAL;
	}

	plane_size = vb2_plane_size(&vbuf->vb2_buf, 0);
	if (plane_size < vadp_dev->format.sizeimage) {
		mse_err("buffer too small (%lu < %u)\n",
			plane_size, vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	mse_debug("END\n");

	return 0;
}

static void mse_adapter_v4l2_stop_streaming(struct vb2_queue *vq)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);
	unsigned long flags;
	bool wait_for_buffers;

	mse_debug("START\n");

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return;
	}

	err = mse_stop_streaming(vadp_dev->index_instance);
	if (err < 0) {
		mse_err("Failed mse_stop_streaming()\n");
		return;
	}

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);
	wait_for_buffers = !list_empty(&vadp_dev->stream_buf_list);
	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		return_buffers(vadp_dev, &vadp_dev->prepared_buf_list,
			       VB2_BUF_STATE_QUEUED);
		return_buffers(vadp_dev, &vadp_dev->buf_list,
			       VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	if (wait_for_buffers)
		vb2_wait_for_all_buffers(vq);

	return_all_buffers(vadp_dev, VB2_BUF_STATE_ERROR);
	temp_buffer_free(vadp_dev);

	mse_debug("END\n");
}

static void remove_duplicate_buffer(struct v4l2_adapter_device *vadp_dev,
				    struct v4l2_adapter_buffer *buf)
{
	struct v4l2_adapter_buffer *buf_next;
	void *buf_vaddr, *buf_next_vaddr;
	unsigned long buf_size, buf_next_size;

	if (list_is_singular(&vadp_dev->buf_list))
		buf_next = NULL;
	else
		buf_next = list_next_entry(buf, list);

	if (!buf_next)
		return;

	buf_size = vb2_get_plane_payload(&buf->vb.vb2_buf, 0);
	buf_next_size = vb2_get_plane_payload(&buf_next->vb.vb2_buf, 0);
	if (buf_size != buf_next_size)
		return;
	buf_size = min(buf_size, 4096UL);

	buf_vaddr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	buf_next_vaddr = vb2_plane_vaddr(&buf_next->vb.vb2_buf, 0);
	if (memcmp(buf_vaddr, buf_next_vaddr, buf_size))
		return;

	vadp_buffer_done(vadp_dev, buf_next, VB2_BUF_STATE_DONE);

	mse_info("Removed duplicate second buffer.\n");
	mse_info(" Please set show-preroll-frame=false\n");
}

static void prepare_stream_buffer(struct vb2_queue *vq)
{
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);
	struct v4l2_adapter_buffer *vadp_buf;
	void *buf;
	long size;
	int ret = 0;

	mse_debug("START vq=%p, type=%s\n", vq, v4l2_type_stringfy(vq->type));

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return;
	}

	vadp_buf = list_first_entry_or_null(&vadp_dev->buf_list,
					    struct v4l2_adapter_buffer,
					    list);

	if (!vadp_buf)
		return;

	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		list_move_tail(&vadp_buf->list, &vadp_dev->prepared_buf_list);
	} else {
		buf = vb2_plane_vaddr(&vadp_buf->vb.vb2_buf, 0);
		size = vb2_get_plane_payload(&vadp_buf->vb.vb2_buf, 0);

		/* Check stream type */
		if (vadp_dev->sequence == 0) {
			ret = temp_buffer_check_type(vadp_dev, buf, size);
			if (ret < 0) {
				vadp_buffer_done(vadp_dev, vadp_buf,
						 VB2_BUF_STATE_ERROR);
				vb2_queue_error(vq);
				mse_err("Invalid data format\n");

				return;
			}
		}

		vadp_buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vadp_buf->vb.sequence = vadp_dev->sequence++;
		vadp_buf->vb.field = vadp_dev->format.field;

		/* Workaround: remove duplicated second buffer */
		if (vadp_dev->sequence == 1)
			remove_duplicate_buffer(vadp_dev, vadp_buf);

		if (!vadp_dev->use_temp_buffer) {
			list_move_tail(&vadp_buf->list,
				       &vadp_dev->prepared_buf_list);
		} else {
			/* use temp buffer */
			struct v4l2_adapter_temp_buffer *temp;
			int temp_w = vadp_dev->temp_w;

			ret = temp_buffer_write(vadp_dev, buf, size);
			if (ret)
				return;

			temp = temp_buffer_get_prepared(vadp_dev);
			if (temp && temp_w != vadp_dev->temp_w) {
				list_move_tail(&vadp_buf->list,
					       &vadp_dev->prepared_buf_list);
			} else {
				vadp_buffer_done(vadp_dev, vadp_buf,
						 VB2_BUF_STATE_DONE);
			}
		}
	}
}

static int set_stream_buffer(struct vb2_queue *vq)
{
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);
	struct v4l2_adapter_temp_buffer *temp;
	struct v4l2_adapter_buffer *vadp_buf;
	unsigned char *buf = NULL;
	long size = 0;
	int err;
	unsigned long flags;

	mse_debug("START vq=%p, type=%s\n", vq, v4l2_type_stringfy(vq->type));

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);

	vadp_buf = list_first_entry_or_null(&vadp_dev->prepared_buf_list,
					    struct v4l2_adapter_buffer,
					    list);

	if (!vadp_buf) {
		mse_debug("no need to send anything\n");
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

		return 0;
	}

	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		buf = vb2_plane_vaddr(&vadp_buf->vb.vb2_buf, 0);
		size = vb2_plane_size(&vadp_buf->vb.vb2_buf, 0);
	} else {
		if (vadp_dev->use_temp_buffer) {
			temp = temp_buffer_get_prepared(vadp_dev);
			if (temp) {
				buf = temp->buf;
				size = temp->bytesused;
			}
		} else {
			buf = vb2_plane_vaddr(&vadp_buf->vb.vb2_buf, 0);
			size = vb2_get_plane_payload(&vadp_buf->vb.vb2_buf, 0);
		}
	}

	if (!buf) {
		mse_debug("buf is NULL\n");
		vadp_buffer_done(vadp_dev, vadp_buf, VB2_BUF_STATE_DONE);
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

		return 0;
	}

	if (vadp_dev->prev == buf) {
		mse_debug("current buffer is already transmissioned. prev=%p curr=%p\n",
			  vadp_dev->prev, buf);
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

		return 0;
	}

	mse_debug("buf=%p size=%lu\n", buf, size);

	err = mse_start_transmission(vadp_dev->index_instance,
				     buf,
				     size,
				     vq,
				     mse_adapter_v4l2_callback);

	if (err < 0) {
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);
		if (err == -EAGAIN)
			return err;

		mse_err("Failed mse_start_transmission()\n");
		return_all_buffers(vadp_dev, VB2_BUF_STATE_ERROR);
		vb2_queue_error(vq);

		return err;
	}

	vadp_dev->prev = buf;
	list_move_tail(&vadp_buf->list, &vadp_dev->stream_buf_list);
	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	mse_debug("END\n");

	return err;
}

static int mse_adapter_v4l2_callback(void *priv, int size)
{
	struct vb2_queue *vq = priv;
	int err;
	struct v4l2_adapter_device *vadp_dev;
	unsigned long flags;
	struct v4l2_adapter_buffer *vadp_buf;
	struct v4l2_adapter_temp_buffer *temp;

	mse_debug("START vq=%p, type=%s\n", vq, v4l2_type_stringfy(vq->type));

	if (!vq) {
		mse_err("Private data is NULL\n");
		return -EINVAL;
	}

	vadp_dev = vb2_get_drv_priv(vq);

	if (!vadp_dev) {
		mse_err("vadp_dev is NULL\n");
		return -EINVAL;
	}

	if (size < 0 && size != -EAGAIN) {
		vb2_queue_error(vq);
		return_all_buffers(vadp_dev, VB2_BUF_STATE_ERROR);
		mse_err("priv=%p size=%d", priv, size);

		return size;
	}

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);
	vadp_buf = list_first_entry_or_null(&vadp_dev->stream_buf_list,
					    struct v4l2_adapter_buffer,
					    list);

	if (!vadp_buf) {
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);
		mse_debug("vadp_buf is NULL\n");

		return 0;
	}

	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		vb2_set_plane_payload(&vadp_buf->vb.vb2_buf, 0, size);
		vadp_buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vadp_buf->vb.sequence = vadp_dev->sequence++;
		vadp_buf->vb.field = vadp_dev->format.field;
	} else {
		if (vadp_dev->use_temp_buffer) {
			temp = temp_buffer_get_prepared(vadp_dev);
			if (temp) {
				temp->prepared = false;
				temp->bytesused = 0;
				vadp_dev->temp_r =
					(vadp_dev->temp_r + 1) % NUM_BUFFERS;
			}
		}
	}

	vadp_buffer_done(vadp_dev, vadp_buf, VB2_BUF_STATE_DONE);

	prepare_stream_buffer(vq);

	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	err = set_stream_buffer(vq);

	mse_debug("END err=%d\n", err);

	return err;
}

static void mse_adapter_v4l2_buf_queue(struct vb2_buffer *vb)
{
	unsigned long flags;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_buffer *vadp_buf = to_v4l2_adapter_buffer(vbuf);

	mse_debug("START vb=%p type=%s\n",
		  vb2_plane_vaddr(vb, 0),
		  v4l2_type_stringfy(vb->vb2_queue->type));

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return;
	}

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);


	list_add_tail(&vadp_buf->list, &vadp_dev->buf_list);
	vadp_dev->queued++;

	mse_debug("vb=%p queued=%u dequeued=%u\n",
		  vb2_plane_vaddr(vb, 0),
		  vadp_dev->queued,
		  vadp_dev->dequeued);

	/* start_streaming is not called yet */
	if (!vb2_start_streaming_called(vb->vb2_queue)) {
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);
		mse_debug("start_streaming is not called yet\n");

		return;
	}

	prepare_stream_buffer(vb->vb2_queue);
	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	set_stream_buffer(vb->vb2_queue);
}

static int set_media_config_mpeg(struct v4l2_adapter_device *vadp_dev)
{
	int err;
	struct mse_mpeg2ts_config config;

	err = mse_get_mpeg2ts_config(vadp_dev->index_instance, &config);
	if (err < 0) {
		mse_err("Failed mse_get_mpeg2ts_config()\n");
		return err;
	}

	config.mpeg2ts_type = vadp_dev->mpeg2ts_type;

	err = mse_set_mpeg2ts_config(vadp_dev->index_instance, &config);
	if (err < 0) {
		mse_err("Failed mse_set_mpeg2ts_config()\n");
		return err;
	}

	return 0;
}

static int set_media_config_video(struct v4l2_adapter_device *vadp_dev)
{
	int err;
	struct mse_video_config config;

	err = mse_get_video_config(vadp_dev->index_instance, &config);
	if (err < 0) {
		mse_err("Failed mse_get_video_config()\n");
		return err;
	}

	switch (vadp_dev->format.pixelformat) {
	case V4L2_PIX_FMT_H264:
		config.format = MSE_VIDEO_FORMAT_H264_BYTE_STREAM;
		break;
	case V4L2_PIX_FMT_H264_NO_SC:
		config.format = MSE_VIDEO_FORMAT_H264_AVC;
		break;
	case V4L2_PIX_FMT_MJPEG:
		config.format = MSE_VIDEO_FORMAT_MJPEG;
		break;
	default:
		mse_err("invalid format=%c%c%c%c\n",
			vadp_dev->format.pixelformat >> 0,
			vadp_dev->format.pixelformat >> 8,
			vadp_dev->format.pixelformat >> 16,
			vadp_dev->format.pixelformat >> 24);
		return -EINVAL;
	}

	if (vadp_dev->frameintervals.numerator > 0 &&
	    vadp_dev->frameintervals.denominator > 0) {
		config.fps.denominator = vadp_dev->frameintervals.numerator;
		config.fps.numerator = vadp_dev->frameintervals.denominator;
	}

	err = mse_set_video_config(vadp_dev->index_instance, &config);
	if (err < 0) {
		mse_err("Failed mse_set_video_config()\n");
		return err;
	}

	return 0;
}

static int set_media_config(struct v4l2_adapter_device *vadp_dev)
{
	if (vadp_dev->format.pixelformat == V4L2_PIX_FMT_MPEG)
		return set_media_config_mpeg(vadp_dev);
	else
		return set_media_config_video(vadp_dev);
}

static int mse_adapter_v4l2_start_streaming(struct vb2_queue *vq,
					    unsigned int count)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);
	unsigned long flags;
	int i;

	mse_debug("START vq=%p, type=%s count=%d\n",
		  vq, v4l2_type_stringfy(vq->type), count);

	if (!vadp_dev) {
		mse_err("Failed vb2_get_drv_priv()\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);
	prepare_stream_buffer(vq);
	spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

	err = set_media_config(vadp_dev);
	if (err < 0) {
		mse_err("Fail to set media config\n");
		goto error_cannot_start_streaming;
	}

	err = mse_start_streaming(vadp_dev->index_instance);
	if (err < 0) {
		mse_err("Failed mse_start_streaming()\n");
		goto error_cannot_start_streaming;
	}

	for (i = 0; i < count; i++) {
		spin_lock_irqsave(&vadp_dev->lock_buf_list, flags);
		prepare_stream_buffer(vq);
		spin_unlock_irqrestore(&vadp_dev->lock_buf_list, flags);

		err = set_stream_buffer(vq);

		if (err < 0) {
			if (err == -EAGAIN)
				break;

			goto error_cannot_start_streaming;
		}
	}

	return 0;

error_cannot_start_streaming:
	return_all_buffers(vadp_dev, VB2_BUF_STATE_QUEUED);
	vb2_streamoff(vq, vq->type);
	vb2_queue_error(vq);

	return err;
}

static const struct vb2_ops g_mse_adapter_v4l2_queue_ops = {
	.queue_setup		= mse_adapter_v4l2_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= mse_adapter_v4l2_buf_prepare,
	.start_streaming	= mse_adapter_v4l2_start_streaming,
	.stop_streaming		= mse_adapter_v4l2_stop_streaming,
	.buf_queue		= mse_adapter_v4l2_buf_queue,
};

static const struct v4l2_ioctl_ops g_mse_adapter_v4l2_ioctl_ops = {
	.vidioc_querycap		= mse_adapter_v4l2_querycap,
	.vidioc_enum_fmt_vid_cap	= mse_adapter_v4l2_enum_fmt_vid,
	.vidioc_enum_fmt_vid_out	= mse_adapter_v4l2_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap		= mse_adapter_v4l2_g_fmt_vid,
	.vidioc_g_fmt_vid_out		= mse_adapter_v4l2_g_fmt_vid,
	.vidioc_s_fmt_vid_cap		= mse_adapter_v4l2_s_fmt_vid,
	.vidioc_s_fmt_vid_out		= mse_adapter_v4l2_s_fmt_vid,
	.vidioc_try_fmt_vid_cap		= mse_adapter_v4l2_try_fmt_vid,
	.vidioc_try_fmt_vid_out		= mse_adapter_v4l2_try_fmt_vid,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_streamon		= mse_adapter_v4l2_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_g_parm			= mse_adapter_v4l2_g_parm,
	.vidioc_s_parm			= mse_adapter_v4l2_s_parm,
	.vidioc_enum_framesizes		= mse_adapter_v4l2_enum_framesizes
,
};

static struct v4l2_file_operations g_mse_adapter_v4l2_fops = {
	.owner		= THIS_MODULE,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.open		= mse_adapter_v4l2_fop_open,
	.release	= mse_adapter_v4l2_fop_release,
};

static int register_mse_core(struct v4l2_adapter_device *vadp_dev,
			     enum MSE_TYPE type)
{
	int index_mse;
	struct video_device *vdev = &vadp_dev->vdev;
	char device_name[MSE_NAME_LEN_MAX];

	sprintf(device_name, "/dev/%s", video_device_node_name(vdev));

	index_mse = mse_register_adapter_media(type,
					       vdev->name,
					       device_name);
	if (index_mse < 0)
		return index_mse;

	vadp_dev->index_mse = index_mse;

	return 0;
}

static struct v4l2_adapter_device *g_v4l2_adapter;

static void mse_adapter_v4l2_cleanup(struct v4l2_adapter_device *vadp_dev)
{
	v4l2_device_unregister(&vadp_dev->v4l2_dev);
	video_unregister_device(&vadp_dev->vdev);
}

static int vadp_vb2_queue_init(struct v4l2_adapter_device *vadp_dev,
			       struct vb2_queue *vq,
			       enum v4l2_buf_type type)
{
	vq->type = type;
	vq->io_modes = VB2_MMAP;
	vq->drv_priv = vadp_dev;
	vq->buf_struct_size = sizeof(struct v4l2_adapter_buffer);
	vq->ops = &g_mse_adapter_v4l2_queue_ops;
	vq->mem_ops = &vb2_vmalloc_memops;
	vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vq->lock = &vadp_dev->mutex_vb2;
	vq->min_buffers_needed = 2;

	return vb2_queue_init(vq);
}

static int mse_adapter_v4l2_probe(int dev_num, enum MSE_TYPE type)
{
	int err;
	struct v4l2_adapter_device *vadp_dev;
	struct video_device *vdev;
	struct v4l2_device *v4l2_dev;

	mse_debug("START device number=%d\n", dev_num);

	vadp_dev = &g_v4l2_adapter[dev_num];
	vadp_dev->index_mse = MSE_INDEX_UNDEFINED;
	vadp_dev->type = type;
	vadp_dev->index_instance = MSE_INDEX_UNDEFINED;

	vdev = &vadp_dev->vdev;
	vdev->release = video_device_release_empty;
	vdev->fops = &g_mse_adapter_v4l2_fops;
	vdev->vfl_type = VFL_TYPE_GRABBER;
	vdev->ioctl_ops = &g_mse_adapter_v4l2_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->queue = &vadp_dev->q_cap; /* default queue is capture */

	mutex_init(&vadp_dev->mutex_vb2);

	err = vadp_vb2_queue_init(vadp_dev, &vadp_dev->q_cap,
				  V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (err) {
		mse_err("Failed vb2_queue_init() Rtn=%d\n", err);
		return err;
	}

	err = vadp_vb2_queue_init(vadp_dev, &vadp_dev->q_out,
				  V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (err) {
		mse_err("Failed vb2_queue_init() Rtn=%d\n", err);
		return err;
	}

	INIT_LIST_HEAD(&vadp_dev->buf_list);
	INIT_LIST_HEAD(&vadp_dev->prepared_buf_list);
	INIT_LIST_HEAD(&vadp_dev->stream_buf_list);
	spin_lock_init(&vadp_dev->lock_buf_list);

	vdev->lock = &vadp_dev->mutex_vb2;

	video_set_drvdata(vdev, vadp_dev);

	v4l2_dev = &vadp_dev->v4l2_dev;
	snprintf(v4l2_dev->name, sizeof(v4l2_dev->name), "%s",
		 MSE_ADAPTER_V4L2_NAME_BASE);
	err = v4l2_device_register(NULL, v4l2_dev);
	if (err) {
		mse_err("Failed v4l2_device_register() Rtn=%d\n", err);
		return -EPERM;
	}

	vdev->v4l2_dev = v4l2_dev;
	err = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (err) {
		mse_err("Failed video_register_device() Rtn=%d\n", err);
		v4l2_device_unregister(&vadp_dev->v4l2_dev);
		return -EPERM;
	}

	mse_debug("video device was registered as (%s)",
		  video_device_node_name(vdev));

	err = register_mse_core(vadp_dev, type);
	if (err < 0) {
		mse_err("Failed register_mse_core() Rtn=%d\n", err);
		mse_adapter_v4l2_cleanup(vadp_dev);
		return err;
	}

	snprintf(vdev->name, sizeof(vdev->name), "%s.v4l2.%s.mse%d",
		 v4l2_dev->name, convert_type_to_str(type),
		 vadp_dev->index_mse);

	return 0;
}

static void unregister_mse_core(struct v4l2_adapter_device *vadp_dev)
{
	int err;
	int index = vadp_dev->index_mse;

	if (index == MSE_INDEX_UNDEFINED) {
		mse_info("already unregistered(%d)\n", index);
		return;
	}

	err = mse_unregister_adapter_media(index);
	if (err < 0)
		mse_err("Failed mse_unregister_adapter_media()\n");
}

static int mse_adapter_v4l2_free(int dev_num)
{
	mse_debug("START device number=%d\n", dev_num);

	unregister_mse_core(&g_v4l2_adapter[dev_num]);
	mse_adapter_v4l2_cleanup(&g_v4l2_adapter[dev_num]);

	mse_debug("END\n");

	return 0;
}

static int __init mse_adapter_v4l2_init(void)
{
	int err, i, j, type;

	mse_debug("Start v4l2 adapter\n");

	if (v4l2_video_devices < 0) {
		mse_err("Invalid devices video=%d\n", v4l2_video_devices);
		return -EINVAL;
	}

	if (v4l2_mpeg2ts_devices < 0) {
		mse_err("Invalid devices mpeg2ts=%d\n", v4l2_mpeg2ts_devices);
		return -EINVAL;
	}

	v4l2_devices = v4l2_video_devices + v4l2_mpeg2ts_devices;
	if (v4l2_devices > MSE_ADAPTER_V4L2_DEVICE_MAX) {
		mse_err("Too many devices, %d (video=%d mpeg2ts=%d)\n",
			v4l2_devices, v4l2_video_devices, v4l2_mpeg2ts_devices);
		return -EINVAL;
	} else if (v4l2_devices <= 0) {
		mse_err("Invalid devices, %d (video=%d mpeg2ts=%d)\n",
			v4l2_devices, v4l2_video_devices, v4l2_mpeg2ts_devices);
		return -EINVAL;
	} else {
		;
	}

	g_v4l2_adapter = kcalloc(v4l2_devices, sizeof(*g_v4l2_adapter),
				 GFP_KERNEL);
	if (!g_v4l2_adapter)
		return -ENOMEM;

	for (i = 0; i < v4l2_devices; i++) {
		if (i < v4l2_video_devices)
			type = MSE_TYPE_ADAPTER_VIDEO;
		else
			type = MSE_TYPE_ADAPTER_MPEG2TS;

		err = mse_adapter_v4l2_probe(i, type);
		if (err) {
			mse_err("Failed creating device=%d Rtn=%d\n", i, err);
			goto init_fail;
		}
	}

	return 0;

init_fail:
	for (j = 0; j < i; j++)
		mse_adapter_v4l2_free(j);

	kfree(g_v4l2_adapter);

	return err;
}

/* module clean up */
static void __exit mse_adapter_v4l2_exit(void)
{
	int i;

	mse_debug("Stop v4l2 adapter\n");

	for (i = 0; i < v4l2_devices; i++)
		mse_adapter_v4l2_free(i);

	kfree(g_v4l2_adapter);
}

module_init(mse_adapter_v4l2_init)
module_exit(mse_adapter_v4l2_exit)

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
