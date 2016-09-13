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

#include "mse_core.h"

/****************/
/* Return value */
/****************/
#define MSE_ADAPTER_V4L2_RTN_OK				0
#define MSE_ADAPTER_V4L2_RTN_NG				-1

/******************/
/* Maximum number */
/******************/
#define MSE_ADAPTER_V4L2_CAPTURE_DEVICE_MAX		1
#define MSE_ADAPTER_V4L2_PLAYBACK_DEVICE_MAX		1
#define MSE_ADAPTER_V4L2_DEVICE_MAX  (MSE_ADAPTER_V4L2_CAPTURE_DEVICE_MAX + \
					MSE_ADAPTER_V4L2_PLAYBACK_DEVICE_MAX)

/*********************/
/* Maximum I/O index */
/*********************/
#define MSE_ADAPTER_V4L2_OUTPUT_DEVICE_MAX_INDEX	0
#define MSE_ADAPTER_V4L2_INPUT_DEVICE_MAX_INDEX		0
#define MSE_ADAPTER_V4L2_OUTPUT_FORMAT_MAX_INDEX	0
#define MSE_ADAPTER_V4L2_INPUT_FORMAT_MAX_INDEX		0

#define NUM_BUFFERS 2
#define NUM_PLANES  1

static struct mutex g_lock;

/*************/
/* Structure */
/*************/
/* Buffer information */
struct v4l2_adapter_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

/* Device information */
struct v4l2_adapter_device {
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	/* mutex lock */
	struct mutex		lock;
	struct v4l2_pix_format	format;
	struct v4l2_fract	frameintervals;
	unsigned int		input;
	struct vb2_queue	queue;
	/* spin lock */
	spinlock_t		qlock;
	struct list_head	buf_list;
	unsigned int		sequence;
	unsigned int		output;
	/* index for register */
	int			index_mse;
	/* index for MSE instance */
	int			index_instance;
};

/* Init information */
struct v4l2_adapter_init_info {
	const struct v4l2_ioctl_ops	*ioctl_ops;
	int				vfl_dir;
	unsigned int			type;
	const struct vb2_ops		*queue_ops;
	const char			*adapter_name;
};

/* Format information */
struct v4l2_adapter_fmt {
	u32			fourcc;
	unsigned int		width;
	unsigned int		height;
};

static const struct v4l2_fmtdesc g_playback_formats[] = {
	{
		.description = "H264 with start codes",
		.pixelformat = V4L2_PIX_FMT_H264,
	},
	{
		.description = "MPEG-1/2/4 Multiplexed",
		.pixelformat = V4L2_PIX_FMT_MPEG,
	},
	{
		.description = "Motion-JPEG",
		.pixelformat = V4L2_PIX_FMT_MJPEG,
	},
};

static const struct v4l2_fmtdesc g_capture_formats[] = {
	{
		.description = "H264 with start codes",
		.pixelformat = V4L2_PIX_FMT_H264,
	},
	{
		.description = "MPEG-1/2/4 Multiplexed",
		.pixelformat = V4L2_PIX_FMT_MPEG,
	},
	{
		.description = "Motion-JPEG",
		.pixelformat = V4L2_PIX_FMT_MJPEG,
	},
};

/* Playback, capture Common format sizes */
static const struct v4l2_adapter_fmt g_mse_adapter_v4l2_common_fmt_sizes[] = {
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.width	= 1920,
		.height	= 1080,
	},
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.width	= 1280,
		.height	= 960,
	},
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.width	= 1280,
		.height	= 720,
	},
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.width	= 640,
		.height	= 480,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG,
		.width	= 1920,
		.height	= 1080,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG,
		.width	= 1280,
		.height	= 960,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG,
		.width	= 1280,
		.height	= 720,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MPEG,
		.width	= 640,
		.height	= 480,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MJPEG,
		.width	= 1920,
		.height	= 1080,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MJPEG,
		.width	= 1280,
		.height	= 960,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MJPEG,
		.width	= 1280,
		.height	= 720,
	},
	{
		.fourcc	= V4L2_PIX_FMT_MJPEG,
		.width	= 640,
		.height	= 480,
	},
};

static const struct v4l2_fract g_mse_adapter_v4l2_common_frame_intervals[] = {
	{
		.numerator = 1,
		.denominator = 15,
	},
	{
		.numerator = 1,
		.denominator = 30,
	},
	{
		.numerator = 1,
		.denominator = 60,
	},

};

/************/
/* Function */
/************/
static inline struct v4l2_adapter_buffer *to_v4l2_adapter_buffer(
						struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct v4l2_adapter_buffer, vb);
}

static int try_mse_open(struct v4l2_adapter_device *vadp_dev)
{
	enum MSE_DIRECTION adp_direction;
	int index;

	/* probe is not finish yet */
	if (vadp_dev->index_mse == MSE_INDEX_UNDEFINED) {
		pr_info("[%s]probe is not finish yet\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_OK;
	}

	if (vadp_dev->vdev.vfl_dir == VFL_DIR_TX)
		adp_direction = MSE_DIRECTION_INPUT;
	else
		adp_direction = MSE_DIRECTION_OUTPUT;

	index = mse_open(vadp_dev->index_mse, adp_direction);
	if (index < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	vadp_dev->index_instance = index;

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int try_mse_close(struct v4l2_adapter_device *vadp_dev)
{
	int err;

	/* probe is not finish yet */
	if (vadp_dev->index_mse == MSE_INDEX_UNDEFINED) {
		pr_info("[%s]probe is not finish yet\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_OK;
	}

	if (vadp_dev->index_instance == MSE_INDEX_UNDEFINED) {
		pr_info("[%s]mse_start is not finish yet\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_OK;
	}

	err = mse_close(vadp_dev->index_instance);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_fop_open(struct file *filp)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	mutex_lock(&g_lock);
	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = v4l2_fh_open(filp);
	if (err) {
		pr_err("[%s]Failed v4l2_fh_open()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = try_mse_open(vadp_dev);
	if (err) {
		pr_err("[%s]Failed mse_open()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]END\n", __func__);

	mutex_unlock(&g_lock);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_fop_release(struct file *filp)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	mutex_lock(&g_lock);
	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = try_mse_close(vadp_dev);
	if (err) {
		pr_err("[%s]Failed mse_close()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = v4l2_fh_release(filp);
	if (err) {
		pr_err("[%s]Failed v4l2_fh_release()\n", __func__);
		mutex_unlock(&g_lock);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]END\n", __func__);

	mutex_unlock(&g_lock);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_querycap(struct file *filp,
					      void *priv,
					      struct v4l2_capability *vcap)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	strlcpy(vcap->driver, "renesas-mse", sizeof(vcap->driver));
	strlcpy(vcap->card, vadp_dev->vdev.name, sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:%s",
		 vadp_dev->v4l2_dev.name);
	vcap->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_enum_output(struct file *filp,
						 void *priv,
						 struct v4l2_output *vout)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START vout->index=%d\n", __func__, vout->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (vout->index > MSE_ADAPTER_V4L2_OUTPUT_DEVICE_MAX_INDEX) {
		pr_info("[%s]vout->index(%d) is bigger than %d\n",
			__func__, vout->index,
			MSE_ADAPTER_V4L2_OUTPUT_DEVICE_MAX_INDEX);
		return -EINVAL;
	}

	snprintf(vout->name, sizeof(vout->name), "Renesas VideoOut %d",
		 vout->index);
	vout->type = V4L2_OUTPUT_TYPE_ANALOG;
	vout->std = V4L2_STD_ALL;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_g_output(struct file *filp,
					      void *priv,
					      unsigned int *outp)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	*outp = vadp_dev->output;

	pr_debug("[%s]END *outp=%d\n", __func__, *outp);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_s_output(struct file *filp,
					      void *priv,
					      unsigned int outp)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START outp=%d\n", __func__, outp);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (outp > MSE_ADAPTER_V4L2_OUTPUT_DEVICE_MAX_INDEX) {
		pr_err("[%s]outp(%d) is bigger than %d\n", __func__,
		       outp, MSE_ADAPTER_V4L2_OUTPUT_DEVICE_MAX_INDEX);
		return -EINVAL;
	}

	vadp_dev->output = outp;

	pr_debug("[%s]END vadp_dev->output=%d\n", __func__, vadp_dev->output);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static const struct v4l2_fmtdesc *get_default_fmtdesc(
					const struct v4l2_fmtdesc fmtdescs[],
					int dflt_format)
{
	return &fmtdescs[dflt_format];
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

static const struct v4l2_adapter_fmt *get_fmt_sizes(
				const struct v4l2_adapter_fmt *dflt_fmts,
				int arr_size,
				struct v4l2_format *fmt)
{
	int i;
	int last_one;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (i = 0; i < arr_size; i++) {
		if (pix->pixelformat != dflt_fmts[i].fourcc)
			continue;

		if (pix->width >= dflt_fmts[i].width &&
		    pix->height >= dflt_fmts[i].height) {
			return &dflt_fmts[i];
		}
	}

	last_one = arr_size - 1;
	return &dflt_fmts[last_one];
}

static int set_config_format(int index_instance, struct v4l2_pix_format *pix)
{
	int err;
	struct mse_video_config config;

	err = mse_get_video_config(index_instance, &config);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_get_video_config()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	config.height = pix->height;
	config.width = pix->width;
	if (pix->field == V4L2_FIELD_INTERLACED)
		config.interlaced = 1;
	else
		config.interlaced = 0;
	config.bytes_per_frame = pix->sizeimage;

	err = mse_set_video_config(index_instance, &config);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_set_video_config()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_enum_fmt_vid_out(struct file *filp,
						      void *priv,
						      struct v4l2_fmtdesc *fmt)
{
	unsigned int index;
	const struct v4l2_fmtdesc *fmtdesc;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START fmt->index=%d\n", __func__, fmt->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (fmt->index >= ARRAY_SIZE(g_playback_formats)) {
		pr_info("[%s]fmt->index(%d) is equal or bigger than %d\n",
			__func__, fmt->index,
			(int)ARRAY_SIZE(g_playback_formats));
		return -EINVAL;
	}

	index = fmt->index;
	memset(fmt, 0, sizeof(*fmt));

	fmtdesc = &g_playback_formats[index];

	fmt->index = index;
	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	strcpy(fmt->description, fmtdesc->description);
	fmt->pixelformat = fmtdesc->pixelformat;

	pr_debug("[%s]END format: %s\n", __func__, fmtdesc->description);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_g_fmt_vid_out(struct file *filp,
						   void *priv,
						   struct v4l2_format *fmt)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pix->width = vadp_dev->format.width;
	pix->height = vadp_dev->format.height;
	pix->pixelformat = vadp_dev->format.pixelformat;
	pix->bytesperline = vadp_dev->format.bytesperline;
	pix->sizeimage = vadp_dev->format.sizeimage;
	pix->field = vadp_dev->format.field;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_try_fmt_vid_out(struct file *filp,
						     void *priv,
						     struct v4l2_format *fmt)
{
	const struct v4l2_fmtdesc *fmtdesc;
	const struct v4l2_adapter_fmt *vadp_fmt;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	fmtdesc = get_fmtdesc(g_playback_formats,
			      ARRAY_SIZE(g_playback_formats),
			      fmt);
	if (!fmtdesc) {
		pr_info("[%s]Unknown fourcc format=(0x%08x)\n",
			__func__, pix->pixelformat);
		fmtdesc = get_default_fmtdesc(g_playback_formats, 0);
		pix->pixelformat = fmtdesc->pixelformat;
	}

	if (pix->field != V4L2_FIELD_NONE &&
	    pix->field != V4L2_FIELD_INTERLACED)
		pix->field = V4L2_FIELD_NONE;

	vadp_fmt = get_fmt_sizes(
			g_mse_adapter_v4l2_common_fmt_sizes,
			ARRAY_SIZE(g_mse_adapter_v4l2_common_fmt_sizes),
			fmt);
	pix->width = vadp_fmt->width;
	pix->height = vadp_fmt->height;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_s_fmt_vid_out(struct file *filp,
						   void *priv,
						   struct v4l2_format *fmt)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		pr_err("[%s]Failed wrong buffer type\n", __func__);
		return -EINVAL;
	}

	err = mse_adapter_v4l2_playback_try_fmt_vid_out(filp, priv, fmt);
	if (err) {
		pr_err("[%s]Failed playback_try_fmt_vid_out()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (vb2_is_busy(&vadp_dev->queue)) {
		pr_err("[%s]Failed vb2 is busy\n", __func__);
		return -EBUSY;
	}

	vadp_dev->format.width = pix->width;
	vadp_dev->format.height = pix->height;
	vadp_dev->format.pixelformat = pix->pixelformat;
	vadp_dev->format.bytesperline = pix->bytesperline;
	vadp_dev->format.sizeimage = pix->sizeimage;
	vadp_dev->format.field = pix->field;

	err = set_config_format(vadp_dev->index_instance, pix);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_queue_setup(struct vb2_queue *vq,
						 unsigned int *nbuffers,
						 unsigned int *nplanes,
						 unsigned int sizes[],
						 void *alloc_ctxs[])
{
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]vq->num_buffers=%d, nbuffers=%d",
		 __func__, vq->num_buffers, *nbuffers);
	if (vq->num_buffers + *nbuffers < NUM_BUFFERS)
		*nbuffers = NUM_BUFFERS - vq->num_buffers;

	if (*nplanes && sizes[0] < vadp_dev->format.sizeimage) {
		pr_err("[%s]sizeimage too small (%d < %d)\n",
		       __func__, sizes[0], vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	if (!*nplanes)
		sizes[0] = vadp_dev->format.sizeimage;
	*nplanes = NUM_PLANES;

	pr_debug("[%s]END nbuffers=%d\n", __func__, *nbuffers);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_buf_prepare(struct vb2_buffer *vb)
{
	unsigned long plane_size;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);

	pr_debug("[%s]START vb=%p\n", __func__, vb2_plane_vaddr(vb, 0));

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	plane_size = vb2_plane_size(&vbuf->vb2_buf, 0);
	if (plane_size < vadp_dev->format.sizeimage) {
		pr_err("[%s]buffer too small (%lu < %u)\n",
		       __func__, plane_size, vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	vbuf->vb2_buf.planes[0].bytesused =
				vb2_get_plane_payload(&vbuf->vb2_buf, 0);

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_callback(int index, int size);

static int playback_send_first_buffer(struct v4l2_adapter_device *vadp_dev)
{
	struct v4l2_adapter_buffer *new_buf = NULL;
	void *buf_to_send;
	unsigned long flags;
	long new_buf_size;
	int err;

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (!list_empty(&vadp_dev->buf_list)) {
		new_buf = list_first_entry(&vadp_dev->buf_list,
					   struct v4l2_adapter_buffer,
					   list);
	}
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	if (!new_buf) {
		pr_debug("[%s]new_buf is NULL\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_OK;
	}

	buf_to_send = vb2_plane_vaddr(&new_buf->vb.vb2_buf, 0);
	pr_debug("[%s]buf_to_send=%p\n", __func__, buf_to_send);

	new_buf_size = vb2_get_plane_payload(&new_buf->vb.vb2_buf, 0);
	new_buf->vb.vb2_buf.timestamp = ktime_get_ns();
	new_buf->vb.sequence = vadp_dev->sequence++;
	new_buf->vb.field = vadp_dev->format.field;

	err = mse_start_transmission(vadp_dev->index_instance,
				     buf_to_send,
				     new_buf_size,
				     mse_adapter_v4l2_playback_callback);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_start_transmission()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static void mse_adapter_v4l2_playback_buf_queue(struct vb2_buffer *vb)
{
	unsigned long flags;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_buffer *buf = to_v4l2_adapter_buffer(vbuf);
	int is_need_send = 0;

	pr_debug("[%s]START vb=%p\n", __func__, vb2_plane_vaddr(vb, 0));

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return;
	}

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (list_empty(&vadp_dev->buf_list))
		is_need_send = 1;
	list_add_tail(&buf->list, &vadp_dev->buf_list);
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	/* start_streaming is not called yet */
	if (!vb2_start_streaming_called(&vadp_dev->queue)) {
		pr_debug("[%s]start_streaming is not called yet\n", __func__);
		return;
	}
	/* no need to send anything */
	if (!is_need_send) {
		pr_debug("[%s]no need to send anything\n", __func__);
		return;
	}
	playback_send_first_buffer(vadp_dev);

	pr_debug("[%s]END\n", __func__);
}

static void return_all_buffers(struct v4l2_adapter_device *vadp_dev,
			       enum vb2_buffer_state state)
{
	unsigned long flags;
	struct v4l2_adapter_buffer *buf, *node;

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	list_for_each_entry_safe(buf, node, &vadp_dev->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);
}

static int mse_adapter_v4l2_playback_callback(int index, int size)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = NULL;
	unsigned long flags;
	struct v4l2_adapter_buffer *buf = NULL;

	err = mse_get_private_data(index, (void **)&vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK || !vadp_dev) {
		pr_err("[%s]Failed mse_get_private_data()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]START\n", __func__);

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (!list_empty(&vadp_dev->buf_list)) {
		buf = list_first_entry(&vadp_dev->buf_list,
				       struct v4l2_adapter_buffer,
				       list);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	if (!buf) {
		pr_err("[%s]buf is NULL\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	err = playback_send_first_buffer(vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int playback_start_streaming(struct v4l2_adapter_device *vadp_dev,
				    unsigned int count)
{
	int err;
	int index = vadp_dev->index_instance;

	vadp_dev->sequence = 0;

	err = mse_start_streaming(index);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_start_streaming()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = playback_send_first_buffer(vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_start_streaming(struct vb2_queue *vq,
						     unsigned int count)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START count=%d\n", __func__, count);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = playback_start_streaming(vadp_dev, count);
	if (err) {
		pr_err("[%s]Failed start streaming\n", __func__);
		return_all_buffers(vadp_dev, VB2_BUF_STATE_QUEUED);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static void mse_adapter_v4l2_playback_stop_streaming(struct vb2_queue *vq)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return;
	}

	return_all_buffers(vadp_dev, VB2_BUF_STATE_ERROR);

	err = mse_stop_streaming(vadp_dev->index_instance);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_stop_streaming()\n", __func__);
		return;
	}

	pr_debug("[%s]END\n", __func__);
}

static int mse_adapter_v4l2_capture_querycap(struct file *filp,
					     void *priv,
					     struct v4l2_capability *vcap)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	strlcpy(vcap->driver, "renesas-mse", sizeof(vcap->driver));
	strlcpy(vcap->card, vadp_dev->vdev.name, sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:%s",
		 vadp_dev->v4l2_dev.name);
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_enum_input(struct file *filp,
					       void *priv,
					       struct v4l2_input *vin)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START vin->index=%d\n", __func__, vin->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (vin->index > MSE_ADAPTER_V4L2_INPUT_DEVICE_MAX_INDEX) {
		pr_info("[%s]vin->index(%d) is bigger than %d\n",
			__func__, vin->index,
			MSE_ADAPTER_V4L2_INPUT_DEVICE_MAX_INDEX);
		return -EINVAL;
	}

	snprintf(vin->name, sizeof(vin->name), "Renesas Camera %d",
		 vin->index);
	vin->type = V4L2_INPUT_TYPE_CAMERA;
	vin->std = V4L2_STD_ALL;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_g_input(struct file *filp,
					    void *priv,
					    unsigned int *inp)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	*inp = vadp_dev->input;

	pr_debug("[%s]END *inp=%d\n", __func__, *inp);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_s_input(struct file *filp,
					    void *priv,
					    unsigned int input)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START input=%d\n", __func__, input);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (input > MSE_ADAPTER_V4L2_INPUT_DEVICE_MAX_INDEX) {
		pr_err("[%s]input(%d) is bigger than %d\n", __func__, input,
		       MSE_ADAPTER_V4L2_INPUT_DEVICE_MAX_INDEX);
		return -EINVAL;
	}

	vadp_dev->input = input;
	pr_debug("[%s]END vadp_dev->input=%d\n", __func__, vadp_dev->input);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_enum_fmt_vid_cap(
						struct file *filp,
						void *priv,
						struct v4l2_fmtdesc *fmt)
{
	unsigned int index;
	const struct v4l2_fmtdesc *fmtdesc;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START fmt->index=%d\n", __func__, fmt->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (fmt->index >= ARRAY_SIZE(g_capture_formats)) {
		pr_info("[%s]fmt->index(%d) is equal or bigger than %d\n",
			__func__, fmt->index,
			(int)ARRAY_SIZE(g_capture_formats));
		return -EINVAL;
	}

	index = fmt->index;
	memset(fmt, 0, sizeof(*fmt));

	fmtdesc = &g_capture_formats[index];

	fmt->index = index;
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strcpy(fmt->description, fmtdesc->description);
	fmt->pixelformat = fmtdesc->pixelformat;

	pr_debug("[%s]END format: %s\n", __func__, fmtdesc->description);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_g_fmt_vid_cap(struct file *filp,
						  void *priv,
						  struct v4l2_format *fmt)
{
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pix->width = vadp_dev->format.width;
	pix->height = vadp_dev->format.height;
	pix->pixelformat = vadp_dev->format.pixelformat;
	pix->bytesperline = vadp_dev->format.bytesperline;
	pix->sizeimage = vadp_dev->format.sizeimage;
	pix->field = vadp_dev->format.field;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_try_fmt_vid_cap(struct file *filp,
						    void *priv,
						    struct v4l2_format *fmt)
{
	const struct v4l2_fmtdesc *fmtdesc;
	const struct v4l2_adapter_fmt *vadp_fmt;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	fmtdesc = get_fmtdesc(g_capture_formats,
			      ARRAY_SIZE(g_capture_formats),
			      fmt);
	if (!fmtdesc) {
		pr_info("[%s]Unknown fourcc format=(0x%08x)\n",
			__func__, pix->pixelformat);
		fmtdesc = get_default_fmtdesc(g_capture_formats, 0);
		pix->pixelformat = fmtdesc->pixelformat;
	}

	if (pix->field != V4L2_FIELD_NONE &&
	    pix->field != V4L2_FIELD_INTERLACED)
		pix->field = V4L2_FIELD_NONE;

	vadp_fmt = get_fmt_sizes(
			g_mse_adapter_v4l2_common_fmt_sizes,
			ARRAY_SIZE(g_mse_adapter_v4l2_common_fmt_sizes),
			fmt);
	pix->width = vadp_fmt->width;
	pix->height = vadp_fmt->height;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_s_fmt_vid_cap(struct file *filp,
						  void *priv,
						  struct v4l2_format *fmt)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("[%s]Failed wrong buffer type\n", __func__);
		return -EINVAL;
	}

	err = mse_adapter_v4l2_capture_try_fmt_vid_cap(filp, priv, fmt);
	if (err) {
		pr_err("[%s]Failed capture_try_fmt_vid_cap()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (vb2_is_busy(&vadp_dev->queue)) {
		pr_err("[%s]Failed vb2 is busy\n", __func__);
		return -EBUSY;
	}

	vadp_dev->format.width = pix->width;
	vadp_dev->format.height = pix->height;
	vadp_dev->format.pixelformat = pix->pixelformat;
	vadp_dev->format.bytesperline = pix->bytesperline;
	vadp_dev->format.sizeimage = pix->sizeimage;
	vadp_dev->format.field = pix->field;

	pr_info("[%s]END format=%c%c%c%c, width=%d, height=%d\n",
		__func__,
		pix->pixelformat >> 0,
		pix->pixelformat >> 8,
		pix->pixelformat >> 16,
		pix->pixelformat >> 24,
		pix->width,
		pix->height);

	err = set_config_format(vadp_dev->index_instance, pix);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_queue_setup(struct vb2_queue *vq,
						unsigned int *nbuffers,
						unsigned int *nplanes,
						unsigned int sizes[],
						void *alloc_ctxs[])
{
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]vq->num_buffers=%d, nbuffers=%d",
		 __func__, vq->num_buffers, *nbuffers);
	if (vq->num_buffers + *nbuffers < NUM_BUFFERS)
		*nbuffers = NUM_BUFFERS - vq->num_buffers;

	if (*nplanes && sizes[0] < vadp_dev->format.sizeimage) {
		pr_err("[%s]sizeimage too small (%d < %d)\n",
		       __func__, sizes[0], vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	if (!*nplanes)
		sizes[0] = vadp_dev->format.sizeimage;
	*nplanes = NUM_PLANES;

	pr_debug("[%s]END nbuffers=%d\n", __func__, *nbuffers);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_buf_prepare(struct vb2_buffer *vb)
{
	unsigned long plane_size;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);

	pr_debug("[%s]START vb=%p\n", __func__, vb2_plane_vaddr(vb, 0));

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	plane_size = vb2_plane_size(&vbuf->vb2_buf, 0);
	if (plane_size < vadp_dev->format.sizeimage) {
		pr_err("[%s]buffer too small (%lu < %u)\n",
		       __func__, plane_size, vadp_dev->format.sizeimage);
		return -EINVAL;
	}

	vbuf->vb2_buf.planes[0].bytesused =
				vb2_get_plane_payload(&vbuf->vb2_buf, 0);

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_callback(int index, int size);

static int capture_send_first_buffer(struct v4l2_adapter_device *vadp_dev)
{
	struct v4l2_adapter_buffer *new_buf = NULL;
	void *buf_to_send;
	unsigned long flags;
	long new_buf_size;
	int err;

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (!list_empty(&vadp_dev->buf_list))
		new_buf = list_first_entry(&vadp_dev->buf_list,
					   struct v4l2_adapter_buffer,
					   list);
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	if (!new_buf) {
		pr_debug("[%s]new_buf is NULL\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_OK;
	}

	buf_to_send = vb2_plane_vaddr(&new_buf->vb.vb2_buf, 0);
	pr_debug("[%s]buf_to_send=%p\n", __func__, buf_to_send);
	new_buf_size = vb2_plane_size(&new_buf->vb.vb2_buf, 0);

	err = mse_start_transmission(vadp_dev->index_instance,
				     buf_to_send,
				     new_buf_size,
				     mse_adapter_v4l2_capture_callback);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_start_transmission()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static void mse_adapter_v4l2_capture_buf_queue(struct vb2_buffer *vb)
{
	unsigned long flags;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_adapter_buffer *buf = to_v4l2_adapter_buffer(vbuf);
	int is_need_send = 0;

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return;
	}

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (list_empty(&vadp_dev->buf_list))
		is_need_send = 1;
	list_add_tail(&buf->list, &vadp_dev->buf_list);
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	/* start_streaming is not called yet */
	if (!vb2_start_streaming_called(&vadp_dev->queue)) {
		pr_debug("[%s]start_streaming is not called yet\n", __func__);
		return;
	}
	/* no need to send anything */
	if (!is_need_send) {
		pr_debug("[%s]no need to send anything\n", __func__);
		return;
	}
	capture_send_first_buffer(vadp_dev);

	pr_debug("[%s]END\n", __func__);
}

static int mse_adapter_v4l2_capture_callback(int index, int size)
{
	int err;
	unsigned long flags;
	struct v4l2_adapter_buffer *buf = NULL;
	struct v4l2_adapter_device *vadp_dev = NULL;
	enum vb2_buffer_state buf_state = VB2_BUF_STATE_DONE;

	err = mse_get_private_data(index, (void **)&vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK || !vadp_dev) {
		pr_err("[%s]Failed mse_get_private_data()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]START size=%d\n", __func__, size);

	spin_lock_irqsave(&vadp_dev->qlock, flags);
	if (!list_empty(&vadp_dev->buf_list)) {
		buf = list_first_entry(&vadp_dev->buf_list,
				       struct v4l2_adapter_buffer,
				       list);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&vadp_dev->qlock, flags);

	if (!buf) {
		pr_debug("[%s]buf is NULL\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	if (size == 0)
		buf_state = VB2_BUF_STATE_ERROR;

	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, size);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = vadp_dev->sequence++;
	buf->vb.field = vadp_dev->format.field;
	vb2_buffer_done(&buf->vb.vb2_buf, buf_state);

	err = capture_send_first_buffer(vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);
	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int capture_start_streaming(struct v4l2_adapter_device *vadp_dev,
				   unsigned int count)
{
	int err;
	int index = vadp_dev->index_instance;

	vadp_dev->sequence = 0;

	err = mse_start_streaming(index);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_start_streaming()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = capture_send_first_buffer(vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_start_streaming(struct vb2_queue *vq,
						    unsigned int count)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START count=%d\n", __func__, count);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	err = capture_start_streaming(vadp_dev, count);
	if (err) {
		pr_err("[%s]Failed start streaming\n", __func__);
		return_all_buffers(vadp_dev, VB2_BUF_STATE_QUEUED);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static void mse_adapter_v4l2_capture_stop_streaming(struct vb2_queue *vq)
{
	int err;
	struct v4l2_adapter_device *vadp_dev = vb2_get_drv_priv(vq);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed vb2_get_drv_priv()\n", __func__);
		return;
	}

	return_all_buffers(vadp_dev, VB2_BUF_STATE_ERROR);

	err = mse_stop_streaming(vadp_dev->index_instance);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_stop_streaming()\n", __func__);
		return;
	}

	pr_debug("[%s]END\n", __func__);
}

static int mse_adapter_v4l2_playback_g_parm(struct file *filp,
					    void *priv,
					    struct v4l2_streamparm *sp)
{
	struct v4l2_fract *fract = &sp->parm.output.timeperframe;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	sp->parm.output.outputmode = 0;
	/* sp->parm.capture.capability = V4L2_CAP_TIMEPERFRAME; */
	fract->denominator = vadp_dev->frameintervals.denominator;
	fract->numerator = vadp_dev->frameintervals.numerator;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int set_config_timerperframe(int index_instance,
				    struct v4l2_fract *fract)
{
	int err;
	struct mse_video_config config;

	err = mse_get_video_config(index_instance, &config);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_get_video_config()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	config.fps.n = fract->numerator;
	config.fps.m = fract->denominator;

	err = mse_set_video_config(index_instance, &config);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed mse_set_video_config()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_s_parm(struct file *filp,
					    void *priv,
					    struct v4l2_streamparm *sp)
{
	int err;
	struct v4l2_fract *fract = &sp->parm.output.timeperframe;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	vadp_dev->frameintervals.denominator = fract->denominator;
	vadp_dev->frameintervals.numerator = fract->numerator;

	err = set_config_timerperframe(vadp_dev->index_instance, fract);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_enum_framesizes(
					struct file *filp,
					void *priv,
					struct v4l2_frmsizeenum *fsize)
{
	int total_frame_sizes = 0;
	const struct v4l2_adapter_fmt *vadp_fmt = NULL;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	int i;

	pr_debug("[%s]START fsize->index=%d\n", __func__, fsize->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	/* get total frame sizes */
	for (i = 0; i < ARRAY_SIZE(g_mse_adapter_v4l2_common_fmt_sizes); i++) {
		if (g_mse_adapter_v4l2_common_fmt_sizes[i].fourcc ==
						fsize->pixel_format) {
			vadp_fmt = &g_mse_adapter_v4l2_common_fmt_sizes[i];
			vadp_fmt++;
			total_frame_sizes++;
		}
	}

	if (fsize->index >= total_frame_sizes) {
		pr_info("[%s]fsize->index(%d) is equal or bigger than %d\n",
			__func__, fsize->index, total_frame_sizes);
		return -EINVAL;
	}

	vadp_fmt = vadp_fmt - (total_frame_sizes - fsize->index);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = vadp_fmt->width;
	fsize->discrete.height = vadp_fmt->height;

	pr_debug("[%s]END format=%c%c%c%c, width=%d, height=%d\n",
		 __func__,
		 vadp_fmt->fourcc >> 0,
		 vadp_fmt->fourcc >> 8,
		 vadp_fmt->fourcc >> 16,
		 vadp_fmt->fourcc >> 24,
		 vadp_fmt->width,
		 vadp_fmt->height);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_playback_enum_frameintervals(
					struct file *filp,
					void *priv,
					struct v4l2_frmivalenum *fival)
{
	int index;
	const struct v4l2_fract *fract;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START fival->index=%d\n", __func__, fival->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	index = ARRAY_SIZE(g_mse_adapter_v4l2_common_frame_intervals);
	if (fival->index >= index) {
		pr_info("[%s]fival->index(%d) is equal or bigger than %d\n",
			__func__, fival->index, index);
		return -EINVAL;
	}

	fract = &g_mse_adapter_v4l2_common_frame_intervals[fival->index];
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = fract->numerator;
	fival->discrete.denominator = fract->denominator;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_g_parm(struct file *filp,
					   void *priv,
					   struct v4l2_streamparm *sp)
{
	struct v4l2_fract *fract = &sp->parm.capture.timeperframe;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	sp->parm.capture.capturemode = 0;
	sp->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	fract->denominator = vadp_dev->frameintervals.denominator;
	fract->numerator = vadp_dev->frameintervals.numerator;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_s_parm(struct file *filp,
					   void *priv,
					   struct v4l2_streamparm *sp)
{
	int err;
	struct v4l2_fract *fract = &sp->parm.capture.timeperframe;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START\n", __func__);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	vadp_dev->frameintervals.denominator = fract->denominator;
	vadp_dev->frameintervals.numerator = fract->numerator;

	err = set_config_timerperframe(vadp_dev->index_instance, fract);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_enum_framesizes(
					struct file *filp,
					void *priv,
					struct v4l2_frmsizeenum *fsize)
{
	int total_frame_sizes = 0;
	const struct v4l2_adapter_fmt *vadp_fmt = NULL;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);
	int i;

	pr_debug("[%s]START fsize->index=%d\n", __func__, fsize->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	/* get total frame sizes */
	for (i = 0; i < ARRAY_SIZE(g_mse_adapter_v4l2_common_fmt_sizes); i++) {
		if (g_mse_adapter_v4l2_common_fmt_sizes[i].fourcc ==
						fsize->pixel_format) {
			vadp_fmt = &g_mse_adapter_v4l2_common_fmt_sizes[i];
			vadp_fmt++;
			total_frame_sizes++;
		}
	}

	if (fsize->index >= total_frame_sizes) {
		pr_info("[%s]fsize->index(%d) is equal or bigger than %d\n",
			__func__, fsize->index, total_frame_sizes);
		return -EINVAL;
	}

	vadp_fmt = vadp_fmt - (total_frame_sizes - fsize->index);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = vadp_fmt->width;
	fsize->discrete.height = vadp_fmt->height;

	pr_debug("[%s]format=%c%c%c%c, width=%d, height=%d\n",
		 __func__,
		 vadp_fmt->fourcc >> 0,
		 vadp_fmt->fourcc >> 8,
		 vadp_fmt->fourcc >> 16,
		 vadp_fmt->fourcc >> 24,
		 vadp_fmt->width,
		 vadp_fmt->height);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_capture_enum_frameintervals(
					struct file *filp,
					void *priv,
					struct v4l2_frmivalenum *fival)
{
	int index;
	const struct v4l2_fract *fract;
	struct v4l2_adapter_device *vadp_dev = video_drvdata(filp);

	pr_debug("[%s]START fival->index=%d\n", __func__, fival->index);

	if (!vadp_dev) {
		pr_err("[%s]Failed video_drvdata()\n", __func__);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	index = ARRAY_SIZE(g_mse_adapter_v4l2_common_frame_intervals);
	if (fival->index >= index) {
		pr_info("[%s]fival->index(%d) is equal or bigger than %d\n",
			__func__, fival->index, index);
		return -EINVAL;
	}

	fract = &g_mse_adapter_v4l2_common_frame_intervals[fival->index];
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = fract->numerator;
	fival->discrete.denominator = fract->denominator;

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static const struct vb2_ops g_mse_adapter_v4l2_capture_queue_ops = {
	.queue_setup		= mse_adapter_v4l2_capture_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= mse_adapter_v4l2_capture_buf_prepare,
	.start_streaming	= mse_adapter_v4l2_capture_start_streaming,
	.stop_streaming		= mse_adapter_v4l2_capture_stop_streaming,
	.buf_queue		= mse_adapter_v4l2_capture_buf_queue,
};

static const struct vb2_ops g_mse_adapter_v4l2_playback_queue_ops = {
	.queue_setup		= mse_adapter_v4l2_playback_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= mse_adapter_v4l2_playback_buf_prepare,
	.start_streaming	= mse_adapter_v4l2_playback_start_streaming,
	.stop_streaming		= mse_adapter_v4l2_playback_stop_streaming,
	.buf_queue		= mse_adapter_v4l2_playback_buf_queue,
};

static const struct v4l2_ioctl_ops g_mse_adapter_v4l2_capture_ioctl_ops = {
	.vidioc_querycap		= mse_adapter_v4l2_capture_querycap,
	.vidioc_enum_fmt_vid_cap	=
			mse_adapter_v4l2_capture_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		=
			mse_adapter_v4l2_capture_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		=
			mse_adapter_v4l2_capture_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		=
			mse_adapter_v4l2_capture_try_fmt_vid_cap,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_input		= mse_adapter_v4l2_capture_enum_input,
	.vidioc_g_input			= mse_adapter_v4l2_capture_g_input,
	.vidioc_s_input			= mse_adapter_v4l2_capture_s_input,
	.vidioc_g_parm			= mse_adapter_v4l2_capture_g_parm,
	.vidioc_s_parm			= mse_adapter_v4l2_capture_s_parm,
	.vidioc_enum_framesizes		=
			mse_adapter_v4l2_capture_enum_framesizes,
	.vidioc_enum_frameintervals	=
			mse_adapter_v4l2_capture_enum_frameintervals,
};

static const struct v4l2_ioctl_ops g_mse_adapter_v4l2_playback_ioctl_ops = {
	.vidioc_querycap		= mse_adapter_v4l2_playback_querycap,
	.vidioc_enum_fmt_vid_out	=
			mse_adapter_v4l2_playback_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		=
			mse_adapter_v4l2_playback_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out		=
			mse_adapter_v4l2_playback_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out		=
			mse_adapter_v4l2_playback_try_fmt_vid_out,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_output		=
					mse_adapter_v4l2_playback_enum_output,
	.vidioc_g_output		= mse_adapter_v4l2_playback_g_output,
	.vidioc_s_output		= mse_adapter_v4l2_playback_s_output,
	.vidioc_g_parm			= mse_adapter_v4l2_playback_g_parm,
	.vidioc_s_parm			= mse_adapter_v4l2_playback_s_parm,
	.vidioc_enum_framesizes		=
			mse_adapter_v4l2_playback_enum_framesizes,
	.vidioc_enum_frameintervals	=
			mse_adapter_v4l2_playback_enum_frameintervals,
};

static struct v4l2_file_operations g_mse_adapter_v4l2_fops = {
	.owner		= THIS_MODULE,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.open		= mse_adapter_v4l2_fop_open,
	.release	= mse_adapter_v4l2_fop_release,
};

/* Playback init information */
static const struct v4l2_adapter_init_info g_playback_init_info = {
	.ioctl_ops	= &g_mse_adapter_v4l2_playback_ioctl_ops,
	.vfl_dir	= VFL_DIR_TX,
	.type		= V4L2_BUF_TYPE_VIDEO_OUTPUT,
	.queue_ops	= &g_mse_adapter_v4l2_playback_queue_ops,
	.adapter_name	= "Renesas MSE Adapter playback %d",
};

/* Capture init information */
static const struct v4l2_adapter_init_info g_capture_init_info = {
	.ioctl_ops	= &g_mse_adapter_v4l2_capture_ioctl_ops,
	.vfl_dir	= VFL_DIR_RX,
	.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
	.queue_ops	= &g_mse_adapter_v4l2_capture_queue_ops,
	.adapter_name	= "Renesas MSE Adapter capture %d",
};

static const struct v4l2_adapter_init_info *get_device_init_info(
							unsigned int buf_type)
{
	if (buf_type == g_capture_init_info.type)
		return &g_capture_init_info;
	else
		return &g_playback_init_info;
}

static int register_mse_core(struct v4l2_adapter_device *vadp_dev)
{
	int index_mse;
	struct video_device *vdev = &vadp_dev->vdev;
	char device_name[MSE_NAME_LEN_MAX];
	enum MSE_DIRECTION inout;

	sprintf(device_name, "/dev/%s", video_device_node_name(vdev));

	if (vdev->vfl_dir == VFL_DIR_RX) {
		inout = MSE_DIRECTION_OUTPUT;
	} else if (vdev->vfl_dir == VFL_DIR_TX) {
		inout = MSE_DIRECTION_INPUT;
	} else {
		pr_err("[%s] illegal vfl_dir=%d\n", __func__, vdev->vfl_dir);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	index_mse = mse_register_adapter_media(MSE_TYPE_ADAPTER_VIDEO_H264,
					       inout,
					       vdev->name,
					       vadp_dev,
					       device_name);
	if (index_mse < MSE_ADAPTER_V4L2_RTN_OK)
		return MSE_ADAPTER_V4L2_RTN_NG;

	vadp_dev->index_mse = index_mse;

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static struct v4l2_adapter_device *g_v4l2_adapter[MSE_ADAPTER_V4L2_DEVICE_MAX];

static int mse_adapter_v4l2_probe(int dev_num, unsigned int buf_type)
{
	int err;
	static int reg_num;
	struct v4l2_adapter_device *vadp_dev;
	struct video_device *vdev;
	struct v4l2_device *v4l2_dev;
	struct vb2_queue *q;
	const struct v4l2_adapter_init_info *init_info;

	pr_debug("[%s]START device number=%d, buf_type=%d\n",
		 __func__, dev_num, buf_type);

	vadp_dev = kzalloc(sizeof(*vadp_dev), GFP_KERNEL);
	if (!vadp_dev)
		return MSE_ADAPTER_V4L2_RTN_NG;

	g_v4l2_adapter[reg_num++] = vadp_dev;
	vadp_dev->index_mse = MSE_INDEX_UNDEFINED;
	vadp_dev->index_instance = MSE_INDEX_UNDEFINED;

	init_info = get_device_init_info(buf_type);

	vdev = &vadp_dev->vdev;
	snprintf(vdev->name, sizeof(vdev->name),
		 init_info->adapter_name, dev_num);
	vdev->release = video_device_release_empty;
	vdev->fops = &g_mse_adapter_v4l2_fops;
	vdev->vfl_type = VFL_TYPE_GRABBER;
	vdev->ioctl_ops = init_info->ioctl_ops;
	vdev->vfl_dir = init_info->vfl_dir;

	mutex_init(&vadp_dev->lock);

	q = &vadp_dev->queue;
	q->type = init_info->type;
	q->io_modes = VB2_MMAP;
	q->drv_priv = vadp_dev;
	q->buf_struct_size = sizeof(struct v4l2_adapter_buffer);
	q->ops = init_info->queue_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &vadp_dev->lock;
	q->gfp_flags = GFP_DMA32;
	q->min_buffers_needed = 2;

	err = vb2_queue_init(q);
	if (err) {
		pr_err("[%s]Failed vb2_queue_init() Rtn=%d\n", __func__, err);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	INIT_LIST_HEAD(&vadp_dev->buf_list);
	spin_lock_init(&vadp_dev->qlock);

	vdev->lock = &vadp_dev->lock;
	vdev->queue = q;
	video_set_drvdata(vdev, vadp_dev);

	v4l2_dev = &vadp_dev->v4l2_dev;
	snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
		 "Renesas MSE Device %d", dev_num);
	err = v4l2_device_register(NULL, v4l2_dev);
	if (err) {
		pr_err("[%s]Failed v4l2_device_register() Rtn=%d\n",
		       __func__, err);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	vdev->v4l2_dev = v4l2_dev;
	err = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (err) {
		pr_err("[%s]Failed video_register_device() Rtn=%d\n",
		       __func__, err);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	pr_debug("[%s]video device was registered as (%s)",
		 __func__, video_device_node_name(vdev));

	err = register_mse_core(vadp_dev);
	if (err < MSE_ADAPTER_V4L2_RTN_OK) {
		pr_err("[%s]Failed register_mse_core() Rtn=%d\n",
		       __func__, err);
		return MSE_ADAPTER_V4L2_RTN_NG;
	}

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static void mse_adapter_v4l2_cleanup(struct v4l2_adapter_device *vadp_dev)
{
	v4l2_device_unregister(&vadp_dev->v4l2_dev);
	video_unregister_device(&vadp_dev->vdev);
	kfree(vadp_dev);
}

static void unregister_mse_core(struct v4l2_adapter_device *vadp_dev)
{
	int err;
	int index = vadp_dev->index_mse;

	if (index == MSE_INDEX_UNDEFINED) {
		pr_info("[%s]already unregistered(%d)\n", __func__, index);
		return;
	}

	err = mse_unregister_adapter_media(index);
	if (err < MSE_ADAPTER_V4L2_RTN_OK)
		pr_err("[%s]Failed mse_unregister_adapter_media()\n",
		       __func__);
}

static int mse_adapter_v4l2_free(void)
{
	int dev_num;
	struct v4l2_adapter_device *vadp_dev;

	pr_debug("[%s]START\n", __func__);

	for (dev_num = 0; dev_num < MSE_ADAPTER_V4L2_DEVICE_MAX; dev_num++) {
		vadp_dev = g_v4l2_adapter[dev_num];
		if (!vadp_dev)
			break;

		unregister_mse_core(vadp_dev);
		mse_adapter_v4l2_cleanup(vadp_dev);
		g_v4l2_adapter[dev_num] = NULL;
	}

	pr_debug("[%s]END\n", __func__);

	return MSE_ADAPTER_V4L2_RTN_OK;
}

static int mse_adapter_v4l2_init(void)
{
	int i;
	int dev_num;
	int err;
	unsigned int type;
	int rtn = MSE_ADAPTER_V4L2_RTN_OK;

	pr_debug("Start v4l2 adapter\n");
	mutex_init(&g_lock);
	for (i = 0; i < MSE_ADAPTER_V4L2_DEVICE_MAX; i++) {
		if (i < MSE_ADAPTER_V4L2_CAPTURE_DEVICE_MAX) {
			dev_num = i;
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		} else {
			dev_num = i - MSE_ADAPTER_V4L2_CAPTURE_DEVICE_MAX;
			type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		}

		err = mse_adapter_v4l2_probe(dev_num, type);
		if (err) {
			pr_err("Failed creating device=%d Rtn=%d\n",
			       dev_num, err);
			rtn = MSE_ADAPTER_V4L2_RTN_NG;
			break;
		}
	}

	if (rtn == MSE_ADAPTER_V4L2_RTN_NG)
		mse_adapter_v4l2_free();

	return rtn;
}

/* module clean up */
static void mse_adapter_v4l2_exit(void)
{
	pr_debug("Stop v4l2 adapter\n");
	mse_adapter_v4l2_free();
}

module_init(mse_adapter_v4l2_init)
module_exit(mse_adapter_v4l2_exit)

MODULE_AUTHOR("Jose Luis HIRANO");
MODULE_DESCRIPTION("Renesas Media Streaming Engine");
MODULE_LICENSE("Dual MIT/GPL");
