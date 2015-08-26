/*
 * Copyright 2009-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file drivers/media/video/mxc/capture/vbase_v4l2_capture.c
 * This file is derived from csi_v4l2_capture.c
 *
 * @brief Video For Linux 2 capture driver
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#include <linux/busfreq-imx6.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/pagemap.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-chip-ident.h>
#include "mxc_v4l2_capture.h"
#include "fsl_csi.h"


static int req_buf_number;

static int csi_v4l2_master_attach(struct v4l2_int_device *slave);
static void csi_v4l2_master_detach(struct v4l2_int_device *slave);
static u8 camera_power(cam_data *cam, bool cameraOn);

/*! Information about this driver. */
static struct v4l2_int_master csi_v4l2_master = {
  .attach = csi_v4l2_master_attach,
  .detach = csi_v4l2_master_detach,
};


/*!
 * Camera V4l2 callback function.
 *
 * @param mask     u32
 * @param dev      void device structure
 *
 * @return none
 */
static void camera_callback(u32 mask, void *dev)
{
  struct mxc_v4l_frame *done_frame;
  struct mxc_v4l_frame *ready_frame;
  cam_data *cam;

  cam = (cam_data *) dev;
  if (cam == NULL)
    return;

  spin_lock(&cam->queue_int_lock);
  spin_lock(&cam->dqueue_int_lock);

  if (mask == 0) {
    if (cam->ping_pong_csi == 1) {
      if (!list_empty(&cam->working_q)) {

        csi_dmareq_rff_disable(cam->csi_soc);
        csi_disable_int(cam);
        csi_enable_int(cam, 0);

        done_frame = list_entry(cam->working_q.next, struct mxc_v4l_frame, queue);

        if (done_frame->buffer.flags & V4L2_BUF_FLAG_QUEUED) {

          done_frame->buffer.flags |= V4L2_BUF_FLAG_DONE;
          done_frame->buffer.flags &= ~V4L2_BUF_FLAG_QUEUED;

          /* Added to the done queue */
          list_del(cam->working_q.next);
          list_add_tail(&done_frame->queue, &cam->done_q);
          cam->enc_counter++;
          wake_up_interruptible(&cam->enc_queue);
        } else {

        }
      }
    }
  } else {
    unsigned long val;

    if (!list_empty(&cam->ready_q)) {
      ready_frame = list_entry(cam->ready_q.next, struct mxc_v4l_frame, queue);
      list_del(cam->ready_q.next);
      list_add_tail(&ready_frame->queue, &cam->working_q);
      csi_write(cam->csi_soc, ready_frame->paddress, CSI_CSIDMASA_FB1);
      csi_write(cam->csi_soc, cam->dummy_frame.paddress, CSI_CSIDMASA_FB2);

      val = csi_read(cam->csi_soc, CSI_CSICR3);
      csi_write(cam->csi_soc, val | BIT_DMA_REFLASH_RFF, CSI_CSICR3);

      val = csi_read(cam->csi_soc, CSI_CSICR1);
      csi_write(cam->csi_soc, val | BIT_CLR_RXFIFO, CSI_CSICR1);

      csi_dmareq_rff_enable(cam->csi_soc);
      csi_enable_int(cam, 1);

      val = csi_read(cam->csi_soc, CSI_CSICR18);
      csi_write(cam->csi_soc, val | BIT_CSI_ENABLE, CSI_CSICR18);
    }
    else
    {
      if (list_empty(&cam->working_q))
      {
        csi_dmareq_rff_disable(cam->csi_soc);
        csi_disable_int(cam);
        csi_enable_int(cam, 0);
      }else{

      }
    }
  }

  spin_unlock(&cam->dqueue_int_lock);
  spin_unlock(&cam->queue_int_lock);

  return;
}

/*!
 * Make csi ready for capture image.
 *
 * @param cam      structure cam_data *
 *
 * @return status 0 success
 */
static int csi_cap_image(cam_data *cam)
{
  unsigned int value;

  value = csi_read(cam->csi_soc, CSI_CSICR3);
  csi_write(cam->csi_soc, value | BIT_FRMCNT_RST, CSI_CSICR3);
  value = csi_read(cam->csi_soc, CSI_CSISR);
  csi_write(cam->csi_soc, value, CSI_CSISR);

  return 0;
}

/***************************************************************************
 * Functions for handling Frame buffers.
 **************************************************************************/

/*!
 * Free frame buffers
 *
 * @param cam      Structure cam_data *
 *
 * @return status  0 success.
 */
static int csi_free_frame_buf(cam_data *cam)
{
  int i;

  pr_debug("MVC: In %s\n", __func__);

  for (i = 0; i < FRAME_NUM; i++) {
    if (cam->frame[i].vaddress != 0) {
      dma_free_coherent(0,
                        cam->frame[i].buffer.length,
                        cam->frame[i].vaddress,
                        cam->frame[i].paddress);
      cam->frame[i].vaddress = 0;
    }
  }

  if (cam->dummy_frame.vaddress != 0) {
    dma_free_coherent(0,
                      cam->dummy_frame.buffer.length,
                      cam->dummy_frame.vaddress,
                      cam->dummy_frame.paddress);
    cam->dummy_frame.vaddress = 0;
  }

  return 0;
}

/*!
 * Allocate frame buffers
 *
 * @param cam      Structure cam_data *
 * @param count    int number of buffer need to allocated
 *
 * @return status  -0 Successfully allocated a buffer, -ENOBUFS failed.
 */
static int csi_allocate_frame_buf(cam_data *cam, int count)
{
  int i;

  pr_debug("In MVC:%s- size=%d\n",
     __func__, cam->v2f.fmt.pix.sizeimage);
  for (i = 0; i < count; i++) {
    cam->frame[i].vaddress = dma_alloc_coherent(0,
                                                PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
                                                &cam->frame[i].paddress,
                                                GFP_DMA | GFP_KERNEL);
    if (cam->frame[i].vaddress == 0) {
      pr_err("ERROR: v4l2 capture: %s failed.\n", __func__);
      csi_free_frame_buf(cam);
      return -ENOBUFS;
    }
    cam->frame[i].buffer.index    = i;
    cam->frame[i].buffer.flags    = V4L2_BUF_FLAG_MAPPED;
    cam->frame[i].buffer.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cam->frame[i].buffer.length   = cam->v2f.fmt.pix.sizeimage;
    cam->frame[i].buffer.memory   = V4L2_MEMORY_MMAP;
    cam->frame[i].buffer.m.offset = cam->frame[i].paddress;
    cam->frame[i].index           = i;
    cam->frame[i].csi_buf_num     = 0;
  }

  return 0;
}

/*!
 * Free frame buffers status
 *
 * @param cam    Structure cam_data *
 *
 * @return none
 */
static void csi_free_frames(cam_data *cam)
{
  int i;

  pr_debug("In MVC: %s\n", __func__);

  for (i = 0; i < FRAME_NUM; i++)
    cam->frame[i].buffer.flags = V4L2_BUF_FLAG_MAPPED;

  cam->enc_counter = 0;
  INIT_LIST_HEAD(&cam->ready_q);
  INIT_LIST_HEAD(&cam->working_q);
  INIT_LIST_HEAD(&cam->done_q);

  return;
}

/*!
 * Return the buffer status
 *
 * @param cam      Structure cam_data *
 * @param buf      Structure v4l2_buffer *
 *
 * @return status  0 success, EINVAL failed.
 */
static int csi_v4l2_buffer_status(cam_data *cam, struct v4l2_buffer *buf)
{
  pr_debug("In MVC: %s\n", __func__);

  if (buf->index < 0 || buf->index >= FRAME_NUM) {
    pr_err("ERROR: v4l2 capture: %s buffers not allocated\n", __func__);
    return -EINVAL;
  }
  memcpy(buf, &(cam->frame[buf->index].buffer), sizeof(*buf));

  return 0;
}

static int csi_v4l2_release_bufs(cam_data *cam)
{
  pr_debug("In MVC:csi_v4l2_release_bufs\n");
  return 0;
}

static int csi_v4l2_prepare_bufs(cam_data *cam, struct v4l2_buffer *buf)
{
  pr_debug("In MVC:csi_v4l2_prepare_bufs\n");

  if (buf->index < 0 ||
      buf->index >= FRAME_NUM ||
      buf->length < cam->v2f.fmt.pix.sizeimage) {
    pr_err("ERROR: v4l2 capture: csi_v4l2_prepare_bufs buffers not allocated,index=%d, length=%d\n",
            buf->index,
            buf->length);
    return -EINVAL;
  }

  cam->frame[buf->index].buffer.index     = buf->index;
  cam->frame[buf->index].buffer.flags     = V4L2_BUF_FLAG_MAPPED;
  cam->frame[buf->index].buffer.length    = buf->length;
  cam->frame[buf->index].buffer.m.offset  = buf->m.offset;
  cam->frame[buf->index].paddress         = buf->m.offset;
  cam->frame[buf->index].buffer.type      = buf->type;
  cam->frame[buf->index].buffer.memory    = V4L2_MEMORY_USERPTR;
  cam->frame[buf->index].index            = buf->index;

  return 0;
}

/*!
 * Indicates whether the palette is supported.
 *
 * @param palette V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_UYVY or V4L2_PIX_FMT_YUV420
 *
 * @return 0 if failed
 */
static inline int valid_mode(u32 palette)
{
  return  (palette == V4L2_PIX_FMT_RGB565) ||
          (palette == V4L2_PIX_FMT_YUYV)   ||
          (palette == V4L2_PIX_FMT_RGB32)  ||
          (palette == V4L2_PIX_FMT_UYVY)   ||
          (palette == V4L2_PIX_FMT_YUV444) ||
          (palette == V4L2_PIX_FMT_YUV420);
}

/*!
 * Start stream I/O
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int csi_streamon(cam_data *cam)
{
  pr_debug("In MVC: %s\n", __func__);

  if (NULL == cam) {
    pr_err("ERROR: v4l2 capture: %s cam parameter is NULL\n", __func__);
    return -1;
  }

  cam->dummy_frame.vaddress = dma_alloc_coherent(0,
                                                 PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
                                                 &cam->dummy_frame.paddress,
                                                 GFP_DMA | GFP_KERNEL);
  if (cam->dummy_frame.vaddress == 0) {
    pr_err("ERROR: v4l2 capture: Allocate dummy frame failed.\n");
    return -ENOBUFS;
  }
  cam->dummy_frame.buffer.type     = V4L2_BUF_TYPE_PRIVATE;
  cam->dummy_frame.buffer.length   = cam->v2f.fmt.pix.sizeimage;
  cam->dummy_frame.buffer.m.offset = cam->dummy_frame.paddress;

  cam->capture_pid = current->pid;
  cam->capture_on  = true;

  csi_cap_image(cam);
  csi_dmareq_rff_disable(cam->csi_soc);
  csi_enable_int(cam, 0);
  csi_enable(cam, 1);

  return 0;
}

/*!
 * Stop stream I/O
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static int csi_streamoff(cam_data *cam)
{
  pr_debug("In MVC: %s\n", __func__);

  if (cam->capture_on == false)
    return 0;

  csi_dmareq_rff_disable(cam->csi_soc);
  csi_disable_int(cam);
  cam->capture_on = false;

  /* set CSI_CSIDMASA_FB1 and CSI_CSIDMASA_FB2 to default value */
  csi_write(cam->csi_soc, 0, CSI_CSIDMASA_FB1);
  csi_write(cam->csi_soc, 0, CSI_CSIDMASA_FB2);

  csi_enable(cam, 0);

  csi_free_frames(cam);
  csi_free_frame_buf(cam);

  return 0;
}


/***************************************************************************
 * VIDIOC Functions.
 **************************************************************************/

/*!
 *
 * @param cam         structure cam_data *
 *
 * @param f           structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int csi_v4l2_g_fmt(cam_data *cam, struct v4l2_format *f)
{
  int retval = 0;

  switch (f->type) {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
      pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
      f->fmt.pix = cam->v2f.fmt.pix;
      break;

    default:
      pr_debug("   type is invalid\n");
      retval = -EINVAL;
      break;
  }

  pr_debug("End of %s: v2f pix widthxheight %d x %d\n",
     __func__, cam->v2f.fmt.pix.width, cam->v2f.fmt.pix.height);

  return retval;
}

/*!
 * V4L2 - csi_v4l2_s_fmt function
 *
 * @param cam         structure cam_data *
 *
 * @param f           structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int csi_v4l2_s_fmt(cam_data *cam, struct v4l2_format *f)
{
  int retval = 0;
  int size = 0;
  int bytesperline = 0;

  pr_debug("In MVC: %s\n", __func__);

  switch (f->type) {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
      pr_debug("   type=V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
      if (!valid_mode(f->fmt.pix.pixelformat)) {
        pr_err("ERROR: v4l2 capture: %s: format not supported\n", __func__);
        return -EINVAL;
      }

      /* disable swap function */
      csi_format_swap16(cam, false);
      cam->bswapenable = false;

      switch (f->fmt.pix.pixelformat) {
        case V4L2_PIX_FMT_RGB32:
          size = f->fmt.pix.width * f->fmt.pix.height * 4;
          bytesperline = f->fmt.pix.width * 4;
          break;

        case V4L2_PIX_FMT_RGB565:
          size = f->fmt.pix.width * f->fmt.pix.height * 2;
          bytesperline = f->fmt.pix.width * 2;
          break;

        case V4L2_PIX_FMT_YUV444:
          size = f->fmt.pix.width * f->fmt.pix.height * 4;
          bytesperline = f->fmt.pix.width * 4;
          break;

        case V4L2_PIX_FMT_UYVY:
          size = f->fmt.pix.width * f->fmt.pix.height * 2;
          bytesperline = f->fmt.pix.width * 2;
          if (cam->input_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
            csi_format_swap16(cam, true);
            cam->bswapenable = true;
          }
          break;

        case V4L2_PIX_FMT_YUYV:
          size = f->fmt.pix.width * f->fmt.pix.height * 2;
          bytesperline = f->fmt.pix.width * 2;
          if (cam->input_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY) {
            csi_format_swap16(cam, true);
            cam->bswapenable = true;
          }
          break;

        case V4L2_PIX_FMT_YUV420:
          size = f->fmt.pix.width * f->fmt.pix.height * 3 / 2;
          bytesperline = f->fmt.pix.width;
          break;

        case V4L2_PIX_FMT_YUV422P:
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
        case V4L2_PIX_FMT_BGR32:
        case V4L2_PIX_FMT_NV12:
        default:
          pr_debug("   case not supported\n");
          break;
      }

      if (f->fmt.pix.bytesperline < bytesperline)
        f->fmt.pix.bytesperline = bytesperline;
      else
        bytesperline = f->fmt.pix.bytesperline;

      if (f->fmt.pix.sizeimage < size)
        f->fmt.pix.sizeimage = size;
      else
        size = f->fmt.pix.sizeimage;

      if (cam->input_fmt.fmt.pix.sizeimage > f->fmt.pix.sizeimage)
         f->fmt.pix.sizeimage = cam->input_fmt.fmt.pix.sizeimage;

      cam->v2f.fmt.pix = f->fmt.pix;

      if (cam->v2f.fmt.pix.priv != 0) {
        if (copy_from_user(&cam->offset, (void *)cam->v2f.fmt.pix.priv, sizeof(cam->offset))) {
          retval = -EFAULT;
          break;
        }
      }
      break;

    default:
      retval = -EINVAL;
      break;
  }

  pr_debug("End of %s: v2f pix widthxheight %d x %d\n", __func__,
                                                        cam->v2f.fmt.pix.width,
                                                        cam->v2f.fmt.pix.height);

  return retval;
}

/*!
 * V4L2 - csi_v4l2_s_param function
 * Allows setting of capturemode and frame rate.
 *
 * @param cam         structure cam_data *
 * @param parm        structure v4l2_streamparm *
 *
 * @return  status    0 success, EINVAL failed
 */
static int csi_v4l2_s_param(cam_data *cam, struct v4l2_streamparm *parm)
{
  struct v4l2_ifparm ifparm;
  struct v4l2_format *f;
  struct v4l2_streamparm currentparm;
  int err = 0;
  int size = 0;

  pr_debug("In %s\n", __func__);

  if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
    pr_err(KERN_ERR "%s invalid type\n", __func__);
    return -EINVAL;
  }

  currentparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  /* First check that this device can support the changes requested. */
  err = vidioc_int_g_parm(cam->sensor, &currentparm);
  if (err) {
    pr_err("%s: vidioc_int_g_parm returned an error %d\n", __func__, err);
    goto exit;
  }

  pr_debug("   Current capabilities are %x\n", currentparm.parm.capture.capability);
  pr_debug("   Current capturemode is %d  change to %d\n", currentparm.parm.capture.capturemode,
                                                           parm->parm.capture.capturemode);
  pr_debug("   Current framerate is %d  change to %d\n", currentparm.parm.capture.timeperframe.denominator,
                                                         parm->parm.capture.timeperframe.denominator);

  err = vidioc_int_s_parm(cam->sensor, parm);
  if (err) {
    pr_err("%s: vidioc_int_s_parm returned an error %d\n", __func__, err);
    goto exit;
  }

  vidioc_int_g_ifparm(cam->sensor, &ifparm);
  cam->input_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vidioc_int_g_fmt_cap(cam->sensor, &cam->input_fmt);

  pr_debug("   g_fmt_cap returns widthxheight of input as %d x %d\n", cam->input_fmt.fmt.pix.width,
                                                                      cam->input_fmt.fmt.pix.height);

  f = &cam->input_fmt;
  switch (f->fmt.pix.pixelformat) {
    case V4L2_PIX_FMT_YUV444:
      size = f->fmt.pix.width * f->fmt.pix.height * 4;
      csi_set_32bit_imagpara(cam, f->fmt.pix.width, f->fmt.pix.height);
      break;

    case V4L2_PIX_FMT_RGB32:
      size = f->fmt.pix.width * f->fmt.pix.height * 4;
      csi_set_16bit_imagpara(cam, f->fmt.pix.width, f->fmt.pix.height);
      break;

    case V4L2_PIX_FMT_UYVY:
      size = f->fmt.pix.width * f->fmt.pix.height * 2;
      csi_set_32bit_imagpara(cam, f->fmt.pix.width, f->fmt.pix.height);
      break;

    case V4L2_PIX_FMT_YUYV:
      size = f->fmt.pix.width * f->fmt.pix.height * 2;
      csi_set_32bit_imagpara(cam, f->fmt.pix.width, f->fmt.pix.height);
      break;

    case V4L2_PIX_FMT_YUV420:
      size = f->fmt.pix.width * f->fmt.pix.height * 3 / 2;
      csi_set_12bit_imagpara(cam, f->fmt.pix.width, f->fmt.pix.height);
      break;

    case V4L2_PIX_FMT_YUV422P:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_BGR24:
    case V4L2_PIX_FMT_BGR32:
    case V4L2_PIX_FMT_NV12:
    default:
      pr_debug("   case not supported\n");
      return -EINVAL;
  }

  f->fmt.pix.sizeimage = size;

exit:
  return err;
}

/*!
 * Dequeue one V4L capture buffer
 *
 * @param cam         structure cam_data *
 * @param buf         structure v4l2_buffer *
 *
 * @return  status    0 success, EINVAL invalid frame number
 *                    ETIME timeout, ERESTARTSYS interrupted by user
 */
static int csi_v4l_dqueue(cam_data *cam, struct v4l2_buffer *buf)
{
  int retval = 0;
  struct mxc_v4l_frame *frame;
  unsigned long lock_flags;

  if (!wait_event_interruptible_timeout(cam->enc_queue, cam->enc_counter != 0, 10 * HZ)) {
    pr_err("ERROR: v4l2 capture: mxc_v4l_dqueue timeout enc_counter %x\n", cam->enc_counter);
    return -ETIME;
  } else if (signal_pending(current)) {
    pr_err("ERROR: v4l2 capture: mxc_v4l_dqueue() interrupt received\n");
    return -ERESTARTSYS;
  }

  if (down_interruptible(&cam->busy_lock))
    return -EBUSY;

  spin_lock_irqsave(&cam->dqueue_int_lock, lock_flags);

  if (list_empty(&cam->done_q)) {
    spin_unlock_irqrestore(&cam->dqueue_int_lock, lock_flags);
    up(&cam->busy_lock);
    return -EINVAL;
  }

  cam->enc_counter--;

  frame = list_entry(cam->done_q.next, struct mxc_v4l_frame, queue);
  list_del(cam->done_q.next);

  if (frame->buffer.flags & V4L2_BUF_FLAG_DONE)
  {
    frame->buffer.flags &= ~V4L2_BUF_FLAG_DONE;
  }
  else if (frame->buffer.flags & V4L2_BUF_FLAG_QUEUED)
  {
    pr_err("ERROR: v4l2 capture: VIDIOC_DQBUF: Buffer not filled.\n");
    frame->buffer.flags &= ~V4L2_BUF_FLAG_QUEUED;
    retval = -EINVAL;
  }
  else if ((frame->buffer.flags & 0x7) == V4L2_BUF_FLAG_MAPPED)
  {
    pr_err("ERROR: v4l2 capture: VIDIOC_DQBUF: Buffer not queued.\n");
    retval = -EINVAL;
  }

  spin_unlock_irqrestore(&cam->dqueue_int_lock, lock_flags);

  buf->bytesused  = cam->v2f.fmt.pix.sizeimage;
  buf->index      = frame->index;
  buf->flags      = frame->buffer.flags;
  buf->m          = cam->frame[frame->index].buffer.m;

  up(&cam->busy_lock);

  return retval;
}

/*!
 * V4L interface - open function
 *
 * @param file         structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *                    ENODEV timeout, ERESTARTSYS interrupted by user
 */
static int csi_v4l_open(struct file *file)
{
  struct v4l2_ifparm ifparm;
  struct v4l2_format cam_fmt;
  struct video_device *dev = video_devdata(file);
  cam_data *cam = video_get_drvdata(dev);
  struct sensor_data *sensor;
  int err = 0;

  pr_debug("   device name is %s\n", dev->name);

  if (!cam) {
    pr_err("%s: Internal error, cam_data not found!\n", __func__);
    return -EBADF;
  }

  if (!cam->sensor) {
    pr_err("%s: Internal error, camera is not found!\n", __func__);
    return -EBADF;
  }

  sensor = cam->sensor->priv;
  if (!sensor) {
    pr_err("%s: Internal error, sensor_data is not found!\n", __func__);
    return -EBADF;
  }

  down(&cam->busy_lock);
  err = 0;
  if (signal_pending(current))
    goto oops;

  if (cam->open_count++ == 0) {
    pm_runtime_get_sync(&cam->pdev->dev);

    wait_event_interruptible(cam->power_queue, cam->low_power == false);

    cam->enc_counter = 0;
    INIT_LIST_HEAD(&cam->ready_q);
    INIT_LIST_HEAD(&cam->working_q);
    INIT_LIST_HEAD(&cam->done_q);

    vidioc_int_g_ifparm(cam->sensor, &ifparm);

    cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    csi_clk_enable();
    clk_prepare_enable(sensor->sensor_clk);
    vidioc_int_s_power(cam->sensor, 1);
    vidioc_int_init(cam->sensor);
    vidioc_int_dev_init(cam->sensor);
  }

  file->private_data = dev;

oops:
  up(&cam->busy_lock);
  return err;
}

/*!
 * V4L interface - close function
 *
 * @param file     struct file *
 *
 * @return         0 success
 */
static int csi_v4l_close(struct file *file)
{
  struct video_device *dev = video_devdata(file);
  int err = 0;
  cam_data *cam = video_get_drvdata(dev);
  struct sensor_data *sensor;

  pr_debug("In MVC:%s\n", __func__);

  if (!cam) {
    pr_err("%s: Internal error, cam_data not found!\n", __func__);
    return -EBADF;
  }

  if (!cam->sensor) {
    pr_err("%s: Internal error, camera is not found!\n", __func__);
    return -EBADF;
  }

  sensor = cam->sensor->priv;
  if (!sensor) {
    pr_err("%s: Internal error, sensor_data is not found!\n", __func__);
    return -EBADF;
  }

  csi_streamoff(cam);

  if (--cam->open_count == 0) {
    wait_event_interruptible(cam->power_queue, cam->low_power == false);
    file->private_data = NULL;
    vidioc_int_s_power(cam->sensor, 0);
    clk_disable_unprepare(sensor->sensor_clk);
    pm_runtime_put_sync_suspend(&cam->pdev->dev);
  }

  return err;
}

/*
 * V4L interface - read function
 *
 * @param file       struct file *
 * @param read buf   char *
 * @param count      size_t
 * @param ppos       structure loff_t *
 *
 * @return           bytes read
 */
static ssize_t csi_v4l_read(struct file *file, char *buf, size_t count,
          loff_t *ppos)
{
  int err = 0;
  struct video_device *dev = video_devdata(file);
  cam_data *cam = video_get_drvdata(dev);

  if (down_interruptible(&cam->busy_lock))
    return -EINTR;

  if (cam->still_buf_vaddr == NULL) {
    cam->still_buf_vaddr = dma_alloc_coherent(0,
                                              PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
                                              &cam->still_buf[0],
                                              GFP_DMA | GFP_KERNEL);
    if (cam->still_buf_vaddr == NULL) {
      pr_err("alloc dma memory failed\n");
      return -ENOMEM;
    }
    cam->still_counter = 0;
    csi_write(cam->csi_soc, cam->still_buf[0], CSI_CSIDMASA_FB2);
    csi_write(cam->csi_soc, cam->still_buf[0], CSI_CSIDMASA_FB1);
    csi_write(cam->csi_soc, csi_read(cam->csi_soc, CSI_CSICR3) | BIT_DMA_REFLASH_RFF, CSI_CSICR3);
    csi_write(cam->csi_soc, csi_read(cam->csi_soc, CSI_CSISR), CSI_CSISR);
    csi_write(cam->csi_soc, csi_read(cam->csi_soc, CSI_CSICR3) | BIT_FRMCNT_RST, CSI_CSICR3);
    csi_enable_int(cam, 1);
    csi_enable(cam, 1);
  }

  wait_event_interruptible(cam->still_queue, cam->still_counter);
  csi_disable_int(cam);
  err = copy_to_user(buf, cam->still_buf_vaddr, cam->v2f.fmt.pix.sizeimage);

  if (cam->still_buf_vaddr != NULL) {
    dma_free_coherent(0, PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage), cam->still_buf_vaddr, cam->still_buf[0]);
    cam->still_buf[0] = 0;
    cam->still_buf_vaddr = NULL;
  }

  up(&cam->busy_lock);
  if (err < 0)
    return err;

  return cam->v2f.fmt.pix.sizeimage - err;
}

/*!
 * V4L interface - ioctl function
 *
 * @param file       struct file*
 *
 * @param ioctlnr    unsigned int
 *
 * @param arg        void*
 *
 * @return           0 success, ENODEV for invalid device instance,
 *                   -1 for other errors.
 */
static long csi_v4l_do_ioctl(struct file *file,
          unsigned int ioctlnr, void *arg)
{
  struct video_device *dev = video_devdata(file);
  cam_data *cam = video_get_drvdata(dev);
  int retval = 0;
  unsigned long lock_flags;

  pr_debug("In MVC: %s, %x\n", __func__, ioctlnr);
  wait_event_interruptible(cam->power_queue, cam->low_power == false);
  /* make this _really_ smp-safe */
  if (ioctlnr != VIDIOC_DQBUF)
    if (down_interruptible(&cam->busy_lock))
      return -EBUSY;

  switch (ioctlnr) {
    /*!
     * V4l2 VIDIOC_G_FMT ioctl
     */
  case VIDIOC_G_FMT:{
      struct v4l2_format *gf = arg;
      pr_debug("   case VIDIOC_G_FMT\n");
      retval = csi_v4l2_g_fmt(cam, gf);
      break;
    }

    /*!
     * V4l2 VIDIOC_S_FMT ioctl
     */
  case VIDIOC_S_FMT:{
      struct v4l2_format *sf = arg;
      pr_debug("   case VIDIOC_S_FMT\n");
      retval = csi_v4l2_s_fmt(cam, sf);
      vidioc_int_s_fmt_cap(cam->sensor, sf);
      break;
    }

    /*!
     * V4l2 VIDIOC_G_FBUF ioctl
     */
  case VIDIOC_G_FBUF:{
      struct v4l2_framebuffer *fb = arg;
      *fb = cam->v4l2_fb;
      fb->capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
      break;
    }

    /*!
     * V4l2 VIDIOC_S_FBUF ioctl
     */
  case VIDIOC_S_FBUF:{
      struct v4l2_framebuffer *fb = arg;
      cam->v4l2_fb = *fb;
      break;
    }

  case VIDIOC_G_PARM:{
      struct v4l2_streamparm *parm = arg;
      pr_debug("   case VIDIOC_G_PARM\n");
      vidioc_int_g_parm(cam->sensor, parm);
      break;
    }

  case VIDIOC_S_PARM:{
      struct v4l2_streamparm *parm = arg;
      pr_debug("   case VIDIOC_S_PARM\n");
      retval = csi_v4l2_s_param(cam, parm);
      break;
    }

  case VIDIOC_QUERYCAP:{
      struct v4l2_capability *cap = arg;
      pr_debug("   case VIDIOC_QUERYCAP\n");
      strcpy(cap->driver, "csi_v4l2");
      cap->version      = KERNEL_VERSION(0, 1, 11);
      cap->capabilities = V4L2_CAP_VIDEO_OVERLAY |
                          V4L2_CAP_VIDEO_CAPTURE |
                          V4L2_CAP_STREAMING |
                          V4L2_CAP_VIDEO_OUTPUT_OVERLAY |
                          V4L2_CAP_READWRITE;
      cap->card[0]      = '\0';
      cap->bus_info[0]  = '\0';
      break;
    }

  case VIDIOC_REQBUFS: {
    struct v4l2_requestbuffers *req = arg;
    pr_debug("   case VIDIOC_REQBUFS\n");

    if (req->count > FRAME_NUM) {
      pr_err("ERROR: v4l2 capture: VIDIOC_REQBUFS: "
          "not enough buffers\n");
      req->count = FRAME_NUM;
    }

    if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
      pr_err("ERROR: v4l2 capture: VIDIOC_REQBUFS: "
          "wrong buffer type\n");
      retval = -EINVAL;
      break;
    }

    csi_streamoff(cam);
    if (req->memory & V4L2_MEMORY_MMAP) {
      csi_free_frame_buf(cam);
      retval = csi_allocate_frame_buf(cam, req->count + 1);
      req_buf_number = req->count;
    }
    break;
  }

  case VIDIOC_QUERYBUF: {
    struct v4l2_buffer *buf = arg;

    pr_debug("   case VIDIOC_QUERYBUF\n");

    if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
      retval = -EINVAL;
      break;
    }

    down(&cam->param_lock);
    if (buf->memory & V4L2_MEMORY_USERPTR) {
      csi_v4l2_release_bufs(cam);
      retval = csi_v4l2_prepare_bufs(cam, buf);
    }
    if (buf->memory & V4L2_MEMORY_MMAP)
      retval = csi_v4l2_buffer_status(cam, buf);
    up(&cam->param_lock);
    break;
  }

  case VIDIOC_QBUF: {
    struct v4l2_buffer *buf = arg;
    int index = buf->index;
    pr_debug("   case VIDIOC_QBUF\n");

    spin_lock_irqsave(&cam->queue_int_lock, lock_flags);

    cam->frame[index].buffer.m.offset = buf->m.offset;
    if (cam->frame[index].buffer.flags & V4L2_BUF_FLAG_MAPPED)
    {
      cam->frame[index].buffer.flags |= V4L2_BUF_FLAG_QUEUED;
      list_add_tail(&cam->frame[index].queue, &cam->ready_q);
    }
    else if (cam->frame[index].buffer.flags & V4L2_BUF_FLAG_QUEUED)
    {
      pr_err("ERROR: v4l2 capture: VIDIOC_QBUF: buffer already queued\n");
      retval = -EINVAL;
    }
    else if (cam->frame[index].buffer.flags & V4L2_BUF_FLAG_DONE)
    {
      pr_err("ERROR: v4l2 capture: VIDIOC_QBUF: overwrite done buffer.\n");
      cam->frame[index].buffer.flags &= ~V4L2_BUF_FLAG_DONE;
      cam->frame[index].buffer.flags |=  V4L2_BUF_FLAG_QUEUED;
      retval = -EINVAL;
    }
    buf->flags = cam->frame[index].buffer.flags;

    spin_unlock_irqrestore(&cam->queue_int_lock, lock_flags);

    break;
  }

  case VIDIOC_DQBUF: {
    struct v4l2_buffer *buf = arg;
    pr_debug("   case VIDIOC_DQBUF\n");

    retval = csi_v4l_dqueue(cam, buf);

    break;
  }

  case VIDIOC_STREAMON: {
    pr_debug("   case VIDIOC_STREAMON\n");
    retval = csi_streamon(cam);
    break;
  }

  case VIDIOC_STREAMOFF: {
    pr_debug("   case VIDIOC_STREAMOFF\n");
    retval = csi_streamoff(cam);
    break;
  }
  case VIDIOC_ENUM_FMT: {
    struct v4l2_fmtdesc *fmt = arg;
    if (cam->sensor)
      retval = vidioc_int_enum_fmt_cap(cam->sensor, fmt);
    else {
      pr_err("ERROR: v4l2 capture: slave not found!\n");
      retval = -ENODEV;
    }
    break;
  }
  case VIDIOC_ENUM_FRAMESIZES: {
    struct v4l2_frmsizeenum *fsize = arg;
    if (cam->sensor)
      retval = vidioc_int_enum_framesizes(cam->sensor, fsize);
    else {
      pr_err("ERROR: v4l2 capture: slave not found!\n");
      retval = -ENODEV;
    }
    break;
  }
  case VIDIOC_ENUM_FRAMEINTERVALS: {
    struct v4l2_frmivalenum *fival = arg;
    if (cam->sensor)
      retval = vidioc_int_enum_frameintervals(cam->sensor,
                fival);
    else {
      pr_err("ERROR: v4l2 capture: slave not found!\n");
      retval = -ENODEV;
    }
    break;
  }
  case VIDIOC_DBG_G_CHIP_IDENT: {
    struct v4l2_dbg_chip_ident *p = arg;
    p->ident = V4L2_IDENT_NONE;
    p->revision = 0;
    if (cam->sensor)
      retval = vidioc_int_g_chip_ident(cam->sensor, (int *)p);
    else {
      pr_err("ERROR: v4l2 capture: slave not found!\n");
      retval = -ENODEV;
    }
    break;
  }

  case VIDIOC_CROPCAP:
  case VIDIOC_S_CROP:
  case VIDIOC_G_CROP:
  case VIDIOC_OVERLAY:
  case VIDIOC_ENUMINPUT:
  case VIDIOC_G_INPUT:
  case VIDIOC_S_INPUT:
  case VIDIOC_G_STD:
  case VIDIOC_S_STD:
  case VIDIOC_S_CTRL:
  case VIDIOC_G_CTRL:
  case VIDIOC_QUERYCTRL:
  case VIDIOC_G_OUTPUT:
  case VIDIOC_S_OUTPUT:
  case VIDIOC_ENUMSTD:
  case VIDIOC_TRY_FMT:
  case VIDIOC_G_TUNER:
  case VIDIOC_S_TUNER:
  case VIDIOC_G_FREQUENCY:
  case VIDIOC_S_FREQUENCY:
  case VIDIOC_ENUMOUTPUT:
  default:
    pr_err("   case not supported\n");
    retval = -EINVAL;
    break;
  }

  if (ioctlnr != VIDIOC_DQBUF)
    up(&cam->busy_lock);
  return retval;
}

/*
 * V4L interface - ioctl function
 *
 * @return  None
 */
static long csi_v4l_ioctl(struct file *file,
       unsigned int cmd, unsigned long arg)
{
  return video_usercopy(file, cmd, arg, csi_v4l_do_ioctl);
}

/*!
 * V4L interface - mmap function
 *
 * @param file        structure file *
 *
 * @param vma         structure vm_area_struct *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int csi_mmap(struct file *file, struct vm_area_struct *vma)
{
  struct video_device *dev = video_devdata(file);
  unsigned long size;
  int res = 0;
  cam_data *cam = video_get_drvdata(dev);

  pr_debug("%s\n", __func__);
  pr_debug("\npgoff=0x%lx, start=0x%lx, end=0x%lx\n", vma->vm_pgoff,
                                                      vma->vm_start,
                                                      vma->vm_end);

  /* make this _really_ smp-safe */
  if (down_interruptible(&cam->busy_lock))
    return -EINTR;

  size = vma->vm_end - vma->vm_start;
  vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

  if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot)) {
    pr_err("ERROR: v4l2 capture: %s : remap_pfn_range failed\n", __func__);
    res = -ENOBUFS;
    goto csi_mmap_exit;
  }

  vma->vm_flags &= ~VM_IO;  /* using shared anonymous pages */

csi_mmap_exit:
  up(&cam->busy_lock);
  return res;
}

/*!
 * This structure defines the functions to be called in this driver.
 */
static struct v4l2_file_operations csi_v4l_fops = {
  .owner   = THIS_MODULE,
  .open    = csi_v4l_open,
  .release = csi_v4l_close,
  .read    = csi_v4l_read,
  .ioctl   = csi_v4l_ioctl,
  .mmap    = csi_mmap,
};

static struct video_device csi_v4l_template = {
  .name    = "VBase Camera",
  .fops    = &csi_v4l_fops,
  .release = video_device_release,
};

/*!
 * initialize cam_data structure
 *
 * @param cam      structure cam_data *
 *
 * @return status  0 Success
 */
static void init_camera_struct(cam_data *cam, struct platform_device *pdev)
{
  struct pxp_proc_data *proc_data = &cam->pxp_conf.proc_data;
  struct device_node *np = pdev->dev.of_node;
  int ret = 0;
  int csi_id;
  pr_debug("In MVC: %s\n", __func__);

  ret = of_property_read_u32(np, "csi_id", &csi_id);
  if (ret) {
    dev_err(&pdev->dev, "csi_id missing or invalid\n");
    return;
  }

  proc_data->hflip = 0;
  proc_data->vflip = 0;
  proc_data->rotate = 0;
  proc_data->bgcolor = 0;

  /* Default everything to 0 */
  memset(cam, 0, sizeof(cam_data));

  sema_init(&cam->param_lock, 1);
  sema_init(&cam->busy_lock, 1);

  /* TODO sanity check */
  cam->csi_soc = csi_get_soc(csi_id);

  cam->video_dev = video_device_alloc();
  if (cam->video_dev == NULL)
    return;

  *(cam->video_dev) = csi_v4l_template;

  video_set_drvdata(cam->video_dev, cam);
  cam->video_dev->minor = -1;

  init_waitqueue_head(&cam->enc_queue);
  init_waitqueue_head(&cam->still_queue);

  cam->streamparm.parm.capture.capturemode = 0;

  cam->standard.index                       = 0;
  cam->standard.id                          = V4L2_STD_UNKNOWN;
  cam->standard.frameperiod.denominator     = 60;
  cam->standard.frameperiod.numerator       = 1;
  cam->standard.framelines                  = 1280;
  cam->standard_autodetect                  = true;
  cam->streamparm.type                      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cam->streamparm.parm.capture.timeperframe = cam->standard.frameperiod;
  cam->streamparm.parm.capture.capability   = V4L2_CAP_TIMEPERFRAME;
  cam->overlay_on                           = false;
  cam->capture_on                           = false;
  cam->v4l2_fb.flags                        = V4L2_FBUF_FLAG_OVERLAY;
  cam->v2f.fmt.pix.sizeimage                = 1280 * 720 * 2;
  cam->v2f.fmt.pix.bytesperline             = 1280 * 2;
  cam->v2f.fmt.pix.width                    = 1280;
  cam->v2f.fmt.pix.height                   = 720;
  cam->v2f.fmt.pix.pixelformat              = V4L2_PIX_FMT_UYVY;
  cam->win.w.width                          = 160;
  cam->win.w.height                         = 160;
  cam->win.w.left                           = 0;
  cam->win.w.top                            = 0;
  cam->still_counter                        = 0;

  cam->csi = csi_id;
  cam->enc_callback = camera_callback;
  csi_start_callback(cam);
  init_waitqueue_head(&cam->power_queue);
  spin_lock_init(&cam->queue_int_lock);
  spin_lock_init(&cam->dqueue_int_lock);

  cam->self = kmalloc(sizeof(struct v4l2_int_device), GFP_KERNEL);
  cam->self->module = THIS_MODULE;
  sprintf(cam->self->name, "csi_v4l2_cap%d", cam->csi);
  cam->self->type = v4l2_int_type_master;
  cam->self->u.master = &csi_v4l2_master;

  cam->pdev = pdev;
}

/*!
 * camera_power function
 *    Turns Sensor power On/Off
 *
 * @param       cam           cam data struct
 * @param       cameraOn      true to turn camera on, false to turn off power.
 *
 * @return status
 */
static u8 camera_power(cam_data *cam, bool cameraOn)
{
  pr_debug("In MVC: %s on=%d\n", __func__, cameraOn);

  if (cameraOn == true) {
    vidioc_int_s_power(cam->sensor, 1);
  } else {
    vidioc_int_s_power(cam->sensor, 0);
  }
  return 0;
}

static const struct of_device_id imx_csi_v4l2_dt_ids[] = {
  { .compatible = "fsl,imx6sl-csi-v4l2", },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_csi_v4l2_dt_ids);

static int csi_v4l2_probe(struct platform_device *pdev)
{
  static cam_data *g_cam;
  struct scatterlist *sg;
  u8 err = 0;

  /* Create g_cam and initialize it. */
  g_cam = kmalloc(sizeof(cam_data), GFP_KERNEL);
  if (g_cam == NULL) {
    pr_err("ERROR: v4l2 capture: failed to register camera\n");
    err = -ENOMEM;
    goto out;
  }
  memset(&g_cam->input_fmt, 0, sizeof(g_cam->input_fmt));
  init_camera_struct(g_cam, pdev);
  platform_set_drvdata(pdev, (void *)g_cam);

  /* Set up the v4l2 device and register it */
  g_cam->self->priv = g_cam;
  v4l2_int_device_register(g_cam->self);

  /* register v4l video device */
  if (video_register_device(g_cam->video_dev, VFL_TYPE_GRABBER, -1) == -1) {
    kfree(g_cam);
    g_cam = NULL;
    pr_err("ERROR: v4l2 capture: video_register_device failed\n");
    err = -ENODEV;
    goto out;
  }
  pr_debug("   Video device registered: %s #%d\n", g_cam->video_dev->name,
                                                   g_cam->video_dev->minor);

  g_cam->pxp_chan = NULL;
  /* Initialize Scatter-gather list containing 2 buffer addresses. */
  sg = g_cam->sg;
  sg_init_table(sg, 2);

  pm_runtime_enable(&g_cam->pdev->dev);
out:
  return err;
}

static int csi_v4l2_remove(struct platform_device *pdev)
{
  cam_data *g_cam = platform_get_drvdata(pdev);

  if (g_cam == NULL)
    return -EINVAL;

  if (g_cam->open_count) {
    pr_err("ERROR: v4l2 capture:camera open "
           "-- setting ops to NULL\n");
  } else {
    pr_info("V4L2 freeing image input device\n");
    v4l2_int_device_unregister(g_cam->self);
    csi_stop_callback(g_cam);
    video_unregister_device(g_cam->video_dev);
    platform_set_drvdata(pdev, NULL);
    pm_runtime_disable(&g_cam->pdev->dev);

    kfree(g_cam);
    g_cam = NULL;
  }

  return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int csi_v4l2_runtime_suspend(struct device *dev)
{
  int ret = 0;

  release_bus_freq(BUS_FREQ_HIGH);
  dev_dbg(dev, "csi v4l2 busfreq high release.\n");

  csi_regulator_disable();

  return ret;
}

static int csi_v4l2_runtime_resume(struct device *dev)
{
  int ret = 0;

  request_bus_freq(BUS_FREQ_HIGH);
  dev_dbg(dev, "csi v4l2 busfreq high request.\n");

  csi_regulator_enable();

  return ret;
}
#else
#define mxsfb_runtime_suspend NULL
#define mxsfb_runtime_resume  NULL
#endif

#ifdef CONFIG_PM_SLEEP
/*!
 * This function is called to put the sensor in a low power state.
 * Refer to the document driver-model/driver.txt in the kernel source tree
 * for more information.
 *
 * @param   pdev  the device structure used to give information on which I2C
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and -1 on failure.
 */
static int csi_v4l2_suspend(struct device *dev)
{
  cam_data *cam = dev_get_drvdata(dev);

  pr_debug("In MVC: %s\n", __func__);

  if (cam == NULL)
    return -1;

  cam->low_power = true;

  if (cam->capture_on == true || cam->overlay_on == true)
    camera_power(cam, false);

  return 0;
}

/*!
 * This function is called to bring the sensor back from a low power state.
 * Refer to the document driver-model/driver.txt in the kernel source tree
 * for more information.
 *
 * @param   pdev   the device structure
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int csi_v4l2_resume(struct device *dev)
{
  cam_data *cam = dev_get_drvdata(dev);

  pr_debug("In MVC: %s\n", __func__);

  if (cam == NULL)
    return -1;

  cam->low_power = false;
  wake_up_interruptible(&cam->power_queue);
  if (cam->capture_on == true || cam->overlay_on == true)
    camera_power(cam, true);

  return 0;
}
#else
#define csi_v4l2_suspend  NULL
#define csi_v4l2_resume   NULL
#endif

static const struct dev_pm_ops csi_v4l2_pm_ops = {
  SET_RUNTIME_PM_OPS(csi_v4l2_runtime_suspend, csi_v4l2_runtime_resume, NULL)
  SET_SYSTEM_SLEEP_PM_OPS(csi_v4l2_suspend, csi_v4l2_resume)
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver csi_v4l2_driver = {
  .driver = {
       .name = "csi_v4l2",
       .of_match_table = of_match_ptr(imx_csi_v4l2_dt_ids),
       .pm = &csi_v4l2_pm_ops,
       },
  .probe = csi_v4l2_probe,
  .remove = csi_v4l2_remove,
  .shutdown = NULL,
};

/*!
 * Initializes the camera driver.
 */
static int csi_v4l2_master_attach(struct v4l2_int_device *slave)
{
  cam_data *cam = slave->u.slave->master->priv;
  struct sensor_data *sdata = slave->priv;
  struct v4l2_format cam_fmt;

  pr_debug("In MVC: %s\n", __func__);
  pr_debug("   slave.name = %s\n", slave->name);
  pr_debug("   master.name = %s\n", slave->u.slave->master->name);

  if (slave == NULL) {
    pr_err("ERROR: v4l2 capture: slave parameter not valid.\n");
    return -1;
  }
  if (sdata->csi != cam->csi) {
    pr_debug("%s: csi doesn't match\n", __func__);
    return -1;
  }

  cam->sensor = slave;

  cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vidioc_int_g_fmt_cap(cam->sensor, &cam_fmt);

  /* Used to detect TV in (type 1) vs. camera (type 0) */
  cam->device_type = cam_fmt.fmt.pix.priv;

  pr_debug("End of %s: v2f pix widthxheight %d x %d\n", __func__,
                                                        cam->v2f.fmt.pix.width,
                                                        cam->v2f.fmt.pix.height);

  return 0;
}

/*!
 * Disconnects the camera driver.
 */
static void csi_v4l2_master_detach(struct v4l2_int_device *slave)
{
  pr_debug("In MVC: %s\n", __func__);

  vidioc_int_dev_exit(slave);
}

module_platform_driver(csi_v4l2_driver);

MODULE_AUTHOR("Ivan Zaitsev");
MODULE_DESCRIPTION("V4L2 capture driver for VBase camera");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
