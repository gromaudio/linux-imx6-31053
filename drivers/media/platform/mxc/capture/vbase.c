/*
 * Copyright (C) 2014 X-Media tech, Inc. All Rights Reserved.
 *
 * authors: Ivan Zaitsev
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define VBASE_XCLK_MIN        6000000
#define VBASE_XCLK_MAX        13500000

#define GET_NUM_OF_INPUTS     0x01
#define GET_INPUT_LIST        0x02
#define SELECT_INPUT          0x03
#define CAMERA_INPUT_NAME_MAX 10
//#define CUSTOM_PIXELFORMAT    V4L2_PIX_FMT_RGB32


typedef struct
{
  uint8_t   name[ CAMERA_INPUT_NAME_MAX ];
  uint16_t  width;
  uint16_t  height;
  uint8_t   framerate;
  uint8_t   interlace;
  uint32_t  clock;
  uint32_t  pixelformat;
  uint32_t  audiosamplerate;
  uint32_t  reserved[ 10 ];
}__attribute__ ((packed))video_input;

static struct sensor_data   vbase_data;
static video_input         *video_input_array;
static char                 num_of_inputs;

static const struct i2c_device_id vbase_id[] = {
  {"vbase camera", 0},
  {},
};

MODULE_DEVICE_TABLE(i2c, vbase_id);

/*
 * vbase_write_reg - write value into camera register.
 * @reg: register
 * @val: value
 */
static s32 vbase_write_reg(u8 reg, u8 val)
{
  u8 au8Buf[2] = {0};

  au8Buf[0] = reg;
  au8Buf[1] = val;

  if (i2c_master_send(vbase_data.i2c_client, au8Buf, sizeof(au8Buf)) < 0) {
    pr_err("%s:write reg error:reg=%x,val=%x\n",
      __func__, reg, val);
    return -1;
  }
  return 0;
}

/*
 * vbase_read_reg - read value from camera register.
 * @reg: register
 * @val: value
 */
static s32 vbase_read_reg(u8 reg, u8 *val)
{
  u8 u8RdVal = 0;

  if (1 != i2c_master_send(vbase_data.i2c_client, &reg, 1)) {
    pr_err("%s:write reg error:reg=%x\n",
        __func__, reg);
    return -1;
  }

  if (1 != i2c_master_recv(vbase_data.i2c_client, &u8RdVal, 1)) {
    pr_err("%s:read reg error:reg=%x,val=%x\n",
        __func__, reg, u8RdVal);
    return -1;
  }

  *val = u8RdVal;
  return u8RdVal;
}

/*
 * vbase_read_data - read data from camera .
 * @reg: register to start from
 * @data: pointer to destination data buffer
 * @size: data buffer size
 */
static s32 vbase_read_data(u8 reg, u8 *data, u16 size )
{
  if (1 != i2c_master_send(vbase_data.i2c_client, &reg, 1)) {
    pr_err("%s:write reg error:reg=%x\n",
        __func__, reg);
    return -1;
  }

  if (size != i2c_master_recv(vbase_data.i2c_client, data, size)) {
    pr_err("%s:read data error:reg=%x,size=%x\n",
        __func__, reg, size);
    return -1;
  }

  return size;
}

static uint32_t vbase_get_pixelformat(int idx)
{
#if defined( CUSTOM_PIXELFORMAT )
  return CUSTOM_PIXELFORMAT;
#else
  return video_input_array[idx].pixelformat;
#endif
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
  pr_debug("%s\n", __func__);
  return 0;
}

/*
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
  pr_debug("%s\n", __func__);
  return 0;
}

/*
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
  pr_debug("%s\n", __func__);
  return 0;
}

/*
 * ioctl_g_ifparm - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to structure to retorn interface parameters
 *
 * Return interface parameters.
 */
static int ioctl_g_ifparm( struct v4l2_int_device *s,
                           struct v4l2_ifparm *p)
{
  pr_debug("%s\n", __func__);

  memset(p, 0, sizeof(*p));
  p->if_type                 = V4L2_IF_TYPE_BT656;
  p->u.bt656.mode            = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
  p->u.bt656.clock_curr      = VBASE_XCLK_MIN;
  p->u.bt656.clock_min       = VBASE_XCLK_MIN;
  p->u.bt656.clock_max       = VBASE_XCLK_MAX;
  p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
  p->u.bt656.nobt_vs_inv     = 0;
  p->u.bt656.nobt_hs_inv     = 0;
  p->u.bt656.latch_clk_inv   = 0;
  return 0;
}

/*
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
  pr_debug("%s: %d\n", __func__, on);

  return 0;
}

/*
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap( struct v4l2_int_device *s,
                               struct v4l2_fmtdesc *fmt)
{
  pr_debug("%s\n", __func__);

  if( fmt->index < 2 )
  {
    fmt->pixelformat    = vbase_get_pixelformat(fmt->index);
    fmt->description[0] = fmt->pixelformat & 0xFF;
    fmt->description[1] = ( fmt->pixelformat >> 8 ) & 0xFF;
    fmt->description[2] = ( fmt->pixelformat >> 16 ) & 0xFF;
    fmt->description[3] = ( fmt->pixelformat >> 24 ) & 0xFF;
    fmt->description[4] = 0;
    return 0;
  }

  return -EINVAL;
}

/*
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *         VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes( struct v4l2_int_device *s,
                                  struct v4l2_frmsizeenum *fsize)
{
  pr_debug("%s: %d %08X\n", __func__, fsize->index, fsize->pixel_format);

  if( fsize->pixel_format == vbase_get_pixelformat( 0 ) &&
      fsize->index == 0 )
  {
    fsize->discrete.width  = video_input_array[fsize->index].width;
    fsize->discrete.height = video_input_array[fsize->index].height;
    return 0;
  }

  if( fsize->pixel_format == vbase_get_pixelformat( 1 ) &&
      fsize->index < 2 )
  {
    fsize->pixel_format    = vbase_get_pixelformat(fsize->index + 1);
    fsize->discrete.width  = video_input_array[fsize->index + 1].width;
    fsize->discrete.height = video_input_array[fsize->index + 1].height;
    return 0;
  }

  return -EINVAL;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap( struct v4l2_int_device *s,
                            struct v4l2_format *f)
{
  pr_debug("%s\n", __func__);

  f->fmt.pix = vbase_data.pix;
  return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm( struct v4l2_int_device *s,
                         struct v4l2_streamparm *a)
{
  struct sensor_data *sensor     = s->priv;
  struct v4l2_captureparm *cparm = &a->parm.capture;
  int ret = 0;

  pr_debug("%s\n", __func__);

  switch(a->type)
  {
  /* This is the only case currently handled. */
  case V4L2_BUF_TYPE_VIDEO_CAPTURE:
    memset(a, 0, sizeof(*a));
    a->type             = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cparm->capability   = sensor->streamcap.capability;
    cparm->timeperframe = sensor->streamcap.timeperframe;
    cparm->capturemode  = sensor->streamcap.capturemode;
    ret = 0;
    break;

  /* These are all the possible cases. */
  case V4L2_BUF_TYPE_VIDEO_OUTPUT:
  case V4L2_BUF_TYPE_VIDEO_OVERLAY:
  case V4L2_BUF_TYPE_VBI_CAPTURE:
  case V4L2_BUF_TYPE_VBI_OUTPUT:
  case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
  case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
    ret = -EINVAL;
    break;

  default:
    pr_err("   type is unknown - %d\n", a->type);
    ret = -EINVAL;
    break;
  }

  return ret;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm( struct v4l2_int_device *s,
                         struct v4l2_streamparm *a)
{
  struct sensor_data *sensor      = s->priv;
  struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
  int ret = 0;

  pr_debug("%s\n", __func__);

  if(a->parm.capture.capturemode >= num_of_inputs)
    return -EINVAL;

  switch (a->type)
  {
  /* This is the only case currently handled. */
  case V4L2_BUF_TYPE_VIDEO_CAPTURE:
    timeperframe->denominator      = video_input_array[0].framerate;
    timeperframe->numerator        = 1;
    sensor->streamcap.timeperframe = *timeperframe;
    sensor->streamcap.capturemode  = (u32)a->parm.capture.capturemode;
    vbase_data.pix.width           = video_input_array[a->parm.capture.capturemode].width;
    vbase_data.pix.height          = video_input_array[a->parm.capture.capturemode].height;
    vbase_data.pix.pixelformat     = vbase_get_pixelformat(a->parm.capture.capturemode);
    vbase_write_reg(SELECT_INPUT, a->parm.capture.capturemode);
    msleep(100);
    break;

  /* These are all the possible cases. */
  case V4L2_BUF_TYPE_VIDEO_OUTPUT:
  case V4L2_BUF_TYPE_VIDEO_OVERLAY:
  case V4L2_BUF_TYPE_VBI_CAPTURE:
  case V4L2_BUF_TYPE_VBI_OUTPUT:
  case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
  case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
    ret = -EINVAL;
    break;

  default:
    pr_err("   type is unknown - %d\n", a->type);
    ret = -EINVAL;
    break;
  }

  return ret;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl( struct v4l2_int_device *s,
                         struct v4l2_control *vc)
{
  pr_debug("%s\n", __func__);
  return -EINVAL; // No controls supported.
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl( struct v4l2_int_device *s,
                         struct v4l2_control *vc)
{
  pr_debug("%s\n", __func__);
  return -EINVAL;
}

/*
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *      VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
  pr_debug("%s\n", __func__);

  ((struct v4l2_dbg_chip_ident *)id)->ident      = V4L2_IDENT_UNKNOWN;
  ((struct v4l2_dbg_chip_ident *)id)->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
  strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "VBase camera");

  return 0;
}

/*
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc vbase_ioctl_desc[] = {
  {vidioc_int_dev_init_num,         (v4l2_int_ioctl_func *)ioctl_dev_init},
  {vidioc_int_dev_exit_num,         (v4l2_int_ioctl_func *)ioctl_dev_exit},
  {vidioc_int_s_power_num,          (v4l2_int_ioctl_func *)ioctl_s_power},
  {vidioc_int_g_ifparm_num,         (v4l2_int_ioctl_func *)ioctl_g_ifparm},
  {vidioc_int_init_num,             (v4l2_int_ioctl_func *)ioctl_init},
  {vidioc_int_enum_fmt_cap_num,     (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
  {vidioc_int_g_fmt_cap_num,        (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
  {vidioc_int_g_parm_num,           (v4l2_int_ioctl_func *)ioctl_g_parm},
  {vidioc_int_s_parm_num,           (v4l2_int_ioctl_func *)ioctl_s_parm},
  {vidioc_int_g_ctrl_num,           (v4l2_int_ioctl_func *)ioctl_g_ctrl},
  {vidioc_int_s_ctrl_num,           (v4l2_int_ioctl_func *)ioctl_s_ctrl},
  {vidioc_int_enum_framesizes_num,  (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
  {vidioc_int_g_chip_ident_num,     (v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave vbase_slave = {
  .ioctls     = vbase_ioctl_desc,
  .num_ioctls = ARRAY_SIZE( vbase_ioctl_desc ),
};

static struct v4l2_int_device vbase_int_device = {
  .module = THIS_MODULE,
  .name   = "vbase camera",
  .type   = v4l2_int_type_slave,
  .u = {
    .slave = &vbase_slave,
  },
};

/*
 * VBase camera I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int vbase_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
  int     retval;
  struct  device *dev = &client->dev;

  dev_dbg(dev, "%s, I2C device \"%s\", addr 0x%02X\n", __func__, client->name, client->addr);

  memset(&vbase_data, 0, sizeof(vbase_data));
  vbase_data.i2c_client = client;

  retval = of_property_read_u32(dev->of_node, "csi_id", &(vbase_data.csi));
  if(retval)
  {
    dev_err(dev, "csi_id missing or invalid\n");
    return retval;
  }

  if(vbase_read_reg(GET_NUM_OF_INPUTS, &num_of_inputs) < 0)
  {
    dev_err(dev, "%s: camera vbase is not found\n", __func__);
    retval = -ENODEV;
    goto exit;
  }

  video_input_array = kmalloc(num_of_inputs*sizeof(video_input), GFP_KERNEL);
  if(!video_input_array)
  {
    dev_err(dev, "%s: cannot allocate memory\n", __func__);
    retval = -ENOMEM;
    goto exit;
  }

  if(vbase_read_data(GET_INPUT_LIST, (char*)video_input_array, num_of_inputs*sizeof(video_input)) < 0)
  {
    dev_err(dev, "%s: cannot get input list\n", __func__);
    retval = -ENODEV;
    kfree(video_input_array);
    goto exit;
  }

  vbase_data.pix.width                          = video_input_array[0].width;
  vbase_data.pix.height                         = video_input_array[0].height;
  vbase_data.pix.pixelformat                    = vbase_get_pixelformat(0);
  vbase_data.streamcap.capability               = V4L2_CAP_TIMEPERFRAME | V4L2_MODE_HIGHQUALITY;
  vbase_data.streamcap.capturemode              = 0;
  vbase_data.streamcap.timeperframe.denominator = video_input_array[0].framerate;
  vbase_data.streamcap.timeperframe.numerator   = 1;

  vbase_int_device.priv = &vbase_data;
  if((retval=v4l2_int_device_register(&vbase_int_device)))
  {
    retval = -ENODEV;
    kfree(video_input_array);
  }

exit:
  return retval;
}

/*
 * VBase camera I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int vbase_remove(struct i2c_client *client)
{
  pr_debug("%s\n", __func__);

  v4l2_int_device_unregister(&vbase_int_device);
  kfree(video_input_array);
  return 0;
}

static struct i2c_driver vbase_i2c_driver = {
  .driver = {
      .owner = THIS_MODULE,
      .name  = "vbase camera",
      },
  .probe    = vbase_probe,
  .remove   = vbase_remove,
  .id_table = vbase_id,
};

/*
 * VBase camera init function
 * Called by insmod vbase_int.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int vbase_init(void)
{
  u8 err;

  pr_debug("%s\n", __func__);

  err = i2c_add_driver(&vbase_i2c_driver);
  if (err != 0)
    pr_err("%s:driver registration failed, error=%d \n", __func__, err);

  return err;
}

/*
 * VBase cleanup function
 * Called on rmmod vbase_int.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit vbase_clean(void)
{
  pr_debug("%s\n", __func__);

  i2c_del_driver(&vbase_i2c_driver);
}

module_init(vbase_init);
module_exit(vbase_clean);

MODULE_AUTHOR("Ivan Zaitsev");
MODULE_DESCRIPTION("VBase camera driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
