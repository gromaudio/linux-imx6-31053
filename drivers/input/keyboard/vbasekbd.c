/*****************************************************************************
* vbasekbd.c: VBase input device driver.
*
* Authors: Ivan Zaitsev <ivan.zaitsev@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include "vbasekbd.h"

MODULE_AUTHOR("Ivan Zaitsev <ivan.zaitsev@gmail.com>");
MODULE_DESCRIPTION("VBase input device driver");
MODULE_LICENSE("GPL");

static struct input_dev *vbase_android_dev;
static struct input_dev *vbase_native_dev;

static void init_input_device_struct(struct input_dev *dev)
{
  int i;

  dev->id.bustype   = BUS_HOST;
  dev->id.vendor    = 0x0001;
  dev->id.product   = 0x0001;
  dev->id.version   = 0x0100;
  dev->evbit[0]     = BIT_MASK(EV_KEY) |  BIT_MASK(EV_REL) | BIT_MASK(EV_ABS);
  dev->keycode      = vbase_key_array;
  dev->keycodesize  = sizeof(unsigned char);
  dev->keycodemax   = ARRAY_SIZE(vbase_key_array);

  __set_bit(ABS_X, dev->absbit);
  __set_bit(ABS_Y, dev->absbit);
  __set_bit(ABS_PRESSURE, dev->absbit);

  input_set_abs_params(dev, ABS_X, 0, 255, 0, 0);
  input_set_abs_params(dev, ABS_Y, 0, 255, 0, 0);
  input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);

  for (i = 0; i < ARRAY_SIZE(vbase_key_array); i++) {
    set_bit(vbase_key_array[i], dev->keybit);
  }

  set_bit(EV_REL, dev->evbit);
  for (i = 0; i < ARRAY_SIZE(vbase_rel_array); i++) {
    set_bit(vbase_rel_array[i], dev->relbit);
  }
}

static int __init vbase_init(void)
{
  int error;

  vbase_android_dev = input_allocate_device();
  vbase_native_dev  = input_allocate_device();
  if (!vbase_android_dev || !vbase_native_dev)
  {
    input_free_device(vbase_android_dev);
    input_free_device(vbase_native_dev);
    return -ENOMEM;
  }

  init_input_device_struct(vbase_android_dev);
  vbase_android_dev->name = "vbase input android";
  error = input_register_device(vbase_android_dev);

  init_input_device_struct(vbase_native_dev);
  vbase_native_dev->name = "vbase input native";
  error = input_register_device(vbase_native_dev);

  if (error) {
    input_free_device(vbase_android_dev);
    input_free_device(vbase_native_dev);
    return error;
  }

  return 0;
}

static void __exit vbase_exit(void)
{
  input_unregister_device(vbase_android_dev);
  input_unregister_device(vbase_native_dev);
  input_free_device(vbase_android_dev);
  input_free_device(vbase_native_dev);
}

module_init(vbase_init);
module_exit(vbase_exit);
