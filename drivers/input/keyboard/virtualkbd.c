/*****************************************************************************
* virtualkbd.c: Virtual input device driver.
*
* No. | Date       | Author       | Description
* ============================================================================
* 1   | 8 Jan 2014 | Ivan Zaitsev | First release.
*
******************************************************************************/
/*
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
#include "virtualkbd.h"

MODULE_AUTHOR("Ivan Zaitsev <ivan.zaitsev@gmail.com>");
MODULE_DESCRIPTION("Virtual input device driver");
MODULE_LICENSE("GPL");

static struct input_dev *virtual_dev;

static int __init virtual_init(void)
{
  int i, error;

  virtual_dev = input_allocate_device();
  if (!virtual_dev)
    return -ENOMEM;

  virtual_dev->name         = "Virtual input device";
  virtual_dev->id.bustype   = BUS_HOST;
  virtual_dev->id.vendor    = 0x0001;
  virtual_dev->id.product   = 0x0001;
  virtual_dev->id.version   = 0x0100;
  virtual_dev->evbit[0]     = BIT_MASK(EV_KEY) |  BIT_MASK(EV_REL) | BIT_MASK(EV_ABS);
  virtual_dev->keycode      = virtual_key_array;
  virtual_dev->keycodesize  = sizeof(unsigned char);
  virtual_dev->keycodemax   = ARRAY_SIZE(virtual_key_array);

  __set_bit(ABS_X, virtual_dev->absbit);
  __set_bit(ABS_Y, virtual_dev->absbit);
  __set_bit(ABS_PRESSURE, virtual_dev->absbit);

  input_set_abs_params(virtual_dev, ABS_X,
            0,
            255,
            0, 0);
  input_set_abs_params(virtual_dev, ABS_Y,
          0,
          255,
          0, 0);
  input_set_abs_params(virtual_dev, ABS_PRESSURE,
          0, 255, 0, 0);

  for (i = 0; i < ARRAY_SIZE(virtual_key_array); i++) {
    set_bit(virtual_key_array[i], virtual_dev->keybit);
  }

  set_bit(EV_REL, virtual_dev->evbit);
  for (i = 0; i < ARRAY_SIZE(virtual_rel_array); i++) {
    set_bit(virtual_rel_array[i], virtual_dev->relbit);
  }

  error = input_register_device(virtual_dev);
  if (error) {
    input_free_device(virtual_dev);
    return error;
  }

  return 0;
}

static void __exit virtual_exit(void)
{
  printk(KERN_ERR "VIRTUAL_KBD_exit\n" );
  input_unregister_device(virtual_dev);
}

module_init(virtual_init);
module_exit(virtual_exit);
