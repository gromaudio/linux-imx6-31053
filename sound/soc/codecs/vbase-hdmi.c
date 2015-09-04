/*
 * VBase HDMI codec driver
 *
 * Author:      Ivan Zaitsev, <ivan.zaitsev@gmail.com>
 * Copyright:   (C) 2015 X-Media tech, Inc.,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

//-----------------------------------------------------------------------------
static int hdmi_hw_params(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *params,
                          struct snd_soc_dai       *dai)
{
  dev_dbg(dai->codec->dev, "hw_params: rate = %d, format = %08X\n", params_rate(params),
                                                                    params_format(params));
  return 0;
}

static int hdmi_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                               int                 clk_id,
                               unsigned int        freq,
                               int                 dir)
{
  dev_dbg("%s: %d, %d\n", __func__, freq, dir);
  return 0;
}

static int hdmi_set_dai_fmt(struct snd_soc_dai *codec_dai,
                            unsigned int        fmt)
{
  dev_dbg("%s: 0x%08X\n", __func__, fmt);
  return 0;
}

//-----------------------------------------------------------------------------
static int hdmi_codec_soc_probe(struct snd_soc_codec *codec)
{
  dev_dbg(codec->dev, "%s\n", __func__);
  return 0;
}

static int hdmi_codec_soc_remove(struct snd_soc_codec *codec)
{
  dev_dbg(codec->dev, "%s\n", __func__);
  return 0;
}

//-----------------------------------------------------------------------------
static struct snd_soc_codec_driver soc_hdmi_codec_driver = {
  .probe  = hdmi_codec_soc_probe,
  .remove = hdmi_codec_soc_remove,
};

//-----------------------------------------------------------------------------
static struct snd_soc_dai_ops hdmi_dai_ops = {
  .hw_params    = hdmi_hw_params,
  .set_sysclk   = hdmi_set_dai_sysclk,
  .set_fmt      = hdmi_set_dai_fmt,
};

static struct snd_soc_dai_driver soc_hdmi_dai_driver = {
	.name = "hdmi-in",
  .capture = {
    .stream_name  = "Capture",
    .channels_min = 2,
    .channels_max = 2,
    .rates        = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_44100,
    .formats      = SNDRV_PCM_FMTBIT_S16_LE,},
  .ops             = &hdmi_dai_ops,
	.symmetric_rates = 1,
};

//-----------------------------------------------------------------------------
static int hdmi_codec_probe(struct platform_device *pdev)
{
  int ret = 0;

  dev_dbg(&pdev->dev, "%s\n", __func__);

  ret = snd_soc_register_codec(&pdev->dev,
                               &soc_hdmi_codec_driver,
                               &soc_hdmi_dai_driver,
                               1);
  if (ret) {
    dev_err(&pdev->dev, "failed to register codec\n");
    return ret;
  }
  return 0;
}

static int hdmi_codec_remove(struct platform_device *pdev)
{
  dev_dbg(&pdev->dev, "%s\n", __func__);

  snd_soc_unregister_codec(&pdev->dev);
  platform_set_drvdata(pdev, NULL);

  return 0;
}

//-----------------------------------------------------------------------------
static const struct of_device_id hdmi_of_match[] = {
  { .compatible = "gromaudio,vbase-hdmi", },
  {},
};
MODULE_DEVICE_TABLE(of, hdmi_of_match);


struct platform_driver hdmi_codec_driver = {
  .driver = {
       .name  = "vbase-hdmi",
       .owner = THIS_MODULE,
       .of_match_table = of_match_ptr(hdmi_of_match),
       },
  .probe  = hdmi_codec_probe,
  .remove = hdmi_codec_remove,
};

//-----------------------------------------------------------------------------
static int __init hdmi_codec_init(void)
{
  return platform_driver_register(&hdmi_codec_driver);
}

static void __exit hdmi_codec_exit(void)
{
  return platform_driver_unregister(&hdmi_codec_driver);
}

//-----------------------------------------------------------------------------
module_init(hdmi_codec_init);
module_exit(hdmi_codec_exit);

MODULE_AUTHOR("Ivan Zaitsev");
MODULE_DESCRIPTION("VBase HDMI codec driver");
MODULE_LICENSE("GPL");
