/*
 * sound/soc/imx/imx-vbase.c --  SoC audio for iMX6 VBase board.
 *
 * Authors: Ivan Zaitsev, <ivan.zaitsev@gmail.com>
 *
 * Copyright 2015 X-Media tech, Inc.
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include "imx-audmux.h"

//-------------------------------------------------------------------------------------------------
#define DAI_NAME_SIZE 32

//-------------------------------------------------------------------------------------------------
struct imx_vbase_data {
  struct        snd_soc_dai_link dai;
  struct        snd_soc_card card;
  char          codec_dai_name[DAI_NAME_SIZE];
  char          platform_name[DAI_NAME_SIZE];
  struct clk   *codec_clk;
  unsigned int  clk_frequency;
};

//-------------------------------------------------------------------------------------------------
static int imx_vbase_dai_init(struct snd_soc_pcm_runtime *rtd)
{
  struct imx_vbase_data *data = container_of(rtd->card,
                                                     struct imx_vbase_data,
                                                     card);
  struct device *dev = rtd->card->dev;
  int ret;

  ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0, data->clk_frequency, 0);
  if (ret) {
    dev_err(dev, "could not set codec driver clock params\n");
    return ret;
  }

  return 0;
}

//-------------------------------------------------------------------------------------------------
static int imx_vbase_audmux_config(struct platform_device *pdev)
{
  struct device_node *np = pdev->dev.of_node;
  int int_port,
      ext_port;
  int ret;

  ret = of_property_read_u32(np, "mux-int-port", &int_port);
  if (ret) {
    dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
    return ret;
  }
  ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
  if (ret) {
    dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
    return ret;
  }

  /*
   * The port numbering in the hardware manual starts at 1, while
   * the audmux API expects it starts at 0.
   */
  int_port--;
  ext_port--;
  ret = imx_audmux_v2_configure_port(int_port,
      IMX_AUDMUX_V2_PTCR_SYN |
      IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
      IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
      IMX_AUDMUX_V2_PTCR_TFSDIR |
      IMX_AUDMUX_V2_PTCR_TCLKDIR,
      IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
  if (ret) {
    dev_err(&pdev->dev, "audmux internal port setup failed\n");
    return ret;
  }
  ret = imx_audmux_v2_configure_port(ext_port,
      IMX_AUDMUX_V2_PTCR_SYN,
      IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
  if (ret) {
    dev_err(&pdev->dev, "audmux external port setup failed\n");
    return ret;
  }

  return 0;
}

//-------------------------------------------------------------------------------------------------
static const struct snd_soc_dapm_widget imx_vbase_dapm_widgets[] = {
  SND_SOC_DAPM_HP("Headphone Jack", NULL),
  SND_SOC_DAPM_HP("Line In 1 Jack", NULL),
  SND_SOC_DAPM_HP("Line In 2 Jack", NULL),
};

//-------------------------------------------------------------------------------------------------
static int imx_vbase_probe(struct platform_device *pdev)
{
  struct device_node            *cpu_np,
                                *codec_np;
  struct platform_device        *cpu_pdev;
  struct i2c_client             *codec_dev;
  struct imx_vbase_data         *data;
  int ret;

	dev_err(&pdev->dev, "imx_vbase_probe\n");

  cpu_np   = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
  codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
  if (!cpu_np || !codec_np) {
    dev_err(&pdev->dev, "phandle missing or invalid\n");
    ret = -EINVAL;
    goto fail;
  }

  if (strstr(cpu_np->name, "ssi")) {
    ret = imx_vbase_audmux_config(pdev);
    if (ret)
      goto fail;
  }

  cpu_pdev = of_find_device_by_node(cpu_np);
  if (!cpu_pdev) {
    dev_err(&pdev->dev, "failed to find SSI platform device\n");
    ret = -EINVAL;
    goto fail;
  }
  codec_dev = of_find_i2c_device_by_node(codec_np);
  if (!codec_dev) {
    dev_err(&pdev->dev, "failed to find codec platform device\n");
    return -EINVAL;
  }

  data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data) {
    ret = -ENOMEM;
    goto fail;
  }

  data->codec_clk = clk_get(&codec_dev->dev, NULL);
  if (IS_ERR(data->codec_clk)) {
    /* assuming clock enabled by default */
    data->codec_clk = NULL;
    ret = of_property_read_u32(codec_np, "clock-frequency", &data->clk_frequency);
    if (ret) {
      dev_err(&codec_dev->dev, "clock-frequency missing or invalid\n");
      goto fail;
    }
  } else {
    data->clk_frequency = clk_get_rate(data->codec_clk);
    clk_prepare_enable(data->codec_clk);
  }

  data->dai.name              = "HiFi";
  data->dai.stream_name       = "HiFi";
  data->dai.codec_dai_name    = "vbase-hifi";
  data->dai.codec_of_node     = codec_np;
  data->dai.cpu_of_node       = cpu_np;
  data->dai.platform_of_node  = cpu_np;
  data->dai.init              = &imx_vbase_dai_init;
  data->dai.dai_fmt           = SND_SOC_DAIFMT_I2S;
  data->card.dev              = &pdev->dev;

  ret = snd_soc_of_parse_card_name(&data->card, "model");
  if (ret)
    goto clk_fail;

  data->card.num_links        = 1;
  data->card.owner            = THIS_MODULE;
  data->card.dai_link         = &data->dai;

  ret = snd_soc_register_card(&data->card);
  if (ret) {
    dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
    goto clk_fail;
  }

  platform_set_drvdata(pdev, data);
clk_fail:
  clk_put(data->codec_clk);
fail:
  if (cpu_np)
    of_node_put(cpu_np);
  if (codec_np)
    of_node_put(codec_np);

  return ret;
}

static int imx_vbase_remove(struct platform_device *pdev)
{
  struct imx_vbase_data *data = platform_get_drvdata(pdev);

  dev_err(&pdev->dev, "imx_vbase_remove\n");

  if (data->codec_clk) {
    clk_disable_unprepare(data->codec_clk);
    clk_put(data->codec_clk);
  }
  snd_soc_unregister_card(&data->card);

	return 0;
}

//-------------------------------------------------------------------------------------------------
static const struct of_device_id imx_vbase_dt_ids[] = {
  { .compatible = "fsl,imx-audio-vbase", },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_vbase_dt_ids);

static struct platform_driver imx_vbase_audio_driver = {
  .driver = {
    .name           = "imx-vbase",
    .owner          = THIS_MODULE,
    .of_match_table = imx_vbase_dt_ids,
  },
  .probe  = imx_vbase_probe,
  .remove = imx_vbase_remove,
};
module_platform_driver(imx_vbase_audio_driver);

MODULE_AUTHOR("Ivan Zaitsev <ivan.zaitsev@gmail.com>");
MODULE_DESCRIPTION("iMX6 VBase ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-vbase");
