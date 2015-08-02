/*
 * ALSA SoC VBase codec driver
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
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

//-----------------------------------------------------------------------------
// codec private data
struct vbase_priv {
	struct snd_soc_codec 			 *codec;
	enum snd_soc_control_type 	control_type;
};

//-----------------------------------------------------------------------------
static int vbase_hw_params(struct snd_pcm_substream *substream,
												   struct snd_pcm_hw_params *params,
			   									 struct snd_soc_dai *dai)
{
	return 0;
}

static int vbase_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int vbase_set_dai_sysclk(struct snd_soc_dai *codec_dai,
																int clk_id,
																unsigned int freq,
																int dir)
{
	return 0;
}

static int vbase_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     									 unsigned int fmt)
{
	return 0;
}

//-----------------------------------------------------------------------------
static const struct snd_soc_dai_ops vbase_dai_ops = {
	.hw_params    = vbase_hw_params,
	.digital_mute = vbase_mute,
	.set_sysclk   = vbase_set_dai_sysclk,
	.set_fmt      = vbase_set_dai_fmt,
};

//-----------------------------------------------------------------------------
static struct snd_soc_dai_driver vbase_dai = {
	.name = "vbase-hifi",
	.playback = {
		.stream_name 	= "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates 				= SNDRV_PCM_RATE_44100,
		.formats 			= SNDRV_PCM_FMTBIT_S16_LE},
	.capture = {
		.stream_name 	= "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates 				= SNDRV_PCM_RATE_44100,
		.formats 			= SNDRV_PCM_FMTBIT_S16_LE},
	.ops 						 = &vbase_dai_ops,
	.symmetric_rates = 1,
};

static int vbase_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int vbase_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int vbase_probe(struct snd_soc_codec *codec)
{
	struct vbase_priv *vbase = snd_soc_codec_get_drvdata(codec);

	vbase->codec = codec;
	return 0;
}

static int vbase_remove(struct snd_soc_codec *codec)
{
	return 0;
}

//-----------------------------------------------------------------------------
static struct snd_soc_codec_driver soc_codec_dev_vbase = {
	.probe             = vbase_probe,
	.remove            = vbase_remove,
	.suspend           = vbase_suspend,
	.resume            = vbase_resume,
};


static const struct i2c_device_id vbase_i2c_id[] = {
	{ "vbase", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vbase_i2c_id);

//-----------------------------------------------------------------------------
static int vbase_i2c_probe(	struct i2c_client *i2c,
			   										const struct i2c_device_id *id)
{
	struct vbase_priv *vbase;
	int ret;

	dev_err(&i2c->dev, "vbase_i2c_probe: 0x%08x\n", (u32)i2c->dev.of_node);

	vbase = devm_kzalloc(&i2c->dev, sizeof(struct vbase_priv), GFP_KERNEL);
	if (vbase == NULL) {
		dev_err(&i2c->dev, "failed to create private data\n");
		return -ENOMEM;
	}

	vbase->control_type = SND_SOC_I2C;

	i2c_set_clientdata(i2c, vbase);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_vbase, &vbase_dai, 1);
	return ret;
}

static int vbase_i2c_remove(struct i2c_client *client)
{
	dev_err(&client->dev, "vbase_i2c_remove\n");
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

//-----------------------------------------------------------------------------
#if defined(CONFIG_OF)
static const struct of_device_id vbase_of_match[] = {
	{ .compatible = "gromaudio,vbase", },
	{},
};
MODULE_DEVICE_TABLE(of, vbase_of_match);
#endif

/* machine i2c codec control layer */
static struct i2c_driver vbase_i2c_driver = {
	.driver = {
		.name 					= "vbase-codec",
		.owner 					= THIS_MODULE,
		.of_match_table = of_match_ptr(vbase_of_match),
	},
	.probe	  = vbase_i2c_probe,
	.remove   = vbase_i2c_remove,
	.id_table = vbase_i2c_id,
};

module_i2c_driver(vbase_i2c_driver);

MODULE_DESCRIPTION("VBase codec driver");
MODULE_AUTHOR("Ivan Zaitsev");
MODULE_LICENSE("GPL");
