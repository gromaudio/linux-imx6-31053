/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/opp.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <linux/memblock.h>
#include <asm/setup.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static struct platform_device imx6sl_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6sl_fec_clk_init(void)
{
	struct regmap *gpr;

	/* set FEC clock from internal PLL clock source */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sl-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX2_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX1_SEL_MASK, 0);
	} else
		pr_err("failed to find fsl,imx6sl-iomux-gpr regmap\n");
}

static inline void imx6sl_fec_init(void)
{
	imx6sl_fec_clk_init();
	imx6_enet_mac_init("fsl,imx6sl-fec");
}

static void __init imx6sl_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	imx6sl_fec_init();
	imx_anatop_init();
	imx6_pm_init();
}

static void __init imx6sl_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

put_node:
	of_node_put(np);
}

static void __init imx6sl_init_late(void)
{
	struct regmap *gpr;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sl-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT_MASK,
			IMX6Q_GPR1_GINT_ASSERT);
	else
		pr_err("failed to find fsl,imx6sl-iomux-gpr regmap\n");

	/* Init CPUIDLE */
	imx6sl_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		imx6sl_opp_init(&imx6sl_cpufreq_pdev.dev);
		platform_device_register(&imx6sl_cpufreq_pdev);
	}

}

static void __init imx6sl_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
	imx6_busfreq_map_io();
}

static void __init imx6sl_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init imx6sl_timer_init(void)
{
	of_clk_init(NULL);
}

static const char *imx6sl_dt_compat[] __initdata = {
	"fsl,imx6sl",
	NULL,
};

extern unsigned long int ramoops_phys_addr;
extern unsigned long int ramoops_mem_size;
static void imx6sl_reserve(void)
{
	phys_addr_t phys;
	phys_addr_t max_phys;
	struct meminfo *mi;
	struct membank *bank;

#ifdef CONFIG_PSTORE_RAM
	mi = &meminfo;
	if (!mi) {
		pr_err("no memory reserve for ramoops.\n");
		return;
	}

	/* use memmory last bank for ram console store */
	bank = &mi->bank[mi->nr_banks - 1];
	if (!bank) {
		pr_err("no memory reserve for ramoops.\n");
		return;
	}
	max_phys = bank->start + bank->size;
	/* reserve 64M for uboot avoid ram console data is cleaned by uboot */
	phys = memblock_alloc_base(SZ_1M, SZ_4K, max_phys - SZ_64M);
	if (phys) {
		memblock_remove(phys, SZ_1M);
		memblock_reserve(phys, SZ_1M);
		ramoops_phys_addr = phys;
		ramoops_mem_size = SZ_1M;
	} else {
		ramoops_phys_addr = 0;
		ramoops_mem_size = 0;
		pr_err("no memory reserve for ramoops.\n");
	}
#endif
	return;
}

DT_MACHINE_START(IMX6SL, "Freescale i.MX6 SoloLite (Device Tree)")
	.map_io		= imx6sl_map_io,
	.init_irq	= imx6sl_init_irq,
	.init_time	= imx6sl_timer_init,
	.init_machine	= imx6sl_init_machine,
	.init_late      = imx6sl_init_late,
	.dt_compat	= imx6sl_dt_compat,
	.reserve     = imx6sl_reserve,
	.restart	= mxc_restart,
MACHINE_END
