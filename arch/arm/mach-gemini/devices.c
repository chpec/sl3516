/*
 * Common devices definition for Gemini
 *
 * Copyright (C) 2008-2009 Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/mtd/physmap.h>
#include <linux/skbuff.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/global_reg.h>
#include <mach/gmac.h>

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (void *)IO_ADDRESS(GEMINI_UART_BASE),
		.mapbase	= GEMINI_UART_BASE,
		.irq		= IRQ_UART,
		.uartclk	= UART_CLK,
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.type		= PORT_16550A,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_FIXED_TYPE,
	},
	{},
};

static struct platform_device serial_device = {
	.name	= "serial8250",
	.id	= PLAT8250_DEV_PLATFORM,
	.dev	= {
		.platform_data = serial_platform_data,
	},
};

int platform_register_uart(void)
{
	return platform_device_register(&serial_device);
}

static struct resource usb0_resources[] = {
	{
		.start	= 0x68000000,
		.end	= 0x68000fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_USB0,
		.end	= IRQ_USB0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource usb1_resources[] = {
	{
		.start	= 0x69000000,
		.end	= 0x69000fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_USB1,
		.end	= IRQ_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 usb0_dmamask = 0xffffffffUL;

static u64 usb1_dmamask = 0xffffffffUL;

static struct platform_device usb_device[] = {
	{
		.name	= "ehci-fotg2xx",
		.id	= 0,
		.dev	= {
			.dma_mask = &usb0_dmamask,
			.coherent_dma_mask = 0xffffffff,
		},
		.num_resources	= ARRAY_SIZE(usb0_resources),
		.resource	= usb0_resources,
	},
	{
		.name	= "ehci-fotg2xx",
		.id	= 1,
		.dev	= {
			.dma_mask = &usb1_dmamask,
			.coherent_dma_mask = 0xffffffff,
		},
		.num_resources	= ARRAY_SIZE(usb1_resources),
		.resource	= usb1_resources,
	},
};

int platform_register_usb(unsigned int i)
{
	if (i > 1)
		return -EINVAL;

	return platform_device_register(&usb_device[i]);
}

static struct resource gmac_resources[] = {
	{
		.start	= 0x60000000,
		.end	= 0x6000ffff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_GMAC0,
		.end	= IRQ_GMAC0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_GMAC1,
		.end	= IRQ_GMAC1,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 gmac_dmamask = 0xffffffffUL;

static struct platform_device ethernet_device = {
	.name	= "gemini-gmac",
	.id	= 0,
	.dev	= {
		.dma_mask = &gmac_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(gmac_resources),
	.resource	= gmac_resources,
};

int platform_register_ethernet(struct gemini_gmac_platform_data *pdata)
{
	unsigned int reg = __raw_readl(IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);

	reg &= ~(GMAC_GMII | GMAC_1_ENABLE);

	if (pdata->bus_id[1])
		reg |= GMAC_1_ENABLE;
	else if (pdata->interface[0] == PHY_INTERFACE_MODE_GMII)
		reg |= GMAC_GMII;

	__raw_writel(reg, IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);

	ethernet_device.dev.platform_data = pdata;

	return platform_device_register(&ethernet_device);
}

static struct resource flash_resource = {
	.start	= GEMINI_FLASH_BASE,
	.flags	= IORESOURCE_MEM,
};

static struct physmap_flash_data pflash_platform_data = {};

static struct platform_device pflash_device = {
	.name	= "physmap-flash",
	.id	= 0,
	.dev 	= {
		.platform_data = &pflash_platform_data,
	},
	.resource = &flash_resource,
	.num_resources = 1,
};

int platform_register_pflash(unsigned int size, struct mtd_partition *parts,
			     unsigned int nr_parts)
{
	unsigned int reg;

	reg = __raw_readl(IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_STATUS);

	if ((reg & FLASH_TYPE_MASK) != FLASH_TYPE_PARALLEL)
		return -ENXIO;

	if (reg & FLASH_WIDTH_16BIT)
		pflash_platform_data.width = 2;
	else
		pflash_platform_data.width = 1;

	/* enable parallel flash pins and disable others */
	reg = __raw_readl(IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);
	reg &= ~PFLASH_PADS_DISABLE;
	reg |= SFLASH_PADS_DISABLE | NAND_PADS_DISABLE;
	__raw_writel(reg, IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);

	flash_resource.end = flash_resource.start + size - 1;

	pflash_platform_data.parts = parts;
	pflash_platform_data.nr_parts = nr_parts;

	return platform_device_register(&pflash_device);
}

static struct resource wdt_resource = {
	.start	= GEMINI_WAQTCHDOG_BASE,
	.end	= GEMINI_WAQTCHDOG_BASE + 0x18,
	.flags  = IORESOURCE_MEM,
};

static struct platform_device wdt_device = {
	.name		= "gemini-wdt",
	.id		= 0,
	.resource	= &wdt_resource,
	.num_resources	= 1,
};

int platform_register_watchdog(void)
{
	return platform_device_register(&wdt_device);
}

/*
* RTC
*/
static struct resource gemini_rtc_resources[] = {
 [0] = {
   .start  = GEMINI_RTC_BASE,
   .end    = GEMINI_RTC_BASE + 0x24,
   .flags  = IORESOURCE_MEM,
 },
 [1] = {
   .start  = IRQ_RTC,
   .end    = IRQ_RTC,
   .flags  = IORESOURCE_IRQ,
 },
};

static struct platform_device gemini_rtc_device = {
 .name   = "rtc-gemini",
 .id   = 0,
 .num_resources  = ARRAY_SIZE(gemini_rtc_resources),
 .resource = gemini_rtc_resources,
};

int __init platform_register_rtc(void)
{
 return platform_device_register(&gemini_rtc_device);
}

/*
* PWC
*/

static struct resource gemini_pwc_resources[] = {
 [0] = {
   .start  = GEMINI_POWER_CTRL_BASE,
   .end    = GEMINI_POWER_CTRL_BASE + 0x0C,
   .flags  = IORESOURCE_MEM,
 },
 [1] = {
   .start  = IRQ_PWR,
   .end    = IRQ_PWR,
   .flags  = IORESOURCE_IRQ,
 },
};

static struct platform_device gemini_pwc_device = {
 .name   = "gemini_pwc",
 .id   = 0,
 .num_resources  = ARRAY_SIZE(gemini_pwc_resources),
 .resource = gemini_pwc_resources,
};

int __init platform_register_pwc(void)
{
 return platform_device_register(&gemini_pwc_device);
}


/*
* PATA
*/

static u64 gemini_pata_dmamask0 = 0xffffffffUL;
static u64 gemini_pata_dmamask1 = 0xffffffffUL;

static struct resource gemini_pata_resources0[] = {
 [0] = {
   .start  = GEMINI_IDE0_BASE,
   .end    = GEMINI_IDE0_BASE + 0x40,
   .flags  = IORESOURCE_MEM,
 },
 [1] = {
   .start  = IRQ_IDE0,
   .end    = IRQ_IDE0,
   .flags  = IORESOURCE_IRQ,
 },
};

static struct resource gemini_pata_resources1[] = {
 [0] = {
   .start  = GEMINI_IDE1_BASE,
   .end    = GEMINI_IDE1_BASE + 0x40,
   .flags  = IORESOURCE_MEM,
 },
 [1] = {
   .start  = IRQ_IDE1,
   .end    = IRQ_IDE1,
   .flags  = IORESOURCE_IRQ,
 },
};

static struct platform_device gemini_pata_devices[] = {
 {
   .name   = "pata_gemini",
   .id   = 0,
   .dev    = {
     .dma_mask = &gemini_pata_dmamask0,
     .coherent_dma_mask = 0xffffffff,
   },
   .num_resources  = ARRAY_SIZE(gemini_pata_resources0),
   .resource       = gemini_pata_resources0,
 },
 {
   .name   = "pata_gemini",
   .id   = 1,
   .dev    = {
     .dma_mask = &gemini_pata_dmamask1,
     .coherent_dma_mask = 0xffffffff,
   },
   .num_resources  = ARRAY_SIZE(gemini_pata_resources1),
   .resource       = gemini_pata_resources1,
 },
};

int __init platform_register_pata(unsigned int i)
{
 switch (i) {
 case 0:
   return platform_device_register(&gemini_pata_devices[0]);
 case 1:
   return platform_device_register(&gemini_pata_devices[1]);
 default:
   return -EINVAL;
 }
}

