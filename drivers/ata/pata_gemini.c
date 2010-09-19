/*
 *  Support for Gemini PATA
 *
 *  Copyright (C) 2009 Janos Laube <janos.dev@gmail.com>
 *  Copyright (C) 2010 Frederic Pecourt <opengemini@free.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* Values of IOMUX
 * 26:24 bits is "IDE IO Select"
 * 111:100 - Reserved
 * 011 - ata0 <-> sata0, sata1; bring out ata1
 * 010 - ata1 <-> sata1, sata0; bring out ata0
 * 001 - ata0 <-> sata0, ata1 <-> sata1; bring out ata1
 * 000 - ata0 <-> sata0, ata1 <-> sata1; bring out ata0
 *
 */

#include <linux/platform_device.h>
#include <linux/libata.h>
#include <linux/leds.h>

#include <mach/hardware.h>
#include <mach/global_reg.h>

#define DRV_NAME		"pata_gemini"
#define DRV_VERSION		"0.11"

#define GEMINI_PATA_PORTS	1
#define PIO_TIMING_REG		(ap->ioaddr.bmdma_addr + 0x10)
#define MDMA_TIMING_REG		(ap->ioaddr.bmdma_addr + 0x11)
#define UDMA_TIMING0_REG	(ap->ioaddr.bmdma_addr + 0x12)
#define UDMA_TIMING1_REG	(ap->ioaddr.bmdma_addr + 0x13)
#define CLK_MOD_REG		(ap->ioaddr.bmdma_addr + 0x14)

static unsigned char PIO_TIMING[5] = {
	0xaa, 0xa3, 0xa1, 0x33, 0x31
};

static unsigned char TIMING_UDMA_50M[6] = {
	0x33, 0x31, 0x21, 0x21, 0x11, 0x91
};

static unsigned char TIMING_UDMA_66M[7] = {
	0x44, 0x42, 0x31, 0x21, 0x11, 0x91, 0x91
};

static struct scsi_host_template gemini_pata_sht = {
	ATA_NCQ_SHT(DRV_NAME),
	.can_queue	= 1,
	.sg_tablesize	= 128,
	.dma_boundary	= 0xffffU,
};

static void gemini_set_dmamode(struct ata_port *ap, struct ata_device *adev)
{
	unsigned int udma	= adev->dma_mode;
	u8 speed		= udma;
	unsigned int drive_dn	= adev->devno;
	u8 reg;

	reg = ioread8(CLK_MOD_REG);
	reg |= (1 << (4 + (drive_dn & 0x01)));
	iowrite8(reg, CLK_MOD_REG);

	if ((speed == XFER_UDMA_6) || (speed == XFER_UDMA_3)
		|| (speed == XFER_UDMA_4) || (speed & XFER_MW_DMA_0))
	{
		reg = ioread8(CLK_MOD_REG);
		reg |= (1 << (drive_dn & 0x01));
		iowrite8(reg, CLK_MOD_REG);
		iowrite8(TIMING_UDMA_66M[speed & ~XFER_UDMA_0],
			UDMA_TIMING0_REG);
	}
	else
	{
		reg = ioread8(CLK_MOD_REG);
		reg &= ~(1 << (drive_dn & 0x01));
		iowrite8(reg, CLK_MOD_REG);
		iowrite8(TIMING_UDMA_50M[speed & ~XFER_UDMA_0],
			UDMA_TIMING0_REG);
	}
	return;
}

static void gemini_set_piomode(struct ata_port *ap, struct ata_device *adev)
{
	unsigned int pio = adev->pio_mode - XFER_PIO_0;
	iowrite8(PIO_TIMING[pio], PIO_TIMING_REG);
}

unsigned int gemini_qc_issue(struct ata_queued_cmd *qc)
{
	ledtrig_ide_activity();
	return ata_sff_qc_issue(qc);
}

static struct ata_port_operations gemini_pata_port_ops = {
	.inherits	= &ata_bmdma_port_ops,
	.set_dmamode	= gemini_set_dmamode,
	.set_piomode	= gemini_set_piomode,
	.qc_issue	= gemini_qc_issue,
};

static struct ata_port_info gemini_pata_portinfo = {
	.flags		= ATA_FLAG_NO_LEGACY | ATA_FLAG_MMIO | ATA_FLAG_SLAVE_POSS,
	.udma_mask= ATA_UDMA7,
	.pio_mask	= ATA_PIO4,
	.port_ops	= &gemini_pata_port_ops,
};

static irqreturn_t gemini_pata_interrupt(int irq, void *dev)
{
	struct ata_host *host = dev;
	unsigned int i, handled = 0;

	spin_lock_irq(&host->lock);
	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];
		struct ata_queued_cmd *qc;
		qc = ata_qc_from_tag(ap, ap->link.active_tag);

		if (qc && (qc->tf.ctl & ATA_NIEN))
			ap->ops->sff_check_status(ap);
		else if (qc && (!(qc->tf.flags & ATA_TFLAG_POLLING)) &&
				(qc->flags & ATA_QCFLAG_ACTIVE))
			handled |= ata_sff_host_intr(ap, qc);
		else
			ap->ops->sff_check_status(ap);
	}
	spin_unlock_irq(&host->lock);

	return IRQ_RETVAL(handled);
}

static int gemini_pata_platform_probe(struct platform_device *pdev)
{
	struct ata_host *host;
	struct resource *res;
	const struct ata_port_info *ppi[] = {&gemini_pata_portinfo, 0};
	unsigned int irq, i;
	void __iomem *mmio_base;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		return -ENODEV;
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mmio_base = devm_ioremap(&pdev->dev, res->start,
		res->end - res->start + 1);

	printk("pata_gemini: configuring port with irq %d, base %p\n",
			irq, mmio_base);

	host = ata_host_alloc_pinfo(&pdev->dev, ppi, GEMINI_PATA_PORTS);
	if (!host)
		return -ENOMEM;

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];
		struct ata_ioports *ioaddr = &ap->ioaddr;

		ioaddr->bmdma_addr		= mmio_base;
		ioaddr->cmd_addr		= mmio_base + 0x20;
		ioaddr->altstatus_addr	=
		ioaddr->ctl_addr		= mmio_base + 0x36;
		ata_sff_std_ports(ioaddr);
		host->ports[i]->cbl = ATA_CBL_SATA;
	}
	return ata_host_activate(host, irq, gemini_pata_interrupt,
		IRQF_SHARED, &gemini_pata_sht);
}

static int gemini_pata_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ata_host *host = dev_get_drvdata(dev);
	ata_host_detach(host);
	return 0;
}

static struct platform_driver gemini_pata_driver = {
	.probe		= gemini_pata_platform_probe,
	.remove		= gemini_pata_platform_remove,
	.driver = {
		.name = DRV_NAME,
	}
};

static int __init gemini_pata_module_init(void)
{
	unsigned int reg;
	u8 phy_status;
	unsigned long timeout = jiffies + (HZ * 1);

	/* iomux 2, slave mode */
	reg = readl(IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);
	reg &= ~0x07000000;
	reg |= 0x02000012;
//	reg |= 0x02000010;
  writel(reg, IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_MISC_CTRL);

	/* enabling ports for presence detection */
	writel(0x00000003, IO_ADDRESS(GEMINI_SATA_BASE) + 0x18);
	writel(0x00000011, IO_ADDRESS(GEMINI_SATA_BASE) + 0x1c);

  	/* disabling port if no drive is present */
	do
	{
		msleep(100);
		phy_status = readb(IO_ADDRESS(GEMINI_SATA_BASE) + 0x08);
		if (phy_status & 0x01) break;
	} while (time_before(jiffies, timeout));
	if (!(phy_status & 0x01))
		writel(0x00, IO_ADDRESS(GEMINI_SATA_BASE) + 0x18);

	do
	{
		msleep(100);
		phy_status = readb(IO_ADDRESS(GEMINI_SATA_BASE) + 0x0C);
		if (phy_status & 0x01) break;
	} while (time_before(jiffies, timeout));
	if (!(phy_status & 0x01))
		writel(0x00, IO_ADDRESS(GEMINI_SATA_BASE) + 0x1C);

	return platform_driver_register(&gemini_pata_driver);
}

static void __exit gemini_pata_module_exit(void)
{
	platform_driver_unregister(&gemini_pata_driver);
}

module_init(gemini_pata_module_init);
module_exit(gemini_pata_module_exit);

MODULE_AUTHOR("Janos Laube <janos.dev@gmail.com>");
MODULE_DESCRIPTION("Parallel ATA driver for Gemini SoC's");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:" DRV_NAME);
