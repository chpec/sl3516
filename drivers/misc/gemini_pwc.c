/*
 *  Gemini Power Control driver
 *
 *  Copyright (C) 2009 Janos Laube <janos.dev@gmail.com>
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

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/hardware.h>

#define DRV_VERSION		0.1

#define GEMINI_PWC_ID		0x00010500
#define	GEMINI_PWC_IDREG	0x00
#define	GEMINI_PWC_CTRLREG	0x04
#define	GEMINI_PWC_STATREG	0x08
#define	GEMINI_POWERBUTTON	0x40

struct gemini_pwc
{
	void __iomem*	pwc_base;
	int		pwc_irq;
};

static spinlock_t pwc_lock;
/* work around bad designed hardware, the IB-4220-B powerbutton seems not
 * to be debounced very well
 */
static int pwc_poweroff_issued = 0;

static void gemini_power_off(void)
{
	unsigned int reg;
	printk("gemini_pwc: power off\n");
	reg = readl(IO_ADDRESS(GEMINI_POWER_CTRL_BASE) + GEMINI_PWC_CTRLREG);
	writel(reg | BIT(2) | BIT(1),
		IO_ADDRESS(GEMINI_POWER_CTRL_BASE) + GEMINI_PWC_CTRLREG);
	reg &= ~BIT(1);
	reg |= BIT(0);
	writel(reg | BIT(2),
		IO_ADDRESS(GEMINI_POWER_CTRL_BASE) + GEMINI_PWC_CTRLREG);
}

static irqreturn_t gemini_pwc_interrupt(int irq, void* dev)
{
	unsigned int reg, src;

	spin_lock_irq(&pwc_lock);
	/* clear pwc interrupt */
	writel(readl(IO_ADDRESS(GEMINI_POWER_CTRL_BASE) + GEMINI_PWC_CTRLREG)
		| (1 << 2), IO_ADDRESS(GEMINI_POWER_CTRL_BASE) +
		GEMINI_PWC_CTRLREG);
	reg = readl(IO_ADDRESS(GEMINI_INTERRUPT_BASE) + 0x08);
	reg |= (1 << IRQ_PWR);
	writel(reg, IO_ADDRESS(GEMINI_INTERRUPT_BASE) + 0x08);
	barrier();

	src = readl(IO_ADDRESS(GEMINI_POWER_CTRL_BASE) +
		GEMINI_PWC_STATREG) & 0x70;
	if ((src == GEMINI_POWERBUTTON) && (!pwc_poweroff_issued))
	{
		printk("gemini_pwc: shutting down machine\n");
		orderly_poweroff(1);
		pwc_poweroff_issued = 1;
	}
	spin_unlock_irq(&pwc_lock);
	return IRQ_HANDLED;
}

static int __devinit gemini_pwc_probe(struct platform_device *pdev)
{
	struct gemini_pwc *pwc;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;
	unsigned int reg;

	pwc = kzalloc(sizeof(struct gemini_pwc), GFP_KERNEL);
	if (unlikely(!pwc))
		return -ENOMEM;
	platform_set_drvdata(pdev, pwc);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
	{
		ret = -ENODEV;
		goto err;
	}
	pwc->pwc_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
	{
		ret = -ENODEV;
		goto err;
	}
	pwc->pwc_base =	devm_ioremap(&pdev->dev, res->start,
		res->end - res->start + 1);

	reg = readl(pwc->pwc_base + GEMINI_PWC_IDREG);
	reg = reg & 0xFFFFFF00;
	if (reg != GEMINI_PWC_ID)
	{
		ret = -ENODEV;
		goto wrongid;
	}

	pm_power_off = gemini_power_off;

	/* clear pwc interrupt */
	writel(readl(pwc->pwc_base + GEMINI_PWC_CTRLREG)
		| (1 << 2), pwc->pwc_base + GEMINI_PWC_CTRLREG);
	reg = readl(IO_ADDRESS(GEMINI_INTERRUPT_BASE)+0x08);
	reg |= (1 << IRQ_PWR);
	writel(reg, IO_ADDRESS(GEMINI_INTERRUPT_BASE)+0x08);
	barrier();
	mdelay(1);

	ret = request_irq(pwc->pwc_irq, gemini_pwc_interrupt, IRQF_DISABLED,
		pdev->name, dev);
	if (unlikely(ret))
		goto err;

	/* enable pwc device */
	writel(readl(pwc->pwc_base + GEMINI_PWC_CTRLREG)
			| (1 << 1), pwc->pwc_base + GEMINI_PWC_CTRLREG);

	return 0;

wrongid:
	printk("gemini_pwc: wrong PWC id\n");
err:
	kfree(pwc);
	return ret;
}

static int __devexit gemini_pwc_remove(struct platform_device *pdev)
{
	struct gemini_pwc *pwc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_power_off = 0;
	free_irq(pwc->pwc_irq, dev);
	platform_set_drvdata(pdev, NULL);
	kfree(pwc);

	return 0;
}

static struct platform_driver gemini_pwc_driver = {
	.driver		= {
		.name	= "gemini_pwc",
		.owner	= THIS_MODULE,
	},
	.probe		= gemini_pwc_probe,
	.remove		= __devexit_p(gemini_pwc_remove),
};

static int __init gemini_pwc_init(void)
{
	return platform_driver_register(&gemini_pwc_driver);
}

static void __exit gemini_pwc_exit(void)
{
	platform_driver_unregister(&gemini_pwc_driver);
}

module_init(gemini_pwc_init);
module_exit(gemini_pwc_exit);

MODULE_AUTHOR("Janos Laube <janos.dev@gmail.com>");
MODULE_ALIAS("platform:gemini_pwc");
MODULE_DESCRIPTION("Driver for the Gemini Power Control device");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
