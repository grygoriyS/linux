/*
 * TI DaVinci GPIO Support
 *
 * Copyright (c) 2006-2007 David Brownell
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/platform_data/gpio-davinci.h>

struct davinci_gpio_regs {
	u32	dir;
	u32	out_data;
	u32	set_data;
	u32	clr_data;
	u32	in_data;
	u32	set_rising;
	u32	clr_rising;
	u32	set_falling;
	u32	clr_falling;
	u32	intstat;
};

struct davinci_gpio_controller {
	void __iomem	*reg_base;
};

struct davinci_gpio_bank {
	struct gpio_chip	chip;
	struct device		*dev;
	struct davinci_gpio_regs *regs;
	void __iomem		*reg_base;
	u32			gpio_unbanked;
	unsigned		gpio_irq;
	int			irq_base;
};

#define MAX_GPIO_PER_BANK 32
#define BINTEN_REG	0x8 /* GPIO Interrupt Per-Bank Enable Register */

#define chip2bank(chip)	\
	container_of(chip, struct davinci_gpio_bank, chip)

static struct davinci_gpio_regs davinci_gpio_hw_regs = {
	.dir		= 0x00,
	.out_data	= 0x04,
	.set_data	= 0x08,
	.clr_data	= 0x0c,
	.in_data	= 0x10,
	.set_rising	= 0x14,
	.clr_rising	= 0x18,
	.set_falling	= 0x1c,
	.clr_falling	= 0x20,
	.intstat	= 0x24,
};

/*--------------------------------------------------------------------------*/

/* board setup code *MUST* setup pinmux and enable the GPIO clock. */
static inline int __davinci_direction(struct gpio_chip *chip,
			unsigned offset, bool out, int value)
{
	struct davinci_gpio_bank *bank = chip2bank(chip);
	void __iomem *g = bank->reg_base;
	struct davinci_gpio_regs *regs = bank->regs;
	u32 temp;
	u32 mask = 1 << offset;

	temp = readl(g + regs->dir);
	if (out) {
		temp &= ~mask;
		writel(mask, value ? g + regs->set_data : g + regs->clr_data);
	} else {
		temp |= mask;
	}
	writel(temp, g + regs->dir);

	return 0;
}

static int davinci_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return __davinci_direction(chip, offset, false, 0);
}

static int
davinci_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	return __davinci_direction(chip, offset, true, value);
}

/*
 * Read the pin's value (works even if it's set up as output);
 * returns zero/nonzero.
 *
 * Note that changes are synched to the GPIO clock, so reading values back
 * right after you've set them may give old values.
 */
static int davinci_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct davinci_gpio_bank *bank = chip2bank(chip);
	void __iomem *g = bank->reg_base;

	return (1 << offset) & readl(g + bank->regs->in_data);
}

/*
 * Assuming the pin is muxed as a gpio output, set its output value.
 */
static void
davinci_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct davinci_gpio_bank *bank = chip2bank(chip);
	void __iomem *g = bank->reg_base;

	g += value ? bank->regs->set_data : bank->regs->clr_data;
	writel((1 << offset), g);
}

static void davinci_gpio_ctrl_irqen(struct device *dev, u32 bank)
{
	u32 val;
	struct davinci_gpio_controller *chipc = dev_get_drvdata(dev);

	/*
	 * BINTEN -- per-bank interrupt enable. genirq would also let these
	 * bits be set/cleared dynamically.
	 */
	val = readl(chipc->reg_base + BINTEN_REG);

	val |= 0x3 << (bank << 1);

	writel(val, chipc->reg_base + BINTEN_REG);
}

static int davinci_gpio_unbanked_irq_init(struct davinci_gpio_bank *chipb);
static int davinci_gpio_banked_irq_init(struct davinci_gpio_bank *chipb);

static int davinci_gpio_bank_probe(struct platform_device *pdev)
{
	struct davinci_gpio_bank *chipb;
	struct davinci_gpio_bank_pdata *bank_pdata;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;
	/*
	 * Calculate GPIO base continuously according to GPIO banks
	 * registration in sequence
	 */
	static int base;

	bank_pdata = dev_get_platdata(dev);
	if (!bank_pdata) {
		dev_err(dev, "No platform data found\n");
		return -EINVAL;
	}

	chipb = devm_kzalloc(dev, sizeof(*chipb), GFP_KERNEL);
	if (!chipb) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chipb->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(chipb->reg_base))
		return PTR_ERR(chipb->reg_base);

	chipb->dev = dev;
	chipb->regs = &davinci_gpio_hw_regs;
	chipb->gpio_unbanked = max_t(u32, bank_pdata->gpio_unbanked,
				     MAX_GPIO_PER_BANK);

	chipb->chip.label = "DaVinci";
	chipb->chip.direction_input = davinci_direction_in;
	chipb->chip.get = davinci_gpio_get;
	chipb->chip.direction_output = davinci_direction_out;
	chipb->chip.set = davinci_gpio_set;
	chipb->chip.ngpio = min_t(u32, bank_pdata->width, MAX_GPIO_PER_BANK);
	chipb->chip.base = base;

	base += chipb->chip.ngpio;

	platform_set_drvdata(pdev, chipb);

	ret = gpiochip_add(&chipb->chip);
	if (ret)
		return ret;

	if (chipb->gpio_unbanked)
		davinci_gpio_unbanked_irq_init(chipb);
	else
		davinci_gpio_banked_irq_init(chipb);

	dev_info(dev, "GPIO bank registered ngpio(%u)\n", chipb->chip.ngpio);

	return 0;
}

static int davinci_gpio_probe(struct platform_device *pdev)
{
	struct davinci_gpio_controller *chipc;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk	*clk;

	chipc = devm_kzalloc(dev, sizeof(*chipc), GFP_KERNEL);
	if (!chipc) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chipc->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(chipc->reg_base))
		return PTR_ERR(chipc->reg_base);

	clk = devm_clk_get(dev, "gpio");
	if (IS_ERR(clk)) {
		dev_err(dev, "Error %ld getting gpio clock?\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}
	clk_prepare_enable(clk);

	platform_set_drvdata(pdev, chipc);

	dev_info(dev, "GPIO controller registered\n");

	return 0;
}

/*--------------------------------------------------------------------------*/
/*
 * We expect irqs will normally be set up as input pins, but they can also be
 * used as output pins ... which is convenient for testing.
 *
 * NOTE:  The first few GPIOs also have direct INTC hookups in addition
 * to their GPIOBNK0 irq, with a bit less overhead.
 *
 * All those INTC hookups (direct, plus several IRQ banks) can also
 * serve as EDMA event triggers.
 */

static void gpio_irq_disable(struct irq_data *d)
{
	struct davinci_gpio_bank *bank = irq_get_chip_data(d->irq);
	struct davinci_gpio_regs *regs = bank->regs;
	u32 mask = (u32) irq_data_get_irq_handler_data(d);

	writel(mask, bank->reg_base + regs->clr_falling);
	writel(mask, bank->reg_base + regs->clr_rising);
}

static void gpio_irq_enable(struct irq_data *d)
{
	struct davinci_gpio_bank *bank = irq_get_chip_data(d->irq);
	struct davinci_gpio_regs *regs = bank->regs;
	u32 mask = (u32) irq_data_get_irq_handler_data(d);
	unsigned status = irqd_get_trigger_type(d);

	status &= IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING;
	if (!status)
		status = IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING;

	if (status & IRQ_TYPE_EDGE_FALLING)
		writel(mask, bank->reg_base + regs->set_falling);
	if (status & IRQ_TYPE_EDGE_RISING)
		writel(mask, bank->reg_base + regs->set_rising);
}

static int gpio_irq_type(struct irq_data *d, unsigned trigger)
{
	if (trigger & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	return 0;
}

static struct irq_chip gpio_irqchip = {
	.name		= "GPIO",
	.irq_enable	= gpio_irq_enable,
	.irq_disable	= gpio_irq_disable,
	.irq_set_type	= gpio_irq_type,
	.flags		= IRQCHIP_SET_TYPE_MASKED,
};

static void
gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	u32 mask = 0xffff;
	struct davinci_gpio_bank *bank;
	struct davinci_gpio_regs *regs;

	bank = irq_desc_get_handler_data(desc);
	regs = bank->regs;

	/* we only care about one bank */
	if (irq & 1)
		mask <<= 16;

	/* temporarily mask (level sensitive) parent IRQ */
	desc->irq_data.chip->irq_mask(&desc->irq_data);
	desc->irq_data.chip->irq_ack(&desc->irq_data);
	while (1) {
		u32		status;
		int		n;
		int		res;

		/* ack any irqs */
		status = readl(bank->reg_base + regs->intstat) & mask;
		if (!status)
			break;
		writel(status, bank->reg_base + regs->intstat);

		/* now demux them to the right lowlevel handler */
		n = bank->irq_base;
		if (irq & 1) {
			n += 16;
			status >>= 16;
		}

		while (status) {
			res = ffs(status);
			n += res;
			generic_handle_irq(n - 1);
			status >>= res;
		}
	}
	desc->irq_data.chip->irq_unmask(&desc->irq_data);
	/* now it may re-trigger */
}

static int gpio_to_irq_banked(struct gpio_chip *chip, unsigned offset)
{
	struct davinci_gpio_bank *d = chip2bank(chip);

	if (d->irq_base >= 0)
		return d->irq_base + offset;
	else
		return -ENODEV;
}

static int gpio_to_irq_unbanked(struct gpio_chip *chip, unsigned offset)
{
	struct davinci_gpio_bank *d = chip2bank(chip);

	/*
	 * NOTE:  we assume for now that only irqs in the first gpio_chip
	 * can provide direct-mapped IRQs to AINTC (up to 32 GPIOs).
	 */
	if (offset < d->gpio_unbanked)
		return d->gpio_irq + offset;
	else
		return -ENODEV;
}

static int gpio_irq_type_unbanked(struct irq_data *data, unsigned trigger)
{
	struct davinci_gpio_bank *bank;
	void __iomem *g;
	u32 mask;

	bank = irq_get_handler_data(data->irq);
	g = bank->reg_base;
	mask = __gpio_mask(data->irq - bank->gpio_irq);

	if (trigger & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	g = bank->reg_base + ((trigger & IRQ_TYPE_EDGE_FALLING)
		     ? bank->regs->set_falling : bank->regs->clr_falling);
	writel(mask, g);
	g = bank->reg_base + ((trigger & IRQ_TYPE_EDGE_RISING)
		     ? bank->regs->set_rising : bank->regs->clr_rising);
	writel(mask, g);

	return 0;
}


static int davinci_gpio_unbanked_irq_init(struct davinci_gpio_bank *chipb)
{
	unsigned	gpio, irq;
	unsigned	bank_irq;
	struct resource *res;
	struct platform_device *pdev;
	static struct irq_chip_type gpio_unbanked;

	pdev = to_platform_device(chipb->dev);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(chipb->dev, "Invalid IRQ resource\n");
		return -ENODEV;
	}

	bank_irq = res->start;

	if (!bank_irq) {
		dev_err(chipb->dev, "Invalid IRQ resource\n");
		return -EINVAL;
	}

	chipb->irq_base = -EINVAL;
	/* pass "bank 0" GPIO IRQs to AINTC */
	chipb->chip.to_irq = gpio_to_irq_unbanked;
	chipb->gpio_irq = bank_irq;

	/*
	 * AINTC can handle direct/unbanked IRQs for GPIOs, with the GPIO
	 * controller only handling trigger modes.  We currently assume no
	 * IRQ mux conflicts; gpio_irq_type_unbanked() is only for GPIOs.
	 */

	/* AINTC handles mask/unmask; GPIO handles triggering */
	irq = bank_irq;
	gpio_unbanked = *container_of(irq_get_chip(irq),
				      struct irq_chip_type, chip);
	gpio_unbanked.chip.name = "GPIO-AINTC";
	gpio_unbanked.chip.irq_set_type = gpio_irq_type_unbanked;

	/* set the direct IRQs up to use that irqchip */
	for (gpio = 0; gpio < chipb->gpio_unbanked; gpio++, irq++) {
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
		irq_set_chip(irq, &gpio_unbanked.chip);
		irq_set_handler_data(irq, chipb);
		irq_set_irq_type(irq, IRQ_TYPE_NONE);
	}

	davinci_gpio_ctrl_irqen(chipb->dev->parent,
				chipb->chip.base / MAX_GPIO_PER_BANK);

	dev_info(chipb->dev, "configured %u - %u unbanked gpio irqs\n",
		 bank_irq, irq - 1);

	return 0;
}

static int davinci_gpio_banked_irq_init(struct davinci_gpio_bank *chipb)
{
	unsigned	gpio, irq;
	unsigned	bank_irq;
	struct resource	 *res;
	struct platform_device *pdev;
	struct davinci_gpio_bank_pdata *pdata = dev_get_platdata(chipb->dev);

	pdev = to_platform_device(chipb->dev);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(chipb->dev, "Invalid IRQ resource\n");
		return -ENODEV;
	}

	/* Each bank uses 2 banked IRQ */
	bank_irq = res->start;

	if (!bank_irq) {
		dev_err(chipb->dev, "Invalid IRQ resource\n");
		return -EINVAL;
	}

	chipb->chip.to_irq = gpio_to_irq_banked;
	chipb->irq_base = pdata->intc_irq_num + chipb->chip.base;

	/* disabled by default, enabled only as needed */
	writel(~0, chipb->reg_base + chipb->regs->clr_falling);
	writel(~0, chipb->reg_base + chipb->regs->clr_rising);

	/*
	 * Or, AINTC can handle IRQs for banks of 16 GPIO IRQs, which we
	 * then chain through our own handler.
	 */
	for (gpio = 0, irq = gpio_to_irq(chipb->chip.base);
			gpio < chipb->chip.ngpio; bank_irq++) {
		unsigned		i;

		/* set up all irqs in this bank */
		irq_set_chained_handler(bank_irq, gpio_irq_handler);

		/*
		 * Each chip handles 32 gpios, and each irq bank consists of 16
		 * gpio irqs. Pass the irq bank's corresponding controller to
		 * the chained irq handler.
		 */
		irq_set_handler_data(bank_irq, chipb);

		for (i = 0; i < 16 && gpio < chipb->chip.ngpio;
			    i++, irq++, gpio++) {
			irq_set_chip(irq, &gpio_irqchip);
			irq_set_chip_data(irq, chipb);
			irq_set_handler_data(irq, (void *)__gpio_mask(gpio));
			irq_set_handler(irq, handle_simple_irq);
			set_irq_flags(irq, IRQF_VALID);
		}
	}

	davinci_gpio_ctrl_irqen(chipb->dev->parent,
				chipb->chip.base / MAX_GPIO_PER_BANK);

	dev_info(chipb->dev, "configured %u - %u banked gpio irqs\n",
		 gpio_to_irq(chipb->chip.base), irq - 1);

	return 0;
}

static struct platform_driver davinci_gpio_bank_driver = {
	.probe		= davinci_gpio_bank_probe,
	.driver		= {
		.name	= "davinci_gpio_bank",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver davinci_gpio_driver = {
	.probe		= davinci_gpio_probe,
	.driver		= {
		.name	= "davinci_gpio",
		.owner	= THIS_MODULE,
	},
};

/**
 * GPIO driver registration needs to be done before machine_init functions
 * access GPIO. Hence davinci_gpio_drv_reg() is a postcore_initcall.
 */
static int __init davinci_gpio_drv_reg(void)
{
	return platform_driver_register(&davinci_gpio_driver);
}
postcore_initcall(davinci_gpio_drv_reg);

static int __init davinci_gpio_bank_drv_reg(void)
{
	return platform_driver_register(&davinci_gpio_bank_driver);
}
postcore_initcall(davinci_gpio_bank_drv_reg);
