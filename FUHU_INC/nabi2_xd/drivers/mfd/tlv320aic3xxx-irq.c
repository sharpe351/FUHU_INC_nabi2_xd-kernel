/*
 * tlv320aic3xxx-irq.c  --  Interrupt controller support for
 *			 TI TLV320AIC3XXX
 *
 * Author:      Mukund Navada <navada@ti.com>
 *              Mehar Bajwa <mehar.bajwa@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>

#include <linux/mfd/tlv320aic3xxx-core.h>
#include <linux/mfd/tlv320aic3xxx-registers.h>

#include <linux/delay.h>

struct aic3xxx_irq_data {
	int mask;
	int status;
};

static struct aic3xxx_irq_data aic3xxx_irqs[] = {
	{
	 .mask = AIC3XXX_HEADSET_IN_M,
	 .status = AIC3XXX_HEADSET_PLUG_UNPLUG_INT,
	 },
	{
	 .mask = AIC3XXX_BUTTON_PRESS_M,
	 .status = AIC3XXX_BUTTON_PRESS_INT,
	 },
	{
	 .mask = AIC3XXX_DAC_DRC_THRES_M,
	 .status = AIC3XXX_LEFT_DRC_THRES_INT | AIC3XXX_RIGHT_DRC_THRES_INT,
	 },
	{
	 .mask = AIC3XXX_OVER_CURRENT_M,
	 .status = AIC3XXX_LEFT_OUTPUT_DRIVER_OVERCURRENT_INT |
			AIC3XXX_RIGHT_OUTPUT_DRIVER_OVERCURRENT_INT,
	 },
	{
	.mask = AIC3XXX_OVERFLOW_M,
	.status = AIC3XXX_LEFT_DAC_OVERFLOW_INT |
			AIC3XXX_RIGHT_DAC_OVERFLOW_INT |
			AIC3XXX_LEFT_ADC_OVERFLOW_INT |
			AIC3XXX_RIGHT_ADC_OVERFLOW_INT,
	},

};


static inline struct aic3xxx_irq_data *irq_to_aic3xxx_irq(struct aic3xxx
							  *aic3xxx, int irq)
{
	return &aic3xxx_irqs[irq - aic3xxx->irq_base];
}

static void aic3xxx_irq_lock(struct irq_data *data)
{
	struct aic3xxx *aic3xxx = irq_data_get_irq_chip_data(data);

	mutex_lock(&aic3xxx->irq_lock);
}

static void aic3xxx_irq_sync_unlock(struct irq_data *data)
{

	struct aic3xxx *aic3xxx = irq_data_get_irq_chip_data(data);

	/* write back to hardware any change in irq mask */
	if (aic3xxx->irq_masks_cur != aic3xxx->irq_masks_cache) {
		aic3xxx->irq_masks_cache = aic3xxx->irq_masks_cur;
		aic3xxx_reg_write(aic3xxx, AIC3XXX_INT1_CNTL,
				  aic3xxx->irq_masks_cur);
	}

	mutex_unlock(&aic3xxx->irq_lock);
}


static void aic3xxx_irq_unmask(struct irq_data *data)
{
	struct aic3xxx *aic3xxx = irq_data_get_irq_chip_data(data);
	struct aic3xxx_irq_data *irq_data =
			irq_to_aic3xxx_irq(aic3xxx, data->irq);

	aic3xxx->irq_masks_cur |= irq_data->mask;
}

static void aic3xxx_irq_mask(struct irq_data *data)
{
	struct aic3xxx *aic3xxx = irq_data_get_irq_chip_data(data);
	struct aic3xxx_irq_data *irq_data =
			irq_to_aic3xxx_irq(aic3xxx, data->irq);


	aic3xxx->irq_masks_cur &= ~irq_data->mask;
}


#if 0
int aic3xxx_request_irq(struct aic3xxx *aic3xxx, int irq, char *name,
					irq_handler_t handler, void *data)
{
	irq = irq_create_mapping(aic3xxx->domain, irq);
	if (irq < 0)
		return irq;

	return request_threaded_irq(irq, NULL, handler, IRQF_ONESHOT,
					name, data);
}
EXPORT_SYMBOL_GPL(aic3xxx_request_irq);
#endif

#if 0
static void aic3262_irq_unmask(struct irq_data *data)
{
	struct aic3xxx *aic3262 = irq_data_get_irq_chip_data(data);
	struct aic3262_irq_data *irq_data =
	irq_to_aic3262_irq(aic3262, data->irq);

	aic3262->irq_masks_cur |= irq_data->mask;
}

static void aic3262_irq_mask(struct irq_data *data)
{
	struct aic3xxx *aic3262 = irq_data_get_irq_chip_data(data);
	struct aic3262_irq_data *irq_data =
	irq_to_aic3262_irq(aic3262, data->irq);

	aic3262->irq_masks_cur &= ~irq_data->mask;
}
#endif

static irqreturn_t aic3xxx_irq_thread(int irq, void *data)
{

	struct aic3xxx *aic3xxx = data;
	u8 status[4];

	/* Reading sticky bit registers acknowledges
			the interrupt to the device */
	aic3xxx_bulk_read(aic3xxx, AIC3XXX_INT_STICKY_FLAG1, 4, status);
	dev_info(aic3xxx->dev, "aic3xxx_irq_thread %x\n", status[2]);

	/* report  */
	if (status[2] & aic3xxx_irqs[AIC3256_IRQ_HEADSET_DETECT].status)
		handle_nested_irq(aic3xxx->irq_base);
	if (status[2] & aic3xxx_irqs[AIC3256_IRQ_BUTTON_PRESS].status)
		handle_nested_irq(aic3xxx->irq_base + 1);
	if (status[2] & aic3xxx_irqs[AIC3256_IRQ_DAC_DRC].status)
		handle_nested_irq(aic3xxx->irq_base + 2);
	if (status[2] & aic3xxx_irqs[AIC3256_IRQ_OVER_CURRENT].status)
		handle_nested_irq(aic3xxx->irq_base + 3);
	if (status[0] & aic3xxx_irqs[AIC3256_IRQ_OVERFLOW_EVENT].status)
		handle_nested_irq(aic3xxx->irq_base + 4);

	/* ack unmasked irqs */
	/* No need to acknowledge the interrupt on AIC3xxx */


	return IRQ_HANDLED;


#if 0
	struct aic3xxx *aic3262 = data;
	u8 status[4];

	/* Reading sticky bit registers acknowledges
		the interrupt to the device */
	aic3xxx_bulk_read(aic3262, AIC3262_INT_STICKY_FLAG1, 4, status);

	/* report  */
	if (status[2] & aic3262_irqs[AIC3262_IRQ_HEADSET_DETECT].status)
		handle_nested_irq(aic3262->irq_base);
	if (status[2] & aic3262_irqs[AIC3262_IRQ_BUTTON_PRESS].status)
		handle_nested_irq(aic3262->irq_base + 1);
	if (status[2] & aic3262_irqs[AIC3262_IRQ_DAC_DRC].status)
		handle_nested_irq(aic3262->irq_base + 2);
	if (status[3] & aic3262_irqs[AIC3262_IRQ_AGC_NOISE].status)
		handle_nested_irq(aic3262->irq_base + 3);
	if (status[2] & aic3262_irqs[AIC3262_IRQ_OVER_CURRENT].status)
		handle_nested_irq(aic3262->irq_base + 4);
	if (status[0] & aic3262_irqs[AIC3262_IRQ_OVERFLOW_EVENT].status)
		handle_nested_irq(aic3262->irq_base + 5);
	if (status[3] & aic3262_irqs[AIC3262_IRQ_SPEAKER_OVER_TEMP].status)
		handle_nested_irq(aic3262->irq_base + 6);

	/* ack unmasked irqs */
	/* No need to acknowledge the interrupt on AIC3262 */

	return IRQ_HANDLED;
#endif
}


static struct irq_chip aic3xxx_irq_chip = {
	.name = "tlv320aic3256",
	.irq_bus_lock = aic3xxx_irq_lock,
	.irq_bus_sync_unlock = aic3xxx_irq_sync_unlock,
	.irq_mask = aic3xxx_irq_mask,
	.irq_unmask = aic3xxx_irq_unmask,
};



#if 0
static int aic3xxx_irq_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	struct aic3xxx *aic3xxx = h->host_data;

	irq_set_chip_data(virq, aic3xxx);
	irq_set_chip_and_handler(virq, &aic3xxx_irq_chip, handle_edge_irq);
	irq_set_nested_thread(virq, 1);

	/* ARM needs us to explicitly flag the IRQ as valid
		and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static struct irq_domain_ops aic3xxx_domain_ops = {
	.map	= aic3xxx_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};
#endif


int aic3xxx_irq_init(struct aic3xxx *aic3xxx)
{
	int ret;
	unsigned int cur_irq;
	mutex_init(&aic3xxx->irq_lock);

	aic3xxx->irq_masks_cur = 0x0;
	aic3xxx->irq_masks_cache = 0x0;
	aic3xxx_reg_write(aic3xxx, AIC3XXX_INT1_CNTL, 0x0);
#if 0
	if (aic3xxx->irq_base > 0) {
		aic3xxx->domain = irq_domain_add_legacy(aic3xxx->dev->of_node,
						ARRAY_SIZE(aic3xxx_irqs),
						aic3xxx->irq_base,
						0,
						&aic3xxx_domain_ops,
						aic3xxx);
	} else {
		aic3xxx->domain = irq_domain_add_linear(aic3xxx->dev->of_node,
						ARRAY_SIZE(aic3xxx_irqs),
						&aic3xxx_domain_ops, aic3xxx);
}
	if (!aic3xxx->domain) {
		dev_err(aic3xxx->dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	aic3xxx->irq_base = irq_find_mapping(aic3xxx->domain, 0);
#endif
	if (!aic3xxx->irq) {
		dev_warn(aic3xxx->dev,
				"no interrupt specified, no interrupts\n");
		aic3xxx->irq_base = 0;
		return 0;
	}

	if (!aic3xxx->irq_base) {
		dev_err(aic3xxx->dev,
				"no interrupt base specified, no interrupts\n");
		return 0;
	}


	/* Register them with genirq */
	for (cur_irq = aic3xxx->irq_base;
		cur_irq < aic3xxx->irq_base + ARRAY_SIZE(aic3xxx_irqs);
		cur_irq++) {
		irq_set_chip_data(cur_irq, aic3xxx);
		irq_set_chip_and_handler(cur_irq, &aic3xxx_irq_chip,
				handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		set_irq_noprobe(cur_irq);
#endif
	}

	ret = request_threaded_irq(aic3xxx->irq, NULL, aic3xxx_irq_thread,
					IRQF_TRIGGER_RISING,
					"tlv320aic3256", aic3xxx);


	if (ret < 0) {
		dev_err(aic3xxx->dev, "failed to request IRQ %d: %d\n",
					aic3xxx->irq, ret);
		return ret;
	}

	return 0;

}
EXPORT_SYMBOL(aic3xxx_irq_init);

void aic3xxx_irq_exit(struct aic3xxx *aic3xxx)
{
	if (aic3xxx->irq)
		free_irq(aic3xxx->irq, aic3xxx);
}
EXPORT_SYMBOL(aic3xxx_irq_exit);

MODULE_DESCRIPTION("Interrupt controller support for TI TLV320AIC3XXXX");
MODULE_LICENSE("GPL");
