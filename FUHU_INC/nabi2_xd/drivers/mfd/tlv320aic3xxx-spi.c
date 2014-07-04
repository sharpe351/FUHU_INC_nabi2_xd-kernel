
#include <linux/err.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/mfd/tlv320aic3xxx-core.h>
//carl
#include <sound/tlv320aic325x.h>
#include <linux/gpio.h>
#include <linux/delay.h>
//carl end

struct regmap_config tlv320aic3xxx_spi_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = 0x1,
#if 0 /*Available in K3.6, need to set for our SPI*/
	.reg_shift = 1;
#endif
};

static int __devinit tlv320aic3xxx_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct aic3xxx *tlv320aic3xxx;
	const struct regmap_config *regmap_config;
	struct aic325x_pdata *pdata = NULL;
	
	int ret;	
	switch (id->driver_data) {
#ifdef CONFIG_AIC3262_CORE
	case TLV320AIC3262:
		regmap_config = &tlv320aic3xxx_spi_regmap;
		break;
#endif
	case TLV320AIC3256:
		//regmap_config = &tlv320aic3xxx_spi_regmap;
		printk("device is TLV320AIC3256\n");
		break;
	default:
		dev_err(&spi->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	tlv320aic3xxx = devm_kzalloc(&spi->dev,
					sizeof(struct aic3xxx), GFP_KERNEL);
	if (tlv320aic3xxx == NULL)
		return -ENOMEM;
	/*
	tlv320aic3xxx->regmap = devm_regmap_init_spi(spi, regmap_config);
	if (IS_ERR(tlv320aic3xxx->regmap)) {
		ret = PTR_ERR(tlv320aic3xxx->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	*/
	
	tlv320aic3xxx->control_data = spi;
	tlv320aic3xxx->type = id->driver_data;
	tlv320aic3xxx->dev = &spi->dev;
	//tlv320aic3xxx->irq = spi->irq;
	tlv320aic3xxx->pdata.gpio_reset = 0;
	tlv320aic3xxx->pdata.gpio_irq = 0;
	tlv320aic3xxx->pdata.naudint_irq = 0;
	
	pdata = spi->dev.platform_data;
	if(gpio_request(pdata->cspin, "SPI_CS_SELECT") < 0) {
		printk("Failed to request SPI_CS_SELECT 0x%x\n", pdata->cspin);
		return -1;
	} else {
		tegra_gpio_enable(pdata->cspin);
	}
	
	return aic3xxx_device_init(tlv320aic3xxx);
}

//carl
void spi_cs_en(unsigned int cs, unsigned int cspin)
{
	if(cs ==1)
	{
		gpio_direction_output(cspin, 1);
		udelay(1);
	}
	if(cs == 0)
	{
		gpio_direction_output(cspin, 0);
		udelay(1);
	}
}

//&*&*&*CT1_130408: unuse kzalloc function to avoid unexcepted exception issue
//static int aic3262_hw_write_spi(struct snd_soc_codec *codec, u8 *buf,  int count)
int aic3xxx_spi_write_device(struct aic3xxx *aic3xxx, u8 offset,
					const void *src, int count)
{

	int ret = 0;
	
	struct aic325x_pdata *pdata = NULL;
	struct spi_device *spi = aic3xxx->control_data;
	//u8 *data = kzalloc(count, GFP_KERNEL);
	
	u8 write_buf[count+1];
	write_buf[0] = offset;
	
	memcpy(&write_buf[1], src, count);
    
	//memcpy(data,write_buf,count+1);
	
	write_buf[0] = write_buf[0] << 1;
	//data[0] = data[0] << 1;
	pdata = spi->dev.platform_data;
	
	spi_cs_en(0, pdata->cspin);
	ret = spi_write(spi,write_buf,count+1);
	//ret = spi_write(spi,data,count+1);
	spi_cs_en(1, pdata->cspin);
	
	if(ret < 0) {
		printk("Error in spi write\n");
		return -EIO;
		}
	
	//kzfree(data);
	
	return 0;
}
//&*&*&*CT2_130408: unuse kzalloc function to avoid unexcepted exception issue
EXPORT_SYMBOL_GPL(aic3xxx_spi_write_device);

//unsigned int aic3262_spi_series_read(struct snd_soc_codec *codec, unsigned int reg, const char *pbuf, int count)
unsigned int aic3xxx_spi_read_device(struct aic3xxx *aic3xxx, u8 offset, void *dest,
					int count)
{
	
	int ret = 0;
      u8 data = (u8)offset;
      unsigned int i;
      unsigned int time;
      unsigned int last_count;
      unsigned int spi_read_bufsiz = max(32,SMP_CACHE_BYTES)-1;

      struct aic325x_pdata *pdata = NULL;
      struct spi_device *spi;

      if(aic3xxx->control_data == NULL)
	      printk("the control data is null\n");

      spi = aic3xxx->control_data;
      pdata = spi->dev.platform_data;

      if (data > 127)
         return 0;

      data = (data<<1) | 0x01;

      time = count / spi_read_bufsiz;
      last_count = count % spi_read_bufsiz;
      for (i=0; i<time; i=i+1)
      {
	   		spi_cs_en(0, pdata->cspin);
        ret = spi_write_then_read(aic3xxx->control_data, &data, 1, (u8 *)(dest+i*spi_read_bufsiz), spi_read_bufsiz);
	   		spi_cs_en(1, pdata->cspin);
        if (ret < 0)
        {
        	printk("[codec] aic3262_spi_read reg=%x write error\n", data>>1);
          return -EIO;
        }
	    	data = data + (spi_read_bufsiz<<1);
      }
      	
      spi_cs_en(0, pdata->cspin);
      ret = spi_write_then_read(aic3xxx->control_data, &data, 1, (u8 *)(dest+i*spi_read_bufsiz), last_count);
      spi_cs_en(1, pdata->cspin);
      if (ret < 0)
      {
           printk("[codec] aic3262_spi_read reg=%x write error\n", data>>1);
           return -EIO;
      }
			
      return ret;
}

EXPORT_SYMBOL_GPL(aic3xxx_spi_read_device);
//carl end

static int __devexit tlv320aic3xxx_spi_remove(struct spi_device *spi)
{
	struct aic3xxx *tlv320aic3xxx = dev_get_drvdata(&spi->dev);
	aic3xxx_device_exit(tlv320aic3xxx);
	return 0;
}

static const struct spi_device_id aic3xxx_spi_ids[] = {
	{"tlv320aic3262", TLV320AIC3262},
	{"tlv320aic325x", TLV320AIC3256},
	{ }
};
MODULE_DEVICE_TABLE(spi, aic3xxx_spi_ids);

static struct spi_driver tlv320aic3xxx_spi_driver = {
	.driver = {
		.name	= "tlv320aic3xxx",
		.owner	= THIS_MODULE,
		/*.pm	= &tlv320aic3xxx_pm_ops, */
	},
	.probe		= tlv320aic3xxx_spi_probe,
	.remove		= __devexit_p(tlv320aic3xxx_spi_remove),
	.id_table	= aic3xxx_spi_ids,
};

//carl
static int __init tlv320aic3xxx_modinit(void)
{
	int ret = 0;
	printk(KERN_INFO "XXX In %s\n",__func__);

	ret = spi_register_driver(&tlv320aic3xxx_spi_driver);

	if (ret != 0)
		printk(KERN_ERR "Failed to register aic3262 SPI driver: %d\n", ret);

	return ret;
}


//module_spi_driver(tlv320aic3xxx_spi_driver);
module_init(tlv320aic3xxx_modinit);
//carl end
MODULE_DESCRIPTION("TLV320AIC3XXX SPI bus interface");
MODULE_AUTHOR("Mukund Navada <navada@ti.com>");
MODULE_LICENSE("GPL");
