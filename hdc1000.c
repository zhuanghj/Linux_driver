/*hdc1000 驱动  这个驱动的sb的地方就是 在读取数据的时候  步骤如下
* 初始化->写地址->等待->跳管脚->读数据
* 中间不能有其他东西，但是linux下i2c的一些层次比较高的函数都是读写在一起，中间没有等管脚
* 这种做法，所以要用相对底层一点的函数，将具体的动作分解开。
*/
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>


enum hdctype
{
	temperature,
	humidity,
};



struct hdc1000{
	int irq;
	int irq_gpio;
	int temperature;
	int humidity;
	
	struct device * dev;
	struct timer_list timer;	
	struct work_struct ws_timer;
	struct work_struct ws_gpio;
	struct workqueue_struct * queue;
	enum hdctype type;
};




#define DT_LABEL_IRQ_GPIO						"irq-gpio"
//struct device * hdcdev;

#define HDC_TRIGGER_TEMPERATURE					0x00
#define HDC_TRIGGER_HUMIDITY					0x01

static const struct of_device_id hdc1000_of_match[] = {
	{ .compatible = "hdc1000" },
	{ }
};

static const struct i2c_device_id hdc1000_id[] = {
	{ "hdc1000", 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, hdc1000_id);


static irqreturn_t hdc_data_handler(int irq,void * dev_id);
static void hdc_data_real_handler(struct work_struct * work);
static void hdc_timer_real_handler(struct work_struct * work);

#define DEBUGPRINT() (debug_print(__FUNCTION__,__LINE__))
static void debug_print(const char * func_name,int lineno)
{
	printk(KERN_INFO "func:%s,line:%d\r\n",func_name,lineno);
}


static struct hdc1000 * hdc1000_parse_dt(struct device * dev)
{
	struct hdc1000 *pdata;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;

	
	match = of_match_device(of_match_ptr(hdc1000_of_match), dev);
	if (!match)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;


	pdata->irq_gpio = of_get_named_gpio(np, DT_LABEL_IRQ_GPIO, 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		dev_err(dev, "Failed to get IRQ GPIO\n");
		return NULL;
	}

	
	return pdata;
}


static int translate_value(enum hdctype type,unsigned char  valuem,unsigned char valuel)
{
	int tmp = (valuem << 8) + valuel;
//	printk(KERN_INFO "tmp value = %x\r\n",tmp);
	if(type)
	{
		tmp = (tmp * 100) >> 16;	
	}
	else
	{	
		tmp = ((tmp * 165) >> 16) - 40;
	}
	return tmp;
}

static irqreturn_t hdc_data_handler(int irq,void * dev_id)
{
	struct hdc1000 * hdc = (struct hdc1000 *)dev_id;
	schedule_work(&hdc->ws_gpio);
	return IRQ_HANDLED;
}

static void hdc_data_real_handler(struct work_struct * work)
{
	int ret = 0;
	struct i2c_msg msg[2];
	unsigned char uret[2] = {0x00,0x00};
	struct hdc1000 * pdata = container_of(work,struct hdc1000,ws_gpio);
	struct i2c_client * client = to_i2c_client(pdata->dev);

	memset((void *)&msg,0,sizeof(msg));
	msg[0].addr = 0x40;
	msg[0].flags = I2C_M_RD;
	msg[0].buf = &uret;
	msg[0].len = 2;
	i2c_transfer(client->adapter,msg,1);
	disable_irq(pdata->irq);

	ret = translate_value(pdata->type,uret[0],uret[1]);
	if(pdata->type == temperature)
		pdata->temperature = ret;
	else
		pdata->humidity = ret;
	

	
		
}

static void hdc_timer_real_handler(struct work_struct * work)
{
	struct i2c_msg msg;
	struct hdc1000 * pdata = container_of(work,struct hdc1000,ws_timer);
	struct i2c_client * client = to_i2c_client(pdata->dev);


	
	unsigned char reg =pdata->type?0x00:0x01;
	pdata->type = pdata->type?0x00:0x01;

	msg.addr = 0x40;
	msg.flags = 0;
	msg.buf = &reg;
	msg.len = 1;
	i2c_transfer(client->adapter,&msg,1);


	pdata->timer.expires = msecs_to_jiffies(5000) + jiffies;
	add_timer(&pdata->timer);
	enable_irq(pdata->irq);

	
}

static void hdc_timer_handler(unsigned long arg)
{
	struct i2c_client * client = to_i2c_client((struct device *)arg);
	struct hdc1000 * pdata = i2c_get_clientdata(client);
	schedule_work(&pdata->ws_timer);
}

static int hdc_init(struct i2c_client * client)
{
	i2c_smbus_write_word_data(client,0x02,0x0010);
	return 0;
}



static ssize_t hdc1000_show_temperature(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int ret = 0;
	struct i2c_client * client = to_i2c_client((struct device *)dev);
	struct hdc1000 * pdata = i2c_get_clientdata(client);


	printk(KERN_INFO "dev = %x\r\n",dev);		
	printk(KERN_INFO "pdata = %x\r\n",pdata);	
	ret = sprintf(buf, "%d\n", pdata->temperature);

	return ret;
}

static ssize_t hdc1000_show_humidity(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct i2c_client * client = to_i2c_client((struct device *)dev);
	struct hdc1000 * pdata = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", pdata->humidity);
}



/* sysfs attributes */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, hdc1000_show_temperature,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, hdc1000_show_humidity,
	NULL, 0);

static struct attribute *hdc1000_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	NULL
};

static const struct attribute_group hdc1000_attr_group = {
	.attrs = hdc1000_attributes,
};




static int hdc1000_probe(struct i2c_client * client,const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret = 0;
	struct hdc1000 *pdata;
//	struct workqueue_struct * queue;


	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
			"adapter does not support SMBus word transactions\n");
		return -ENODEV;
	}
	pdata = hdc1000_parse_dt(dev);	
	
	printk(KERN_INFO "pdata = %x\r\n",pdata);	
	if(!pdata)
	{
		dev_err(dev,"device tree error");
		return -EINVAL;
	}
	pdata->irq = gpio_to_irq(pdata->irq_gpio);
	ret = request_irq(gpio_to_irq(pdata->irq_gpio),hdc_data_handler,IRQF_TRIGGER_FALLING,"hdc_irq_check",pdata);
	if(ret < 0 )
	{
		dev_err(dev,"request irq error");
		return -EINVAL;
	}
	

	init_timer(&pdata->timer);
	pdata->timer.function = hdc_timer_handler;
	pdata->timer.data = (unsigned long)dev;
	pdata->timer.expires = jiffies + msecs_to_jiffies(5000);
	add_timer(&pdata->timer);

//tasklet 函数中不能存在等待 延时等操作，但是在此处我们需要使用i2c的读写操作。只能使用work_struct

	INIT_WORK(&pdata->ws_gpio,hdc_data_real_handler);
	INIT_WORK(&pdata->ws_timer,hdc_timer_real_handler);
	
	ret = sysfs_create_group(&client->dev.kobj, &hdc1000_attr_group);
	if (ret) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		return ret;
	}

	ret = hwmon_device_register(&client->dev);
	if (IS_ERR(ret)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		ret = PTR_ERR(ret);
		goto fail_remove_sysfs;
	}
	ret = 0;
	pdata->dev = dev;
	
	i2c_set_clientdata(client,(void *)pdata);

	disable_irq(pdata->irq);
		printk(KERN_INFO "dev = %x\r\n",dev);	
	printk(KERN_INFO "pdata = %x\r\n",pdata);	

	hdc_init(client);
	return ret;

fail_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &hdc1000_attr_group);
	return ret;
}

static int hdc1000_remove(struct i2c_client * client)
{
	return 0;
}



static struct i2c_driver hdc1000_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "hdc1000",
		.of_match_table = of_match_ptr(hdc1000_of_match),
		},
	.probe       = hdc1000_probe,
	.remove      = hdc1000_remove,
	.id_table    = hdc1000_id,
};

module_i2c_driver(hdc1000_driver);



MODULE_AUTHOR("zhuanghj");
MODULE_DESCRIPTION("HDC1000 Driver");
MODULE_LICENSE("GPL");

