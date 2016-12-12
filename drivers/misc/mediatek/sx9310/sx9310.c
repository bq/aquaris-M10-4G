#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "sx9310.h"

/*----------------------------------------------------------------------------*/
#define SX9310_DEV_NAME   "SX9310"
#define SAR_DEBUG_ENABLE
/*----------------------------------------------------------------------------*/
#define SAR_TAG                  "[SAR]"
#ifdef SAR_DEBUG_ENABLE
#define SAR_FUN(f)               printk(KERN_ERR SAR_TAG"%s\n", __FUNCTION__)
#define SAR_ERR(fmt, args...)    printk(KERN_ERR  SAR_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SAR_ERR_ST(f)    printk(KERN_ERR  SAR_TAG"%s %d : ", __FUNCTION__, __LINE__)
#define SAR_LOG(fmt, args...)    printk(KERN_ERR SAR_TAG fmt, ##args)
#define SAR_DBG(fmt, args...)    printk(KERN_ERR SAR_TAG fmt, ##args)
#else
#define SAR_FUN(f)
#define SAR_ERR(fmt, args...)  printk(KERN_ERR  SAR_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SAR_ERR_ST(f)
#define SAR_LOG(fmt, args...)  printk(KERN_ERR SAR_TAG fmt, ##args)
#define SAR_DBG(fmt, args...)
#endif
/*----------------------------------------------------------------------------*/
//ioctl
#define SAR             0X84
#define GET_SAR_MSB_DATA        _IOR(SAR, 0x09, int)
#define GET_SAR_LSB_DATA        _IOR(SAR, 0x10, int)
/*----------------------------------------------------------------------------*/
struct platform_device * sar_pltfm_dev;
static const struct i2c_device_id sx9310_i2c_id[] = {{SX9310_DEV_NAME,0},{}};
static struct i2c_client * sx9310_i2c_client = NULL;
static psx93XX_t  sx9310_obj = NULL;
static int SAR_EINT_PIN;
/*----------------------------------------------------------------------------*/
static int sx9310_probe(struct platform_device *pdev);
static int sx9310_remove(struct platform_device *pdev);
static int sx9310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int sx9310_i2c_remove(struct i2c_client *client);
static int sx9310_open(struct inode *inode, struct file *file);
static long sx9310_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg);
/*----------------------------------------------------------------------------*/
typedef struct sx9310
{
        pbuttonInformation_t pbuttonInformation;
        psx9310_platform_data_t hw; /* specific platform data settings */
} sx9310_t, *psx9310_t;
/*----------------------------------------------------------------------------*/
static struct _totalButtonInformation smtcButtonInformation = {
        .buttons = psmtcButtons,
        .buttonSize = ARRAY_SIZE(psmtcButtons),
};
/*----------------------------------------------------------------------------*/
static const struct of_device_id sar_of_pltmatch[] = {
	{.compatible = "mediatek,sarsensor",},
	{},
};
/*----------------------------------------------------------------------------*/
static const struct of_device_id sar_of_i2cmatch[] = {
	{.compatible = "mediatek,SAR",},
	{},
};
/*----------------------------------------------------------------------------*/
static struct platform_driver sx9310_driver = {
	.probe	  = sx9310_probe,
	.remove	 = sx9310_remove,
	.driver = {
		.name  = "sar",
		.of_match_table = sar_of_pltmatch,
	}
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver sx9310_i2c_driver = {	
	.probe      = sx9310_i2c_probe,
	.remove     = sx9310_i2c_remove,
	.id_table   = sx9310_i2c_id,
	.driver = {
                .name           = SX9310_DEV_NAME,         
                .of_match_table = sar_of_i2cmatch,
	},
};
/*----------------------------------------------------------------------------*/
static struct file_operations sx9310_fops = {
	.open = sx9310_open,
	.unlocked_ioctl = sx9310_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice sx9310_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sar",
	.fops = &sx9310_fops,
};
/*----------------------------------------------------------------------------*/
static int sx9310_get_nirq_state(void)
{
	return !gpio_get_value(SAR_EINT_PIN);
}
/*----------------------------------------------------------------------------*/
static sx9310_platform_data_t sx9310_config = {
        /* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
        .get_is_nirq_low = sx9310_get_nirq_state,
        /*  pointer to an initializer function. Here in case needed in the future */
        .init_platform_hw = NULL,
        /*  pointer to an exit function. Here in case needed in the future */
        .exit_platform_hw = NULL,

        .pi2c_reg = sx9310_i2c_reg_setup,
        .i2c_reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup),

        .pbuttonInformation = &smtcButtonInformation,
};
/*----------------------------------------------------------------------------*/
static void ForcetoTouched(psx93XX_t this)
{
        psx9310_t pDevice = NULL;
        struct input_dev *input = NULL;
        struct _buttonInfo *pCurrentButton  = NULL;

        if (this && (pDevice = this->pDevice))
        {
                SAR_FUN();

                pCurrentButton = pDevice->pbuttonInformation->buttons;
                input = pDevice->pbuttonInformation->input;

                //modify xmysx20161027
                //input_report_key(input, pCurrentButton->keycode, 1);
                input_report_key(input, KEY_SAR_ACTIVE, 1);
                input_sync(input);
                input_report_key(input, KEY_SAR_ACTIVE, 0);        
                pCurrentButton->state = ACTIVE;

                input_sync(input);
        }
}
/*----------------------------------------------------------------------------*/
static int write_register(psx93XX_t this, u8 address, u8 value)
{
        struct i2c_client * client;
        char buffer[2];
        int returnValue = 0;
        buffer[0] = address;
        buffer[1] = value;
        returnValue = -ENOMEM;
        if (this && this->bus) {
                client = this->bus;
                returnValue = i2c_master_send(client,buffer,2);
                SAR_LOG("write_register Address: 0x%x Value: 0x%x Return: %d\n",
                                    address,value,returnValue);
        }
        if(returnValue < 0){
                ForcetoTouched(this);
                SAR_LOG("Write_register-ForcetoTouched()\n");
        }
        return returnValue;
}
/*----------------------------------------------------------------------------*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
        struct i2c_client * client = 0;
        s32 returnValue = 0;
        if (this && value && this->bus) {
                client = this->bus;
                returnValue = i2c_smbus_read_byte_data(client,address);
                SAR_LOG("read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
                if (returnValue >= 0) {
                        *value = returnValue;
                        return 0;
                } else {
                        return returnValue;
                }
        }
        ForcetoTouched(this);
        SAR_LOG("read_register-ForcetoTouched()\n");
        return -ENOMEM;
}
/*----------------------------------------------------------------------------*/
static int read_regStat(psx93XX_t this)
{
        u8 data = 0;
        if (this) {
                if (read_register(this,SX9310_IRQSTAT_REG,&data) == 0)
                        return (data & 0x00FF);
        }
        return 0;
}
/*----------------------------------------------------------------------------*/
static int manual_offset_calibration(psx93XX_t this)
{
        s32 returnValue = 0;
        returnValue = write_register(this,SX9310_IRQSTAT_REG,0xFF);
        return returnValue;
}
/*----------------------------------------------------------------------------*/
static int sx9310_open(struct inode *inode, struct file *file)
{
	file->private_data = sx9310_i2c_client;

	if (!file->private_data)
	{
		SAR_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static long sx9310_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
        psx93XX_t this = i2c_get_clientdata(sx9310_i2c_client);
        int err = 0;
        void __user *ptr = (void __user*) arg;
        u8 dat;

        switch (cmd)
        {
            case GET_SAR_MSB_DATA:
                    read_register(this,SX9310_DIFFMSB,&dat);
                    if(copy_to_user(ptr, &dat, sizeof(dat)))
                    {
                            err = -EFAULT;
                            goto err_out;
                    }
                    break;
            case GET_SAR_LSB_DATA:
                    read_register(this,SX9310_DIFFLSB,&dat);
                    if(copy_to_user(ptr, &dat, sizeof(dat)))
                    {
                            err = -EFAULT;
                            goto err_out;
                    }
                    break;
    	    default:
                    SAR_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
                    err = -ENOIOCTLCMD;
                    break;
        }
err_out:
	return err;
}
/*----------------------------------------------------------------------------*/
static ssize_t sx9310_show_sarreg(struct device_driver *ddri, char *buf)
{
        u8 Value1 = -1;
        u8 Value2 = -1;
        u8 Value3 = -1;
        u8 Value4 = -1;
        read_register(sx9310_obj,SX9310_STAT0_REG,&Value1);
        read_register(sx9310_obj,SX9310_STAT1_REG,&Value2);
        read_register(sx9310_obj,SX9310_DIFFMSB,&Value3);
        read_register(sx9310_obj,SX9310_DIFFLSB,&Value4);
        return sprintf(buf,"STAT0:0x%02X  STAT1:0x%02X  DIFFMSB:0x%02X  DIFFLSB:0x%02X\n"
                        ,Value1,Value2,Value3,Value4);
}
/*----------------------------------------------------------------------------*/
static ssize_t sx9310_show_nirq(struct device_driver *ddri, char *buf)
{
        u8 value;
        value = sx9310_obj->refreshStatus(sx9310_obj);
        return sprintf(buf,"reset NIRQ %s, val:%d\n",(value != 0)?"success":"fail", value);
}
/*----------------------------------------------------------------------------*/
static ssize_t sx9310_show_reset(struct device_driver *ddri, char *buf)
{
        u8 value;
        value = sx9310_obj->init(sx9310_obj);
        return sprintf(buf,"reset chip %s\n",(value == 0)?"success":"fail");
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(sarreg,S_IWUSR | S_IRUGO,sx9310_show_sarreg,NULL);
static DRIVER_ATTR(nirq,S_IWUSR | S_IRUGO,sx9310_show_nirq,NULL);
static DRIVER_ATTR(rst,S_IWUSR | S_IRUGO,sx9310_show_reset,NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *sx9310_attr_list[] = {
        &driver_attr_sarreg,
        &driver_attr_nirq,
        &driver_attr_rst,
};
/*----------------------------------------------------------------------------*/
static int sx9310_create_attr(struct device_driver *driver) 
{
        int idx, err = 0;
        int num = (int)(sizeof(sx9310_attr_list)/sizeof(sx9310_attr_list[0]));

        if (driver == NULL)
        {
                return -EINVAL;
        }

        for(idx = 0; idx < num; idx++)
        {
                err = driver_create_file(driver, sx9310_attr_list[idx]);
                if(err)
                {            
                        SAR_ERR("driver_create_file (%s) = %d\n", sx9310_attr_list[idx]->attr.name, err);
                        break;
                }
        }    
        return err;
}
/*----------------------------------------------------------------------------*/
static int sx9310_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(sx9310_attr_list)/sizeof(sx9310_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
                driver_remove_file(driver, sx9310_attr_list[idx]);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int hw_init(psx93XX_t this)
{
        psx9310_t pDevice = 0;
        psx9310_platform_data_t pdata = 0;
        int i = 0;
        int err;
        /* configure device */
        SAR_LOG("Going to Setup I2C Registers\n");
        if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
        {
                while ( i < pdata->i2c_reg_num) {
                        /* Write all registers/values contained in i2c_reg */
                        SAR_LOG("Going to Write Reg: 0x%x Value: 0x%x\n",
                                            pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);     
                        err = write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
                        if(err < 0)
                                return -1;
                        i++;
                }
        } else {
                SAR_ERR("ERROR! platform data 0x%p\n",pDevice->hw);
                //Force to touched if error
                ForcetoTouched(this);
                SAR_ERR("Hardware_init-ForcetoTouched()\n");
        }
        return 0;
}
/*----------------------------------------------------------------------------*/
static int initialize(psx93XX_t this)
{
        int err;
        
        /* perform a reset */
        err = write_register(this,SX9310_SOFTRESET_REG,SX9310_SOFTRESET);
        if(err < 0)
                goto init_fail;
        /* wait until the reset has finished by monitoring NIRQ */
        SAR_LOG("Sent Software Reset. Waiting until device is back from reset to continue.\n");
        /* just sleep for awhile instead of using a loop with reading irq status */
        msleep(300);

        err = hw_init(this);
        if(err < 0)
                goto init_fail;
        msleep(100); /* make sure everything is running */
        err = manual_offset_calibration(this);
        if(err < 0)
                goto init_fail;
        read_regStat(this);         //need to release NIRQ
        
        return 0;

init_fail:
        return -1;
}
/*----------------------------------------------------------------------------*/
static void touchProcess(psx93XX_t this)
{
        int counter = 0;
        u8 i = 0;
        int numberOfButtons = 0;
        psx9310_t pDevice = this->pDevice;
        struct _buttonInfo *buttons = NULL;
        struct input_dev *input = NULL;
        struct _buttonInfo *pCurrentButton  = NULL;

        SAR_FUN();

        read_register(this, SX9310_STAT0_REG, &i);

        buttons = pDevice->pbuttonInformation->buttons;
        input = pDevice->pbuttonInformation->input;
        numberOfButtons = pDevice->pbuttonInformation->buttonSize;

        if (unlikely( (buttons==NULL) || (input==NULL) )) {
                SAR_ERR("button or input device is NULL\n");
                return;
        }

        for (counter = 0; counter < numberOfButtons; counter++) {
                pCurrentButton = &buttons[counter];
                if (pCurrentButton==NULL) {
                        SAR_ERR("current button at index: %d is NULL\n", counter);
                        return;
                }
                switch (pCurrentButton->state) {
                        case IDLE: /* Button is not being touched! */
                        if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
                                /* User pressed button */
                                SAR_LOG("cap button %d touched\n",counter);
                                input_report_key(input, KEY_SAR_ACTIVE, 1);
                                input_sync(input);
                                input_report_key(input, KEY_SAR_ACTIVE, 0);
                                input_sync(input);
                                pCurrentButton->state = ACTIVE;
                        } else {
                                SAR_LOG("Button %d already released\n",counter);
                        }
                        break;
                        case ACTIVE: /* Button is being touched! */ 
                        if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
                                /* User released button */
                                SAR_LOG("cap button %d released\n",counter);
                                input_report_key(input, KEY_SAR_IDLE, 1);
                                input_sync(input);
                                input_report_key(input, KEY_SAR_IDLE, 0);
                                input_sync(input);
                                pCurrentButton->state = IDLE;
                        } else {
                                SAR_LOG("Button %d still touched\n",counter);
                        }
                        break;
                        default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                        break;
                };
        }
}
/*----------------------------------------------------------------------------*/
static void sx9310_eint_work(struct work_struct *work)
{

        psx93XX_t this = 0;
        int status = 0;
        int counter = 0;
        this = container_of(work,sx93XX_t,dworker.work);

        status = this->refreshStatus(this);
        counter = -1;
        SAR_LOG("Worker - Refresh Status %d\n",status);
        while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
                if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
                        SAR_LOG("Function Pointer Found. Calling\n");
                        this->statusFunc[counter](this);
                }
        }

        enable_irq(this->irq);
}
/*----------------------------------------------------------------------------*/
static irqreturn_t sx9310_eint_handler(int irq, void *desc)
{
        disable_irq_nosync(sx9310_obj->irq);
        SAR_FUN();
        schedule_delayed_work(&sx9310_obj->dworker,0);

	return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
static int sx9310_setup_eint(struct i2c_client *client)
{
        int ret;
        struct pinctrl * pinctrl;
        struct pinctrl_state * pins_default;
        struct pinctrl_state * pins_cfg;
        u32 ints[2] = {0,0};

        SAR_FUN();
        /* gpio setting */
        pinctrl = devm_pinctrl_get(&sar_pltfm_dev->dev);
        if (IS_ERR(pinctrl)) {
        	ret = PTR_ERR(pinctrl);
        	SAR_ERR("Cannot find alsps pinctrl!\n");
        	return ret;
        }
        pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
        if (IS_ERR(pins_default)) {
        	ret = PTR_ERR(pins_default);
        	SAR_ERR("Cannot find alsps pinctrl default!\n");
        }
        pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
        if (IS_ERR(pins_cfg)) {
        	ret = PTR_ERR(pins_cfg);
        	SAR_ERR("Cannot find alsps pinctrl pin_cfg!\n");
        	return ret;
        }
        pinctrl_select_state(pinctrl, pins_cfg);

        /* eint request */
        sx9310_obj->irq_node = of_find_matching_node(sx9310_obj->irq_node, sar_of_pltmatch);
        if (sx9310_obj->irq_node) {
        	of_property_read_u32_array(sx9310_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        	//gpio_request(ints[0], "sarsensor");
        	SAR_EINT_PIN = ints[0];
        	gpio_set_debounce(ints[0], ints[1]);
        	SAR_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        	sx9310_obj->irq = irq_of_parse_and_map(sx9310_obj->irq_node, 0);
        	SAR_LOG("sx9310_obj->irq = %d\n", sx9310_obj->irq);
        	if (!sx9310_obj->irq) {
        		SAR_ERR("irq_of_parse_and_map fail!!\n");
        		return -EINVAL;
        	}

        	if (request_irq(sx9310_obj->irq, sx9310_eint_handler, IRQF_TRIGGER_NONE, "SAR-eint", NULL)) {
        		SAR_ERR("IRQ LINE NOT AVAILABLE!!\n");
        		return -EINVAL;
        	}
        } else {
        	SAR_ERR("null irq node!!\n");
        	return -EINVAL;
        }

        return 0;
}
/*----------------------------------------------------------------------------*/
static int sx9310_init_client(psx93XX_t this)
{
        int res = 0;
        SAR_FUN();

        this->init(this);
        if(res != 0)
        {
        	SAR_ERR("SX9310 init failed\n");
        	return res;
        }

        res = sx9310_setup_eint(sx9310_i2c_client);
        if(res != 0)
        {
        	SAR_ERR("setup eint: %d\n", res);
        	return res;
        }

        return res;
}
/*----------------------------------------------------------------------------*/
static int sx9310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int err = 0;
        psx93XX_t this;
        psx9310_t pDevice;
        psx9310_platform_data_t pplatData;
        struct input_dev * input = NULL;
        int i;

        SAR_FUN();

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)){
                SAR_ERR("i2c check error\n");
                return -EIO;
        }

        this = kzalloc(sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
        if(this == NULL){
		err = -ENOMEM;
		goto exit;
	}

        pplatData = &sx9310_config;

        /* In case we need to reinitialize data 
        * (e.q. if suspend reset device) */
        this->init = initialize;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regStat;
        /* pointer to function from platform data to get pendown 
        * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;

        /* Setup function to call on corresponding reg irq source bit */
        if (MAX_NUM_STATUS_BITS>= 8)
        {
                this->statusFunc[0] = 0; /* TXEN_STAT */
                this->statusFunc[1] = 0; /* UNUSED */
                this->statusFunc[2] = 0; /* UNUSED */
                this->statusFunc[3] = 0; /* CONV_STAT */
                this->statusFunc[4] = 0; /* COMP_STAT */
                this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
                this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
                this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* create memory for device specific struct */
        this->pDevice = pDevice = kzalloc(sizeof(sx9310_t), GFP_KERNEL);
        if(this->pDevice == NULL){
                err = -ENOMEM;
                goto exit;
        }

        /* Check if we hava a platform initialization function to call*/
        if (pplatData->init_platform_hw)
                pplatData->init_platform_hw();

        /* Add Pointer to main platform data struct */
        pDevice->hw = pplatData;

        /* Initialize the button information initialized with keycodes */
        pDevice->pbuttonInformation = pplatData->pbuttonInformation;

        /* Create the input device */
        input = input_allocate_device();
        if (!input) {
                SAR_ERR("allocate input device failed\n");
                err = -ENOMEM;
                goto exit;
        }

        /* Set all the keycodes */
        __set_bit(EV_KEY, input->evbit);
        for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
                __set_bit(pDevice->pbuttonInformation->buttons[i].keycode, 
                                input->keybit);
                pDevice->pbuttonInformation->buttons[i].state = IDLE;
        }

        __set_bit(KEY_SAR_ACTIVE, input->keybit);
        __set_bit(KEY_SAR_IDLE, input->keybit);

        /* save the input pointer and finish initialization */
        pDevice->pbuttonInformation->input = input;
        input->name = "SX9310 Cap Touch";
        input->id.bustype = BUS_I2C;
        if(input_register_device(input)){
                SAR_ERR("register input device failed\n");
                err = -ENOMEM;
                goto exit;
        }

        /* initialize worker function */
        INIT_DELAYED_WORK(&this->dworker, sx9310_eint_work);

        this->bus = client;
        sx9310_i2c_client = client;
        i2c_set_clientdata(client, this);
        sx9310_obj = this;

        err = sx9310_init_client(this);
        if(err)
        {
        	goto exit;
        }

        err = sx9310_create_attr(&sx9310_driver.driver);
        if(err)
        {
        	goto exit;
        }

        err = misc_register(&sx9310_device);
        if(err)
        {
                SAR_ERR("sx9310_device register failed\n");
                goto exit;
        }

        SAR_LOG("%s: OK\n", __func__);
        return 0;
 
exit:
        sx9310_i2c_client = NULL;           
        SAR_ERR("%s: err = %d\n", __func__, err);
        return err;
}
/*----------------------------------------------------------------------------*/
static int sx9310_i2c_remove(struct i2c_client *client)
{
        int err = 0;
        err = sx9310_delete_attr(&sx9310_driver.driver);
        if(err)
        {
        	SAR_ERR("sx9310_delete_attr fail: %d\n", err);
        }
        sx9310_i2c_client = NULL;
        i2c_unregister_device(client);
        kfree(i2c_get_clientdata(client));

        return 0;
}
/*----------------------------------------------------------------------------*/
static int sx9310_probe(struct platform_device *pdev)
{
        SAR_FUN();
        sar_pltfm_dev = pdev;
        i2c_add_driver(&sx9310_i2c_driver);
        return 0;
}
/*----------------------------------------------------------------------------*/
static int sx9310_remove(struct platform_device *pdev)
{
	SAR_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init sx9310_init(void)
{
        SAR_FUN();
        platform_driver_register(&sx9310_driver);
        return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit sx9310_exit(void)
{
        SAR_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(sx9310_init);
module_exit(sx9310_exit);
/*----------------------------------------------------------------------------*/
MODULE_DESCRIPTION("SX9310 SAR SENSOR Driver");
MODULE_LICENSE("GPL");

