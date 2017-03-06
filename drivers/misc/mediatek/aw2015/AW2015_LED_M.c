/*************************************************************************************************
*  AW2015_LED.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
*************************************************************************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AW2015_I2C_NAME		"pressure" 
#define AW2015_I2C_BUS		0
#define AW2015_I2C_ADDR		0x64
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

static ssize_t AW2015_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW2015_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_set_debug(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_get_debug(struct device* cd, struct device_attribute *attr, char* buf);
static ssize_t AW2015_Set_Red(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Green(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Blue(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Red_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Red_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Green_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Green_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Blue_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Blue_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_White(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_White_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_White_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Gray(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Gray_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Gray_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Yellow(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Yellow_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Yellow_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Pink(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Pink_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Pink_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Orange(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Orange_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Orange_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Violet(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Violet_delay_off(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2015_Set_Violet_delay_on(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);




static DEVICE_ATTR(reg, 0660, AW2015_get_reg,  AW2015_set_reg);
static DEVICE_ATTR(debug, 0660, AW2015_get_debug,  AW2015_set_debug);

static DEVICE_ATTR(Red, 0660, NULL,  AW2015_Set_Red);
static DEVICE_ATTR(Green, 0660, NULL,  AW2015_Set_Green);
static DEVICE_ATTR(Blue, 0660, NULL,  AW2015_Set_Blue);
static DEVICE_ATTR(White, 0660, NULL,  AW2015_Set_White);
static DEVICE_ATTR(Gray, 0660, NULL,  AW2015_Set_Gray);
static DEVICE_ATTR(Yellow, 0660, NULL,  AW2015_Set_Yellow);
static DEVICE_ATTR(Pink, 0660, NULL,  AW2015_Set_Pink);
static DEVICE_ATTR(Orange, 0660, NULL,  AW2015_Set_Orange);
static DEVICE_ATTR(Violet, 0660, NULL,  AW2015_Set_Violet);


static int RED_DELAY_OFF = 0,RED_DELAY_ON = 0;
static DEVICE_ATTR(Red_delay_off, 0660, NULL,  AW2015_Set_Red_delay_off);
static DEVICE_ATTR(Red_delay_on, 0660, NULL,  AW2015_Set_Red_delay_on);
static int GREEN_DELAY_OFF = 0,GREEN_DELAY_ON = 0;
static DEVICE_ATTR(Green_delay_off, 0660, NULL,  AW2015_Set_Green_delay_off);
static DEVICE_ATTR(Green_delay_on, 0660, NULL,  AW2015_Set_Green_delay_on);
static int BLUE_DELAY_OFF = 0,BLUE_DELAY_ON = 0;
static DEVICE_ATTR(Blue_delay_off, 0660, NULL,  AW2015_Set_Blue_delay_off);
static DEVICE_ATTR(Blue_delay_on, 0660, NULL,  AW2015_Set_Blue_delay_on);

static int WHITE_DELAY_OFF = 0,WHITE_DELAY_ON = 0;
static DEVICE_ATTR(White_delay_off, 0660, NULL,  AW2015_Set_White_delay_off);
static DEVICE_ATTR(White_delay_on, 0660, NULL,  AW2015_Set_White_delay_on);

static int GRAY_DELAY_OFF = 0,GRAY_DELAY_ON = 0;
static DEVICE_ATTR(Gray_delay_off, 0660, NULL,  AW2015_Set_Gray_delay_off);
static DEVICE_ATTR(Gray_delay_on, 0660, NULL,  AW2015_Set_Gray_delay_on);

static int YELLOW_DELAY_OFF = 0,YELLOW_DELAY_ON = 0;
static DEVICE_ATTR(Yellow_delay_off, 0660, NULL,  AW2015_Set_Yellow_delay_off);
static DEVICE_ATTR(Yellow_delay_on, 0660, NULL,  AW2015_Set_Yellow_delay_on);

static int PINK_DELAY_OFF = 0,PINK_DELAY_ON = 0;
static DEVICE_ATTR(Pink_delay_off, 0660, NULL,  AW2015_Set_Pink_delay_off);
static DEVICE_ATTR(Pink_delay_on, 0660, NULL,  AW2015_Set_Pink_delay_on);

static int ORANGE_DELAY_OFF = 0,ORANGE_DELAY_ON = 0;
static DEVICE_ATTR(Orange_delay_off, 0660, NULL,  AW2015_Set_Orange_delay_off);
static DEVICE_ATTR(Orange_delay_on, 0660, NULL,  AW2015_Set_Orange_delay_on);

static int VIOLET_DELAY_OFF = 0,VIOLET_DELAY_ON = 0;
static DEVICE_ATTR(Violet_delay_off, 0660, NULL,  AW2015_Set_Violet_delay_off);
static DEVICE_ATTR(Violet_delay_on, 0660, NULL,  AW2015_Set_Violet_delay_on);

struct i2c_client *AW2015_i2c_client;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char I2C_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};


	
	struct i2c_msg msg[] = {
            {
			.addr	= AW2015_i2c_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
	        },
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(AW2015_i2c_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	int ret = 0;
	u8 rdbuf[512] = {0};


	struct i2c_msg msg[2] = {
		{
			.addr	= AW2015_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= AW2015_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;
	
	ret = i2c_transfer(AW2015_i2c_client->adapter, msg, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AW2015 LED 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int ms2timer(unsigned int time)
{
	int i;
	unsigned int ref[16] = {4, 128, 256, 384, 512, 762, 1024, 1524, 2048, 2560, 3072, 4096, 5120, 6144, 7168, 8192};
	
	for(i=0; i<15; i++)
	{
		if(time <= ref[0])
		{
			return 0;
		}
		else if(time > ref[15])
		{
			return 15;
		}
		else if((time>ref[i]) && (time<=ref[i+1]))
		{
			if((time-ref[i]) <= (ref[i+1]-time))
			{
				return i;
			}
			else
			{
				return (i+1);
			}
		}
	}
    return 0;
}	


unsigned char AW2015_LED_ON(unsigned char r, unsigned char g, unsigned char b)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	
	I2C_write_reg(0x10, r	);		// ILED1
	I2C_write_reg(0x11, g	);		// ILED2
	I2C_write_reg(0x12, b	);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	return 0;
}

unsigned char AW2015_LED_OFF(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset
    I2C_write_reg(0x10, 0x00);      // Color1_R
    I2C_write_reg(0x11, 0x00);      // Color1_G
    I2C_write_reg(0x12, 0x00);      // Color1_B
    I2C_write_reg(0x1C, 0x00);      // PWM1
    I2C_write_reg(0x1D, 0x00);      // PWM2
    I2C_write_reg(0x1E, 0x00);      // PWM3

	return 0;
}
EXPORT_SYMBOL(AW2015_LED_OFF);

static int RED_DELAY_OFF_BEFORE;
static int RED_DELAY_ON_BEFORE;
// set red delay off and on to 0 by xmwuwh@20161122
unsigned char AW2015_LED_RED_ON(void)
{
    RED_DELAY_OFF_BEFORE = RED_DELAY_OFF;
    RED_DELAY_ON_BEFORE = RED_DELAY_ON;
    RED_DELAY_OFF = 0;
    RED_DELAY_ON = 0;
    I2C_write_reg(0x00, 0x55);      // software reset

    I2C_write_reg(0x01, 0x03);      // GCR
    I2C_write_reg(0x03, 0x01);      // IMAX
    I2C_write_reg(0x04, 0x00);      // LCFG1
    I2C_write_reg(0x05, 0x00);      // LCFG2
    I2C_write_reg(0x06, 0x00);      // LCFG3
    I2C_write_reg(0x07, 0x07);      // LEDEN

    I2C_write_reg(0x10, 0xFF);		// Color1_R
    I2C_write_reg(0x11, 0x00);		// Color1_G
    I2C_write_reg(0x12, 0x00);		// Color1_B
    I2C_write_reg(0x1C, 0xFF);		// PWM1
    I2C_write_reg(0x1D, 0xFF);		// PWM2
    I2C_write_reg(0x1E, 0xFF);		// PWM3
    I2C_write_reg(0x09, 0x07);		// PAT_RIN
    return 0;
}
EXPORT_SYMBOL(AW2015_LED_RED_ON);

// set red delay off and on to before value  by xmwuwh@20161122
unsigned char AW2015_LED_RED_OFF(void)
{
    RED_DELAY_OFF = RED_DELAY_OFF_BEFORE;
    RED_DELAY_ON = RED_DELAY_ON_BEFORE;
    return 0;
}
EXPORT_SYMBOL(AW2015_LED_RED_OFF);


unsigned char AW2015_LED_Blink(unsigned char r, unsigned char g, unsigned char b, unsigned int trise_ms, unsigned int ton_ms, unsigned int tfall_ms, unsigned int toff_ms)
{
	unsigned char trise, ton, tfall, toff;
	
	trise = ms2timer(trise_ms);
	ton   = ms2timer(ton_ms);
	tfall = ms2timer(tfall_ms);
	toff  = ms2timer(toff_ms);
	
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, r	);		// ILED1
	I2C_write_reg(0x11, g	);		// ILED2
	I2C_write_reg(0x12, b	);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, (trise<<4)|ton);	// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, (tfall<<4)|toff);	// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x00);		// PAT_T3				Tdelay
	I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}

static ssize_t AW2015_Set_Red_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    RED_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Red_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    RED_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Green_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    GREEN_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Green_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    GREEN_DELAY_ON = iNewPsSensorState;
    return len;
}
static ssize_t AW2015_Set_Blue_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    BLUE_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Blue_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    BLUE_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_White_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    WHITE_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_White_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    WHITE_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Gray_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    GRAY_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Gray_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    GRAY_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Yellow_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    YELLOW_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Yellow_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    YELLOW_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Pink_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    PINK_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Pink_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    PINK_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Orange_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    ORANGE_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Orange_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    ORANGE_DELAY_ON = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Violet_delay_off(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    VIOLET_DELAY_OFF = iNewPsSensorState;
    return len;
}

static ssize_t AW2015_Set_Violet_delay_on(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned long iNewPsSensorState = simple_strtoul(buf, NULL, 10);
    VIOLET_DELAY_ON = iNewPsSensorState;
    return len;
}


static ssize_t AW2015_Set_Orange(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(ORANGE_DELAY_OFF == 500)
        {
            if(ORANGE_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xCE);      // Color1_G
                I2C_write_reg(0x12, 0x1F);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(ORANGE_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xCE);      // Color1_G
                I2C_write_reg(0x12, 0x1F);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(ORANGE_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xCE);      // Color1_G
                I2C_write_reg(0x12, 0x1F);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }   else if(ORANGE_DELAY_ON!=0 &&ORANGE_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xCE);      // Color1_G
                I2C_write_reg(0x12, 0x1F);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
        }
        else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0xFF);		// Color1_R
            I2C_write_reg(0x11, 0xCE);		// Color1_G
            I2C_write_reg(0x12, 0x1F);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        ORANGE_DELAY_ON = 0;
        ORANGE_DELAY_OFF = 0;
    }
	return len;

}


static ssize_t AW2015_Set_Violet(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(VIOLET_DELAY_OFF == 500)
        {
            if(VIOLET_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xD2);      // Color1_R
                I2C_write_reg(0x11, 0x74);      // Color1_G
                I2C_write_reg(0x12, 0xF9);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(VIOLET_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xD2);      // Color1_R
                I2C_write_reg(0x11, 0x74);      // Color1_G
                I2C_write_reg(0x12, 0xF9);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(VIOLET_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xD2);      // Color1_R
                I2C_write_reg(0x11, 0x74);      // Color1_G
                I2C_write_reg(0x12, 0xF9);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }   else if(VIOLET_DELAY_ON!=0 &&VIOLET_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xD2);      // Color1_R
                I2C_write_reg(0x11, 0x74);      // Color1_G
                I2C_write_reg(0x12, 0xF9);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
        }
        else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0xD2);		// Color1_R
            I2C_write_reg(0x11, 0x74);		// Color1_G
            I2C_write_reg(0x12, 0xF9);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        VIOLET_DELAY_ON = 0;
        VIOLET_DELAY_OFF = 0;
    }
	return len;

}

static ssize_t AW2015_Set_White(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(WHITE_DELAY_OFF == 500)
        {
            if(WHITE_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(WHITE_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(WHITE_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }   else if(WHITE_DELAY_ON!=0 &&WHITE_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
        }
        else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0xFF);		// Color1_R
            I2C_write_reg(0x11, 0xFF);		// Color1_G
            I2C_write_reg(0x12, 0xFF);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        WHITE_DELAY_ON = 0;
        WHITE_DELAY_OFF = 0;
    }
	return len;

}

static ssize_t AW2015_Set_Gray(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(GRAY_DELAY_OFF == 500)
        {
            if(GRAY_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x80);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(GRAY_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x80);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(GRAY_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x80);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }else if(GRAY_DELAY_ON!=0 && GRAY_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x80);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0x00);		// Color1_R
            I2C_write_reg(0x11, 0xFF);		// Color1_G
            I2C_write_reg(0x12, 0x80);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        GRAY_DELAY_ON = 0;
        GRAY_DELAY_OFF = 0;
    }
	return len;

}

static ssize_t AW2015_Set_Yellow(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(YELLOW_DELAY_OFF == 500)
        {
            if(YELLOW_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(YELLOW_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(YELLOW_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }else if(YELLOW_DELAY_ON!=0 && YELLOW_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0xFF);		// Color1_R
            I2C_write_reg(0x11, 0xFF);		// Color1_G
            I2C_write_reg(0x12, 0x00);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        YELLOW_DELAY_ON = 0;
        YELLOW_DELAY_OFF = 0;
    }
	return len;

}

static ssize_t AW2015_Set_Pink(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)

{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(PINK_DELAY_OFF == 500)
        {
            if(PINK_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(PINK_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(PINK_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }else if(PINK_DELAY_ON!=0 &&PINK_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN

            I2C_write_reg(0x10, 0xFF);		// Color1_R
            I2C_write_reg(0x11, 0x00);		// Color1_G
            I2C_write_reg(0x12, 0xFF);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        PINK_DELAY_ON = 0;
        PINK_DELAY_OFF = 0;
    }
	return len;

}

static ssize_t AW2015_Set_Red(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);

    if(databuf[0]!=0)
    {
        if(RED_DELAY_OFF == 500)
        {
            if(RED_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0x00);      // PWM2
                I2C_write_reg(0x1E, 0x00);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN
                
                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(RED_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0x00);      // PWM2
                I2C_write_reg(0x1E, 0x00);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(RED_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0x00);      // PWM2
                I2C_write_reg(0x1E, 0x00);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }else if(RED_DELAY_ON!=0 && RED_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0xFF);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0x00);      // PWM2
                I2C_write_reg(0x1E, 0x00);      // PWM3
                I2C_write_reg(0x09, 0x07);      // PAT_RIN

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN
    
            I2C_write_reg(0x10, 0xFF);		// Color1_R
            I2C_write_reg(0x11, 0x00);		// Color1_G
            I2C_write_reg(0x12, 0x00);		// Color1_B
            I2C_write_reg(0x1C, 0xFF);		// PWM1
            I2C_write_reg(0x1D, 0xFF);		// PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
    else
    {
        I2C_write_reg(0x00, 0x55);		// software reset

    	I2C_write_reg(0x01, 0x03);		// GCR
    	I2C_write_reg(0x03, 0x01);		// IMAX
    	I2C_write_reg(0x04, 0x00);		// LCFG1
    	I2C_write_reg(0x05, 0x00);		// LCFG2
    	I2C_write_reg(0x06, 0x00);		// LCFG3
    	I2C_write_reg(0x07, 0x07);		// LEDEN
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        RED_DELAY_ON = 0;
        RED_DELAY_OFF = 0;
    }
	return len;

}


static ssize_t AW2015_Set_Green(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(GREEN_DELAY_OFF == 500)
        {
            if(GREEN_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(GREEN_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(GREEN_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0xFF);      // Color1_G
                I2C_write_reg(0x12, 0x00);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }
        else if (GREEN_DELAY_ON!=0 &&GREEN_DELAY_OFF!=0)
         {
             I2C_write_reg(0x00, 0x55);      // software reset

             I2C_write_reg(0x01, 0x03);      // GCR
             I2C_write_reg(0x03, 0x01);      // IMAX
             I2C_write_reg(0x04, 0x01);      // LCFG1
             I2C_write_reg(0x05, 0x01);      // LCFG2
             I2C_write_reg(0x06, 0x01);      // LCFG3
             I2C_write_reg(0x07, 0x07);      // LEDEN
             I2C_write_reg(0x08, 0x08);      // LEDCTR

             I2C_write_reg(0x10, 0x00);      // Color1_R
             I2C_write_reg(0x11, 0xFF);      // Color1_G
             I2C_write_reg(0x12, 0x00);      // Color1_B
             I2C_write_reg(0x1C, 0xFF);      // PWM1
             I2C_write_reg(0x1D, 0xFF);      // PWM2
             I2C_write_reg(0x1E, 0xFF);      // PWM3

             I2C_write_reg(0x30, 0x14);      // PAT_T1       Trise & Ton
             I2C_write_reg(0x31, 0x01);      // PAT_T2       Tfall & Toff
             I2C_write_reg(0x32, 0x31);      // PAT_T3       Tslot & Tdelay
             I2C_write_reg(0x33, 0x10);      // PAT_T4     PAT_CTR & Color
             I2C_write_reg(0x34, 0x00);      // PAT_T5           Timer

             I2C_write_reg(0x09, 0x07);      // PAT_RIN
         }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN
            I2C_write_reg(0x08, 0x08);      // LEDCTR

            I2C_write_reg(0x10, 0x00);      // Color1_R
            I2C_write_reg(0x11, 0xFF);      // Color1_G
            I2C_write_reg(0x12, 0x00);      // Color1_B
            I2C_write_reg(0x1C, 0xFF);      // PWM1
            I2C_write_reg(0x1D, 0xFF);      // PWM2
            I2C_write_reg(0x1E, 0xFF);		// PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }
 
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        GREEN_DELAY_ON = 0;
        GREEN_DELAY_OFF = 0;
    }
    return len;

}

unsigned char AW2015_Set_Green_on_off(int status)
{

    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(status!=0)
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0xFF);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0xFF);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
    }
    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
    }
    return 0;

}
EXPORT_SYMBOL(AW2015_Set_Green_on_off);


static ssize_t AW2015_Set_Blue(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
    unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
    I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN

    if(databuf[0]!=0)
    {
        if(BLUE_DELAY_OFF == 500)
        {
            if(BLUE_DELAY_ON == 500)
            {
                I2C_write_reg(0x00, 0x55);		// software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x00);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x02);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer


                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(BLUE_DELAY_ON == 1000)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x13);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x20);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }
            else if(BLUE_DELAY_ON == 1499)
            {
                I2C_write_reg(0x00, 0x55);      // software reset
                
                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x00);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x00);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN
            }
        }else if(BLUE_DELAY_ON!=0 &&BLUE_DELAY_OFF!=0)
            {
                I2C_write_reg(0x00, 0x55);      // software reset

                I2C_write_reg(0x01, 0x03);      // GCR
                I2C_write_reg(0x03, 0x01);      // IMAX
                I2C_write_reg(0x04, 0x01);      // LCFG1
                I2C_write_reg(0x05, 0x01);      // LCFG2
                I2C_write_reg(0x06, 0x01);      // LCFG3
                I2C_write_reg(0x07, 0x07);      // LEDEN
                I2C_write_reg(0x08, 0x08);      // LEDCTR

                I2C_write_reg(0x10, 0x00);      // Color1_R
                I2C_write_reg(0x11, 0x00);      // Color1_G
                I2C_write_reg(0x12, 0xFF);      // Color1_B
                I2C_write_reg(0x1C, 0xFF);      // PWM1
                I2C_write_reg(0x1D, 0xFF);      // PWM2
                I2C_write_reg(0x1E, 0xFF);      // PWM3

                I2C_write_reg(0x30, 0x14);		// PAT_T1		Trise & Ton
                I2C_write_reg(0x31, 0x01);		// PAT_T2		Tfall & Toff
                I2C_write_reg(0x32, 0x31);		// PAT_T3		Tslot & Tdelay
                I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
                I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer

                I2C_write_reg(0x09, 0x07);		// PAT_RIN	
            }else
        {
            I2C_write_reg(0x00, 0x55);      // software reset
            
            I2C_write_reg(0x01, 0x03);      // GCR
            I2C_write_reg(0x03, 0x01);      // IMAX
            I2C_write_reg(0x04, 0x00);      // LCFG1
            I2C_write_reg(0x05, 0x00);      // LCFG2
            I2C_write_reg(0x06, 0x00);      // LCFG3
            I2C_write_reg(0x07, 0x07);      // LEDEN
            I2C_write_reg(0x08, 0x08);      // LEDCTR

            I2C_write_reg(0x10, 0x00);      // Color1_R
            I2C_write_reg(0x11, 0x00);      // Color1_G
            I2C_write_reg(0x12, 0xFF);      // Color1_B
            I2C_write_reg(0x1C, 0xFF);      // PWM1
            I2C_write_reg(0x1D, 0xFF);      // PWM2
            I2C_write_reg(0x1E, 0xFF);      // PWM3
            I2C_write_reg(0x09, 0x07);		// PAT_RIN
        }
        
    }

    else
    {
        I2C_write_reg(0x10, 0x00);		// Color1_R
        I2C_write_reg(0x11, 0x00);		// Color1_G
        I2C_write_reg(0x12, 0x00);		// Color1_B
        I2C_write_reg(0x1C, 0x00);		// PWM1
        I2C_write_reg(0x1D, 0x00);		// PWM2
        I2C_write_reg(0x1E, 0x00);		// PWM3
        I2C_write_reg(0x09, 0x07);		// PAT_RIN
        BLUE_DELAY_ON = 0;
        BLUE_DELAY_OFF = 0;

    }
    return len;

}

///////////////////////////////////////////////////////////////////////////////////////////
// |-|_|-|______|-|_|-|______|-|_|-|______
///////////////////////////////////////////////////////////////////////////////////////////
unsigned char AW2015_Multi_Pulse(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, 0xFF);		// Color1_R
	I2C_write_reg(0x11, 0x00);		// Color1_G
	I2C_write_reg(0x12, 0x00);		// Color1_B
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, 0x03);		// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, 0x08);		// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x30);		// PAT_T3		Tslot & Tdelay
	I2C_write_reg(0x33, 0x10);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//  R -> G -> B -> W -> R ... 
//////////////////////////////////////////////////////////////////////////////////////////
unsigned char AW2015_Multi_Color(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, 0xFF);		// Color1_R
	I2C_write_reg(0x11, 0x00);		// Color1_G
	I2C_write_reg(0x12, 0x00);		// Color1_B
	I2C_write_reg(0x13, 0x00);		// Color2_R
	I2C_write_reg(0x14, 0xFF);		// Color2_G
	I2C_write_reg(0x15, 0x00);		// Color2_B
	I2C_write_reg(0x16, 0x00);		// Color3_R
	I2C_write_reg(0x17, 0x00);		// Color3_G
	I2C_write_reg(0x18, 0xFF);		// Color3_B
	I2C_write_reg(0x19, 0xFF);		// Color4_R
	I2C_write_reg(0x1A, 0xFF);		// Color4_G
	I2C_write_reg(0x1B, 0xFF);		// Color4_B
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, 0x60);		// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, 0x64);		// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x00);		// PAT_T3				Tdelay
	I2C_write_reg(0x33, 0x0F);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//         S                            O                              S   
//       . . .                         - - -                          . . .
// |-|__|-|__|-|_______      |---|__|---|__|---|_______         |-|__|-|__|-|_______
//////////////////////////////////////////////////////////////////////////////////////////
unsigned char AW2015_SOS(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x03);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, 0xFF);		// Color1_R
	I2C_write_reg(0x11, 0x00);		// Color1_G
	I2C_write_reg(0x12, 0x00);		// Color1_B
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, 0x04);		// PAT1_T1		Trise & Ton
	I2C_write_reg(0x31, 0x0B);		// PAT1_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x70);		// PAT1_T3		Tslot & Tdelay
	I2C_write_reg(0x33, 0xE0);		// PAT1_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT1_T5		    Timer
	I2C_write_reg(0x35, 0x07);		// PAT2_T1		Trise & Ton
	I2C_write_reg(0x36, 0x0B);		// PAT2_T2		Tfall & Toff
	I2C_write_reg(0x37, 0x70);		// PAT2_T3		Tslot & Tdelay
	I2C_write_reg(0x38, 0xE0);		// PAT2_T4 	  PAT_CTR & Color
	I2C_write_reg(0x39, 0x00);		// PAT2_T5		    Timer
	I2C_write_reg(0x3A, 0x04);		// PAT3_T1		Trise & Ton
	I2C_write_reg(0x3B, 0x0B);		// PAT3_T2		Tfall & Toff
	I2C_write_reg(0x3C, 0x70);		// PAT3_T3		Tslot & Tdelay
	I2C_write_reg(0x3D, 0xE0);		// PAT3_T4 	  PAT_CTR & Color
	I2C_write_reg(0x3E, 0x00);		// PAT3_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t AW2015_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<0x3A;i++)
	{
		reg_val = I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val);
	}

	return len;
}

static ssize_t AW2015_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		I2C_write_reg(databuf[0],databuf[1]);
	}
	return len;
}


static ssize_t AW2015_get_debug(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_LED_OFF(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_LED_ON(r, g, b)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1  r   g   b  > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 255 255 255 > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_LED_Blink(r, g, b, trise, ton, tfall, tfall)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 2  r   g   b  trise ton tfall toff > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 2 255 255 255 1000    0 1000  1000 > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_Multi_Pulse(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 3  > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_Multi_Color(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 4  > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "AW2015_SOS(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 5  > debug\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}
static ssize_t AW2015_set_debug(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[16];
	sscanf(buf,"%d",&databuf[0]);
	if(databuf[0] == 0) {		// OFF
		AW2015_LED_OFF();	
	} else if(databuf[0] == 1) {	//ON
		sscanf(&buf[1], "%d %d %d", &databuf[1], &databuf[2], &databuf[3]);
		AW2015_LED_ON(databuf[1], databuf[2], databuf[3]);
	} else if(databuf[0] == 2) {	//Blink
		sscanf(&buf[1], "%d %d %d %d %d %d %d", &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5], &databuf[6], &databuf[7]);
		AW2015_LED_Blink(databuf[1], databuf[2], databuf[3], databuf[4], databuf[5], databuf[6], databuf[7]);
	} else if(databuf[0] == 3) {	//Multi Pulse
		AW2015_Multi_Pulse();
	} else if(databuf[0] == 4) {	//Multi Color
		AW2015_Multi_Color();
	} else if(databuf[0] == 5) {	//SOS
		AW2015_SOS();
	}
	return len;
}


static int AW2015_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_debug);
	err = device_create_file(dev, &dev_attr_Red);
	err = device_create_file(dev, &dev_attr_Green);
    err = device_create_file(dev, &dev_attr_Blue);
    err = device_create_file(dev, &dev_attr_Red_delay_off);
    err = device_create_file(dev, &dev_attr_Red_delay_on);
    err = device_create_file(dev, &dev_attr_Green_delay_off);
    err = device_create_file(dev, &dev_attr_Green_delay_on);
    err = device_create_file(dev, &dev_attr_Blue_delay_off);
    err = device_create_file(dev, &dev_attr_Blue_delay_on);
    err = device_create_file(dev, &dev_attr_White);
    err = device_create_file(dev, &dev_attr_White_delay_off);
    err = device_create_file(dev, &dev_attr_White_delay_on);
    err = device_create_file(dev, &dev_attr_Gray);
    err = device_create_file(dev, &dev_attr_Gray_delay_off);
    err = device_create_file(dev, &dev_attr_Gray_delay_on);
    err = device_create_file(dev, &dev_attr_Yellow);
    err = device_create_file(dev, &dev_attr_Yellow_delay_off);
    err = device_create_file(dev, &dev_attr_Yellow_delay_on);
    err = device_create_file(dev, &dev_attr_Pink);
    err = device_create_file(dev, &dev_attr_Pink_delay_off);
    err = device_create_file(dev, &dev_attr_Pink_delay_on);
    err = device_create_file(dev, &dev_attr_Orange);
    err = device_create_file(dev, &dev_attr_Orange_delay_off);
    err = device_create_file(dev, &dev_attr_Orange_delay_on);
    err = device_create_file(dev, &dev_attr_Violet);
    err = device_create_file(dev, &dev_attr_Violet_delay_off);
    err = device_create_file(dev, &dev_attr_Violet_delay_on);

	return err;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int AW2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_value;
	unsigned char cnt = 5;
	int err = 0;
	//pr_err("AW2015_i2c_probe xmwuwh start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
    //client->addr = 0x64;
	AW2015_i2c_client = client;

	while(cnt>0)
	{
		reg_value = I2C_read_reg(0x00);
		printk("AW2015 CHIPID=0x%2x\n", reg_value);
		if(reg_value == 0x31)
		{
			break;
		}
		cnt --;
		msleep(10);
	}
	if(!cnt)
	{
		err = -ENODEV;
		goto exit_create_singlethread;
	}

	AW2015_create_sysfs(client);	
	I2C_write_reg(0x00, 0x55);		// software reset
	return 0;

exit_create_singlethread:
	AW2015_i2c_client = NULL;
exit_check_functionality_failed:
	return err;	
}

static int AW2015_i2c_remove(struct i2c_client *client)
{
	AW2015_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id AW2015_i2c_id[] = {
	{ AW2015_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id aw2015_i2c_of_match[] = {
	{.compatible = "awinic,pressure"},
	{},
};
#endif

static struct i2c_driver AW2015_i2c_driver = {
        .driver = {
                .name   = AW2015_I2C_NAME,
#ifdef CONFIG_OF
				.of_match_table = aw2015_i2c_of_match,
#endif
		},
        .probe          = AW2015_i2c_probe,
        .remove         = AW2015_i2c_remove,
        .id_table       = AW2015_i2c_id,
};


static int AW2015_led_remove(struct platform_device *pdev)
{
	printk("AW2015 remove\n");
	i2c_del_driver(&AW2015_i2c_driver);
	return 0;
}

static int AW2015_led_probe(struct platform_device *pdev)
{
	int ret;

	printk("%s 1 start!\n", __func__);
		
	ret = i2c_add_driver(&AW2015_i2c_driver);
	if (ret != 0) {
		printk("[%s] failed to register AW2015 i2c driver.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to register AW2015 i2c driver.\n", __func__);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw2015plt_of_match[] = {
	{.compatible = "awinic,aw2015_led"},
	{},
};
#endif
//static void aw2015_led_shutdown(struct platform_device *dev)
//{
//    AW2015_LED_OFF();
//	pr_err("******** xmwuwh aw2015_led_shutdown!! ********\n");

//}

static struct platform_driver AW2015_led_driver = {
		.probe	 = AW2015_led_probe,
		.remove	 = AW2015_led_remove,
		//.shutdown = aw2015_led_shutdown,
        .driver = {
                .name   = "aw2015_led",
#ifdef CONFIG_OF
				.of_match_table = aw2015plt_of_match,
#endif
        }
};

static int __init AW2015_led_init(void) {
	int ret;
	//pr_err("%s xmwuwh start\n", __func__);
	
	ret = platform_driver_register(&AW2015_led_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}		
	return 0;
}

static void __exit AW2015_led_exit(void) {
	printk("%s exit\n", __func__);
	platform_driver_unregister(&AW2015_led_driver);	
}

module_init(AW2015_led_init);
module_exit(AW2015_led_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW2015 LED Driver");
MODULE_LICENSE("GPL");

