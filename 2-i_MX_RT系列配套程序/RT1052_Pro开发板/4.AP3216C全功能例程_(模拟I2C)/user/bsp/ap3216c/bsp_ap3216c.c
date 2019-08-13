/**
  ******************************************************************************
  * @file    ap3216.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   AP3216C 驱动文件
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "./ap3216c/bsp_ap3216c.h"
#include "./i2c/bsp_i2c.h"
#include "./delay/core_delay.h"   
#include "./bsp/nvic/bsp_nvic.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
#include "pad_config.h"  
/*******************************************************************************
 * 宏
 ******************************************************************************/
#define HAL_Delay 		CPU_TS_Tmr_Delay_MS
/* 所有引脚均使用同样的PAD配置 */
#define INT_PAD_CONFIG_DATA            (SRE_0_SLOW_SLEW_RATE| \
                                        DSE_0_OUTPUT_DRIVER_DISABLED| \
                                        SPEED_2_MEDIUM_100MHz| \
                                        ODE_0_OPEN_DRAIN_DISABLED| \
                                        PKE_1_PULL_KEEPER_ENABLED| \
                                        PUE_1_PULL_SELECTED| \
                                        PUS_3_22K_OHM_PULL_UP| \
                                        HYS_1_HYSTERESIS_ENABLED)   

    /* 配置说明 : */
    /* 转换速率: 转换速率慢
      驱动强度: 关闭
      速度配置 : medium(100MHz)
      开漏配置: 关闭 
      拉/保持器配置: 使能
      拉/保持器选择: 上下拉
      上拉/下拉选择: 22K欧姆上拉
      滞回器配置: 开启 （仅输入时有效，施密特触发器，使能后可以过滤输入噪声）*/
/*******************************************************************************
 * 声明
 ******************************************************************************/
static void  INT_IOMUXC_MUX_Config(void);
static void  INT_IOMUXC_PAD_Config(void);
static void  INT_GPIO_Mode_Config(void);

/**
* @brief  初始化INT相关IOMUXC的MUX复用配置
* @param  无
* @retval 无
*/
static void INT_IOMUXC_MUX_Config(void)
{ 
  /* 设置引脚的复用模式为GPIO，不使用SION功能 */
  IOMUXC_SetPinMux(INT_IOMUXC, 0U);
}

/**
* @brief  初始化INT相关IOMUXC的MUX复用配置
* @param  无
* @retval 无
*/
static void INT_IOMUXC_PAD_Config(void)
{
  /* 中断引脚 */    
  IOMUXC_SetPinConfig(INT_IOMUXC, INT_PAD_CONFIG_DATA);  
}

 /**
  * @brief  初始化INT相关的GPIO模式
  * @param  无
  * @retval 无
  */
static void INT_GPIO_Mode_Config(void)
{     
  /* 定义gpio初始化配置结构体 */
  gpio_pin_config_t io_config;      
   
  io_config.direction = kGPIO_DigitalInput; //输入模式
  io_config.outputLogic =  1;                //默认高电平
	io_config.interruptMode = kGPIO_IntFallingEdge; //下降沿中断
	
  /* 初始化 INT GPIO. */
  GPIO_PinInit(INT_GPIO, INT_GPIO_PIN, &io_config);
}

/**
 * @brief  初始化中断相关的内容
 * @param  无
 * @retval 无
 */
static void INT_Interrupt_Config(void)   
{
  /* 开启GPIO引脚的中断 */
  GPIO_PortEnableInterrupts(INT_GPIO, 1U << INT_GPIO_PIN);                             
  /*设置中断优先级,*/
	set_IRQn_Priority(INT_IRQ,Group4_PreemptPriority_6, Group4_SubPriority_1);  
  /* 开启GPIO端口中断 */
  EnableIRQ(INT_IRQ);
}


/**
  * @brief  初始化控制中断的IO
  * @param  无
  * @retval 无
  */
void INT_GPIO_Config(void)
{
  /* 初始化GPIO复用、属性、模式 以及中断*/
  INT_IOMUXC_MUX_Config();
  INT_IOMUXC_PAD_Config();
  INT_GPIO_Mode_Config();
	INT_Interrupt_Config();
}


/* 写寄存器的值 */
static void write_reg(uint8_t reg, uint8_t data)
{
  Sensor_write(reg, data);
}

/* 读寄存器的值 */
static void read_regs(uint8_t reg, uint8_t len, uint8_t *buf)
{
  Sensor_Read(reg, buf, len);
}

/* 软件复位传感器 */
static void reset_sensor(void)
{
    write_reg(AP3216C_SYS_CONFIGURATION_REG, AP3216C_MODE_SW_RESET); //reset
}

/**
 * This function is convenient to getting data except including high and low data for this sensor.
 * note:after reading lower register first,reading higher add one.
 */
static uint32_t read_low_and_high(uint8_t reg, uint8_t len)
{
    uint32_t data;
    uint8_t buf = 0;

    read_regs( reg, len, &buf);        // 读低字节
    data = buf;
    read_regs( reg + 1, len, &buf);    // 读高字节
    data = data + (buf << len * 8);    // 合并数据

    return data;
}


/**
 * This function is only used to set threshold without filtering times
 *
 * @param cmd first register , and other cmd count by it.
 * @param threshold threshold and filtering times of als threshold
 */
static void set_als_threshold(ap3216c_cmd_t cmd, ap3216c_threshold_t threshold)
{
    uint8_t Resolution;
    double DB;

    /* 读光照强度的范围 */
    ap3216c_get_param(AP3216C_ALS_RANGE, &Resolution);

    if (Resolution == AP3216C_ALS_RANGE_20661)     // 光照强度范围 0 - 20661
    {
        DB = 0.35;    // 光照强度的分辨率
    }
    else if (Resolution == AP3216C_ALS_RANGE_5162)     // 光照强度范围 0 - 5162
    {
        DB = 0.0788;    // 光照强度的分辨率
    }
    else if (Resolution == AP3216C_ALS_RANGE_1291)     // 光照强度范围 0 - 1291
    {
        DB = 0.0197;    // 光照强度的分辨率
    }
    else if (Resolution == AP3216C_ALS_RANGE_323)     // 光照强度范围 0 - 323
    {
        DB = 0.0049;    // 光照强度的分辨率
    }
  
    threshold.min /= DB;    // 根据不同的分辨率来设置
    threshold.max /= DB;
      
    ap3216c_set_param(cmd, (threshold.min & 0xff));
    ap3216c_set_param((ap3216c_cmd_t)(cmd + 1), (threshold.min >> 8));
    ap3216c_set_param((ap3216c_cmd_t)(cmd + 2), (threshold.max & 0xff));
    ap3216c_set_param((ap3216c_cmd_t)(cmd + 3), threshold.max >> 8);
}

static void set_ps_threshold(ap3216c_cmd_t cmd, ap3216c_threshold_t threshold)
{
    if (threshold.min > 1020)    // 大于1020 时需要设置低字节的低两位
      ap3216c_set_param(cmd, (threshold.min - 1020 & 0x03));
    
    ap3216c_set_param((ap3216c_cmd_t)(cmd + 1), threshold.min/4);    // 设置高字节参数
    
    if (threshold.max > 1020)    // 大于1020 时需要设置低字节的低两位
      ap3216c_set_param((ap3216c_cmd_t)(cmd + 2), (threshold.max - 1020 & 0x03));
    
    ap3216c_set_param((ap3216c_cmd_t)(cmd + 3), threshold.max/4);    // 设置高字节参数
}

/**
 * This function reads status register by ap3216c sensor measurement
 *
 * @param no
 *
 * @return status register value.
 */

uint8_t ap3216c_get_IntStatus(void)
{
    uint8_t IntStatus;
    
    /* 读中断状态寄存器 */
  
    read_regs(AP3216C_SYS_INT_STATUS_REG, 1, &IntStatus);
    // IntStatus 第 0 位表示 ALS 中断，第 1 位表示 PS 中断。
    
    return IntStatus;    // 返回状态
}

static void ap3216c_int_init(void)
{ 
    ap3216c_threshold_t threshold;    // 设置报警值阈值结构体
  
    threshold.min = 10;      // 光照下限产生报警
    threshold.max = 1000;    // 光照上限产生报警
    threshold.noises_time = 1;    // ALS 达到阈值后持续 threshold.noises_time * 4 个周期后开始报警， threshold.noises_time 最大为； 15
    set_als_threshold(AP3216C_ALS_LOW_THRESHOLD_L, threshold);    // 设置 ALS 的报警值
  
  
    threshold.min = 200;    // PS 低于 200 产生报警
    threshold.max = 500;    // PS 高于 500 产生报警
    set_ps_threshold(AP3216C_PS_LOW_THRESHOLD_L, threshold);     // 设置 PS 的报警值
}

/**
  * @brief  配置 中断输入引脚
  * @param  无
  * @retval 无
  */
void AP_INT_Config(void)
{
		INT_GPIO_Config();
}

/**
 * This function initializes ap3216c registered device driver
 *
 * @param no
 *
 * @return the ap3216c device.
 */
void ap3216c_init(void)
{
  I2C_Init();

  /* 关闭所有功能 */
  write_reg(AP3216C_SYS_CONFIGURATION_REG, 0x00);
  /* reset ap3216c */
  reset_sensor();
  HAL_Delay(10);
  ap3216c_set_param(AP3216C_SYSTEM_MODE, AP3216C_MODE_ALS_AND_PS);
//  HAL_Delay(100); // delay at least  100ms

  /* 配置中断脚 和 中断数据	*/
  ap3216c_int_init();
  AP_INT_Config();
}

/**
 * This function reads light by ap3216c sensor measurement
 *
 * @param no
 *
 * @return the ambient light converted to float data.
 */
float ap3216c_read_ambient_light(void)
{
    float brightness = 0.0; // default error data
    uint32_t read_data;
    uint8_t temp;

    read_data = read_low_and_high(AP3216C_ALS_DATA_L_REG, 1);

    ap3216c_get_param(AP3216C_ALS_RANGE, &temp);
  
    if (temp == AP3216C_ALS_RANGE_20661)
    {
        brightness = 0.35 * read_data; //sensor ambient light converse to reality
    }
    else if (temp == AP3216C_ALS_RANGE_5162)
    {
        brightness = 0.0788 * read_data; //sensor ambient light converse to reality
    }
    else if (temp == AP3216C_ALS_RANGE_1291)
    {
        brightness = 0.0197 * read_data; //sensor ambient light converse to reality
    }
    else if (temp == AP3216C_ALS_RANGE_323)
    {
        brightness = 0.0049 * read_data; //sensor ambient light converse to reality
    }

    return brightness;
}

/**
 * This function reads proximity by ap3216c sensor measurement
 *
 * @param no
 *
 * @return the proximity data.
 */
uint16_t ap3216c_read_ps_data(void)
{
    uint16_t proximity = 0;

    uint32_t read_data;
    read_data = read_low_and_high(AP3216C_PS_DATA_L_REG, 1); //read two data

    if (1 == ((read_data >> 6) & 0x01 || (read_data >> 14) & 0x01))
    {
       return proximity = 55555;    // 红外过高（IR），PS无效 返回一个 55555 的无效数据
    }

    proximity = (read_data & 0x000f) + (((read_data >> 8) & 0x3f) << 4); //sensor proximity converse to reality

    proximity |= read_data & 0x8000;    // 取最高位，0 表示物体远离，1 表示物体靠近
    
    return proximity;    // proximity 后十位是数据位，最高位为状态位
}

/**
 * This function reads ir by ap3216c sensor measurement
 *
 * @param no
 *
 * @return the ir data.
 */
uint16_t ap3216c_read_ir_data(void)
{
    uint16_t proximity = 0;


    uint32_t read_data;
    read_data = read_low_and_high(AP3216C_IR_DATA_L_REG, 1); //read two data

    proximity = (read_data & 0x0003) + ((read_data >> 8) & 0xFF); //sensor proximity converse to reality

    return proximity;
}

/**
 * This function sets parameter of ap3216c sensor
 *
 * @param cmd the parameter cmd of device
 * @param value for setting value in cmd register
 *
 * @return the setting parameter status,RT_EOK reprensents setting successfully.
 */
void ap3216c_set_param(ap3216c_cmd_t cmd, uint8_t value)
{
    switch (cmd)
    {
      case AP3216C_SYSTEM_MODE:
      {
          /* default 000,power down */
          write_reg(AP3216C_SYS_CONFIGURATION_REG, value);

          break;
      }
      case AP3216C_INT_PARAM:
      {
          write_reg(AP3216C_SYS_INT_CLEAR_MANNER_REG, value);

          break;
      }

      case AP3216C_ALS_RANGE:
      {
          uint8_t args;
        
          read_regs( AP3216C_ALS_CONFIGURATION_REG, 1, &args);
          args &= 0xcf;
          args |= value << 4;
          write_reg( AP3216C_ALS_CONFIGURATION_REG, args);

          break;
      }
      case AP3216C_ALS_PERSIST:
      {
          uint8_t args = 0;

          read_regs(AP3216C_ALS_CONFIGURATION_REG, 1, &args);
          args &= 0xf0;
          args |= value;
          write_reg(AP3216C_ALS_CONFIGURATION_REG, args);

          break;
      }
      case AP3216C_ALS_LOW_THRESHOLD_L:
      {
          write_reg(AP3216C_ALS_THRESHOLD_LOW_L_REG, value);

          break;
      }
      case AP3216C_ALS_LOW_THRESHOLD_H:
      {
          write_reg(AP3216C_ALS_THRESHOLD_LOW_H_REG, value);

          break;
      }
      case AP3216C_ALS_HIGH_THRESHOLD_L:
      {
          write_reg(AP3216C_ALS_THRESHOLD_HIGH_L_REG, value);

          break;
      }
      case AP3216C_ALS_HIGH_THRESHOLD_H:
      {
          write_reg(AP3216C_ALS_THRESHOLD_HIGH_H_REG, value);

          break;
      }
      case AP3216C_PS_GAIN:
      {
          uint8_t args = 0;

          read_regs(AP3216C_PS_CONFIGURATION_REG, 1, &args);
          args &= 0xf3;
          args |= value;
          write_reg(AP3216C_PS_CONFIGURATION_REG, args);

          break;
      }
      case AP3216C_PS_PERSIST:
      {
          uint8_t args = 0;

          read_regs(AP3216C_PS_CONFIGURATION_REG, 1, &args);
          args &= 0xfc;
          args |= value;
          write_reg(AP3216C_PS_CONFIGURATION_REG, args);

          break;
      }
      case AP3216C_PS_LOW_THRESHOLD_L:
      {
          
          write_reg( AP3216C_PS_THRESHOLD_LOW_L_REG, value);

          break;
      }
      case AP3216C_PS_LOW_THRESHOLD_H:
      {
          write_reg( AP3216C_PS_THRESHOLD_LOW_H_REG, value);

          break;
      }
      case AP3216C_PS_HIGH_THRESHOLD_L:
      {
          write_reg( AP3216C_PS_THRESHOLD_HIGH_L_REG, value);

          break;
      }
      case AP3216C_PS_HIGH_THRESHOLD_H:
      {
          write_reg( AP3216C_PS_THRESHOLD_HIGH_H_REG, value);

          break;
      }

    default:
    {
         
    }
    }
}

/**
 * This function gets parameter of ap3216c sensor
 *
 * @param cmd the parameter cmd of device
 * @param value to get value in cmd register
 *
 * @return the getting parameter status,RT_EOK reprensents getting successfully.
 */
void ap3216c_get_param(ap3216c_cmd_t cmd, uint8_t *value)
{
    switch (cmd)
    {
    case AP3216C_SYSTEM_MODE:
    {
        read_regs( AP3216C_SYS_CONFIGURATION_REG, 1, value);

        break;
    }
    case AP3216C_INT_PARAM:
    {
        read_regs( AP3216C_SYS_INT_CLEAR_MANNER_REG, 1, value);

        break;
    }
    case AP3216C_ALS_RANGE:
    {
        uint8_t temp;

        read_regs( AP3216C_ALS_CONFIGURATION_REG, 1, value);
        temp = (*value & 0xff) >> 4;

        *value = temp;

        break;
    }
    case AP3216C_ALS_PERSIST:
    {
        uint8_t temp;

        read_regs( AP3216C_ALS_CONFIGURATION_REG, 1, value);
        temp = *value & 0x0f;

        *value = temp;

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_L:
    {
        read_regs( AP3216C_ALS_THRESHOLD_LOW_L_REG, 1, value);

        break;
    }
    case AP3216C_ALS_LOW_THRESHOLD_H:
    {
        read_regs( AP3216C_ALS_THRESHOLD_LOW_H_REG, 1, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_L:
    {
        read_regs( AP3216C_ALS_THRESHOLD_HIGH_L_REG, 1, value);

        break;
    }
    case AP3216C_ALS_HIGH_THRESHOLD_H:
    {
        read_regs( AP3216C_ALS_THRESHOLD_HIGH_H_REG, 1, value);

        break;
    }
    case AP3216C_PS_GAIN:
    {
        uint8_t temp;

        read_regs( AP3216C_PS_CONFIGURATION_REG, 1, &temp);

        *value = (temp & 0xc) >> 2;

        break;
    }
    case AP3216C_PS_PERSIST:
    {
        uint8_t temp;

        read_regs( AP3216C_PS_CONFIGURATION_REG, 1, &temp);

        *value = temp & 0x3;

        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_L:
    {
        read_regs( AP3216C_PS_THRESHOLD_LOW_L_REG, 1, value);

        break;
    }
    case AP3216C_PS_LOW_THRESHOLD_H:
    {
        read_regs( AP3216C_PS_THRESHOLD_LOW_H_REG, 1, value);
        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_L:
    {
        read_regs( AP3216C_PS_THRESHOLD_HIGH_L_REG, 1, value);

        break;
    }
    case AP3216C_PS_HIGH_THRESHOLD_H:
    {
        read_regs( AP3216C_PS_THRESHOLD_HIGH_H_REG, 1, value);

        break;
    }

    default:
    {
         
    }
    }
}

