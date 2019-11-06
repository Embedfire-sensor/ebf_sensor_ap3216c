#include "./AP3216C/AP3216C.h"
#include "./usart/bsp_debug_usart.h"
#include "./i2c/bsp_i2c.h"
#include "stm32f7xx_hal.h"


#define AP3216C_ERROR 		I2C_ERROR
#define AP3216C_INFO 		I2C_INFO
/**
  * @brief   写数据到AP3216C寄存器
  * @param   reg_add:寄存器地址
	* @param	 reg_data:要写入的数据
  * @retval  
  */
void AP3216C_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	Sensor_write(reg_add, reg_dat);
}

/**
  * @brief   从AP3216C寄存器读取数据
  * @param   reg_add:寄存器地址
	* @param	 Read：存储数据的缓冲区
	* @param	 num：要读取的数据量
  * @retval  
  */
void AP3216C_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num)
{
	Sensor_Read(reg_add, Read, num);
}

/**
  * @brief   复位AP3216C传感器
  * @param   
  * @retval  
  */
void AP3216C_Reset(void)
{
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, AP3216C_MODE_SW_RESET);
}

/**
  * @brief   设置AP3216C环境光传感器的上下限阈值
  * @param   low_threshold：下限阈值
  * @param   high_threshold：上限阈值
  * @retval  
  */
void AP3216C_Set_ALS_Threshold(uint16_t low_threshold, uint16_t high_threshold)
{
  uint8_t resolution;
  double DR;
  
  /* 读光照强度的范围 */
  AP3216C_ReadData(AP3216C_ALS_CONFIG, &resolution, 1);
  if((resolution >> 4) == AP3216C_ALS_RANGE_20661)
  {
    DR = 0.36;
  }
  else if((resolution >> 4) == AP3216C_ALS_RANGE_5162)
  {
    DR = 0.089;
  }
  else if((resolution >> 4) == AP3216C_ALS_RANGE_1291)
  {
    DR = 0.022;
  }
  else if((resolution >> 4) == AP3216C_ALS_RANGE_323)
  {
    DR = 0.0056;
  }
  
  low_threshold = (uint16_t)low_threshold / DR;
  high_threshold = (uint16_t)high_threshold / DR;
  
  AP3216C_WriteReg(AP3216C_ALS_LOW_THRESHOLD7_0, (low_threshold & 0xff));
  AP3216C_WriteReg(AP3216C_ALS_LOW_THRESHOLD15_8, low_threshold >> 8);
  AP3216C_WriteReg(AP3216C_ALS_HIGH_THRESHOLD7_0, (high_threshold & 0xff));
  AP3216C_WriteReg(AP3216C_ALS_HIGH_THRESHOLD15_8, high_threshold >> 8);
}

/**
  * @brief   设置AP3216C接近传感器的上下限阈值
  * @param   low_threshold：下限阈值
  * @param   high_threshold：上限阈值
  * @retval  
  */
void AP3216C_Set_PS_Threshold(uint16_t low_threshold, uint16_t high_threshold)
{
  if(low_threshold > 1020)
  {
    AP3216C_WriteReg(AP3216C_PS_LOW_THRESHOLD2_0, (low_threshold - 1020 & 0x03));
  }
  AP3216C_WriteReg(AP3216C_PS_LOW_THRESHOLD10_3, (low_threshold / 4));
  
  if(high_threshold > 1020)
  {
    AP3216C_WriteReg(AP3216C_PS_HIGH_THRESHOLD2_0, (high_threshold - 1020 & 0x03));
  }
  AP3216C_WriteReg(AP3216C_PS_HIGH_THRESHOLD10_3, (high_threshold / 4));
}

/**
  * @brief   配置AP3216C中断引脚
  * @param   
  * @param   
  * @retval  
  */
static void AP3216C_INT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 

  /*开启按键GPIO口的时钟*/
  AP_INT_GPIO_CLK_ENABLE();

  /* 选择按键1的引脚 */ 
  GPIO_InitStructure.Pin = AP_INT_GPIO_PIN;
  /* 设置引脚为输入模式 */ 
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;	    		
  /* 设置引脚不上拉也不下拉 */
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  /* 使用上面的结构体初始化按键 */
  HAL_GPIO_Init(AP_INT_GPIO_PORT, &GPIO_InitStructure); 
}

/**
  * @brief   读取AP3216C传感器的中断状态
  * @param   
  * @retval  INTstatus：第0位表示ALS中断，第1位表示PS中断
  */
uint8_t AP3216C_Get_INTStatus(void)
{
  uint8_t INTstatus;
  
  AP3216C_ReadData(AP3216C_INT_STATUS, &INTstatus, 1);
  
  return INTstatus;
}

/**
  * @brief   初始化AP3216C传感器
  * @param   
  * @retval  
  */
void AP3216C_Init(void)
{
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, 0x00);//关闭所有功能
  AP3216C_Reset();//复位
  HAL_Delay(10);//复位后一定要延时10ms及以上，否则会出错
  AP3216C_WriteReg(AP3216C_SYS_CONFIG, AP3216C_MODE_ALS_AND_PS);//开启所有功能
//  Delay(100);
  AP3216C_Set_ALS_Threshold(10, 1000);//环境光下限10，上限1000，超限触发中断
  AP3216C_Set_PS_Threshold(200, 400);//接近值下限200，上限400，超限触发中断
  AP3216C_INT_Config();
}

/**
  * @brief   读取AP3216C的环境光传感器数据
  * @param   
  * @retval  
  */
float AP3216C_ReadALS(void)
{
  uint8_t temp, buf[2];
  uint16_t ALS_RAW;
  float ALS_Data = 0.0;
  
  AP3216C_ReadData(AP3216C_ALS_DATA_LOW, buf, 2);
  ALS_RAW = (buf[1] << 8) | buf[0];
    
  AP3216C_ReadData(AP3216C_ALS_CONFIG, &temp, 1);
  if((temp >> 4) == AP3216C_ALS_RANGE_20661)
  {
    ALS_Data = ALS_RAW * 0.36;
  }
  else if((temp >> 4) == AP3216C_ALS_RANGE_5162)
  {
    ALS_Data = ALS_RAW * 0.089;
  }
  else if((temp >> 4) == AP3216C_ALS_RANGE_1291)
  {
    ALS_Data = ALS_RAW * 0.022;
  }
  else if((temp >> 4) == AP3216C_ALS_RANGE_323)
  {
    ALS_Data = ALS_RAW * 0.0056;
  }
  return ALS_Data;
}

/**
  * @brief   读取AP3216C的接近传感器数据
  * @param   
  * @retval  
  */
uint16_t AP3216C_ReadPS(void)
{
  uint8_t buf[2];
  uint16_t PS_Data;
  uint16_t proximity = 0;
  
  AP3216C_ReadData(AP3216C_PS_DATA_LOW, buf, 2);
  PS_Data = (buf[1] << 8) + buf[0];
  
  if(1 == ((PS_Data >> 6) & 0x01 || (PS_Data >> 14) & 0x01))
  {
    return PS_Data = 55555;//红外太强时接近传感器无效，返回55555
  }
  else
  {
    proximity = (PS_Data & 0x000f) + (((PS_Data >> 8) & 0x3f) << 4);
    proximity |= PS_Data & 0x8000;//最高位表示对象的位置，0表示远离，1表示接近
    
    return proximity;
  }
//  return PS_Data;
}

/**
  * @brief   读取AP3216C的红外光传感器数据
  * @param   
  * @retval  
  */
uint16_t AP3216C_ReadIR(void)
{
  uint8_t buf[2];
  uint16_t IR_Data;
  
  AP3216C_ReadData(AP3216C_IR_DATA_LOW, buf, 2);
  IR_Data = (buf[1] << 8) + buf[0];
  IR_Data = (IR_Data & 0x0003) + ((IR_Data >> 8) & 0xFF);
  
  return IR_Data;
}
