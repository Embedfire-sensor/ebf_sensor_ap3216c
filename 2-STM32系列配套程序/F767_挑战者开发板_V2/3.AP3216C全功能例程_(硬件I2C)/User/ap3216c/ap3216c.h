#ifndef __AP3216C_H
#define __AP3216C_H
#include "stm32f7xx.h"

//中断引脚定义
/*******************************************************/

#define AP_INT_GPIO_PORT                GPIOE
#define AP_INT_GPIO_CLK_ENABLE()        __GPIOE_CLK_ENABLE();
#define AP_INT_GPIO_PIN                 GPIO_PIN_3

/* 读中断引脚状态 */
#define AP_INT_Read()    HAL_GPIO_ReadPin(AP_INT_GPIO_PORT, AP_INT_GPIO_PIN)
/*******************************************************/

// AP3216C, Standard address 0x1E
#define AP3216C_ADDRESS                 0x3C
#define AP3216C_WHO_AM_I                0x75

#define AP3216C_SYS_CONFIG              0x00
#define AP3216C_INT_STATUS              0x01
#define AP3216C_INT_CLEAR_MANNER        0x02
#define AP3216C_IR_DATA_LOW             0x0A
#define AP3216C_IR_DATA_HIGH            0x0B
#define AP3216C_ALS_DATA_LOW            0x0C
#define AP3216C_ALS_DATA_HIGH           0x0D
#define AP3216C_PS_DATA_LOW             0x0E
#define AP3216C_PS_DATA_HIGH            0x0F

#define AP3216C_ALS_CONFIG              0x10
#define AP3216C_ALS_CALIBRAE            0x19
#define AP3216C_ALS_LOW_THRESHOLD7_0    0x1A
#define AP3216C_ALS_LOW_THRESHOLD15_8   0x1B
#define AP3216C_ALS_HIGH_THRESHOLD7_0   0x1C
#define AP3216C_ALS_HIGH_THRESHOLD15_8  0x1D

#define AP3216C_PS_CONFIG               0x20
#define AP3216C_PS_LED_CTRL             0x21
#define AP3216C_PS_INT_FORM             0x22
#define AP3216C_PS_MEAN_TIME            0x23 
#define AP3216C_PS_LED_WAITING_TIME     0x24
#define AP3216C_PS_CALIBRATION_L        0x28
#define AP3216C_PS_CALIBRATION_H        0x29
#define AP3216C_PS_LOW_THRESHOLD2_0     0x2A
#define AP3216C_PS_LOW_THRESHOLD10_3    0x2B
#define AP3216C_PS_HIGH_THRESHOLD2_0    0x2C
#define AP3216C_PS_HIGH_THRESHOLD10_3   0x2D

enum ap3216c_mode_value
{
    AP3216C_MODE_POWER_DOWN,      //Power down (Default)
    AP3216C_MODE_ALS,             //ALS function active
    AP3216C_MODE_PS,              //PS+IR function active
    AP3216C_MODE_ALS_AND_PS,      //ALS and PS+IR functions active
    AP3216C_MODE_SW_RESET,        //SW reset
    AP3216C_MODE_ALS_ONCE,        //ALS function once
    AP3216C_MODE_PS_ONCE,         //PS+IR function once
    AP3216C_MODE_ALS_AND_PS_ONCE, //ALS and PS+IR functions once
};

enum als_range
{
    AP3216C_ALS_RANGE_20661, //Resolution = 0.35 lux/count(default).
    AP3216C_ALS_RANGE_5162,  //Resolution = 0.0788 lux/count.
    AP3216C_ALS_RANGE_1291,  //Resolution = 0.0197 lux/count.
    AP3216C_ALS_RANGE_323,   //Resolution = 0.0049 lux/count
};
typedef enum als_range als_range_t;

static void AP3216C_WriteReg(uint8_t reg_add,uint8_t reg_dat);
static void AP3216C_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num);

void AP3216C_Set_ALS_Threshold(uint16_t low_threshold, uint16_t high_threshold);
void AP3216C_Set_PS_Threshold(uint16_t low_threshold, uint16_t high_threshold);
uint8_t AP3216C_Get_INTStatus(void);
static void AP3216C_INT_Config(void);

float AP3216C_ReadALS(void);
uint16_t AP3216C_ReadPS(void);
uint16_t AP3216C_ReadIR(void);
void AP3216C_Init(void);

#endif  /*__AP3216C*/
