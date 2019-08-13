#ifndef __BSP_AP3216C_H__
#define __BSP_AP3216C_H__

#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#define INT_GPIO             	GPIO1
#define INT_GPIO_PIN         	(18U)
#define INT_IOMUXC           	IOMUXC_GPIO_AD_B1_02_GPIO1_IO18

//中断相关，IRQ中断号及IRQHandler中断服务函数
#define INT_IRQ           GPIO1_Combined_16_31_IRQn
#define INT_IRQHandler    GPIO1_Combined_16_31_IRQHandler



/*设定的AP3216C IIC设备地址*/
#define AP3216C_ADDR              0x3C
//System Register
#define AP3216C_SYS_CONFIGURATION_REG 0x00 //default
#define AP3216C_SYS_INT_STATUS_REG 0x01
#define AP3216C_SYS_INT_CLEAR_MANNER_REG 0x02
#define AP3216C_IR_DATA_L_REG 0x0A
#define AP3216C_IR_DATA_H_REG 0x0B
#define AP3216C_ALS_DATA_L_REG 0x0C
#define AP3216C_ALS_DATA_H_REG 0x0D
#define AP3216C_PS_DATA_L_REG 0x0E
#define AP3216C_PS_DATA_H_REG 0x0F

//ALS Register
#define AP3216C_ALS_CONFIGURATION_REG 0x10 //range 5:4,persist 3:0
#define AP3216C_ALS_CALIBRATION_REG 0x19
#define AP3216C_ALS_THRESHOLD_LOW_L_REG 0x1A  //bit 7:0
#define AP3216C_ALS_THRESHOLD_LOW_H_REG 0x1B  //bit 15:8
#define AP3216C_ALS_THRESHOLD_HIGH_L_REG 0x1C //bit 7:0
#define AP3216C_ALS_THRESHOLD_HIGH_H_REG 0x1D //bit 15:8

//PS Register
#define AP3216C_PS_CONFIGURATION_REG 0x20
#define AP3216C_PS_LED_DRIVER_REG 0x21
#define AP3216C_PS_INT_FORM_REG 0x22
#define AP3216C_PS_MEAN_TIME_REG 0x23
#define AP3216C_PS_LED_WAITING_TIME_REG 0x24
#define AP3216C_PS_CALIBRATION_L_REG 0x28
#define AP3216C_PS_CALIBRATION_H_REG 0x29
#define AP3216C_PS_THRESHOLD_LOW_L_REG 0x2A  //bit 1:0
#define AP3216C_PS_THRESHOLD_LOW_H_REG 0x2B  //bit 9:2
#define AP3216C_PS_THRESHOLD_HIGH_L_REG 0x2C //bit 1:0
#define AP3216C_PS_THRESHOLD_HIGH_H_REG 0x2D //bit 9:2

//#define AP3216C_ADDR 0x1e /*0x3c=0x1e<<1*/ // AP3216C 的 7 位地址

void AP_INT_Config(void);

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

enum ap3216c_int_clear_manner
{
    AP3216C_INT_CLEAR_MANNER_BY_READING,  //INT is automatically cleared by reading data registers(Default)
    AP3216C_ALS_CLEAR_MANNER_BY_SOFTWARE, //Software clear after writing 1 into address 0x01 each bit
};

enum als_range
{
    AP3216C_ALS_RANGE_20661, //Resolution = 0.35 lux/count(default).
    AP3216C_ALS_RANGE_5162,  //Resolution = 0.0788 lux/count.
    AP3216C_ALS_RANGE_1291,  //Resolution = 0.0197 lux/count.
    AP3216C_ALS_RANGE_323,   //Resolution = 0.0049 lux/count
};
typedef enum als_range als_range_t;

enum ps_gain
{
    AP3216C_PS_GAIN1, //detection distance *1.
    AP3216C_PS_GAIN2, //detection distance *2 (default).
    AP3216C_PS_GAIN4, //detection distance *4.
    AP3216C_PS_GAIN8, //detection distance *8.
};
typedef enum als_gain ps_gain_t;

enum ap3216c_cmd
{
    AP3216C_SYSTEM_MODE,          //system  Configuration(Default : 000)
    AP3216C_INT_PARAM,            //INT Clear Manner(Default : 0)
    AP3216C_ALS_RANGE,            //ALS dynamic range(Default : 00)
    AP3216C_ALS_PERSIST,          //ALS persist(Default : 0000)
    AP3216C_ALS_CALIBRATION,      //ALS window loss calibration(Default : 0x40)
    AP3216C_ALS_LOW_THRESHOLD_L,  //Lower byte of low interrupt threshold for ALS(Default : 0x00)
    AP3216C_ALS_LOW_THRESHOLD_H,  //Higher byte of low interrupt threshold for ALS(Default : 0x00)
    AP3216C_ALS_HIGH_THRESHOLD_L, //Lower byte of high interrupt threshold for ALS (Default : 0xFF)
    AP3216C_ALS_HIGH_THRESHOLD_H, //Higher byte of high interrupt threshold for ALS(Default : 0xFF)
    AP3216C_PS_INTEGRATED_TIME,   //PS or IR Integrated time select(Default : 0000)
    AP3216C_PS_GAIN,              //PS gain (Default : 01)
    AP3216C_PS_PERSIST,           //Interrupt filter(Default : 01)
    AP3216C_PS_LED_CONTROL,       //LED pulse(Default : 01)
    AP3216C_PS_LED_DRIVER_RATIO,  //LED driver ratio(Default : 11)
    AP3216C_PS_INT_MODE,          //PS INT Mode(Default : 0x01)
    AP3216C_PS_MEAN_TIME,         //PS mean time(Default : 0x00)
    AP3216C_PS_WAITING_TIME,      //PS LED Waiting(Default : 0x00)
    AP3216C_PS_CALIBRATION_L,     //PS Calibration L(Default : 0x00)
    AP3216C_PS_CALIBRATION_H,     //PS Calibration H(Default : 0x00)
    AP3216C_PS_LOW_THRESHOLD_L,   //PS Low Threshold L(Default :0x00)
    AP3216C_PS_LOW_THRESHOLD_H,   //PS Low Threshold H(Default :0x00)
    AP3216C_PS_HIGH_THRESHOLD_L,  //PS high Threshold L(Default :0xff)
    AP3216C_PS_HIGH_THRESHOLD_H,  //PS high Threshold H(Default :0xff)
};
typedef enum ap3216c_cmd ap3216c_cmd_t;

/* intrrupt parameters of ap3216c on ps or als */
struct ap3216c_threshold
{
    uint16_t min;        /* als 16 bits, ps 10 bits available(low byte ：0-1 bit and High Byte ：8-15 bit ) */
    uint16_t max;        /* als 16 bits, ps 10 bits available(low byte ：0-1 bit and High Byte ：8-15 bit ) */
    uint8_t noises_time; /* filter special noises trigger interrupt */
};
typedef struct ap3216c_threshold ap3216c_threshold_t;


uint8_t ap3216c_get_IntStatus(void);


/**
 * This function initializes ap3216c registered device driver
 *
 * @param no
 *
 * @return no
 */
void ap3216c_init(void);

/**
 * This function reads temperature by ap3216c sensor measurement
 *
 * @param no
 *
 * @return the ambient light converted to float data.
 */
float ap3216c_read_ambient_light(void);

/**
 * This function reads temperature by ap3216c sensor measurement
 *
 * @param no
 *
 * @return the proximity data.
 */
uint16_t ap3216c_read_ps_data(void);

uint16_t ap3216c_read_ir_data(void);

/**
 * This function sets parameter of ap3216c sensor
 *
 * @param cmd the parameter cmd of device
 * @param value for setting value in cmd register
 *
 * @return the setting parameter status,RT_EOK reprensents setting successfully.
 */
void ap3216c_set_param(ap3216c_cmd_t cmd, uint8_t value);

/**
 * This function gets parameter of ap3216c sensor
 *
 * @param cmd the parameter cmd of device
 * @param value to get value in cmd register
 *
 * @return the getting parameter status,RT_EOK reprensents getting successfully.
 */
void ap3216c_get_param(ap3216c_cmd_t cmd, uint8_t *value);

#endif /*__DRV_AP3216C_H__ */
