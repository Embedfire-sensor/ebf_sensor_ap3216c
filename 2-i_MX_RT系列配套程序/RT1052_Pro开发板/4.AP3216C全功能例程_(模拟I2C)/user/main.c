/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   AP3216C 读光照实验
  ******************************************************************
  * @attention
  *
  * 实验平台:野火  i.MXRT1052开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************
  */
#include <stdio.h>

#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "./i2c/bsp_i2c.h"
#include "./delay/core_delay.h"   
#include "./ap3216c/bsp_ap3216c.h"
#include "./led/bsp_led.h"   
/*******************************************************************
 * Prototypes
 *******************************************************************/

/*******************************************************************
 * Code
 *******************************************************************/

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
		/**/
		float ALS = 0.0;
		uint16_t PS = 0;
		uint16_t IR = 0;
		uint8_t IntStatus;
	
	/* 初始化内存保护单元 */
		BOARD_ConfigMPU();
		/* 初始化开发板引脚 */
		BOARD_InitPins();
		/* 初始化开发板时钟 */
		BOARD_BootClockRUN();
		/* 初始化调试串口 */
		BOARD_InitDebugConsole();
		/* 打印系统时钟 */
		PRINTF("\r\n");
		PRINTF("*****欢迎使用 野火i.MX RT1052 开发板*****\r\n");
		PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
		PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
		PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
		PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
		PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
		PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
		PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
		PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));	
    PRINTF("\r\n 这是一个三合一光照传感器测试例程 \r\n");
		PRINTF(" 芯片初始化中.....\n");
		LED_GPIO_Config();
		/* 精确延时 */
		CPU_TS_TmrInit();
		/* 初始化 光照传感器 */
		ap3216c_init();
		while(1)
		{
				IntStatus = ap3216c_get_IntStatus();    // 先读状态位，读ADC数据位会清除状态位（默认设置）
				ALS = ap3216c_read_ambient_light();
				PS = ap3216c_read_ps_data();
				IR = ap3216c_read_ir_data();

				PRINTF("\n光照强度是：%.2fLux\n红外强度是：%d\n", ALS, IR);

				if (PS == 55555)    // IR 太强 PS 数据无效
					PRINTF("IR 太强 PS 数据无效\n");
				else
				{
					PRINTF("接近距离是：%d\n", PS & 0x3FF);
				}
				
				if ((PS >> 15) & 1)
					PRINTF("物体接近\n");
				else
					PRINTF("物体远离\n");
				
				if (IntStatus & 0x1)
					PRINTF("ALS 产生中断\n");
				
				if (IntStatus >> 1 & 0x1)
					PRINTF("PS 产生中断\n");
				RGB_RED_LED_TOGGLE
				CPU_TS_Tmr_Delay_MS(225);// 最小采样间隔225ms
		}

}


/****************************END OF FILE**********************/
