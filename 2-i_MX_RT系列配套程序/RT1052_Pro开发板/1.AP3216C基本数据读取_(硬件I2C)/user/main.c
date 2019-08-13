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
#include "./lpi2c/bsp_lpi2c.h"
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
		static uint16_t ALS_RAW;
		static uint16_t PS_RAW;
		static uint16_t IR_RAW;
		
		float ALSValue;
	
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
		AP3216C_Init();
		while(1)
		{
				AP3216CReadALS(&ALS_RAW);
				AP3216CReadPS(&PS_RAW);
				AP3216CReadIR(&IR_RAW);
				ALSValue = ALS_RAW * 0.36;// Lux = 16 bit ALS data * Resolution
				PRINTF("环境光：%.2flux ",ALSValue);
				PRINTF("接近值：%d ",PS_RAW);
				PRINTF("红外光：%d\r\n",IR_RAW);
				RGB_RED_LED_TOGGLE
				CPU_TS_Tmr_Delay_MS(225);// 最小采样间隔225ms
		}

}


/****************************END OF FILE**********************/
