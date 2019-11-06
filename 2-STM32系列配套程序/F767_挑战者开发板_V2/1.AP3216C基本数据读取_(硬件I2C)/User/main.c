/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   AP3216C测试程序
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32F767 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f7xx.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_debug_usart.h"
#include <stdlib.h>
#include "main.h"
#include "./i2c/i2c.h"
#include "./ap3216c/ap3216c.h"
//设置是否使用LCD进行显示，不需要的话把这个宏注释掉即可
//#define USE_LCD_DISPLAY

#ifdef USE_LCD_DISPLAY
 #include "./lcd/bsp_lcd.h"
#endif

/*简单任务管理*/
uint32_t Task_Delay[NumOfTask]={0};
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
//  float PSValue;
//  float IRValue;
	
    /* 系统时钟初始化成216 MHz */
    SystemClock_Config();
    /* LED 端口初始化 */
    LED_GPIO_Config();
	
#ifdef USE_LCD_DISPLAY		
    /* LCD 端口初始化 */ 
    LCD_Init();
    /* LCD 第一层初始化 */ 
    LCD_LayerInit(0, LCD_FB_START_ADDRESS,ARGB8888);
	/* LCD 第二层初始化 */ 
    LCD_LayerInit(1, LCD_FB_START_ADDRESS+(LCD_GetXSize()*LCD_GetYSize()*4),ARGB8888);
    /* 使能LCD，包括开背光 */ 
    LCD_DisplayOn(); 

    /* 选择LCD第一层 */
    LCD_SelectLayer(0);

    /* 第一层清屏，显示全黑 */ 
    LCD_Clear(LCD_COLOR_BLACK);  

    /* 选择LCD第二层 */
    LCD_SelectLayer(1);

    /* 第二层清屏，显示透明 */ 
    LCD_Clear(LCD_COLOR_TRANSPARENT);

    /* 配置第一和第二层的透明度,最小值为0，最大值为255*/
    LCD_SetTransparency(0, 255);
    LCD_SetTransparency(1, 0);
	
	/* 选择LCD第一层 */
    LCD_SelectLayer(0);
	/*设置字体颜色及字体的背景颜色*/
	LCD_SetColors(LCD_COLOR_RED,LCD_COLOR_BLACK);	
#endif
  /*初始化USART1*/
	DEBUG_USART_Config(); 
		
	//初始化 I2C
	I2cMaster_Init(); 

	printf("\r\n 欢迎使用野火 STM32F767 开发板。\r\n");		 

	printf("\r\n 这是一个硬件I2C外设(AP3216C)读写测试例程 \r\n");

 	//AP3216C初始化
	AP3216C_Init();
  
  Delay(250);
	
  while(1)
  {
    if(Task_Delay[0]==TASK_ENABLE)
    {
      LED2_TOGGLE;
      Task_Delay[0]=1000;
    }
    
    if(Task_Delay[1]==0)
    {
      AP3216CReadALS(&ALS_RAW);
      AP3216CReadPS(&PS_RAW);
      AP3216CReadIR(&IR_RAW);
      ALSValue = ALS_RAW * 0.36;// Lux = 16 bit ALS data * Resolution
      printf("环境光：%.2flux ",ALSValue);
      printf("接近值：%d ",PS_RAW);
      printf("红外光：%d\r\n",IR_RAW);			
      
      
      #ifdef USE_LCD_DISPLAY	
        {
          char cStr [ 70 ];
          sprintf ( cStr, "ALS：%8.2flux",ALSValue);	//环境光数据

          LCD_DisplayStringLine(7,(uint8_t* )cStr);			

          sprintf ( cStr, "PS ：%8d ",PS_RAW);	//接近值数据

          LCD_DisplayStringLine(8,(uint8_t* )cStr);			

          sprintf ( cStr, "IR ：%8d",IR_RAW);	//红外光
          LCD_DisplayStringLine(9,(uint8_t* )cStr);			

        }
      #endif
      
      Task_Delay[1]=500; //更新一次数据，可根据自己的需求，提高采样频率，如100ms采样一次
      
    }

    //*************************************	下面是增加任务的格式************************************//
//		if(Task_Delay[i]==0)
//		{
//			Task(i);
//			Task_Delay[i]=;
//		}

	}
}

/**
  * @brief  System Clock 配置
  *         system Clock 配置如下 : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  无
  * @retval 无
  */
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* 使能HSE，配置HSE为PLL的时钟源，配置PLL的各种分频因子M N P Q 
	 * PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;

	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}

	/* 激活 OverDrive 模式以达到216M频率  */  
	ret = HAL_PWREx_EnableOverDrive();
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}

	/* 选择PLLCLK作为SYSCLK，并配置 HCLK, PCLK1 and PCLK2 的时钟分频因子 
	 * SYSCLK = PLLCLK     = 216M
	 * HCLK   = SYSCLK / 1 = 216M
	 * PCLK2  = SYSCLK / 2 = 108M
	 * PCLK1  = SYSCLK / 4 = 54M
	 */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	if(ret != HAL_OK)
	{
		while(1) { ; }
	}  
}

/*********************************************END OF FILE**********************/

