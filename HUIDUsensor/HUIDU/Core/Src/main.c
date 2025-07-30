/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "stdio.h"
#include "string.h"

#include "tjc_usart_hmi.h"
#define FRAME_LENGTH 7
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*unsigned char Digtal;
unsigned char rx_buff[256]={0};
uint8_t gw_gray_serial_read(){
	uint8_t ret = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		// 输出时钟下降�? 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
		Delay_us(2);
		//避免GPIO翻转过快导致反应不及�?
		ret |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << i;

		//输出时钟上升�?,让传感器更新数据
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);
	
		//延迟�?要在5us左右 
		Delay_us(5);
	}
	
	return ret;
}*/



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  initRingBuffer();                   // 初始化环形缓冲区
  HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1); // 启动串口接收中断
  int a = 100;
  char str[100];
  uint32_t nowtime = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*Digtal=gw_gray_serial_read();
		sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
		uart1_send_string((char *)rx_buff);
		memset(rx_buff,0,256);
		Delay_ms(1);
  */
	  if (HAL_GetTick() - nowtime >= 1500)
		{
			nowtime = HAL_GetTick();

			sprintf(str, "n0.val=%d", a);
			tjc_send_string(str);
			sprintf(str, "t0.txt=\"%d\"\xff\xff\xff", a);
			tjc_send_string(str);
			sprintf(str, "click b0,1\xff\xff\xff");
			tjc_send_string(str);
			HAL_Delay(50);
			sprintf(str, "click b0,0\xff\xff\xff");
			tjc_send_string(str);

			a++;
		}
		// 接线说明�?
    // STM32F103 �? GND 连接到串口屏�? GND
    // STM32F103 �? TX2 (PA2) 连接到串口屏�? RX
    // STM32F103 �? RX2 (PA3) 连接到串口屏�? TX
    // STM32F103 �? 5V 连接到串口屏�? 5V，注意供电是否稳�?

    // 数据帧格式说明：
    // 数据帧长度：7 字节
    // 帧结构：
    // 起始字节 | 数据1 | 数据2 | 数据3 | 结束标志�?3字节�?0xFF, 0xFF, 0xFF�?
    // 0x55     | 1字节 | 1字节 | 1字节 | 0xFFFFFF

    // 当数�?1�? 0x01 时：
    // 帧结构：
    // 0x55 | 0x01 | led编号 | led状�?? | 0xFFFFFF
    // 示例1：printh 55 01 01 00 ff ff ff --> 控制 led 1 关闭
    // 示例2：printh 55 01 04 01 ff ff ff --> 控制 led 4 打开
    // 示例3：printh 55 01 00 01 ff ff ff --> 控制 led 0 打开
    // 示例4：printh 55 01 04 00 ff ff ff --> 控制 led 4 关闭

    // 当数�?1�? 0x02 �? 0x03 时：
    // 帧结构：
    // 0x55 | 0x02/0x03 | 数�?�高�? | 数�?�低�? | 0xFFFFFF
    // 示例1：printh 55 02 64 00 ff ff ff --> 设置 h0.val=100
    // 示例2：printh 55 02 00 00 ff ff ff --> 设置 h0.val=0
    // 示例3：printh 55 03 64 00 ff ff ff --> 设置 h1.val=100
    // 示例4：printh 55 03 00 00 ff ff ff --> 设置 h1.val=0

    // 当环形缓冲区中有足够数据时处�?
    while (getRingBufferLength() >= FRAME_LENGTH)
    {
      // �?查帧头帧尾是否匹�?
      if (getRingBufferLength() >= FRAME_LENGTH &&
          read1ByteFromRingBuffer(0) == 0x55 &&
          read1ByteFromRingBuffer(4) == 0xff &&
          read1ByteFromRingBuffer(5) == 0xff &&
          read1ByteFromRingBuffer(6) == 0xff)
      {
        // 匹配命令类型
        if (read1ByteFromRingBuffer(1) == 0x01)
        {
          // 控制LED状�??
          sprintf(str, "msg.txt=\"led %d is %s\"", read1ByteFromRingBuffer(2),
                  read1ByteFromRingBuffer(3) ? "on" : "off");
          tjc_send_string(str);
        }
        else if (read1ByteFromRingBuffer(1) == 0x02)
        {
          // 显示 h0.val 的�??
          sprintf(str, "msg.txt=\"h0.val is %d\"", read1ByteFromRingBuffer(2));
          tjc_send_string(str);
        }
        else if (read1ByteFromRingBuffer(1) == 0x03)
        {
          // 显示 h1.val 的�??
          sprintf(str, "msg.txt=\"h1.val is %d\"", read1ByteFromRingBuffer(2));
          tjc_send_string(str);
        }

        deleteRingBuffer(7); // 删除已处理的7字节数据
      }
      else
      {
        // 如果未匹配，则删�?1字节尝试重新同步
        deleteRingBuffer(1);
        break;
      }
    }
  }	  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
