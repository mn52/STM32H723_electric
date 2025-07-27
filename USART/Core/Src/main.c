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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define PACKET_SIZE 7
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
volatile bool packetReady = 0;
uint8_t targetX_cm = 0;
uint8_t targetY_cm = 0;
float targetX_m = 0.00;
float targetY_m = 0.00;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//设置状�??

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define _2PI_ 6.2831853070f
#define MAX_X 0.33f   // X轴最大移动距�?33cm
#define MAX_Y 0.33f   // Y轴最大移动距�?33cm
#define MAX_Z 0.15f   // Z轴最大移动距�?15cm

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float currentPosition[3] = {0.0f, 0.0f, 0.0f}; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Stop_Motor(uint8_t addr);
void motors12_forward(uint16_t vel, float distance);
void motors12_backward(uint16_t vel, float distance);

void motor3_forward(uint16_t vel, float distance);
void motor3_backward(uint16_t vel, float distance);
void motor4_forward(uint16_t vel, float distance);
uint16_t x_handle(float x);
void Estimate_motors1234();
void UpdatePosition(float x, float y, float z);
void HomePosition(uint16_t vel_x,uint16_t vel_y,uint16_t vel_z);
void MoveToPoint(float x_m, float y_m, float z_m, uint16_t vel_x, uint16_t vel_y,uint16_t vel_z);
void CheckLimitSwitches(void);
void MoveTo(float x_m, float y_m,float z_m, uint16_t vel_x, uint16_t vel_y, uint16_t vel_z);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
	return ch;
}

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
  MX_UART9_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
 
  
  HAL_UART_Receive_IT(&huart2, &rxBuffer[0], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  //电机1的方�?1backward
	 //电机2的方�?0backward
	//Emm_V5_Pos_Control(1, 1, 50, 10, 32000, 0, 1);
	//HAL_Delay(500);
	//Emm_V5_Pos_Control(2, 0, 50, 10, 32000, 0, 1);
	//HAL_Delay(500);
	  //Emm_V5_Synchronous_motion(0);
	 //Emm_V5_Pos_Control(3, 0, 3, 1, 32000, 0, 0);
	  //HAL_Delay(500);
	 // motor3_backward(100, 0.5);
	 //motor4_forward(4, 0.03);
	   //HAL_UART_Transmit_IT(&huart1,(uint8_t*)message,strlen(message));
	 
	 if(packetReady){
	 MoveToPoint(targetX_m, targetY_m, 0, 800, 800, 0); 
		packetReady = 0;
	  HAL_Delay(1000);
	
  }
	  
	// MoveToPoint( 0.03, 0.03, 0.00, 1000, 1000, 3);
	//   MoveToPoint( 0.02, 0.01, 0.00, 1000, 1000, 3);
	//MoveTo( 0.07, 0.07, 0.00, 3, 3, 3);
	 //while(1){Estimate_motors1234();
		 //HAL_Delay(100);
	 //}
	
	 // MoveToPoint(0.03, 0.03, 3, 3);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        //uint8_t byte;
		uint8_t byte = rxBuffer[rxIndex];
      
      
        if(rxIndex == 0){
			if(byte != 0xAA){
				rxIndex = 0;
                HAL_UART_Receive_IT(huart, &rxBuffer[rxIndex], 1);
                return;
			}
		} 

        rxIndex++;
        // 判断是否收到完整数据�?
        if(rxIndex >= PACKET_SIZE)
        {
            if(rxBuffer[6] == 0x55 && rxBuffer[5] == 0x01)
            {
				
				//char message[] = "received";
				//HAL_UART_Transmit_IT(&huart1,(uint8_t*)message,strlen(message));
                targetX_cm = (rxBuffer[1] << 8) | rxBuffer[2];
                targetY_cm = (rxBuffer[3] << 8) | rxBuffer[4];
				targetX_m = targetX_cm*(0.01);
				targetY_m = targetY_cm*(0.01);
				HAL_UART_Transmit_IT(&huart1,&targetX_cm,1);
				HAL_UART_Transmit_IT(&huart1,&targetY_cm,1);
				
                packetReady = 1;
            }
            rxIndex = 0;
        }

        HAL_UART_Receive_IT(huart, &rxBuffer[rxIndex], 1);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){}
}
 void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{ 
  uint8_t cmd[13] = {0};

  cmd[0]  =  addr;                      
  cmd[1]  =  0xFD;                      
  cmd[2]  =  dir;                       
  cmd[3]  =  (uint8_t)(vel >> 8);       
  cmd[4]  =  (uint8_t)(vel >> 0);        
  cmd[5]  =  acc;                       
  cmd[6]  =  (uint8_t)(clk >> 24);      
  cmd[7]  =  (uint8_t)(clk >> 16);      
  cmd[8]  =  (uint8_t)(clk >> 8);      
  cmd[9]  =  (uint8_t)(clk >> 0);       
  cmd[10] =  raF;                      
  cmd[11] =  snF;                      
  cmd[12] =  0x6B;                     
  
  HAL_UART_Transmit(&huart9,cmd,13,1000);
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  cmd[0] =  addr;                       
  cmd[1] =  0xFF;                       
  cmd[2] =  0x66;                       
  cmd[3] =  0x6B;                       
  
  HAL_UART_Transmit(&huart9,cmd,4,1000);
}
void Stop_Motor(uint8_t addr)
{
	Emm_V5_Pos_Control(addr, 0, 0, 0, 0, 0, 0);
}
uint16_t x_handle(float x){
    uint16_t location=0;
    location=3200*((x*100.0/(_2PI_ * 0.6 )));
    return location;
} 

void motors12_forward(uint16_t vel, float distance)
{
  HAL_Delay(200);
  uint32_t steps = x_handle(distance);
 
  Emm_V5_Pos_Control(1, 1, vel, 5, steps, 0, 1);
  HAL_Delay(50);
 
  Emm_V5_Pos_Control(2, 0, vel, 5, steps, 0, 1);
  HAL_Delay(50);
  // 发鿁同步命仿
  Emm_V5_Synchronous_motion(0);
  
  HAL_Delay(200);
}

void motors12_backward(uint16_t vel, float distance){
  HAL_Delay(200);
  //if(distance>0.24){distance=0.24;}
  uint32_t steps = x_handle(distance);
  
  Emm_V5_Pos_Control(1, 0, vel, 5, steps, 0, 1);
  HAL_Delay(50);
 
  Emm_V5_Pos_Control(2, 1, vel, 5, steps, 0, 1);
  HAL_Delay(20);
  // 发鿁同步命仿
  Emm_V5_Synchronous_motion(0);
  
  HAL_Delay(200);
}

void motor3_backward(uint16_t vel, float distance){
	HAL_Delay(200);
	//if(distance>0.21){distance=0.21;}
	
	uint32_t steps = x_handle(distance);
	Emm_V5_Pos_Control(3, 1, vel, 5, steps, 0, 0);
	HAL_Delay(20);
}
void motor3_forward(uint16_t vel, float distance){
	HAL_Delay(200);
	uint32_t steps = x_handle(distance);
	Emm_V5_Pos_Control(3, 0, vel, 5, steps, 0, 0);
	HAL_Delay(20);
}
//电机4是上下运�?
void motor4_down(uint16_t vel, float distance){
	HAL_Delay(200);
	//if(distance>0.15){distance=0.15;}
	
	uint32_t steps = x_handle(distance);
	Emm_V5_Pos_Control(4, 1, vel, 5, steps, 0, 0);
	HAL_Delay(20);
}
void motor4_up(uint16_t vel, float distance){
	HAL_Delay(200);
	uint32_t  
	steps = x_handle(distance);
	Emm_V5_Pos_Control(4, 0, vel, 5, steps, 0, 0);
	HAL_Delay(20);
}


/*static bool limitPA0Triggered = false;
static bool limitPA1Triggered = false;
static bool limitPA2Triggered = false;

void Estimate_motors1234(){
	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET){
		if (!limitPA0Triggered) {
            Stop_Motor(1);
			HAL_Delay(300);
			Stop_Motor(2);
			//更新位置承在坐栿
			currentPosition[0] = 0.0f;
			//motor1_state.is_moving = false;
			//motor2_state.is_moving = false;
			
            limitPA0Triggered = true;
        }
		else{limitPA0Triggered = false;}
	}

	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_RESET){
		if (!limitPA1Triggered) {
            Stop_Motor(3);
			
			currentPosition[1] = 0.0f;
            limitPA1Triggered = true;
			
			//motor3_state.is_moving = false;
			
        }
		else{limitPA1Triggered = false;}
	}
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_RESET){
		if (!limitPA2Triggered) {
            Stop_Motor(4);
			currentPosition[2] = 0.0f;
            limitPA2Triggered = true;
			
			//motor1_state.is_moving =false;
			
        }
		else{limitPA2Triggered = false;}
	}
	
	//if(CheckMovementComplete()){
		
	//}
	
}
*/
// 全局限位触发标志
volatile bool x_limit_triggered = false;
volatile bool y_limit_triggered = false;
volatile bool z_limit_triggered = false;


void CheckLimitSwitches(void)
{
    // X轴限�? (PA0)
    GPIO_PinState x_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if (x_state == GPIO_PIN_RESET) {if (!x_limit_triggered) {x_limit_triggered = true;}} 
	else {if (x_limit_triggered) {x_limit_triggered = false;}}
    
    // Y轴限�? (PA1)
    GPIO_PinState y_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    if (y_state == GPIO_PIN_RESET) {if (!y_limit_triggered) { y_limit_triggered = true;}} 
	else {if (y_limit_triggered) {y_limit_triggered = false;}}
    
    // Z轴限�? (PA2)
    GPIO_PinState z_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    if (z_state == GPIO_PIN_RESET) {if (!z_limit_triggered) {z_limit_triggered = true;}} 
	else {if (z_limit_triggered) {z_limit_triggered = false;}}
}


//跟新位置 
void UpdatePosition(float x, float y, float z)
{
    currentPosition[0] = x;
    currentPosition[1] = y;
    currentPosition[2] = z;
    
    // 边界约束
    currentPosition[0] = fmaxf(0.0f, fminf(MAX_X, currentPosition[0]));
    currentPosition[1] = fmaxf(0.0f, fminf(MAX_Y, currentPosition[1]));
    currentPosition[2] = fmaxf(0.0f, fminf(MAX_Z, currentPosition[2]));
}
// 返回原点位置
void HomePosition(uint16_t vel_x,uint16_t vel_y,uint16_t vel_z)
{
    // 移动到原�?(0,0)
    MoveToPoint(0.0, 0.0, 0.0, vel_x, vel_y, vel_z);
    
    // 更新位置
    UpdatePosition(0.0f, 0.0f, 0.0f);
}
// 移动到指定XY坐标位置 (单位:厘米)
void MoveToPoint(float x_m, float y_m,float z_m, uint16_t vel_x, uint16_t vel_y, uint16_t vel_z)
{    
	uint32_t tim=0;
	
	//要运动的距离
	float dx = x_m - currentPosition[0];
	float dy = y_m - currentPosition[1];
	float dz = z_m - currentPosition[2];
	//运动的方�?
	uint8_t x_dir = (dx >= 0) ? 0 : 1; // 0=正向, 1=反向
    uint8_t y_dir = (dy >= 0) ? 0 : 1;
	uint8_t z_dir = (dz >= 0) ? 0 : 1;
	
	float abs_dx = fabsf(dx);
    float abs_dy = fabsf(dy);
	float abs_dz = fabsf(dz);
	
    uint16_t x_steps = x_handle(abs_dx);
	uint16_t y_steps = x_handle(abs_dy);
	uint16_t z_steps = x_handle(abs_dz);
  
	//重置丿下标志�?
	x_limit_triggered = false;
    y_limit_triggered = false;
    z_limit_triggered = false;
    
	//计算运动时间
	uint32_t x_time = (x_steps > 0) ? (x_steps * 1000) / vel_x : 0;
    uint32_t y_time = (y_steps > 0) ? (y_steps * 1000) / vel_y : 0;
    uint32_t z_time = (z_steps > 0) ? (z_steps * 1000) / vel_z : 0;
	
	tim = x_time;
    if (y_time > tim) tim = y_time;
    if (z_time > tim) tim = z_time;
		
	uint32_t start_time = HAL_GetTick();

	
	if(x_dir == 0){motors12_forward( vel_x,abs_dx);}
	if(x_dir == 1){motors12_backward( vel_x,abs_dx);}
	
	if(y_dir == 0){motor3_forward( vel_y, abs_dy);}
	if(y_dir == 1){motor3_backward( vel_y, abs_dy);}
	
	if(z_dir == 0){motor4_up( vel_z, abs_dz);}
	if(z_dir == 1){motor4_down( vel_z, abs_dz);}
	
	bool x_complete = (x_steps == 0);
    bool y_complete = (y_steps == 0);
    bool z_complete = (z_steps == 0);
	
    while(1)
	{
        
            CheckLimitSwitches();
            
            // 如果X限位触发且X轴还在运动，标记为完�?
            if (x_limit_triggered && !x_complete) {
                Stop_Motor(1); 				
				HAL_Delay(10);
                Stop_Motor(2); 
                currentPosition[0] = 0.33f;  // 更新X位置
                x_complete = true;
            }
            
            // 如果Y限位触发且Y轴还在运动，标记为完�?
            if (y_limit_triggered && !y_complete) {
                Stop_Motor(3); 
                currentPosition[1] = 0.33f;  // 更新Y位置
                y_complete = true;
            }
            
            // 如果Z限位触发且Z轴还在运动，标记为完�?
            if (z_limit_triggered && !z_complete) {
                Stop_Motor(4); 
                currentPosition[2] = 0.0f;  // 更新Z位置
                z_complete = true;
            }
            
        uint32_t elapsed = HAL_GetTick() - start_time;
		
        
		if(elapsed>tim){
            // 更新未触发限位的位置
            if (!x_limit_triggered) currentPosition[0] = x_m;
            if (!y_limit_triggered) currentPosition[1] = y_m;
            if (!z_limit_triggered) currentPosition[2] = z_m;
            break;
		}
	   HAL_Delay(5);
    }

}

/*void MoveTo(float x_m, float y_m,float z_m, uint16_t vel_x, uint16_t vel_y, uint16_t vel_z)
{    
	uint16_t tim=0;
	 x_m = fmaxf(0.0f, fminf(MAX_X, x_m));
     y_m = fmaxf(0.0f, fminf(MAX_Y, y_m));
	 z_m = fmaxf(0.0f, fminf(MAX_Z, z_m));
	
	
	//要运动的距离
	float dx = x_m - currentPosition[0];
	float dy = y_m - currentPosition[1];
	float dz = z_m - currentPosition[2];
	//运动的方�? 
	uint8_t x_dir = (dx >= 0) ? 0 : 1; // 0=正向, 1=反向
    uint8_t y_dir = (dy >= 0) ? 0 : 1;
	uint8_t z_dir = (dz >= 0) ? 0 : 1;
	
	float abs_dx = fabsf(dx);
    float abs_dy = fabsf(dy);
	float abs_dz = fabsf(dz);
	
	uint32_t x_steps = x_handle(abs_dx);
    uint32_t y_steps = x_handle(abs_dy);
	uint32_t z_steps = x_handle(abs_dz);
	//计算运动时间
	uint32_t x_time = (x_steps > 0) ? (x_steps * 1000) / vel_x : 0;
    uint32_t y_time = (y_steps > 0) ? (y_steps * 1000) / vel_y : 0;
    uint32_t z_time = (z_steps > 0) ? (z_steps * 1000) / vel_z : 0;

	
	if(x_dir == 0){motors12_forward( vel_x,abs_dx);}
	if(x_dir == 1){motors12_backward( vel_x,abs_dx);}
	
	if(y_dir == 0){motor3_forward( vel_y, abs_dy);}
	if(y_dir == 1){motor3_backward( vel_y, abs_dy);}
	
	if(z_dir == 0){motor4_up( vel_z, abs_dz);}
	if(z_dir == 1){motor4_down( vel_z, abs_dz);}
	
    if ((x_time >= y_time) && (x_time >= z_time)) {
	    tim = x_time;
	}
			
    if ((y_time >= x_time) && (y_time >= z_time)){ 
		tim = y_time;
	}
    if ((z_time >= x_time) && (z_time >= y_time)){
		tim = z_time;
	}
	HAL_Delay(tim);
	
	//跟新丿下坐栿
  
}
*/
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
