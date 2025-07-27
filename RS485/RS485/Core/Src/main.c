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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wit_c_sdk.h"
#include "usart6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ID  0x01
#define POSITION_REG 0x80
#define TORQUE_REG 0x81
#define PACKET_SIZE 20


#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
/*typedef struct {
    short x;
    short y;
    short z;
} GyroData;

GyroData gyro_data;
uint8_t mpu_data_ready = 0;
uint32_t mpu_last_update = 0;
char uart_buf[64];  // 用于存储格式化后的字符串*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define MODBUS_READ_REG 0x03
#define MODBUS_WRITE_SINGLE_REG 0x06
#define MODBUS_WRITE_MULTI_REG 0x10
#define MAX_ANGLE 360.0f
#define RESOLUTION 4096
volatile uint32_t sys_tick = 0;
uint8_t flag_10ms = 0;
uint8_t flag_100ms = 0;
uint16_t flag_1000ms = 0;
uint32_t last_tick = 0;
uint16_t target_angle = 0;
uint16_t current_angle = 0;

uint8_t usart_read_byte;

uint8_t usart1_tx_byte;
uint8_t usart1_tx_busy = 0;

#pragma pack(push, 1)
typedef struct {
    uint8_t header;     // 包头 0xAA
    float pitch;        // 俯仰�?
    float roll;         // 横滚�?
    float yaw;          // 偏航�?
    int16_t gx;         // �?螺仪X�?
    int16_t gy;         // �?螺仪Y�?
    int16_t gz;         // �?螺仪Z�?
    uint8_t footer;     // 包尾 0x55
} IMU_DataPacket;
#pragma pack(pop)

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
uint8_t packetReady = 0;
IMU_DataPacket received_data;

#define PRINT_BUFFER_SIZE 128
char print_buffer[PRINT_BUFFER_SIZE];
uint8_t print_ready = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

typedef enum {
    STATE_IDLE,
    STATE_SEND_CMD1,
    STATE_WAIT_CMD1,
    STATE_SEND_CMD2,
    STATE_WAIT_CMD2,
	STATE_SEND_CMD3,
	STATE_WAIT_CMD3,
	STATE_SEND_CMD4,
	STATE_WAIT_CMD4
	
} ServoControlState;


ServoControlState servo_state = STATE_SEND_CMD1;
uint32_t state_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Modbus_CRC16(uint8_t *data,uint16_t length);
void Set_Servo_Angle(UART_HandleTypeDef *huart, uint8_t servo_id, uint16_t angle_deg);
void Set_Servo_Speed(UART_HandleTypeDef *huart,uint8_t servo_id, int16_t speed_deg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void CmdProcess(void);
void CopeCmdData(unsigned char ucData);

static void ShowHelp(void);

static void AutoScanSensor(void);

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void Delayms(uint16_t ucMs);

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
void USART6_IRQHandler(void);


void Uart6Send(unsigned char *p_data, unsigned int uiSize);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint16_t Modbus_CRC16(uint8_t *data,uint16_t length);
void Enable_Servo_Torque(UART_HandleTypeDef *huart, uint8_t servo_id, uint16_t enable);
void Set_Servo_Angle(UART_HandleTypeDef *huart, uint8_t servo_id, uint16_t angle_deg);
void Set_Servo_Speed(UART_HandleTypeDef *huart,uint8_t servo_id, int16_t speed_deg);
void Update_Servo_State(uint16_t x1,uint16_t x2,uint16_t x3,uint16_t x4);
void MoveToAngle   (uint16_t x1,uint16_t x2,uint16_t x3,uint16_t x4);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		static uint8_t counter_10ms = 0;
		static uint8_t counter_100ms = 0;
		static uint16_t counter_1000ms = 0;
		sys_tick++;
		if(++counter_10ms >= 10){
		flag_10ms = 1;
		counter_10ms = 0;
		}
		if(++counter_100ms >= 100){
		flag_100ms = 1;
		counter_100ms = 0;
		}
		if(++counter_1000ms >= 1000){
		flag_1000ms = 1;
		counter_1000ms = 0;
		}
	}
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    /*short gx, gy, gz;
	uint32_t last_mpu_read = HAL_GetTick();
    uint32_t last_mpu_print = HAL_GetTick();
	char uart_buf[64];*/
	float fAcc[3], fGyro[3], fAngle[3];
	int i;
	Usart6Init(9600);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart3, &rxBuffer[0], 1);
  
  Enable_Servo_Torque(&huart2, 0x03, 1);
  //HAL_Delay(100);
    //current_angle = 90;
    //target_angle = 270;
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();
	//servo_state = STATE_SEND_CMD1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //CmdProcess();
	CmdProcess();
	for(i = 0; i < 3; i++)
			   {
			    	fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
			   	  fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			   }
			  /* if(s_cDataUpdate & ACC_UPDATE)
			   {
				    printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				    s_cDataUpdate &= ~ACC_UPDATE;
			   }
			   if(s_cDataUpdate & GYRO_UPDATE)
			   {
			   	printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
			   	s_cDataUpdate &= ~GYRO_UPDATE;
			   }*/
			   if(s_cDataUpdate & ANGLE_UPDATE)
			   {
			   	printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
			   	s_cDataUpdate &= ~ANGLE_UPDATE;
			   }
			  /* if(s_cDataUpdate & MAG_UPDATE)
			   {
			   	printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
			   	s_cDataUpdate &= ~MAG_UPDATE;
			   }*/
	  
	  //Set_Servo_Angle(&huart2,0x02,90);
	   
	 /* if (flag_10ms) {
            flag_10ms = 0;
            
            if (current_angle != target_angle) {
                if (current_angle < target_angle) {
                    current_angle++;
                } else {
                    current_angle--;
                }
                
                Set_Servo_Angle(&huart2, 0x02, current_angle);
            }
        }*/
 
	 
	  //Set_Servo_Angle(&huart2, 0x03, 90);
	  //HAL_Delay(100); 
      /*if (packetReady) {
        packetReady = 0;
        
        // 提取并处理传感器数据
        float pitch = received_data.pitch;
        float roll = received_data.roll;
        float yaw = received_data.yaw;
        int16_t gx = received_data.gx;
        int16_t gy = received_data.gy;
        int16_t gz = received_data.gz;
        int len = snprintf(print_buffer,PRINT_BUFFER_SIZE, "P:%.2f\r\n" , pitch);
		
		  
	    if(!usart1_tx_busy){
			usart1_tx_busy = 1;
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)print_buffer, len);
		
		uint16_t pitch_putout = (uint16_t)((pitch*2)+180);
	    if(flag_1000ms){  
		  Set_Servo_Angle(&huart2, 0x03, pitch_putout);
		
		HAL_Delay(1000);
		}
	  }
		
		int len = snprintf(print_buffer,PRINT_BUFFER_SIZE, "P:%.2f  R:%.2f Y:%.2f\r\nGX:%d GY:%d GZ:%d\r\n\r\n",
			  received_data.pitch,
		      received_data.roll,
		      received_data.yaw,
		      received_data.gx,
		      received_data.gy,
		      received_data.gz);
		
		print_ready = 1;}
	    if(print_ready && !usart1_tx_busy){
			usart1_tx_busy = 1;
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)print_buffer, strlen(print_buffer));
		}*char msg[64];
		  int len = sprintf(msg, "P:%.2f  R:%.2f Y:%.2f GX:%d GY:%d GZ:%d\r\n",
			  pitch,
		      roll,
		      yaw,
		      gx,
		      gy,
		      gz);
        HAL_UART_Transmit(&huart1,(uint8_t*)msg,len,10);	  
	}*/
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 275-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint16_t Modbus_CRC16(uint8_t *data,uint16_t length){
	uint16_t crc = 0xFFFF;
	for(uint16_t i = 0; i< length; i++){
		crc ^= data[i];
		for(uint8_t j = 0; j<8; j++){
			if(crc & 0x0001){
				crc >>= 1;
				crc ^= 0xA001;
			}
		else{
			crc >>= 1;
		}	
		}
	}
	return crc;	
}
void Enable_Servo_Torque(UART_HandleTypeDef *huart, uint8_t servo_id, uint16_t enable){
	uint8_t cmd[8] = {
		ID,
		MODBUS_WRITE_SINGLE_REG,
		0x00,0x81,
		0x00,
		(uint8_t)(enable & 0xFF),
		0x00,0x00
	};
	uint16_t crc = Modbus_CRC16(cmd,6);
	cmd[6] = crc & 0xFF;
	cmd[7] = crc >> 8;
	
	HAL_UART_Transmit(huart, cmd, 8, 100);
}

void Set_Servo_Angle(UART_HandleTypeDef *huart, uint8_t servo_id, uint16_t angle_deg){
	
	
	uint16_t register_value = (angle_deg * RESOLUTION) / MAX_ANGLE;
	uint8_t cmd[8] = {
		servo_id,
		0x06,
		0x00, 0x80,
		register_value >> 8,
		register_value & 0xFF,
		0x00, 0x00
		
	};
	uint16_t crc = Modbus_CRC16(cmd,6);
	cmd[6] = crc & 0xFF;
	cmd[7] = crc >> 8;
	
	HAL_UART_Transmit(huart, cmd, 8, 1);
}
/*void Change_Angle(uint16_t angle_deg){
	Set_Servo_Angle(&huart2, 0x01, angle_deg);
	if(flag_10ms == 1){
	for(int angle = angle_deg;angle++; angle < 360){
		Set_Servo_Angle(&huart2, 0x01, angle);
		HAL_Delay(10);
		if(angle == 365){
		angle = 0;
		}
	}
}
}*/
void Set_Servo_Speed(UART_HandleTypeDef *huart,uint8_t servo_id, int16_t speed_deg){
	int16_t register_value = (speed_deg * 1000) / 360;
	
	uint8_t cmd[8] = {
		servo_id,
		0x06,
		0x00, 0x83,
		register_value >>8,
		register_value & 0xFF,
		0x00,0x00
	};
	uint16_t crc = Modbus_CRC16(cmd,6);
	cmd[6] = crc & 0xFF;
	cmd[7] = crc >> 8;
	
	HAL_UART_Transmit(huart, cmd , 8, 100);
}   
void Set_Servo_Mode(UART_HandleTypeDef *huart, uint8_t servo_id, uint8_t mode){
	uint8_t lock_cmd[8] = {
		servo_id,
		MODBUS_WRITE_SINGLE_REG,
		0x00, 0x85,
		0x00, 0x00,
		0x00, 0x00
	};
	uint16_t crc = Modbus_CRC16(lock_cmd,6);
	lock_cmd[6] = crc & 0xFF;
	lock_cmd[7] = crc >> 8;
	
	HAL_UART_Transmit(huart, lock_cmd , 8, 100);
	HAL_Delay(20);
	
	uint8_t cmd[8] = {
		servo_id,
		MODBUS_WRITE_SINGLE_REG,
		0X00,0X10,
		0X00,mode,
		0x00,0x00
	};
    crc = Modbus_CRC16(cmd,6);
	cmd[6] = crc & 0xFF;
	cmd[7] = crc >> 8;
	
	HAL_UART_Transmit(huart, cmd , 8, 100);

}

void Update_Servo_State(uint16_t x1,uint16_t x2,uint16_t x3,uint16_t x4) {
    switch(servo_state) {
        case STATE_IDLE:
          
            break;
            
        case STATE_SEND_CMD1:
            // 发�?�第�?个舵机命�?
            Set_Servo_Angle(&huart2, 0x01, x1);
            state_timer = sys_tick;  // 记录发�?�时�?
            servo_state = STATE_WAIT_CMD1;  // 切换到等待状�?
            break;
            
        case STATE_WAIT_CMD1:
            
            if(sys_tick - state_timer >= 5) {
                servo_state = STATE_SEND_CMD2;  // 切换到发送第二个命令
            }
            break;
            
        case STATE_SEND_CMD2:
            // 发�?�第二个舵机命令
            Set_Servo_Angle(&huart2, 0x02, x2);
            state_timer = sys_tick; 
            servo_state = STATE_WAIT_CMD2;  
            break;
            
        case STATE_WAIT_CMD2:
           
            if(sys_tick - state_timer >= 5) {
                servo_state = STATE_SEND_CMD3; 
            }
            break;
			
		case STATE_SEND_CMD3:
			//发�?�第三个舵机命令
		    Set_Servo_Angle(&huart2, 0x03, x3);
		    state_timer = sys_tick; 
            servo_state = STATE_WAIT_CMD3;  
            break;
		
        case STATE_WAIT_CMD3:
           
            if(sys_tick - state_timer >= 10) {
                servo_state = STATE_SEND_CMD4; 
            }
            break;	
      
        case STATE_SEND_CMD4:
			//发�?�第四个舵机命令
		    Set_Servo_Angle(&huart2, 0x04, x4);
		    state_timer = sys_tick; 
            servo_state = STATE_WAIT_CMD4;  
            break;
       
        case STATE_WAIT_CMD4:
           
            if(sys_tick - state_timer >= 5) {
                servo_state = STATE_SEND_CMD1; 
            }
            break;

			
    }
}

void MoveToAngle(uint16_t x1,uint16_t x2,uint16_t x3,uint16_t x4){
	 Update_Servo_State(x1,x2,x3,x4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)  
    {   
        uint8_t byte = rxBuffer[rxIndex];  // 获取接收到的字节
        
		/*if(!usart1_tx_busy){
			usart1_tx_byte = byte;
			usart1_tx_busy = 1;
			HAL_UART_Transmit_IT(&huart1, &usart1_tx_byte, 1);
		}*/
        // 状�?�机处理
		         
        if(rxIndex == 0) {
            // �?查包�?
            if(byte != 0xAA) {
                rxIndex = 0;  // 包头错误，重置索�?
                HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);
                return;
            }
        } 
           if(rxIndex == PACKET_SIZE - 1) {
            // �?查包�?
			
            if(byte == 0x55) {
                // 完整接收�?包数�?
                for(int i = 0; i < PACKET_SIZE; i++) {
                    ((uint8_t*)&received_data)[i] = rxBuffer[i];
				
				}
                packetReady = 1;
            }
            rxIndex = 0; 
        }
        else {
            rxIndex++;  // 继续接收数据
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart3, &rxBuffer[rxIndex], 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart ->Instance == USART1){
		usart1_tx_busy = 0;
	}
}

//传感器部�?

static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart6Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart6Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart6Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart6Init(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			HAL_Delay(100);
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}


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
  /* User can add his own implementation to report the file name  and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
