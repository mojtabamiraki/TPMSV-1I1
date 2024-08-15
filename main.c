/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "INA219.h"
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURRENT_THRESHOLD_CONNECTED 5.0f        // Voltage threshold for detecting device connection
#define CURRENT_MIN_THRESHOLD 0xFFDC            // Lower bound for current check
#define CURRENT_MAX_THRESHOLD 0xFFFF            // Upper bound for current check
#define SHORT_CIRCUIT_CURRENT_THRESHOLD 900.0f   // Short circuit current threshold in (milliamps*10)
uint32_t TxMailbox;
uint8_t a; //declare byte to be transmitted //declare a receive byte
uint8_t DataReceived_CAN[8];
uint8_t Sensor_ID[8] = {0x01, 0x12, 0x29, 0x18, 0x00, 0x00, 0x00, 0x00};
int DataReceived_CAN_Flag = 0;
INA219_t ina219;
float BusCurrent = 0.0f;
static bool wasDisconnected = false;  // Variable to track previous device connection status
static bool testInProgress = false;    // Variable to track if a test is currently in progress
static bool TestFlag = true;
uint16_t ShortCircuitCurrent = 0;
bool ShortCircuitFlag = false;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void Test_Procces(void);
void PowerOn_System(void);
void PowerOn_Ignition(void);
void WaitForCANData(void);
bool CheckSensorIDMatch(const uint8_t*, const uint8_t*);
void PowerOff_Test(void);
void ResetCANData(uint8_t*);
void SendStringOverUSART(int , int , int );
bool checkShortCircuit(void);
bool performTest(void);
void checkConnection(void);
void Delay_ms(uint32_t );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
	MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS)){}                                       // Setting up the current sensor
		/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
     Test_Procces();

		/* USER CODE END 3 */
  }
	/* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */
void Test_Procces ( void )
{
	uint8_t dataToSend[8] = {0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,};                            // 8-byte Data to send on CAN -> Then TPMS Returns the ID of the sensor.
	uint8_t StartTest_Switch_State = 0;                                                            // A variable to check Test Start
	int Normal_Current = 0;                                                                        // A variable to store the current value in the normal working state of the system.
	bool allMatch = false;                                                                         // To check if the Data Reveived on CAN matches the sensor ID or not?
	bool dataWasSent = false;                                                                      // Flag to track if data was sent
  uint16_t IDL_Current = 0;                                                                      // A variable to store the current value in the Idle working state of the system.
	StartTest_Switch_State = LL_GPIO_ReadInputPort(GPIOA) & LL_GPIO_PIN_0;                         // Read The state of the GPIOA_PIN0 that is connected to start key.	                                                                             // Continuously check the connection status
	checkConnection();
	if ( StartTest_Switch_State == 0 )                                                             // To check if the Test started or not?
	{
			PowerOn_System();
			HAL_Delay(1000);
		  //checkConnection();
			if ( performTest() )
			{
				IDL_Current = INA219_ReadCurrent_raw(&ina219);                                               // Save current in idle mode.
				PowerOn_Ignition();
				Delay_ms(2500);                                                                             // Create a time to activate the trigger.
				Normal_Current = INA219_ReadCurrent_raw(&ina219);	                                           // Current measurement in normal working mode.
				if  ( (HAL_CAN_AddTxMessage(&hcan, &pHeader, dataToSend, &TxMailbox) == HAL_OK) || (ShortCircuitFlag == true))              // Whether the message was sent successfully or not.
				{		
					dataWasSent = true; // Set flag to indicate data was sent
				}		
				if ( dataWasSent == true)
				{
					WaitForCANData();			                                                                     // Wait for 3 seconds to receive data from CAN or not, otherwise end the test.
					if (DataReceived_CAN_Flag == 1)
					{
						allMatch = CheckSensorIDMatch( DataReceived_CAN , Sensor_ID );                           // Check whether the data received from CAN matches the existing sensor ID or not.						
						if  (allMatch)
							{
								PowerOff_Test();                                                                     // Disconnect the system relays, the sensor is healthy and working properly and finish the test.
								ResetCANData(DataReceived_CAN);                                                      // Empty the buffer of data received from CAN.
								SendStringOverUSART( 2 , IDL_Current, Normal_Current );                              // Test success data by sending number 2, and send idle and normal current on Uart.
								HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);                                               // Turn on the green LED to show that the test is successful.
								IDL_Current = 0;
								Normal_Current = 0;
							}
						else
							{
								HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,0);                                               // Turn on the red LED for test failure.
								PowerOff_Test();                                                                     // Disconnect the system power relays.
								ResetCANData(DataReceived_CAN);	                                                     // Empty the buffer of data received from CAN.
								SendStringOverUSART( 1 , IDL_Current, Normal_Current );	                             // Send the number 1 as we received data from the sensor but it did not match the sensor ID and failed the test.				
								IDL_Current = 0;
								Normal_Current = 0;
							}				
					}
					else if ( (DataReceived_CAN_Flag != 1) && (ShortCircuitFlag == true) )
					{							                                                                       
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);	                                               // Turn on the red LED for test failure.				
							PowerOff_Test();                                                                       // If short circuit exists, power off the system
						  ResetCANData(DataReceived_CAN);	                                                       // Empty the buffer of data received from CAN.				
							SendStringOverUSART( 3 , 0 , 0 );                                                       // Send 0 as no data received from CAN and test failed.
							IDL_Current = 0;
							Normal_Current = 0;
						  ShortCircuitCurrent = 0;						
					}
					else
						{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,0);	                                               // Turn on the red LED for test failure.				
							PowerOff_Test();                                                                       // Disconnect the system power relays.
							ResetCANData(DataReceived_CAN);	                                                       // Empty the buffer of data received from CAN.
							SendStringOverUSART( 0 , IDL_Current, Normal_Current );                                // Send 0 as no data received from CAN and test failed.
							IDL_Current = 0;
							Normal_Current = 0;
						}
				}
				else
					{
						SendStringOverUSART( 3 , 0, 0 );			                                                   // The tester has a problem and cannot send data on CAN.
					}
					wasDisconnected = true;
					TestFlag = false;
			}
			else{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,0);	                                               // Turn on the red LED for test failure.				
							PowerOff_Test();                                                                       // Disconnect the system power relays.
			}
			
	 }
}


void Delay_ms(uint32_t Delay)
{
    __IO uint32_t tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
    /* Indicate that the local variable is not used */
    ((void)tmp);

    /* Add a period to guarantee minimum wait */
    if (Delay < LL_MAX_DELAY)
    {
        Delay++;
    }

    while (Delay)
    {
        // Check the state of ShortCircuitFlag
       // if (ShortCircuitFlag == true)
       // {
        //    break; // Exit the function if ShortCircuitFlag is true
       // }

        // Check if the delay time has elapsed
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
        {
            Delay--;
        }
    }
}
void checkConnection(void) {
			
	BusCurrent = INA219_ReadCurrent_raw(&ina219);
    // Check device connection based on current
    if (BusCurrent <= CURRENT_THRESHOLD_CONNECTED || 
        (BusCurrent >= CURRENT_MIN_THRESHOLD && BusCurrent <= CURRENT_MAX_THRESHOLD)) {
        
        // Device is disconnected
        wasDisconnected = true;  // Mark that the device is disconnected
				TestFlag = true;
    } else {
        // Device is connected
        if (wasDisconnected && !testInProgress) {
            // If the device was previously disconnected and no test is in progress, allow test to start
            testInProgress = true;  // Mark that a test is now in progress
        }
        wasDisconnected = false;  // Mark that the device is connected
    }
}

bool performTest(void) {
    // Check if the device was previously disconnected
    if ( (!wasDisconnected && testInProgress) && (TestFlag) ){
        // Start the test
        // ...
        wasDisconnected = false;  // Reset the disconnection status after starting the test
        testInProgress = false;    // Reset test in progress status
        return true;  // Test started successfully
    } else {
        // The device was not previously disconnected or a test is already in progress
        return false; // Cannot start the test yet
    }
}
bool checkShortCircuit() {
    // Read current value from sensor
    ShortCircuitCurrent = INA219_ReadCurrent_raw(&ina219);
    
    // Check if current is greater than 100 mA
		if ( (ShortCircuitCurrent > SHORT_CIRCUIT_CURRENT_THRESHOLD) &&  (ShortCircuitCurrent < CURRENT_MIN_THRESHOLD )) {
        return true; // If current is greater than 100 mA, return true (indicating a potential short circuit)
    } else {
        return false; // Otherwise, return false
    }
}
void SendStringOverUSART(int status, int IDL_Current, int Normal_Current)
{
 char SendData_Over_USART[13];
 char buffer1[3];
	SendData_Over_USART[0] = '0';
	SendData_Over_USART[1] = 'X';
	SendData_Over_USART[2] = '0';
	SendData_Over_USART[3] = '0';
//-----------------------------
	snprintf(buffer1, 3 , "%d", status);
	SendData_Over_USART[4] = buffer1[0];
/////////////////////////////////////////	
	snprintf(buffer1, 3 , "%d", IDL_Current);
	SendData_Over_USART[5] = buffer1[0];
	SendData_Over_USART[6] = buffer1[1];
	snprintf(buffer1, 3 , "%d", Normal_Current);
	SendData_Over_USART[7] = buffer1[0];
	SendData_Over_USART[8] = buffer1[1];
/////////////////////////////////////////
	SendData_Over_USART[9] = '0';
	SendData_Over_USART[10] = 'F';
	SendData_Over_USART[11] = '0';
	SendData_Over_USART[12] = '0';	
  for (int i = 0; i < 13; i++)
	{
    while (!LL_USART_IsActiveFlag_TXE(USART1)); 
    LL_USART_TransmitData8(USART1, SendData_Over_USART[i]);
		HAL_Delay(20);

  }
}
void PowerOn_System(void)
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1); // To turn off the green LED used by the user in the initial start mode.
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,1); // To turn off the Red LED used by the user in the initial start mode.	
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0); // Connecting the power supply relay.
}
void PowerOn_Ignition (void)
{
	  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);	
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
}
void WaitForCANData(void)
{
		uint32_t CAN_ReceiveCounter = 0;                                                               // Variable to track the time of Receiving Data on CAN
		CAN_ReceiveCounter = HAL_GetTick();
		while ( ((DataReceived_CAN_Flag == 0) && (HAL_GetTick() - CAN_ReceiveCounter <= 3000)) && (ShortCircuitFlag == false) );	
}

bool CheckSensorIDMatch(const uint8_t* DataReceived_CAN, const uint8_t* Sensor_ID)
{
	  // Array matching check
    for (int i = 0; i < 8; i++) {
        if (DataReceived_CAN[i] != Sensor_ID[i]) {
            return false; // If a mismatch is found.
        }
    }
    return true; // If all matches.
}
void PowerOff_Test(void)
{
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
}

void ResetCANData(uint8_t* DataReceived_CAN)
{
  for (int i = 0; i < 8; i++)
	{
    DataReceived_CAN[i] = 0; 
  }
    DataReceived_CAN_Flag = 0; 
}
 

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  pHeader.DLC = 8; //give message size of 8 byte
  pHeader.IDE = CAN_ID_STD; //set identifier to standard
  pHeader.RTR = CAN_RTR_DATA; //set data type to remote transmission request
  pHeader.StdId = 0x244; //define a standard identifier, used for message identification by filters

  //filter one (stack light blink)
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
  sFilterConfig.FilterIdHigh = 0x245 << 5; //the ID that the filter looks for
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

  HAL_CAN_Start(&hcan); //start CAN
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    /**/
    // Reset output pins (excluding PA1)
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5);
    /**/
    
    // Configure PA1 as input with Pull-Up
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1; // Set PA1 pin
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT; // Input mode
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; // Enable Pull-Up
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    // Configure EXTI for PA1
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE1); // Change to LINE1 for PA1

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1; // Change to LINE1 for PA1
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT; // Interrupt mode
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING; // Trigger on falling edge
    LL_EXTI_Init(&EXTI_InitStruct);

    /* Configure other pins as output */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5; // Other output pins
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; // Output mode
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW; // Low speed
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; // Push-pull output
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure GPIO pins on Port B
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9; // GPIO pins on Port B
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set GPIO pins on Port B to high level
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9);

    /* EXTI interrupt init */
    NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); // Set priority for EXTI1
    NVIC_EnableIRQ(EXTI1_IRQn); // Enable EXTI1 interrupt

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // Enable clock for TIM2
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Configure TIM2
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7200 - 1; // Set Prescaler for 10 kHz (72MHz / 7200)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 500 - 1; // Set period for 50 milliseconds (10 kHz * 50ms)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler(); // Handle error
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler(); // Handle error
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler(); // Handle error
    }

    // Start the timer with interrupt
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    // Check for short circuit condition
   // if (checkShortCircuit())
   // {
		//	ShortCircuitFlag = true;
   // }
  /* USER CODE END TIM2_IRQn 1 */
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
