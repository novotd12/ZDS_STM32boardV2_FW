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
#include "fir.h"

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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
volatile uint16_t clockCounter = 0;
volatile int32_t decRatio = 32; // 14bit output
volatile uint8_t sendFlag = 0;
volatile uint8_t uartDmaFlag = 0;
volatile uint8_t dmaBuff[128]={'t','e','s','t','\r','\n'};
volatile uint8_t dmaSize = 0;

uint8_t printNumToBuff(uint8_t offset, uint8_t *buff, int32_t val, uint8_t dig, uint8_t signs) ;
volatile int32_t actFirValDec = 0;
volatile uint8_t actAdcFlag = 0;
volatile uint8_t adcRunning = 0;
volatile int32_t actFirVal = 0;
volatile int32_t actAdcVal = 0;
volatile float outValue = 0;
volatile int32_t accValue = 0;

#define OFFmode	0
#define FIRmode	1
#define MAmode	2
#define SARmode	3
#define RAWmode	4
#define SINCmode 5
#define TRACKmode 6

volatile uint8_t mode = OFFmode;
volatile uint8_t rxData = 0;

volatile int64_t Integrator_1 = 0;
volatile int64_t Integrator_2 = 0;
volatile int64_t Integrator_3 = 0;
volatile int64_t Integrator_3_d2 = 0;
volatile int64_t diff_1 = 0;
volatile int64_t diff_2 = 0;
volatile int64_t diff_3 = 0;
volatile int64_t diff_1_d = 0;
volatile int64_t diff_2_d = 0;

volatile int32_t uartRxCt=0;
volatile int32_t uartTxCt=0;
volatile float MAoutput = 0;

#define LOW		0
#define HIGH	1
volatile int32_t adcTrackingCount = 0;
volatile int32_t test_cnt = 0;

MovingAverage_t ma_filter;
volatile uint32_t lastDecRatio = 0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sincReset(void){
	diff_1  = 0;
	diff_3 = 0;
	diff_2 = 0;
	diff_1_d = 0;
	diff_2_d = 0;
	Integrator_1 = 0;
	Integrator_2 = 0;
	Integrator_3 = 0;
	Integrator_3_d2 = 0;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uartRxCt=200;

	Integrator_1 =0;
	Integrator_2 =0;
	Integrator_3 =0;

	switch(rxData){


		case 'h': mode = SARmode; decRatio = 1;    break; // SAR 0 0 ... up to 5 kHz
		case 'i': mode = RAWmode; decRatio = 1;    break; // RAW (1/0) ... up to 50 kHz

		case 'j': mode = SINCmode; decRatio = 32;   break; // SAR SINC MA ... up to 100kHz
		case 'k': mode = SINCmode; decRatio = 64;   break; // SAR SINC MA ... up to 100kHz
		case 'l': mode = SINCmode; decRatio = 128;  break; // SAR SINC MA ... up to 100kHz
		case 'm': mode = SINCmode; decRatio = 256;  break; // SAR SINC MA ... up to 100kHz
		case 'n': mode = SINCmode; decRatio = 512;  break; // SAR SINC MA ... up to 100kHz
		case 'o': mode = SINCmode; decRatio = 1024; break; // SAR SINC MA ... up to 100kHz

		case 'r': mode = TRACKmode; htim4.Instance->ARR = 999; break; // 1kHz
		case 's': mode = TRACKmode; htim4.Instance->ARR = 99; break; // 10kHz
		case 't': mode = TRACKmode; htim4.Instance->ARR = 19; break; // 50kHz

		default: break;
	}
	if(lastDecRatio != decRatio){
		lastDecRatio = decRatio;
		MovingAverage_SetSize(&ma_filter, decRatio);
		sincReset();
	}

	  UART_Start_Receive_IT(&huart1, (uint8_t*)&rxData, 1);
}

void EXTI3_IRQHandler(void){
	if(__HAL_GPIO_EXTI_GET_IT(SD_CLK_Pin) != 0x00u) {
	    __HAL_GPIO_EXTI_CLEAR_IT(SD_CLK_Pin);
	    if(mode!=RAWmode){
    	if(mode!=SARmode){
    		if(mode!=OFFmode){
    			int32_t data_in = 0;
				if(HAL_GPIO_ReadPin(SD_DOUT_GPIO_Port, SD_DOUT_Pin)==GPIO_PIN_RESET){
					//accValue += 10000;
					data_in = 1;
					//if(mode == FIRmode){
					//	actFirVal = updateFIR(+10000);
					//}
				}else{
					//accValue -= 10000;
					data_in = -1;
					//if(mode == FIRmode){
					//	actFirVal = updateFIR(-10000);
					//}
				}
		        MAoutput = MovingAverage_Update(&ma_filter, data_in*10000);

				if(clockCounter<(decRatio-1)){
					clockCounter++;
				}else{
					clockCounter=0;


					diff_3 = diff_2 - diff_2_d;
					diff_2_d = diff_2;
					diff_2 = diff_1 - diff_1_d;
					diff_1_d = diff_1;
					diff_1 = Integrator_3 - Integrator_3_d2;
					Integrator_3_d2 = Integrator_3;

					outValue = diff_3*10000.0f;

					accValue = 0;
					sendFlag = 1;
					if(adcRunning==0){
						HAL_ADC_Start_IT(&hadc1);
						adcRunning = 1;
					}
				}
				Integrator_3 += Integrator_2;
				Integrator_2 += Integrator_1;
				Integrator_1 += data_in;
    		}
    	}else{
			actFirValDec = 0;
			outValue = 0;
			sendFlag = 1;
    		if(adcRunning==0){
    	 	  HAL_ADC_Start_IT(&hadc1);
    	 	  adcRunning = 1;
    		}
			}
		} else {
			if (HAL_GPIO_ReadPin(SD_DOUT_GPIO_Port, SD_DOUT_Pin) == GPIO_PIN_RESET) {
				if (uartDmaFlag == 0) {
					dmaSize = 0;
					*(dmaBuff + dmaSize) = '0';
					dmaSize++;
					*(dmaBuff + dmaSize) = '\n';
					dmaSize++;
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dmaBuff, dmaSize);
					uartDmaFlag = 1;
				}
			} else {
				if (uartDmaFlag == 0) {
					dmaSize = 0;
					*(dmaBuff + dmaSize) = '1';
					dmaSize++;
					*(dmaBuff + dmaSize) = '\n';
					dmaSize++;
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dmaBuff, dmaSize);
					uartDmaFlag = 1;
				}
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uartDmaFlag  = 0;
	uartTxCt=200;
}

uint8_t printNumToBuff(uint8_t offset, uint8_t *buff, int32_t val, uint8_t dig, uint8_t signs) {
	uint8_t pos = offset;
	uint8_t i = 0;
	uint8_t digits[40];

	if (val < 0) {
		val *= -1;
		*(buff+pos) = '-';
		pos++;
	} else {
		if(signs){
			*(buff+pos) = '+';
			pos++;
		}
	}

		while (val > 0) {
			digits[i] = val % 10;
			val /= 10;
			i++;
		}
		for (uint8_t x = dig; x > 0; x--) {
			if (i < x) {
				*(buff+pos) = '0';
				pos++;
			}
		}
		for (uint8_t x = 0; x < i; x++) {
			*(buff+pos)  = digits[i - x - 1] + 0x30;
			pos++;
		}
		return pos;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	  adcRunning = 0;
	  actAdcVal = HAL_ADC_GetValue(hadc);
	  actAdcFlag = 1;
}




void myDelayUs(uint32_t time){
htim2.Instance->CNT = 0;
while((htim2.Instance->CNT)<time);
}


void writeGPIO(uint8_t pinNum, uint8_t state){
switch(pinNum){
case 0: if(state){HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);} break;
case 1: if(state){HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);}break;
case 2: if(state){HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_RESET);}break;
case 3: if(state){HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_RESET);}break;
case 4: if(state){HAL_GPIO_WritePin(GPIO4_GPIO_Port, GPIO4_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO4_GPIO_Port, GPIO4_Pin, GPIO_PIN_RESET);}break;
case 5: if(state){HAL_GPIO_WritePin(GPIO5_GPIO_Port, GPIO5_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO5_GPIO_Port, GPIO5_Pin, GPIO_PIN_RESET);}break;
case 6: if(state){HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_RESET);}break;
case 7: if(state){HAL_GPIO_WritePin(GPIO7_GPIO_Port, GPIO7_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO7_GPIO_Port, GPIO7_Pin, GPIO_PIN_RESET);}break;
case 8: if(state){HAL_GPIO_WritePin(GPIO8_GPIO_Port, GPIO8_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO8_GPIO_Port, GPIO8_Pin, GPIO_PIN_RESET);}break;
case 9: if(state){HAL_GPIO_WritePin(GPIO9_GPIO_Port, GPIO9_Pin, GPIO_PIN_SET);}else{HAL_GPIO_WritePin(GPIO9_GPIO_Port, GPIO9_Pin, GPIO_PIN_RESET);}break;
default: break;
}
}

void writeDACcode(uint8_t code){
writeGPIO(4, LOW);
if(code&1){writeGPIO(3, HIGH);}else{writeGPIO(3, LOW);}
if(code&2){writeGPIO(2, HIGH);}else{writeGPIO(2, LOW);}
if(code&4){writeGPIO(1, HIGH);}else{writeGPIO(1, LOW);}
if(code&8){writeGPIO(0, HIGH);}else{writeGPIO(0, LOW);}
myDelayUs(1);
writeGPIO(4, HIGH);
myDelayUs(1);
}

void adcTracking(void){
	if(HAL_GPIO_ReadPin(SD_CNT_GPIO_Port, SD_CNT_Pin)==GPIO_PIN_SET){
		adcTrackingCount--;
	}else{
		adcTrackingCount++;
	}
	if(adcTrackingCount<0){
		adcTrackingCount = 0;
	}
	if(adcTrackingCount>15){
		adcTrackingCount = 15;
	}
	writeDACcode(adcTrackingCount);
	if(uartDmaFlag == 0){
		dmaSize = 1;
		*(dmaBuff+0) = adcTrackingCount;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)dmaBuff, dmaSize);
		uartDmaFlag = 1;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim==&htim3){
	if(uartRxCt>0){
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		uartRxCt--;
	}else{
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	}
	if(uartTxCt>0){
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		uartTxCt--;
	}else{
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	}
}
if(htim==&htim4){
	if(mode==TRACKmode){
		adcTracking();
	}
}
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(1000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  MovingAverage_Init(&ma_filter, 32);

  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(100);
  for(uint8_t i=0;i<5;i++){
	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	  HAL_Delay(250);
	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	  HAL_Delay(250);
  }
  UART_Start_Receive_IT(&huart1, (uint8_t*)&rxData, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if((mode!=RAWmode)&&(mode!=OFFmode)&&(mode!=TRACKmode)&&(sendFlag==1)&&(actAdcFlag==1)){
		  if(uartDmaFlag == 0){
			  dmaSize = printNumToBuff(0, (uint8_t*)dmaBuff, actAdcVal, 1, 1);
			  *(dmaBuff+dmaSize) = ' ';
			  dmaSize++;
			  dmaSize = printNumToBuff(dmaSize, (uint8_t*)dmaBuff, outValue/(decRatio*decRatio*decRatio), 1, 1);
			  *(dmaBuff+dmaSize) = ' ';
			  dmaSize++;
			  dmaSize = printNumToBuff(dmaSize, (uint8_t*)dmaBuff, MAoutput, 1, 1);
			  *(dmaBuff+dmaSize) = '\r';
			  dmaSize++;
			  *(dmaBuff+dmaSize) = '\n';
			  dmaSize++;
			  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dmaBuff, dmaSize);
			  uartDmaFlag = 1;
		  }
		  sendFlag = 0;
		  actAdcFlag = 0;
	  }else if(mode==OFFmode){
		  if(uartDmaFlag == 0){
			  dmaSize = printNumToBuff(0, (uint8_t*)dmaBuff, test_cnt, 1, 1);
			  *(dmaBuff+dmaSize) = ' ';
			  dmaSize++;
			  dmaSize = printNumToBuff(dmaSize, (uint8_t*)dmaBuff, test_cnt, 1, 1);
			  *(dmaBuff+dmaSize) = ' ';
			  dmaSize++;
			  dmaSize = printNumToBuff(dmaSize, (uint8_t*)dmaBuff, test_cnt, 1, 1);
			  *(dmaBuff+dmaSize) = '\r';
			  dmaSize++;
			  *(dmaBuff+dmaSize) = '\n';
			  dmaSize++;
			  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dmaBuff, dmaSize);
			  uartDmaFlag = 1;
			  test_cnt++;
			  if(test_cnt>1000){
				  test_cnt=-1000;
			  }
		  }
		  HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16799;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 1500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO1_Pin|GPIO2_Pin|GPIO3_Pin|GPIO5_Pin
                          |GPIO6_Pin|GPIO7_Pin|GPIO8_Pin|GPIO9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO0_Pin|GPIO4_Pin|LED_B_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO1_Pin GPIO2_Pin GPIO3_Pin GPIO5_Pin
                           GPIO6_Pin GPIO7_Pin GPIO8_Pin GPIO9_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin|GPIO2_Pin|GPIO3_Pin|GPIO5_Pin
                          |GPIO6_Pin|GPIO7_Pin|GPIO8_Pin|GPIO9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO0_Pin GPIO4_Pin LED_B_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin|GPIO4_Pin|LED_B_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CLK_Pin */
  GPIO_InitStruct.Pin = SD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CNT_Pin SD_DOUT_Pin */
  GPIO_InitStruct.Pin = SD_CNT_Pin|SD_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
