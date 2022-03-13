/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
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

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
uint8_t hrs;
uint8_t min;
uint8_t sec;
uint8_t nowHr;
uint8_t nowMin;
uint8_t nowSec;
uint8_t alarm =0;
uint16_t i=0;
uint32_t adc_val;
uint8_t gcount = 0;


  float adcval;
	float temperature;
	double Balance_Res = 9710;
	double Max_ADC = 4096;
	double Beta=3950;
	double Room_Temp=298.15;
	double Res_Room_Temp=10000;
	double rThermistor=0;
	uint32_t temp;
	double tKelvin;
	double log_value;
	
//char *data[16];
__IO uint16_t Rx_Data[1];
__IO uint16_t Data[1];
__IO uint16_t Time[1];


uint8_t string[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void StandbyMode(void) {
    RTC_AlarmTypeDef sAlarm;
 /* Disable Wake-up timer */
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    
    /* Disable RTC Alarm */
   // HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

    /*## Clear all related wakeup flags ######################################*/
    /* Clear PWR wake up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
    /* Clear the Alarm Flag */
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

    /*## Re-enable all used wakeup sources ###################################*/
    /* Set RTC alarm */
    if(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) 
    {
      /* Initialization Error */
      Error_Handler();
    }

    /* Enable WKUP pin */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    

    /*## Enter Standby Mode ##################################################*/
    HAL_PWR_EnterSTANDBYMode();
  }


void get_Time(void)
	{
	 RTC_TimeTypeDef myTime;
	 HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR3 ,0x32F2);
	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR4,myTime.Hours);
	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR5,myTime.Minutes);
	 HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR6,myTime.Seconds);
		hrs = myTime.Hours;
	  min = myTime.Minutes;
	  sec = myTime.Seconds;
		
	}
void setAlarm(void)  //set alarm to make the controller to go into low power mode
{
	get_Time();
	RTC_AlarmTypeDef sAlarm;
	sAlarm.AlarmTime.Hours = hrs;
	sAlarm.AlarmTime.Minutes = min;
	sAlarm.AlarmTime.Seconds = sec+10;
	sAlarm.Alarm = RTC_ALARM_A;

	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	  {
		 Error_Handler();
	   // _Error_Handler(__FILE__, __LINE__);
	  }
}

void setWakeAlarm(void)   //set alarm to wake up the controller from low power mode
{
	RTC_AlarmTypeDef sAlarm;
//	RTC_TimeTypeDef sTime;
  get_Time();
	sAlarm.AlarmTime.Hours = hrs;
	sAlarm.AlarmTime.Minutes = min;
	sAlarm.AlarmTime.Seconds = sec+10;
	sAlarm.Alarm = RTC_ALARM_A;
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	  {
		 Error_Handler();
	  }
			   HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

}

void set_time(void)   //Reset the Time to 00:00:00 for timer functionality
{
	RTC_TimeTypeDef sTime;

	sTime.Hours = 0x1;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
		  Error_Handler();
	  }
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, 0x32F2);  // backup register
	}






uint32_t GetPage(uint32_t Address)
{
  for (int indx=0; indx<128; indx++)
  {
	  if((Address < (0x08000000 + (1024 *(indx+1))) ) && (Address >= (0x08000000 + 1024*indx)))
	  {
		  return (0x08000000 + 1024*indx);
	  }
  }

  return -1;
}
																																							 
uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint16_t*  DATA_32)
{

	//uint32_t SECTORError;
	int sofar=0;

	int numberofwords = (strlen((char*)DATA_32)/4) + ((strlen((char*)DATA_32) % 4) != 0);

	 uint32_t StartPage = GetPage(StartSectorAddress);
	 uint32_t EndPageAdress = StartSectorAddress + numberofwords*4;
	 uint32_t EndPage = GetPage(EndPageAdress);

	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	 

	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	     you have to make sure that these data are rewritten before they are accessed during code
	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	     DCRST and ICRST bits in the FLASH_CR register. */
//	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
//	  {
//		  return HAL_FLASH_GetError ();
//	  }

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, StartSectorAddress, DATA_32[sofar]) == HAL_OK)
	     {
	    	 StartSectorAddress += 2;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}


void Convert_To_Str (uint16_t *data, char *str)
{
	int numberofbytes = ((strlen((char*)data)/4) + ((strlen((char*)data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		str[i] = data[i/4]>>(8*(i%4));
	}
}


void Flash_Read_Data (uint32_t StartPageAddress, __IO uint16_t * DATA_32)
{
	while (1)
	{
		*DATA_32 = *(__IO uint32_t *)StartPageAddress;
		if (*DATA_32 == 0xffff)
		{
			*DATA_32 = '\0';
			break;
		}
		StartPageAddress += 4;
		DATA_32++;
	}
}
void to_do_wakeup(void)
{
	 	 
  setWakeAlarm(); //Timer(alarm) to wake up from low power mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_SET);
	HAL_Delay(300);
  HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	adcval=HAL_ADC_GetValue(&hadc1);
	rThermistor=Balance_Res*((Max_ADC/adcval)-1);
	tKelvin=(Beta*Room_Temp)/(Beta + (Room_Temp*log(rThermistor/Res_Room_Temp)));
	temp=tKelvin-273.15;
	Data[0] =temp;
	Time[0]=0x36;
	Flash_Write_Data(0x0801FBF8+i, (uint16_t*)Data);
 // Flash_Read_Data(0x0801FBF8+i, Rx_Data);
	//Convert_To_Str((uint16_t*)Rx_Data, (char*)string);
  i=i+2;
	Flash_Write_Data(0x0801FBF8+i, (uint16_t*)Data);
	i+=2;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,i);
	  
  HAL_PWR_EnterSTANDBYMode();

 	
}
void to_do_on_alarm (void)
{
	//setTime();  //Reset the timer (time)
 	setWakeAlarm(); //Timer(alarm) to wake up from low power mode

		HAL_Delay(100);
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adcval=HAL_ADC_GetValue(&hadc1);
		rThermistor=Balance_Res*((Max_ADC/adcval)-1);
		tKelvin=(Beta*Room_Temp)/(Beta + (Room_Temp*log(rThermistor/Res_Room_Temp)));
		temp=tKelvin-273.15;
		Rx_Data[0] =temp;
 
	Flash_Write_Data(0x0801FBF8+i, (uint16_t*)Rx_Data);
 // Flash_Read_Data(0x0801FBF8+i, Rx_Data);
	//Convert_To_Str((uint16_t*)Rx_Data, (char*)string);
  i=i+2;
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,i);    
  HAL_PWR_EnterSTANDBYMode();

 	
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)   //This routine is called when alarm interrupt occurs
{
	alarm = 1; //make alarm variable high to indicate alarm interrupt occurs
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
	  SystemClock_Config ();
	  HAL_ResumeTick();
	  gcount++;
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


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  RTC_TimeTypeDef sTime;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	// rtc_reg =0
	i=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
	setAlarm();
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3) != 0x32F2)
      {
      //   Set the time
        set_time();
      }
     
    

    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		get_Time();
    /* USER CODE END WHILE */
		if(alarm)
	 {
		to_do_on_alarm();
		alarm=0;
	 }
      else if(HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_2)== SET)
	    {
				to_do_wakeup();
	    }		     
	      
		 	 

    /* USER CODE BEGIN 3 */
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  DateToUpdate.Month = RTC_MONTH_SEPTEMBER;
  DateToUpdate.Date = 3;
  DateToUpdate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
