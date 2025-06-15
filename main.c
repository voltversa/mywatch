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
#include "cg9a01.h"

#include "font8x8_basic.h"
#include "font8x16.h"
#include "max30102_for_stm32_hal.h"
#include <stdio.h>     // for snprintf()
#include <stdbool.h>   // for bool, true, false
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
#define IR_BUFFER_SIZE 800
uint32_t ir_buffer[IR_BUFFER_SIZE];
uint32_t ir_index = 0;
float g_bpm = 0.0f;
float bpm_buffer[20] = {0};
int bpm_index = 0;
int bpm_count = 0;
uint32_t last_beat_time = 0;
#define MAX30205_ADDR       (0x48 << 1)  // 0x90 write, 0x91 read
#define MAX30205_TEMP_REG   0x00
RTC_TimeTypeDef nowTime;
RTC_DateTypeDef nowDate;
char received_char;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void calculate_bpm(uint32_t *buffer, uint32_t length);
volatile uint8_t uart_flag = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void __attribute__((naked)) SysTickDelayCount(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n     SysTickDelayCount\n"
          "    bx      lr");
}




// Override plot function
bool detectPulse(uint32_t irValue)
{
    static uint32_t prev_ir_value = 0;
    static uint8_t falling = 0;
    static uint32_t lastBeat = 0;

    bool pulseDetected = false;

    if (irValue > 50000 && irValue < 300000) // instead of 100000
    {
        if (irValue > prev_ir_value)
        {
            falling = 0;
        }
        else if (irValue < prev_ir_value)
        {
            if (!falling)
            {
                // Detected peak
                uint32_t now = HAL_GetTick();
                uint32_t beat_interval = now - lastBeat;
                lastBeat = now;

                if (beat_interval > 300 && beat_interval < 1500)
                {
                    float bpm = 60000.0f / beat_interval;
                    bpm_buffer[bpm_index++] = bpm;
                    if (bpm_index >= 10) bpm_index = 0;
                    if (bpm_count < 10) bpm_count++;

                    float bpm_sum = 0.0f;
                    for (int i = 0; i < bpm_count; ++i) bpm_sum += bpm_buffer[i];
                    g_bpm = bpm_sum / bpm_count;
                    pulseDetected = true;
                }
                falling = 1;
            }
        }
    }

    prev_ir_value = irValue;

    return pulseDetected;
}
float MAX30205_ReadTemp(void)
{
    uint8_t tempData[2];
    if (HAL_I2C_Mem_Read(&hi2c2, MAX30205_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, tempData, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return -100.0f;
    }

    int16_t raw = (tempData[0] << 8) | tempData[1];
    float tempC = raw / 256.0f;
    return tempC;

}

// MAX30102 object
max30102_t max30102;

////void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
////{
////    if (huart->Instance == USART1)  // Adjust for your UART if different
////    {
////        uart_flag = 1;
////        HAL_UART_Receive_IT(&huart1, &received_char, 1);  // Re-arm interrupt
////    }
////}
//float readBatteryVoltage(void)
//{
//    uint32_t adc_val = 0;
//    HAL_ADC_Start(&hadc1);
//    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//    {
//        adc_val = HAL_ADC_GetValue(&hadc1);
//    }
//    HAL_ADC_Stop(&hadc1);
//
//    float v_adc = (3.3f * adc_val) / 4095.0f; // 12-bit ADC
//    float v_bat = v_adc * 2.0f;              // Because of voltage divider (100k + 100k)
//
//    return v_bat;
//}
//
//uint8_t getBatteryPercentage(float voltage)
//{
//    // Adjust range depending on your battery's specs
//    float min = 3.0f; // Fully discharged
//    float max = 4.2f; // Fully charged
//
//    if (voltage >= max) return 100;
//    if (voltage <= min) return 0;
//    return (uint8_t)(((voltage - min) / (max - min)) * 100);
//}
//float readBatteryVoltageAvg(uint8_t samples)
//{
//    uint32_t sum = 0;
//
//    for (uint8_t i = 0; i < samples; i++)
//    {
//        HAL_ADC_Start(&hadc1);
//        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//        {
//            sum += HAL_ADC_GetValue(&hadc1);
//        }
//        HAL_ADC_Stop(&hadc1);
//        HAL_Delay(2);  // Slight delay between samples
//    }
//
//    float adc_avg = (float)sum / samples;
//    float v_adc = (3.3f * adc_avg) / 4095.0f;
//    float v_bat = v_adc * 2.0f;  // Voltage divider 100k + 100k
//
//    return v_bat;
//}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
 // HAL_UART_Receive_IT(&huart1, &received_char, 1); // Enable UART RX interrupt

  GC9A01_Initial();


	ClearScreen2(BLACK);
	  max30102_init(&max30102, &hi2c1);
	  max30102_reset(&max30102);
	  max30102_clear_fifo(&max30102);
	  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

	  // Sensor settings
	  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
	  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
	  max30102_set_sampling_rate(&max30102, max30102_sr_50);
	  max30102_set_led_current_1(&max30102, 6.2);
	  max30102_set_led_current_2(&max30102, 15);

	  // Enter SpO2 mode
	  max30102_set_mode(&max30102, max30102_spo2);
	  max30102_set_a_full(&max30102, 1);

	  // Initiate 1 temperature measurement
	  max30102_set_die_temp_en(&max30102, 1);
	  max30102_set_die_temp_rdy(&max30102, 1);

	  uint8_t en_reg[2] = {0};
	  max30102_read(&max30102, 0x00, en_reg, 1);
		ClearScreen2(BLACK);

	  show_picture();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    static uint32_t lastUpdate = 0;
	    uint32_t now = HAL_GetTick();

	    // Read MAX30102 data
	    max30102_read_fifo(&max30102);
	    max30102_clear_fifo(&max30102);

	    uint32_t ir = max30102._ir_samples[0];
	    uint32_t red = max30102._red_samples[0];
	    float temp = MAX30205_ReadTemp();

	    detectPulse(ir);  // Updates g_bpm if beat is detected
//	    float vbat = readBatteryVoltageAvg(10); // Smooth battery voltage over 10 samples
//	    uint8_t battery_percent = getBatteryPercentage(vbat);


	    // Periodically update the display (every 500ms)
	    if (now - lastUpdate >= 500)
	    {
	        lastUpdate = now;

	        // Read RTC
	        HAL_RTC_GetTime(&hrtc, &nowTime, RTC_FORMAT_BIN);
	        HAL_RTC_GetDate(&hrtc, &nowDate, RTC_FORMAT_BIN); // Must follow GetTime

	        // Format strings
	        char bpm_str[32], tempStr[32], ir_str[32], red_str[32], timeStr[32], dateStr[32];
	        char battStr[32];

	        snprintf(bpm_str, sizeof(bpm_str), "BPM: %.1f", g_bpm);
	        snprintf(tempStr, sizeof(tempStr), "Temp: %.2f C", temp);  // Remove +10 if not needed
	        snprintf(timeStr, sizeof(timeStr), "TIME: %02d:%02d:%02d", nowTime.Hours, nowTime.Minutes, nowTime.Seconds);
	        snprintf(dateStr, sizeof(dateStr), "DATE: %02d-%02d-20%02d", nowDate.Date, nowDate.Month, nowDate.Year);
//	        snprintf(battStr, sizeof(battStr), "BAT: %d%%", battery_percent);

	        // Display values
	        GC9A01_Draw_String(30, 60, bpm_str);
	        GC9A01_Draw_String(30, 80, tempStr);
	        GC9A01_Draw_String(30, 100, timeStr);
	        GC9A01_Draw_String(30, 120, dateStr);
//	        GC9A01_Draw_String(30, 160, battStr);


	    }


//	    if (uart_flag)
//	    {
//	        uart_flag = 0;
//
//	        if (received_char == '1')
//	        {
//	            GC9A01_Draw_String(30, 140, "Connected     ");
//	        }
//	        else if (received_char == '0')
//	        {
//	            GC9A01_Draw_String(30, 140, "Disconnected  ");
//	        }
//	    }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  sTime.Hours = 0x4;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_JUNE;
  DateToUpdate.Date = 0x15;
  DateToUpdate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin motor_Pin */
  GPIO_InitStruct.Pin = LED_Pin|motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
