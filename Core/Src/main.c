/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include "string.h"
#include <stdlib.h>

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BMP180_ADDRESS 0xEE // Adres 0x77 przesunięty w lewo o 1
#define MPU6050_ADDR 0xD0 // Adres I2C MPU6050

int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Gyro_X, Gyro_Y, Gyro_Z;

// Zmienne dla Akcelerometru
int16_t Acc_X_RAW, Acc_Y_RAW, Acc_Z_RAW;
float Acc_X, Acc_Y, Acc_Z;

// Zmienne kątów (Fuzja)
float Roll = 0.0f;
float Pitch = 0.0f;
float Yaw = 0.0f;

// Czas dla filtra
uint32_t lastTime = 0;

// Bufor UART (zwiększony dla bezpieczeństwa)
char uartBuf[200];

// Zmienne kalibracyjne (zgodnie z notą katalogową BMP180)
short AC1, AC2, AC3;
unsigned short AC4, AC5, AC6;
short B1, B2;
short MB, MC, MD;

// Zmienne do obliczeń
long B5; 
short OSS = 0; // Oversampling setting (0: ultra low power ... 3: ultra high res)

// Wyniki
float Temperature = 0.0f;
float Pressure = 0.0f;
float Altitude = 0.0f;

// Bufor UART
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Read_Accel(void);
void BMP180_ReadCalibration(void);
uint16_t BMP180_ReadUT(void);
uint32_t BMP180_ReadUP(void);
void BMP180_Calculate(void);
void MPU6050_Init(void);
void MPU6050_Read_Gyro(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Inicjalizacja BMP180...\r\n");
  BMP180_ReadCalibration();
  printf("Inicjalizacja MPU6050...\r\n");
  MPU6050_Init();
  lastTime = HAL_GetTick();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
    // 1. Pobranie danych
    BMP180_Calculate(); 
    MPU6050_Read_Gyro();
    MPU6050_Read_Accel();

    // 2. Czas dla filtra
    uint32_t currentTime = HAL_GetTick();
    float dt = (float)(currentTime - lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    lastTime = currentTime;

    // 3. Obliczenia fuzji (zmienne float są OK do obliczeń, ale nie do sprintf)
    float accRoll = atan2f(Acc_Y, Acc_Z) * 57.2957f;
    float accPitch = atan2f(-Acc_X, sqrtf(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * 57.2957f;

    Roll = 0.98f * (Roll + Gyro_X * dt) + 0.02f * accRoll;
    Pitch = 0.98f * (Pitch + Gyro_Y * dt) + 0.02f * accPitch;
    Yaw += Gyro_Z * dt;

    // --- 4. KONWERSJA FLOAT NA INTEGER (Ręczne przygotowanie części dziesiętnych) ---
    
    // Temperatura (1 miejsce po przecinku)
    int temp_i = (int)Temperature;
    int temp_f = abs((int)((Temperature - temp_i) * 10));

    // Wysokość (1 miejsce po przecinku)
    int alt_i = (int)Altitude;
    int alt_f = abs((int)((Altitude - alt_i) * 10));

    // Kąty (Roll, Pitch, Yaw - 1 miejsce po przecinku)
    int r_i = (int)Roll;
    int r_f = abs((int)((Roll - r_i) * 10));
    
    int p_i = (int)Pitch;
    int p_f = abs((int)((Pitch - p_i) * 10));
    
    int y_i = (int)Yaw;
    int y_f = abs((int)((Yaw - y_i) * 10));

    // --- 5. WYSYŁKA TYLKO JAKO INTEGER ---
    // Używamy %d dla części całkowitej i %d dla ułamka
    //memset(uartBuf, 0, sizeof(uartBuf));
    int len = sprintf(uartBuf, "[BARO] T:%d.%dC P:%dPa A:%d.%dm | [ATT] R:%d.%d P:%d.%d Y:%d.%d\r\n",
                      temp_i, temp_f, (int)Pressure, alt_i, alt_f,
                      r_i, r_f, p_i, p_f, y_i, y_f);

    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 100);

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(50); // Zwiększyłem opóźnienie, aby terminal nadążył z wyświetlaniem
  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU6050_Read_Accel(void)
{
    uint8_t Rec_Data[6];
    // Odczyt 6 rejestrów akcelerometru od 0x3B
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

    Acc_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Acc_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Acc_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Konwersja na jednostki 'g' (dla zakresu +/- 2g dzielnik to 16384)
    Acc_X = Acc_X_RAW / 16384.0f;
    Acc_Y = Acc_Y_RAW / 16384.0f;
    Acc_Z = Acc_Z_RAW / 16384.0f;
}
// Odczyt danych kalibracyjnych z EEPROM czujnika
void BMP180_ReadCalibration(void)
{
    uint8_t Callib_Data[22];
    HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xAA, 1, Callib_Data, 22, 1000);

    AC1 = ((short)Callib_Data[0] << 8) | Callib_Data[1];
    AC2 = ((short)Callib_Data[2] << 8) | Callib_Data[3];
    AC3 = ((short)Callib_Data[4] << 8) | Callib_Data[5];
    AC4 = ((unsigned short)Callib_Data[6] << 8) | Callib_Data[7];
    AC5 = ((unsigned short)Callib_Data[8] << 8) | Callib_Data[9];
    AC6 = ((unsigned short)Callib_Data[10] << 8) | Callib_Data[11];
    B1  = ((short)Callib_Data[12] << 8) | Callib_Data[13];
    B2  = ((short)Callib_Data[14] << 8) | Callib_Data[15];
    MB  = ((short)Callib_Data[16] << 8) | Callib_Data[17];
    MC  = ((short)Callib_Data[18] << 8) | Callib_Data[19];
    MD  = ((short)Callib_Data[20] << 8) | Callib_Data[21];
}

// Odczyt surowej temperatury (Uncompensated Temperature)
uint16_t BMP180_ReadUT(void)
{
    uint8_t rawData[2];
    uint8_t ctrl = 0x2E;
    
    // Zapisz 0x2E do rejestru 0xF4
    HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, 0xF4, 1, &ctrl, 1, 1000);
    
    // Czekaj min 4.5ms
    HAL_Delay(5);
    
    // Odczytaj 0xF6 (MSB) i 0xF7 (LSB)
    HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xF6, 1, rawData, 2, 1000);
    
    return ((rawData[0] << 8) | rawData[1]);
}

// Odczyt surowego ciśnienia (Uncompensated Pressure)
uint32_t BMP180_ReadUP(void)
{
    uint8_t rawData[3];
    uint8_t ctrl = 0x34 + (OSS << 6);
    
    // Zapisz komendę do 0xF4
    HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, 0xF4, 1, &ctrl, 1, 1000);
    
    // Czekaj zależnie od OSS (dla OSS=0: 4.5ms, OSS=3: 25.5ms)
    switch (OSS) {
        case 0: HAL_Delay(5); break;
        case 1: HAL_Delay(8); break;
        case 2: HAL_Delay(14); break;
        case 3: HAL_Delay(26); break;
    }
    
    // Odczytaj 3 bajty od 0xF6 (MSB, LSB, XLSB)
    HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xF6, 1, rawData, 3, 1000);
    
    return (((rawData[0] << 16) | (rawData[1] << 8) | rawData[2]) >> (8 - OSS));
}

// Główna funkcja obliczeniowa (Algorytm Bosch)
void BMP180_Calculate(void)
{
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    
    // 1. Oblicz Temperaturę
    uint16_t ut = BMP180_ReadUT();
    
    x1 = (ut - (long)AC6) * (long)AC5 >> 15;
    x2 = ((long)MC << 11) / (x1 + (long)MD);
    B5 = x1 + x2;
    
    Temperature = (B5 + 8) >> 4;
    Temperature = Temperature / 10.0f; // Wynik w stopniach Celsjusza
    
    // 2. Oblicz Ciśnienie
    uint32_t up = BMP180_ReadUP();
    
    b6 = B5 - 4000;
    x1 = (B2 * (b6 * b6 >> 12)) >> 11;
    x2 = AC2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((long)AC1 * 4 + x3) << OSS) + 2;
    b3 = (b3 + 2) >> 2; // BUGFIX dla int math
    
    x1 = AC3 * b6 >> 13;
    x2 = (B1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (AC4 * (unsigned long)(x3 + 32768)) >> 15;
    b7 = ((unsigned long)up - b3) * (50000 >> OSS);
    
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    
    p = p + ((x1 + x2 + 3791) >> 4);
    
    Pressure = (float)p; // Wynik w Pa
    
    // 3. Oblicz Wysokość (Wzór barometryczny)
    Altitude = 44330.0f * (1.0f - powf(Pressure / 101325.0f, 1.0f / 5.255f));
}

void MPU6050_Init(void)
{
    uint8_t check, Data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);

    if (check == 0x68) 
    {
        Data = 0; // Wybudzenie
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);

        Data = 0x00; // Accel config +/- 2g
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);

        Data = 0x00; // Gyro config 250 dps
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);
    }
}

void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[6];

    // Odczytaj 6 bajtów zaczynając od rejestru 0x43 (GYRO_XOUT_H)
    // Kolejność w pamięci: X_H, X_L, Y_H, Y_L, Z_H, Z_L
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

    // Łączenie bajtów w liczby 16-bitowe ze znakiem
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Konwersja na stopnie na sekundę (dps)
    // Dla domyślnego zakresu +/- 250dps, dzielnik wynosi 131.0
    Gyro_X = Gyro_X_RAW / 131.0f;
    Gyro_Y = Gyro_Y_RAW / 131.0f;
    Gyro_Z = Gyro_Z_RAW / 131.0f;
}

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
#ifdef USE_FULL_ASSERT
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
