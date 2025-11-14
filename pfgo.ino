/*
 * -----------------------------------------------------------------------------
 * --- STM32 3-Channel Function Generator & 3-Channel Oscilloscope ---
 * -----------------------------------------------------------------------------
 *
 * This sketch generates three simultaneous waveforms on PA4, PA5, and PA6.
 * It also uses pins A0, A1, and A2 as three "oscilloscope" inputs.
 *
 * --- HOW TO USE ---
 * 1. Upload this code.
 * 2. Connect jumper wires from your outputs to your inputs:
 * - PA4 (SINE)   -> A0 (SCOPE_1)
 * - PA5 (SQUARE) -> A1 (SCOPE_2)
 * - PA6 (TRI)    -> A2 (SCOPE_3)
 * 3. Open the Arduino Serial Plotter (Tools > Serial Plotter).
 * 4. Set the baud rate to 115200.
 *
 * You will see 6 graphs: 3 internal generator values and the 3
 * corresponding signals being read by the ADCs.
 */

// Includes are needed for HAL types and string formatting
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h> 

/* USER CODE BEGIN Includes */
// (Original code had no user includes here)
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/
// --- ALL HAL Handles must be global ---
DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac1_ch1;
UART_HandleTypeDef hlpuart1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim6_up;

/* USER CODE BEGIN PV */
// 12-bit (0-4095) sine wave lookup table with 64 samples (for smoother output)
#define SINE_SAMPLES 64
const uint32_t sine_wave[SINE_SAMPLES] = {
    2047, 2248, 2447, 2642, 2831, 3013, 3185, 3346,
    3495, 3630, 3750, 3853, 3939, 4007, 4056, 4085,
    4095, 4085, 4056, 4007, 3939, 3853, 3750, 3630,
    3495, 3346, 3185, 3013, 2831, 2642, 2447, 2248,
    2047, 1846, 1647, 1452, 1263, 1081, 909, 748,
    599, 464, 344, 241, 155, 87, 38, 9,
    0, 9, 38, 87, 155, 241, 344, 464,
    599, 748, 909, 1081, 1263, 1452, 1647, 1846
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// --- ALL helper functions from main.c must be here ---
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC2_Init(void);
static void MX_TIM7_Init(void);
static void MX_LPUART1_UART_Init(void);
void App_Error_Handler(void); // <-- Renamed function prototype

// ============================================================================
// --- ARDUINO SETUP FUNCTION ---
// (Replaces the main() function)
// ============================================================================
void setup()
{
  /*
   * NOTE: The Arduino STM32 Core automatically calls HAL_Init()
   * and SystemClock_Config() before setup() runs.
   * We do not need to call them again.
   */

  /* MPU Configuration--------------------------------------------------------*/
  // This was in your original main(), so we call it here.
  MPU_Config();

  /* Initialize all configured peripherals */
  // These are the exact functions from your original project.
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_DAC2_Init();
  MX_TIM7_Init();
  Serial.begin(115200); // Start the USB serial port

  analogReadResolution(12); // Set ADC to 12-bit (0-4095) to match DAC

  /*
   * We are NOT calling any BSP_... functions (BSP_LED_Init,
   * BSP_COM_Init, etc.) as these are part of a Board Support Package
   * and not standard Arduino/HAL code.
   */

  /* USER CODE BEGIN 2 */

  // --- Start 0.1 Hz Sine Wave on PA4 ---
  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sine_wave, SINE_SAMPLES, DAC_ALIGN_12B_R);

  // --- Start 0.1 Hz Square Wave on PA5 ---
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // --- Start 0.1 Hz Triangle Wave on PA6 (DAC2_OUT1) ---
  HAL_TIM_Base_Start(&htim7);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
  // The triangle wave generation is started inside MX_DAC2_Init()
  
  /* USER CODE END 2 */
}


// ============================================================================
// --- ARDUINO LOOP FUNCTION ---
// (Replaces the while(1) loop)
// ============================================================================
// ============================================================================
// --- ARDUINO LOOP FUNCTION ---
// (Replaces the while(1) loop)
// ============================================================================
void loop()
{
  /*
   * This loop reads the *actual* hardware output values from the
   * peripherals and sends them to the serial plotter.
   */

  // Define our conversion constants
  const float V_REF = 3.3;      // STM32 ADC/DAC reference voltage is 3.3V
  const float MAX_RAW = 4095.0; // Max 12-bit value

  // === ADDED THIS CHECK: Only run if Serial Monitor/Plotter is open ===
  if (Serial)
  {
    // 1. Read the current RAW values (0-4095)
    uint32_t sine_raw = HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1);
    uint32_t square_raw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) ? 4095 : 0;
    uint32_t triangle_raw = HAL_DAC_GetValue(&hdac2, DAC_CHANNEL_1);
    uint32_t scope_raw_1 = analogRead(A0);
    uint32_t scope_raw_2 = analogRead(A1);
    uint32_t scope_raw_3 = analogRead(A2);

    // 2. --- NEW: Convert all raw values to float voltages ---
    float sine_volt = (sine_raw / MAX_RAW) * V_REF;
    float square_volt = (square_raw / MAX_RAW) * V_REF;
    float triangle_volt = (triangle_raw / MAX_RAW) * V_REF;
    float scope_volt_1 = (scope_raw_1 / MAX_RAW) * V_REF;
    float scope_volt_2 = (scope_raw_2 / MAX_RAW) * V_REF;
    float scope_volt_3 = (scope_raw_3 / MAX_RAW) * V_REF;

    // 3. --- UPDATED: Format and send voltage data ---
    Serial.print("SINE_V:");
    Serial.print(sine_volt);
    Serial.print(",SQUARE_V:");
    Serial.print(square_volt);
    Serial.print(",TRI_V:");
    Serial.print(triangle_volt);
    Serial.print(",SCOPE_1_V:");
    Serial.print(scope_volt_1);
    Serial.print(",SCOPE_2_V:");
    Serial.print(scope_volt_2);
    Serial.print(",SCOPE_3_V:");
    Serial.print(scope_volt_3);

    // 4. --- NEW: Add dummy traces to lock the Y-axis from 0 to 4 Volts ---
    Serial.print(",Y_MIN:0.0");
    Serial.print(",Y_MAX:4.0");

    Serial.println(); // Send the newline to complete the packet
    
    // 5. Wait for a short period
    delay(100);

  } // === END OF "if (Serial)" BLOCK ===
}


/*
 * ============================================================================
 * --- ALL OTHER FUNCTIONS FROM main.c ---
 * (Pasted here unchanged)
 * ============================================================================
 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */

  // VVV THIS IS THE CORRECTED LINE VVV
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
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
    App_Error_Handler(); // <-- Renamed
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }

  /** Configure Triangle wave generation on DAC OUT1
  */
  // === FIXED AMPLITUDE TO 12-BIT (4095) ===
  if (HAL_DACEx_TriangleWaveGenerate(&hdac2, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_4095) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  // === FIXED FOR 0.1 HZ SQUARE WAVE ===
  htim2.Init.Period = 4999;

  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;

  // === FIXED FOR 0.1 HZ SINE WAVE ===
  htim6.Init.Prescaler = 65535;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 151;

  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  // This is correct for 0.1 Hz triangle wave
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 780;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for DAC1
  */
/**
  * Enable DMA controller clock
  * Configure DMA for DAC1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* --- ADD THIS CODE --- */
  /* Configure the DMA stream for DAC1_CH1 */
  hdma_dac1_ch1.Instance = DMA1_Stream0; // Or whatever stream is mapped to DAC1_CH1 on your chip
  hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CH1;
  hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // For 12-bit (uint32_t)
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dac1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    App_Error_Handler(); // <-- Renamed
  }

  /* Link the DMA handle to the DAC handle */
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);
  /* --- END OF ADDED CODE --- */


  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  
  /* NOTE: You may not need Stream1, but it was in your original code */
  /* DMA1_Stream1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* --- ADD THIS CODE --- */
  /* Configure GPIO pin : PA5 (for TIM2_CH1) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // Set alternate function to TIM2
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* --- END OF ADDED CODE --- */


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // The pins PA4 and PA6 (DAC) are configured as
  // ANALOG by the HAL_DAC_MspInit, which the
  // Arduino core *does* handle correctly.
  
  // Pins A0, A1, A2 will be configured as analog inputs
  // automatically by the first call to analogRead().
  /* USER CODE END MX_GPIO_Init_2 */
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
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE; // <-- CORRECTED
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
// === RENAMED ERROR HANDLER FUNCTION ===
void App_Error_Handler(void) // <-- Renamed
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq(); // Stop all interrupts

  // Try to send a final error message to the Serial Monitor
  Serial.println("\n*** A HAL ERROR HAS OCCURRED ***");
  Serial.println("System has halted. Please check peripheral configuration.");
  Serial.flush(); // Try to force the message out

  // Fast-blink the LED on PF11 (which was initialized in MX_GPIO_Init)
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_11);
    HAL_Delay(100); // Blink 5 times a second
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r.n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */