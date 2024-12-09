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
#include "chalie_leds.h"
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
SMBUS_HandleTypeDef hsmbus1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM2_Init(void);

static void MX_I2C1_SMBUS_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_TIM2_Init();
    MX_I2C1_SMBUS_Init();
    /* USER CODE BEGIN 2 */
    chalie_leds_init();
    HAL_SMBUS_EnableListen_IT(&hsmbus1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    int code = 0;
    uint8_t soc = 0;
    uint8_t address = 0x17;
    uint8_t soc_address = 0x0d;
    uint8_t read = 0;
    uint8_t current_address = 0x0a;
    int16_t current = 0;
    uint8_t last_timer_it_state = 0;
    uint8_t DSG_address = 0x0020;
    uint8_t CHG_address = 0x001F;
    uint8_t PCHG_address = 0x001E;
    uint8_t button_state_machine = 0;
    enum {
        BUTTON_STATE_IDLE = 0,
        BUTTON_STATE_SHORT_PRESS = 1,
        BUTTON_STATE_L
    };

    // 启动DSG，CHG，PCHG
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // SMBUS 读取0x0D寄存器 State of Charge，格式WORD
        if (HAL_GetTick() % 1000 > 500 && !read) {
            // HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t *) &soc_address, 1, 100);
            // HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t *) &soc, 1, 100);
            // HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t *) &current_address, 1, 100);
            // HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t *) &current, 2, 100);

            read = 1;
            // 将SOC0～100转换到0-7的范围
            code = soc / 12.5 + 1;
        } else if (HAL_GetTick() % 1000 < 500) {
            read = 0;
        }

        // 如果PA0被按下，LED显示SOC
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
                for (int i = 1; i < code; i += 2) {
                    chalie_led_code(i);
                    HAL_Delay(100);
                }
                chalie_led_code(code);
            }

            HAL_Delay(3000);
            HAL_TIM_Base_Stop_IT(&htim2);
            last_timer_it_state = 0;
            chalie_led_timer_update(1);
            chalie_leds_set(1, GPIO_PIN_RESET);
        } else if (current > 50 || current < -100) {
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
                for (int i = 1; i < code; i += 2) {
                    chalie_led_code(i);
                    HAL_Delay(100);
                }
                chalie_led_code(code);
            }
            chalie_led_code(code);
            HAL_Delay(10);
        } else {
            if (last_timer_it_state) {
                HAL_TIM_Base_Stop_IT(&htim2);
                last_timer_it_state = 0;
                chalie_led_timer_update(1);
                chalie_leds_set(1, GPIO_PIN_RESET);
            }

            HAL_Delay(10);
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_SMBUS_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */
    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hsmbus1.Instance = I2C1;
    hsmbus1.Init.Timing = 0x00000000;
    hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
    hsmbus1.Init.OwnAddress1 = 24;
    hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
    hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
    hsmbus1.Init.OwnAddress2 = 0;
    hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
    hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
    hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
    hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
    hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
    hsmbus1.Init.SMBusTimeout = 0x0000800C;
    if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2620;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2)
        chalie_led_timer_update(0);
}

void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode) {

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
