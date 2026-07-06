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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wit_c_sdk.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>

/* printf 重定向到 USART1，newlib-nano 要求实现 __io_putchar */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

/* ---- JY901P 角度读取 ---- */

/* 收到新角度数据时 SDK 回调置 1，主循环检测后清零 */
volatile uint8_t angle_data_ready = 0;

/* SDK 需要发数据给传感器时调用，通过 USART2 发出 */
static void SensorUartSend(uint8_t *p_data, uint32_t len)
{
    HAL_UART_Transmit(&huart2, p_data, len, 10);
}

/* SDK 需要延时等待时调用 */
static void DelayMs(uint16_t ms)
{
    HAL_Delay(ms);
}

/* SDK 每收到一个完整数据包时回调，通知我们更新了哪些寄存器 */
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    /* 只有收到角度寄存器 (Roll=0x3d) 时才标记有效 */
    if (uiReg == Roll) angle_data_ready = 1;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---- JY901P SDK 初始化 ---- */
  /* 1. 选择正常协议 (WIT_PROTOCOL_NORMAL)，传感器地址 0x50 */
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  /* 2. 注册发送函数：SDK 需要写传感器时，调 SensorUartSend → USART2 */
  WitSerialWriteRegister(SensorUartSend);
  /* 3. 注册数据回调：SDK 收到完整包时，调 SensorDataUpdate 通知我们 */
  WitRegisterCallBack(SensorDataUpdate);
  /* 4. 注册延时函数：SDK 写寄存器后需要等待，用 HAL_Delay */
  WitDelayMsRegister(DelayMs);
  printf("JY901P init done\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 将 SDK 最新收到的原始数据包写入 sReg[] 寄存器表 */
    CopeWitData(ucRegIndex, usRegDataBuff, uiRegDataLen);

    /* 角度数据就绪：从 sReg[] 取出 Roll/Pitch/Yaw，整数运算避免浮点 */
    if (angle_data_ready)
    {
        angle_data_ready = 0;

        /* raw × 1800 / 32768 = 角度 × 10（例：raw=-334 → -18 → -1.8°） */
        int32_t r10 = (int32_t)sReg[Roll]  * 1800 / 32768;
        int32_t p10 = (int32_t)sReg[Pitch] * 1800 / 32768;
        int32_t y10 = (int32_t)sReg[Yaw]   * 1800 / 32768;

        /* 处理负数：拆成符号 + 绝对值，方便 %d.%d 打印 */
        int rs = r10 < 0, ra = rs ? -r10 : r10;   /* rs=1为负 */
        int ps = p10 < 0, pa = ps ? -p10 : p10;
        int ys = y10 < 0, ya = ys ? -y10 : y10;

        printf("Roll=%c%d.%d Pitch=%c%d.%d Yaw=%c%d.%d\r\n",
               rs ? '-' : ' ', ra / 10, ra % 10,   /* 例: -1.8 */
               ps ? '-' : ' ', pa / 10, pa % 10,
               ys ? '-' : ' ', ya / 10, ya % 10);
    }

    HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
