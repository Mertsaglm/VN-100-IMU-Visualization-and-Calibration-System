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
#include "vn100.h"
#include "vn100_port_stm32.h"
#include "vn100_protocol.h"
#include "vn100_binary.h"
#include "host_link.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* TX timeout to the PC (VCP) - 30 ms: the HAL timeout is a SINGLE budget for
   the WHOLE transfer; 10 ms (~115 B ceiling) would truncate a 12-float
   $VNRRG response (~160 B, buffer 256 B, ~22 ms) EVERY TIME. The VCP applies
   no back-pressure, so blocking is bounded by wire time. Details:
   protocol.md sec 8. */
#define HOST_TX_TIMEOUT_MS 30U

/**
 * Ground-station relay: the STM32 re-publishes every new VN-100 measurement
 * as a clean $VNYMR line to the PC (USART3 / ST-Link VCP), which the
 * dashboard parses. Closes the "dashboard <-> STM32 <-> VN-100" data path
 * over the same VCP as the host command channel (command responses start
 * with "VN...", so they don't interfere with $VNYMR parsing).
 *
 * VN_RELAY_HUMAN=1 additionally prints a 4 Hz human-readable debug line
 * (ignored by the parser; terminal reading only).
 */
#define VN_RELAY_HUMAN   0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* FW-F1: FORCE-links newlib-nano's float-printf support - the IN-CODE
   equivalent of the '-u _printf_float' linker flag: creates an undefined
   global symbol, so the linker pulls the float-printf object out of libc to
   resolve it. PRESERVED even if .cproject/the makefile is regenerated (it
   lives in USER CODE). Without it, all %f output (relay $VNYMR + Reg 23/84
   calibration writes) comes out BLANK on real hardware - a SILENT failure
   (invisible in the simulator, since Python uses real floats). BELT+
   SUSPENDERS: the same '-u _printf_float' flag is also in .cproject's
   linker "Other flags" (two independent mechanisms). On first flash,
   visually confirm a known float (e.g. 9.81) doesn't print blank; CubeIDE's
   "Use float with printf from newlib-nano" checkbox can also stay
   checked. */
__asm__(".global _printf_float");

/** VN-100 portable core + STM32 port */
vn100_t       hvn;
vn100_stm32_t hvn_stm;

/** Host command channel (PC <-> STM32) */
host_link_t   hhl;

/** Relay state: packet counter of the last measurement forwarded to the PC (detects new data) */
static uint32_t last_relay_count = 0U;

#if VN_RELAY_HUMAN
/** Timing variable for the optional human-readable debug output */
static uint32_t last_print_tick = 0U;
#endif

/* Relay drop counters - make SILENT loss VISIBLE: without them, a
   missing/truncated frame (HAL_UART_Transmit failure or encoding overflow)
   could go out to the PC with no trace anywhere. Readable via debugger (or,
   later, a 'VN STATS' verb). NOTE: only meaningful on REAL HARDWARE. */
static volatile uint32_t g_host_tx_drop = 0U;      /* TX to PC failed/timed out */
static volatile uint32_t g_relay_encode_fail = 0U; /* frame encoding failed (buffer overflow) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void on_vn_packet(const vn100_data_t *d, void *user);
static void on_vn_error(vn100_status_t err, void *user);
static void vn_relay(void);
static void vn_forward_response(void);
#if VN_RELAY_HUMAN
static void vn_debug_print(void);
#endif
static int  host_reply(void *ctx, const uint8_t *data, uint16_t len);
static void bringup_detect_baud(void);
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* -- Protocol + binary self-test (no hardware required) -- */
  /* Same vectors as Python: xor("VNYMR")=0x5E, crc16("123456789")=0x31C3 */
  if (!vn100_protocol_selftest() || !vn100_binary_selftest())
  {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);  /* LD3 red - self-test failure */
      Error_Handler();
  }

  /* -- VN-100 core + STM32 port -- */
  if (vn100_stm32_init(&hvn_stm, &hvn, &huart6,
                       on_vn_packet, on_vn_error, NULL) != VN100_OK)
  {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      Error_Handler();
  }

  /* Start circular DMA RX + IDLE */
  if (vn100_stm32_start(&hvn_stm) != VN100_OK)
  {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      Error_Handler();
  }

  /* -- Host command channel (PC <-> STM32, USART3 VCP) -- */
  host_link_init(&hhl, &hvn, host_reply, NULL);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);       /* enable the RX interrupt in code */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  /* -- Bring-up: detect the sensor's actual baud rate --
     If the sensor's saved baud isn't 115200 (e.g. previously set to
     921600), try candidate rates and stay on whichever has data flowing;
     fall back to 115200 if none do. Called AFTER host_link_init so PC
     commands are still processed during the scan. */
  bringup_detect_baud();

  /* -- Deterministic startup: set the sensor to default ASCII $VNYMR @ 50 Hz --
     (demo mode; 'VN MODE BINARY' switches to 200 Hz binary in normal
     operation.) Ensures the stream starts in the format firmware expects
     after reset, instead of trusting whatever the sensor had saved (baud
     already confirmed by bringup_detect_baud). */
  (void)vn100_set_output_mode(&hvn, VN100_FMT_ASCII, 50U);

  /* Blue LED - system ready */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Process host commands (dashboard -> STM32 -> VN-100) */
    host_link_process(&hhl);

    /* -- Ground-station relay: forward new measurements to the PC as $VNYMR -- */
    vn_relay();

    /* -- Forward sensor command responses ($VNRRG/$VNWRG) to the PC -- */
    vn_forward_response();

#if VN_RELAY_HUMAN
    /* -- Optional human-readable debug output (250 ms = 4 Hz) -- */
    if ((HAL_GetTick() - last_print_tick) >= 250U)
    {
        last_print_tick = HAL_GetTick();
        vn_debug_print();
    }
#endif

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Successful-packet callback - toggles the green LED.
  * @note   LED/GPIO is the APPLICATION's responsibility, not the core's (for portability).
  */
static void on_vn_packet(const vn100_data_t *d, void *user)
{
    (void)d; (void)user;
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);   /* LD1 green - valid packet */
}

/**
  * @brief  Error callback - toggles the red LED.
  */
static void on_vn_error(vn100_status_t err, void *user)
{
    (void)err; (void)user;
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);  /* LD3 red - error */
}

/**
  * @brief  Forwards a new measurement to the PC (VCP) as a $VNYMR line, if one arrived.
  * @note   Called from the main loop (not an ISR), so the blocking
  *         HAL_UART_Transmit is safe. Always sends the LATEST data; if the
  *         loop can't keep up with the sensor rate, intermediate packets
  *         are skipped (correct for a live view). %f output requires
  *         "float with printf" (newlib-nano). Could move to DMA-TX later
  *         for higher throughput.
  */
static void vn_relay(void)
{
    vn100_data_t d;
    char     abuf[160];                 /* ASCII $VNYMR buffer */
    uint8_t  bbuf[VN100_BIN_FRAME_LEN]; /* binary frame buffer */
    int  n;
    uint32_t cnt;

    /* Measurement AND packet_count are read in a SINGLE critical section:
       reading the counter outside it could let an ISR insert a new packet
       in between, publishing the same measurement twice (a repeated frame
       in the latest-sample relay). */
    if (!vn100_get_data_counted(&hvn, &d, &cnt))
    {
        return;
    }
    if (cnt == last_relay_count)
    {
        return;                         /* no new data */
    }
    last_relay_count = cnt;

    /* Stream goes out in the selected format (hhl.out_fmt, set via VN
       MODE): BINARY -> 42B frame; ASCII -> $VNYMR line. The PC's dual-mode
       parser decodes whichever arrives; command responses always arrive as
       ASCII and the parser separates them out. */
    if (hhl.out_fmt == VN100_FMT_BINARY)
    {
        n = vn100_binary_encode(bbuf, (int)sizeof(bbuf), &d);
        if (n > 0)
        {
            if (HAL_UART_Transmit(&huart3, bbuf, (uint16_t)n,
                                  HOST_TX_TIMEOUT_MS) != HAL_OK)
            {
                g_host_tx_drop++;          /* make a truncated/dropped frame VISIBLE */
            }
        }
        else
        {
            g_relay_encode_fail++;         /* encoding overflowed -> don't drop the frame SILENTLY */
        }
    }
    else
    {
        n = vn100_encode_vnymr(abuf, (int)sizeof(abuf), &d);
        if (n > 0)
        {
            if (HAL_UART_Transmit(&huart3, (uint8_t *)abuf, (uint16_t)n,
                                  HOST_TX_TIMEOUT_MS) != HAL_OK)
            {
                g_host_tx_drop++;
            }
        }
        else
        {
            g_relay_encode_fail++;
        }
    }
}

/**
  * @brief  Forwards a command response from the sensor ($VNRRG/$VNWRG) to the PC (VCP).
  * @note   How register reads (e.g. Reg 46/47 HSI status) reach the PC: the
  *         sensor's reply arrives on USART6, the core places it in the
  *         mailbox, and this function forwards it verbatim from the main
  *         loop.
  */
static void vn_forward_response(void)
{
    char resp[VN100_RESP_MAX];
    uint16_t rlen = 0U;

    /* Drain the response QUEUE COMPLETELY each pass, so back-to-back
       $VNRRG responses (e.g. Reg 23+44 for a snapshot) all go out in the
       same pass, not one at a time. */
    while (vn100_take_response(&hvn, resp, sizeof(resp), &rlen) && (rlen > 0U))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)resp, rlen, HOST_TX_TIMEOUT_MS);
    }
}

#if VN_RELAY_HUMAN
/**
  * @brief  Prints the latest IMU data over USART3 (ST-Link VCP).
  * @note   %f output requires "Use float with printf" (newlib-nano) enabled
  *         in the project settings; otherwise numbers print blank.
  */
static void vn_debug_print(void)
{
    vn100_data_t d;
    char buf[160];
    int len;

    if (!vn100_get_data(&hvn, &d))
    {
        const char msg[] = "[VN100] Waiting for data...\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U), 100U);
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "[VN100] YPR: %7.2f %7.2f %7.2f | Gyro: %7.4f %7.4f %7.4f | "
                   "Acc: %6.2f %6.2f %6.2f | Pkt:%lu Err:%lu\r\n",
                   d.yaw, d.pitch, d.roll,
                   d.gyro_x, d.gyro_y, d.gyro_z,
                   d.accel_x, d.accel_y, d.accel_z,
                   (unsigned long)hvn.packet_count,
                   (unsigned long)hvn.error_count);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 100U);
    }
}
#endif /* VN_RELAY_HUMAN */

/**
  * @brief  Host channel reply function - writes to the PC over USART3 (VCP).
  */
static int host_reply(void *ctx, const uint8_t *data, uint16_t len)
{
    (void)ctx;
    return (HAL_UART_Transmit(&huart3, (uint8_t *)data, len, HOST_TX_TIMEOUT_MS) == HAL_OK)
               ? (int)len : -1;
}

/* -- Also drain the ring on DMA half/full-transfer --------------
   Draining can't depend on IDLE alone: under a continuous/high-Hz stream
   the ring must not be overwritten before being read, even if the line
   never goes idle. HAL calls these weak callbacks from the DMA IRQ; trigger
   vn100_stm32_drain() for USART6 (the sensor). */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        vn100_stm32_drain(&hvn_stm);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        vn100_stm32_drain(&hvn_stm);
    }
}

/* -- Reset DMA reception on a sensor-line error ----------------
   Flags are cleared in the USART6 ISR before HAL sees them, so this
   callback rarely fires; it's still a SAFETY NET that restarts reception if
   an error ever cancels the DMA. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        HAL_UART_DMAStop(&huart6);
        (void)vn100_stm32_start(&hvn_stm);          /* re-arm: last_pos=0 + restart circular DMA */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);  /* red LED: line error */
    }
}

/* -- Bring-up automatic baud detection --------------------------
   If the sensor's saved baud isn't 115200 (e.g. previously set to 921600),
   the STM32's 115200 commands come out as garbage and the link dies. Tries
   each candidate in turn, stays on whichever has valid packets flowing, and
   falls back to 115200 (the out-of-box default) if none do - a failed scan
   is at worst the previous (safe) behavior.
   NOTE: only verifiable on REAL hardware; the simulator has no USART6.
   LIMIT: the detection signal is only a valid $VNYMR/binary packet
   (packet_count). If the sensor was previously saved with a DIFFERENT
   async message type (e.g. $VNQMR) AND a non-standard baud, no packets are
   counted at that baud and it falls back to 115200 (a narrow edge case; a
   factory-fresh sensor works fine at 115200). The IDLE interrupt also stays
   enabled across the baud switch; an IDLE firing mid-transition could read
   stale NDTR off the stopped DMA, but the read stays within the buffer and
   the parser rejects the garbage, so it's harmless. */
static void bringup_detect_baud(void)
{
    static const uint32_t bauds[] = { 115200U, 921600U, 230400U, 460800U };
    uint32_t i;

    for (i = 0U; i < (sizeof(bauds) / sizeof(bauds[0])); i++)
    {
        uint32_t start_cnt = hvn.packet_count;
        uint32_t t0;

        if (bauds[i] != huart6.Init.BaudRate)
        {
            (void)vn100_stm32_stop(&hvn_stm);   /* stop DMA + disable IDLE IT (DMAStop alone leaves IDLE enabled) */
            huart6.Init.BaudRate = bauds[i];
            if (HAL_UART_Init(&huart6) != HAL_OK)
            {
                continue;
            }
            (void)vn100_stm32_start(&hvn_stm);
        }
        /* Is a valid $VNYMR/binary packet flowing at this baud for ~300 ms? (the ISR counts) */
        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 300U)
        {
            host_link_process(&hhl);                /* don't go deaf to PC commands during this */
        }
        if (hvn.packet_count != start_cnt)
        {
            return;                                 /* data present -> stay at this baud */
        }
    }
    /* No data at any baud -> fall back to 115200 (sensor may be quiet; config can enable it later) */
    (void)vn100_stm32_stop(&hvn_stm);           /* stop DMA + disable IDLE IT */
    huart6.Init.BaudRate = 115200U;
    (void)HAL_UART_Init(&huart6);
    (void)vn100_stm32_start(&hvn_stm);
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
