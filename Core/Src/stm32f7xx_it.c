/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vn100_port_stm32.h"
#include "host_link.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* LINE ERROR COUNTERS: the flags (ORE/FE/NE/PE) are deliberately cleared in
   USART6_IRQHandler (rationale there); without counting them, a lost byte
   would vanish without a trace, since `HAL_UART_ErrorCallback` is
   structurally unreachable for UART errors in this design. Read via
   debugger for diagnosis (or, later, a 'VN STATS' verb). Only meaningful
   with real values on actual hardware. */
volatile uint32_t g_usart6_err_ore = 0U;   /* sensor line: overrun (byte LOSS)      */
volatile uint32_t g_usart6_err_fne = 0U;   /* sensor line: framing/noise/parity     */
volatile uint32_t g_usart3_err_ore = 0U;   /* host line:   overrun (lost command byte) */
volatile uint32_t g_usart3_err_fne = 0U;   /* host line:   framing/noise/parity     */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
extern vn100_stm32_t hvn_stm;
extern UART_HandleTypeDef huart3;
extern host_link_t hhl;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
  /* VN-100 IDLE-line bridge + LINE ERROR RECOVERY: clear the error flags
     (ORE/FE/NE/PE) BEFORE HAL_UART_IRQHandler. Otherwise a SINGLE
     framing/noise/overrun error in DMA-RX mode makes HAL cancel the DMA
     PERMANENTLY - on a noisy real line the system silently goes deaf. The
     circular DMA + IDLE design doesn't use HAL's RX-error path; flags are
     cleared here and reception continues. The IDLE flag is also handled
     BEFORE HAL so it stays set. */
  /* COUNT BEFORE clearing: which error class occurred how many times is
     the only diagnostic trace available. ORE = actual byte LOSS (F7 has no
     RX FIFO); FE/NE/PE = line/noise issue. */
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE))
  {
    g_usart6_err_ore++;
  }
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE))
  {
    g_usart6_err_fne++;
  }
  __HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
  vn100_stm32_on_uart_idle(&hvn_stm);
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief USART3 (ST-Link VCP) RX interrupt - collects host command bytes.
  * @note  Added by hand because the USART3 interrupt is NOT ENABLED in
  *        CubeMX; the NVIC is enabled in code (main.c USER CODE 2). No
  *        blocking - only accumulates bytes; processing happens in the
  *        main loop (host_link_process).
  */
void USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
    {
        g_usart3_err_ore++;                     /* DROPPED host command byte */
        __HAL_UART_CLEAR_OREFLAG(&huart3);      /* clear the overrun */
    }
    /* Also clear FE/NE/PE: unlike ORE these don't stop RXNE (lower risk),
       but are cleared consistently so the flag doesn't stick on a noisy VCP. */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE))
    {
        g_usart3_err_fne++;                     /* line error counter */
        __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    }
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        uint8_t b = (uint8_t)(huart3.Instance->RDR & 0xFFU);
        host_link_feed(&hhl, &b, 1U);
    }
}

/* USER CODE END 1 */
