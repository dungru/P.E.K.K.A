/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.3.4
  * @date    06-May-2016
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit_User(UART_HandleTypeDef *huart)
{  
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_RX_GPIO_CLK_ENABLE();
    USARTx_TX_GPIO_CLK_ENABLE();
    /* Enable USART1 clock */
    USARTx_CLK_ENABLE(); 
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;
    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
      
    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;
    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure UART1 ##########################################*/ 
    huart->Instance          = USART1;
    huart->Init.BaudRate     = 115200;
    huart->Init.WordLength   = UART_WORDLENGTH_8B;
    huart->Init.StopBits     = UART_STOPBITS_1;
    huart->Init.Parity       = UART_PARITY_NONE;
    huart->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart->Init.Mode         = UART_MODE_TX_RX;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
      
    if(HAL_UART_Init(huart) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_UART_ENABLE(huart);

}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit_User(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

}

void uart1_puts(char* s)
{
    while(*s) {
        while(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);
        HAL_UART_Transmit(&UartHandle, (uint8_t *)s, 1, 5000);
        s++;
    }
}

static void Error_Handler(void)
{
  while(1)
  {
    BSP_LED_Toggle(LED3);
    HAL_Delay(500);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
