/******************************************************************************
 * @file    hw_usart.c
 * @author  MCD Application Team
 * @version V1.2.0
 * @date    10-July-2018
 * @brief   This file provides code for the configuration of the USART
 *          instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"


/*private variables*/
static struct {
  char buffTx[256];                         /* structure have to be simplified*/
  char buffRx[256];
  int rx_idx_free;
  int rx_idx_toread;
  HW_LockTypeDef Lock;
  __IO HAL_UART_StateTypeDef gState;
  __IO HAL_UART_StateTypeDef RxState;
} uart_context;

/* private function */
static void receive(char rx);

#ifdef USE_LRWAN_NS1

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 and USART2 init function */

HAL_StatusTypeDef HW_UART_Modem_Init(uint32_t BaudRate)
{
  huart1.Instance = USART1;

  huart1.Init.BaudRate = BaudRate; /*9600;*/
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  huart2.Instance = USART2;

  huart2.Init.BaudRate = BaudRate; /*9600;*/
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
    /* Initialization Error */
    Error_Handler();
  }

  /*******************************************************************/
  /*see Application Note AN4991 : how to wake up an STM32L0 MCU from */
  /*low power mode with the USART or the LPUART                      */
  /*******************************************************************/

  /*Enable RX Not Empty Interrupt*/   /*for test -- added today @@@@*/
  SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);


  return (HAL_OK);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance==USART1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /***************************************************************/
    /*      GPIO Configuration   for UART1 and for UART2           */
    /*           PA2     ------> USART2_TX                         */
    /*           PA3     ------> USART2_RX                         */
    /*           PA9     ------> USART1_TX                         */
    /*           PA10    ------> USART1_RX                         */
    /***************************************************************/

    GPIO_InitStruct.Pin = GPIO_PIN_9;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  } else if (huart->Instance==USART2)
  {

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

    /***************************************************************/
    /*      GPIO Configuration   for UART1 and for UART2           */
    /*           PA2     ------> USART2_TX                         */
    /*           PA3     ------> USART2_RX                         */
    /*           PA9     ------> USART1_TX                         */
    /*           PA10    ------> USART1_RX                         */
    /***************************************************************/

  if (huart->Instance==USART1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  } else if (huart->Instance==USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* To prevent interference from UART2 TX to PC */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}

#else

UART_HandleTypeDef huart2;

/* USART2 init function */

HAL_StatusTypeDef HW_UART_Modem_Init(uint32_t BaudRate)
{


#if USE_USART2
  huart2.Instance = USART2;
#else
  huart2.Instance = LPUART1;
#endif

  huart2.Init.BaudRate = BaudRate; /*9600;*/
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*******************************************************************/
  /*see Application Note AN4991 : how to wake up an STM32L0 MCU from */
  /*low power mode with the USART or the LPUART                      */
  /*******************************************************************/

   /*Enable RX Not Empty Interrupt*/   /*for test -- added */
    SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);

  /*Enable UART Stop Mode*/
  HAL_UARTEx_EnableStopMode(&huart2);


  /*Enable Wake Up from Stop Mode Interrupt*/
  SET_BIT(huart2.Instance->CR3, USART_CR3_WUFIE);         


  return (HAL_OK);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
#if USE_USART2
  if(huart->Instance==USART2)
#else
  if(huart->Instance==LPUART1)
#endif

  {
    /* Peripheral clock enable */
#if USE_USART2
    __HAL_RCC_USART2_CLK_ENABLE();
#else
    __HAL_RCC_LPUART1_CLK_ENABLE();
#endif

    /***************************************************************/
    /*      GPIO Configuration   for UART2 or for LPUART1          */
    /*      if USE_USART2 is defined (in hw_conf.h)                */
    /*           PA2     ------> USART2_TX                         */
    /*           PA3     ------> USART2_RX                         */
    /*      else                                                   */
    /*           PC10     ------> LPUART1_TX                       */
    /*           PC11     ------> LPUART1_RX                       */
    /***************************************************************/

#if USE_USART2
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
#else
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
#endif

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

#if USE_USART2
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
    GPIO_InitStruct.Alternate = GPIO_AF0_LPUART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif



    /* Peripheral interrupt init */
#if USE_USART2
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#else
    HAL_NVIC_SetPriority(RNG_LPUART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RNG_LPUART1_IRQn);
#endif

  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

    /***************************************************************/
    /*      GPIO DeConfiguration   for UART2 or for LPUART1        */
    /*      if USE_USART2 is defined (in hw_conf.h)                */
    /*           PA2     ------> USART2_TX                         */
    /*           PA3     ------> USART2_RX                         */
    /*      else                                                   */
    /*           PC10     ------> LPUART1_TX                       */
    /*           PC11     ------> LPUART1_RX                       */
    /***************************************************************/

#if USE_USART2
  if(huart->Instance==USART2)
#else
  if(huart->Instance==LPUART1)
#endif
  {
    /* Peripheral clock disable */
#if USE_USART2
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
#else
    __HAL_RCC_LPUART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);
#endif

  }

}

#endif

/******************************************************************************
  * @brief Handler on Rx IRQ
  * @param handle to the UART
  * @retval void
******************************************************************************/
void HW_UART_Modem_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its = READ_REG(huart->Instance->CR3);;
  uint32_t errorflags;
  int rx_ready = 0;



  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);

     /* forbid stop mode */
     LPM_SetStopMode(LPM_UART_RX_Id , LPM_Disable );

    /* Enable the UART Data Register not empty Interrupts */
    SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);

    /* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;

  }


  /* UART in mode Receiver ---------------------------------------------------*/
  if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
  {
  /* Check that a Rx process is ongoing */
    if(huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
            /*RXNE flag is auto cleared by reading the data*/
                        *huart->pRxBuffPtr++ = (uint8_t)READ_REG(huart->Instance->RDR);

                        /* allow stop mode*/
                        LPM_SetStopMode(LPM_UART_RX_Id , LPM_Enable );

      if(--huart->RxXferCount == 0U)
                        {
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
        huart->RxState = HAL_UART_STATE_READY;
        rx_ready = 1;  /* not used RxTC callback*/
      }
    }
    else
    {
      /* Clear RXNE interrupt flag */
      __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
      return;
    }
  }

  /* If error occurs */
   errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
   if (errorflags != RESET)
   {
      /* Error on receiving */
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
     /*   *((huart->pRxBuffPtr)-1) = 0x01;   */        /*we skip the overrun case*/
     rx_ready = 1;
   }

  if(rx_ready)
  {
    /*character in the ring buffer*/
    receive(*((huart->pRxBuffPtr)-1));
  }
}


/******************************************************************************
  * @brief To check if data has been received
  * @param none
  * @retval Reset no data / set data
******************************************************************************/
FlagStatus HW_UART_Modem_IsNewCharReceived(void)
{
  FlagStatus status;


  uint32_t primask_bit= __get_PRIMASK();

  __disable_irq();

  status = ((uart_context.rx_idx_toread == uart_context.rx_idx_free) ? RESET : SET);


  __set_PRIMASK(primask_bit);
  return status;
}




/******************************************************************************
  * @brief Get the received character
  * @param none
  * @retval Return the data received
******************************************************************************/
uint8_t HW_UART_Modem_GetNewChar(void)
{
  uint8_t NewChar;


  uint32_t primask_bit= __get_PRIMASK();

  __disable_irq();

  NewChar = uart_context.buffRx[uart_context.rx_idx_toread];
  uart_context.rx_idx_toread = (uart_context.rx_idx_toread + 1) % sizeof(uart_context.buffRx);


  __set_PRIMASK(primask_bit);
  return NewChar;
}



/******************************************************************************
  * @brief Store in ring buffer the received character
  * @param none
  * @retval none
******************************************************************************/
static void receive(char rx)
{
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.buffRx[uart_context.rx_idx_free] = rx;
  next_free = (uart_context.rx_idx_free + 1) % sizeof(uart_context.buffRx);
  if (next_free != uart_context.rx_idx_toread)
  {
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx_idx_free = next_free;
  }

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
