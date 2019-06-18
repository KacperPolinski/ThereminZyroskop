/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32l4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dac_ch1;

extern DMA_HandleTypeDef hdma_usart2_rx;

extern DMA_HandleTypeDef hdma_usart2_tx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
//void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(hdac->Instance==DAC1)
//  {
//  /* USER CODE BEGIN DAC1_MspInit 0 */
//
//  /* USER CODE END DAC1_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_DAC1_CLK_ENABLE();
//
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**DAC1 GPIO Configuration
//    PA4     ------> DAC1_OUT1
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    /* DAC1 DMA Init */
//    /* DAC_CH1 Init */
//    hdma_dac_ch1.Instance = DMA1_Channel3;
//    hdma_dac_ch1.Init.Request = DMA_REQUEST_6;
//    hdma_dac_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_dac_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_dac_ch1.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_dac_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_dac_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_dac_ch1.Init.Mode = DMA_CIRCULAR;
//    hdma_dac_ch1.Init.Priority = DMA_PRIORITY_LOW;
//    if (HAL_DMA_Init(&hdma_dac_ch1) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    __HAL_LINKDMA(hdac,DMA_Handle1,hdma_dac_ch1);
//
//  /* USER CODE BEGIN DAC1_MspInit 1 */
//
//  /* USER CODE END DAC1_MspInit 1 */
//  }
//
//}
//
///**
//* @brief DAC MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hdac: DAC handle pointer
//* @retval None
//*/
//
//void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
//{
//
//  if(hdac->Instance==DAC1)
//  {
//  /* USER CODE BEGIN DAC1_MspDeInit 0 */
//
//  /* USER CODE END DAC1_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_DAC1_CLK_DISABLE();
//
//    /**DAC1 GPIO Configuration
//    PA4     ------> DAC1_OUT1
//    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
//
//    /* DAC1 DMA DeInit */
//    HAL_DMA_DeInit(hdac->DMA_Handle1);
//  /* USER CODE BEGIN DAC1_MspDeInit 1 */
//
//  /* USER CODE END DAC1_MspDeInit 1 */
//  }
//
//}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
//{
//
//  if(htim_base->Instance==TIM6)
//  {
//  /* USER CODE BEGIN TIM6_MspInit 0 */
//
//  /* USER CODE END TIM6_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_TIM6_CLK_ENABLE();
//  /* USER CODE BEGIN TIM6_MspInit 1 */
//
//  /* USER CODE END TIM6_MspInit 1 */
//  }
//
//}
//
///**
//* @brief TIM_Base MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param htim_base: TIM_Base handle pointer
//* @retval None
//*/
//
//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
//{
//
//  if(htim_base->Instance==TIM6)
//  {
//  /* USER CODE BEGIN TIM6_MspDeInit 0 */
//
//  /* USER CODE END TIM6_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM6_CLK_DISABLE();
//  /* USER CODE BEGIN TIM6_MspDeInit 1 */
//
//  /* USER CODE END TIM6_MspDeInit 1 */
//  }
//
//}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

extern DMA_HandleTypeDef hdma_sai1_a;

static uint32_t SAI1_client =0;

//void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct;
///* SAI1 */
//    if(hsai->Instance==SAI1_Block_A)
//    {
//    /* Peripheral clock enable */
//    if (SAI1_client == 0)
//    {
//       __HAL_RCC_SAI1_CLK_ENABLE();
//    }
//    SAI1_client ++;
//
//    /**SAI1_A_Block_A GPIO Configuration
//    PE2     ------> SAI1_MCLK_A
//    PE4     ------> SAI1_FS_A
//    PE5     ------> SAI1_SCK_A
//    PE6     ------> SAI1_SD_A
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
//    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//    /* Peripheral DMA init*/
//
//    hdma_sai1_a.Instance = DMA2_Channel1;
//    hdma_sai1_a.Init.Request = DMA_REQUEST_1;
//    hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
//    hdma_sai1_a.Init.Priority = DMA_PRIORITY_MEDIUM;
//    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    /* Several peripheral DMA handle pointers point to the same DMA handle.
//     Be aware that there is only one channel to perform all the requested DMAs. */
//    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_a);
//
//    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_a);
//
//    }
//}
//
//void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
//{
//
///* SAI1 */
//    if(hsai->Instance==SAI1_Block_A)
//    {
//    SAI1_client --;
//    if (SAI1_client == 0)
//      {
//      /* Peripheral clock disable */
//       __HAL_RCC_SAI1_CLK_DISABLE();
//      }
//
//    /**SAI1_A_Block_A GPIO Configuration
//    PE2     ------> SAI1_MCLK_A
//    PE4     ------> SAI1_FS_A
//    PE5     ------> SAI1_SCK_A
//    PE6     ------> SAI1_SD_A
//    */
//    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
//
//    HAL_DMA_DeInit(hsai->hdmarx);
//    HAL_DMA_DeInit(hsai->hdmatx);
//    }
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
