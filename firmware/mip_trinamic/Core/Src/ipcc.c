/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ipcc.c
  * @brief   This file provides code for the configuration
  *          of the IPCC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ipcc.h"

/* USER CODE BEGIN 0 */
//#include "comm.h"


__IO uint8_t CM0_rx;
uint8_t *CommTxBusy;
__IO uint8_t *ipccCommBuff = (uint8_t *)IPCC_COMM_ADDR;
__IO uint8_t *ipccServBuff = (uint8_t *)IPCC_SERV_ADDR;
/* The address 0x2000FEFC is reserved to indicate the state of the CPU2: Initialised or not */
__IO uint8_t *cpu2InitDone = (uint8_t *)CPU2_INIT_ADDR;
/* USER CODE END 0 */

IPCC_HandleTypeDef hipcc;

/* IPCC init function */
void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */
	memset ((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);
	memset ((uint8_t *) ipccServBuff, 0, IPCC_SERV_SIZE);
	*cpu2InitDone = CPU2_NOT_INITIALISED;
	CM0_rx = 0;
  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* Configure user callbacks */
  /* The callback is triggered when the other cpu has finished to handle the message. (IPCC_CHANNEL_DIR_TX) */
  if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX, CM0_COMM_tx_callback) != HAL_OK)
  {
    Error_Handler();
  }

  /* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
  if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX, CM0_COMM_rx_callback) != HAL_OK)
  {
    Error_Handler();
  }

  /* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
  if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX, CM0_SERV_tx_callback) != HAL_OK)
  {
    Error_Handler();
  }

  /* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
  if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_RX, CM0_SERV_rx_callback) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END IPCC_Init 2 */

}

void HAL_IPCC_MspInit(IPCC_HandleTypeDef* ipccHandle)
{

  if(ipccHandle->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspInit 0 */

  /* USER CODE END IPCC_MspInit 0 */
    /* IPCC clock enable */
    __HAL_RCC_IPCC_CLK_ENABLE();

    /* IPCC interrupt Init */
    HAL_NVIC_SetPriority(IPCC_C1_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
    HAL_NVIC_SetPriority(IPCC_C1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);
  /* USER CODE BEGIN IPCC_MspInit 1 */

  /* USER CODE END IPCC_MspInit 1 */
  }
}

void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* ipccHandle)
{

  if(ipccHandle->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspDeInit 0 */

  /* USER CODE END IPCC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_IPCC_CLK_DISABLE();

    /* IPCC interrupt Deinit */
    HAL_NVIC_DisableIRQ(IPCC_C1_RX_IRQn);
    HAL_NVIC_DisableIRQ(IPCC_C1_TX_IRQn);
  /* USER CODE BEGIN IPCC_MspDeInit 1 */

  /* USER CODE END IPCC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CM0_COMM_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	CM0_rx = CM0_RX_COMM;
}

void CM0_COMM_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	CM0_rx = CM0_TX_COMM;
}

void CM0_SERV_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	*CommTxBusy = 0;
}

void CM0_SERV_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	CM0_rx = CM0_RX_SERV;
}
/* USER CODE END 1 */
