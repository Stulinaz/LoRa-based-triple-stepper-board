/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ipcc.h
  * @brief   This file contains all the function prototypes for
  *          the ipcc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IPCC_H__
#define __IPCC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "core_cm4.h"
/* USER CODE END Includes */

extern IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN Private defines */
/* A buffer is reserved inside the shared memory to exchange data between cpu.*/
#define IPCC_COMM_ADDR			0x2000FEFC;
#define IPCC_SERV_ADDR 			0x2000FEEC;
/* The address 0x2000FEE8 is reserved to indicate the state of the CPU2: Initialised or not */
#define CPU2_INIT_ADDR 			0x2000FEE8;

#define IPCC_COMM_SIZE 			260
#define IPCC_SERV_SIZE 			16
#define CPU2_INITIALISED 		0xAA
#define CPU2_NOT_INITIALISED 	0xBB

#define CH_ID_COMM				0
#define CH_ID_SERV				1

#define CM0_TX_COMM				1
#define CM0_RX_COMM				2
#define CM0_RX_SERV				3

#define CM0_TX_END				0xA5

#define CM0_REQ_LPM				0x01
#define CM0_REQ_WUP				0x02
#define CM0_REQ_UPD_BRATE		0x05
/* USER CODE END Private defines */

void MX_IPCC_Init(void);

/* USER CODE BEGIN Prototypes */
void CM0_COMM_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_COMM_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_SERV_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_SERV_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
/* USER CODE END Prototypes */

/* USER CODE BEGIN Globals */
extern __IO uint8_t CM0_rx;
extern __IO uint8_t *ipccCommBuff;
extern __IO uint8_t *ipccServBuff;
extern __IO uint8_t *cpu2InitDone;
/* USER CODE END Globals */

#ifdef __cplusplus
}
#endif

#endif /* __IPCC_H__ */

