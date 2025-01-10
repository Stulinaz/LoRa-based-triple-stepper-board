/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "app_lora.h"
#include <string.h>
#include "ipcc.h"
#include "sys_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define COMM_BUF_MAX_LEN 260
#define CMD_HEADER_BYTE	 0xAA
#define IFC_UART		 0
#define IFC_SPI			 1
#define IFC_I2C			 2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void AppLoRaManager(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t CommInterface;
uint32_t *CommRxStringLen;
uint8_t  *CommTxBuf;
uint8_t  *CommRxBuf;
static uint32_t SendMessageWithLen = 0;
volatile uint32_t tmLpm;
volatile uint8_t waitForSleep;
static uint16_t SendMessageDelay = 0;
uint8_t uartBaudRateSel;
uint32_t shouldUpdateUartSpeed = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

void vTaskAppLoRaManager(void)
{
	(void)xTaskCreate(AppLoRaManager,"AppLoRaManager",250, NULL, LORA_TASK_PRIORITY, &lora_task_handle);
}

static void AppLoRaManager(void)
{
	if (*CommRxStringLen > 0)
	{
		// Copy message to IPCC common buffer
	    memcpy ((uint8_t *)ipccCommBuff, CommRxBuf, *CommRxStringLen);

	    // Notify remote cpu of the on-going transaction
	    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX) != HAL_OK)
	    {
	    	Error_Handler();
	    }

		memset (CommRxBuf, 0, COMM_BUF_MAX_LEN);
		*CommRxStringLen = 0;
	}

	if ((CM0_rx == CM0_TX_COMM) || (CM0_rx == CM0_RX_COMM))
	{
		if ((ipccCommBuff[0] == CMD_HEADER_BYTE) && (ipccCommBuff[2] < IPCC_COMM_SIZE - 4))
		{
			SendMessageWithLen = ipccCommBuff[2] + 4;
			memcpy (CommTxBuf, (uint8_t *)ipccCommBuff, SendMessageWithLen);
		}

		if (CM0_rx == CM0_TX_COMM)
		{
		    // Notify remote cpu that transaction is completed
		    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX) != HAL_OK)
		    {
		    	Error_Handler();
		    }
		}
		CM0_rx = 0;

		memset ((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);

		if (SendMessageWithLen > 0)
		{
			if(SendMessageDelay == 0)
			{
				if (CommInterface == IFC_UART)
				{
					//startTxUart (SendMessageWithLen);
				}
				SendMessageWithLen = 0;
			}
		}
	}

	if (CM0_rx == CM0_RX_SERV)
	{
		CM0_rx = 0;

		switch (ipccServBuff[0])
		{
			case CM0_REQ_LPM:
				waitForSleep = 1;
				tmLpm = 5;
				break;

			case CM0_REQ_WUP:
				break;

			case CM0_REQ_UPD_BRATE:
				uartBaudRateSel = ipccServBuff[1];
				shouldUpdateUartSpeed = 1;
				break;
		}
		memset ((uint8_t *) ipccServBuff, 0, IPCC_SERV_SIZE);

	    // Notify remote cpu that transaction is completed
	    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_RX) != HAL_OK)
	    {
	    	Error_Handler();
	    }

	    if (shouldUpdateUartSpeed)
		{
			if(*CommTxBusy == 0)
			{
				shouldUpdateUartSpeed = 0;
				//updateUartSpeed();
			}
		}
	}

	if (*CommTxBusy == 2)
	{
		(*CommTxBusy)++;

		ipccServBuff[0] = CM0_TX_END;

	    // Notify remote cpu of the on-going transaction
	    if (HAL_IPCC_NotifyCPU (&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX) != HAL_OK)
	    {
	    	Error_Handler();
	    }
	}
}
