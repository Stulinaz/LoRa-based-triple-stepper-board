/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "app_cortexcomm.h"
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
static void CortexCommManager(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t cm0_tx_buff[COMM_BUF_MAX_LEN];
uint8_t cm0_rx_buff[COMM_BUF_MAX_LEN];

uint32_t *CommRxStringLen;
uint8_t  *CommTxBuf;
uint8_t  *CommRxBuf;
static uint32_t SendMessageWithLen = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void vTaskAppCortexComm(void)
{
	(void)xTaskCreate(CortexCommManager,"CortexCommManager",250, NULL, CORTEXCOMM_TASK_PRIORITY, &cortexcomm_task_handle);
}

static void CortexCommManager(void *pvParameters)
{
	uint8_t len = 4;
	const TickType_t xDelay = pdMS_TO_TICKS(100);
	CommRxBuf = (uint8_t *) cm0_rx_buff;
	CommTxBuf = (uint8_t *) cm0_tx_buff;
	cm0_rx_buff[0] = 0xAA;
	cm0_rx_buff[1] = 0x34;
	cm0_rx_buff[2] = 0x00;
	cm0_rx_buff[3] = 0x22;
	for(;;)
	{
		/* FROM CORTEX M4(USER APPLICATION) TO CORTEX M0(LORA APPLICATION) */
		if (len > 0)
		{
			/* Copy message to IPCC common buffer */
			memcpy ((uint8_t *)ipccCommBuff, CommRxBuf, len);
			/* Notify remote cpu of the on-going transaction */
			(void)HAL_IPCC_NotifyCPU (&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX);
			memset (CommRxBuf, 0, COMM_BUF_MAX_LEN);
			len = 0;
		}

		/* FROM CORTEX M0(LORA APPLICATION) TO CORTEX M4(USER APPLICATION) */
		if ((CM0_rx == CM0_TX_COMM) || (CM0_rx == CM0_RX_COMM))
		{
			if ((ipccCommBuff[0] == CMD_HEADER_BYTE) && (ipccCommBuff[2] < IPCC_COMM_SIZE - 4))
			{
				SendMessageWithLen = ipccCommBuff[2] + 4;
				memcpy (CommTxBuf, (uint8_t *)ipccCommBuff, SendMessageWithLen);
			}
			if (CM0_rx == CM0_TX_COMM)
			{
				/* Notify remote cpu that transaction is completed */
				(void)HAL_IPCC_NotifyCPU(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX);
			}
			CM0_rx = 0;
			memset ((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);
		}
		(void)vTaskDelay(xDelay);
	}
	(void)vTaskDelete(NULL);
}
