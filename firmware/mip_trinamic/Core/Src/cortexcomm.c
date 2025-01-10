/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "cortexcomm.h"
#include "trinamic_tim.h"
#include "ipcc.h"
#include "mip_d.h"
#include "mip_d_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIP_IPCC_TX_TIMEOUT 100

typedef enum{
	IPCC_TXRX_HANDLER_IDLE                  = 0x00U,
	IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION  = 0x01U,
	IPCC_TXRX_HANDLER_RX_DATA_WAIT          = 0x02U,
	IPCC_TXRX_HANDLER_DATA_COPY             = 0x03U,
	IPCC_TXRX_HANDLER_COMPLETED             = 0x04U,
	IPCC_TXRX_HANDLER_ERROR                 = 0x05U
}cortexcomm_ipcc_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
cortexcomm_ipcc_t ipcc_handler = IPCC_TXRX_HANDLER_IDLE;

/*******************************************************************************
 * Code
 ******************************************************************************/
enum mip_error_t MipTransmitAndReceiveData(uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms)
{
	enum mip_error_t stat = unknown_error;
	uint32_t time;
	ipcc_handler = IPCC_TXRX_HANDLER_IDLE;
	while( (ipcc_handler != IPCC_TXRX_HANDLER_COMPLETED) && (ipcc_handler != IPCC_TXRX_HANDLER_ERROR) )
	{
		switch(ipcc_handler)
		{
			case IPCC_TXRX_HANDLER_IDLE:
			{
				if(tx_dim > 0)
				{
					ipcc_handler = IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION;
				}
				break;
			}

			case IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION:
			{
				/* Copy message to IPCC common buffer */
				memcpy ((uint8_t *)ipccCommBuff, tx_buff, tx_dim);
				/* Notify remote cpu of the on-going transaction */
				(void)HAL_IPCC_NotifyCPU(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX);
				CM0_rx = 0;
				time = GetTick();
				ipcc_handler = IPCC_TXRX_HANDLER_RX_DATA_WAIT;
				break;
			}

			case IPCC_TXRX_HANDLER_RX_DATA_WAIT:
			{
				if( GetTick() - time >= MIP_IPCC_TX_TIMEOUT)
				{
					stat =  tx_timeout;
					ipcc_handler = IPCC_TXRX_HANDLER_ERROR;
				}
				if(CM0_RX_COMM == CM0_rx)
				{
					ipcc_handler = IPCC_TXRX_HANDLER_DATA_COPY;
				}
			}

			case IPCC_TXRX_HANDLER_DATA_COPY:
			{
				memcpy(rx_buff, (uint8_t *)ipccCommBuff, ipccCommBuff[2] + 4);
				*rx_dim = ipccCommBuff[2] + 4;
				stat = no_error;
				ipcc_handler = IPCC_TXRX_HANDLER_COMPLETED;
				break;
			}

			case IPCC_TXRX_HANDLER_COMPLETED:
			{
				break;
			}

			case IPCC_TXRX_HANDLER_ERROR:
			{
				break;
			}

			default:
				ipcc_handler = IPCC_TXRX_HANDLER_IDLE;
		}
	}
	return stat;
}
