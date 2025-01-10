/**
* Copyright (c) Mipot S.p.A. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       mip_b.c
* @date
* @version
*
*/

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <string.h>
#include "trinamic_tim.h"
#include "mip_d.h"
#include "mip_d_def.h"
#include "cortexcomm.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPD_TASK_PRIOTITY 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipdInit(struct mip_d *const dev);
static void MipdApp(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_d mipd;
TaskHandle_t mipd_device_task_handle;

/*******************************************************************************
 * Code
 ******************************************************************************/

void MipdTask(void)
{
	enum mip_error_t stat;
	(void)MipdInit(&mipd);
	mipd.delay_ms_fn(100);
	stat = mipd_get_fw_version(&mipd);
	if(stat != no_error)
		return;
	(void)xTaskCreate(MipdApp,"MipdApp",100, NULL,  MIPD_TASK_PRIOTITY, &mipd_device_task_handle);
}

static void MipdApp(void *pvParameters)
{
	const TickType_t xDelayMipDApp = pdMS_TO_TICKS(100);
	(void)mipd_get_fw_version(&mipd);
	(void)mipd_get_serial_no(&mipd);
	for(;;)
	{
		(void)mipd_get_serial_no(&mipd);
		(void)vTaskDelay(xDelayMipDApp);
	}
	(void)vTaskDelete(NULL);
}

static void MipdInit(struct mip_d *const dev)
{
	dev->is_present                         = false;
	dev->hardware_init_fn                   = NULL;
	dev->send_and_receive_fn                = MipTransmitAndReceiveData;
	dev->receive_fn                         = NULL;
	dev->hardware_reset_fn                  = NULL;
	dev->delay_ms_fn                        = Delay_ms;
	dev->radio_py.power                     = MIPD_POWER_DEF;
	dev->radio_py.frequency_channel         = MIPD_CHANNEL_FREQUENCY_DEF;
	dev->radio_py.bandwidth                 = MIPD_CHANNEL_BANDWIDTH_DEF;
	dev->radio_py.spreading_factor          = MIPD_SPREADING_FACTOR_DEF;
	dev->radio_py.code_rate                 = MIPD_CODE_RATE_DEF;
	dev->module_param.DATA_INDICATE_TIMEOUT = DATA_IND_TIMEOUT_DEF_VAL;
	dev->module_param.UartBaudrate          = MIPD_UartBaudrate_DEF_VAL;
	dev->module_param.AppEnAes              = MIPD_AppEnAes_DEF_VAL;
}
