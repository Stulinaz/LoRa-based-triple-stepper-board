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
#include "mip_d.h"
#include "mip_d_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPD_BUFF_SZ 250
#define MIPD_DELAY_MSG 50

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t mipd_tx_buff[MIPD_BUFF_SZ] = {0};
uint8_t mipd_rx_buff[MIPD_BUFF_SZ] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Validate the device pointer for null conditions.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_null_ptr_check(const struct mip_d *const dev)
{
	enum mip_error_t status = no_error;
    if ( (dev == NULL) ||
         (dev->hardware_init_fn == NULL) ||
    	 (dev->send_and_receive_fn == NULL) ||
		 (dev->receive_fn == NULL) ||
		 (dev->delay_ms_fn == NULL) ||
		 (dev->hardware_reset_fn == NULL) )
    {
        status = null_ptr;
    }
    return status;
}

/*!
 * @brief This command performs a module Reset.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_reset(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_RESET_CMD;
	mipd_tx_buff[2] = 0x00;
	mipd_tx_buff[3] = mip_generate_checksum(mipd_tx_buff, 3);
	status = dev->send_and_receive_fn(mipd_tx_buff, 4, mipd_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIP_DELAY_RESET);
	return status;
}

/*!
 * @brief This command restores EEPROM factory default values.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_factory_reset(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_FACTORY_RESET_CMD;
	mipd_tx_buff[2] = 0x00;
	mipd_tx_buff[3] = mip_generate_checksum(mipd_tx_buff, 3);
	/* After a factory reset cmd response from mipd is coming after 140 ms */
	status = dev->send_and_receive_fn(mipd_tx_buff, 4, mipd_rx_buff, &rx_msg_len, 200);
	dev->delay_ms_fn(MIP_DELAY_FACTORY_RESET);
	return status;
}

/*!
 * @brief Get 32-bit firmware version.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_get_fw_version(struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_GET_FW_VERSION_CMD;
	mipd_tx_buff[2] = 0x00;
	mipd_tx_buff[3] = mip_generate_checksum(mipd_tx_buff, 3);
	status = dev->send_and_receive_fn(mipd_tx_buff, 4, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			dev->fw_version =  ((uint32_t) mipd_rx_buff[6] << 24);
			dev->fw_version |= ((uint32_t) mipd_rx_buff[5] << 16);
			dev->fw_version |= ((uint32_t) mipd_rx_buff[4] << 8);
			dev->fw_version |= ((uint32_t) mipd_rx_buff[3] );
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief Get unique 32-bit Serial Number.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_get_serial_no(struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_GET_SERIALNO_CMD;
	mipd_tx_buff[2] = 0x00;
	mipd_tx_buff[3] = mip_generate_checksum(mipd_tx_buff, 3);
	status = dev->send_and_receive_fn(mipd_tx_buff, 4, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			dev->serialno  = ((uint32_t) mipd_rx_buff[6] << 24);
			dev->serialno |= ((uint32_t) mipd_rx_buff[5] << 16);
			dev->serialno |= ((uint32_t) mipd_rx_buff[4] << 8);
			dev->serialno |= ((uint32_t) mipd_rx_buff[3] );
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief TODO
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_eeprom_write_radio_phy_param(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipd_tx_buff[2] = 0x06; /* lenght */
	mipd_tx_buff[3] = 0x00; /* start address */
	mipd_tx_buff[4] = dev->radio_py.power;
	mipd_tx_buff[5] = dev->radio_py.frequency_channel;
	mipd_tx_buff[6] = dev->radio_py.bandwidth;
	mipd_tx_buff[7] = dev->radio_py.spreading_factor;
	mipd_tx_buff[8] = dev->radio_py.code_rate;
	mipd_tx_buff[9] = mip_generate_checksum(mipd_tx_buff, 9);
	status = dev->send_and_receive_fn(mipd_tx_buff, 10, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			if (mipd_rx_buff[3] != MIP_RET_SUCCESS)
			{
				status = invalid_address;
			}
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief TODO
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_eeprom_read_radio_phy_param(const struct mip_d *const dev, struct d_radio_phy_t *const config)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipd_tx_buff[2] = 0x02;
	mipd_tx_buff[3] = 0x00; /* Start Address */
	mipd_tx_buff[4] = 0x05; /* Number of bytes */
	mipd_tx_buff[5] = mip_generate_checksum(mipd_tx_buff, 5);
	status = dev->send_and_receive_fn(mipd_tx_buff, 6, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			if (mipd_rx_buff[3] != MIP_RET_SUCCESS)
			{
				status = failure;
			}
			else
			{
				config->power             = mipd_rx_buff[4];
				config->frequency_channel = mipd_rx_buff[5];
				config->bandwidth         = mipd_rx_buff[6];
				config->spreading_factor  = mipd_rx_buff[7];
				config->code_rate         = mipd_rx_buff[8];
			}
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief TODO
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_eeprom_write_module_parameters(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipd_tx_buff[2] = 0x04; /* lenght */
	mipd_tx_buff[3] = 0x05; /* start address */
	mipd_tx_buff[4] = dev->module_param.DATA_INDICATE_TIMEOUT;
	mipd_tx_buff[5] = dev->module_param.UartBaudrate;
	mipd_tx_buff[6] = dev->module_param.AppEnAes;
	mipd_tx_buff[7] = mip_generate_checksum(mipd_tx_buff, 7);
	status = dev->send_and_receive_fn(mipd_tx_buff, 8, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			if (mipd_rx_buff[3] != MIP_RET_SUCCESS)
			{
				status = invalid_address;
			}
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief Get the module parameters from device EEPROM.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_eeprom_read_module_parameters(const struct mip_d *const dev, struct d_module_param_t *const config)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipd_tx_buff[2] = 0x02;
	mipd_tx_buff[3] = 0x05; /* Start Address */
	mipd_tx_buff[4] = 0x03; /* Number of bytes */
	mipd_tx_buff[5] = mip_generate_checksum(mipd_tx_buff, 5);
	status = dev->send_and_receive_fn(mipd_tx_buff, 6, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			if (mipd_rx_buff[3] != MIP_RET_SUCCESS)
			{
				status = failure;
			}
			else
			{
				config->DATA_INDICATE_TIMEOUT = mipd_rx_buff[4];
				config->UartBaudrate          = mipd_rx_buff[5];
				config->AppEnAes              = mipd_rx_buff[6];
			}
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief TODO
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_enable_ndata_indicate_pin(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = 0x32;
	mipd_tx_buff[2] = 0x02;
	mipd_tx_buff[3] = 0x91;
	mipd_tx_buff[4] = 0x01;
	mipd_tx_buff[5] = mip_generate_checksum(mipd_tx_buff, 5);
	status = dev->send_and_receive_fn(mipd_tx_buff, 6, mipd_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}


/*!
 * @brief This command performs the transmission of radio frames.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_tx_msg_cmd(const uint8_t *msg, uint8_t msg_len, struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	if ( (msg_len == 0) || (msg_len > 240) )
		return tx_message_len_error;
	mipd_tx_buff[idx++] = MIP_HEADER;
	mipd_tx_buff[idx++] = MIPD_TX_MSG_CMD;
	mipd_tx_buff[idx++] = msg_len;
	for(i=0; i<msg_len; i++)
	{
		mipd_tx_buff[idx++] = msg[i];
	}
	crc = mip_generate_checksum(mipd_tx_buff, idx);
	mipd_tx_buff[idx++] = crc;

	/* Send the trasmission command with payload */
	status = dev->send_and_receive_fn(mipd_tx_buff, idx, mipd_rx_buff, &rx_msg_len, 5);
	if(status != no_error)
		return status;

	status = mip_validate_response(mipd_rx_buff, rx_msg_len);
	if(status != no_error)
		return status;

	if(mipd_rx_buff[3] != no_error)
	{
		if(mipd_rx_buff[3] == 0x01)
			status = device_busy;
		else if(mipd_rx_buff[3] == 0x03)
			status = payload_size_error;
		else
			status = unknown_error;
		return status;
	}

	/* Receive the tx message indicator from module*/
	status = dev->receive_fn(mipd_rx_buff, &rx_msg_len, dev->module_param.DATA_INDICATE_TIMEOUT + 12000);
	if(status != no_error)
		return status;

	status = mip_validate_response(mipd_rx_buff, rx_msg_len);
	if(status != no_error)
		return status;

	if(mipd_rx_buff[3] == no_error)
	{
		dev->tx_data_info.on_air_time  = ((uint32_t) mipd_rx_buff[7] << 24);
		dev->tx_data_info.on_air_time |= ((uint32_t) mipd_rx_buff[6] << 16);
		dev->tx_data_info.on_air_time |= ((uint32_t) mipd_rx_buff[5] << 8);
		dev->tx_data_info.on_air_time |= ((uint32_t) mipd_rx_buff[4]);
		dev->tx_data_info.tx_msg_num++;
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief TODO
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_receive_message(struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	uint8_t payload_len;
	uint8_t i;
	status = dev->receive_fn(mipd_rx_buff, &rx_msg_len, dev->module_param.DATA_INDICATE_TIMEOUT + 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			dev->rx_data_info.RssiLSB   = mipd_rx_buff[4];
			dev->rx_data_info.RssiMSB   = mipd_rx_buff[5];
			dev->rx_data_info.SNR       = mipd_rx_buff[6];
			payload_len = rx_msg_len - 4;
			for(i=0; i<payload_len; i++)
			{
				dev->rx_data_info.last_rx_msg[i] = mipd_rx_buff[7+i];
			}
			dev->rx_data_info.last_rx_msg_len = payload_len;
			dev->rx_data_info.rx_msg_num++;
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief Write EEPROM parameter AES encryption key.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_set_aes_key(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipd_tx_buff[idx++] = MIP_HEADER;
	mipd_tx_buff[idx++] = MIPD_SET_AES_KEY_CMD;
	mipd_tx_buff[idx++] = 0x10;
	for(i=0; i<16; i++)
	{
		mipd_tx_buff[idx++] = dev->AESKey[i];
	}
	crc = mip_generate_checksum(mipd_tx_buff, idx);
	mipd_tx_buff[idx++] = crc;
	status = dev->send_and_receive_fn(mipd_tx_buff, idx, mipd_rx_buff, &rx_msg_len, 5);
	if(status == no_error)
	{
		status = mip_validate_response(mipd_rx_buff, rx_msg_len);
		if(status == no_error)
		{
			if(mipd_rx_buff[3] != no_error)
			{
				status = unknown_error;
			}
		}
	}
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief Write EEPROM parameter IV for encryption.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipd_set_init_vector(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipd_tx_buff[idx++] = MIP_HEADER;
	mipd_tx_buff[idx++] = MIPD_SET_IV_CMD;
	mipd_tx_buff[idx++] = 0x0A;
	for(i=0; i<16; i++)
	{
		mipd_tx_buff[idx++] = dev->InitVector[i];
	}
	crc = mip_generate_checksum(mipd_tx_buff, idx);
	mipd_tx_buff[idx++] = crc;
	status = dev->send_and_receive_fn(mipd_tx_buff, idx, mipd_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief After this command, the module enters sleep mode.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval Success.
 */
enum mip_error_t mipd_set_sleep(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_SLEEP_CMD;
	mipd_tx_buff[2] = 0x01;
	mipd_tx_buff[3] = 0x00;
	mipd_tx_buff[4] = mip_generate_checksum(mipd_tx_buff, 4);
	status = dev->send_and_receive_fn(mipd_tx_buff, 5, mipd_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}

/*!
 * @brief After this command, the module exits sleep mode.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution status.
 * @retval Success.
 */
enum mip_error_t mipd_wakeup(const struct mip_d *const dev)
{
	enum mip_error_t status;
	uint16_t rx_msg_len;
	mipd_tx_buff[0] = MIP_HEADER;
	mipd_tx_buff[1] = MIP_SLEEP_CMD;
	mipd_tx_buff[2] = 0x01;
	mipd_tx_buff[3] = 0x01;
	mipd_tx_buff[4] = mip_generate_checksum(mipd_tx_buff, 4);
	status = dev->send_and_receive_fn(mipd_tx_buff, 5, mipd_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPD_DELAY_MSG);
	return status;
}
