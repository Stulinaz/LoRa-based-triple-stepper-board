/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_dac.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DAC_GPIO_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
DAC_HandleTypeDef hdac;

/*******************************************************************************
 * Code
 ******************************************************************************/
void DAC_Init(void)
{
	/* Enable clock @ DAC */
	RCC->APB1ENR1 |= RCC_APB1ENR1_DACEN;
	__NOP();
	__NOP();

	DAC_ChannelConfTypeDef sConfig = {0};
	hdac.Instance = DAC;
	(void)HAL_DAC_Init(&hdac);
	sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
	(void)HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
	DAC_GPIO_Init();

	/* DAC channel1 enabled */
	DAC->CR |= DAC_CR_EN1;
}

static void DAC_GPIO_Init(void)
{
	/* Configure GPIO pin : AIN_REF_MIP_Pin */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = AIN_REF_MIP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AIN_REF_MIP_Port, &GPIO_InitStruct);
}
