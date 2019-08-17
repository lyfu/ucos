#include "bsp_led.h"


/**
  * @brief  Led Init
  * @retval None
  */

void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_PORT, (LED1_PIN|LED2_PIN), GPIO_PIN_RESET);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = LED1_PIN|LED2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(LED1_PORT, (LED1_PIN|LED2_PIN), GPIO_PIN_SET);
}
