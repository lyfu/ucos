#ifndef __BSP__LED__H__
#define __BSP__LED__H__

#include "stm32f4xx_hal.h"


#define LED1_PORT       GPIOB
#define LED1_PIN        GPIO_PIN_8
#define LED2_PORT       GPIOB
#define LED2_PIN        GPIO_PIN_9


void Led_Init(void);

#endif
