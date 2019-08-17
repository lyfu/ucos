#include "bsp_clock.h"


#define  BSP_BIT_RCC_PLLCFGR_PLLM               25u
#define  BSP_BIT_RCC_PLLCFGR_PLLN              336u
#define  BSP_BIT_RCC_PLLCFGR_PLLP                2u
#define  BSP_BIT_RCC_PLLCFGR_PLLQ                7u

void Error_Handler(void);


/**
  * @brief SysTick Clock Configuration
  * @retval None
  */

static void  SysTick_Init(void)
{
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);       //1ms
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef  RCC_OscInitStruct;
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;

    HAL_RCC_DeInit();

	/** Configure the main internal regulator output voltage 
	*/
    __HAL_RCC_PWR_CLK_ENABLE();                                 /* Enable Power Control clock.                          */
                                                                /* See Note 3.                                          */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


	/** Initializes the CPU, AHB and APB busses clocks 
	*/
                                                                /* PLLCLK    = HSE * (PLLN / PLLM)      = 336MHz.       */
                                                                /* SYSCLK    = PLLCLK / PLLP            = 168MHz.       */
                                                                /* OTG_FSCLK = PLLCLK / PLLQ            =  48MHz.       */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = BSP_BIT_RCC_PLLCFGR_PLLM;
    RCC_OscInitStruct.PLL.PLLN       = BSP_BIT_RCC_PLLCFGR_PLLN;
    RCC_OscInitStruct.PLL.PLLP       = BSP_BIT_RCC_PLLCFGR_PLLP;
    RCC_OscInitStruct.PLL.PLLQ       = BSP_BIT_RCC_PLLCFGR_PLLQ;
   // HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;          /* HCLK    = AHBCLK  = PLL / AHBPRES(1) = 168MHz.       */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           /* APB1CLK = AHBCLK  / APB1DIV(4)       = 42MHz (max).  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           /* APB2CLK = AHBCLK  / APB2DIV(2)       = 84MHz.        */
    //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
	
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	
	/*systick init 1ms*/
	SysTick_Init();

}




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
