

#include "stm8s_conf.h"
#include "stm8s_it.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

#define LED_GPIO_PORT   GPIOB
#define LED_GPIO_PIN    GPIO_PIN_5

void DelayMsCycle(uint32_t ms)
{
  while (--ms)
  {
    uint16_t t = 0x400; /* ~1ms */
    while (--t)
      ;
  }
}

void main(void)
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  GPIO_Init(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  while (1)
  {
    GPIO_WriteReverse(LED_GPIO_PORT, LED_GPIO_PIN);
    DelayMsCycle(1000);
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif
