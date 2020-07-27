
#include "define.h"
#include "stdio.h"
#include "stm8s.h"
#include "SHT3x.h"

#if SDCC_VERSION >= 30605
  #define PUTCHAR_PROTOTYPE int putchar(int c)
  #define GETCHAR_PROTOTYPE int getchar(void)
#else
  #define PUTCHAR_PROTOTYPE void putchar(char c)
  #define GETCHAR_PROTOTYPE char getchar(void)
#endif 

PUTCHAR_PROTOTYPE
{
  UART1_SendData8(c);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  return (c);
}

volatile uint32_t tickms = 0;

SHT3x Sensor;
float Temp, Hum;
uint8_t TempX10, HumX10;

void TickIncrement(void);
uint32_t GetTick(void);
static void DelayMsTick(volatile uint32_t ms);
static void CLK_Config(void);
static void UART_Config(void);
static void TIM4_Config(void);
static void DelayMsCycle(uint32_t ms);

void main(void)
{
  CLK_Config();  
  TIM4_Config();
  UART_Config();

  // GPIO_Init(RS485_DR_GPIO_PORT, RS485_DR_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
  // GPIO_WriteHigh(RS485_DR_GPIO_PORT, RS485_DR_GPIO_PIN);

  SHT3x_Begin(&Sensor);

  enableInterrupts();

  while (1)
  {
    SHT3x_UpdateData(&Sensor);
    Temp = SHT3x_GetTemperature(&Sensor, Cel);
    Hum = SHT3x_GetRelHumidity(&Sensor);
    TempX10 = Temp*10;
    HumX10 = Hum*10;
    printf("Temperature: %d C, Humidity: %d\n", TempX10, HumX10);
    DelayMsTick(333);
  }
}



void TickIncrement(void) 
{
  tickms++;
}

uint32_t GetTick(void)
{
  return tickms;
}

void DelayMsTick(volatile uint32_t ms) 
{
  volatile uint32_t wait = tickms + ms;
  while (tickms != wait);
}

void CLK_Config(void) 
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

void UART_Config(void)
{
  UART1_DeInit();
  UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
}

void TIM4_Config(void) 
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

void DelayMsCycle(uint32_t ms)
{
  while (--ms)
  {
    uint16_t t = 0x400; /* ~1ms */
    while (--t)
      ;
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
