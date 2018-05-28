#include "stm32f10x.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_gpio.h"

void delay(uint32_t t);

int main()
{
    // set clock
    RCC->CR |= RCC_CR_HSEON; // enable external clock

    // wait for ready HSE
    while (!(RCC->CR | RCC_CR_HSERDY))
    {
    }

    // switch to HSE clock
    RCC->CFGR |= RCC_CFGR_SW_HSE;

    // Enable clock for PC port
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    // GPIO configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_1; // MODE13[1:0] bits (01) - max speed 10Mhz
    GPIOC->CRH &= ~GPIO_CRH_CNF13; // Reset CNF13[1:0] bits (00) - push pull mode


    while (1)
    {
      GPIOC->BSRR |= GPIO_BSRR_BS13;
      delay(200000);
      GPIOC->BSRR |= GPIO_BSRR_BR13;
      delay(50000);
    }

    return 0;
}

void delay(uint32_t t)
{
  for (uint32_t i = 0; i < t; i++)
  {
    asm("NOP");
  }
}




/*
int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef Init;

	Init.GPIO_Mode = GPIO_Mode_Out_PP;
	Init.GPIO_Pin = GPIO_Pin_13;
	Init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &Init);

	while(1)
    {
        if	(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET)
        {
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            delay(2000000);
        }
        if	(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET)
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            delay(5000000);
        }
	}
	return 0;
}*/




/*#include <stm32f10x_conf.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#define RCC_GPIO RCC_APB2Periph_GPIOC
#define LED_PORT GPIOC
#define LED1_PIN GPIO_Pin_13
#define LED2_PIN GPIO_Pin_14

void Delay(volatile uint32_t nCount) {
    for (; nCount != 0; nCount--);
}

int main(void) {
    // SystemInit() startup_stm32f10x_md_vl.c

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED1_PIN | LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init( LED_PORT , &GPIO_InitStructure);

    LED_PORT->ODR ^= LED2_PIN;
    while (1) {
        LED_PORT->ODR ^= LED2_PIN;
        LED_PORT->ODR ^= LED1_PIN;
        Delay(0x7FFFF);
    }
    return 0;
}*/
