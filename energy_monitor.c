#include "stm32f4xx.h"

int main()
{
    int i;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER = 0x55000000;
    GPIOD->ODR = 0x00008000;
    for(i = 0; i < 1000000; ++i);
    GPIOD->ODR = 0x00000000;
    while(1)
    {
        for(i = 0; i < 1000000; ++i);
            GPIOD->ODR = 0x00008000;
        for(i = 0; i < 1000000; ++i);
            GPIOD->ODR = 0x00004000;
    }

}


void exit()
{
    while(1);
}