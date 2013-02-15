#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

// Max number of shorts in conversion buffer
#define CONV_BUF_SIZE   2048

// Patterns
#define PATTERN_15C_1V  0
#define PATTERN_8C_8V   1

typedef struct {
    unsigned short tstep;                   // Number of ticks per sample
    unsigned long timestamp;                // Timestamp for beginning
    unsigned short data[CONV_BUF_SIZE];     // The data
    unsigned char  filled;                  // Whether this slot is filled
    unsigned char  pattern;                 // Pattern of voltages and currents
    unsigned char  input;                   // Which ADC this came from
} ADC_Data;

ADC_Data data_bufs[4];

unsigned short ADC1_data0[CONV_BUF_SIZE];     // The data for ADC1
unsigned short ADC1_data1[CONV_BUF_SIZE];     // The data for ADC1

int dma_count = 0;

void DMA2_Stream0_IRQHandler()
{
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
        dma_count = (dma_count + 1) & ((1 << 8)-1);
        if(dma_count == 0)
            GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
}

int tm_count = 0;

void TIM2_IRQHandler()
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    tm_count = (tm_count + 1) & ((1 << 18)-1);
    if(tm_count == 0)
        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

    ADC_SoftwareStartConv(ADC1);
}


int main()
{
    int i;

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;
    GPIO_InitTypeDef  GPIOD_InitStructure, GPIOA_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    __disable_irq();

    // RESET STUFF
    DMA_DeInit(DMA2_Stream0);
    ADC_DeInit();
    TIM_DeInit(TIM2);
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOD);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);

    GPIO_Write(GPIOD, GPIO_Pin_12);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, GPIO_Pin_13);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, GPIO_Pin_14);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, GPIO_Pin_15);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, GPIO_Pin_14);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, GPIO_Pin_13);
    for(i = 0; i < 1000000; ++i);
    GPIO_Write(GPIOD, 0);

    // Set up analog inputs
    GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIOA_InitStructure);

     /* Configure and enable DMA interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Set up DMA
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1_data0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = CONV_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_3QuartersFull;
    // DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 100; //~280kHz
    TIM_TimeBaseStructure.TIM_Period = 115; //~250kHz
    TIM_TimeBaseStructure.TIM_Period = 230; //~125kHz
    // TIM_TimeBaseStructure.TIM_Period = 400; //~70kHz trigger
    // TIM_TimeBaseStructure.TIM_Period = 15; //~2MHz trigger
    // TIM_TimeBaseStructure.TIM_Period = 30; //~1MHz trigger
    TIM_TimeBaseStructure.TIM_Period = 101; //~0.5MHz trigger
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock / 2) / 50000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1000 ;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    // ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    // ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfConversion = 2;
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC_DiscModeChannelCountConfig(ADC1, 1);
    // ADC_DiscModeCmd(ADC1, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 4, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 5, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 7, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 8, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 9, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 10, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 11, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 12, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 13, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 14, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 15, ADC_SampleTime_3Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 16, ADC_SampleTime_3Cycles);

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);

    __enable_irq();

    TIM_Cmd(TIM2, ENABLE);

    /* PD12 to be toggled */
    while(1);
    {

        // while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

        // i = ADC_GetConversionValue(ADC1);

        // GPIO_Write(GPIOD, GPIO_Pin_13);
        // for(i = 0; i < 1000000; ++i);
        // GPIO_Write(GPIOD, GPIO_Pin_14);
        // for(i = 0; i < 1000000; ++i);
        GPIO_Write(GPIOD, GPIO_Pin_15);
        for(i = 0; i < 1000000; ++i);
        GPIO_Write(GPIOD, GPIO_Pin_14);
        for(i = 0; i < 1000000; ++i);

        // ADC_SoftwareStartConv(ADC1);
    }
}


void exit()
{
    while(1);
}