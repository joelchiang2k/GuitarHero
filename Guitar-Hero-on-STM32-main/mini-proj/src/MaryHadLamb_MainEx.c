#include "stm32f0xx.h"
#include <math.h>   // for M_PI

#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}


void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

void init_wavetable(void)
{
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//=============================================================================
// Part 1: 7-segment display update with DMA
//=============================================================================

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);


//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void)
{
    int IOPBEN = 0x40000;
    RCC->AHBENR |= IOPBEN;

    int IOPCEN = 0x80000;
    RCC->AHBENR |= IOPCEN;

    GPIOB->MODER &= ~(0x003fffff);
    GPIOB->MODER |= 0x00155555;

    GPIOC->MODER &= ~(0x0000ffff);
    GPIOC->MODER |= 0x00005500;
    GPIOC->OTYPER |= 0x00f0;

    GPIOC->PUPDR &= ~(0x000000ff);
    GPIOC->PUPDR |= 0x00000055;

    return;
}

//============================================================================
// setup_dma()
//============================================================================
void setup_dma(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR = (uint32_t) 0x48000414;
    DMA1_Channel5->CMAR = (uint32_t) msg;
    DMA1_Channel5->CNDTR = 8;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;

    DMA1_Channel5->CCR &= ~(0x0f00);
    DMA1_Channel5->CCR |= 0x0500;

    DMA1_Channel5->CCR |= 1<<5;

    return;
}

//============================================================================
// enable_dma()
//============================================================================
void enable_dma(void)
{
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    return;
}

//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void)
{
    RCC->APB2ENR |= 1<<16;
    TIM15->PSC = 4800-1;
    TIM15->ARR = 10-1;
    TIM15->DIER |= 1<<8;
    TIM15->CR1 |= 1;

    return;
}

//=============================================================================
// Part 3: Analog-to-digital conversion for a volume level.
//=============================================================================
uint32_t volume = 2048;

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0xc; //ADC_IN1 == PA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while((ADC1->CR & ADC_CR_ADSTART));
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    return;
}

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;
//============================================================================
// Timer 2 ISR
//============================================================================

// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler()
{
    TIM2->SR &= ~(0x1);
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE;

    return;
}
//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void)
{
    RCC->APB1ENR |= 1;
    TIM2->PSC = 4800-1;
    TIM2->ARR = 1000-1;
    TIM2->DIER |= 1;
    TIM2->CR1 |= 1;
    NVIC->ISER[0] = 1<<15;

    return;
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~(DAC_CR_TSEL1);
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_EN1;

    return;
}

//============================================================================
// Timer 6 ISR
//============================================================================

// Write the Timer 6 ISR here.  Be sure to give it the right name.
void TIM6_DAC_IRQHandler()
{
    TIM6->SR &= ~(0x1);
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N << 16))
        offset0 -= (N << 16);
    if (offset1 >= (N << 16))
        offset1 -= (N << 16);
    int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
    samp = samp * volume;
    samp = samp >> 17;
    samp += 2048;
    DAC->DHR12R1 = samp;

    return;
}

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void)
{
    RCC->APB1ENR |= 1<<4;
    TIM6->PSC = 600-1;
    TIM6->ARR = 4-1;
    TIM6->DIER |= 1;
    TIM6->CR1 |= 1;
    TIM6->CR2 &= ~(0x70);
    TIM6->CR2 |= 0x20;
    NVIC->ISER[0] = 1<<17;

    return;
}

int main(void)
{
    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    setup_adc();
    init_tim2();
    init_wavetable();
    setup_dac();
    init_tim6();

    // sharp c, d, f, g
    float a3 = 220.00;
    float a4 = 440.00;
    float a5 = 880.00;

    float b3 = 246.94;
    float b4 = 493.88;

    float c4 = 261.63;
    float c5 = 523.25;
    float c5s = 554.37;

    float d4 = 293.66;
    float d4s = 311.13;
    float d5 = 587.33;
    float d5s = 622.25;

    float e4 = 329.63;
    float e5 = 659.25;

    float f4 = 349.23;
    float f5 = 689.46;
    float f5s = 739.99;

    float g3 = 196.00;
    float g4 = 392.00;
    float g4s = 415.30;
    float g5s = 830.61;

    float r = 0;


    float line1s[] = {c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4, r,
                    c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4}; // line 1
    float line2_3s[] = {r, b4, c5s, d5, d5, e5, c5s, b4, a4, r, b4, b4, c5s, d5, b4, r, a4,
                    a5, a5, e5, f5s, e5, d5, r, a4, a4, c5s, d5, b4, d5, e5}; // line 2-3 rep
    float line4_5s[] = {r, c5s, b4, c5s, b4, a4, r, b4, b4, c5s, d5, b4, a4,
                    e5, e5, e5, f5s, e5, d5, e5, f5s, d5}; // line 4-5
    float line6s[] = {e5, e5, e5, f5s, e5, a4, r, a4, b4, c5s, d5, b4,
                    r, e5, f5s, e5, a4, b4, d5, b4}; // line 6
    float line7s[] = {f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
                    a4, b4, d5, b4, d5, e5, c5s, a4, a4}; //line 7
    float line8s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
                    a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
                    a4, b4, d5, b4}; // line 8
    float line9s[] = {d5, e5, c5s, b4, a4, a4, /* line 10*/ e5, d5 /*back to line 2*/}; // line 9
    float line10s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
                    a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
                    a4, b4, d5, b4, d5, e5, c5s, a4, a4}; // line 10
    float line11s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
                    a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
                    a4, b4, d5, b4}; // line 11
    float line12s[] = {d5, e5, c5s, b4, a4, a4, e5, d5}; // line 12
    float line13s[] = {a4, d5, d5, r, c5s, r}; // line 13
    float line14s[] = {f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
                        a4, b4, d5, b4, d5, e5, c5s, a4, a4,
                        e5, d5, a4, b4, d5, b4}; // line 14
    float line15s[] = {f5s, f5s, e5, a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
                    a4, b4, d5, b4}; // line 15
    float line16s[] = {d5, e5, c5s, b4, a4, a4, e5, d5, a4, b4, d5, b4}; // line 16
    float last_measures[] = {e5, d5}; // last measure
    float song[] = {c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4, r,
            c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4, // line 1
            r, b4, c5s, d5, d5, e5, c5s, b4, a4, r, b4, b4, c5s, d5, b4, r, a4,
            a5, a5, e5, f5s, e5, d5, r, a4, a4, c5s, d5, b4, d5, e5, // lin3 2-3
            r, c5s, b4, c5s, b4, a4, r, b4, b4, c5s, d5, b4, a4,
            e5, e5, e5, f5s, e5, d5, e5, f5s, d5, // line 4-5
            e5, e5, e5, f5s, e5, a4, r, a4, b4, c5s, d5, b4,
            r, e5, f5s, e5, a4, b4, d5, b4, // line 6
            f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
            a4, b4, d5, b4, d5, e5, c5s, a4, a4, // line 7
            e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
            a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
            a4, b4, d5, b4, // line 8
            d5, e5, c5s, b4, a4, a4, /* line 10*/ e5, d5, /*back to line 2*/ // line 9
            //repeat
            a5, a5, e5, f5s, e5, d5, r, a4, a4, c5s, d5, b4, d5, e5, // lin3 2-3
            r, c5s, b4, c5s, b4, a4, r, b4, b4, c5s, d5, b4, a4,
            e5, e5, e5, f5s, e5, d5, e5, f5s, d5, // line 4-5
            e5, e5, e5, f5s, e5, a4, r, a4, b4, c5s, d5, b4,
            r, e5, f5s, e5, a4, b4, d5, b4, // line 6
            f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
            a4, b4, d5, b4, d5, e5, c5s, a4, a4, // line 7
            e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
            a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
            a4, b4, d5, b4, // line 8
            d5, e5, c5s, b4, a4, a4,
            e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
            a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
            a4, b4, d5, b4, d5, e5, c5s, a4, a4, // line 10
            e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
            a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
            a4, b4, d5, b4, // line 11
            d5, e5, c5s, b4, a4, a4, e5, d5, // line 12
            a4, d5, d5, r, c5s, r, // line 13
            f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
            a4, b4, d5, b4, d5, e5, c5s, a4, a4,
            e5, d5, a4, b4, d5, b4, // line 14
            f5s, f5s, e5, a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
            a4, b4, d5, b4, // line 15
            d5, e5, c5s, b4, a4, a4, //line 16
            e5, d5, r, r, r, r, r, r // last measure
            };

    // 1/4 = 0; 1/2 = 1; whole = 2; 1/16 = 3 ; 1/8 = 4; 3/16 =5; 1/4 + 1/8 = 6; 1/2 + 1/4 + 1/8 = 7; 1/2 + 1/8 = 8; 1/8+1/16 = case 9

    int line1n[] ={5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0, 1,
                     5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0}; // line 1
    int line2_3n[] = {6, 4, 4, 4, 4, 4, 5, 3, 7, 4, 4, 4, 4, 4, 4, 4, 4,
                        0, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}; // line 2-3
    int line4_5n[]= {4, 4, 4, 3, 3, 1, 4, 4, 4, 4, 4, 4, 0,
                       4, 4, 4, 4, 1, 8, 4, 4, 4}; //line 4-5
    int line6n[]= {4, 4, 4, 4, 0, 0, 6, 4, 4, 4, 4, 4,
                     4, 4, 4, 6, 3, 3, 3, 3}; // line 6
    int line7n[]= {9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                     3, 3, 3, 3, 0, 4, 0, 0, 4}; // line7
    int line8n[]= {0, 1, 3, 3, 3, 3, 9, 9, 7,
                    3, 3, 3, 3, 0, 4, 9, 3, 4,
                     3, 3, 3, 3}; // line 8
    int line9n[]= {0, 4, 9, 3, 0, 4, 0, 10}; // line 9
    int line10n[] = {0, 1, 3, 3, 3, 3, 9, 9, 6,
                        3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4}; // line 10
    int line11n[] = {0, 1, 3, 3, 3, 3, 9, 9, 7,
                        3, 3, 3, 3, 0, 4, 9, 3, 4,
                        3, 3, 3, 3}; // line 11
    int line12n[]= {0, 4, 9, 3, 0, 4, 0, 10}; // line 12
    int line13n[]= {2, 9, 3, 4, 4, 1}; // line 13
    int line14n[]= {9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4,
                         0, 1, 3, 3, 3, 3}; // line 14
    int line15n[]= {9, 9, 6, 3, 3, 3, 3, 0, 4, 9, 3, 0,
                        3, 3, 3, 3}; // line 15
    int line16n[]= {0, 4, 9, 3, 0, 4, 0, 1, 3, 3, 3, 3}; // line 16
    int last_measuren[] = {0, 10}; //last measure
    int notetype[] = {5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0, 1,
                        5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0, // line 1
                        6, 4, 4, 4, 4, 4, 5, 3, 7, 4, 4, 4, 4, 4, 4, 4, 4,
                        0, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // line 2-3
                        4, 4, 4, 3, 3, 1, 4, 4, 4, 4, 4, 4, 0,
                        4, 4, 4, 4, 1, 8, 4, 4, 4, // line 4-5
                        4, 4, 4, 4, 0, 0, 6, 4, 4, 4, 4, 4,
                        4, 4, 4, 6, 3, 3, 3, 3, //line 6
                        9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4, // line 7
                        0, 1, 3, 3, 3, 3, 9, 9, 7,
                        3, 3, 3, 3, 0, 4, 9, 3, 4,
                        3, 3, 3, 3, // line 8
                        0, 4, 9, 3, 0, 4, 0, 10, // line 9
                        // repeat
                        0, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // line 2-3
                        4, 4, 4, 3, 3, 1, 4, 4, 4, 4, 4, 4, 0,
                        4, 4, 4, 4, 1, 8, 4, 4, 4, // line 4-5
                        4, 4, 4, 4, 0, 0, 6, 4, 4, 4, 4, 4,
                        4, 4, 4, 6, 3, 3, 3, 3, //line 6
                        9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4, // line 7
                        0, 1, 3, 3, 3, 3, 9, 9, 7,
                        3, 3, 3, 3, 0, 4, 9, 3, 4,
                        3, 3, 3, 3, // line 8
                        0, 4, 9, 3, 0, 4,
                        0, 1, 3, 3, 3, 3, 9, 9, 6,
                        3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4, // line 10
                        0, 1, 3, 3, 3, 3, 9, 9, 7,
                        3, 3, 3, 3, 0, 4, 9, 3, 4,
                        3, 3, 3, 3, // line 11
                        0, 4, 9, 3, 0, 4, 0, 10, //line 12
                        2, 9, 3, 4, 4, 1, // line 13
                        9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                        3, 3, 3, 3, 0, 4, 0, 0, 4,
                        0, 1, 3, 3, 3, 3, // line 14
                        9, 9, 6, 3, 3, 3, 3, 0, 4, 9, 3, 0,
                        3, 3, 3, 3, //line 15
                        0, 4, 9, 3, 0, 4, // line 16
                        0, 11, 2, 2, 2, 2, 2, 2
                        };

    int base_time = 500000000/3;
    for(int i = 0; i <= 500; i++)
    {
        switch(notetype[i]) {
                case 0: // quarter note
                    set_freq(0, song[i]);
                    nano_wait(base_time);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 1: // half note
                    set_freq(0, song[i]);
                    nano_wait(base_time * 2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 2: // whole note
                    set_freq(0, song[i]);
                    nano_wait(base_time * 4);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 3: // 16th note
                    set_freq(0, song[i]);
                    nano_wait(base_time/4);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 4: // 8th note
                    set_freq(0, song[i]);
                    nano_wait(base_time/2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 5: // 3/16th note
                    set_freq(0, song[i]);
                    nano_wait(base_time/4+ base_time/2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 6: // 1/4 + 1/8th note
                    set_freq(0, song[i]);
                    nano_wait(base_time + base_time/2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 7: // 1/2 + 1/4 + 1/8th note
                    set_freq(0, song[i]);
                    nano_wait(base_time + base_time/2 + base_time * 2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 8: // 1/2 + 1/8th note
                    set_freq(0, song[i]);
                    nano_wait(base_time/2 + base_time * 2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 9: // 1/16 + 1/8th note
                    set_freq(0, song[i]);
                    nano_wait(base_time/4 + base_time/2);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 10: // 3/4
                    set_freq(0, song[i]);
                    nano_wait(base_time*2 + base_time);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                case 11: // 1/2 + whole
                    set_freq(0, song[i]);
                    nano_wait(base_time*2 + base_time*4);
                    set_freq(0,0);
                    nano_wait(base_time);
                    break;
                }

    }

    set_freq(0,0);
    nano_wait(80000000);
}


