#include "stm32f0xx.h"
#include <stdint.h>
#include <stdlib.h>
#include "lcd.h"
#include <math.h>   // for M_PI
#include <stdlib.h>

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

//#define Song
#if defined(Song)
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

//int main(void)
//{
//    enable_ports();
//    setup_dma();
//    enable_dma();
//    init_tim15();
//    setup_adc();
//    init_tim2();
//    init_wavetable();
//    setup_dac();
//    init_tim6();
//
//
//
////    float line1s[] = {c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4, r,
////                    c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4}; // line 1
////    float line2_3s[] = {r, b4, c5s, d5, d5, e5, c5s, b4, a4, r, b4, b4, c5s, d5, b4, r, a4,
////                    a5, a5, e5, f5s, e5, d5, r, a4, a4, c5s, d5, b4, d5, e5}; // line 2-3 rep
////    float line4_5s[] = {r, c5s, b4, c5s, b4, a4, r, b4, b4, c5s, d5, b4, a4,
////                    e5, e5, e5, f5s, e5, d5, e5, f5s, d5}; // line 4-5
////    float line6s[] = {e5, e5, e5, f5s, e5, a4, r, a4, b4, c5s, d5, b4,
////                    r, e5, f5s, e5, a4, b4, d5, b4}; // line 6
////    float line7s[] = {f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
////                    a4, b4, d5, b4, d5, e5, c5s, a4, a4}; //line 7
////    float line8s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
////                    a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
////                    a4, b4, d5, b4}; // line 8
////    float line9s[] = {d5, e5, c5s, b4, a4, a4, /* line 10*/ e5, d5 /*back to line 2*/}; // line 9
////    float line10s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
////                    a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
////                    a4, b4, d5, b4, d5, e5, c5s, a4, a4}; // line 10
////    float line11s[] = {e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
////                    a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
////                    a4, b4, d5, b4}; // line 11
////    float line12s[] = {d5, e5, c5s, b4, a4, a4, e5, d5}; // line 12
////    float line13s[] = {a4, d5, d5, r, c5s, r}; // line 13
////    float line14s[] = {f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
////                        a4, b4, d5, b4, d5, e5, c5s, a4, a4,
////                        e5, d5, a4, b4, d5, b4}; // line 14
////    float line15s[] = {f5s, f5s, e5, a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
////                    a4, b4, d5, b4}; // line 15
////    float line16s[] = {d5, e5, c5s, b4, a4, a4, e5, d5, a4, b4, d5, b4}; // line 16
////    float last_measures[] = {e5, d5}; // last measure
//
//
//    // 1/4 = 0; 1/2 = 1; whole = 2; 1/16 = 3 ; 1/8 = 4; 3/16 =5; 1/4 + 1/8 = 6; 1/2 + 1/4 + 1/8 = 7; 1/2 + 1/8 = 8; 1/8+1/16 = case 9
//
//    /*int line1n[] ={5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0, 1,
//                     5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0}; // line 1
//    int line2_3n[] = {6, 4, 4, 4, 4, 4, 5, 3, 7, 4, 4, 4, 4, 4, 4, 4, 4,
//                        0, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}; // line 2-3
//    int line4_5n[]= {4, 4, 4, 3, 3, 1, 4, 4, 4, 4, 4, 4, 0,
//                       4, 4, 4, 4, 1, 8, 4, 4, 4}; //line 4-5
//    int line6n[]= {4, 4, 4, 4, 0, 0, 6, 4, 4, 4, 4, 4,
//                     4, 4, 4, 6, 3, 3, 3, 3}; // line 6
//    int line7n[]= {9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
//                     3, 3, 3, 3, 0, 4, 0, 0, 4}; // line7
//    int line8n[]= {0, 1, 3, 3, 3, 3, 9, 9, 7,
//                    3, 3, 3, 3, 0, 4, 9, 3, 4,
//                     3, 3, 3, 3}; // line 8
//    int line9n[]= {0, 4, 9, 3, 0, 4, 0, 10}; // line 9
//    int line10n[] = {0, 1, 3, 3, 3, 3, 9, 9, 6,
//                        3, 3, 3, 3, 9, 9, 9, 3, 4,
//                        3, 3, 3, 3, 0, 4, 0, 0, 4}; // line 10
//    int line11n[] = {0, 1, 3, 3, 3, 3, 9, 9, 7,
//                        3, 3, 3, 3, 0, 4, 9, 3, 4,
//                        3, 3, 3, 3}; // line 11
//    int line12n[]= {0, 4, 9, 3, 0, 4, 0, 10}; // line 12
//    int line13n[]= {2, 9, 3, 4, 4, 1}; // line 13
//    int line14n[]= {9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
//                        3, 3, 3, 3, 0, 4, 0, 0, 4,
//                         0, 1, 3, 3, 3, 3}; // line 14
//    int line15n[]= {9, 9, 6, 3, 3, 3, 3, 0, 4, 9, 3, 0,
//                        3, 3, 3, 3}; // line 15
//    int line16n[]= {0, 4, 9, 3, 0, 4, 0, 1, 3, 3, 3, 3}; // line 16
//    int last_measuren[] = {0, 10}; //last measure*/
//
//
//    int base_time = 500000000/3;
//    for(int i = 0; i <= 500; i++)
//    {
//        switch(notetype[i]) {
//                case 0: // quarter note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 1: // half note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time * 2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 2: // whole note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time * 4);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 3: // 16th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time/4);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 4: // 8th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time/2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 5: // 3/16th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time/4+ base_time/2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 6: // 1/4 + 1/8th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time + base_time/2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 7: // 1/2 + 1/4 + 1/8th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time + base_time/2 + base_time * 2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 8: // 1/2 + 1/8th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time/2 + base_time * 2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 9: // 1/16 + 1/8th note
//                    set_freq(0, song[i]);
//                    nano_wait(base_time/4 + base_time/2);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 10: // 3/4
//                    set_freq(0, song[i]);
//                    nano_wait(base_time*2 + base_time);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                case 11: // 1/2 + whole
//                    set_freq(0, song[i]);
//                    nano_wait(base_time*2 + base_time*4);
//                    set_freq(0,0);
//                    nano_wait(base_time);
//                    break;
//                }
//
//    }
//
//    set_freq(0,0);
//    nano_wait(80000000);
//}


#endif



#define LCD
#if defined(LCD)
#include "lcd.h"

//fix syntax and add constants for columns
#define BASEDELAY 50000000/2.5
#define SCREENLENGTH 320
#define BLUETOP1 225
#define BLUETOP2 226
#define BLUEBOTTOM1 275
#define BLUEBOTTOM2 276
#define COLUMN_LENGTH 29
#define BOXSIZE 20

//320 is max speed for 1 box on screen at time
//320 can be easy mode and smaller input and more on screen can be harder but not as fast
#define NOTESPACE 320
#define NOTSET 255

struct DISPLAY_NOTE
{
    int color; //not set
    int noteType;
    float freq;
    int displayCol;
};

// sharp c, d, f, g
//cleanup to CAPITAL LETTERS
#define a3  220.00f
#define a4  440.00f
#define a5  880.00f

#define b3  246.94f
#define b4  493.88f

#define c4  261.63f
#define c5  523.25f
#define c5s  554.37f

#define d4  293.66f
#define d4s  311.13f
#define d5  587.33f
#define d5s  622.25f

#define e4  329.63f
#define e5 659.25f

#define f4  349.23f
#define f5  689.46f
#define f5s  739.99f

#define g3  196.00f
#define g4  392.00f
#define g4s  415.30f
#define g5s  830.61f

#define r  0.00f


float noteFreq[] = {c5s, d5s, g4s, d5s, f5, g5s, f5s, f5, d5s, c5s, d5s, g4s, d4s, f4, r,
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

int noteType[] = {5, 6, 0, 5, 6, 3, 3, 3, 3, 5, 6, 0, 5, 0, 1,
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


void init_lcd_spi(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(0x30c30000);
    GPIOB->MODER |= 0x10410000;

    GPIOB->ODR |= 1<<8;
    GPIOB->ODR |= 1<<11;
    GPIOB->ODR |= 1<<14;

    //alternate function 0
    GPIOB->MODER &= ~(0x00000cc0);
    GPIOB->MODER |= 0x00000880;
    GPIOB->AFR[0] &= ~(0x00f0f000);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 &= ~SPI_CR2_DS;
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;

    return;
}

void setup_buttons(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
    return;
}

//get column positions
int getColumnStartX(int col)
{
    return 30 * col + col;
}
//get column positions
int getColumnEndX(int col)
{
    return 30 * col + col + COLUMN_LENGTH;
}

//void copyNote(struct DISPLAY_NOTE source, struct DISPLAY_NOTE *destination)
//{
//    destination->color = source.color;
//    destination->displayCol = source.displayCol;
//    destination->freq = source.freq;
//    destination->noteType = source.noteType;
//
//    return;
//}

//taking next element and moving screen array down and adding new line at top
//can be made faster
void addToScreen(int screen[], int addValue)
{
    for (int s = SCREENLENGTH-1; s > 0; s--)
    {
        screen[s] = screen[s-1];
    }

    screen[0] = addValue;
    return;
}

//check for inputs and add as parameter
//drawing on screen if changed; if drawing for red it creates rectangle; if clearing to gray it only draws top line of rectangle
void drawScreen(int screen[], int prevScreen[])
{
    for(int s = 0; s < SCREENLENGTH; s++)
    {
        if(screen[s] != prevScreen[s])
        {
            int displayCol = screen[s] == NOTSET ? prevScreen[s] : screen[s];
            int color = screen[s] == NOTSET ? LIGHTGRAY : RED;
            if(color == LIGHTGRAY)
            {

                switch(s)
                {
                    case BLUETOP1:
                    case BLUETOP2:
                    case BLUEBOTTOM1:
                    case BLUEBOTTOM2:
                        color = BLUE;
                        break;
                }
                LCD_DrawLine(getColumnStartX(displayCol), s, getColumnEndX(displayCol), s, color);
            }
            else
            {
                //check for valid input from player
                //LCD_DrawFillRectangle(getColumnStartX(displayCol), s, getColumnEndX(displayCol), s+BOXSIZE, color);
                LCD_DrawLine(getColumnStartX(displayCol), s, getColumnEndX(displayCol), s, color);
            }

            //setting previous screen so it will match to get rid of copyScreen call for speed
            prevScreen[s] = screen[s];
        }

    }
}

//keeping track of everything in screen that is changed
void copyScreen(int screen[], int prevScreen[])
{
    for(int s = 0; s < SCREENLENGTH; s++)
    {
        if(screen[s] != prevScreen[s])
        {
            prevScreen[s] = screen[s];
        }
    }
}

//add parameters+logic for any of the 8 columns and turn column into x,y position for square
void fallingSquares(void)
{
    //int songSize = (sizeof(noteFreq) / sizeof(noteFreq[0])) * (BOXSIZE + NOTESPACE);
    int songSize = (sizeof(noteFreq) / sizeof(noteFreq[0]));
    char song[songSize];
    int screen[SCREENLENGTH];
    int prevScreen[SCREENLENGTH];

    for(int i = 0; i < songSize; i++)
    {
        if(noteFreq[i] == r)
        {
           song[i] = NOTSET;
        }
        else
        {
            int randCol = rand() % 8 + 0;
            song[i] = randCol;
        }

    }

    for(int i = 0; i < SCREENLENGTH; i++)
    {
        screen[i] = NOTSET; //screen is 0-320 out of full song array and moving down the screen
    }
    copyScreen(screen, prevScreen);

    for(int i = 0; i < songSize; i++)
    {
        //check inputs and pass into drawScreen
        //button function would pass NOTSET for no buttons and 0-7 for column buttons
        addToScreen(screen, song[i]);

        //buttons passed in
        drawScreen(screen, prevScreen);
        //will need to check through song array for y value between blue lines for column button was hit
        //to see if the song value is the same value as buttons. otherwise play shorter length of note
        //playing correct note can change color to show correct or error around the input area for right/wrong answer
        //change boxes to neutral color so right and wrong can be green/red

        //copyScreen(screen, prevScreen);
        //nano_wait(BASEDELAY/5);


        //draw amount of lines for box size
        for(int b = 1; b < BOXSIZE; b++)
        {
            addToScreen(screen, song[i]);

            //buttons passed in
            drawScreen(screen, prevScreen);
        }

        //draw the spaces between notes and go to next note
        for(int space = 0; space < NOTESPACE; space++)
        {
            //check inputs and pass into drawScreen
            //button function would pass NOTSET for no buttons and 0-7 for column buttons
            addToScreen(screen, NOTSET);

            //buttons passed in
            drawScreen(screen, prevScreen);
            //will need to check through song array for y value between blue lines for column button was hit
            //to see if the song value is the same value as buttons. otherwise play shorter length of note
            //playing correct note can change color to show correct or error around the input area for right/wrong answer
            //change boxes to neutral color so right and wrong can be green/red

            //copyScreen(screen, prevScreen);

            //nano_wait(BASEDELAY/5);
        }

    }

}

//get background onto LCD;
//Fix the Blue lines being overwritten

int main(void)
{
    //setup_buttons();
    LCD_Setup(); // this will call init_lcd_spi()
    LCD_DrawFillRectangle(0, 0, 240, 320, LIGHTGRAY);
    //note lines
    LCD_DrawLine(0, 225, 240,225, BLUE);
    LCD_DrawLine(0, 275, 240, 275, BLUE);
    LCD_DrawLine(0, 226, 240,226, BLUE);
    LCD_DrawLine(0, 276, 240, 276, BLUE);
    //columns
    LCD_DrawLine(30, 0, 30, 320, GRAY);
    LCD_DrawLine(61, 0, 61, 320, GRAY);
    LCD_DrawLine(92, 0, 92, 320, GRAY);
    LCD_DrawLine(123, 0, 123, 320, GRAY);
    LCD_DrawLine(154, 0, 154, 320, GRAY);
    LCD_DrawLine(185, 0, 185, 320, GRAY);
    LCD_DrawLine(216, 0, 216, 320, GRAY);
    //hardcode color for last few x pixels to be even

    //for playing music: make input area odd for exact middle; when moving note hits
    //middle check other array for those spots in music like hardcoded example above to
    //play the correct note/length and if wrong can be played shorter
    //at exact middle call setfreq to play note and then stop freq based on duration
    //don't want to just play note and not drawing the squares

    //difficulties could have tighter input range,smaller squares, smaller nanowaits
    fallingSquares();

}

#endif
