#include "stm32f0xx.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#include <math.h>   // for M_PI


#define N 1000
#define RATE 20000

//fix syntax and add constants for columns
#define BASEDELAY 50000000/2.5

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


//320 is max speed for 1 box on screen at time
//320 can be easy mode and smaller input and more on screen can be harder but not as fast
#define NOTSET 255
#define SCREENLENGTH 320
#define BLUETOP1 225
#define BLUETOP2 226
#define BLUEBOTTOM1 275
#define BLUEBOTTOM2 276
#define COLUMN_LENGTH 29
#define NOTE_PLAY 251


//#define USE_NOTE_LENGTH //can be switched to use varying boxsize

struct DISPLAY_NOTE
{
    int color; //not set
    int noteType;
    float freq;
    int displayCol;
};

struct NODE
{
    int screenLocation;
    char column;
    int songIndex;
    int note;
    unsigned int waitTime;
    struct NODE *next;
};

struct NODE *head = NULL;
struct NODE *current = NULL;
int playCount = 0;
int noteLength = -1;
int playingNote = -1; //if notePlaying note playing from that column
char inputSuccess = -1;
char score[10];
char score_acc_array[5];
int scoreValue = 0;

int NOTESPACE; //space in between the notes on screen: 320 greatest
int BOXSIZE; //size of box
int  BASETIMENOTE; //base number of lines note is played
int  NUMBOXES; //# of columns used
float score_acc = 0.00; //accumulated score
int easyMode = 0; //easyMode

// 1/4 = 0; 1/2 = 1; whole = 2; 1/16 = 3 ; 1/8 = 4; 3/16 =5; 1/4 + 1/8 = 6; 1/2 + 1/4 + 1/8 = 7; 1/2 + 1/8 = 8; 1/8+1/16 = case 9
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
        d5, e5, c5s, b4, a4, a4, /* line 10*/ e5, d5,r, r, /*back to line 2*/ // line 9
        //repeat
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
        d5, e5, c5s, b4, a4, a4,
        e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
        a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
        a4, b4, d5, b4, d5, e5, c5s, a4, a4, // line 10
        e5, d5, a4, b4, d5, b4, f5s, f5s, e5,
        a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
        a4, b4, d5, b4, // line 11
        d5, e5, c5s, b4, a4, a4, e5, d5, // line 12
        a4, b4, c5s, a4,
//        a4, d5, d5, c5s, r, // line 13

        f5s, f5s, e5, a4, b4, c5s, a4, e5, e5, d5, c5s, b4,
        a4, d5, d5, b4, d5, e5, c5s, a4, a4,
        e5, d5, a4, b4, d5, b4, // line 14
        f5s, f5s, e5, a4, b4, c5s, a4, a5, c5s, d5, c5s, b4,
        a4, b4, d5, b4, // line 15
        d5, e5, c5s, b4, a4, a4, //line 16
        e5, d5,r,r,r,r,r,r // last measure
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
                    0, 4, 9, 3, 0, 4, 0, 10, r, r,// line 9
                    // repeat
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
                    0, 4, 9, 3, 0, 4,
                    0, 1, 3, 3, 3, 3, 9, 9, 6,
                    3, 3, 3, 3, 9, 9, 9, 3, 4,
                    3, 3, 3, 3, 0, 4, 0, 0, 4, // line 10
                    0, 1, 3, 3, 3, 3, 9, 9, 7,
                    3, 3, 3, 3, 0, 4, 9, 3, 4,
                    3, 3, 3, 3, // line 11
                    0, 4, 9, 3, 0, 4, 0, 10,//line 12

                    3,3,3,3,
 //                   2, 9, 3, 4, 1, // line 13
                    9, 9, 6, 3, 3, 3, 3, 9, 9, 9, 3, 4,
                    3, 3, 3, 3, 0, 4, 0, 0, 4,
                    0, 1, 3, 3, 3, 3, // line 14
                    9, 9, 6, 3, 3, 3, 3, 0, 4, 9, 3, 0,
                    3, 3, 3, 3, //line 15
                    0, 4, 9, 3, 0, 4, // line 16
                    0, 11, 3, 3,3,3,3,3
                    };

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

#define Song
#if defined(Song)


//=============================================================================
// Part 1: 7-segment display update with DMA
//=============================================================================

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };

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


#endif




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

void init_lcd_spi(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(0x00cf0000);
    GPIOB->MODER |= 0x00450000;

    GPIOB->ODR |= 1<<9;
    GPIOB->ODR |= 1<<11;
    GPIOB->ODR |= 1<<8;

    //alternate function 0
    GPIOB->MODER &= ~(0xcc000000);
    GPIOB->MODER |= 0x88000000;
    GPIOB->AFR[1] &= ~(0xf0f00000);

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 &= ~SPI_CR1_BR;
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR2 &= ~SPI_CR2_DS;
    SPI2->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI2->CR1 |= SPI_CR1_SSM;
    SPI2->CR1 |= SPI_CR1_SSI;
    SPI2->CR1 |= SPI_CR1_SPE;

    return;
}


//spi1 for OLED Score
void init_spi1()
{
    // PA5  SPI1_SCK
    // PA6  SPI1_MISO
    // PA7  SPI1_MOSI
    // PA15 SPI1_NSS
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0xc000fc00);
    GPIOA->MODER |= 0x8000a800;
    GPIOA->AFR[0] &= ~(0xfff00000);
    GPIOA->AFR[1] &= ~(0xf0000000);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_3;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;

    return;
}

void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
void spi1_display2(char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

void setup_buttons(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    return;
}

void intro(void)
{
    spi1_display1("RICK'S ROLL");
    do
    {
        spi1_display2("LEFTMOST = START");
    } while((GPIOC->IDR &= GPIO_IDR_0) != 1);

    nano_wait(500000000);

    do
    {
        spi1_display1("LEFTMOST = EASY ");
        spi1_display2("RIGHTMOST = HARD ");
    } while ((GPIOC->IDR &= GPIO_IDR_0) == 0 && (GPIOC->IDR &= GPIO_IDR_7) == 0);

    if(GPIOC->IDR &= GPIO_IDR_0)
    {
        spi1_display1("EASY MODE         ");
        spi1_display2("                  ");

        easyMode = 1;
        NOTESPACE = 50; //space in between the notes on screen: 320 greatest
        BOXSIZE = 14; //size of box
        BASETIMENOTE = 40; //base number of lines note is played
        NUMBOXES = 4; //# of columns used

    }
    if (GPIOC->IDR &= GPIO_IDR_7)
    {
        spi1_display1("HARD MODE         ");
        spi1_display2("GOOD LUCK         ");

        NOTESPACE = 70; //space in between the notes on screen: 320 greatest
        BOXSIZE = 14; //size of box
        BASETIMENOTE = 40; //base number of lines note is played
        NUMBOXES = 8; //# of columns used

    }

}

void insert(int screenLocation, char column, int songIndex)
{
    struct NODE *newNode = (struct NODE*) malloc(sizeof(struct NODE));
    unsigned int base_time = 500000000/2.5;



    //CHANGE TO BASEWAIT TIME OFF OF CLOCK
    unsigned int waitTime = 0;

    newNode->screenLocation = screenLocation;
    newNode->column = column;
    newNode->songIndex = songIndex;

    int maxNoteLength = 0;
    switch(noteType[songIndex])
    {
        case 0:
            waitTime = base_time;
//            maxNoteLength = 4;
            maxNoteLength = BASETIMENOTE;
            break;
        case 1:
            waitTime = base_time * 2;
//            maxNoteLength = 8;
            maxNoteLength = BASETIMENOTE * 2;
            break;
        case 2:
            waitTime = base_time * 4;
//            maxNoteLength = 16;
            maxNoteLength = BASETIMENOTE * 4;
            break;
        case 3:
            waitTime = base_time/4;
            //maxNoteLength = 1;
            maxNoteLength = BASETIMENOTE / 4;
            break;
        case 4:
            waitTime = base_time/2;
            //maxNoteLength = 2;
            maxNoteLength = BASETIMENOTE / 2;
            break;
        case 5:
            waitTime = (base_time/4+ base_time/2);
            //maxNoteLength = 3;
            maxNoteLength = BASETIMENOTE/4 + BASETIMENOTE/2;
            break;
        case 6:
            waitTime = (base_time + base_time/2);
            //maxNoteLength = 6;
            maxNoteLength = BASETIMENOTE + BASETIMENOTE/2;
            break;
        case 7:
            waitTime = (base_time + base_time/2 + base_time * 2);
            //maxNoteLength = 14;
            maxNoteLength = BASETIMENOTE + BASETIMENOTE/2 + BASETIMENOTE*2;
            break;
        case 8:
            waitTime = base_time/2 + base_time * 2;
            //maxNoteLength = 10;
            maxNoteLength = BASETIMENOTE/2 + BASETIMENOTE*2;
            break;
        case 9:
            waitTime = base_time/4 + base_time/2;
            //maxNoteLength = 3;
            maxNoteLength = BASETIMENOTE/4 + BASETIMENOTE/2;
            break;
        case 10:
            waitTime = base_time*2 + base_time;
            //maxNoteLength = 12;
            maxNoteLength = BASETIMENOTE*2 + BASETIMENOTE;
            break;
        case 11:
            waitTime = base_time*2 + base_time*4;
            //maxNoteLength = 24;
            maxNoteLength = BASETIMENOTE*2 + BASETIMENOTE*4;
            break;
        default:
            waitTime = base_time;
            //maxNoteLength = 0;
            maxNoteLength = BASETIMENOTE;

    }
    newNode->waitTime = waitTime;
    newNode->note = maxNoteLength * 2;
    newNode->next = NULL;
    if(head == NULL)
    {
        head = newNode;
    }
    else
    {
        struct NODE *temp = head;
        while(temp->next != NULL)
        {
            temp = temp->next;
        }
        temp->next = newNode;
    }

    return;
}

void deleteHead()
{
    struct NODE *temp = head;
    head = head->next;

    free(temp);
}

//get column positions
int getColumnStartX(int col)
{
    return 30 * col + col + 8;
}
//get column positions
int getColumnEndX(int col)
{
    return 30 * col + col + COLUMN_LENGTH - 8;
}

void successNote(int col)
{
    drawLine(col, getColumnStartX(col), NOTE_PLAY, getColumnEndX(col), NOTE_PLAY, GREEN);
    scoreValue++;
    score_acc = scoreValue / 4;
    sprintf(score, "%d", scoreValue / 4);
    spi1_display2(score);
}

void failedNote(int col)
{
    if(col <= 7)
    {

        drawLine(col, getColumnStartX(col), NOTE_PLAY, getColumnEndX(col), NOTE_PLAY, RED);
        set_freq(0,0);
    }
    //used to stop the frequency on failure

}


void drawLine(int col, u16 x1, u16 y1, u16 x2, u16 y2, u16 c)
{
    if(y1 >= 0 && y1 <= SCREENLENGTH)
    {
        switch(y1)
        {
            case BLUETOP1: //225
            case BLUETOP2: //226
                playingNote = col;
                inputSuccess = -1;

                break;
            case BLUEBOTTOM1: //275
            case BLUEBOTTOM2: //276
                if(playingNote != -1 && playingNote != col)
                {
                    playingNote = -1;
                    inputSuccess = -1;
                }
                if(inputSuccess == col && inputSuccess != 255)
                {
                    //successful hit
                    successNote(col);
                }
                else
                {
                    //failed to play note
                    failedNote(col);
                }

                //inputSuccess = -1;
                playingNote = -1;
                break;
            default:
                //LCD_DrawLine(x1, y1, x2, y2, c);
                for(int x = x1; x < x2; x++)
                {
                    LCD_DrawPoint(x, y1, c);
                }
        }
    }
    if(playingNote != -1)
    {

        //can tweak to decide rule boundaries for if multi buttons can be hit


        //flickering green: need to fix
        int temp = 1 << col;
        int reg = GPIOC->IDR;
        int input = reg & temp;
        if(input)
        {
            inputSuccess = col;
        }
    }

    return;
}

int getNodeLength()
{
    int length = 0;
    struct NODE *temp = head;
    while(temp != NULL)
    {
        length++;
        temp = temp->next;

    }
    return length;
}


//check for inputs and add as parameter
//drawing on screen if changed; if drawing for red it creates rectangle; if clearing to gray it only draws top line of rectangle
void drawScreen()
{
    struct NODE *temp = head;

    while(temp != NULL)
    {
        if(noteFreq[temp->songIndex] != r)
        {
            switch(temp->column)
            {
                case 0:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, RED);
                    break;
                case 1:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, WHITE);
                    break;
                case 2:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, BLUE);
                    break;
                case 3:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, RED);
                    break;
                case 4:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, WHITE);
                    break;
                case 5:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, BLUE);
                    break;
                case 6:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, RED);
                    break;
                case 7:
                    drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, WHITE);
                    break;

            }
            //drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation, getColumnEndX(temp->column), temp->screenLocation, BLUE);
        }


        #if defined(USE_NOTE_LENGTH)
        if(temp->screenLocation > temp->note)
        {
            drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation - temp->note, getColumnEndX(temp->column), temp->screenLocation - temp->note, LIGHTGRAY);

        }
        #else
        if(temp->screenLocation > BOXSIZE)
        {
            if(temp->column <= 7)
            {
                drawLine(temp->column, getColumnStartX(temp->column), temp->screenLocation - BOXSIZE, getColumnEndX(temp->column), temp->screenLocation - BOXSIZE, BLACK);

            }
        }
        #endif

        if(temp->screenLocation == NOTE_PLAY)
        {
            set_freq(0,0);
            set_freq(0, noteFreq[temp->songIndex]);
            //nano_wait(temp->waitTime);
            playCount = 1;
            noteLength = temp->note;
        }

        temp->screenLocation++;

        temp = temp->next;
    }

    if(playCount > 0)
    {
        //if(playCount > (100 * 24 / (24 - noteLength + 1)))
        if(playCount > noteLength)
        {
            set_freq(0,0);
            playCount = 0;
        }
        else
        {
            playCount++;
        }
    }

#if defined(USE_NOTE_LENGTH)
    if(head->screenLocation >= (SCREENLENGTH + head->note))
    {
        int temp = getNodeLength();
        deleteHead();
    }
#else
    if(head->screenLocation >= (SCREENLENGTH + BOXSIZE))
    {
        deleteHead();
    }
#endif

    return;
}

//add parameters+logic for any of the 8 columns and turn column into x,y position for square
void fallingSquares(void)
{
    int songSize = (sizeof(noteFreq) / sizeof(noteFreq[0]));
    char song[songSize];

    for(int i = 0; i < songSize; i++)
    {
        // initialize song array based off of note array
        if(noteFreq[i] == r)
        {
           song[i] = NOTSET;
        }
        else
        {
            int randCol = rand() % NUMBOXES + 0;
            song[i] = randCol;
        }
    }

    for(int i = 0; i < songSize; i++)
    {

        insert(0, song[i], i);
        //draw the spaces between notes and go to next note
        for(int space = 0; space < (NOTESPACE + head->note); space++)
        {
            if(head->column >= 8 && head->column != 255)
            {
               break;
            }
            drawScreen();
            if(easyMode)
            {
                nano_wait(BASEDELAY/30);
            }

        }

    }

    //might need to add loop to get last note off screen by repeatedly calling drawscreen
    for(int i = 0; i < 20; i++)
    {
        drawScreen();
    }


}



int main(void)
{


    for(;;)
    {
        //difficulties could have tighter input range,smaller squares, smaller nanowaits
        setup_buttons();
        setup_dma();
        enable_dma();
        setup_adc();
        init_tim2();
        init_wavetable();
        setup_dac();
        init_tim6();


        init_spi1();
        spi1_init_oled();

        LCD_Setup(); // this will call init_lcd_spi()
        LCD_DrawFillRectangle(0, 0, 240, 320, BLACK);
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

        intro(); //for start screen and selection for difficulty

        nano_wait(2000000000);

        spi1_display1("SCORE:       ");
        spi1_display2("0            ");

        fallingSquares(); //main game loop call

        nano_wait(2000000000);

        float score_percent = (score_acc / 370) * 100;

        sprintf(score_acc_array, "%d", (int) score_percent);
        spi1_display1("Good Job!");
        spi1_display2("         ");
        nano_wait(2000000000);
        //nano_wait(2000000000);
        spi1_display1("Accuracy % = ");
        spi1_display2(score_acc_array);

        nano_wait(2000000000);

        scoreValue = 0;

        do
        {
            spi1_display1("NEW GAME?      ");
            spi1_display2("PRESS LEFT MOST ");
        } while((GPIOC->IDR &= GPIO_IDR_0) != 1);

    }

}

