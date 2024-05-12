#include "stm32f0xx.h"
#include "lcd.h"

void init_lcd_spi(void)
{
    //test time 
    /*
    The function init_lcd_spi() initializes the SPI (Serial Peripheral Interface) communication for the LCD. It configures the required GPIO pins (GPIOB pins 8, 11, and 14) as outputs 
    and sets their corresponding alternate function registers (AFR). It enables the clock for GPIOB and SPI1 peripherals using the RCC (Reset and Clock Control) registers. It configures SPI1 as the master
     device and sets the clock division ratio (BR). It also configures the data size (DS) as 8 bits and enables the SPI1 peripheral.
    */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(0x30C30CC0);
    GPIOB->MODER |= 0x10410880;
    GPIOB->ODR |= (1<<8) | (1<<11)| (1<<14);
    GPIOB->AFR[0] &= ~(0xF0F000);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~(SPI_CR1_SPE);
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR;
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void setup_buttons(void)
{
    /*
    The function setup_buttons() configures buttons or switches connected to GPIOC pins for input. 
    It enables the clock for GPIOC peripheral using the RCC registers. It sets the corresponding GPIOC pins (0-7) as inputs and configures them with pull-up resistors.
    */
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~(0xFF00);
    GPIOC->MODER |= 0x5500;
    GPIOC->OTYPER |= 0xF0;

    GPIOC->MODER &= ~(0xFF);
    GPIOC->PUPDR &= ~(0xFF);
    GPIOC->PUPDR |= 0x55;
}

void basic_drawing(void);
void move_ball(void);

int main(void)
{
    /*The function main() is the entry point of the program. It calls setup_buttons() to configure the buttons and then calls LCD_Setup() function (presumably from the LCD library) 
    to initialize the LCD using the init_lcd_spi() function. It also calls basic_drawing()
     and move_ball() functions, but it seems that the latter is currently commented out.*/
    setup_buttons();
    LCD_Setup(); // this will call init_lcd_spi()
    basic_drawing();
    //move_ball();
}
