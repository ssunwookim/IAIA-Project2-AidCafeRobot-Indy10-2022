/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : KIM SUNWOO LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<( pin *2 ));
	Port->OSPEEDR |=   speed<<(pin *2);	
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
   Port->OTYPER &= ~(1UL<< pin);
	 Port->OTYPER |=	type << pin; 
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
   	Port->PUPDR &= ~(3UL<<( pin *2));					  // 00: none
		Port->PUPDR |=   pupd<<(pin *2);
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
	int val = (Port->IDR)>>pin & 1;
	return val;    	
}

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL<<pin);
	Port->ODR |= Output<<pin;
}

void sevensegment_init(){
	
  GPIO_init(GPIOB, 9, OUTPUT);
	GPIO_otype(GPIOB, 9, 0);
	GPIO_pupd(GPIOB, 9, EC_NO);
	GPIO_ospeed(GPIOB,9, EC_MEDIUM);	
	
  GPIO_init(GPIOA, 6, OUTPUT); 
	GPIO_otype(GPIOA, 6, 0);
	GPIO_pupd(GPIOA, 6, EC_NO);
	GPIO_ospeed(GPIOA, 6, EC_MEDIUM);
	
  GPIO_init(GPIOA, 7, OUTPUT);       
	GPIO_otype(GPIOA, 7, 0);
	GPIO_pupd(GPIOA, 7, EC_NO);
	GPIO_ospeed(GPIOA, 7, EC_MEDIUM);	
  
  GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_otype(GPIOB, 6, 0);
	GPIO_pupd(GPIOB, 6, EC_NO);
	GPIO_ospeed(GPIOB, 6, EC_MEDIUM);	
	
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_otype(GPIOC, 7, 0);
	GPIO_pupd(GPIOC, 7, EC_NO);
	GPIO_ospeed(GPIOC, 7, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 9, OUTPUT);       
	GPIO_otype(GPIOA, 9, 0);
	GPIO_pupd(GPIOA, 9, EC_NO);
	GPIO_ospeed(GPIOA, 9, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 8, OUTPUT);       
	GPIO_otype(GPIOA, 8, 0);
	GPIO_pupd(GPIOA, 8, EC_NO);
	GPIO_ospeed(GPIOA, 8, EC_MEDIUM);	
	
	GPIO_init(GPIOB, 10, OUTPUT);
	GPIO_otype(GPIOB, 10, 0);
	GPIO_pupd(GPIOB, 10, EC_NO);
	GPIO_ospeed(GPIOB, 10, EC_MEDIUM);	
}

void sevensegment_decoder(uint8_t  num){
	 
	int ledState[10][8]={
	
	//LED              a b c d e f g dot
                    {0,0,0,0,0,0,1,0},          //zero
                    {1,0,0,1,1,1,1,0},          //one
                    {0,0,1,0,0,1,0,0},          //two
                    {0,0,0,0,1,1,0,0},          //three
                    {1,0,0,1,1,0,0,0},          //four
                    {0,1,0,0,1,0,0,0},          //five
                    {0,1,0,0,0,0,0,0},          //six
                    {0,0,0,1,1,0,1,0},          //seven
                    {0,0,0,0,0,0,0,0},          //eight
                    {0,0,0,0,1,0,0,0},          //nine
									};
		
		GPIO_write(GPIOA, 5, ledState[num][0]); 	//LED a
		GPIO_write(GPIOA, 6, ledState[num][1]); 	//LED b
		GPIO_write(GPIOA, 7, ledState[num][2]); 	//LED c
		GPIO_write(GPIOB, 6, ledState[num][3]); 	//LED d
		GPIO_write(GPIOC, 7, ledState[num][4]); 	//LED e
		GPIO_write(GPIOA, 9, ledState[num][5]); 	//LED f
		GPIO_write(GPIOA, 8, ledState[num][6]); 	//LED g
		GPIO_write(GPIOB, 10, ledState[num][7]);	//LED dot
}

void sevensegment_display_init(){
	
	GPIO_init(GPIOA, 7, OUTPUT);       
	GPIO_otype(GPIOA, 7, EC_OUT_PU);
	GPIO_pupd(GPIOA, 7, EC_NO);
	GPIO_ospeed(GPIOA, 7, EC_MEDIUM);	
  
  GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_otype(GPIOB, 6, 0);
	GPIO_pupd(GPIOB, 6, EC_NO);
	GPIO_ospeed(GPIOB, 6, EC_MEDIUM);	
	
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_otype(GPIOC, 7, 0);
	GPIO_pupd(GPIOC, 7, EC_NO);
	GPIO_ospeed(GPIOC, 7, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 9, OUTPUT);       
	GPIO_otype(GPIOA, 9, 0);
	GPIO_pupd(GPIOA, 9, EC_NO);
	GPIO_ospeed(GPIOA, 9, EC_MEDIUM);	

}

void sevensegment_display(uint8_t  num){
	int number[10][4] = {
		//                  A7 B6 C7 A9
		//                   D C B A
												{0,0,0,0},          //zero
												{0,0,0,1},          //one
												{0,0,1,0},          //two
												{0,0,1,1},          //three
												{0,1,0,0},          //four
												{0,1,0,1},          //five
												{0,1,1,0},          //six
												{0,1,1,1},          //seven
												{1,0,0,0},          //eight
												{1,0,0,1},          //nine
											};
	
		GPIO_write(GPIOA, 7, number[num][0]); 	//binary 2^3
		GPIO_write(GPIOB, 6, number[num][1]); 	//binary 2^2
		GPIO_write(GPIOC, 7, number[num][2]); 	//binary 2^1
		GPIO_write(GPIOA, 9, number[num][3]); 	//binary 2^0
}

void LED_toggle(void){
	GPIOA->ODR ^= 1<<5;
}

void LED_binary(uint8_t  num){
	int number[16][4] = {
		//                  A7 B6 C7 A9
		//                   D C B A
												{0,0,0,0},          //zero
												{0,0,0,1},          //one
												{0,0,1,0},          //two
												{0,0,1,1},          //three
												{0,1,0,0},          //four
												{0,1,0,1},          //five
												{0,1,1,0},          //six
												{0,1,1,1},          //seven
												{1,0,0,0},          //eight
												{1,0,0,1},          //nine
												{1,0,1,0},
												{1,0,1,1},
												{1,1,0,0},
												{1,1,0,1},
												{1,1,1,0},
												{1,1,1,1}
											};
	
		GPIO_write(GPIOA, 7, number[num][0]); 	//binary 2^3
		GPIO_write(GPIOB, 6, number[num][1]); 	//binary 2^2
		GPIO_write(GPIOC, 7, number[num][2]); 	//binary 2^1
		GPIO_write(GPIOA, 9, number[num][3]); 	//binary 2^0
}