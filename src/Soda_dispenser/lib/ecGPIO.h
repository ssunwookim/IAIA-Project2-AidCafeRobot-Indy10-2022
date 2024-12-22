/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : KIM SUNWOO LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

// MODER
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

// IDR & ODR
#define HIGH 1
#define LOW  0

// OUTPUT Type
#define EC_OUT_PU 0
#define EC_OUT_OP 1

// OSPEED
#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

// PUDR
#define EC_NO 0
#define EC_PU 1
#define EC_PD 2
#define EC_RE 3

// PIN
// #define LED_PIN 	6
#define BUTTON_PIN 13

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode); //0
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output); //0
int  GPIO_read(GPIO_TypeDef *Port, int pin); //0 
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode); //0
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed); //0
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type); //0
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd); //0

void sevensegment_init(void); 
void sevensegment_decoder(uint8_t  num);

void sevensegment_display_init(void); 
void sevensegment_display(uint8_t  num);
	 
void LED_toggle(void);
void LED_binary(uint8_t  num);

 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif