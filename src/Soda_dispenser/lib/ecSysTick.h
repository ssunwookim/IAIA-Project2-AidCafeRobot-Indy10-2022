/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : KIM SUNWOO LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for Systic
/----------------------------------------------------------------*/


#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

	extern volatile uint32_t msTicks;
	void SysTick_init(void);
	void SysTick_Handler(void);
	void SysTick_counter();
	void delay_ms(uint32_t msec);
	void SysTick_reset(void);
	uint32_t SysTick_val(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif