#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F411.h"

// Definition Pin of the motor driver
#define BUTTON_PIN 13
#define DIR_PIN 2
#define PWM_PIN PA_0
// Definition Pin of ultrasonic senser
#define TRIG PA_6
#define ECHO PB_6

// ultrasonic value
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

uint32_t count = 0;
uint32_t state = 1;
volatile static int i = 0;
volatile static float duty = 0;

uint32_t count1 = 0;
uint32_t count2 = 0;
uint32_t flag1 = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
		GPIO_write(GPIOC, DIR_PIN, LOW);
		PWM_duty(PWM_PIN, duty);
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
		printf("%f cm\r\n", distance);
		
		switch (flag1) {
			
			case 0:
				
				if(distance < 5 && distance > 0){
					count1++;
				}
				else if(distance > 30 || distance < 0) {
					count2++;	
				}
					
				if(count1 > 10 && count1 <= 80){
					flag1 = 1;	
				}
				else if(count2 > 10){
					count1 = 0;
					count2 = 0;
				}
				
				break;
				
			case 1:
				
				count1++;
			
				if (count1 > 80){
					flag1 = 0;
				}
				
				break;
		}
		
		delay_ms(500);
	}		
}

// Initialiization 
void setup(void) {	
	//clock
	RCC_PLL_init();
	SysTick_init();
	UART2_init();
	// Initialize Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	// Initialize Direction pin
	GPIO_init(GPIOC, DIR_PIN, OUTPUT);  // calls RCC_GPIOC_enable()
	GPIO_otype(GPIOC, DIR_PIN, EC_OUT_PU);
	// Priority Highest(0) External Interrupt
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	// PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 1);   // 1 msec PWM period
	// TIM3 period 1 msec, interrupt of 500mec
	TIM_UI_init(TIM3, 1);
	TIM_UI_enable(TIM3);
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);							  	// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
	
// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    							// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, IC_1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, IC_2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		if(state == 0){
			state = 1; 
			i=0;
		}
		else	state = 0; duty = 0;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}

void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){ // update interrupt flag
		//Create the code to rotate angle of RC by 500ms
		
		switch (flag1){
			
			case 0:
				duty = 0;
				break;
			
			case 1:
				duty = 0.9;
				break;
		}
		
		clear_UIF(TIM3);    // clear by writing 0
	}
}

void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     			// Update interrupt 
		ovf_cnt++			 ;													// overflow count
		clear_UIF(TIM4);  							   				// clear update interrupt flag
	}
	if(is_CCIF(TIM4, IC_1)){ 										// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, IC_1);					// Capture TimeStart
		clear_CCIF(TIM4, 1);                			// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM4, IC_2)){ 							// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4, IC_2);					// Capture TimeEnd
		timeInterval = ((time2 - time1) + ovf_cnt * (TIM2->ARR + 1)) / 100; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                        			// overflow reset
		clear_CCIF(TIM4,2);								  			// clear capture/compare interrupt flag 
	}
}



