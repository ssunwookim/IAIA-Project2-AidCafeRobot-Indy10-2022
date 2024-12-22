/**
******************************************************************************
* @author  KIM SUNWOO
* @Mod		 2023-11-03
* @brief   Embedded Controller:  EC_HAL_for_ecStepper
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
// uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
		uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
	
// A B A' B'
 	{0b1100,{S3,S1}},
	{0b0110,{S0,S2}},
	{0b0011,{S1,S3}},
	{0b1001,{S2,S0}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 	{0b1000,{S7,S1}},
	{0b1100,{S0,S2}},
	{0b0100,{S1,S3}},
	{0b0110,{S2,S4}},
	{0b0010,{S3,S5}},
	{0b0011,{S4,S6}},
	{0b0001,{S5,S7}},
	{0b1001,{S6,S0}}

};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
	//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	myStepper.port2 = port2;
	myStepper.pin2  = pin2;
	myStepper.port3 = port3;
	myStepper.pin3  = pin3;
	myStepper.port4 = port4;
	myStepper.pin4	= pin4;
	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	
	GPIO_init(port1, pin1, OUTPUT);
	GPIO_otype(port1, pin1, EC_OUT_PU);
	GPIO_pupd(port1, pin1, EC_NO);
	GPIO_ospeed(port1, pin1, EC_FAST);	
	GPIO_init(port2, pin2, OUTPUT);
	GPIO_otype(port2, pin2, EC_OUT_PU);
	GPIO_pupd(port2, pin2, EC_NO);
	GPIO_ospeed(port2, pin2, EC_FAST);	
	GPIO_init(port3, pin3, OUTPUT);
	GPIO_otype(port3, pin3, EC_OUT_PU);
	GPIO_pupd(port3, pin3, EC_NO);
	GPIO_ospeed(port3, pin3, EC_FAST);	
	GPIO_init(port4, pin4, OUTPUT);
	GPIO_otype(port4, pin4, EC_OUT_PU);
	GPIO_pupd(port4, pin4, EC_NO);
	GPIO_ospeed(port4, pin4, EC_FAST);	
}


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0b1000) >> 3);
		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0b0010) >> 1);
		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0b0001) >> 0);

	}	 
 	else if (mode == HALF){    // HALF mode
		GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0b1000) >> 3);
		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_half[state].out & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (FSM_half[state].out & 0b0010) >> 1);
		GPIO_write(myStepper.port4, myStepper.pin4, (FSM_half[state].out & 0b0001) >> 0);
	}
}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
		step_delay = (60 * 1000) / (whatSpeed * step_per_rev) ; // Convert rpm to  [msec] delay
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
		delay_ms(step_delay);                       // delay (step_delay); 				 
	    	if (mode == FULL) 		 												
			state = FSM_full[state].next[direction]; // YOUR CODE       // state = next state
		else if (mode == HALF) 
			state = FSM_half[state].next[direction]; // YOUR CODE       // state = next state		
		Stepper_pinOut(state, mode);
   	}
}


void Stepper_stop (void){ 
    	myStepper._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
		GPIO_write(myStepper.port1, myStepper.pin1, (myStepper._step_num & 0b1000) >> 3);
		GPIO_write(myStepper.port2, myStepper.pin2, (myStepper._step_num & 0b0100) >> 2);
		GPIO_write(myStepper.port3, myStepper.pin3, (myStepper._step_num & 0b0010) >> 1);
		GPIO_write(myStepper.port4, myStepper.pin4, (myStepper._step_num & 0b0001) >> 0);
}