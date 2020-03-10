/******************************************************************************
 * @file     io_ctrl.c
 * @brief
 *           GPIO control.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "StdDriver\NUC121.h"

#include "io_ctrl.h"

//#define CAPLED PC13
#define NUMLED PC13
//#define SCRLED PB7
	
extern	volatile	uint8_t 	LED_Status[2];		// LED Status
				volatile	uint8_t 	NumLk_flag = 0;		// 1= Number Lock, 0= not	

//*******************************
// Initial LED Control IO 
//				  
// GPB.4 = CAPLED as Output mode, =1 LED off, 0=LED on
// GPB.6 = NUMLED as Output mode, =1 LED off, 0=LED on
// GPB.7 = SCRLED as Output mode, =1 LED off, 0=LED on
//*******************************
void Init_LED_Ctrl_IO(void)
{
	/* Set PB.4 (CAPLED) as Output mode and High */
	//GPIO_SetMode(PC, BIT13, GPIO_MODE_OUTPUT);
	//CAPLED = 1;

	/* Set PB.6 (NUMLED) as Output mode and High */
	GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT);
	NUMLED = 0;

	/* Set PB.7 (SCRLED) as Output mode and High */
	//GPIO_SetMode(PB, BIT7, GPIO_MODE_OUTPUT);
	//SCRLED = 1;

}

//*******************************
// Change LED OnOff 
//				  
//*******************************
void Change_LED_OnOff(void)
{
	/* NumLk LED */
	if(LED_Status[0] & 0x01){
		NUMLED = 0;
		NumLk_flag = 1;
	}else{
		NUMLED = 1;
		NumLk_flag = 0;
	}
		
	/* Caps LED 
	if(LED_Status[0] & 0x02)
		CAPLED = 0;
	else
		CAPLED = 1; */
					
	/* ScrLk LED
	if(LED_Status[0] & 0x04)
		SCRLED = 0;
	else
		SCRLED = 1; */
	
}




