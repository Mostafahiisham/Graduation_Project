/*
 * SYSTTICK_Prv.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Mostafa Hisham
 */

#ifndef SYSTTICK_PRV_H_
#define SYSTTICK_PRV_H_

#define SYSTICK_BASE_ADDRESS 	0xE000E010

typedef struct{
	u32 CTRL;
	u32 LOAD;
	u32 VAL;
	u32 CALIB;
}SYSTICK_t;

#define SYSTICK ((volatile SYSTICK_t*)(SYSTICK_BASE_ADDRESS))

#define ENABLE			0
#define TICKINT		1
#define CLKSOURCE			2
#define COUNTFLAG			16

#endif /* SYSTTICK_PRV_H_ */
