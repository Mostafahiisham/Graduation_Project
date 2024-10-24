/*
 * car.h
 *
 *  Created on: Oct 18, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_CAR_H_
#define INC_CAR_H_

#define SPEED1	 13107
#define SPEED2	 26214
#define SPEED3	 39321
#define SPEED4	 52428
#define SPEED5	 62259

#define MOVE_FORWARD  0b11100100
#define MOVE_BACKWARD 0b00011011
#define MOVE_RIGHT    0b10100000
					  //3a-2a-1a-1b-2b-4a-3b-4b
#define MOVE_LEFT     0b01000100
#define STOP_CAR	  0b00000000

void CAR_Control(void);
void Voltage_Read(void);
void CAR_Movement_Init(void);
/*Car movement fucntions*/
void CAR_MoveForward(void);
void CAR_MoveBackWard(void);
void CAR_MoveRight(void);
void CAR_MoveLeft(void);
void CAR_STOP(void);


void CAR_FirstSpeed(void);
void CAR_SecondSpeed(void);
void CAR_ThirdSpeed(void);
void CAR_FourthSpeed(void);
void CAR_FifthSpeed(void);

#endif /* INC_CAR_H_ */
