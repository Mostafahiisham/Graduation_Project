/*
 * TIMER1_Cfg.h
 *
 *  Created on: Aug 13, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_TIMER1_CFG_H_
#define INC_TIMER1_CFG_H_

/*
 * PRESCALER VALUE FROM 0 -->65535
 */
#define TIMER1_PRESCALER	16

/*	Enabled with timer1 busy wait and interrupt Enable
 * UpDate_On
 * UpDate_Off
 */
#define UpDateRequest_SRC		UpDate_Off

/*Enabled with timer1 busy wait and interrupt Enable
 * Update_Gen_On
 * Update_Gen_Off
 */

#define UpDate_Generation		Update_Gen_Off

/*Enabled with timer1 busy wait only
 * One_Pulse_On
 * One_Pulse_Off
 */

#define ONE_PULSE_MODE 		One_Pulse_Off

/*Enabled with timer1 busy wait and interrupt Enable
 * Counter_Enable
 * Counter_Disable
 */

#define COUNTER_STATE		Counter_Enable

/*Enabled with timer1 PWM mode only
 * Auto_RE_PRE_Enable
 * Auto_RE_PRE_Disable
 */

#define AUTO_RELOAD_PRELOAD_STATE	Auto_RE_PRE_Enable

/*
 *Milli_Prescaler
 *Micro_Prescaler
 */

#define PRESCALER_Type			Micro_Prescaler
#endif /* INC_TIMER1_CFG_H_ */
