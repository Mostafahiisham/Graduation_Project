/*
 * ADC.h
 *
 *  Created on: Sep 15, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_





typedef enum{
	ADCx_CHANNEL0,
	ADCx_CHANNEL1,
	ADCx_CHANNEL2,
	ADCx_CHANNEL3,
	ADCx_CHANNEL4,
	ADCx_CHANNEL5,
	ADCx_CHANNEL6,
	ADCx_CHANNEL7,
	ADCx_CHANNEL8,
	ADCx_CHANNEL9,
	ADCx_CHANNEL10,
	ADCx_CHANNEL11,
	ADCx_CHANNEL12,
	ADCx_CHANNEL13,
	ADCx_CHANNEL14,
	ADCx_CHANNEL15
}ADC_Channel_no;
typedef enum{
	ADCx_Sequence1,
	ADCx_Sequence2,
	ADCx_Sequence3,
	ADCx_Sequence4,
	ADCx_Sequence5,
	ADCx_Sequence6,
	ADCx_Sequence7,
	ADCx_Sequence8,
	ADCx_Sequence9,
	ADCx_Sequence10,
	ADCx_Sequence11,
	ADCx_Sequence12,
	ADCx_Sequence13,
	ADCx_Sequence14,
	ADCx_Sequence15,
	ADCx_Sequence16
}ADC_Sequence_no;

typedef enum{
	PCLK2_2,
	PCLK2_4,
	PCLK2_6,
	PCLK2_8,
}ADC_Prescaler;

typedef enum{
	ADCx_Res12,
	ADCx_Res10,
	ADCx_Res8,
	ADCx_Res6
}ADC_Res_t;

typedef enum{
	Single_Conv,
	Continous_Conv
}Channel_Mode;

typedef enum{
	ADCx_3Cycles,
	ADCx_15Cycles,
	ADCx_28Cycles,
	ADCx_56Cycles,
	ADCx_84Cycles,
	ADCx_112Cycles,
	ADCx_144Cycles,
	ADCx_480Cycles,
}Sampling_Time;
typedef enum{
	Regular,
	Injected
}Channel_t;
typedef enum{
	TRI_Disabled,
	TRI_RisingeEdge,
	TRI_FallingEdge,
	TRI_BothEdges,
}ADCx_EXITrigger;
typedef enum{
	ADCx_Timer1CC1event,
	ADCx_Timer1CC2event,
	ADCx_Timer1CC3event,
	ADCx_Timer2CC2event,
	ADCx_Timer2CC3event,
	ADCx_Timer2CC4event,
	ADCx_Timer2TRGOevent,
	ADCx_Timer3CC1event,
	ADCx_Timer3TRGOevent,
	ADCx_Timer4CC4event,
	ADCx_Timer5CC1event,
	ADCx_Timer5CC2event,
	ADCx_Timer5CC3event,
	ADCx_Reserved0,
	ADCx_Reserved1,
	ADCx_EXTIline11
}ADCx_EXITrigger_S;
typedef enum{
	ADCx_J_Timer1CC4event,
	ADCx_J_Timer1TRGOevent,
	ADCx_J_Timer2CC1event,
	ADCx_J_Timer2TRGOevent,
	ADCx_J_Timer3CC2event,
	ADCx_J_Timer3CC4event,
	ADCx_J_Timer4CC1event,
	ADCx_J_Timer4CC2event,
	ADCx_J_Timer4CC3event,
	ADCx_J_Timer4TRGOevent,
	ADCx_J_Timer5CC4event,
	ADCx_J_Timer5TRGOevent,
	ADCx_J_Reserved00,
	ADCx_J_Reserved01,
	ADCx_J_Reserved02,
	ADCx_J_EXTIline15
}ADCx_JEXITrigger_S;
typedef enum{
	ADCx_DMA_En,
	ADCx_DMA_Dis
}ADCx_DMA_St;
void ADC1_vInit(ADC_Res_t  Resolution,ADC_Prescaler Prescaler);
u16 ADC1_u16ReadRegularSingleChannel(ADC_Channel_no Channel,Channel_Mode Channel_M);
u16 ADC1_u16ReadRegularSingleChannelAsynchronous(ADC_Channel_no Channel,Channel_Mode Channel_M);
void ADC1_RegularSwStartEnable(void);
void ADC1_RegularSwStartDisable(void);
void ADC1_vSetRegularChannelSequence(ADC_Channel_no Channel,ADC_Sequence_no Sqn_no);
u16 ADC1_vReadInjectedSingleChannelAsynchronous(ADC_Channel_no Channel);
void ADC1_InjectedSwStartEnable(void);
void ADC1_InjectedSwStartDisable(void);
void ADC1_vSetSamplingTime(ADC_Channel_no Channel,Sampling_Time Sampling);
u16 ADC1_vReadInjectedSingleChannel(ADC_Channel_no Channel);
void ADC1_vSetInjectedChannelSequence(ADC_Channel_no Channel,ADC_Sequence_no Sqn_no);
void ADC1_vSetScanMode(u8 Copy_u8Channel);
void ADC1_InterruptEnable(Channel_t Channel_T);
void ADC1_ExtTriggerRegular(ADCx_EXITrigger Trigger_Edge,ADCx_EXITrigger_S Trigger_Src);
void ADC1_ExtTriggerInjected(ADCx_EXITrigger Trigger_Edge,ADCx_JEXITrigger_S Trigger_Src);
void ADC1_DMAMode(ADCx_DMA_St DMA_St);
void ADC1_CallBack(void(*ptr)(void),Channel_t Channel_T);
#endif /* INC_ADC_H_ */
