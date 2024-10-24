/*
 * adc.c
 *
 *  Created on: Sep 15, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<SYSTICK.h>
#include<ADC.h>
#include<ADC_Cfg.h>
#include<ADC_Prv.h>


static void (*GLB_PTR[2])(void) ;
/**/
void ADC1_vInit(ADC_Res_t  Resolution,ADC_Prescaler Prescaler)
{	/*Select the ADC1 Resolution*/
	ADC1->CR1 &=~(RES_MUSK<<24);
	ADC1->CR1 |=(Resolution<<24);

	/*Select the PreScaler for ADC1 */
	ADC1->CCR &=~(PRE_MUSK<<0);
	ADC1->CCR |= (Prescaler<<0);

	/*Start the ADC*/
	SET_BIT(ADC1->CR2,CR2_ADON);

}
u16 ADC1_u16ReadRegularSingleChannel(ADC_Channel_no Channel,Channel_Mode Channel_M)
{
	/*SW Start for Conversion in Regular Channels*/
	SET_BIT(ADC1->CR2,CR2_SWSTART);
	/*To set the sequence of the single channel at first conversion*/
	ADC1_vSetRegularChannelSequence(Channel, ADCx_Sequence1);
	/*Select the channel Continuous conversion mode or single conversion mode*/
	switch(Channel_M)
	{
	case Single_Conv:
		CLR_BIT(ADC1->CR2,CR2_CONT);
	break;
	case Continous_Conv:
		SET_BIT(ADC1->CR2,CR2_CONT);
	break;
	default:
	break;
	}
	/*Busy Wait on the End of Conversion Flag to be equal 1 */
	while(GET_BIT(ADC1->SR,SR_EOC)==0);
	/*Read the data from the Regular Data Register */
	/*Clear the End of Conversion flag when Read the data Register */
	return ADC1-> DR;
}
u16 ADC1_u16ReadRegularSingleChannelAsynchronous(ADC_Channel_no Channel,Channel_Mode Channel_M)
{
	/*To set the sequence of the single channel at first conversion*/
	ADC1_vSetRegularChannelSequence(Channel, ADCx_Sequence1);
	/*Select the channel Continuous conversion mode or single conversion mode*/
	switch(Channel_M)
	{
	case Single_Conv:
		CLR_BIT(ADC1->CR2,CR2_CONT);
	break;
	case Continous_Conv:
		SET_BIT(ADC1->CR2,CR2_CONT);
	break;
	default:
	break;
	}
	/*Read the data from the Regular Data Register */
	/*Clear the End of Conversion flag when Read the data Register */
	return ADC1-> DR;
}
void ADC1_RegularSwStartEnable(void)
{
	/*Enable SW Start for Conversion in Regular Channels*/
		SET_BIT(ADC1->CR2,CR2_SWSTART);
}
void ADC1_RegularSwStartDisable(void)
{
	/*Enable SW Start for Conversion in Regular Channels*/
		CLR_BIT(ADC1->CR2,CR2_SWSTART);
}
void ADC1_vSetRegularChannelSequence(ADC_Channel_no Channel,ADC_Sequence_no Sqn_no)
{
	/*Channel position from Sequence 0 t0 5 */
	if(Sqn_no < ADCx_Sequence7)
	{
		ADC1->SQR3 &=~((SQR_MUSK)<< Sqn_no * 5);
		ADC1->SQR3 |= ((Channel)<<Sqn_no * 5);
	}
	else if(Sqn_no < ADCx_Sequence13)
	{
		ADC1->SQR2 &=~((SQR_MUSK)<< (Sqn_no-6) * 5);
		ADC1->SQR2 |= ((Channel)<< (Sqn_no-6) * 5);
	}
	else
	{
		ADC1->SQR1 &=~((SQR_MUSK)<< (Sqn_no-12) * 5);
		ADC1->SQR1 |= ((Channel)<< (Sqn_no-12) * 5);
	}
}
void ADC1_vSetSamplingTime(ADC_Channel_no Channel,Sampling_Time Sampling)
{
	if(Channel > ADCx_CHANNEL9)
	{
		ADC1->SMPR1 &=~ ((SMPR_MUSK)<<(Channel%10)*3);
		ADC1->SMPR1 |= (Sampling<<(Channel%10)*3);
	}
	else if(Channel < ADCx_CHANNEL10)
	{
		ADC1->SMPR2 &=~ ((SMPR_MUSK)<<(Channel)*3);
		ADC1->SMPR2 |= (Sampling<<(Channel)*3);
	}
	else
	{
		//DoNothing
	}
}
u16 ADC1_vReadInjectedSingleChannel(ADC_Channel_no Channel)
{
	/*SW Start for Conversion in injected Channels*/
	SET_BIT(ADC1->CR2,CR2_JSWSTART);
	/*Set the single Channel Sequence */
	ADC1_vSetInjectedChannelSequence(Channel, ADCx_Sequence4);
	/*Busy wait on the injected channel end of conversion flag to be set*/
	while(!GET_BIT(ADC1->SR,SR_JEOC));
	/*Read the data from the Regular Data Register */
	/*Clear the End of Conversion flag when Read the data Register */
	return ADC1->JDR1;
}
u16 ADC1_vReadInjectedSingleChannelAsynchronous(ADC_Channel_no Channel)
{
	/*Set the single Channel Sequence */
	ADC1_vSetInjectedChannelSequence(Channel, ADCx_Sequence4);
	return ADC1->JDR1;
}
void ADC1_InjectedSwStartEnable(void)
{
	/*Enable SW Start for Conversion in injected Channels*/
	SET_BIT(ADC1->CR2,CR2_JSWSTART);
}
void ADC1_InjectedSwStartDisable(void)
{
	/*Enable SW Start for Conversion in injected Channels*/
	CLR_BIT(ADC1->CR2,CR2_JSWSTART);
}
void ADC1_vSetInjectedChannelSequence(ADC_Channel_no Channel,ADC_Sequence_no Sqn_no)
{
	ADC1->JSQR &=~ ((JSQR_MUSK)<<Sqn_no *5);
	ADC1->JSQR |= (Channel<<Sqn_no *5);

}

void ADC1_vSetScanMode(u8 Copy_u8Channel)
{
	/*Enable the Scan mode */
	SET_BIT(ADC1->CR1,CR1_SCAN);
	/*Enable End of Conversion After each channel Conversion*/
	SET_BIT(ADC1->CR2,CR2_EOCS);
	/*Set the No. of Conversions */
	ADC1->SQR1 &=~((SQR_MUSK)<<SQR1_L);
	ADC1->SQR1 |= (Copy_u8Channel<<SQR1_L);
}
void ADC1_InterruptEnable(Channel_t Channel_T)
{
	switch(Channel_T)
	{
		case Regular:
			SET_BIT(ADC1->CR1,CR1_EOCIE);
		break;
		case Injected:
			SET_BIT(ADC1->CR1,CR1_JEOCIE);
		break;
	}
}
void ADC1_DMAMode(ADCx_DMA_St DMA_St)
{
	switch(DMA_St)
	{
	case ADCx_DMA_En:
		/*Enable DMA Mode*/
		SET_BIT(ADC1->CR2,CR2_DMA);
		/*Enable Continuous DMA Request*/
		SET_BIT(ADC1->CR2,CR2_DDS);
	break;
	case ADCx_DMA_Dis:
		CLR_BIT(ADC1->CR2,CR2_DMA);
		CLR_BIT(ADC1->CR2,CR2_DDS);
	break;
	}
}
void ADC1_ExtTriggerRegular(ADCx_EXITrigger Trigger_Edge,ADCx_EXITrigger_S Trigger_Src)
{
	/*Select the External Trigger Polarity For Regular Channels*/
	ADC1->CR2 &=~((EXTEN_MUSK)<<CR2_EXTEN1);
	ADC1->CR2 |= (Trigger_Edge<<CR2_EXTEN1);
	/*Select the External Trigger Source */
	ADC1->CR2 &=~((EXTSEL_MUSK)<<CR2_EXTSEL1);
	ADC1->CR2 |= (Trigger_Src<<CR2_EXTSEL1);

}
void ADC1_ExtTriggerInjected(ADCx_EXITrigger Trigger_Edge,ADCx_JEXITrigger_S Trigger_Src)
{
	/*Select the External Trigger Polarity For Regular Channels*/
	ADC1->CR2 &=~((JEXTEN_MUSK)<<CR2_JEXTEN1);
	ADC1->CR2 |= (Trigger_Edge<<CR2_JEXTEN1);
	/*Select the External Trigger Source */
	ADC1->CR2 &=~((JEXTSEL_MUSK)<<CR2_JEXTSEL1);
	ADC1->CR2 |= (Trigger_Src<<CR2_JEXTSEL1);

}
void ADC1_DisContionuousMode(Channel_t Channel_T,ADC_Channel_no Channel)
{
	switch(Channel_T)
		{
			case Regular:
				SET_BIT(ADC1->CR1,CR1_DISCEN);
				ADC1->CR1 &=~((DISCNUM_MUSK)<<CR1_DISCNUM1);
				ADC1->CR1 |=(Channel<<CR1_DISCNUM1);
			break;
			case Injected:
				SET_BIT(ADC1->CR1,CR1_JDISCEN);
			break;
		}
}
void ADC1_CallBack(void(*ptr)(void),Channel_t Channel_T)
{
	switch(Channel_T)
	{
	case Regular:
		GLB_PTR[0] = ptr;
		//*ADC_Reading = ADC1->DR;
		break;
	case Injected:
		GLB_PTR[1] = ptr;
		//*ADC_Reading = ADC1->JDR1;
		break;
	}
}

void ADC_IRQHandler(void)
{
	if(GET_BIT(ADC1->SR,SR_EOC))
	{
		GLB_PTR[0]();
		CLR_BIT(ADC1->SR,SR_EOC);
	}
	else if(GET_BIT(ADC1->SR,SR_JEOC))
	{
		GLB_PTR[1]();
		CLR_BIT(ADC1->SR,SR_JEOC);
	}
}
