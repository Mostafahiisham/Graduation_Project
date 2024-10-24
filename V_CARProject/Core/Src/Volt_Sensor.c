/*
 * Volt_Sensor.c
 *
 *  Created on: Oct 17, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<Volt_Sensor.h>

f32 Volt_SensorReading(u16 ADC_u16Reading)
{
	f32 Local_f32AdcReference = 3300.0;
	f32 vin = 0.0;
	f32 vout = 0.0;
	f32 adc_max_value = 4095.0;
	/*To Get the Temperature analog Value in mV*/
	/*4096                 -->      3300*/
	/*Local_u16AnalogValue -->     ADC_u16Reading */

	vout = (((u32)ADC_u16Reading*Local_f32AdcReference)/adc_max_value);

	vin = vout *(25.0/3.3);
	return vin;
}
