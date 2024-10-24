################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DC_Motor.c \
../Core/Src/MRCC_Prg.c \
../Core/Src/Mgpio.c \
../Core/Src/Stp.c \
../Core/Src/Timer1.c \
../Core/Src/Volt_Sensor.c \
../Core/Src/adc.c \
../Core/Src/car.c \
../Core/Src/gptimer.c \
../Core/Src/main.c \
../Core/Src/nvic.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/systick.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/DC_Motor.o \
./Core/Src/MRCC_Prg.o \
./Core/Src/Mgpio.o \
./Core/Src/Stp.o \
./Core/Src/Timer1.o \
./Core/Src/Volt_Sensor.o \
./Core/Src/adc.o \
./Core/Src/car.o \
./Core/Src/gptimer.o \
./Core/Src/main.o \
./Core/Src/nvic.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/systick.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/DC_Motor.d \
./Core/Src/MRCC_Prg.d \
./Core/Src/Mgpio.d \
./Core/Src/Stp.d \
./Core/Src/Timer1.d \
./Core/Src/Volt_Sensor.d \
./Core/Src/adc.d \
./Core/Src/car.d \
./Core/Src/gptimer.d \
./Core/Src/main.d \
./Core/Src/nvic.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/systick.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DC_Motor.cyclo ./Core/Src/DC_Motor.d ./Core/Src/DC_Motor.o ./Core/Src/DC_Motor.su ./Core/Src/MRCC_Prg.cyclo ./Core/Src/MRCC_Prg.d ./Core/Src/MRCC_Prg.o ./Core/Src/MRCC_Prg.su ./Core/Src/Mgpio.cyclo ./Core/Src/Mgpio.d ./Core/Src/Mgpio.o ./Core/Src/Mgpio.su ./Core/Src/Stp.cyclo ./Core/Src/Stp.d ./Core/Src/Stp.o ./Core/Src/Stp.su ./Core/Src/Timer1.cyclo ./Core/Src/Timer1.d ./Core/Src/Timer1.o ./Core/Src/Timer1.su ./Core/Src/Volt_Sensor.cyclo ./Core/Src/Volt_Sensor.d ./Core/Src/Volt_Sensor.o ./Core/Src/Volt_Sensor.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/car.cyclo ./Core/Src/car.d ./Core/Src/car.o ./Core/Src/car.su ./Core/Src/gptimer.cyclo ./Core/Src/gptimer.d ./Core/Src/gptimer.o ./Core/Src/gptimer.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nvic.cyclo ./Core/Src/nvic.d ./Core/Src/nvic.o ./Core/Src/nvic.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/systick.cyclo ./Core/Src/systick.d ./Core/Src/systick.o ./Core/Src/systick.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

