################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ili9325/ili9325.c 

OBJS += \
./Drivers/Components/ili9325/ili9325.o 

C_DEPS += \
./Drivers/Components/ili9325/ili9325.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ili9325/%.o Drivers/Components/ili9325/%.su Drivers/Components/ili9325/%.cyclo: ../Drivers/Components/ili9325/%.c Drivers/Components/ili9325/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/Components" -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/STM32F429I-Discovery" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ili9325

clean-Drivers-2f-Components-2f-ili9325:
	-$(RM) ./Drivers/Components/ili9325/ili9325.cyclo ./Drivers/Components/ili9325/ili9325.d ./Drivers/Components/ili9325/ili9325.o ./Drivers/Components/ili9325/ili9325.su

.PHONY: clean-Drivers-2f-Components-2f-ili9325

