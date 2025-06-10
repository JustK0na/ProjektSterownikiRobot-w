################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/nt35510/nt35510.c 

OBJS += \
./Drivers/Components/nt35510/nt35510.o 

C_DEPS += \
./Drivers/Components/nt35510/nt35510.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/nt35510/%.o Drivers/Components/nt35510/%.su Drivers/Components/nt35510/%.cyclo: ../Drivers/Components/nt35510/%.c Drivers/Components/nt35510/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/Components" -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/STM32F429I-Discovery" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-nt35510

clean-Drivers-2f-Components-2f-nt35510:
	-$(RM) ./Drivers/Components/nt35510/nt35510.cyclo ./Drivers/Components/nt35510/nt35510.d ./Drivers/Components/nt35510/nt35510.o ./Drivers/Components/nt35510/nt35510.su

.PHONY: clean-Drivers-2f-Components-2f-nt35510

