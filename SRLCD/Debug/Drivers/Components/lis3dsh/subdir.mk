################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/lis3dsh/lis3dsh.c 

OBJS += \
./Drivers/Components/lis3dsh/lis3dsh.o 

C_DEPS += \
./Drivers/Components/lis3dsh/lis3dsh.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/lis3dsh/%.o Drivers/Components/lis3dsh/%.su Drivers/Components/lis3dsh/%.cyclo: ../Drivers/Components/lis3dsh/%.c Drivers/Components/lis3dsh/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/Components" -I"/home/ai-thigs/STM32CubeIDE/workspace_1.18.1/SRLCD/Drivers/STM32F429I-Discovery" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-lis3dsh

clean-Drivers-2f-Components-2f-lis3dsh:
	-$(RM) ./Drivers/Components/lis3dsh/lis3dsh.cyclo ./Drivers/Components/lis3dsh/lis3dsh.d ./Drivers/Components/lis3dsh/lis3dsh.o ./Drivers/Components/lis3dsh/lis3dsh.su

.PHONY: clean-Drivers-2f-Components-2f-lis3dsh

