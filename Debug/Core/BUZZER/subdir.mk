################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BUZZER/buzzer.c 

OBJS += \
./Core/BUZZER/buzzer.o 

C_DEPS += \
./Core/BUZZER/buzzer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BUZZER/%.o Core/BUZZER/%.su Core/BUZZER/%.cyclo: ../Core/BUZZER/%.c Core/BUZZER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BUZZER

clean-Core-2f-BUZZER:
	-$(RM) ./Core/BUZZER/buzzer.cyclo ./Core/BUZZER/buzzer.d ./Core/BUZZER/buzzer.o ./Core/BUZZER/buzzer.su

.PHONY: clean-Core-2f-BUZZER

