################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/PID/Double\ Loop\ PID/pid_controller.c 

OBJS += \
./Core/PID/Double\ Loop\ PID/pid_controller.o 

C_DEPS += \
./Core/PID/Double\ Loop\ PID/pid_controller.d 


# Each subdirectory must supply rules for building sources it contributes
Core/PID/Double\ Loop\ PID/pid_controller.o: ../Core/PID/Double\ Loop\ PID/pid_controller.c Core/PID/Double\ Loop\ PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/PID/Double Loop PID/pid_controller.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-PID-2f-Double-20-Loop-20-PID

clean-Core-2f-PID-2f-Double-20-Loop-20-PID:
	-$(RM) ./Core/PID/Double\ Loop\ PID/pid_controller.cyclo ./Core/PID/Double\ Loop\ PID/pid_controller.d ./Core/PID/Double\ Loop\ PID/pid_controller.o ./Core/PID/Double\ Loop\ PID/pid_controller.su

.PHONY: clean-Core-2f-PID-2f-Double-20-Loop-20-PID

