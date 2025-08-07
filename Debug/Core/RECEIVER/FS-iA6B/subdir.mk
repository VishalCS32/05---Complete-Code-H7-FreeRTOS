################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/RECEIVER/FS-iA6B/FS-iA6B.c 

OBJS += \
./Core/RECEIVER/FS-iA6B/FS-iA6B.o 

C_DEPS += \
./Core/RECEIVER/FS-iA6B/FS-iA6B.d 


# Each subdirectory must supply rules for building sources it contributes
Core/RECEIVER/FS-iA6B/%.o Core/RECEIVER/FS-iA6B/%.su Core/RECEIVER/FS-iA6B/%.cyclo: ../Core/RECEIVER/FS-iA6B/%.c Core/RECEIVER/FS-iA6B/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-RECEIVER-2f-FS-2d-iA6B

clean-Core-2f-RECEIVER-2f-FS-2d-iA6B:
	-$(RM) ./Core/RECEIVER/FS-iA6B/FS-iA6B.cyclo ./Core/RECEIVER/FS-iA6B/FS-iA6B.d ./Core/RECEIVER/FS-iA6B/FS-iA6B.o ./Core/RECEIVER/FS-iA6B/FS-iA6B.su

.PHONY: clean-Core-2f-RECEIVER-2f-FS-2d-iA6B

