################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MAX31856/max31856.c 

OBJS += \
./MAX31856/max31856.o 

C_DEPS += \
./MAX31856/max31856.d 


# Each subdirectory must supply rules for building sources it contributes
MAX31856/%.o MAX31856/%.su MAX31856/%.cyclo: ../MAX31856/%.c MAX31856/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/GitHub/phamnamhien/MAX31856/MAX31856" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MAX31856

clean-MAX31856:
	-$(RM) ./MAX31856/max31856.cyclo ./MAX31856/max31856.d ./MAX31856/max31856.o ./MAX31856/max31856.su

.PHONY: clean-MAX31856

