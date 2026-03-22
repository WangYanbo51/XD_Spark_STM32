################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/Src/mpu6050.c \
../Lib/Src/tca9548a.c 

OBJS += \
./Lib/Src/mpu6050.o \
./Lib/Src/tca9548a.o 

C_DEPS += \
./Lib/Src/mpu6050.d \
./Lib/Src/tca9548a.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Src/%.o Lib/Src/%.su Lib/Src/%.cyclo: ../Lib/Src/%.c Lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../App/Inc -I../Lib/Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Src

clean-Lib-2f-Src:
	-$(RM) ./Lib/Src/mpu6050.cyclo ./Lib/Src/mpu6050.d ./Lib/Src/mpu6050.o ./Lib/Src/mpu6050.su ./Lib/Src/tca9548a.cyclo ./Lib/Src/tca9548a.d ./Lib/Src/tca9548a.o ./Lib/Src/tca9548a.su

.PHONY: clean-Lib-2f-Src

