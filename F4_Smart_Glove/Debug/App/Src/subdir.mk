################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/delay.c \
../App/Src/gesture.c \
../App/Src/task_main.c 

OBJS += \
./App/Src/delay.o \
./App/Src/gesture.o \
./App/Src/task_main.o 

C_DEPS += \
./App/Src/delay.d \
./App/Src/gesture.d \
./App/Src/task_main.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../App/Inc -I../Lib/Inc -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/delay.cyclo ./App/Src/delay.d ./App/Src/delay.o ./App/Src/delay.su ./App/Src/gesture.cyclo ./App/Src/gesture.d ./App/Src/gesture.o ./App/Src/gesture.su ./App/Src/task_main.cyclo ./App/Src/task_main.d ./App/Src/task_main.o ./App/Src/task_main.su

.PHONY: clean-App-2f-Src

