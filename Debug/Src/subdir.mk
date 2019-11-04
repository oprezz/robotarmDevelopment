################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/encoder.c \
../Src/freertos.c \
../Src/main.c \
../Src/mainStateMachine.c \
../Src/motors.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_tim.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/utilityFunctions.c 

OBJS += \
./Src/encoder.o \
./Src/freertos.o \
./Src/main.o \
./Src/mainStateMachine.o \
./Src/motors.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_tim.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/utilityFunctions.o 

C_DEPS += \
./Src/encoder.d \
./Src/freertos.d \
./Src/main.d \
./Src/mainStateMachine.d \
./Src/motors.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_tim.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/utilityFunctions.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F429xx -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Inc" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/antal/Documents/STM32/stm32_test_npc/teszt_os/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


