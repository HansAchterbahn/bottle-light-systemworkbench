################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c \
../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.o \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.d \
./Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F3xx_HAL_Driver/Src/%.o: ../Drivers/STM32F3xx_HAL_Driver/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32F334x8 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Inc" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Inc" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/CMSIS/Include" -I"/home/hans/Dokumente/stmSystemWorkbench/workspace/F334K6/bottle-light/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

