################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MatFun.c \
../Core/Src/collab_util.c \
../Core/Src/huansic_jy62lib.c \
../Core/Src/huansic_malloc.c \
../Core/Src/huansic_motorlib.c \
../Core/Src/huansic_xblib.c \
../Core/Src/main.c \
../Core/Src/ming_malloc.c \
../Core/Src/positionpid.c \
../Core/Src/ssd1306.c \
../Core/Src/ssd1306_fonts.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/task.c 

OBJS += \
./Core/Src/MatFun.o \
./Core/Src/collab_util.o \
./Core/Src/huansic_jy62lib.o \
./Core/Src/huansic_malloc.o \
./Core/Src/huansic_motorlib.o \
./Core/Src/huansic_xblib.o \
./Core/Src/main.o \
./Core/Src/ming_malloc.o \
./Core/Src/positionpid.o \
./Core/Src/ssd1306.o \
./Core/Src/ssd1306_fonts.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/task.o 

C_DEPS += \
./Core/Src/MatFun.d \
./Core/Src/collab_util.d \
./Core/Src/huansic_jy62lib.d \
./Core/Src/huansic_malloc.d \
./Core/Src/huansic_motorlib.d \
./Core/Src/huansic_xblib.d \
./Core/Src/main.d \
./Core/Src/ming_malloc.d \
./Core/Src/positionpid.d \
./Core/Src/ssd1306.d \
./Core/Src/ssd1306_fonts.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MatFun.d ./Core/Src/MatFun.o ./Core/Src/MatFun.su ./Core/Src/collab_util.d ./Core/Src/collab_util.o ./Core/Src/collab_util.su ./Core/Src/huansic_jy62lib.d ./Core/Src/huansic_jy62lib.o ./Core/Src/huansic_jy62lib.su ./Core/Src/huansic_malloc.d ./Core/Src/huansic_malloc.o ./Core/Src/huansic_malloc.su ./Core/Src/huansic_motorlib.d ./Core/Src/huansic_motorlib.o ./Core/Src/huansic_motorlib.su ./Core/Src/huansic_xblib.d ./Core/Src/huansic_xblib.o ./Core/Src/huansic_xblib.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ming_malloc.d ./Core/Src/ming_malloc.o ./Core/Src/ming_malloc.su ./Core/Src/positionpid.d ./Core/Src/positionpid.o ./Core/Src/positionpid.su ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/ssd1306_fonts.d ./Core/Src/ssd1306_fonts.o ./Core/Src/ssd1306_fonts.su ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/task.d ./Core/Src/task.o ./Core/Src/task.su

.PHONY: clean-Core-2f-Src

