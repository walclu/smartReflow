################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/utilities/dbg_trace.c \
../Middlewares/ST/STM32_WPAN/utilities/otp.c \
../Middlewares/ST/STM32_WPAN/utilities/stm_list.c \
../Middlewares/ST/STM32_WPAN/utilities/stm_queue.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.o \
./Middlewares/ST/STM32_WPAN/utilities/otp.o \
./Middlewares/ST/STM32_WPAN/utilities/stm_list.o \
./Middlewares/ST/STM32_WPAN/utilities/stm_queue.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.d \
./Middlewares/ST/STM32_WPAN/utilities/otp.d \
./Middlewares/ST/STM32_WPAN/utilities/stm_list.d \
./Middlewares/ST/STM32_WPAN/utilities/stm_queue.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/utilities/%.o Middlewares/ST/STM32_WPAN/utilities/%.su: ../Middlewares/ST/STM32_WPAN/utilities/%.c Middlewares/ST/STM32_WPAN/utilities/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-utilities

clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-utilities:
	-$(RM) ./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.d ./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.o ./Middlewares/ST/STM32_WPAN/utilities/dbg_trace.su ./Middlewares/ST/STM32_WPAN/utilities/otp.d ./Middlewares/ST/STM32_WPAN/utilities/otp.o ./Middlewares/ST/STM32_WPAN/utilities/otp.su ./Middlewares/ST/STM32_WPAN/utilities/stm_list.d ./Middlewares/ST/STM32_WPAN/utilities/stm_list.o ./Middlewares/ST/STM32_WPAN/utilities/stm_list.su ./Middlewares/ST/STM32_WPAN/utilities/stm_queue.d ./Middlewares/ST/STM32_WPAN/utilities/stm_queue.o ./Middlewares/ST/STM32_WPAN/utilities/stm_queue.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-utilities

