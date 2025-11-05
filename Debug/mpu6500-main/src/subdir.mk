################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mpu6500-main/src/driver_mpu6500.c 

OBJS += \
./mpu6500-main/src/driver_mpu6500.o 

C_DEPS += \
./mpu6500-main/src/driver_mpu6500.d 


# Each subdirectory must supply rules for building sources it contributes
mpu6500-main/src/%.o mpu6500-main/src/%.su mpu6500-main/src/%.cyclo: ../mpu6500-main/src/%.c mpu6500-main/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/franu/Desktop/Inzynierka/FC_workspace/FC_inzynierka_v2/mpu6500-main/interface" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-mpu6500-2d-main-2f-src

clean-mpu6500-2d-main-2f-src:
	-$(RM) ./mpu6500-main/src/driver_mpu6500.cyclo ./mpu6500-main/src/driver_mpu6500.d ./mpu6500-main/src/driver_mpu6500.o ./mpu6500-main/src/driver_mpu6500.su

.PHONY: clean-mpu6500-2d-main-2f-src

