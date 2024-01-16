################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/IIS3DWB-PID/iis3dwb_reg.c 

OBJS += \
./Lib/IIS3DWB-PID/iis3dwb_reg.o 

C_DEPS += \
./Lib/IIS3DWB-PID/iis3dwb_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/IIS3DWB-PID/%.o Lib/IIS3DWB-PID/%.su Lib/IIS3DWB-PID/%.cyclo: ../Lib/IIS3DWB-PID/%.c Lib/IIS3DWB-PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/thipa/STM32CubeIDE/workspace_1.14.0/readDataPooling/Lib/IIS3DWB-PID" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-IIS3DWB-2d-PID

clean-Lib-2f-IIS3DWB-2d-PID:
	-$(RM) ./Lib/IIS3DWB-PID/iis3dwb_reg.cyclo ./Lib/IIS3DWB-PID/iis3dwb_reg.d ./Lib/IIS3DWB-PID/iis3dwb_reg.o ./Lib/IIS3DWB-PID/iis3dwb_reg.su

.PHONY: clean-Lib-2f-IIS3DWB-2d-PID

