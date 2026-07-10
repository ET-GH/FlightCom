################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Fusion/FusionAhrs.c \
../Core/Src/Fusion/FusionBias.c \
../Core/Src/Fusion/FusionCompass.c 

OBJS += \
./Core/Src/Fusion/FusionAhrs.o \
./Core/Src/Fusion/FusionBias.o \
./Core/Src/Fusion/FusionCompass.o 

C_DEPS += \
./Core/Src/Fusion/FusionAhrs.d \
./Core/Src/Fusion/FusionBias.d \
./Core/Src/Fusion/FusionCompass.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Fusion/%.o Core/Src/Fusion/%.su Core/Src/Fusion/%.cyclo: ../Core/Src/Fusion/%.c Core/Src/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -DUX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../Core/Inc/Fusion -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I../USBX/App -I../USBX/Target -I../Middlewares/ST/usbx/common/core/inc -I../Middlewares/ST/usbx/ports/generic/inc -I../Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../Middlewares/ST/usbx/common/usbx_device_classes/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Fusion

clean-Core-2f-Src-2f-Fusion:
	-$(RM) ./Core/Src/Fusion/FusionAhrs.cyclo ./Core/Src/Fusion/FusionAhrs.d ./Core/Src/Fusion/FusionAhrs.o ./Core/Src/Fusion/FusionAhrs.su ./Core/Src/Fusion/FusionBias.cyclo ./Core/Src/Fusion/FusionBias.d ./Core/Src/Fusion/FusionBias.o ./Core/Src/Fusion/FusionBias.su ./Core/Src/Fusion/FusionCompass.cyclo ./Core/Src/Fusion/FusionCompass.d ./Core/Src/Fusion/FusionCompass.o ./Core/Src/Fusion/FusionCompass.su

.PHONY: clean-Core-2f-Src-2f-Fusion

