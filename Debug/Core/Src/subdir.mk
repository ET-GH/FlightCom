################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/airbrake.c \
../Core/Src/altitude_ekf.c \
../Core/Src/ambar_hil_usb.c \
../Core/Src/behavior.c \
../Core/Src/bmp388.c \
../Core/Src/controller.c \
../Core/Src/flight_log.c \
../Core/Src/lis2mdl.c \
../Core/Src/lsm6dsv32x.c \
../Core/Src/main.c \
../Core/Src/openrocket_run_data.c \
../Core/Src/radio_bridge.c \
../Core/Src/rocket_sensors.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/sx1280.c \
../Core/Src/sx1280_port.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c \
../Core/Src/tmc5240.c \
../Core/Src/usb_comm.c \
../Core/Src/w25q64.c 

OBJS += \
./Core/Src/airbrake.o \
./Core/Src/altitude_ekf.o \
./Core/Src/ambar_hil_usb.o \
./Core/Src/behavior.o \
./Core/Src/bmp388.o \
./Core/Src/controller.o \
./Core/Src/flight_log.o \
./Core/Src/lis2mdl.o \
./Core/Src/lsm6dsv32x.o \
./Core/Src/main.o \
./Core/Src/openrocket_run_data.o \
./Core/Src/radio_bridge.o \
./Core/Src/rocket_sensors.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/sx1280.o \
./Core/Src/sx1280_port.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/tmc5240.o \
./Core/Src/usb_comm.o \
./Core/Src/w25q64.o 

C_DEPS += \
./Core/Src/airbrake.d \
./Core/Src/altitude_ekf.d \
./Core/Src/ambar_hil_usb.d \
./Core/Src/behavior.d \
./Core/Src/bmp388.d \
./Core/Src/controller.d \
./Core/Src/flight_log.d \
./Core/Src/lis2mdl.d \
./Core/Src/lsm6dsv32x.d \
./Core/Src/main.d \
./Core/Src/openrocket_run_data.d \
./Core/Src/radio_bridge.d \
./Core/Src/rocket_sensors.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/sx1280.d \
./Core/Src/sx1280_port.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d \
./Core/Src/tmc5240.d \
./Core/Src/usb_comm.d \
./Core/Src/w25q64.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -DUX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../Core/Inc/Fusion -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I../USBX/App -I../USBX/Target -I../Middlewares/ST/usbx/common/core/inc -I../Middlewares/ST/usbx/ports/generic/inc -I../Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../Middlewares/ST/usbx/common/usbx_device_classes/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/airbrake.cyclo ./Core/Src/airbrake.d ./Core/Src/airbrake.o ./Core/Src/airbrake.su ./Core/Src/altitude_ekf.cyclo ./Core/Src/altitude_ekf.d ./Core/Src/altitude_ekf.o ./Core/Src/altitude_ekf.su ./Core/Src/ambar_hil_usb.cyclo ./Core/Src/ambar_hil_usb.d ./Core/Src/ambar_hil_usb.o ./Core/Src/ambar_hil_usb.su ./Core/Src/behavior.cyclo ./Core/Src/behavior.d ./Core/Src/behavior.o ./Core/Src/behavior.su ./Core/Src/bmp388.cyclo ./Core/Src/bmp388.d ./Core/Src/bmp388.o ./Core/Src/bmp388.su ./Core/Src/controller.cyclo ./Core/Src/controller.d ./Core/Src/controller.o ./Core/Src/controller.su ./Core/Src/flight_log.cyclo ./Core/Src/flight_log.d ./Core/Src/flight_log.o ./Core/Src/flight_log.su ./Core/Src/lis2mdl.cyclo ./Core/Src/lis2mdl.d ./Core/Src/lis2mdl.o ./Core/Src/lis2mdl.su ./Core/Src/lsm6dsv32x.cyclo ./Core/Src/lsm6dsv32x.d ./Core/Src/lsm6dsv32x.o ./Core/Src/lsm6dsv32x.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/openrocket_run_data.cyclo ./Core/Src/openrocket_run_data.d ./Core/Src/openrocket_run_data.o ./Core/Src/openrocket_run_data.su ./Core/Src/radio_bridge.cyclo ./Core/Src/radio_bridge.d ./Core/Src/radio_bridge.o ./Core/Src/radio_bridge.su ./Core/Src/rocket_sensors.cyclo ./Core/Src/rocket_sensors.d ./Core/Src/rocket_sensors.o ./Core/Src/rocket_sensors.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/sx1280.cyclo ./Core/Src/sx1280.d ./Core/Src/sx1280.o ./Core/Src/sx1280.su ./Core/Src/sx1280_port.cyclo ./Core/Src/sx1280_port.d ./Core/Src/sx1280_port.o ./Core/Src/sx1280_port.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/tmc5240.cyclo ./Core/Src/tmc5240.d ./Core/Src/tmc5240.o ./Core/Src/tmc5240.su ./Core/Src/usb_comm.cyclo ./Core/Src/usb_comm.d ./Core/Src/usb_comm.o ./Core/Src/usb_comm.su ./Core/Src/w25q64.cyclo ./Core/Src/w25q64.d ./Core/Src/w25q64.o ./Core/Src/w25q64.su

.PHONY: clean-Core-2f-Src

