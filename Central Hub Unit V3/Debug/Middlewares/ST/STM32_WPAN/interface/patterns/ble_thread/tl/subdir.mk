################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.c \
../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.c \
../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c \
../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.c 

OBJS += \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.o \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.o \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.o \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.o 

C_DEPS += \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.d \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.d \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.d \
./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/%.o Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/%.su Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/%.cyclo: ../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/%.c Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DZIGBEE_WB -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../STM32_WPAN/App -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/extras -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zgp -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/mac -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zcl -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zcl/key -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zcl/se -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zcl/security -I../Middlewares/ST/STM32_WPAN/zigbee/stack/include/zcl/general -I../Middlewares/ST/STM32_WPAN/zigbee/platform -I../Middlewares/ST/STM32_WPAN/zigbee/core/inc -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-tl

clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-tl:
	-$(RM) ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.cyclo ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.d ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.o ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl.su ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.cyclo ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.d ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.o ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/shci_tl_if.su ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.cyclo ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.d ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.o ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.su ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.cyclo ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.d ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.o ./Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_zigbee_hci.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_WPAN-2f-interface-2f-patterns-2f-ble_thread-2f-tl

