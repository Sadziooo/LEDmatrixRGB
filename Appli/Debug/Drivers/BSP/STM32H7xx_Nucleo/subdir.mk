################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Moje_pliki/Programowanie/STM/LEDmatrixRGB/Drivers/BSP/STM32H7RSxx_Nucleo/stm32h7rsxx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.o: C:/Moje_pliki/Programowanie/STM/LEDmatrixRGB/Drivers/BSP/STM32H7RSxx_Nucleo/stm32h7rsxx_nucleo.c Drivers/BSP/STM32H7xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7S3xx -DUSE_NUCLEO_64 -c -I../Core/Inc -I../../Drivers/STM32H7RSxx_HAL_Driver/Inc -I../../Drivers/STM32H7RSxx_HAL_Driver/Inc/Legacy -I../../Drivers/BSP/STM32H7RSxx_Nucleo -I../../Drivers/CMSIS/Device/ST/STM32H7RSxx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.cyclo ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.d ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.o ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7rsxx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo
