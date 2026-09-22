################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/DEBUG/debug_rtt.c \
../src/DEBUG/exception_debug.c 

OBJS += \
./src/DEBUG/debug_rtt.o \
./src/DEBUG/exception_debug.o 

C_DEPS += \
./src/DEBUG/debug_rtt.d \
./src/DEBUG/exception_debug.d 


# Each subdirectory must supply rules for building sources it contributes
src/DEBUG/%.o: ../src/DEBUG/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/DEBUG/debug_rtt.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


