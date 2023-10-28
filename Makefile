
all: out.elf

out.elf: Makefile main.cc stm32f411-bear-trial.ld
	arm-none-eabi-g++ -std=c++2b main.cc -T stm32f411-bear-trial.ld -o out.elf -I./Drivers/CMSIS/Include -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include -DHSI_VALUE=16000000 -DHSE_VALUE=8000000 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -mcpu=cortex-m4 -nostdlib -Oz -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -fno-builtin
	arm-none-eabi-objdump -h -S out.elf > out.list

run: out.elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program out.elf verify reset exit"
