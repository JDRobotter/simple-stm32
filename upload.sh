#!/bin/bash

ELF=./target/thumbv6m-none-eabi/release/simple-stm32
HEX=$ELF.hex

arm-none-eabi-size $ELF
arm-none-eabi-objcopy -Oihex $ELF $HEX
stm32flash -b921600 -w $HEX -i "rts&-dtr,,-rts&dtr,," -v -g 0 /dev/ttyUSB0 
