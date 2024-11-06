#!/usr/bin/env bash

set -x

ELF=build/stm32/spin_test-f33.elf

openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
