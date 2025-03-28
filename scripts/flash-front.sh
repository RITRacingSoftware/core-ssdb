#!/usr/bin/env bash

set -x

ELF=build_front/stm32/FSSDB-f33.elf

openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
