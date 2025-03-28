#!/usr/bin/env bash

set -x

ELF=build_rear/stm32/RSSDB-f33.elf

openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
