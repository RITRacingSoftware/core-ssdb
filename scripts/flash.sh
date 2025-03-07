#!/usr/bin/env bash

set -x

ELF=$1

openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
