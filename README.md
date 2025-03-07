# Core SSDB source code

This repository contains the source code for the front and rear SSDBs on F33

## Compiling
The driver directories `FrontSSDB` and `RearSSDB` each contain a function for
initializing the inputs to the SSDB and a function for collecting inputs and 
transmitting them over CAN. Which set of functions is used in the main code is
determined by the `SSDB_TARGET` preprocessor macro, which is set either to 
`TARGET_REAR` or `TARGET_FRONT`. When compiling the code, `make` must be run 
with either `SSDB_TARGET=TARGET_REAR` or `SSDB_TARGET=TARGET_FRONT` as the 
argument. Once the code for one SSDB has been compiled, one must run
`make clean` before compiling the code for the other.

## Theory of operation
The main application code in main.c creates two tasks: a `heartbeat` task that
blinks the LED and a `collect_sensors` task that calls the corresponding
function for collecting sensor data in either driver\_rear.c or 
driver\_front.c at a regular interval. The interval at which sensor data is
collected is defined by `SSDB_REAR_LOOP_DELAY` and `SSDB_REAR_LOOP_DELAY` in
ssdb\_config.h
