#include <stm32g4xx.h>

#ifndef DRIVER_CONFIG_H
#define DRIVER_CONFIG_H

#ifdef TARGET_REAR
#warning "Compiling for rear"
#endif

#ifdef TARGET_FRONT
#warning "Compiling for front"
#endif

#define CAN_MAIN                    FDCAN2
#define CAN_SENSOR                  FDCAN1

#define SSDB_ANALOG_INPUT_A_PORT    GPIOB
#define SSDB_ANALOG_INPUT_A_PIN     GPIO_PIN_0
#define SSDB_ANALOG_INPUT_B_PORT    GPIOA
#define SSDB_ANALOG_INPUT_B_PIN     GPIO_PIN_7
#define SSDB_ANALOG_INPUT_L_PORT    GPIOA
#define SSDB_ANALOG_INPUT_L_PIN     GPIO_PIN_1
#define SSDB_ANALOG_INPUT_R_PORT    GPIOB
#define SSDB_ANALOG_INPUT_R_PIN     GPIO_PIN_14

#define SSDB_FRONT_LOOP_DELAY       10
#define SSDB_FRONT_LEFT_PORT        GPIOA
#define SSDB_FRONT_LEFT_PIN         GPIO_PIN_1
#define SSDB_FRONT_RIGHT_PORT       GPIOB
#define SSDB_FRONT_RIGHT_PIN        GPIO_PIN_14
#define SSDB_FRONT_BRAKE_PORT       GPIOB
#define SSDB_FRONT_BRAKE_PIN        GPIO_PIN_0
#define SSDB_FRONT_STEERING_PORT    GPIOA
#define SSDB_FRONT_STEERING_PIN     GPIO_PIN_7

#define SSDB_REAR_LOOP_DELAY        10
#define SSDB_REAR_LEFT_PORT         GPIOA
#define SSDB_REAR_LEFT_PIN          GPIO_PIN_3
#define SSDB_REAR_RIGHT_PORT        GPIOA
#define SSDB_REAR_RIGHT_PIN         GPIO_PIN_7


#endif
