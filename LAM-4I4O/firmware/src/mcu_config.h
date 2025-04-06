#ifndef MCU_CONFIG_H_
#define MCU_CONFIG_H_

#include <ch32v00x.h>

#define MODBUS_UART USART1
#define MODBUS_SPEED 115200
#define MODBUS_SLAVE_ID 230

#define TX_PORT GPIOD
#define TX_PIN GPIO_Pin_6

#define RX_PORT GPIOD
#define RX_PIN GPIO_Pin_5

#define DIR_PORT GPIOA
#define DIR_PIN GPIO_Pin_1

#define DBG_LED_PORT GPIOC
#define DBG_LED_PIN GPIO_Pin_0

#define IN1_PORT GPIOC
#define IN1_PIN GPIO_Pin_4
#define IN1_PORT_SOURCE GPIO_PortSourceGPIOC
#define IN1_PIN_SOURCE GPIO_PinSource4
#define IN1_EXTI_LINE EXTI_Line4

#define IN2_PORT GPIOC
#define IN2_PIN GPIO_Pin_3
#define IN2_PORT_SOURCE GPIO_PortSourceGPIOC
#define IN2_PIN_SOURCE GPIO_PinSource3
#define IN2_EXTI_LINE EXTI_Line3

#define IN3_PORT GPIOC
#define IN3_PIN GPIO_Pin_2
#define IN3_PORT_SOURCE GPIO_PortSourceGPIOC
#define IN3_PIN_SOURCE GPIO_PinSource2
#define IN3_EXTI_LINE EXTI_Line2

#define IN4_PORT GPIOC
#define IN4_PIN GPIO_Pin_1
#define IN4_PORT_SOURCE GPIO_PortSourceGPIOC
#define IN4_PIN_SOURCE GPIO_PinSource1
#define IN4_EXTI_LINE EXTI_Line1

#define OUT1_PORT GPIOC
#define OUT1_PIN GPIO_Pin_6

#define OUT2_PORT GPIOC
#define OUT2_PIN GPIO_Pin_7

#define OUT3_PORT GPIOD
#define OUT3_PIN GPIO_Pin_4

#define OUT4_PORT GPIOA
#define OUT4_PIN GPIO_Pin_2

#endif  // MCU_CONFIG_H
