#include <ch32v00x.h>
#include <memory.h>

#include "mcu_config.h"
#include <modbus.h>

void GPIOInitPin(GPIO_TypeDef* gpio, uint16_t pin, GPIOSpeed_TypeDef speed,
                 GPIOMode_TypeDef mode) {
  GPIO_InitTypeDef init = {0};
  init.GPIO_Pin = pin;
  init.GPIO_Speed = speed;
  init.GPIO_Mode = mode;
  GPIO_Init(gpio, &init);
}

void GPIOInit() {
  RCC_APB2PeriphClockCmd(
      RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
      ENABLE);

  GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

  GPIOInitPin(IN1_PORT, IN1_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  GPIOInitPin(IN2_PORT, IN2_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  GPIOInitPin(IN3_PORT, IN3_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  GPIOInitPin(IN4_PORT, IN4_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);

  GPIOInitPin(OUT1_PORT, OUT1_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  GPIOInitPin(OUT2_PORT, OUT2_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  GPIOInitPin(OUT3_PORT, OUT3_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  GPIOInitPin(OUT4_PORT, OUT4_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
}

void InterruptInit() {
  EXTI_InitTypeDef exti_init;
  GPIO_EXTILineConfig(IN1_PORT_SOURCE, IN1_PIN_SOURCE);
  GPIO_EXTILineConfig(IN2_PORT_SOURCE, IN2_PIN_SOURCE);
  GPIO_EXTILineConfig(IN3_PORT_SOURCE, IN3_PIN_SOURCE);
  GPIO_EXTILineConfig(IN4_PORT_SOURCE, IN4_PIN_SOURCE);

  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  exti_init.EXTI_LineCmd = ENABLE;

  exti_init.EXTI_Line = IN1_EXTI_LINE;
  EXTI_Init(&exti_init);
  exti_init.EXTI_Line = IN2_EXTI_LINE;
  EXTI_Init(&exti_init);
  exti_init.EXTI_Line = IN3_EXTI_LINE;
  EXTI_Init(&exti_init);
  exti_init.EXTI_Line = IN4_EXTI_LINE;
  EXTI_Init(&exti_init);

  NVIC_InitTypeDef nvic_init;
  nvic_init.NVIC_IRQChannel = EXTI7_0_IRQn;
  nvic_init.NVIC_IRQChannelPreemptionPriority = 1;
  nvic_init.NVIC_IRQChannelSubPriority = 2;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
}

void Init() {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  GPIOInit();
  InterruptInit();
  ModbusInit();
}

bool in1_rising_latch = 0;
bool in2_rising_latch = 0;
bool in3_rising_latch = 0;
bool in4_rising_latch = 0;

bool in1_falling_latch = 0;
bool in2_falling_latch = 0;
bool in3_falling_latch = 0;
bool in4_falling_latch = 0;

void EXTI7_0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));

void EXTI7_0_IRQHandler() {
  if (EXTI_GetITStatus(IN1_EXTI_LINE) != RESET) {
    if (GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN)) {
      in1_rising_latch = true;
    } else {
      in1_falling_latch = true;
    }
    EXTI_ClearITPendingBit(IN1_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN2_EXTI_LINE) != RESET) {
    if (GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN)) {
      in2_rising_latch = true;
    } else {
      in2_falling_latch = true;
    }
    EXTI_ClearITPendingBit(IN2_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN3_EXTI_LINE) != RESET) {
    if (GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN)) {
      in3_rising_latch = true;
    } else {
      in3_falling_latch = true;
    }
    EXTI_ClearITPendingBit(IN3_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN4_EXTI_LINE) != RESET) {
    if (GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN)) {
      in4_rising_latch = true;
    } else {
      in4_falling_latch = true;
    }
    EXTI_ClearITPendingBit(IN4_EXTI_LINE);
  }
}

bool ModbusIsReadDiscreteInputValidExt(uint16_t address) {
  if (address >= 0x1000 && address <= 0x1003) {
    return true;
  }
  if (address >= 0x2000 && address <= 0x2003) {
    return true;
  }
  return false;
}

void ModbusWriteCoil(uint16_t coil, bool set) {
  switch (coil) { 
    case 0:
      GPIO_WriteBit(OUT1_PORT, OUT1_PIN, set ? Bit_SET : Bit_RESET);
      break;
    case 1:
      GPIO_WriteBit(OUT2_PORT, OUT2_PIN, set ? Bit_SET : Bit_RESET);
      break;
    case 2:
      GPIO_WriteBit(OUT3_PORT, OUT3_PIN, set ? Bit_SET : Bit_RESET);
      break;
    case 3:
      GPIO_WriteBit(OUT4_PORT, OUT4_PIN, set ? Bit_SET : Bit_RESET);
      break;
  }
}

bool ModbusReadDiscreteInput(uint16_t input) {
  switch (input) {  // NOLINT(hicpp-multiway-paths-covered)
    case 0: {
      return GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN);
    }
    case 1: {
      return GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN);
    }
    case 2: {
      return GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN);
    }
    case 3: {
      return GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN);
    }
    // falling latches
    case 0x1000: {
      if (in1_falling_latch) {
        in1_falling_latch = false;
        return 1;
      }
      return GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN);
    }
    case 0x1001: {
      if (in2_falling_latch) {
        in2_falling_latch = false;
        return 1;
      }
      return GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN);
    }
    case 0x1002: {
      if (in3_falling_latch) {
        in3_falling_latch = false;
        return 1;
      }
      return GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN);
    }
    case 0x1003: {
      if (in4_falling_latch) {
        in4_falling_latch = false;
        return 1;
      }
      return GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN);
    }
    // rising latches
    case 0x2000: {
      if (in1_rising_latch) {
        in1_rising_latch = false;
        return 0;
      }
      return GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN);
    }
    case 0x2001: {
      if (in2_rising_latch) {
        in2_rising_latch = false;
        return 0;
      }
      return GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN);
    }
    case 0x2002: {
      if (in3_rising_latch) {
        in3_rising_latch = false;
        return 0;
      }
      return GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN);
    }
    case 0x2003: {
      if (in4_rising_latch) {
        in4_rising_latch = false;
        return 0;
      }
      return GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN);
    }
  }
  return 0;
}

int main(void) {
  Init();

  while (1) {
    ModbusProcess();
  }
}
