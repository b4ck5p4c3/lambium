#include <ch32v00x.h>
#include <memory.h>

#include "mcu_config.h"
#include "modbus.h"

void InitGPIOPin(GPIO_TypeDef* gpio, uint16_t pin, GPIOSpeed_TypeDef speed,
                 GPIOMode_TypeDef mode) {
  GPIO_InitTypeDef init = {0};
  init.GPIO_Pin = pin;
  init.GPIO_Speed = speed;
  init.GPIO_Mode = mode;
  GPIO_Init(gpio, &init);
}

void InitGPIO(void) {
  RCC_APB2PeriphClockCmd(
      RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
      ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_PA1_2, DISABLE);
  GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

  InitGPIOPin(TX_PORT, TX_PIN, GPIO_Speed_50MHz, GPIO_Mode_AF_PP);
  InitGPIOPin(RX_PORT, RX_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPU);
  InitGPIOPin(DIR_PORT, DIR_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_RESET);

  InitGPIOPin(IN1_PORT, IN1_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  InitGPIOPin(IN2_PORT, IN2_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  InitGPIOPin(IN3_PORT, IN3_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);
  InitGPIOPin(IN4_PORT, IN4_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPD);

  InitGPIOPin(OUT1_PORT, OUT1_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  InitGPIOPin(OUT2_PORT, OUT2_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  InitGPIOPin(OUT3_PORT, OUT3_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  InitGPIOPin(OUT4_PORT, OUT4_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);

  InitGPIOPin(DBG_LED_PORT, DBG_LED_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
}

void InitUART() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  USART_InitTypeDef uart;
  uart.USART_BaudRate = MODBUS_SPEED;
  uart.USART_WordLength = USART_WordLength_8b;
  uart.USART_StopBits = USART_StopBits_1;
  uart.USART_Parity = USART_Parity_No;
  uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(MODBUS_UART, &uart);
  USART_Cmd(MODBUS_UART, ENABLE);
}

void InitButtonInterrupt() {
  EXTI_InitTypeDef exti_init;
  GPIO_EXTILineConfig(IN1_PORT_SOURCE, IN1_PIN_SOURCE);
  GPIO_EXTILineConfig(IN2_PORT_SOURCE, IN2_PIN_SOURCE);
  GPIO_EXTILineConfig(IN3_PORT_SOURCE, IN3_PIN_SOURCE);
  GPIO_EXTILineConfig(IN4_PORT_SOURCE, IN4_PIN_SOURCE);

  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
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
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  InitGPIO();
  InitButtonInterrupt();
  InitUART();
}

uint8_t in1_latch = 0;
uint8_t in2_latch = 0;
uint8_t in3_latch = 0;
uint8_t in4_latch = 0;

void EXTI7_0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));

void EXTI7_0_IRQHandler() {
  if (EXTI_GetITStatus(IN1_EXTI_LINE) != RESET) {
    in1_latch = 1;
    EXTI_ClearITPendingBit(IN1_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN2_EXTI_LINE) != RESET) {
    in2_latch = 1;
    EXTI_ClearITPendingBit(IN2_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN3_EXTI_LINE) != RESET) {
    in3_latch = 1;
    EXTI_ClearITPendingBit(IN3_EXTI_LINE);
  }
  if (EXTI_GetITStatus(IN4_EXTI_LINE) != RESET) {
    in4_latch = 1;
    EXTI_ClearITPendingBit(IN4_EXTI_LINE);
  }
}

void SetOut1(uint8_t set) {
  GPIO_WriteBit(OUT1_PORT, OUT1_PIN, set ? Bit_SET : Bit_RESET);
}

void SetOut2(uint8_t set) {
  GPIO_WriteBit(OUT2_PORT, OUT2_PIN, set ? Bit_SET : Bit_RESET);
}

void SetOut3(uint8_t set) {
  GPIO_WriteBit(OUT3_PORT, OUT3_PIN, set ? Bit_SET : Bit_RESET);
}

void SetOut4(uint8_t set) {
  GPIO_WriteBit(OUT4_PORT, OUT4_PIN, set ? Bit_SET : Bit_RESET);
}

#define MAX_INPUTS 4
#define MAX_COILS 4

uint8_t IsReadValid(uint32_t start, uint32_t length) {
  if (length == 0) {
    return 0;
  }
  uint32_t end = start + length - 1;
  if (start >= 1 && end <= 4) {
    return 1;
  }
  if (start >= 0x1001 && end <= 0x1004) {
    return 1;
  }
  return 0;
}

uint8_t IsWriteValid(uint32_t start, uint32_t length) {
  if (length == 0) {
    return 0;
  }
  uint32_t end = start + length - 1;
  if (start >= 1 && end <= 4) {
    return 1;
  }
  return 0;
}

void SetCoil(uint16_t coil, uint8_t set) {
  switch (coil) {  // NOLINT(hicpp-multiway-paths-covered)
    case 1:
      SetOut1(set);
      break;
    case 2:
      SetOut2(set);
      break;
    case 3:
      SetOut3(set);
      break;
    case 4:
      SetOut4(set);
      break;
  }
}

uint8_t GetInput(uint16_t input) {
  switch (input) {  // NOLINT(hicpp-multiway-paths-covered)
    case 1: {
      return GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN);
    }
    case 2: {
      return GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN);
    }
    case 3: {
      return GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN);
    }
    case 4: {
      return GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN);
    }
    case 0x1001: {
      uint8_t result = in1_latch;
      in1_latch = 0;
      return result || GPIO_ReadInputDataBit(IN1_PORT, IN1_PIN);
    }
    case 0x1002: {
      uint8_t result = in2_latch;
      in2_latch = 0;
      return result || GPIO_ReadInputDataBit(IN2_PORT, IN2_PIN);
    }
    case 0x1003: {
      uint8_t result = in3_latch;
      in3_latch = 0;
      return result || GPIO_ReadInputDataBit(IN3_PORT, IN3_PIN);
    }
    case 0x1004: {
      uint8_t result = in4_latch;
      in4_latch = 0;
      return result || GPIO_ReadInputDataBit(IN4_PORT, IN4_PIN);
    }
  }
  return 0;
}

void SendModbusException(uint8_t function, uint8_t exception_code) {
  uint8_t response[3] = {MODBUS_SLAVE_ID, function | 0x80, exception_code};
  SendModbusPacket(response, sizeof(response));
}

void SendModbusReadDiscreteInputsResponse(uint8_t* data, uint8_t bytes) {
  uint8_t response[1 + 1 + 1 + (MAX_INPUTS + 7) / 8] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS, bytes};
  memcpy(response + 3, data, bytes);
  SendModbusPacket(response, 1 + 1 + 1 + bytes);
}

void SendModbusWriteSingleCoilResponse(uint16_t address, uint16_t value) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL,
      address >> 8,    address & 0xFF,
      value >> 8,      value & 0xFF};
  SendModbusPacket(response, sizeof(response));
}

void SendModbusWriteMultipleCoilResponse(uint16_t address, uint16_t number) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS,
      address >> 8,    address & 0xFF,
      number >> 8,     number & 0xFF};
  SendModbusPacket(response, sizeof(response));
}

void ProcessModbus() {
  ModbusPacket packet;
  enum ModbusReadError error = ReadModbusPacket(&packet);
  switch (error) {
    case MODBUS_READ_ERROR_WRONG_PACKET_SIZE:
    case MODBUS_READ_ERROR_WRONG_CRC:
      return;
    case MODBUS_READ_ERROR_UNSUPPORTED_PACKET:
      if (packet.slave_id == MODBUS_SLAVE_ID) {
        SendModbusException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_FUNCTION);
      }
      return;
    case MODBUS_READ_ERROR_SUCCESS:
      break;
  }

  if (packet.slave_id != MODBUS_SLAVE_ID) {
    return;
  }

  switch (packet.function) {
    case MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS:
      if (!IsReadValid(packet.read_discrete_inputs_data.address,
                           packet.read_discrete_inputs_data.number)) {
        SendModbusException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }
      uint8_t response_data[(MAX_INPUTS + 7) / 8] = {};
      for (uint32_t i = 0; i < packet.read_discrete_inputs_data.number; i++) {
        response_data[i / 8] |=
            GetInput(i + packet.read_discrete_inputs_data.address) << (i % 8);
      }
      SendModbusReadDiscreteInputsResponse(
          response_data, (packet.read_discrete_inputs_data.number + 7) / 8);
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL:
      if (!IsWriteValid(
              packet.write_single_coil_data.address,
              1)) {
        SendModbusException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }
      SetCoil(packet.write_single_coil_data.address,
              packet.write_single_coil_data.value == 0xFF00);
      SendModbusWriteSingleCoilResponse(packet.write_single_coil_data.address,
                                        packet.write_single_coil_data.value);
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS:
      if (!IsWriteValid(packet.write_multiple_coils_data.address,
                        packet.write_multiple_coils_data.number)) {
        SendModbusException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }
      if ((packet.write_multiple_coils_data.number + 7) / 8 !=
          packet.write_multiple_coils_data.bytes) {
        SendModbusException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }
      for (uint32_t i = 0; i < packet.write_multiple_coils_data.number; i++) {
        SetCoil(packet.write_multiple_coils_data.address + i,
                (packet.write_multiple_coils_data.data[i / 8] >> (i % 8)) & 1);
      }
      SendModbusWriteMultipleCoilResponse(
          packet.write_multiple_coils_data.address,
          packet.write_multiple_coils_data.number);
      break;
  }
}

int main(void) {
  Init();

  while (1) {
    ProcessModbus();
  }
}
