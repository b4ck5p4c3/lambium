# LAM-4I4O

Endpoint-style device with four inputs and four outputs.

## Capabilities

- **Inputs:** Four non-isolated discrete inputs (0 to 12V).
- **Outputs:** Four non-isolated outputs (12V, 1A max).
- **Power:** 7-12V DC input. Logic is fused at 0.75A, IO is not fused.
- **Connectivity:** Single 8P8C connector for Modbus RTU over RS-485.

## Modbus

- **Address:** individual for each device, set in firmware.
- **Port configuration:** 115200, 8N1

### Coils Registers

- **0x0001:** Output 1 value (Read, Write)
- **0x0002:** Output 2 value (Read, Write)
- **0x0003:** Output 3 value (Read, Write)
- **0x0004:** Output 4 value (Read, Write)

### Discrete Inputs Registers

- **0x0001** Current Input 1 value (Read)
- **0x0002** Current Input 2 value (Read)
- **0x0003** Current Input 3 value (Read)
- **0x0004** Current Input 4 value (Read)

LAM-4I4O also supports latched inputs.
They hold the positive edge of the input signal until it's read.
It's useful for detecting short pulses, like button presses.

- **0x1001** Latched Input 1 value (Read)
- **0x1002** Latched Input 2 value (Read)
- **0x1003** Latched Input 3 value (Read)
- **0x1004** Latched Input 4 value (Read)
