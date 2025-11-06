# STM32 Motor Control Protocol (MCP)

Python implementation of ST Microelectronics' Motor Control Protocol for STM32-based motor controllers. This project provides tools and utilities for communicating with STM32 ESCs (Electronic Speed Controllers) via serial over the ASPEP (Asymmetric Serial Packet Exchange Protocol).

## Status

**Work in Progress** - This project contains Python scripts for:
- ✅ Serial communication with STM32 motor controllers
- ✅ Motor start/stop commands
- ✅ Basic velocity control
- 🟡 Monitoring and data acquisition (partial)
- ❌ Full registry service (not implemented)
- ❌ Datalog service (not implemented)

## Installation

### From PyPI (Recommended)

```bash
pip install stm32-motor-control-protocol
```

Or with uv (faster):

```bash
uv add stm32-motor-control-protocol
```

### From Source

```bash
git clone --recurse-submodules git@github.com:shanemmattner/stm32_motor_control_protocol.git
cd stm32_motor_control_protocol
uv sync
```

## Quick Start

### Hardware Requirements
- STM32G431-based ESC board
- BLDC motor with hall sensors or encoder
- 24V DC power supply
- USB serial connection

### Basic Usage

```python
from st_mcp.minimal_motor_control import MinimalMotorControl

# Connect to motor controller
controller = MinimalMotorControl(port="/dev/ttyACM0", baudrate=1843200)
controller.run()
```

This will:
1. Establish serial connection
2. Perform beacon handshake
3. Configure velocity mode
4. Turn motor on

Press Ctrl+C to stop and exit safely.

## Project Structure

```
.
├── st_mcp/
│   ├── minimal_motor_control.py      # Working example script
│   └── commands/
│       ├── motor.py                  # Motor control commands
│       └── monitor.py                # Real-time monitoring
├── PROTOCOL_DOCUMENTATION.md         # Full project overview
├── MCP_PROTOCOL_NOTES.md            # Protocol specification
└── README.md                         # This file
```

## Motor Commands

### Start/Stop
```python
from st_mcp.commands.motor import StartMotorCommand, StopMotorCommand

# Start motor
start_cmd = StartMotorCommand(motor_id=0)
response = mcp.send_bytes(start_cmd.to_bytes())
```

### Control Modes
- **Velocity Control**: Speed-based motor operation
- **Torque Control**: Current-based motor operation

### Real-Time Monitoring

```python
from st_mcp.commands.monitor import GetMonitor1Command, Monitor1Data

# Get motor data
monitor_cmd = GetMonitor1Command()
response = mcp.send_bytes(monitor_cmd.to_bytes())
data = monitor_cmd.parse_response(response)

print(f"Speed: {data.speed} RPM")
print(f"Torque: {data.torque} Nm")
print(f"Temperature: {data.temperature}°C")
```

Available monitoring data:
- **Monitor1**: Speed, torque, flux, bus voltage, temperature
- **Monitor2**: Phase currents (Ia, Ib), D-Q currents, alpha-beta currents

## Serial Communication

All commands are sent over serial at **1,843,200 baud** using the ASPEP protocol:

### Packet Structure
```
[Header: 4 bytes] [Payload: N bytes]
```

### Connection Sequence
1. Send beacon: `55 FF FF 77`
2. Receive beacon response
3. Echo response back
4. Receive confirmation
5. Send connection request: `06 00 00 60`

## Protocol Details

### Motor Control Protocol (MCP)
- 4 core services: Command, Registry, Datalog, Notification
- Currently implements: Command Service only
- Registry and Datalog services available in firmware but not yet exposed via Python

### ASPEP (Asymmetric Serial Packet Exchange Protocol)
- Point-to-point serial protocol
- 3 communication channels: Synchronous, Asynchronous, Control
- Optional 16-bit CRC checksum
- Low-level transport for MCP

See `MCP_PROTOCOL_NOTES.md` for full specification.

## Serial Traffic Logs

The original electronics repository contains recorded serial traffic from the Windows Motor Pilot application for reference:
- Basic connect/disconnect sequence
- Motor start with velocity control
- Motor stop with ramp-down

These are useful for protocol validation and reverse engineering.

## Known Issues & Limitations

1. **Registry Service**: Commands for reading/writing motor parameters not exposed
2. **Datalog Service**: Real-time data logging configuration not implemented
3. **Error Handling**: Limited error recovery in connection sequence
4. **Testing**: No automated test suite (hardware-dependent)

## Next Steps

1. Implement Registry Service commands
2. Implement Datalog Service with configurable sampling
3. Add comprehensive error handling
4. Create test harness with mock STM32 device
5. Add parameter tuning utilities

## References

### Official ST Microelectronics Documentation
- [ST Motor Control SDK Community](https://community.st.com/t5/stm32-mcus-motor-control) - Official MCSDK support and documentation
- [STM32G431 Datasheet](https://www.st.com/resource/en/datasheet/stm32g431c6.pdf) - STM32G4 MCU specifications
- [STM32G4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00355902-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) - Register and peripheral details
- [Motor Control SDK Documentation](https://www.st.com/en/development-tools/stm32cubemx.html) - STM32CubeMX with MCSDK
- [ST Motor Workbench](https://www.st.com/en/development-tools/mcsdk.html) - Motor Control SDK tools
- [MCSDK v6.3.1 Release Notes](https://www.st.com/en/embedded-software/stm32-mcu-mcsdk.html) - SDK version history and features

### ARM CMSIS Documentation
- [CMSIS Documentation](https://arm-software.github.io/CMSIS_5/General/html/index.html) - Cortex Microcontroller Software Interface Standard
- [CMSIS-Core](https://arm-software.github.io/CMSIS_5/Core/html/index.html) - Core device abstraction
- [ARM Cortex-M4 Programming Guide](https://developer.arm.com/documentation/dui0553/a) - M4 architecture reference

### Protocol & Interface Documentation
- [ASPEP Protocol Specification](https://www.st.com/content/ccc/resource/technical/document/application_note/58/ae/2e/f6/33/dc/4e/b2/CD00288121.PDF) - Asymmetric Serial Packet Exchange Protocol
- [STM32 HAL Reference Manual](https://www.st.com/resource/en/user_manual/dm00122015-description-of-stm32l0-firmware-libraries-stmicroelectronics.pdf) - Hardware Abstraction Layer
- [Universal Serial UART Guidelines](https://www.st.com/en/microcontrollers-microprocessors/stm32g4-series.html#documentation) - UART communication reference

### Related Motor Control Resources
- [Field-Oriented Control (FOC) Fundamentals](https://www.st.com/resource/en/design_tip/dm00179356-sinusoidal-pwm-modulation-for-motor-control-stmicroelectronics.pdf) - FOC theory and implementation
- [BLDC Motor Control Guide](https://www.st.com/en/applications/industrial-control/brushless-dc-motor-bldc-control.html) - BLDC control techniques
- [PWM and Motor Control](https://www.st.com/content/dam/AME/2014/white-paper/WL_211_Rev2_final.pdf) - PWM fundamentals for motor control

### Project-Specific Documentation
- `MCP_PROTOCOL_NOTES.md` - Detailed Motor Control Protocol specification extracted from firmware
- `PROTOCOL_DOCUMENTATION.md` - Full project architecture and setup guide

## License

See individual file headers for licensing information.
