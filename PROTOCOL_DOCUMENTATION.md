# Motor Control Protocol (MCP) Test Firmware

## Project Overview

This project implements a Motor Control Protocol (MCP) test firmware for an STM32G4 Electronic Speed Controller (ESC). It provides a comprehensive suite of tools and scripts for communicating with and configuring motor control devices using the ASPEP (Asymmetric Serial Packet Exchange Protocol) communication protocol.

## Development Roadmap

### Phase 1: Core Protocol Implementation (Completed)
- ASPEP transport layer integration
- Basic MCP service scaffolding
- STM32G4 HAL configuration
- Motor control SDK integration

### Phase 2: Tooling & Validation (Current Focus)
- Python CLI tool development
- Automated protocol testing
- Serial traffic analysis
- Motor parameter validation
- Safety monitoring implementations

### Phase 3: Advanced Features
- Field-upgradeable firmware
- Runtime configurable FOC parameters
- Thermal management integration
- Predictive maintenance features

## Key Components

### Firmware
- Based on STM32 Motor Control SDK (MCSDK) v6.3.1
- Targets STM32G431 microcontroller
- Implements Field-Oriented Control (FOC) motor control

### Communication Protocols
- **ASPEP**: Low-level communication protocol
- **MCP**: High-level motor control protocol with four primary services:
  1. **Command Service**: Execute motor control commands
  2. **Registry Service**: Access internal variables and state
  3. **Datalog Service**: Monitor and log changing register values
  4. **Notification Service**: Get notified of register value changes

## Serial Traffic Logs
The serial traffic logs are stored in the `serial_traffic_logs/` directory and have descriptions of what was recorded:

1. `1_connect_disconnect_dump.txt`: Short simple connect/disconnect.
2. `2_brief_connect_press_ACK_dump.txt`: Connect to STM32 through Motor Pilot, press the ACK button on Motor Pilot, then disconnect from the chip.
3. `3_brief_run_torque_dump.txt`: Connect to STM32 through Motor Pilot, send command for torque, stop the command for torque, then disconnect.
4. `4_brief_run_velocity_dump.txt`: Connect to STM32 through Motor Pilot, send command for velocity control, then stop the velocity command, and disconnect.

### Python Tools
- Comprehensive Python scripts for device interaction
- Object-oriented command structure
- Type-safe implementations
- Extensive error handling
- Automated testing capabilities

## Project Structure

```
.
├── m-g431-esc-10-24_firmware/     # STM32 Firmware project
│   ├── Inc/                       # Header files
│   ├── Src/                       # Source files
│   └── Drivers/                   # MCU and CMSIS drivers
├── st_mcp/                        # Python MCP communication tools
│   ├── commands/                  # Structured command implementations
│   │   ├── base.py               # Base command classes
│   │   ├── system.py             # System-level commands
│   │   ├── motor.py              # Motor control commands
│   │   ├── config.py             # Configuration commands
│   │   ├── monitor.py            # Monitoring commands
│   │   └── README.md             # Command package documentation
│   ├── app.py                    # Main application script
│   ├── mcp.py                    # Motor Control Protocol implementation
│   ├── mcp_protocol.py           # Protocol-specific utilities
│   ├── session_manager.py        # Session management
│   ├── test_commands.py          # Command generation tests
│   └── test_hardware.py          # Hardware communication tests
├── serial_traffic_logs/           # Recorded communication logs
├── util_scripts/                  # Utility Python scripts
└── README.md                      # Project documentation
```

## Features

### Command Structure
- Organized command hierarchy
- Type-safe implementations
- Clear error handling
- Consistent interface

### System Commands
- Device version retrieval
- Firmware information
- Board identification
- Fault acknowledgment

### Motor Control
- Start/stop operations
- Speed/torque control
- Ramped stopping
- Mode selection

### Configuration
- Motor parameter reading
- Application settings
- FOC configuration
- Parameter validation

### Monitoring
- Real-time data acquisition
- Current measurements
- Speed and position tracking
- Temperature monitoring

## Requirements

- Python 3.7+
- NumPy
- STM32CubeIDE (for firmware development)
- USB-to-Serial or ST-Link programmer

## Setup and Usage

### Hardware Requirements
- STM32G431-based ESC board
- ST-Link v2 programmer/debugger
- 24V DC power supply
- BLDC motor with hall sensors/encoder

### Firmware Flashing
1. Connect ST-Link to SWD headers
2. Build project in STM32CubeIDE
3. Flash using OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program \
m-g431-esc-10-24_firmware/STM32CubeIDE/Debug/m-g431-esc-10-24.elf verify reset exit"
```

### Python Environment
```bash
python3 -m venv mcp-env
source mcp-env/bin/activate
pip install -r st_mcp/requirements.txt
```

### Testing
1. Test command generation:
```bash
python st_mcp/test_commands.py
```

2. Test hardware communication:
```bash
python st_mcp/test_hardware.py
```

### Basic Usage Example
```python
from mcp import MotorControlProtocol
from commands import GetVersionCommand, StartMotorCommand

with MotorControlProtocol('/dev/ttyACM0', 1843200) as mcp:
    # Get version
    cmd = GetVersionCommand()
    response = mcp.send_bytes(cmd.to_bytes())
    if response:
        version = cmd.parse_response(response)
        print(f"MCP Version: {version}")
        
    # Start motor
    cmd = StartMotorCommand(motor_id=0)
    response = mcp.send_bytes(cmd.to_bytes())
    if response and cmd.parse_response(response):
        print("Motor started successfully")
```

## Logging

- Session information is automatically logged
- Detailed error logging for debugging

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## License

Refer to the individual component licenses:
- CMSIS: ARM Limited
- STM32 HAL: ST Microelectronics
- Project-specific code: [Your License]

## References

- [ST Motor Control SDK Community](https://community.st.com/t5/stm32-mcus-motor-control)
- [CMSIS Documentation](https://arm-software.github.io/CMSIS_5/General/html/index.html)
