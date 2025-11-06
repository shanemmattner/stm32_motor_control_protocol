# Development Guide

## Architecture

### Module Structure

```
st_mcp/
├── __init__.py           # Package exports
├── minimal_motor_control.py   # Example working implementation
└── commands/
    ├── __init__.py       # Command exports
    ├── motor.py          # Motor control commands
    └── monitor.py        # Monitoring commands
```

### Design Principles

1. **Serial Communication**: All hardware interaction is through serial/UART
2. **Protocol Abstraction**: Commands encapsulate MCP protocol details
3. **Response Parsing**: Each command handles its own response parsing
4. **Type Safety**: Type hints throughout for IDE support and mypy checking
5. **Testability**: Minimal dependencies, mockable serial interface

## Key Classes

### MinimalMotorControl

Simple working example that demonstrates:
- Serial connection management
- Connection handshake (beacon exchange)
- Command timing and sequencing
- Signal handling for clean shutdown

Use as reference or starting point for your own implementations.

### Motor Commands

Base pattern for command implementation:

```python
from st_mcp.commands import StartMotorCommand

# Create command
cmd = StartMotorCommand(motor_id=0)

# Send (requires serial connection)
response = mcp.send_bytes(cmd.to_bytes())

# Parse response
success = cmd.parse_response(response)
```

### Monitor Commands

Real-time data acquisition:

```python
from st_mcp.commands import GetMonitor1Command

cmd = GetMonitor1Command()
response = mcp.send_bytes(cmd.to_bytes())
data = cmd.parse_response(response)

if data:
    print(f"Speed: {data.speed} RPM")
    print(f"Temperature: {data.temperature}°C")
```

## Protocol Layers

### Layer 1: ASPEP (Serial Protocol)
- Packets: 4-byte header + payload
- Baud rate: 1,843,200
- CRC32 checksums (optional)
- Connection management with beacon/ping

### Layer 2: MCP (Motor Control Protocol)
- 4 services: Command, Registry, Datalog, Notification
- Currently implements: Command service only
- Motor ID + Command ID + Payload format

### Layer 3: Commands (Python API)
- Object-oriented command interface
- Automatic serialization to bytes
- Response parsing and validation

## Testing Strategy

### Unit Tests

Test command generation and response parsing without hardware:

```python
def test_start_motor_to_bytes():
    """Test StartMotorCommand serialization."""
    cmd = StartMotorCommand(motor_id=0)
    data = cmd.to_bytes()

    # Verify packet structure
    assert len(data) >= 4  # Minimum header
    assert data[0] in [0x49, 0xa9]  # Valid header byte
```

### Integration Tests

Test with actual hardware (optional, hardware-dependent):

```python
def test_hardware_connection(serial_port="/dev/ttyACM0"):
    """Test actual hardware communication."""
    controller = MinimalMotorControl(port=serial_port)
    assert controller.connect()
    assert controller.establish_connection()
    controller.serial.close()
```

### Mock Testing

Mock serial responses for isolated testing:

```python
from unittest.mock import MagicMock, patch

def test_monitor_response_parsing():
    """Test monitor data parsing with mock response."""
    cmd = GetMonitor1Command()

    # Mock response (24 bytes minimum)
    mock_response = b'\xa9\x00\x00\x50' + b'\x00' * 20

    data = cmd.parse_response(mock_response)
    assert data is not None
```

## Performance Considerations

### Serial Communication
- Baud rate: 1,843,200 bits/sec
- Typical packet: ~20-30 bytes
- Transfer time: ~150-200 microseconds per packet
- Command-response cycle: ~10-100ms depending on timing requirements

### Timing
- Beacon handshake: ~1-2 seconds
- Configuration: ~1.6 seconds (matched to firmware)
- Motor start/stop: ~100ms
- Monitor reads: ~50-100ms for single read

### Optimization Tips
1. Batch multiple commands when possible
2. Use asynchronous datalog for high-frequency monitoring
3. Minimize serial read timeouts (currently 1 second)
4. Consider caching frequently accessed registers

## Debugging

### Add Logging

```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

logger.debug(f"Sending: {cmd_bytes.hex()}")
```

### Serial Traffic Analysis

Use serial traffic logs in `serial_traffic_logs/` directory:
- Compare your output against reference captures
- Verify packet structure and timing
- Check response values

### Common Issues

1. **No response from device**
   - Check baud rate (must be 1,843,200)
   - Verify serial port name
   - Check USB cable connection
   - Ensure proper firmware is loaded

2. **Timeout errors**
   - Increase serial timeout in MinimalMotorControl
   - Check motor controller power
   - Verify communication protocol

3. **Invalid responses**
   - Compare against serial_traffic_logs
   - Check command byte order (little-endian)
   - Verify payload structure

## Future Work

### High Priority
- [ ] Registry service implementation
- [ ] Datalog service with configurable sampling
- [ ] Comprehensive error handling and recovery
- [ ] Full test suite with 80%+ coverage

### Medium Priority
- [ ] Real-time data streaming
- [ ] Parameter tuning utilities
- [ ] Motor characterization tools
- [ ] Performance profiling

### Low Priority
- [ ] GUI monitoring tool
- [ ] Data logging to CSV/HDF5
- [ ] Integration with control frameworks
- [ ] Machine learning integration

## Resources

- See `MCP_PROTOCOL_NOTES.md` for protocol details
- See `PROTOCOL_DOCUMENTATION.md` for project architecture
- Check `serial_traffic_logs/` for reference captures
- Review `CONTRIBUTING.md` for code style guidelines
