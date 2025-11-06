# Motor Control Examples

Example scripts demonstrating the Registry Service API for STM32 motor control.

## Prerequisites

```bash
pip install pyserial numpy
```

## Available Examples

### speed_control_example.py

Complete speed control demonstration using the Registry Service.

**What it does:**
- Connects to motor controller via serial
- Checks for faults and clears them
- Reads system status (voltage, temperature)
- Configures speed mode with 500 RPM target
- Monitors real-time telemetry for 10 seconds
- Stops motor safely with ramp-down

**Usage:**
```bash
# Linux/Mac
python examples/speed_control_example.py

# If on Windows, edit PORT in script to "COM3" or appropriate port
```

**Expected output:**
```
====================================================================
STM32 Motor Control - Speed Control Example
====================================================================
Port: /dev/ttyACM0
Baudrate: 1843200
Target Speed: 500 RPM
====================================================================

✓ Connected to motor controller

Checking for faults...
✓ No faults detected

Reading system status...
  Bus Voltage: 24.0 V
  Temperature: 35.2 °C

Reading speed PID gains...
  Kp = 1000
  Ki = 50
  Kd = 0

🚀 Starting motor in speed mode: 500 RPM

⏱  Running for 10 seconds...
====================================================================
Time  | Speed | Torque | Voltage | Temp   | Power
--------------------------------------------------------------------
 0.5s |   120 |   1.25 |    24.0 |   35.2 |   50.0
 1.0s |   280 |   1.50 |    24.0 |   35.3 |   75.0
 ...
```

## Creating Your Own Examples

### Basic Motor Control

```python
from st_mcp.motor_controller import MotorController

# Connect to motor
with MotorController("/dev/ttyACM0") as motor:
    # Set speed mode and start
    motor.set_speed_mode(target_rpm=1000)

    # Monitor
    telemetry = motor.get_telemetry()
    print(f"Speed: {telemetry.speed_rpm} RPM")

    # Stop
    motor.stop(ramp=True)
```

### Reading Parameters

```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    # Read measurements
    voltage = motor.registry.read_bus_voltage()
    temp = motor.registry.read_temperature()
    speed = motor.registry.read_speed_meas()

    print(f"Voltage: {voltage:.1f}V")
    print(f"Temperature: {temp:.1f}°C")
    print(f"Speed: {speed} RPM")
```

### Tuning PID Gains

```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    # Read current gains
    kp, ki, kd = motor.registry.read_speed_pid()
    print(f"Current PID: Kp={kp}, Ki={ki}, Kd={kd}")

    # Set new gains
    motor.set_speed_pid(kp=1200, ki=60, kd=0)

    # Test with speed control
    motor.set_speed_mode(target_rpm=1000)
```

### Torque Control

```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    # Set torque mode (2.5A Iq, 0A Id)
    motor.set_torque_mode(iq_ref=2.5, id_ref=0.0)

    # Monitor current
    torque = motor.get_torque()
    print(f"Torque: {torque:.2f}A")

    motor.stop()
```

### Fault Handling

```python
from st_mcp.motor_controller import MotorController
from st_mcp.exceptions import MotorFaultError

with MotorController("/dev/ttyACM0") as motor:
    # Check for faults
    if motor.has_faults():
        fault_flags = motor.registry.read_fault_flags()
        print(f"Faults detected: 0x{fault_flags:08X}")

        # Clear faults
        motor.fault_ack()

        # Verify cleared
        if motor.has_faults():
            print("Faults persist - check motor setup")
        else:
            print("Faults cleared - ready to run")
```

## Safety Notes

⚠️ **Always follow these safety guidelines:**

1. **Secure the motor** - Ensure it cannot cause injury if it spins
2. **Start with low speeds** - Test with < 500 RPM initially
3. **Have emergency stop ready** - Be able to disconnect power quickly
4. **Monitor temperature** - Stop if heatsink exceeds safe limits
5. **Check power supply** - Verify voltage is within motor ratings

## Troubleshooting

### Motor doesn't start
- Check fault flags: `motor.registry.read_fault_flags()`
- Verify control mode: `motor.registry.read_control_mode()`
- Check speed reference is non-zero

### Communication errors
- Verify serial port is correct (`/dev/ttyACM0`, `COM3`, etc.)
- Check baudrate is 1843200 (ASPEP default)
- Ensure motor controller firmware is powered on

### Unexpected values
- All values are in engineering units (V, A, RPM, °C, W)
- Check register scaling in `st_mcp/registers.py`
- Use registry methods (not raw register access)

## Next Examples to Add

Ideas for future examples:
- Continuous telemetry logging to CSV
- PID auto-tuning procedure
- Multi-motor control (motor_id parameter)
- Custom control loop implementation
- Motor characterization and profiling

## Contributing

To add a new example:
1. Create `your_example.py` in this directory
2. Add descriptive docstring at top
3. Include usage instructions
4. Add safety notes if relevant
5. Update this README
