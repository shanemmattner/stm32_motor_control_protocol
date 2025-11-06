# Hardware Test Checklist - Registry Service Validation

**Date:** Tomorrow
**Branch:** `feature/registry-service-implementation`
**Commit:** `5f093d9`

---

## What We Built Today

### Core Modules (All Tested)
✅ **st_mcp/registers.py** - 60+ register definitions with scaling
✅ **st_mcp/frame.py** - Frame Communication Protocol implementation
✅ **st_mcp/commands/registry.py** - Read/Write register commands
✅ **st_mcp/registry.py** - High-level parameter API
✅ **st_mcp/motor_controller.py** - User-friendly interface
✅ **st_mcp/exceptions.py** - Error handling

### Tests
✅ **42 unit tests passing** (registers, frames, commands)

### Example
✅ **examples/speed_control_example.py** - Ready to run

---

## Quick Hardware Validation (10 minutes)

### Test 1: Read Bus Voltage
```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    voltage = motor.registry.read_bus_voltage()
    print(f"Bus voltage: {voltage:.1f}V")  # Should be ~24V
```

**Expected:** Voltage reading around 24V
**Success Criteria:** No exceptions, reasonable voltage value

---

### Test 2: Write and Read Speed Reference
```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    # Write 1000 RPM
    motor.registry.write_speed_ref(1000)

    # Read back
    speed = motor.registry.read_speed_ref()
    print(f"Speed setpoint: {speed} RPM")

    assert speed == 1000, "Speed setpoint mismatch!"
```

**Expected:** Speed reads back as 1000 RPM
**Success Criteria:** Write succeeds, readback matches

---

### Test 3: Read PID Gains
```python
from st_mcp.motor_controller import MotorController

with MotorController("/dev/ttyACM0") as motor:
    kp, ki, kd = motor.registry.read_speed_pid()
    print(f"Speed PID: Kp={kp}, Ki={ki}, Kd={kd}")
```

**Expected:** PID gains are read successfully
**Success Criteria:** No exceptions, reasonable gain values

---

## Full Speed Control Test (5 minutes)

Run the example script:

```bash
python examples/speed_control_example.py
```

**What It Does:**
1. Connects to motor controller
2. Checks for faults
3. Reads bus voltage and temperature
4. Sets speed mode: 500 RPM
5. Monitors telemetry for 10 seconds
6. Stops motor with ramp

**Expected Output:**
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
 1.5s |   420 |   1.35 |    24.0 |   35.4 |   65.0
 2.0s |   500 |   1.20 |    24.0 |   35.5 |   60.0
 ...
```

**Success Criteria:**
- ✅ Motor starts and spins at ~500 RPM
- ✅ Telemetry updates every 0.5 seconds
- ✅ No faults during operation
- ✅ Motor stops smoothly at end
- ✅ No hardcoded hex commands used!

---

## What Changed from Old Code

### Before (hardcoded hex strings):
```python
self.send_command("D9 00 00 40 08 00 29 05 07 00 00 08 00 00 FF 00")
self.send_command("E9 02 00 A0 11 00 19 00 59 00 59 1B 99 00 91 02")
```
❌ Opaque
❌ Non-portable
❌ Can't change parameters
❌ No error handling

### After (registry API):
```python
motor.registry.write_control_mode(MotorControlMode.SPEED)
motor.registry.write_speed_ref(1500)
motor.start()
```
✅ Readable
✅ Portable
✅ Easy to change parameters
✅ Proper error handling

---

## Troubleshooting Guide

### Issue: Connection Fails
**Symptoms:** `MotorControlError: Transport not connected`

**Check:**
1. Serial port correct? (`/dev/ttyACM0` on Linux, `COM3` on Windows)
2. Device powered on?
3. USB cable connected?
4. Correct baudrate? (1843200 for ASPEP)

**Fix:**
```python
# Try different port
motor = MotorController("/dev/ttyACM1")  # or /dev/ttyUSB0
```

---

### Issue: NACK Responses
**Symptoms:** `RegistryNACKError: Register read failed: BAD_CRC`

**Possible Causes:**
1. Incorrect register ID
2. Communication corruption
3. Firmware version mismatch

**Debug:**
```python
try:
    voltage = motor.registry.read_bus_voltage()
except RegistryNACKError as e:
    print(f"Error code: {e.error_code}")
    print(f"Error: {e}")
```

**Error Codes:**
- `BAD_CRC (0x0A)`: Checksum mismatch
- `SET_READ_ONLY (0x02)`: Trying to write read-only register
- `GET_WRITE_ONLY (0x03)`: Trying to read write-only register
- `WRONG_SET (0x05)`: Invalid value for register
- `BAD_MOTOR (0x0B)`: Motor ID not valid

---

### Issue: Motor Doesn't Start
**Symptoms:** start() called but motor doesn't spin

**Check:**
1. Faults present?
```python
if motor.has_faults():
    faults = motor.registry.read_fault_flags()
    print(f"Faults: 0x{faults:08X}")
    motor.fault_ack()
```

2. Control mode set correctly?
```python
mode = motor.registry.read_control_mode()
print(f"Control mode: {mode}")  # Should be SPEED=1
```

3. Speed reference non-zero?
```python
speed_ref = motor.registry.read_speed_ref()
print(f"Speed ref: {speed_ref} RPM")
```

---

### Issue: Register Values Look Wrong
**Symptoms:** Voltage shows 2400 instead of 24.0

**Problem:** Scaling not applied

**Check:**
```python
reg = RegisterMap.BUS_VOLTAGE
print(f"Register: {reg.name}")
print(f"Scaling: {reg.scaling}")
print(f"Unit: {reg.unit}")
```

**Fix:** Use registry methods, not raw register reads:
```python
# ✅ Correct (automatic scaling)
voltage = motor.registry.read_bus_voltage()  # Returns 24.0

# ❌ Wrong (raw value)
cmd = ReadRegisterCommand(RegisterMap.BUS_VOLTAGE)
raw = cmd.parse_response(response)  # Returns 2400
```

---

## Register ID Reference (Quick Lookup)

### Control
- `0x01` - Fault flags (read-only)
- `0x02` - Status (read-only)
- `0x03` - Control mode (0=Torque, 1=Speed)

### Speed Control
- `0x04` - Speed reference (RPM)
- `0x05` - Speed Kp
- `0x06` - Speed Ki
- `0x07` - Speed Kd
- `0x1E` - Speed measured (read-only)
- `0x5B` - Ramp final speed

### Torque Control
- `0x08` - Torque reference (Iq, Amps * 10000)
- `0x09` - Torque Kp
- `0x0A` - Torque Ki
- `0x1F` - Torque measured (read-only)

### Flux Control
- `0x0C` - Flux reference (Id, Amps * 10000)
- `0x0D` - Flux Kp
- `0x0E` - Flux Ki
- `0x20` - Flux measured (read-only)

### Measurements
- `0x19` - Bus voltage (V * 100)
- `0x1A` - Heatsink temp (°C * 10)
- `0x1B` - Motor power (W)

---

## Next Steps After Validation

### If All Tests Pass ✅
1. Commit hardware test results
2. Update README with new API examples
3. Create PR to merge into main
4. Close issue #1

### If Tests Fail ❌
1. Document failure mode
2. Add debug logging
3. Capture serial traffic
4. Compare with Motor Pilot behavior
5. Iterate on fixes

---

## Performance Expectations

**Register Read Latency:** < 10ms per register
**Register Write Latency:** < 10ms per register
**Telemetry Update Rate:** ~10 Hz (100ms per cycle)
**Command Response Time:** < 50ms

If performance significantly worse:
- Check serial baudrate (should be 1843200)
- Look for communication errors
- Verify timeout settings

---

## Success Declaration

✅ **Registry Service is validated** when:
1. Can read bus voltage (correct value)
2. Can write speed reference (no NACK)
3. Can start motor (spins at commanded speed)
4. No hardcoded hex strings used
5. Clean API works end-to-end
6. Example script runs successfully

At that point, we have achieved:
- Full register access via clean API
- Proper parameter control with units
- Real-time telemetry monitoring
- Complete replacement of hardcoded commands

**This is the goal we set out to achieve!** 🎯

---

## Safety Reminders

⚠️ **Before Testing:**
- Secure motor (cannot cause injury if it spins)
- Start with LOW speeds (< 500 RPM)
- Have emergency stop ready
- Check power supply voltage
- Verify motor connections

⚠️ **During Testing:**
- Monitor temperature
- Watch for unusual sounds
- Be ready to disconnect power
- Don't exceed rated motor speed

⚠️ **If Something Goes Wrong:**
1. Disconnect power immediately
2. Check fault flags: `motor.registry.read_fault_flags()`
3. Review serial traffic for errors
4. Don't retry without understanding failure

---

**Good luck with hardware testing! 🚀**
