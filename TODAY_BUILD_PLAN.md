# Today's Build Plan - Hardware Ready Tomorrow

**Goal:** Build core registry infrastructure that can be tested with hardware tomorrow

**Timeline:** 6-8 hours of focused work

---

## What We're Building Today

### 1. Register Definitions Module ⏱️ 1-2 hours
**File:** `st_mcp/registers.py`

**Why This First:** Everything else depends on knowing what registers exist

**What It Does:**
- Defines all 60+ motor control registers
- Type information (int32_t, int16_t, uint16_t, uint8_t)
- Scaling factors for engineering units
- Register lookup by ID or name

**Test Tomorrow:** Read actual bus voltage, verify scaling

---

### 2. Frame Construction ⏱️ 1-2 hours
**File:** `st_mcp/frame.py`

**Why This:** Need proper packet framing instead of hardcoded hex

**What It Does:**
- `Frame` class for building packets
- Checksum calculation (from STM32-Motor-Control-Interface)
- ASPEP wrapper (4-byte header)
- Response parsing (ACK/NACK detection)

**Test Tomorrow:** Send read register command, verify response format

---

### 3. Registry Commands ⏱️ 1 hour
**File:** `st_mcp/commands/registry.py`

**Why This:** Low-level read/write primitives

**What It Does:**
- `ReadRegisterCommand` - Read any register by ID
- `WriteRegisterCommand` - Write any register by ID
- Error code definitions
- Response parsing

**Test Tomorrow:** Read speed_ref, write speed_ref, verify roundtrip

---

### 4. MotorRegistry Class ⏱️ 2 hours
**File:** `st_mcp/registry.py`

**Why This:** High-level API for parameters

**What It Does:**
```python
# Clean API instead of hardcoded hex
registry.read_bus_voltage()  # Returns 24.5 (Volts)
registry.write_speed_ref(1500)  # Set 1500 RPM
registry.read_speed_pid()  # Returns (Kp, Ki, Kd)
```

**Test Tomorrow:**
- Read all measurements
- Write speed setpoint
- Read it back

---

### 5. MotorController API ⏱️ 1-2 hours
**File:** `st_mcp/motor_controller.py`

**Why This:** User-friendly interface

**What It Does:**
```python
with MotorController("/dev/ttyACM0") as motor:
    motor.set_speed_mode(target_rpm=1500)
    telemetry = motor.get_telemetry()
    motor.stop()
```

**Test Tomorrow:** Run complete speed control sequence

---

### 6. Example Script ⏱️ 30 min
**File:** `examples/speed_control_example.py`

**Why This:** Ready-to-run test script

**What It Does:**
- Connect to motor
- Check faults
- Set speed mode
- Monitor telemetry
- Stop motor

**Test Tomorrow:** Just run `python examples/speed_control_example.py`

---

### 7. Basic Unit Tests ⏱️ 1 hour
**Files:** `tests/test_registers.py`, `tests/test_frame.py`

**Why This:** Catch bugs before hardware testing

**What It Does:**
- Test register encoding/decoding
- Test checksum calculation
- Test frame construction
- No mocking needed (pure logic)

**Test Today:** Run `pytest tests/`

---

## Implementation Order (Dependency Chain)

```
1. registers.py (no dependencies)
   ↓
2. frame.py (uses registers for type info)
   ↓
3. commands/registry.py (uses frame + registers)
   ↓
4. registry.py (uses registry commands)
   ↓
5. motor_controller.py (uses registry + existing transport)
   ↓
6. examples/speed_control_example.py (uses motor_controller)
   ↓
7. tests/*.py (test everything)
```

---

## What We're NOT Building Today

- ❌ Transport layer refactoring (use existing `minimal_motor_control.py` transport)
- ❌ Telemetry data classes (can use dict for now)
- ❌ Extensive documentation (add after hardware validation)
- ❌ Complete test coverage (just critical path tests)
- ❌ Monitor command refactoring (existing code works)

**Rationale:** These can wait until after we validate the registry service works with hardware

---

## Hardware Test Plan for Tomorrow

### Quick Validation (10 minutes)
```python
# Test 1: Read bus voltage
voltage = registry.read_bus_voltage()
print(f"Bus voltage: {voltage}V")  # Should be ~24V

# Test 2: Write and read speed reference
registry.write_speed_ref(1000)
speed = registry.read_speed_ref()
assert speed == 1000

# Test 3: Read PID gains
kp, ki, kd = registry.read_speed_pid()
print(f"PID: Kp={kp}, Ki={ki}, Kd={kd}")
```

### Full Speed Control Test (5 minutes)
```python
with MotorController("/dev/ttyACM0") as motor:
    motor.set_speed_mode(target_rpm=500)  # Low speed for safety
    time.sleep(5)
    print(motor.get_telemetry())
    motor.stop(ramp=True)
```

### Success Criteria
- ✅ Can read bus voltage (correct value)
- ✅ Can write speed reference (no NACK)
- ✅ Can start motor (spins at commanded speed)
- ✅ No hardcoded hex strings used
- ✅ Clean API works end-to-end

---

## Key Design Decisions

### 1. Use Existing Transport
**Decision:** Keep `minimal_motor_control.py` transport for now

**Rationale:**
- Beacon handshake already works
- Don't change too many things at once
- Refactor transport after registry validated

**Implementation:**
```python
# motor_controller.py will use existing transport temporarily
from st_mcp.minimal_motor_control import MinimalMotorControl

class MotorController:
    def __init__(self, port: str):
        self._transport = MinimalMotorControl(port)
        self.registry = MotorRegistry(self._transport)
```

### 2. ASPEP + Frame Protocol Hybrid
**Decision:** Wrap Frame Protocol commands in ASPEP packets

**Rationale:**
- ASPEP is the transport layer (4-byte header + payload)
- Frame Protocol is the application layer (register access)
- They work together, not separately

**Implementation:**
```python
def to_bytes(self) -> bytes:
    # Build Frame Protocol packet
    frame = Frame(frame_code=0x02, motor_id=0)
    frame.add_byte(register_id)

    # Wrap in ASPEP header
    aspep_header = bytes([0x49, 0x01, 0x00, 0x70])  # From working code
    return aspep_header + frame.to_bytes()
```

### 3. Register Scaling
**Decision:** Return engineering units (Volts, Amps, RPM)

**Rationale:**
- Users think in engineering units
- Hide firmware scaling factors
- More intuitive API

**Implementation:**
```python
# Firmware stores voltage as uint16_t in 0.01V units
# Registry returns float in Volts
def read_bus_voltage(self) -> float:
    raw = self._read_register(RegisterMap.BUS_VOLTAGE)
    return raw / 100.0  # Convert to Volts
```

---

## Commit Strategy

Small, incremental commits:

1. `feat(registers): Add register definitions and scaling`
2. `feat(frame): Implement frame construction and parsing`
3. `feat(commands): Add registry read/write commands`
4. `feat(registry): Add MotorRegistry high-level API`
5. `feat(controller): Add MotorController user-friendly interface`
6. `feat(examples): Add speed control example`
7. `test: Add unit tests for registers and frame`

---

## Risk Mitigation

### Risk: ASPEP header format unknown
**Mitigation:** Extract exact header bytes from working `minimal_motor_control.py`

### Risk: Checksum algorithm wrong
**Mitigation:** Port exact implementation from STM32-Motor-Control-Interface, test with known packets

### Risk: Register IDs different in MCSDK 6.x
**Mitigation:** Test with hardware tomorrow, adjust register map if needed

### Risk: Frame Protocol doesn't work over ASPEP
**Mitigation:** Capture working packet from Motor Pilot, compare our generated packets

---

## Debug Strategy for Tomorrow

### If Reading Fails
1. Print raw bytes sent/received
2. Compare to working hardcoded command
3. Check checksum calculation
4. Verify register ID from firmware

### If Writing Fails
1. Check for NACK response
2. Decode error code
3. Verify value range (min/max)
4. Check if register is read-only

### If Motor Doesn't Start
1. Read fault flags
2. Check control mode (should be SPEED=1)
3. Verify speed reference was written
4. Try fault_ack() before starting

---

## Today's Success Criteria

By end of day:
- ✅ All 7 files created and committed
- ✅ Unit tests pass
- ✅ Example script runs without import errors
- ✅ Code compiles (no syntax errors)
- ✅ Clean architecture (no hardcoded hex in new code)

Tomorrow's validation will prove it works with hardware!

---

**Let's start building! 🚀**
