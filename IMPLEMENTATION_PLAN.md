# Registry Service Implementation Plan

**Branch:** `feature/registry-service-implementation`
**Issue:** #1 - Investigate integrating register access from STM32-Motor-Control-Interface
**Timeline:** 10-16 days estimated

---

## Overview

Transform the library from hardcoded hex commands to a robust, production-ready motor control interface by implementing the Registry Service for parameter read/write operations.

**Current State:**
- ✅ ASPEP protocol transport layer
- ✅ Beacon handshake for connection
- ✅ Basic monitor commands
- ❌ Hardcoded hex strings in `minimal_motor_control.py`
- ❌ Registry Service (parameter read/write)
- ❌ Command classes not integrated

**Target State:**
- ✅ Clean API: `motor.set_speed(1500)`
- ✅ Full parameter control via Registry Service
- ✅ All command classes integrated
- ✅ Real-time telemetry with engineering units
- ✅ Comprehensive examples

---

## Phase 1: Foundation & Code Analysis

### Step 1.1: Audit Current Codebase
**Goal:** Understand what we have and what needs refactoring

**Tasks:**
- [ ] Map all existing files and their purposes
  - [ ] `st_mcp/minimal_motor_control.py` - Working script (hardcoded)
  - [ ] `st_mcp/commands/motor.py` - Structured commands (unused)
  - [ ] `st_mcp/commands/monitor.py` - Monitor commands
  - [ ] `st_mcp/commands/base.py` - Base command class (if exists)
- [ ] Document all hardcoded hex commands in `minimal_motor_control.py`
  - [ ] Connection sequence: `55 FF FF 77`, `06 00 00 60`
  - [ ] Configuration commands: `D9 00 00 40 08 00 29 05 07 00 00 08 00 00 FF 00`
  - [ ] Velocity commands: `E9 02 00 A0 11 00 19 00 59 00 59 1B 99 00 91 02`
- [ ] Identify which commands map to which operations
- [ ] List missing command types from STM32-Motor-Control-Interface

**Deliverables:**
- `docs/current_state_audit.md` - Complete inventory
- `docs/hex_command_mapping.md` - What each hex string does

**Time Estimate:** 2-3 hours

---

### Step 1.2: Analyze STM32-Motor-Control-Interface Protocol
**Goal:** Extract reusable patterns from reference implementation

**Tasks:**
- [ ] Study Frame Communication Protocol structure
  - [ ] Header format: `(motor_id << 5) | (frame_code & 0x1F)`
  - [ ] Payload length calculation
  - [ ] Checksum algorithm: `sum(bytes) + carry`
- [ ] Document all frame codes
  - [ ] `0x01` - FRAME_CODE_SET_REG
  - [ ] `0x02` - FRAME_CODE_GET_REG
  - [ ] `0x03` - FRAME_CODE_EXECUTE_CMD
  - [ ] `0x06` - FRAME_CODE_GET_BOARD_INFO
  - [ ] `0x07` - FRAME_CODE_SET_RAMP
  - [ ] `0x0A` - FRAME_CODE_SET_CURRENT_REF
  - [ ] `0x0C` - FRAME_CODE_GET_FW_VERSION
- [ ] Extract all register definitions from `frame_communication_protocol.h`
  - [ ] Speed control registers (0x04-0x07, 0x5B, 0x1E)
  - [ ] Torque/Flux registers (0x08-0x0F, 0x1F, 0x20)
  - [ ] Measurement registers (0x19-0x30)
  - [ ] Control registers (0x01-0x03)
- [ ] Document register data types and sizes
- [ ] Study read/write register functions in `mc_interface.py`

**Deliverables:**
- `docs/protocol_analysis.md` - Frame protocol deep dive
- `st_mcp/registers.py` - Register definitions (initial version)

**Time Estimate:** 3-4 hours

---

### Step 1.3: Verify ASPEP Compatibility
**Goal:** Confirm Frame Protocol commands work over ASPEP transport

**Tasks:**
- [ ] Research ASPEP packet structure
  - [ ] 4-byte header format
  - [ ] Payload encapsulation
  - [ ] CRC calculation (if used)
- [ ] Determine if ASPEP carries Frame Protocol commands directly
  - [ ] Review `MCP_PROTOCOL_NOTES.md`
  - [ ] Check if registry commands use same frame codes
  - [ ] Identify any protocol translation needed
- [ ] Test hypothesis with actual hardware (if available)
  - [ ] Send Frame Protocol GET_REG over ASPEP
  - [ ] Verify response format
  - [ ] Document any differences

**Deliverables:**
- `docs/aspep_compatibility.md` - Protocol translation guide
- Decision: Direct port vs. adaptation layer needed

**Time Estimate:** 2-3 hours

---

## Phase 2: Core Infrastructure

### Step 2.1: Create Register Definitions Module
**Goal:** Central registry of all motor control parameters

**Tasks:**
- [ ] Create `st_mcp/registers.py`
- [ ] Define `Register` dataclass
  ```python
  @dataclass
  class Register:
      id: int
      name: str
      dtype: type  # int32_t, int16_t, uint16_t, uint8_t
      size: int  # bytes
      scaling: float = 1.0
      unit: str = ""
      description: str = ""
      min_value: Optional[int] = None
      max_value: Optional[int] = None
  ```
- [ ] Define all registers from `frame_communication_protocol.h`
  - [ ] Control registers (REG_CONTROL_MODE, REG_STATUS, etc.)
  - [ ] Speed control registers
  - [ ] Torque/Flux control registers
  - [ ] Measurement registers
  - [ ] PID gain registers
- [ ] Create register groups/categories
  ```python
  class RegisterMap:
      CONTROL_MODE = Register(0x03, "control_mode", uint8_t, 1, ...)
      SPEED_REF = Register(0x04, "speed_ref", int32_t, 4, ...)
      # ... etc
  ```
- [ ] Add helper functions
  - [ ] `get_register_by_id(reg_id: int) -> Register`
  - [ ] `get_register_by_name(name: str) -> Register`
  - [ ] `encode_value(reg: Register, value: float) -> int`
  - [ ] `decode_value(reg: Register, raw: int) -> float`

**Deliverables:**
- `st_mcp/registers.py` - Complete register definitions
- Unit tests: `tests/test_registers.py`

**Time Estimate:** 4-6 hours

---

### Step 2.2: Implement Frame Construction
**Goal:** Build proper packet framing for ASPEP

**Tasks:**
- [ ] Create `st_mcp/frame.py`
- [ ] Implement `Frame` class
  ```python
  class Frame:
      def __init__(self, frame_code: int, motor_id: int = 0):
          self.frame_code = frame_code
          self.motor_id = motor_id
          self.payload = bytearray()

      def add_byte(self, value: int):
          """Add single byte to payload"""

      def add_bytes(self, data: bytes):
          """Add multiple bytes to payload"""

      def add_register_value(self, value: int, size: int):
          """Add register value in little-endian format"""

      def compute_checksum(self) -> int:
          """Calculate frame checksum"""

      def to_bytes(self) -> bytes:
          """Serialize frame to bytes"""
  ```
- [ ] Implement checksum algorithm from STM32-Motor-Control-Interface
  ```python
  def compute_checksum(self) -> int:
      accumulator = self.header + len(self.payload)
      accumulator += sum(self.payload)
      checksum = (accumulator & 0xFF) + ((accumulator >> 8) & 0xFF)
      return checksum & 0xFF
  ```
- [ ] Add ASPEP header wrapper
  ```python
  def wrap_aspep(frame: bytes) -> bytes:
      """Wrap frame in ASPEP packet structure"""
      # Add 4-byte ASPEP header
      # Add CRC if needed
  ```
- [ ] Implement frame parsing
  ```python
  class FrameResponse:
      def __init__(self, raw_bytes: bytes):
          self.ack = raw_bytes[0]  # 0xF0 = ACK, 0xFF = NACK
          self.error_code = raw_bytes[1] if self.is_nack else 0
          self.payload = raw_bytes[2:-1] if self.is_ack else b""
          self.checksum = raw_bytes[-1]

      @property
      def is_ack(self) -> bool:
          return self.ack == 0xF0

      @property
      def is_nack(self) -> bool:
          return self.ack == 0xFF

      def validate_checksum(self) -> bool:
          """Verify response checksum"""
  ```

**Deliverables:**
- `st_mcp/frame.py` - Frame construction and parsing
- Unit tests: `tests/test_frame.py`

**Time Estimate:** 4-6 hours

---

### Step 2.3: Implement Base Registry Commands
**Goal:** Low-level read/write register primitives

**Tasks:**
- [ ] Create `st_mcp/commands/registry.py`
- [ ] Implement `ReadRegisterCommand`
  ```python
  class ReadRegisterCommand(MotorCommand):
      def __init__(self, register: Register, motor_id: int = 0):
          super().__init__(command_id=0x0002, motor_id=motor_id)
          self.register = register

      def to_bytes(self) -> bytes:
          frame = Frame(frame_code=0x02, motor_id=self.motor_id)
          frame.add_byte(self.register.id)
          return frame.to_bytes()

      def parse_response(self, response: bytes) -> Any:
          frame_resp = FrameResponse(response)
          if frame_resp.is_ack:
              raw_value = int.from_bytes(
                  frame_resp.payload,
                  byteorder='little',
                  signed=self.register.dtype in [int32_t, int16_t]
              )
              return self.register.decode_value(raw_value)
          else:
              raise RegistryError(f"NACK: {frame_resp.error_code}")
  ```
- [ ] Implement `WriteRegisterCommand`
  ```python
  class WriteRegisterCommand(MotorCommand):
      def __init__(self, register: Register, value: Any, motor_id: int = 0):
          super().__init__(command_id=0x0001, motor_id=motor_id)
          self.register = register
          self.value = value

      def to_bytes(self) -> bytes:
          frame = Frame(frame_code=0x01, motor_id=self.motor_id)
          frame.add_byte(self.register.id)
          raw_value = self.register.encode_value(self.value)
          frame.add_register_value(raw_value, self.register.size)
          return frame.to_bytes()

      def parse_response(self, response: bytes) -> bool:
          frame_resp = FrameResponse(response)
          if frame_resp.is_nack:
              raise RegistryError(f"NACK: {frame_resp.error_code}")
          return frame_resp.is_ack
  ```
- [ ] Add error code definitions
  ```python
  class ErrorCode(IntEnum):
      NONE = 0x00
      BAD_FRAME_ID = 0x01
      SET_READ_ONLY = 0x02
      GET_WRITE_ONLY = 0x03
      WRONG_SET = 0x05
      WRONG_CMD = 0x07
      OVERRUN = 0x08
      TIMEOUT = 0x09
      BAD_CRC = 0x0A
      BAD_MOTOR = 0x0B
      MP_NOT_ENABLED = 0x0C
  ```
- [ ] Create `RegistryError` exception class

**Deliverables:**
- `st_mcp/commands/registry.py` - Registry command primitives
- `st_mcp/exceptions.py` - Custom exceptions
- Unit tests: `tests/test_registry_commands.py`

**Time Estimate:** 6-8 hours

---

## Phase 3: High-Level Registry API

### Step 3.1: Build MotorRegistry Class
**Goal:** User-friendly parameter access layer

**Tasks:**
- [ ] Create `st_mcp/registry.py`
- [ ] Implement `MotorRegistry` class
  ```python
  class MotorRegistry:
      def __init__(self, transport: ASPEPTransport, motor_id: int = 0):
          self.transport = transport
          self.motor_id = motor_id

      def _read_register(self, register: Register) -> Any:
          """Low-level register read"""
          cmd = ReadRegisterCommand(register, self.motor_id)
          response = self.transport.send_command(cmd)
          return cmd.parse_response(response)

      def _write_register(self, register: Register, value: Any):
          """Low-level register write"""
          cmd = WriteRegisterCommand(register, value, self.motor_id)
          response = self.transport.send_command(cmd)
          return cmd.parse_response(response)
  ```
- [ ] Add speed control methods
  ```python
  def read_speed_ref(self) -> int:
      """Read speed reference setpoint (RPM)"""
      return self._read_register(RegisterMap.SPEED_REF)

  def write_speed_ref(self, speed_rpm: int):
      """Set speed reference setpoint (RPM)"""
      self._write_register(RegisterMap.SPEED_REF, speed_rpm)

  def read_speed_meas(self) -> int:
      """Read measured speed (RPM)"""
      return self._read_register(RegisterMap.SPEED_MEAS)

  def read_speed_pid(self) -> Tuple[int, int, int]:
      """Read speed PID gains (Kp, Ki, Kd)"""
      kp = self._read_register(RegisterMap.SPEED_KP)
      ki = self._read_register(RegisterMap.SPEED_KI)
      kd = self._read_register(RegisterMap.SPEED_KD)
      return (kp, ki, kd)

  def write_speed_pid(self, kp: int, ki: int, kd: int):
      """Set speed PID gains"""
      self._write_register(RegisterMap.SPEED_KP, kp)
      self._write_register(RegisterMap.SPEED_KI, ki)
      self._write_register(RegisterMap.SPEED_KD, kd)
  ```
- [ ] Add torque control methods
  ```python
  def read_torque_ref(self) -> float:
      """Read torque reference (Amps)"""
      raw = self._read_register(RegisterMap.TORQUE_REF)
      return raw / 10000.0  # Scale to Amps

  def write_torque_ref(self, iq_amps: float):
      """Set torque reference (Amps)"""
      raw = int(iq_amps * 10000)
      self._write_register(RegisterMap.TORQUE_REF, raw)

  def read_torque_meas(self) -> float:
      """Read measured torque (Amps)"""
      raw = self._read_register(RegisterMap.TORQUE_MEAS)
      return raw / 10000.0
  ```
- [ ] Add flux control methods
  ```python
  def read_flux_ref(self) -> float:
      return self._read_register(RegisterMap.FLUX_REF) / 10000.0

  def write_flux_ref(self, id_amps: float):
      self._write_register(RegisterMap.FLUX_REF, int(id_amps * 10000))
  ```
- [ ] Add measurement methods
  ```python
  def read_bus_voltage(self) -> float:
      """Read DC bus voltage (Volts)"""
      raw = self._read_register(RegisterMap.BUS_VOLTAGE)
      return raw / 100.0

  def read_temperature(self) -> float:
      """Read heatsink temperature (Celsius)"""
      raw = self._read_register(RegisterMap.HEATS_TEMP)
      return raw / 10.0

  def read_motor_power(self) -> float:
      """Read motor power (Watts)"""
      return self._read_register(RegisterMap.MOTOR_POWER)
  ```
- [ ] Add control methods
  ```python
  def read_control_mode(self) -> MotorControlMode:
      """Read current control mode"""
      raw = self._read_register(RegisterMap.CONTROL_MODE)
      return MotorControlMode(raw)

  def write_control_mode(self, mode: MotorControlMode):
      """Set control mode (Torque or Speed)"""
      self._write_register(RegisterMap.CONTROL_MODE, mode.value)

  def read_status(self) -> int:
      """Read motor status flags"""
      return self._read_register(RegisterMap.STATUS)

  def read_fault_flags(self) -> int:
      """Read motor fault flags"""
      return self._read_register(RegisterMap.FAULT_FLAGS)
  ```

**Deliverables:**
- `st_mcp/registry.py` - High-level registry API
- Unit tests: `tests/test_registry.py`

**Time Estimate:** 6-8 hours

---

### Step 3.2: Create Telemetry Data Classes
**Goal:** Structured data for real-time monitoring

**Tasks:**
- [ ] Create `st_mcp/telemetry.py`
- [ ] Define `MotorTelemetry` dataclass
  ```python
  @dataclass
  class MotorTelemetry:
      timestamp: float
      speed_rpm: int
      torque_amps: float
      flux_amps: float
      bus_voltage_v: float
      temperature_c: float
      motor_power_w: float
      status: int
      fault_flags: int

      @property
      def has_faults(self) -> bool:
          return self.fault_flags != 0

      def __str__(self) -> str:
          return (
              f"Speed: {self.speed_rpm} RPM, "
              f"Torque: {self.torque_amps:.2f} A, "
              f"Voltage: {self.bus_voltage_v:.1f} V, "
              f"Temp: {self.temperature_c:.1f}°C, "
              f"Power: {self.motor_power_w:.1f} W"
          )
  ```
- [ ] Add `CurrentTelemetry` dataclass for phase currents
  ```python
  @dataclass
  class CurrentTelemetry:
      ia_amps: float
      ib_amps: float
      ialpha_amps: float
      ibeta_amps: float
      iq_amps: float
      id_amps: float
  ```
- [ ] Add `VoltageTelemetry` dataclass
  ```python
  @dataclass
  class VoltageTelemetry:
      vq_v: float
      vd_v: float
      valpha_v: float
      vbeta_v: float
  ```
- [ ] Add telemetry acquisition method to `MotorRegistry`
  ```python
  def get_telemetry(self) -> MotorTelemetry:
      """Read all telemetry data in one call"""
      return MotorTelemetry(
          timestamp=time.time(),
          speed_rpm=self.read_speed_meas(),
          torque_amps=self.read_torque_meas(),
          flux_amps=self.read_flux_meas(),
          bus_voltage_v=self.read_bus_voltage(),
          temperature_c=self.read_temperature(),
          motor_power_w=self.read_motor_power(),
          status=self.read_status(),
          fault_flags=self.read_fault_flags(),
      )
  ```

**Deliverables:**
- `st_mcp/telemetry.py` - Telemetry data structures
- Enhanced `st_mcp/registry.py` with telemetry methods

**Time Estimate:** 3-4 hours

---

## Phase 4: Transport Layer Integration

### Step 4.1: Refactor ASPEP Transport
**Goal:** Clean separation between transport and protocol layers

**Tasks:**
- [ ] Create `st_mcp/transport.py`
- [ ] Implement `ASPEPTransport` class
  ```python
  class ASPEPTransport:
      def __init__(self, port: str, baudrate: int = 1843200):
          self.port = port
          self.baudrate = baudrate
          self.serial = None
          self.connected = False

      def connect(self):
          """Open serial port and perform beacon handshake"""
          self.serial = serial.Serial(
              port=self.port,
              baudrate=self.baudrate,
              bytesize=serial.EIGHTBITS,
              parity=serial.PARITY_NONE,
              stopbits=serial.STOPBITS_ONE,
              timeout=1
          )
          self._beacon_handshake()
          self.connected = True

      def _beacon_handshake(self):
          """Perform ASPEP beacon exchange"""
          # Send: 55 FF FF 77
          # Receive response
          # Echo response
          # Receive confirmation
          # Send connection request: 06 00 00 60

      def disconnect(self):
          """Send disconnect and close serial port"""
          if self.serial:
              self._send_disconnect()
              self.serial.close()
              self.connected = False

      def send_command(self, cmd: MotorCommand) -> bytes:
          """Send command and receive response"""
          if not self.connected:
              raise ConnectionError("Not connected")

          # Serialize command
          cmd_bytes = cmd.to_bytes()

          # Send
          self.serial.write(cmd_bytes)

          # Receive response
          response = self._read_response()

          return response

      def _read_response(self, timeout: float = 1.0) -> bytes:
          """Read response with timeout"""
          # Implement buffered read with timeout
          pass
  ```
- [ ] Extract beacon handshake from `minimal_motor_control.py`
- [ ] Add connection state management
- [ ] Implement timeout and retry logic
- [ ] Add connection health monitoring

**Deliverables:**
- `st_mcp/transport.py` - ASPEP transport layer
- Unit tests: `tests/test_transport.py` (mock serial)

**Time Estimate:** 4-6 hours

---

### Step 4.2: Refactor Motor Commands
**Goal:** Use frame construction instead of hardcoded bytes

**Tasks:**
- [ ] Update `st_mcp/commands/motor.py`
- [ ] Refactor `StartMotorCommand`
  ```python
  class StartMotorCommand(MotorCommand):
      def __init__(self, motor_id: int = 0):
          super().__init__(command_id=MOTOR_CMD_START_MOTOR, motor_id=motor_id)

      def to_bytes(self) -> bytes:
          frame = Frame(frame_code=0x03, motor_id=self.motor_id)  # EXECUTE_CMD
          frame.add_byte(0x01)  # MOTOR_CMD_START_MOTOR
          return frame.to_bytes()

      def parse_response(self, response: bytes) -> bool:
          frame_resp = FrameResponse(response)
          return frame_resp.is_ack
  ```
- [ ] Refactor all motor commands
  - [ ] `StopMotorCommand`
  - [ ] `StopRampCommand`
  - [ ] `StartStopCommand` (toggle)
  - [ ] `FaultAckCommand`
  - [ ] `EncoderAlignCommand`
  - [ ] `IQDRefClearCommand`
- [ ] Remove hardcoded command IDs, use constants
  ```python
  class MotorCommandType(IntEnum):
      START = 0x01
      STOP = 0x02
      STOP_RAMP = 0x03
      START_STOP = 0x06
      FAULT_ACK = 0x07
      ENCODER_ALIGN = 0x08
      IQDREF_CLEAR = 0x09
  ```
- [ ] Update `SetControlModeCommand` to use Frame
  ```python
  class SetControlModeCommand(MotorCommand):
      def __init__(self, mode: MotorControlMode, motor_id: int = 0):
          super().__init__(command_id=0x0001, motor_id=motor_id)
          self.mode = mode

      def to_bytes(self) -> bytes:
          frame = Frame(frame_code=0x01, motor_id=self.motor_id)  # SET_REG
          frame.add_byte(RegisterMap.CONTROL_MODE.id)
          frame.add_byte(self.mode.value)
          return frame.to_bytes()
  ```

**Deliverables:**
- Updated `st_mcp/commands/motor.py` - Proper frame construction
- Unit tests: `tests/test_motor_commands.py`

**Time Estimate:** 4-5 hours

---

## Phase 5: High-Level Motor Controller API

### Step 5.1: Create MotorController Class
**Goal:** Simple, user-friendly interface for common operations

**Tasks:**
- [ ] Create `st_mcp/motor_controller.py`
- [ ] Implement `MotorController` class
  ```python
  class MotorController:
      def __init__(self, port: str, baudrate: int = 1843200, motor_id: int = 0):
          self.transport = ASPEPTransport(port, baudrate)
          self.registry = MotorRegistry(self.transport, motor_id)
          self.motor_id = motor_id
          self._connected = False

      def connect(self):
          """Establish connection to motor controller"""
          self.transport.connect()
          self._connected = True

      def disconnect(self):
          """Close connection"""
          self.stop()  # Safety: stop motor before disconnect
          self.transport.disconnect()
          self._connected = False

      def __enter__(self):
          self.connect()
          return self

      def __exit__(self, exc_type, exc_val, exc_tb):
          self.disconnect()
  ```
- [ ] Add motor control methods
  ```python
  def start(self):
      """Start motor"""
      cmd = StartMotorCommand(self.motor_id)
      self.transport.send_command(cmd)

  def stop(self, ramp: bool = True):
      """Stop motor with optional ramp-down"""
      if ramp:
          cmd = StopRampCommand(self.motor_id)
      else:
          cmd = StopMotorCommand(self.motor_id)
      self.transport.send_command(cmd)

  def fault_ack(self):
      """Acknowledge faults"""
      cmd = FaultAckCommand(self.motor_id)
      self.transport.send_command(cmd)

  def encoder_align(self):
      """Perform encoder alignment"""
      cmd = EncoderAlignCommand(self.motor_id)
      self.transport.send_command(cmd)
  ```
- [ ] Add speed control methods
  ```python
  def set_speed_mode(self, target_rpm: int, ramp_duration_ms: int = 1000):
      """Configure motor for speed control and start"""
      # Set control mode
      self.registry.write_control_mode(MotorControlMode.SPEED)

      # Set target speed
      self.registry.write_speed_ref(target_rpm)

      # Set ramp (if supported)
      if ramp_duration_ms > 0:
          self.registry.write_ramp_final_speed(target_rpm)

      # Start motor
      self.start()

  def get_speed(self) -> int:
      """Read current motor speed (RPM)"""
      return self.registry.read_speed_meas()

  def set_speed_pid(self, kp: int, ki: int, kd: int = 0):
      """Configure speed PID gains"""
      self.registry.write_speed_pid(kp, ki, kd)
  ```
- [ ] Add torque control methods
  ```python
  def set_torque_mode(self, iq_ref: float, id_ref: float = 0.0):
      """Configure motor for torque control and start"""
      # Set control mode
      self.registry.write_control_mode(MotorControlMode.TORQUE)

      # Set torque reference (Iq)
      self.registry.write_torque_ref(iq_ref)

      # Set flux reference (Id)
      self.registry.write_flux_ref(id_ref)

      # Start motor
      self.start()

  def get_torque(self) -> float:
      """Read current motor torque (Amps)"""
      return self.registry.read_torque_meas()
  ```
- [ ] Add telemetry method
  ```python
  def get_telemetry(self) -> MotorTelemetry:
      """Read all real-time measurements"""
      return self.registry.get_telemetry()
  ```
- [ ] Add status monitoring
  ```python
  def is_running(self) -> bool:
      """Check if motor is running"""
      status = self.registry.read_status()
      return bool(status & 0x01)  # Motor running bit

  def has_faults(self) -> bool:
      """Check if motor has faults"""
      faults = self.registry.read_fault_flags()
      return faults != 0

  def get_faults(self) -> List[str]:
      """Get list of active fault descriptions"""
      faults = self.registry.read_fault_flags()
      # Decode fault bits to human-readable strings
      pass
  ```

**Deliverables:**
- `st_mcp/motor_controller.py` - High-level controller API
- Integration tests: `tests/test_motor_controller.py`

**Time Estimate:** 6-8 hours

---

### Step 5.2: Replace minimal_motor_control.py
**Goal:** Rewrite working example using new architecture

**Tasks:**
- [ ] Create `examples/speed_control_example.py`
  ```python
  #!/usr/bin/env python3
  """
  Simple speed control example using MotorController API.
  Replaces minimal_motor_control.py with proper registry service.
  """
  import time
  from st_mcp.motor_controller import MotorController

  def main():
      # Connect to motor
      with MotorController(port="/dev/ttyACM0") as motor:
          print("Connected to motor controller")

          # Check for faults
          if motor.has_faults():
              print("Faults detected, acknowledging...")
              motor.fault_ack()

          # Set speed mode: 1500 RPM with 1 second ramp
          print("Starting motor in speed mode: 1500 RPM")
          motor.set_speed_mode(target_rpm=1500, ramp_duration_ms=1000)

          # Monitor for 10 seconds
          for i in range(10):
              telemetry = motor.get_telemetry()
              print(f"[{i}s] {telemetry}")
              time.sleep(1)

          # Stop with ramp
          print("Stopping motor...")
          motor.stop(ramp=True)

          print("Done!")

  if __name__ == "__main__":
      main()
  ```
- [ ] Create `examples/torque_control_example.py`
  ```python
  #!/usr/bin/env python3
  """Torque control example"""
  from st_mcp.motor_controller import MotorController

  def main():
      with MotorController(port="/dev/ttyACM0") as motor:
          # Set torque mode: 2.5 Amps Iq, 0 Amps Id
          motor.set_torque_mode(iq_ref=2.5, id_ref=0.0)

          # Monitor torque
          for i in range(10):
              torque = motor.get_torque()
              print(f"Torque: {torque:.2f} A")
              time.sleep(0.5)

          motor.stop()

  if __name__ == "__main__":
      main()
  ```
- [ ] Create `examples/telemetry_monitor.py`
  ```python
  #!/usr/bin/env python3
  """Real-time telemetry monitoring"""
  from st_mcp.motor_controller import MotorController
  import time

  def main():
      with MotorController(port="/dev/ttyACM0") as motor:
          print("Monitoring motor telemetry (Ctrl+C to stop)")

          try:
              while True:
                  telem = motor.get_telemetry()
                  print(f"\r{telem}", end="", flush=True)
                  time.sleep(0.1)
          except KeyboardInterrupt:
              print("\nStopped monitoring")

  if __name__ == "__main__":
      main()
  ```
- [ ] Create `examples/pid_tuning.py`
  ```python
  #!/usr/bin/env python3
  """PID tuning example"""
  from st_mcp.motor_controller import MotorController

  def main():
      with MotorController(port="/dev/ttyACM0") as motor:
          # Read current PID gains
          kp, ki, kd = motor.registry.read_speed_pid()
          print(f"Current PID gains: Kp={kp}, Ki={ki}, Kd={kd}")

          # Set new PID gains
          motor.set_speed_pid(kp=1000, ki=50, kd=0)
          print("Updated PID gains")

          # Test with speed control
          motor.set_speed_mode(target_rpm=1000)
          # ... monitoring code ...

  if __name__ == "__main__":
      main()
  ```
- [ ] Update `st_mcp/minimal_motor_control.py` to use new API
  - [ ] Keep for backward compatibility
  - [ ] Add deprecation warning
  - [ ] Redirect to new examples

**Deliverables:**
- `examples/speed_control_example.py`
- `examples/torque_control_example.py`
- `examples/telemetry_monitor.py`
- `examples/pid_tuning.py`
- Updated `st_mcp/minimal_motor_control.py` with deprecation notice

**Time Estimate:** 4-6 hours

---

## Phase 6: Testing & Validation

### Step 6.1: Unit Tests
**Goal:** Test individual components in isolation

**Tasks:**
- [ ] Create test structure
  ```
  tests/
  ├── __init__.py
  ├── test_registers.py
  ├── test_frame.py
  ├── test_registry_commands.py
  ├── test_motor_commands.py
  ├── test_transport.py (mocked)
  ├── test_registry.py (mocked)
  └── test_motor_controller.py (mocked)
  ```
- [ ] Write tests for `registers.py`
  - [ ] Register encoding/decoding
  - [ ] Register lookup by ID and name
  - [ ] Value validation (min/max)
  - [ ] Unit scaling
- [ ] Write tests for `frame.py`
  - [ ] Frame construction
  - [ ] Checksum calculation
  - [ ] ASPEP wrapping
  - [ ] Response parsing
  - [ ] ACK/NACK detection
- [ ] Write tests for registry commands
  - [ ] ReadRegisterCommand serialization
  - [ ] WriteRegisterCommand serialization
  - [ ] Response parsing
  - [ ] Error handling
- [ ] Write tests for motor commands
  - [ ] All motor command serialization
  - [ ] Frame code correctness
  - [ ] Payload structure
- [ ] Write tests with mocked serial
  - [ ] Transport connection/disconnection
  - [ ] Command send/receive
  - [ ] Timeout handling
  - [ ] Retry logic

**Deliverables:**
- Complete unit test suite with >80% coverage
- `tests/conftest.py` - Pytest fixtures
- `tests/mocks.py` - Mock serial port

**Time Estimate:** 8-10 hours

---

### Step 6.2: Hardware Integration Tests
**Goal:** Verify with actual STM32 hardware

**Tasks:**
- [ ] Create hardware test script
  ```python
  # tests/hardware/test_registry_hardware.py
  import pytest
  from st_mcp.motor_controller import MotorController

  @pytest.fixture
  def motor():
      """Fixture requiring actual hardware"""
      motor = MotorController(port="/dev/ttyACM0")
      motor.connect()
      yield motor
      motor.disconnect()

  def test_read_firmware_version(motor):
      """Test reading firmware version"""
      # Implementation
      pass

  def test_read_bus_voltage(motor):
      """Test reading bus voltage"""
      voltage = motor.registry.read_bus_voltage()
      assert 0 < voltage < 50  # Reasonable range

  def test_write_read_speed_ref(motor):
      """Test writing and reading speed reference"""
      motor.registry.write_speed_ref(1000)
      speed = motor.registry.read_speed_ref()
      assert speed == 1000
  ```
- [ ] Test all registry operations
  - [ ] Read control mode
  - [ ] Write control mode
  - [ ] Read/write speed parameters
  - [ ] Read/write torque parameters
  - [ ] Read measurements (voltage, temp, current)
- [ ] Test motor commands
  - [ ] Start motor
  - [ ] Stop motor
  - [ ] Fault acknowledge
  - [ ] Encoder align (if supported)
- [ ] Test full speed control sequence
  - [ ] Connect
  - [ ] Set speed mode
  - [ ] Start motor
  - [ ] Monitor telemetry
  - [ ] Stop motor
  - [ ] Disconnect
- [ ] Test error conditions
  - [ ] Invalid register writes
  - [ ] Communication timeout
  - [ ] Fault detection and recovery
- [ ] Document hardware test setup
  - [ ] Required hardware
  - [ ] Wiring diagram
  - [ ] Safety precautions
  - [ ] Expected behavior

**Deliverables:**
- `tests/hardware/` directory with hardware tests
- `tests/hardware/README.md` - Hardware test setup guide
- Hardware test results log

**Time Estimate:** 6-8 hours (requires hardware)

---

### Step 6.3: Performance & Reliability Testing
**Goal:** Verify real-time performance and stability

**Tasks:**
- [ ] Create performance test script
  ```python
  # tests/performance/test_latency.py
  def test_register_read_latency(motor):
      """Measure register read latency"""
      latencies = []
      for _ in range(100):
          start = time.perf_counter()
          motor.registry.read_speed_meas()
          latencies.append(time.perf_counter() - start)

      print(f"Mean latency: {np.mean(latencies)*1000:.2f} ms")
      print(f"Max latency: {np.max(latencies)*1000:.2f} ms")
      assert np.mean(latencies) < 0.010  # < 10ms average
  ```
- [ ] Test command throughput
  - [ ] Commands per second
  - [ ] Telemetry update rate
  - [ ] Sustained monitoring performance
- [ ] Stress test
  - [ ] Continuous operation for 1 hour
  - [ ] Rapid start/stop cycles
  - [ ] Parameter changes during operation
  - [ ] Monitor for memory leaks
  - [ ] Check for dropped commands
- [ ] Test connection reliability
  - [ ] Reconnection after disconnect
  - [ ] Recovery from communication errors
  - [ ] Handling of device reset

**Deliverables:**
- `tests/performance/` directory
- Performance benchmark results
- Reliability test report

**Time Estimate:** 4-6 hours

---

## Phase 7: Documentation

### Step 7.1: API Documentation
**Goal:** Complete API reference for all public interfaces

**Tasks:**
- [ ] Add comprehensive docstrings
  - [ ] All public classes
  - [ ] All public methods
  - [ ] All parameters and return values
  - [ ] Usage examples in docstrings
- [ ] Generate API docs with Sphinx
  - [ ] Install sphinx: `pip install sphinx sphinx-rtd-theme`
  - [ ] Initialize: `sphinx-quickstart docs`
  - [ ] Configure autodoc
  - [ ] Build HTML docs: `make html`
- [ ] Document all modules
  - [ ] `registers.py` - Register definitions
  - [ ] `frame.py` - Frame construction
  - [ ] `commands/` - All command types
  - [ ] `registry.py` - Registry API
  - [ ] `transport.py` - ASPEP transport
  - [ ] `motor_controller.py` - High-level API
  - [ ] `telemetry.py` - Data structures
- [ ] Create API reference guide
  - [ ] Quick start
  - [ ] Common patterns
  - [ ] Error handling
  - [ ] Best practices

**Deliverables:**
- Complete docstrings in all modules
- `docs/` directory with Sphinx configuration
- Generated HTML API documentation

**Time Estimate:** 4-6 hours

---

### Step 7.2: User Guide & Examples
**Goal:** Comprehensive usage documentation

**Tasks:**
- [ ] Update `README.md`
  - [ ] New architecture overview
  - [ ] Installation instructions
  - [ ] Quick start guide
  - [ ] Link to examples
  - [ ] Link to API docs
- [ ] Create `docs/USER_GUIDE.md`
  - [ ] Hardware setup
  - [ ] Connection establishment
  - [ ] Speed control tutorial
  - [ ] Torque control tutorial
  - [ ] PID tuning guide
  - [ ] Telemetry monitoring
  - [ ] Error handling
  - [ ] Troubleshooting
- [ ] Create `docs/PROTOCOL_GUIDE.md`
  - [ ] Frame Communication Protocol overview
  - [ ] ASPEP transport details
  - [ ] Register map reference
  - [ ] Command reference
  - [ ] Checksum algorithm
- [ ] Document all examples
  - [ ] Add detailed comments
  - [ ] Explain each step
  - [ ] Add safety notes
  - [ ] Include expected output
- [ ] Create migration guide
  - [ ] `docs/MIGRATION.md` - From hardcoded commands to registry API
  - [ ] Before/after comparisons
  - [ ] Breaking changes
  - [ ] Backward compatibility notes

**Deliverables:**
- Updated `README.md`
- `docs/USER_GUIDE.md`
- `docs/PROTOCOL_GUIDE.md`
- `docs/MIGRATION.md`
- Well-documented example scripts

**Time Estimate:** 6-8 hours

---

### Step 7.3: Project Cleanup
**Goal:** Polish and prepare for merge

**Tasks:**
- [ ] Update project metadata
  - [ ] `setup.py` or `pyproject.toml`
  - [ ] Version number
  - [ ] Dependencies
  - [ ] Entry points
- [ ] Add type hints everywhere
  - [ ] Run mypy type checking
  - [ ] Fix all type errors
- [ ] Code formatting
  - [ ] Run black formatter
  - [ ] Run isort for imports
  - [ ] Check with flake8
- [ ] Update changelog
  - [ ] `CHANGELOG.md` with all changes
  - [ ] Breaking changes highlighted
  - [ ] Migration notes
- [ ] Clean up old code
  - [ ] Mark deprecated functions
  - [ ] Remove unused imports
  - [ ] Remove debug print statements
  - [ ] Remove commented-out code
- [ ] Review all TODOs and FIXMEs
  - [ ] Address or document remaining items
- [ ] Final test run
  - [ ] All unit tests pass
  - [ ] All examples work
  - [ ] Hardware tests pass (if available)

**Deliverables:**
- Clean, formatted codebase
- Updated project metadata
- `CHANGELOG.md`
- All tests passing

**Time Estimate:** 4-6 hours

---

## Phase 8: Review & Merge

### Step 8.1: Self-Review
**Goal:** Final quality check before PR

**Tasks:**
- [ ] Review all changed files
  - [ ] Check for bugs
  - [ ] Verify error handling
  - [ ] Check edge cases
  - [ ] Validate documentation
- [ ] Test all examples
  - [ ] Run each example script
  - [ ] Verify output
  - [ ] Check for errors
- [ ] Review test coverage
  - [ ] Run coverage report
  - [ ] Identify untested code paths
  - [ ] Add missing tests
- [ ] Performance check
  - [ ] Profile critical paths
  - [ ] Check for performance regressions
  - [ ] Optimize if needed

**Deliverables:**
- Self-review checklist completed
- All identified issues fixed

**Time Estimate:** 3-4 hours

---

### Step 8.2: Create Pull Request
**Goal:** Document changes for review

**Tasks:**
- [ ] Write comprehensive PR description
  - [ ] Summary of changes
  - [ ] Problem statement
  - [ ] Solution approach
  - [ ] Breaking changes
  - [ ] Migration guide
  - [ ] Testing performed
- [ ] Link to issue #1
- [ ] Add before/after comparison
  ```python
  # Before (hardcoded)
  self.send_command("D9 00 00 40 08 00 29 05 07 00 00 08 00 00 FF 00")

  # After (clean API)
  motor.set_speed_mode(target_rpm=1500, ramp_duration_ms=1000)
  ```
- [ ] List all new files
- [ ] Document testing performed
  - [ ] Unit tests
  - [ ] Integration tests
  - [ ] Hardware tests (if done)
- [ ] Add screenshots or logs (if applicable)
- [ ] Request review (if working in team)

**Deliverables:**
- Pull request with comprehensive description
- Link to #1 issue

**Time Estimate:** 1-2 hours

---

### Step 8.3: Merge & Release
**Goal:** Integrate changes into main branch

**Tasks:**
- [ ] Address review comments (if any)
- [ ] Squash or organize commits
- [ ] Update version number
  - [ ] Semantic versioning: `0.1.0` → `0.2.0`
  - [ ] Major refactor warrants minor version bump
- [ ] Merge to main
- [ ] Create release tag
  - [ ] `git tag -a v0.2.0 -m "Registry Service Implementation"`
  - [ ] `git push origin v0.2.0`
- [ ] Update main branch documentation
- [ ] Close issue #1
- [ ] Announce release (if applicable)
  - [ ] Update ST forum post
  - [ ] Share with community

**Deliverables:**
- Merged PR
- Release tag `v0.2.0`
- Closed issue #1
- Updated ST forum post

**Time Estimate:** 1-2 hours

---

## Summary Checklist

### Core Implementation
- [ ] Phase 1: Foundation & Code Analysis (7-10 hours)
- [ ] Phase 2: Core Infrastructure (14-20 hours)
- [ ] Phase 3: High-Level Registry API (9-12 hours)
- [ ] Phase 4: Transport Layer Integration (8-11 hours)
- [ ] Phase 5: High-Level Motor Controller API (10-14 hours)
- [ ] Phase 6: Testing & Validation (18-24 hours)
- [ ] Phase 7: Documentation (14-20 hours)
- [ ] Phase 8: Review & Merge (5-8 hours)

### Total Estimated Time: 85-119 hours (10-15 days)

### Success Metrics
- [ ] Zero hardcoded hex strings in production code
- [ ] Complete Registry Service implementation
- [ ] All 60+ registers accessible via API
- [ ] Clean high-level API: `motor.set_speed(1500)`
- [ ] >80% test coverage
- [ ] All examples work with hardware
- [ ] Comprehensive documentation
- [ ] ST forum post updated with success story

---

## Risk Mitigation

### Technical Risks
1. **ASPEP compatibility** - Frame Protocol commands may not work over ASPEP
   - Mitigation: Test early in Phase 1.3, implement adapter if needed
2. **Hardware availability** - May not have STM32 hardware for testing
   - Mitigation: Extensive unit tests with mocks, hardware tests optional
3. **Protocol differences** - MCSDK 6.x may have different register IDs
   - Mitigation: Document version differences, add compatibility layer

### Schedule Risks
1. **Scope creep** - Additional features discovered during implementation
   - Mitigation: Stick to core registry service, defer enhancements
2. **Hardware issues** - Device failures during testing
   - Mitigation: Have backup hardware, mock tests as fallback
3. **Unknown unknowns** - Unexpected protocol quirks
   - Mitigation: Build in 20% buffer time, ask for help early

---

## Next Steps

1. **Review this plan** - Confirm approach with stakeholders
2. **Set up development environment** - Install dependencies
3. **Start Phase 1.1** - Begin code audit
4. **Commit frequently** - Small, incremental commits
5. **Test continuously** - Don't wait until the end
6. **Ask for help** - Post on ST forum if blocked

---

## Questions to Answer Early

1. Does ASPEP directly carry Frame Protocol commands?
2. Are register IDs identical between MCSDK 5.x and 6.x?
3. Do we need checksum for ASPEP or does it have its own CRC?
4. What's the maximum command rate we need to support?
5. Should we support both protocols or focus on ASPEP only?

---

**Ready to begin implementation! 🚀**
