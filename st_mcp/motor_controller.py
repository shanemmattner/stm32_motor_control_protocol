"""
Motor Controller High-Level API

User-friendly interface for motor control operations.
Provides simple methods like set_speed_mode() and get_telemetry()
without requiring knowledge of registers or protocols.
"""

import serial
import time
from typing import Optional
from dataclasses import dataclass
from .registry import MotorRegistry
from .registers import MotorControlMode
from .frame import Frame, FrameCode, create_execute_command_frame, ASPEPWrapper
from .exceptions import MotorControlError, MotorFaultError


class MotorCommandType:
    """Motor command identifiers"""
    START_MOTOR = 0x01
    STOP_MOTOR = 0x02
    STOP_RAMP = 0x03
    START_STOP = 0x06
    FAULT_ACK = 0x07
    ENCODER_ALIGN = 0x08
    IQDREF_CLEAR = 0x09


@dataclass
class MotorTelemetry:
    """Motor telemetry data"""
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
        """Check if motor has active faults"""
        return self.fault_flags != 0

    def __str__(self) -> str:
        return (
            f"Speed: {self.speed_rpm:4d} RPM | "
            f"Torque: {self.torque_amps:5.2f} A | "
            f"Voltage: {self.bus_voltage_v:5.1f} V | "
            f"Temp: {self.temperature_c:4.1f}°C | "
            f"Power: {self.motor_power_w:6.1f} W"
        )


class SimpleTransport:
    """
    Simple serial transport for motor commands.

    Temporary solution using basic serial communication.
    TODO: Replace with proper ASPEP transport layer.
    """

    def __init__(self, port: str, baudrate: int = 1843200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None

    def connect(self):
        """Open serial port"""
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        # Clear buffers
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def disconnect(self):
        """Close serial port"""
        if self.serial:
            self.serial.close()
            self.serial = None

    def send_command(self, cmd_bytes: bytes, response_length: int = 16) -> bytes:
        """
        Send command and receive response.

        Args:
            cmd_bytes: Command bytes to send
            response_length: Expected response length

        Returns:
            Response bytes
        """
        if not self.serial or not self.serial.is_open:
            raise MotorControlError("Transport not connected")

        # Wrap in ASPEP if needed (Frame protocol commands need wrapping)
        if len(cmd_bytes) < 4 or cmd_bytes[0] != 0x49:
            # This is a raw frame, wrap it
            aspep_packet = ASPEPWrapper.SYNC_HEADER + cmd_bytes
        else:
            # Already has ASPEP header
            aspep_packet = cmd_bytes

        # Send
        self.serial.write(aspep_packet)

        # Wait for response
        time.sleep(0.05)  # Small delay for controller processing

        # Read response
        response = self.serial.read(response_length)

        return response


class MotorController:
    """
    High-level motor controller interface.

    Provides simple, user-friendly methods for motor control:
    - set_speed_mode(rpm) - Configure speed control
    - set_torque_mode(amps) - Configure torque control
    - get_telemetry() - Read all measurements
    - start() / stop() - Motor control
    """

    def __init__(self, port: str, baudrate: int = 1843200, motor_id: int = 0):
        """
        Initialize motor controller.

        Args:
            port: Serial port (e.g., "/dev/ttyACM0" or "COM3")
            baudrate: Baud rate (default 1843200 for ASPEP)
            motor_id: Target motor ID (0-7)
        """
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id

        # Transport layer
        self._transport = SimpleTransport(port, baudrate)

        # Registry service
        self.registry = MotorRegistry(self._transport, motor_id)

        # Connection state
        self._connected = False

    def connect(self):
        """Establish connection to motor controller"""
        if self._connected:
            return

        self._transport.connect()
        self._connected = True

    def disconnect(self):
        """Close connection to motor controller"""
        if not self._connected:
            return

        # Safety: try to stop motor before disconnect
        try:
            self.stop()
        except Exception:
            pass  # Ignore errors during disconnect

        self._transport.disconnect()
        self._connected = False

    def _ensure_connected(self):
        """Verify connection is established"""
        if not self._connected:
            raise MotorControlError("Not connected. Call connect() first.")

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

    # Motor Control Commands

    def start(self):
        """Start motor"""
        self._ensure_connected()
        frame = create_execute_command_frame(MotorCommandType.START_MOTOR, self.motor_id)
        self._transport.send_command(frame.to_bytes())

    def stop(self, ramp: bool = True):
        """
        Stop motor.

        Args:
            ramp: If True, use ramp-down. If False, immediate stop.
        """
        self._ensure_connected()
        cmd = MotorCommandType.STOP_RAMP if ramp else MotorCommandType.STOP_MOTOR
        frame = create_execute_command_frame(cmd, self.motor_id)
        self._transport.send_command(frame.to_bytes())

    def fault_ack(self):
        """Acknowledge and clear motor faults"""
        self._ensure_connected()
        frame = create_execute_command_frame(MotorCommandType.FAULT_ACK, self.motor_id)
        self._transport.send_command(frame.to_bytes())

    def encoder_align(self):
        """Perform encoder alignment"""
        self._ensure_connected()
        frame = create_execute_command_frame(MotorCommandType.ENCODER_ALIGN, self.motor_id)
        self._transport.send_command(frame.to_bytes())

    # Speed Control

    def set_speed_mode(self, target_rpm: int, ramp_duration_ms: int = 1000):
        """
        Configure motor for speed control and start.

        Args:
            target_rpm: Target speed in RPM
            ramp_duration_ms: Ramp duration (not yet implemented)
        """
        self._ensure_connected()

        # Check for faults
        faults = self.registry.read_fault_flags()
        if faults != 0:
            self.fault_ack()
            time.sleep(0.1)

        # Set control mode to speed
        self.registry.write_control_mode(MotorControlMode.SPEED)

        # Set speed reference
        self.registry.write_speed_ref(target_rpm)

        # Set ramp target (if register exists)
        try:
            self.registry.write_ramp_final_speed(target_rpm)
        except Exception:
            pass  # Ramp may not be supported

        # Start motor
        self.start()

    def get_speed(self) -> int:
        """
        Read current motor speed.

        Returns:
            Motor speed in RPM
        """
        self._ensure_connected()
        return self.registry.read_speed_meas()

    def set_speed_pid(self, kp: int, ki: int, kd: int = 0):
        """
        Configure speed PID gains.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain (optional)
        """
        self._ensure_connected()
        self.registry.write_speed_pid(kp, ki, kd)

    # Torque Control

    def set_torque_mode(self, iq_ref: float, id_ref: float = 0.0):
        """
        Configure motor for torque control and start.

        Args:
            iq_ref: Torque reference in Amps (Iq current)
            id_ref: Flux reference in Amps (Id current, usually 0)
        """
        self._ensure_connected()

        # Check for faults
        faults = self.registry.read_fault_flags()
        if faults != 0:
            self.fault_ack()
            time.sleep(0.1)

        # Set control mode to torque
        self.registry.write_control_mode(MotorControlMode.TORQUE)

        # Set torque reference (Iq)
        self.registry.write_torque_ref(iq_ref)

        # Set flux reference (Id)
        self.registry.write_flux_ref(id_ref)

        # Start motor
        self.start()

    def get_torque(self) -> float:
        """
        Read current motor torque.

        Returns:
            Motor torque in Amps (Iq current)
        """
        self._ensure_connected()
        return self.registry.read_torque_meas()

    # Telemetry

    def get_telemetry(self) -> MotorTelemetry:
        """
        Read all real-time measurements.

        Returns:
            MotorTelemetry object with all sensor data
        """
        self._ensure_connected()

        return MotorTelemetry(
            timestamp=time.time(),
            speed_rpm=self.registry.read_speed_meas(),
            torque_amps=self.registry.read_torque_meas(),
            flux_amps=self.registry.read_flux_meas(),
            bus_voltage_v=self.registry.read_bus_voltage(),
            temperature_c=self.registry.read_temperature(),
            motor_power_w=self.registry.read_motor_power(),
            status=self.registry.read_status(),
            fault_flags=self.registry.read_fault_flags(),
        )

    # Status Monitoring

    def is_running(self) -> bool:
        """
        Check if motor is running.

        Returns:
            True if motor is running
        """
        self._ensure_connected()
        status = self.registry.read_status()
        return bool(status & 0x01)  # Bit 0 = motor running

    def has_faults(self) -> bool:
        """
        Check if motor has active faults.

        Returns:
            True if faults are present
        """
        self._ensure_connected()
        faults = self.registry.read_fault_flags()
        return faults != 0

    def check_faults(self):
        """
        Check for faults and raise exception if present.

        Raises:
            MotorFaultError: If motor has faults
        """
        if self.has_faults():
            fault_flags = self.registry.read_fault_flags()
            raise MotorFaultError(f"Motor has faults: 0x{fault_flags:08X}", fault_flags)
