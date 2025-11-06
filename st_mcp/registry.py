"""
Motor Registry Service

High-level API for reading and writing motor control parameters.
Provides user-friendly methods with engineering units instead of raw register access.
"""

from typing import Tuple, Union, Any
from .registers import Register, RegisterMap, MotorControlMode
from .commands.registry import ReadRegisterCommand, WriteRegisterCommand
from .exceptions import RegistryError


class MotorRegistry:
    """
    High-level motor parameter registry.

    Provides convenient methods for reading and writing motor control
    parameters using engineering units (Volts, Amps, RPM, etc.).
    """

    def __init__(self, transport: Any, motor_id: int = 0):
        """
        Initialize motor registry.

        Args:
            transport: Transport layer with send_command(cmd) -> bytes method
            motor_id: Target motor ID (0-7)
        """
        self.transport = transport
        self.motor_id = motor_id

    def _read_register(self, register: Register) -> Union[int, float]:
        """
        Low-level register read.

        Args:
            register: Register to read

        Returns:
            Register value in engineering units
        """
        cmd = ReadRegisterCommand(register, self.motor_id)
        cmd_bytes = cmd.to_bytes()
        response = self.transport.send_command(cmd_bytes)
        return cmd.parse_response(response)

    def _write_register(self, register: Register, value: Union[int, float]):
        """
        Low-level register write.

        Args:
            register: Register to write
            value: Value in engineering units
        """
        cmd = WriteRegisterCommand(register, value, self.motor_id)
        cmd_bytes = cmd.to_bytes()
        response = self.transport.send_command(cmd_bytes)
        cmd.parse_response(response)  # Raises on NACK

    # Control and Status Methods

    def read_control_mode(self) -> MotorControlMode:
        """Read current control mode."""
        raw = self._read_register(RegisterMap.CONTROL_MODE)
        return MotorControlMode(int(raw))

    def write_control_mode(self, mode: MotorControlMode):
        """Set control mode (Torque or Speed)."""
        self._write_register(RegisterMap.CONTROL_MODE, mode.value)

    def read_status(self) -> int:
        """Read motor status flags."""
        return int(self._read_register(RegisterMap.STATUS))

    def read_fault_flags(self) -> int:
        """Read motor fault flags."""
        return int(self._read_register(RegisterMap.FAULT_FLAGS))

    # Speed Control Methods

    def read_speed_ref(self) -> int:
        """Read speed reference setpoint (RPM)."""
        return int(self._read_register(RegisterMap.SPEED_REF))

    def write_speed_ref(self, speed_rpm: int):
        """Set speed reference setpoint (RPM)."""
        self._write_register(RegisterMap.SPEED_REF, speed_rpm)

    def read_speed_meas(self) -> int:
        """Read measured motor speed (RPM)."""
        return int(self._read_register(RegisterMap.SPEED_MEAS))

    def read_speed_kp(self) -> int:
        """Read speed PID proportional gain."""
        return int(self._read_register(RegisterMap.SPEED_KP))

    def write_speed_kp(self, kp: int):
        """Set speed PID proportional gain."""
        self._write_register(RegisterMap.SPEED_KP, kp)

    def read_speed_ki(self) -> int:
        """Read speed PID integral gain."""
        return int(self._read_register(RegisterMap.SPEED_KI))

    def write_speed_ki(self, ki: int):
        """Set speed PID integral gain."""
        self._write_register(RegisterMap.SPEED_KI, ki)

    def read_speed_kd(self) -> int:
        """Read speed PID derivative gain."""
        return int(self._read_register(RegisterMap.SPEED_KD))

    def write_speed_kd(self, kd: int):
        """Set speed PID derivative gain."""
        self._write_register(RegisterMap.SPEED_KD, kd)

    def read_speed_pid(self) -> Tuple[int, int, int]:
        """Read all speed PID gains (Kp, Ki, Kd)."""
        kp = self.read_speed_kp()
        ki = self.read_speed_ki()
        kd = self.read_speed_kd()
        return (kp, ki, kd)

    def write_speed_pid(self, kp: int, ki: int, kd: int = 0):
        """Set all speed PID gains."""
        self.write_speed_kp(kp)
        self.write_speed_ki(ki)
        if kd != 0:
            self.write_speed_kd(kd)

    def read_ramp_final_speed(self) -> int:
        """Read ramp target speed (RPM)."""
        return int(self._read_register(RegisterMap.RAMP_FINAL_SPEED))

    def write_ramp_final_speed(self, speed_rpm: int):
        """Set ramp target speed (RPM)."""
        self._write_register(RegisterMap.RAMP_FINAL_SPEED, speed_rpm)

    # Torque Control Methods (Iq - quadrature current)

    def read_torque_ref(self) -> float:
        """Read torque reference (Amps)."""
        return self._read_register(RegisterMap.TORQUE_REF)

    def write_torque_ref(self, iq_amps: float):
        """Set torque reference (Amps)."""
        self._write_register(RegisterMap.TORQUE_REF, iq_amps)

    def read_torque_meas(self) -> float:
        """Read measured Iq current (Amps)."""
        return self._read_register(RegisterMap.TORQUE_MEAS)

    def read_torque_kp(self) -> int:
        """Read torque PID proportional gain."""
        return int(self._read_register(RegisterMap.TORQUE_KP))

    def write_torque_kp(self, kp: int):
        """Set torque PID proportional gain."""
        self._write_register(RegisterMap.TORQUE_KP, kp)

    def read_torque_ki(self) -> int:
        """Read torque PID integral gain."""
        return int(self._read_register(RegisterMap.TORQUE_KI))

    def write_torque_ki(self, ki: int):
        """Set torque PID integral gain."""
        self._write_register(RegisterMap.TORQUE_KI, ki)

    def read_torque_kd(self) -> int:
        """Read torque PID derivative gain."""
        return int(self._read_register(RegisterMap.TORQUE_KD))

    def write_torque_kd(self, kd: int):
        """Set torque PID derivative gain."""
        self._write_register(RegisterMap.TORQUE_KD, kd)

    def read_torque_pid(self) -> Tuple[int, int, int]:
        """Read all torque PID gains (Kp, Ki, Kd)."""
        kp = self.read_torque_kp()
        ki = self.read_torque_ki()
        kd = self.read_torque_kd()
        return (kp, ki, kd)

    def write_torque_pid(self, kp: int, ki: int, kd: int = 0):
        """Set all torque PID gains."""
        self.write_torque_kp(kp)
        self.write_torque_ki(ki)
        if kd != 0:
            self.write_torque_kd(kd)

    # Flux Control Methods (Id - direct current)

    def read_flux_ref(self) -> float:
        """Read flux reference (Amps)."""
        return self._read_register(RegisterMap.FLUX_REF)

    def write_flux_ref(self, id_amps: float):
        """Set flux reference (Amps)."""
        self._write_register(RegisterMap.FLUX_REF, id_amps)

    def read_flux_meas(self) -> float:
        """Read measured Id current (Amps)."""
        return self._read_register(RegisterMap.FLUX_MEAS)

    def read_flux_kp(self) -> int:
        """Read flux PID proportional gain."""
        return int(self._read_register(RegisterMap.FLUX_KP))

    def write_flux_kp(self, kp: int):
        """Set flux PID proportional gain."""
        self._write_register(RegisterMap.FLUX_KP, kp)

    def read_flux_ki(self) -> int:
        """Read flux PID integral gain."""
        return int(self._read_register(RegisterMap.FLUX_KI))

    def write_flux_ki(self, ki: int):
        """Set flux PID integral gain."""
        self._write_register(RegisterMap.FLUX_KI, ki)

    def read_flux_kd(self) -> int:
        """Read flux PID derivative gain."""
        return int(self._read_register(RegisterMap.FLUX_KD))

    def write_flux_kd(self, kd: int):
        """Set flux PID derivative gain."""
        self._write_register(RegisterMap.FLUX_KD, kd)

    def read_flux_pid(self) -> Tuple[int, int, int]:
        """Read all flux PID gains (Kp, Ki, Kd)."""
        kp = self.read_flux_kp()
        ki = self.read_flux_ki()
        kd = self.read_flux_kd()
        return (kp, ki, kd)

    def write_flux_pid(self, kp: int, ki: int, kd: int = 0):
        """Set all flux PID gains."""
        self.write_flux_kp(kp)
        self.write_flux_ki(ki)
        if kd != 0:
            self.write_flux_kd(kd)

    # Measurement Methods

    def read_bus_voltage(self) -> float:
        """Read DC bus voltage (Volts)."""
        return self._read_register(RegisterMap.BUS_VOLTAGE)

    def read_temperature(self) -> float:
        """Read heatsink temperature (Celsius)."""
        return self._read_register(RegisterMap.HEATS_TEMP)

    def read_motor_power(self) -> float:
        """Read motor power (Watts)."""
        return self._read_register(RegisterMap.MOTOR_POWER)

    # Phase Current Measurements

    def read_phase_currents(self) -> Tuple[float, float]:
        """Read phase A and B currents (Amps)."""
        ia = self._read_register(RegisterMap.IA_MEAS)
        ib = self._read_register(RegisterMap.IB_MEAS)
        return (ia, ib)

    def read_alphabeta_currents(self) -> Tuple[float, float]:
        """Read alpha-beta currents (Amps)."""
        ialpha = self._read_register(RegisterMap.IALPHA_MEAS)
        ibeta = self._read_register(RegisterMap.IBETA_MEAS)
        return (ialpha, ibeta)

    def read_dq_currents(self) -> Tuple[float, float]:
        """Read D-Q currents (Amps)."""
        iq = self._read_register(RegisterMap.IQ_MEAS)
        id_ = self._read_register(RegisterMap.ID_MEAS)
        return (id_, iq)

    # Voltage Measurements

    def read_dq_voltages(self) -> Tuple[float, float]:
        """Read D-Q voltages (Volts)."""
        vq = self._read_register(RegisterMap.VQ_MEAS)
        vd = self._read_register(RegisterMap.VD_MEAS)
        return (vd, vq)

    def read_alphabeta_voltages(self) -> Tuple[float, float]:
        """Read alpha-beta voltages (Volts)."""
        valpha = self._read_register(RegisterMap.VALPHA_MEAS)
        vbeta = self._read_register(RegisterMap.VBETA_MEAS)
        return (valpha, vbeta)

    # Angle and Speed

    def read_electrical_angle(self) -> float:
        """Read electrical angle (degrees)."""
        return self._read_register(RegisterMap.MEAS_EL_ANGLE)

    def read_rotor_speed(self) -> int:
        """Read rotor speed (RPM)."""
        return int(self._read_register(RegisterMap.MEAS_ROT_SPEED))

    # Speed Limits

    def read_max_speed(self) -> int:
        """Read maximum application speed (RPM)."""
        return int(self._read_register(RegisterMap.MAX_APP_SPEED))

    def write_max_speed(self, max_rpm: int):
        """Set maximum application speed (RPM)."""
        self._write_register(RegisterMap.MAX_APP_SPEED, max_rpm)

    def read_min_speed(self) -> int:
        """Read minimum application speed (RPM)."""
        return int(self._read_register(RegisterMap.MIN_APP_SPEED))

    def write_min_speed(self, min_rpm: int):
        """Set minimum application speed (RPM)."""
        self._write_register(RegisterMap.MIN_APP_SPEED, min_rpm)

    # Board Information

    def read_control_board_id(self) -> int:
        """Read control board ID."""
        return int(self._read_register(RegisterMap.CTRBDID))

    def read_power_board_id(self) -> int:
        """Read power board ID."""
        return int(self._read_register(RegisterMap.PWBDID))
