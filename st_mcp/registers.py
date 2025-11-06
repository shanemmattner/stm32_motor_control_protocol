"""
Motor Control Register Definitions

This module defines all STM32 motor control registers based on the
Frame Communication Protocol specification from ST's Motor Control SDK.

Register definitions include:
- Register ID (address)
- Data type (uint8_t, int16_t, int32_t, uint16_t)
- Size in bytes
- Scaling factor for engineering units
- Valid range (min/max values)
- Human-readable description

Reference: frame_communication_protocol.h from STM32-Motor-Control-Interface
"""

from dataclasses import dataclass
from typing import Optional, Dict, Union
from enum import IntEnum


# Type definitions matching C types (use unique values for proper enum behavior)
class RegisterType(IntEnum):
    """Register data types"""
    UINT8 = 10
    INT8 = 11
    UINT16 = 20
    INT16 = 21
    UINT32 = 40
    INT32 = 41


@dataclass
class Register:
    """Motor control register definition"""
    id: int
    name: str
    dtype: RegisterType
    size: int
    scaling: float = 1.0
    unit: str = ""
    description: str = ""
    min_value: Optional[Union[int, float]] = None
    max_value: Optional[Union[int, float]] = None
    read_only: bool = False

    def encode_value(self, value: Union[int, float]) -> int:
        """Convert engineering unit value to raw register value"""
        raw = int(value * self.scaling)

        # Validate range
        if self.min_value is not None and value < self.min_value:
            raise ValueError(f"{self.name}: value {value} below minimum {self.min_value}")
        if self.max_value is not None and value > self.max_value:
            raise ValueError(f"{self.name}: value {value} above maximum {self.max_value}")

        return raw

    def decode_value(self, raw: int) -> Union[int, float]:
        """Convert raw register value to engineering units"""
        # Handle signed types
        if self.dtype in [RegisterType.INT8, RegisterType.INT16, RegisterType.INT32]:
            # Convert from unsigned to signed if needed
            if raw >= (1 << (self.size * 8 - 1)):
                raw -= (1 << (self.size * 8))

        value = raw / self.scaling
        return value

    @property
    def is_signed(self) -> bool:
        """Check if register type is signed"""
        return self.dtype in [RegisterType.INT8, RegisterType.INT16, RegisterType.INT32]


class RegisterMap:
    """Central register map for all motor control parameters"""

    # Control and Status Registers
    TARGET_MOTOR = Register(
        id=0x00,
        name="target_motor",
        dtype=RegisterType.UINT8,
        size=1,
        description="Target motor ID (0-7)"
    )

    FAULT_FLAGS = Register(
        id=0x01,
        name="fault_flags",
        dtype=RegisterType.UINT32,
        size=4,
        description="Motor fault flags bitfield",
        read_only=True
    )

    STATUS = Register(
        id=0x02,
        name="status",
        dtype=RegisterType.UINT8,
        size=1,
        description="Motor status flags",
        read_only=True
    )

    CONTROL_MODE = Register(
        id=0x03,
        name="control_mode",
        dtype=RegisterType.UINT8,
        size=1,
        description="Control mode: 0=Torque, 1=Speed"
    )

    # Speed Control Registers
    SPEED_REF = Register(
        id=0x04,
        name="speed_ref",
        dtype=RegisterType.INT32,
        size=4,
        scaling=1.0,
        unit="RPM",
        description="Speed reference setpoint"
    )

    SPEED_KP = Register(
        id=0x05,
        name="speed_kp",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Speed PID proportional gain"
    )

    SPEED_KI = Register(
        id=0x06,
        name="speed_ki",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Speed PID integral gain"
    )

    SPEED_KD = Register(
        id=0x07,
        name="speed_kd",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Speed PID derivative gain"
    )

    # Torque Control Registers (Iq - quadrature current)
    TORQUE_REF = Register(
        id=0x08,
        name="torque_ref",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,  # Firmware uses 0.0001 A units
        unit="A",
        description="Torque reference (Iq current)"
    )

    TORQUE_KP = Register(
        id=0x09,
        name="torque_kp",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Torque PID proportional gain"
    )

    TORQUE_KI = Register(
        id=0x0A,
        name="torque_ki",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Torque PID integral gain"
    )

    TORQUE_KD = Register(
        id=0x0B,
        name="torque_kd",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Torque PID derivative gain"
    )

    # Flux Control Registers (Id - direct current)
    FLUX_REF = Register(
        id=0x0C,
        name="flux_ref",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,  # Firmware uses 0.0001 A units
        unit="A",
        description="Flux reference (Id current)"
    )

    FLUX_KP = Register(
        id=0x0D,
        name="flux_kp",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Flux PID proportional gain"
    )

    FLUX_KI = Register(
        id=0x0E,
        name="flux_ki",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Flux PID integral gain"
    )

    FLUX_KD = Register(
        id=0x0F,
        name="flux_kd",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Flux PID derivative gain"
    )

    # Measurement Registers
    BUS_VOLTAGE = Register(
        id=0x19,
        name="bus_voltage",
        dtype=RegisterType.UINT16,
        size=2,
        scaling=100.0,  # Firmware uses 0.01 V units
        unit="V",
        description="DC bus voltage",
        read_only=True
    )

    HEATS_TEMP = Register(
        id=0x1A,
        name="heatsink_temp",
        dtype=RegisterType.UINT16,
        size=2,
        scaling=10.0,  # Firmware uses 0.1 °C units
        unit="°C",
        description="Heatsink temperature",
        read_only=True
    )

    MOTOR_POWER = Register(
        id=0x1B,
        name="motor_power",
        dtype=RegisterType.UINT16,
        size=2,
        scaling=1.0,
        unit="W",
        description="Motor power",
        read_only=True
    )

    SPEED_MEAS = Register(
        id=0x1E,
        name="speed_meas",
        dtype=RegisterType.INT32,
        size=4,
        scaling=1.0,
        unit="RPM",
        description="Measured motor speed",
        read_only=True
    )

    TORQUE_MEAS = Register(
        id=0x1F,
        name="torque_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Measured Iq current",
        read_only=True
    )

    FLUX_MEAS = Register(
        id=0x20,
        name="flux_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Measured Id current",
        read_only=True
    )

    # Phase Current Measurements
    IA_MEAS = Register(
        id=0x23,
        name="ia_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Phase A current",
        read_only=True
    )

    IB_MEAS = Register(
        id=0x24,
        name="ib_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Phase B current",
        read_only=True
    )

    IALPHA_MEAS = Register(
        id=0x25,
        name="ialpha_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Alpha current (Clarke transform)",
        read_only=True
    )

    IBETA_MEAS = Register(
        id=0x26,
        name="ibeta_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Beta current (Clarke transform)",
        read_only=True
    )

    IQ_MEAS = Register(
        id=0x27,
        name="iq_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Q current (Park transform)",
        read_only=True
    )

    ID_MEAS = Register(
        id=0x28,
        name="id_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="D current (Park transform)",
        read_only=True
    )

    # Voltage Measurements
    VQ_MEAS = Register(
        id=0x2B,
        name="vq_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="V",
        description="Q voltage",
        read_only=True
    )

    VD_MEAS = Register(
        id=0x2C,
        name="vd_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="V",
        description="D voltage",
        read_only=True
    )

    VALPHA_MEAS = Register(
        id=0x2D,
        name="valpha_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="V",
        description="Alpha voltage",
        read_only=True
    )

    VBETA_MEAS = Register(
        id=0x2E,
        name="vbeta_meas",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="V",
        description="Beta voltage",
        read_only=True
    )

    # Angle and Speed Measurements
    MEAS_EL_ANGLE = Register(
        id=0x2F,
        name="elec_angle",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="degrees",
        description="Electrical angle",
        read_only=True
    )

    MEAS_ROT_SPEED = Register(
        id=0x30,
        name="rot_speed",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        unit="RPM",
        description="Rotor speed",
        read_only=True
    )

    # Speed Limits
    MAX_APP_SPEED = Register(
        id=0x3F,
        name="max_app_speed",
        dtype=RegisterType.UINT32,
        size=4,
        scaling=1.0,
        unit="RPM",
        description="Maximum application speed"
    )

    MIN_APP_SPEED = Register(
        id=0x40,
        name="min_app_speed",
        dtype=RegisterType.UINT32,
        size=4,
        scaling=1.0,
        unit="RPM",
        description="Minimum application speed"
    )

    IQ_SPEED_MODE = Register(
        id=0x41,
        name="iq_speed_mode",
        dtype=RegisterType.INT16,
        size=2,
        scaling=10000.0,
        unit="A",
        description="Iq reference in speed mode",
        read_only=True
    )

    # Ramp Control
    RAMP_FINAL_SPEED = Register(
        id=0x5B,
        name="ramp_final_speed",
        dtype=RegisterType.INT32,
        size=4,
        scaling=1.0,
        unit="RPM",
        description="Target speed for ramp"
    )

    # Speed PID Divisors
    SPEED_KP_DIV = Register(
        id=0x6E,
        name="speed_kp_div",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Speed Kp divisor"
    )

    SPEED_KI_DIV = Register(
        id=0x6F,
        name="speed_ki_div",
        dtype=RegisterType.INT16,
        size=2,
        scaling=1.0,
        description="Speed Ki divisor"
    )

    # Board IDs
    CTRBDID = Register(
        id=0x72,
        name="control_board_id",
        dtype=RegisterType.UINT32,
        size=4,
        description="Control board ID",
        read_only=True
    )

    PWBDID = Register(
        id=0x73,
        name="power_board_id",
        dtype=RegisterType.UINT32,
        size=4,
        description="Power board ID",
        read_only=True
    )

    # Register lookup maps
    _by_id: Dict[int, Register] = {}
    _by_name: Dict[str, Register] = {}

    @classmethod
    def initialize(cls):
        """Build lookup maps from register definitions"""
        if cls._by_id:
            return  # Already initialized

        # Find all Register attributes
        for attr_name in dir(cls):
            if attr_name.startswith('_'):
                continue
            attr = getattr(cls, attr_name)
            if isinstance(attr, Register):
                cls._by_id[attr.id] = attr
                cls._by_name[attr.name] = attr

    @classmethod
    def get_by_id(cls, reg_id: int) -> Optional[Register]:
        """Get register by ID"""
        cls.initialize()
        return cls._by_id.get(reg_id)

    @classmethod
    def get_by_name(cls, name: str) -> Optional[Register]:
        """Get register by name"""
        cls.initialize()
        return cls._by_name.get(name)

    @classmethod
    def all_registers(cls) -> Dict[int, Register]:
        """Get all register definitions"""
        cls.initialize()
        return cls._by_id.copy()


# Motor control mode enumeration
class MotorControlMode(IntEnum):
    """Motor control modes"""
    TORQUE = 0  # Torque control (Iq reference)
    SPEED = 1   # Speed control (RPM reference)


# Initialize register maps on import
RegisterMap.initialize()
