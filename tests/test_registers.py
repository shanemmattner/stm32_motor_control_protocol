"""
Tests for register definitions and encoding/decoding
"""

import pytest
from st_mcp.registers import Register, RegisterMap, RegisterType, MotorControlMode


class TestRegister:
    """Test Register class"""

    def test_register_encode_decode_uint16(self):
        """Test encoding and decoding uint16 register"""
        reg = Register(
            id=0x19,
            name="bus_voltage",
            dtype=RegisterType.UINT16,
            size=2,
            scaling=100.0,  # 0.01 V units
            unit="V"
        )

        # Encode 24.5 V
        raw = reg.encode_value(24.5)
        assert raw == 2450

        # Decode back
        value = reg.decode_value(2450)
        assert abs(value - 24.5) < 0.001

    def test_register_encode_decode_int16(self):
        """Test encoding and decoding signed int16 register"""
        reg = Register(
            id=0x08,
            name="torque_ref",
            dtype=RegisterType.INT16,
            size=2,
            scaling=10000.0,  # 0.0001 A units
            unit="A"
        )

        # Encode 2.5 A
        raw = reg.encode_value(2.5)
        assert raw == 25000

        # Decode back
        value = reg.decode_value(25000)
        assert abs(value - 2.5) < 0.0001

        # Test negative value
        raw = reg.encode_value(-1.5)
        assert raw == -15000

        value = reg.decode_value(-15000)
        assert abs(value - (-1.5)) < 0.0001

    def test_register_encode_decode_int32(self):
        """Test encoding and decoding int32 register"""
        reg = Register(
            id=0x04,
            name="speed_ref",
            dtype=RegisterType.INT32,
            size=4,
            scaling=1.0,
            unit="RPM"
        )

        # Encode 1500 RPM
        raw = reg.encode_value(1500)
        assert raw == 1500

        # Decode back
        value = reg.decode_value(1500)
        assert value == 1500

    def test_register_is_signed(self):
        """Test signed type detection"""
        uint_reg = Register(
            id=0x19,
            name="bus_voltage",
            dtype=RegisterType.UINT16,
            size=2
        )
        assert not uint_reg.is_signed

        int_reg = Register(
            id=0x08,
            name="torque_ref",
            dtype=RegisterType.INT16,
            size=2
        )
        assert int_reg.is_signed

    def test_register_value_validation(self):
        """Test value range validation"""
        reg = Register(
            id=0x04,
            name="speed_ref",
            dtype=RegisterType.INT32,
            size=4,
            min_value=-5000,
            max_value=5000
        )

        # Valid value
        raw = reg.encode_value(3000)
        assert raw == 3000

        # Below minimum
        with pytest.raises(ValueError, match="below minimum"):
            reg.encode_value(-6000)

        # Above maximum
        with pytest.raises(ValueError, match="above maximum"):
            reg.encode_value(6000)


class TestRegisterMap:
    """Test RegisterMap class"""

    def test_register_map_initialized(self):
        """Test that register map is initialized"""
        assert len(RegisterMap._by_id) > 0
        assert len(RegisterMap._by_name) > 0

    def test_get_register_by_id(self):
        """Test getting register by ID"""
        reg = RegisterMap.get_by_id(0x19)  # BUS_VOLTAGE
        assert reg is not None
        assert reg.name == "bus_voltage"
        assert reg.id == 0x19

    def test_get_register_by_name(self):
        """Test getting register by name"""
        reg = RegisterMap.get_by_name("bus_voltage")
        assert reg is not None
        assert reg.id == 0x19
        assert reg.unit == "V"

    def test_get_invalid_register(self):
        """Test getting non-existent register"""
        reg_id = RegisterMap.get_by_id(0xFF)
        assert reg_id is None

        reg_name = RegisterMap.get_by_name("nonexistent")
        assert reg_name is None

    def test_all_registers(self):
        """Test getting all registers"""
        all_regs = RegisterMap.all_registers()
        assert len(all_regs) > 0
        assert 0x19 in all_regs  # BUS_VOLTAGE exists

    def test_critical_registers_exist(self):
        """Test that critical registers are defined"""
        critical_registers = [
            (0x01, "fault_flags"),
            (0x02, "status"),
            (0x03, "control_mode"),
            (0x04, "speed_ref"),
            (0x08, "torque_ref"),
            (0x0C, "flux_ref"),
            (0x19, "bus_voltage"),
            (0x1A, "heatsink_temp"),
            (0x1E, "speed_meas"),
            (0x1F, "torque_meas"),
            (0x20, "flux_meas"),
        ]

        for reg_id, reg_name in critical_registers:
            reg = RegisterMap.get_by_id(reg_id)
            assert reg is not None, f"Register 0x{reg_id:02X} not found"
            assert reg.name == reg_name, f"Register 0x{reg_id:02X} has wrong name"


class TestMotorControlMode:
    """Test MotorControlMode enum"""

    def test_control_mode_values(self):
        """Test control mode enum values"""
        assert MotorControlMode.TORQUE == 0
        assert MotorControlMode.SPEED == 1

    def test_control_mode_names(self):
        """Test control mode names"""
        assert MotorControlMode.TORQUE.name == "TORQUE"
        assert MotorControlMode.SPEED.name == "SPEED"
