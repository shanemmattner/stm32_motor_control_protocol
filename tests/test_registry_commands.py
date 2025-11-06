"""
Tests for registry commands
"""

import pytest
from st_mcp.registers import RegisterMap
from st_mcp.commands.registry import ReadRegisterCommand, WriteRegisterCommand
from st_mcp.exceptions import RegistryNACKError, RegistryError
from st_mcp.frame import FrameCode, ResponseCode, ErrorCode


class TestReadRegisterCommand:
    """Test ReadRegisterCommand"""

    def test_read_command_to_bytes(self):
        """Test serialization of read command"""
        reg = RegisterMap.BUS_VOLTAGE
        cmd = ReadRegisterCommand(reg, motor_id=0)

        cmd_bytes = cmd.to_bytes()

        # Should be GET_REG frame with register ID
        assert cmd_bytes[0] == 0x02  # GET_REG frame code
        assert cmd_bytes[2] == 0x19  # BUS_VOLTAGE register ID

    def test_read_command_parse_ack_response(self):
        """Test parsing successful read response"""
        reg = RegisterMap.BUS_VOLTAGE
        cmd = ReadRegisterCommand(reg, motor_id=0)

        # ACK response with voltage = 100.0V (10000 * 0.01)
        # 10000 = 0x2710 in little-endian = 0x10, 0x27
        # Checksum: 0xF0 + 0x02 + 0x10 + 0x27 = 0x129 -> (0x29 & 0xFF) + (0x129 >> 8) = 0x29 + 0x01 = 0x2A
        response = bytes([0xF0, 0x02, 0x10, 0x27, 0x2A])

        value = cmd.parse_response(response)

        # Should decode to 100.0 V (10000 / 100.0)
        assert abs(value - 100.0) < 0.01

    def test_read_command_parse_nack_response(self):
        """Test parsing NACK response"""
        reg = RegisterMap.BUS_VOLTAGE
        cmd = ReadRegisterCommand(reg, motor_id=0)

        # NACK response with BAD_CRC error
        response = bytes([0xFF, 0x0A, 0x09])

        with pytest.raises(RegistryNACKError, match="BAD_CRC"):
            cmd.parse_response(response)

    def test_read_command_invalid_response(self):
        """Test parsing invalid response"""
        reg = RegisterMap.BUS_VOLTAGE
        cmd = ReadRegisterCommand(reg, motor_id=0)

        # Too short response
        response = bytes([0xF0])

        with pytest.raises(RegistryError, match="Invalid response"):
            cmd.parse_response(response)


class TestWriteRegisterCommand:
    """Test WriteRegisterCommand"""

    def test_write_command_to_bytes(self):
        """Test serialization of write command"""
        reg = RegisterMap.SPEED_REF
        cmd = WriteRegisterCommand(reg, value=1000, motor_id=0)

        cmd_bytes = cmd.to_bytes()

        # Should be SET_REG frame with register ID and value
        assert cmd_bytes[0] == 0x01  # SET_REG frame code
        assert cmd_bytes[2] == 0x04  # SPEED_REF register ID
        # 1000 in little-endian
        assert cmd_bytes[3:7] == bytes([0xE8, 0x03, 0x00, 0x00])

    def test_write_command_with_scaling(self):
        """Test write command with scaled value"""
        reg = RegisterMap.TORQUE_REF
        cmd = WriteRegisterCommand(reg, value=2.5, motor_id=0)  # 2.5 Amps

        cmd_bytes = cmd.to_bytes()

        # 2.5 * 10000 = 25000 = 0x61A8 in little-endian = 0xA8, 0x61
        assert cmd_bytes[3:5] == bytes([0xA8, 0x61])

    def test_write_command_parse_ack_response(self):
        """Test parsing successful write response"""
        reg = RegisterMap.SPEED_REF
        cmd = WriteRegisterCommand(reg, value=1000, motor_id=0)

        # ACK response
        response = bytes([0xF0, 0x00, 0xF0])

        result = cmd.parse_response(response)
        assert result is True

    def test_write_command_parse_nack_response(self):
        """Test parsing NACK write response"""
        reg = RegisterMap.SPEED_REF
        cmd = WriteRegisterCommand(reg, value=1000, motor_id=0)

        # NACK response with WRONG_SET error
        response = bytes([0xFF, 0x05, 0x04])

        with pytest.raises(RegistryNACKError, match="WRONG_SET"):
            cmd.parse_response(response)

    def test_write_read_only_register(self):
        """Test writing to read-only register"""
        reg = RegisterMap.BUS_VOLTAGE  # Read-only

        with pytest.raises(ValueError, match="read-only"):
            WriteRegisterCommand(reg, value=24.0, motor_id=0)

    def test_write_invalid_value(self):
        """Test writing invalid value"""
        reg = RegisterMap.SPEED_REF
        reg.min_value = -5000
        reg.max_value = 5000

        with pytest.raises(ValueError, match="Invalid value"):
            WriteRegisterCommand(reg, value=10000, motor_id=0)
