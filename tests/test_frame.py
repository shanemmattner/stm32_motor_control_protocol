"""
Tests for frame construction and parsing
"""

import pytest
from st_mcp.frame import (
    Frame, FrameCode, FrameResponse, ResponseCode, ErrorCode, ASPEPWrapper,
    create_read_register_frame, create_write_register_frame, create_execute_command_frame
)


class TestFrame:
    """Test Frame class"""

    def test_frame_header_construction(self):
        """Test frame header with motor ID and frame code"""
        frame = Frame(FrameCode.GET_REG, motor_id=0)
        assert frame.header == 0x02  # Motor 0, Frame code 0x02

        frame = Frame(FrameCode.SET_REG, motor_id=1)
        assert frame.header == 0x21  # Motor 1, Frame code 0x01

        frame = Frame(FrameCode.EXECUTE_CMD, motor_id=7)
        assert frame.header == 0xE3  # Motor 7, Frame code 0x03

    def test_frame_add_byte(self):
        """Test adding single byte to payload"""
        frame = Frame(FrameCode.GET_REG)
        frame.add_byte(0x19)  # Bus voltage register
        assert frame.payload == bytearray([0x19])
        assert frame.payload_length == 1

    def test_frame_add_bytes(self):
        """Test adding multiple bytes to payload"""
        frame = Frame(FrameCode.SET_REG)
        frame.add_bytes(b"\x19\x10\x27")
        assert frame.payload == bytearray([0x19, 0x10, 0x27])
        assert frame.payload_length == 3

    def test_frame_add_register_value(self):
        """Test adding register value in little-endian"""
        frame = Frame(FrameCode.SET_REG)
        frame.add_byte(0x04)  # Speed ref register
        frame.add_register_value(1500, size=4, signed=True)

        # 1500 in little-endian: DC 05 00 00
        expected_payload = bytearray([0x04, 0xDC, 0x05, 0x00, 0x00])
        assert frame.payload == expected_payload

    def test_frame_checksum_calculation(self):
        """Test checksum algorithm"""
        # Example from STM32-Motor-Control-Interface
        # Read register 0x19 (bus voltage)
        frame = Frame(FrameCode.GET_REG, motor_id=0)
        frame.add_byte(0x19)

        # Header: 0x02, Length: 0x01, Payload: 0x19
        # Accumulator: 0x02 + 0x01 + 0x19 = 0x1C
        # Checksum: (0x1C & 0xFF) + (0x1C >> 8) = 0x1C + 0x00 = 0x1C
        checksum = frame.compute_checksum()
        assert checksum == 0x1C

    def test_frame_to_bytes(self):
        """Test frame serialization"""
        frame = Frame(FrameCode.GET_REG, motor_id=0)
        frame.add_byte(0x19)  # Bus voltage register

        frame_bytes = frame.to_bytes()

        # Expected: [Header=0x02, Length=0x01, Payload=0x19, Checksum=0x1C]
        assert frame_bytes == bytes([0x02, 0x01, 0x19, 0x1C])

    def test_frame_write_register(self):
        """Test creating write register frame"""
        # Write speed_ref = 1000 RPM
        frame = Frame(FrameCode.SET_REG, motor_id=0)
        frame.add_byte(0x04)  # SPEED_REF register
        frame.add_register_value(1000, size=4, signed=True)

        frame_bytes = frame.to_bytes()

        # Header: 0x01 (SET_REG, motor 0)
        # Length: 0x05 (1 byte reg ID + 4 bytes value)
        # Payload: 0x04 (reg), 0xE8, 0x03, 0x00, 0x00 (1000 in little-endian)
        # Checksum calculation omitted
        assert frame_bytes[0] == 0x01  # Header
        assert frame_bytes[1] == 0x05  # Length
        assert frame_bytes[2] == 0x04  # Register ID
        assert frame_bytes[3:7] == bytes([0xE8, 0x03, 0x00, 0x00])  # 1000 LE


class TestFrameResponse:
    """Test FrameResponse class"""

    def test_ack_response_parsing(self):
        """Test parsing ACK response"""
        # ACK response with 2-byte payload (e.g., voltage reading)
        # [ACK=0xF0, Length=0x02, Payload=0x10,0x27 (10000 = 100V), Checksum]
        response_bytes = bytes([0xF0, 0x02, 0x10, 0x27, 0x19])

        response = FrameResponse(response_bytes)

        assert response.is_ack
        assert not response.is_nack
        assert response.payload_length == 2
        assert response.payload == bytes([0x10, 0x27])
        assert response.error_code == ErrorCode.NONE

    def test_nack_response_parsing(self):
        """Test parsing NACK response"""
        # NACK response with error code
        # [NACK=0xFF, Error=0x0A (BAD_CRC), Checksum]
        response_bytes = bytes([0xFF, 0x0A, 0x09])

        response = FrameResponse(response_bytes)

        assert response.is_nack
        assert not response.is_ack
        assert response.error_code == ErrorCode.BAD_CRC
        assert response.payload_length == 0
        assert response.payload == b""

    def test_get_register_value_uint16(self):
        """Test extracting uint16 register value"""
        # ACK with 2-byte value: 10000 (0x2710) in little-endian = 0x10, 0x27
        response_bytes = bytes([0xF0, 0x02, 0x10, 0x27, 0x19])
        response = FrameResponse(response_bytes)

        value = response.get_register_value(size=2, signed=False)
        assert value == 10000

    def test_get_register_value_int16(self):
        """Test extracting signed int16 register value"""
        # Positive value: 25000 (0x61A8) in little-endian = 0xA8, 0x61
        response_bytes = bytes([0xF0, 0x02, 0xA8, 0x61, 0xF9])
        response = FrameResponse(response_bytes)

        value = response.get_register_value(size=2, signed=True)
        assert value == 25000

        # Negative value: -15000 (0xC568) in little-endian = 0x68, 0xC5
        response_bytes = bytes([0xF0, 0x02, 0x68, 0xC5, 0x7E])
        response = FrameResponse(response_bytes)

        value = response.get_register_value(size=2, signed=True)
        assert value == -15000

    def test_get_register_value_int32(self):
        """Test extracting int32 register value"""
        # 1500 (0x000005DC) in little-endian = 0xDC, 0x05, 0x00, 0x00
        response_bytes = bytes([0xF0, 0x04, 0xDC, 0x05, 0x00, 0x00, 0xEB])
        response = FrameResponse(response_bytes)

        value = response.get_register_value(size=4, signed=True)
        assert value == 1500

    def test_response_too_short(self):
        """Test handling of too-short response"""
        with pytest.raises(ValueError, match="too short"):
            FrameResponse(bytes([0xF0]))

    def test_unknown_response_code(self):
        """Test handling of unknown response code"""
        with pytest.raises(ValueError, match="Unknown response code"):
            FrameResponse(bytes([0x55, 0x00, 0x55]))


class TestASPEPWrapper:
    """Test ASPEP wrapper"""

    def test_aspep_wrap(self):
        """Test wrapping frame in ASPEP packet"""
        frame = Frame(FrameCode.GET_REG, motor_id=0)
        frame.add_byte(0x19)

        wrapped = ASPEPWrapper.wrap(frame)

        # Should have 4-byte ASPEP header + frame
        assert len(wrapped) >= 4
        assert wrapped[:4] == ASPEPWrapper.SYNC_HEADER
        assert wrapped[4:] == frame.to_bytes()

    def test_aspep_unwrap(self):
        """Test extracting frame from ASPEP packet"""
        frame_bytes = bytes([0x02, 0x01, 0x19, 0x1C])
        aspep_packet = ASPEPWrapper.SYNC_HEADER + frame_bytes

        unwrapped = ASPEPWrapper.unwrap(aspep_packet)

        assert unwrapped == frame_bytes


class TestHelperFunctions:
    """Test helper functions"""

    def test_create_read_register_frame(self):
        """Test read register frame helper"""
        frame = create_read_register_frame(register_id=0x19, motor_id=0)

        assert frame.frame_code == FrameCode.GET_REG
        assert frame.motor_id == 0
        assert frame.payload == bytearray([0x19])

    def test_create_write_register_frame(self):
        """Test write register frame helper"""
        frame = create_write_register_frame(
            register_id=0x04,
            value=1000,
            size=4,
            signed=True,
            motor_id=0
        )

        assert frame.frame_code == FrameCode.SET_REG
        assert frame.motor_id == 0
        assert frame.payload[0] == 0x04  # Register ID
        # 1000 in little-endian
        assert frame.payload[1:5] == bytearray([0xE8, 0x03, 0x00, 0x00])

    def test_create_execute_command_frame(self):
        """Test execute command frame helper"""
        frame = create_execute_command_frame(command_id=0x01, motor_id=0)

        assert frame.frame_code == FrameCode.EXECUTE_CMD
        assert frame.motor_id == 0
        assert frame.payload == bytearray([0x01])
