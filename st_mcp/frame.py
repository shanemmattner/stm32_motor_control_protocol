"""
Frame Communication Protocol Implementation

This module implements packet framing for STM32 motor control communication.
Frames are wrapped in ASPEP (Asymmetric Serial Packet Exchange Protocol) packets
for transmission over serial.

Frame Structure (Frame Communication Protocol):
    [Header: 1 byte] [Length: 1 byte] [Payload: N bytes] [Checksum: 1 byte]

ASPEP Wrapper:
    [ASPEP Header: 4 bytes] [Frame: variable]

Reference: STM32-Motor-Control-Interface implementation
"""

from typing import Optional
from enum import IntEnum


class FrameCode(IntEnum):
    """Frame type codes (master to slave)"""
    SET_REG = 0x01          # Write register
    GET_REG = 0x02          # Read register
    EXECUTE_CMD = 0x03      # Execute motor command
    GET_BOARD_INFO = 0x06   # Get board information
    SET_RAMP = 0x07         # Set speed ramp
    GET_REVUP_DATA = 0x08   # Get rev-up data
    SET_REVUP_DATA = 0x09   # Set rev-up data
    SET_CURRENT_REF = 0x0A  # Set current reference
    GET_FW_VERSION = 0x0C   # Get firmware version
    GET_DATA_ELEMENT = 0x0D # Get data element
    CONFIG_DYN_STRUCT = 0x0E # Configure dynamic structure


class ResponseCode(IntEnum):
    """Response codes (slave to master)"""
    ACK = 0xF0   # Acknowledge
    NACK = 0xFF  # Negative acknowledge


class ErrorCode(IntEnum):
    """Error codes returned in NACK responses"""
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


class Frame:
    """
    Frame Communication Protocol packet builder.

    Constructs packets according to the Frame Communication Protocol
    specification used by ST Motor Control SDK.
    """

    def __init__(self, frame_code: FrameCode, motor_id: int = 0):
        """
        Initialize a new frame.

        Args:
            frame_code: Type of frame (SET_REG, GET_REG, EXECUTE_CMD, etc.)
            motor_id: Target motor ID (0-7)
        """
        self.frame_code = frame_code
        self.motor_id = motor_id & 0x07  # 3-bit motor ID
        self.payload = bytearray()

    @property
    def header(self) -> int:
        """
        Construct frame header byte.

        Header format: [Motor ID: 3 bits] [Frame Code: 5 bits]
        Bits 7-5: Motor ID (0-7)
        Bits 4-0: Frame code
        """
        return ((self.motor_id & 0x07) << 5) | (self.frame_code & 0x1F)

    @property
    def payload_length(self) -> int:
        """Get payload length"""
        return len(self.payload)

    def add_byte(self, value: int):
        """Add single byte to payload"""
        self.payload.append(value & 0xFF)

    def add_bytes(self, data: bytes):
        """Add multiple bytes to payload"""
        self.payload.extend(data)

    def add_register_value(self, value: int, size: int, signed: bool = False):
        """
        Add register value in little-endian format.

        Args:
            value: Register value
            size: Size in bytes (1, 2, or 4)
            signed: Whether value is signed
        """
        # Convert to bytes in little-endian order
        value_bytes = value.to_bytes(size, byteorder='little', signed=signed)
        self.payload.extend(value_bytes)

    def compute_checksum(self) -> int:
        """
        Calculate frame checksum using STM32 Motor Control algorithm.

        Algorithm:
        1. Sum header + length + all payload bytes (16-bit accumulator)
        2. Add low byte and high byte of accumulator
        3. Mask to 8 bits

        This matches the implementation in STM32-Motor-Control-Interface.
        """
        # Use 16-bit accumulator
        accumulator = self.header + self.payload_length

        # Add all payload bytes
        for byte in self.payload:
            accumulator += byte

        # Add low byte and high byte together
        checksum = (accumulator & 0xFF) + ((accumulator >> 8) & 0xFF)

        return checksum & 0xFF

    def to_bytes(self) -> bytes:
        """
        Serialize frame to bytes.

        Returns:
            Complete frame: [Header] [Length] [Payload] [Checksum]
        """
        frame = bytearray()
        frame.append(self.header)
        frame.append(self.payload_length)
        frame.extend(self.payload)
        frame.append(self.compute_checksum())
        return bytes(frame)

    def __repr__(self) -> str:
        """String representation for debugging"""
        frame_bytes = self.to_bytes()
        hex_str = ' '.join(f'{b:02X}' for b in frame_bytes)
        return f"Frame(code={self.frame_code.name}, motor={self.motor_id}, payload_len={self.payload_length}): {hex_str}"


class FrameResponse:
    """
    Parser for frame responses from motor controller.

    Response format:
    - ACK: [0xF0] [Payload length] [Payload...] [Checksum]
    - NACK: [0xFF] [Error code] [Checksum]
    """

    def __init__(self, raw_bytes: bytes):
        """
        Parse response bytes.

        Args:
            raw_bytes: Raw response from motor controller
        """
        if len(raw_bytes) < 2:
            raise ValueError(f"Response too short: {len(raw_bytes)} bytes")

        self.raw = raw_bytes
        self.response_code = raw_bytes[0]

        if self.is_ack:
            # ACK format: [0xF0] [Length] [Payload...] [Checksum]
            if len(raw_bytes) < 3:
                raise ValueError("ACK response too short")

            self.payload_length = raw_bytes[1]
            payload_end = 2 + self.payload_length

            if len(raw_bytes) < payload_end + 1:
                raise ValueError(f"ACK response incomplete: expected {payload_end + 1} bytes, got {len(raw_bytes)}")

            self.payload = raw_bytes[2:payload_end]
            self.checksum = raw_bytes[payload_end]
            self.error_code = ErrorCode.NONE

        elif self.is_nack:
            # NACK format: [0xFF] [Error code] [Checksum]
            if len(raw_bytes) < 3:
                raise ValueError("NACK response too short")

            self.error_code = ErrorCode(raw_bytes[1]) if raw_bytes[1] in ErrorCode._value2member_map_ else raw_bytes[1]
            self.checksum = raw_bytes[2]
            self.payload = b""
            self.payload_length = 0
        else:
            raise ValueError(f"Unknown response code: 0x{self.response_code:02X}")

    @property
    def is_ack(self) -> bool:
        """Check if response is ACK"""
        return self.response_code == ResponseCode.ACK

    @property
    def is_nack(self) -> bool:
        """Check if response is NACK"""
        return self.response_code == ResponseCode.NACK

    def validate_checksum(self) -> bool:
        """
        Verify response checksum.

        Returns:
            True if checksum is valid
        """
        # Compute expected checksum
        if self.is_ack:
            accumulator = self.response_code + self.payload_length
            for byte in self.payload:
                accumulator += byte
        else:  # NACK
            accumulator = self.response_code + int(self.error_code)

        # Add low and high bytes
        expected = (accumulator & 0xFF) + ((accumulator >> 8) & 0xFF)
        expected = expected & 0xFF

        return self.checksum == expected

    def get_register_value(self, size: int, signed: bool = False) -> int:
        """
        Extract register value from payload.

        Args:
            size: Register size in bytes
            signed: Whether value is signed

        Returns:
            Decoded register value
        """
        if not self.is_ack:
            raise ValueError("Cannot get register value from NACK response")

        if len(self.payload) < size:
            raise ValueError(f"Payload too short for {size}-byte register: {len(self.payload)} bytes")

        # Convert from little-endian
        value = int.from_bytes(self.payload[:size], byteorder='little', signed=signed)
        return value

    def __repr__(self) -> str:
        """String representation for debugging"""
        hex_str = ' '.join(f'{b:02X}' for b in self.raw)
        if self.is_ack:
            return f"FrameResponse(ACK, payload_len={self.payload_length}): {hex_str}"
        else:
            return f"FrameResponse(NACK, error={self.error_code.name}): {hex_str}"


class ASPEPWrapper:
    """
    ASPEP (Asymmetric Serial Packet Exchange Protocol) wrapper.

    Wraps Frame Communication Protocol packets in ASPEP transport layer.
    """

    # ASPEP header for synchronous channel (from working minimal_motor_control.py)
    # Format: [Channel/Type] [ID] [Sequence] [Checksum]
    SYNC_HEADER = bytes([0x49, 0x01, 0x00, 0x70])

    @classmethod
    def wrap(cls, frame: Frame) -> bytes:
        """
        Wrap frame in ASPEP packet.

        Args:
            frame: Frame to wrap

        Returns:
            ASPEP packet: [ASPEP Header: 4 bytes] [Frame payload]
        """
        frame_bytes = frame.to_bytes()
        return cls.SYNC_HEADER + frame_bytes

    @classmethod
    def unwrap(cls, aspep_packet: bytes) -> bytes:
        """
        Extract frame from ASPEP packet.

        Args:
            aspep_packet: Complete ASPEP packet

        Returns:
            Frame bytes without ASPEP header
        """
        if len(aspep_packet) < 4:
            raise ValueError("ASPEP packet too short")

        # Skip 4-byte ASPEP header
        return aspep_packet[4:]


def create_read_register_frame(register_id: int, motor_id: int = 0) -> Frame:
    """
    Helper function to create a read register frame.

    Args:
        register_id: Register to read
        motor_id: Target motor ID

    Returns:
        Configured Frame object
    """
    frame = Frame(FrameCode.GET_REG, motor_id)
    frame.add_byte(register_id)
    return frame


def create_write_register_frame(register_id: int, value: int, size: int,
                                 signed: bool = False, motor_id: int = 0) -> Frame:
    """
    Helper function to create a write register frame.

    Args:
        register_id: Register to write
        value: Value to write
        size: Register size in bytes
        signed: Whether value is signed
        motor_id: Target motor ID

    Returns:
        Configured Frame object
    """
    frame = Frame(FrameCode.SET_REG, motor_id)
    frame.add_byte(register_id)
    frame.add_register_value(value, size, signed)
    return frame


def create_execute_command_frame(command_id: int, motor_id: int = 0) -> Frame:
    """
    Helper function to create an execute command frame.

    Args:
        command_id: Command to execute
        motor_id: Target motor ID

    Returns:
        Configured Frame object
    """
    frame = Frame(FrameCode.EXECUTE_CMD, motor_id)
    frame.add_byte(command_id)
    return frame
