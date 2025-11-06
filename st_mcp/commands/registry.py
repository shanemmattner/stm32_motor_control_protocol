"""
Registry Service Commands

This module implements Motor Control Protocol registry commands for
reading and writing motor parameters.

Registry commands provide low-level access to all motor control registers
using the Frame Communication Protocol over ASPEP transport.
"""

from typing import Union
from ..registers import Register, RegisterMap
from ..frame import Frame, FrameResponse, FrameCode, create_read_register_frame, create_write_register_frame
from ..exceptions import RegistryError, RegistryNACKError


class ReadRegisterCommand:
    """
    Command to read a motor control register.

    Sends GET_REG frame and parses response to extract register value.
    """

    def __init__(self, register: Register, motor_id: int = 0):
        """
        Initialize read register command.

        Args:
            register: Register definition to read
            motor_id: Target motor ID (0-7)
        """
        self.register = register
        self.motor_id = motor_id

    def to_bytes(self) -> bytes:
        """
        Generate command bytes.

        Returns:
            Complete frame: [Header] [Length] [Register ID] [Checksum]
        """
        frame = create_read_register_frame(self.register.id, self.motor_id)
        return frame.to_bytes()

    def parse_response(self, response: bytes) -> Union[int, float]:
        """
        Parse response and extract register value.

        Args:
            response: Raw response bytes from motor controller

        Returns:
            Register value in engineering units

        Raises:
            RegistryNACKError: If controller returned NACK
            RegistryError: If response is invalid
        """
        try:
            frame_resp = FrameResponse(response)
        except ValueError as e:
            raise RegistryError(f"Invalid response format: {e}")

        if frame_resp.is_nack:
            raise RegistryNACKError(
                f"Read register 0x{self.register.id:02X} ({self.register.name}) failed: {frame_resp.error_code.name}",
                error_code=frame_resp.error_code
            )

        # Validate checksum
        if not frame_resp.validate_checksum():
            raise RegistryError(f"Checksum validation failed for register {self.register.name}")

        # Extract raw value
        try:
            raw_value = frame_resp.get_register_value(self.register.size, self.register.is_signed)
        except ValueError as e:
            raise RegistryError(f"Failed to extract register value: {e}")

        # Decode to engineering units
        value = self.register.decode_value(raw_value)
        return value

    def __repr__(self) -> str:
        return f"ReadRegisterCommand(reg=0x{self.register.id:02X} {self.register.name}, motor={self.motor_id})"


class WriteRegisterCommand:
    """
    Command to write a motor control register.

    Sends SET_REG frame with register value.
    """

    def __init__(self, register: Register, value: Union[int, float], motor_id: int = 0):
        """
        Initialize write register command.

        Args:
            register: Register definition to write
            value: Value to write (in engineering units)
            motor_id: Target motor ID (0-7)

        Raises:
            ValueError: If register is read-only or value out of range
        """
        if register.read_only:
            raise ValueError(f"Cannot write read-only register: {register.name}")

        self.register = register
        self.value = value
        self.motor_id = motor_id

        # Encode and validate value
        try:
            self.raw_value = self.register.encode_value(value)
        except ValueError as e:
            raise ValueError(f"Invalid value for {register.name}: {e}")

    def to_bytes(self) -> bytes:
        """
        Generate command bytes.

        Returns:
            Complete frame: [Header] [Length] [Register ID] [Value bytes] [Checksum]
        """
        frame = create_write_register_frame(
            self.register.id,
            self.raw_value,
            self.register.size,
            self.register.is_signed,
            self.motor_id
        )
        return frame.to_bytes()

    def parse_response(self, response: bytes) -> bool:
        """
        Parse response and verify write succeeded.

        Args:
            response: Raw response bytes from motor controller

        Returns:
            True if write succeeded

        Raises:
            RegistryNACKError: If controller returned NACK
            RegistryError: If response is invalid
        """
        try:
            frame_resp = FrameResponse(response)
        except ValueError as e:
            raise RegistryError(f"Invalid response format: {e}")

        if frame_resp.is_nack:
            raise RegistryNACKError(
                f"Write register 0x{self.register.id:02X} ({self.register.name}) = {self.value} failed: {frame_resp.error_code.name}",
                error_code=frame_resp.error_code
            )

        # Validate checksum
        if not frame_resp.validate_checksum():
            raise RegistryError(f"Checksum validation failed for register {self.register.name}")

        return True

    def __repr__(self) -> str:
        return f"WriteRegisterCommand(reg=0x{self.register.id:02X} {self.register.name}, value={self.value}, motor={self.motor_id})"


# Convenience functions for common operations

def read_register_by_id(register_id: int, motor_id: int = 0) -> ReadRegisterCommand:
    """
    Create read command for register by ID.

    Args:
        register_id: Register address
        motor_id: Target motor ID

    Returns:
        ReadRegisterCommand

    Raises:
        ValueError: If register ID not found
    """
    register = RegisterMap.get_by_id(register_id)
    if register is None:
        raise ValueError(f"Unknown register ID: 0x{register_id:02X}")

    return ReadRegisterCommand(register, motor_id)


def read_register_by_name(register_name: str, motor_id: int = 0) -> ReadRegisterCommand:
    """
    Create read command for register by name.

    Args:
        register_name: Register name
        motor_id: Target motor ID

    Returns:
        ReadRegisterCommand

    Raises:
        ValueError: If register name not found
    """
    register = RegisterMap.get_by_name(register_name)
    if register is None:
        raise ValueError(f"Unknown register name: {register_name}")

    return ReadRegisterCommand(register, motor_id)


def write_register_by_id(register_id: int, value: Union[int, float], motor_id: int = 0) -> WriteRegisterCommand:
    """
    Create write command for register by ID.

    Args:
        register_id: Register address
        value: Value to write
        motor_id: Target motor ID

    Returns:
        WriteRegisterCommand

    Raises:
        ValueError: If register ID not found or value invalid
    """
    register = RegisterMap.get_by_id(register_id)
    if register is None:
        raise ValueError(f"Unknown register ID: 0x{register_id:02X}")

    return WriteRegisterCommand(register, value, motor_id)


def write_register_by_name(register_name: str, value: Union[int, float], motor_id: int = 0) -> WriteRegisterCommand:
    """
    Create write command for register by name.

    Args:
        register_name: Register name
        value: Value to write
        motor_id: Target motor ID

    Returns:
        WriteRegisterCommand

    Raises:
        ValueError: If register name not found or value invalid
    """
    register = RegisterMap.get_by_name(register_name)
    if register is None:
        raise ValueError(f"Unknown register name: {register_name}")

    return WriteRegisterCommand(register, value, motor_id)
