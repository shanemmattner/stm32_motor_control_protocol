"""
Motor Control Protocol Exceptions

Custom exceptions for motor control operations.
"""

from typing import Optional
from enum import IntEnum


class MotorControlError(Exception):
    """Base exception for motor control operations"""
    pass


class RegistryError(MotorControlError):
    """Registry service error"""
    pass


class RegistryNACKError(RegistryError):
    """Registry command received NACK response"""

    def __init__(self, message: str, error_code: Optional[IntEnum] = None):
        super().__init__(message)
        self.error_code = error_code


class TransportError(MotorControlError):
    """Transport layer error"""
    pass


class ConnectionError(TransportError):
    """Connection establishment failed"""
    pass


class TimeoutError(TransportError):
    """Communication timeout"""
    pass


class MotorFaultError(MotorControlError):
    """Motor fault detected"""

    def __init__(self, message: str, fault_flags: int = 0):
        super().__init__(message)
        self.fault_flags = fault_flags
