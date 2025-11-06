"""
Motor control commands for the Motor Control Protocol (MCP).
These commands handle direct motor operations like start, stop, and control modes.
"""

from enum import IntEnum
from typing import Optional
import numpy as np
from .base import MotorCommand

class MotorCommandType(IntEnum):
    """Motor command identifiers."""
    START = 0x0003
    STOP = 0x0004
    STOP_RAMP = 0x0005
    START_STOP = 0x0006

class StartMotorCommand(MotorCommand):
    """Command to start motor operation."""
    
    def __init__(self, motor_id: int = 0):
        """Initialize start motor command.
        
        Args:
            motor_id: Target motor ID (default 0)
        """
        super().__init__(command_id=MotorCommandType.START, motor_id=motor_id)
    
    def parse_response(self, response: bytes) -> bool:
        """Parse start motor response.
        
        Args:
            response: Raw response bytes
            
        Returns:
            bool: True if motor started successfully
        """
        return len(response) >= 1 and response[0] != 0x0F

class StopMotorCommand(MotorCommand):
    """Command to stop motor operation."""
    
    def __init__(self, motor_id: int = 0):
        """Initialize stop motor command.
        
        Args:
            motor_id: Target motor ID (default 0)
        """
        super().__init__(command_id=MotorCommandType.STOP, motor_id=motor_id)
    
    def parse_response(self, response: bytes) -> bool:
        """Parse stop motor response.
        
        Args:
            response: Raw response bytes
            
        Returns:
            bool: True if motor stopped successfully
        """
        return len(response) >= 1 and response[0] != 0x0F

class StopRampCommand(MotorCommand):
    """Command to stop motor with ramp-down."""
    
    def __init__(self, motor_id: int = 0):
        """Initialize stop ramp command.
        
        Args:
            motor_id: Target motor ID (default 0)
        """
        super().__init__(command_id=MotorCommandType.STOP_RAMP, motor_id=motor_id)
    
    def parse_response(self, response: bytes) -> bool:
        """Parse stop ramp response.
        
        Args:
            response: Raw response bytes
            
        Returns:
            bool: True if ramp-down started successfully
        """
        return len(response) >= 1 and response[0] != 0x0F

class StartStopCommand(MotorCommand):
    """Command to toggle motor start/stop state."""
    
    def __init__(self, motor_id: int = 0):
        """Initialize start/stop toggle command.
        
        Args:
            motor_id: Target motor ID (default 0)
        """
        super().__init__(command_id=MotorCommandType.START_STOP, motor_id=motor_id)
    
    def parse_response(self, response: bytes) -> bool:
        """Parse start/stop toggle response.
        
        Args:
            response: Raw response bytes
            
        Returns:
            bool: True if state toggled successfully
        """
        return len(response) >= 1 and response[0] != 0x0F

class MotorControlMode(IntEnum):
    """Motor control modes."""
    TORQUE = 0
    SPEED = 1

class SetControlModeCommand(MotorCommand):
    """Command to set motor control mode."""
    
    def __init__(self, mode: MotorControlMode, motor_id: int = 0):
        """Initialize control mode command.
        
        Args:
            mode: Desired control mode
            motor_id: Target motor ID (default 0)
        """
        super().__init__(command_id=0x0002, motor_id=motor_id)
        self.mode = mode
    
    def to_bytes(self) -> bytes:
        """Generate control mode command bytes."""
        header = np.array([0x49, 0x01, 0x00, 0x70], dtype=np.uint8)
        payload = np.array([
            self.motor_id,
            0x00,
            self.command_id & 0xFF,
            (self.command_id >> 8) & 0xFF,
            self.mode & 0xFF
        ], dtype=np.uint8)
        return bytes(np.concatenate([header, payload]))
    
    def parse_response(self, response: bytes) -> bool:
        """Parse control mode response.
        
        Args:
            response: Raw response bytes
            
        Returns:
            bool: True if mode set successfully
        """
        return len(response) >= 1 and response[0] != 0x0F
