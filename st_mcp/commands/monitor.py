"""
Monitoring commands for the Motor Control Protocol (MCP).
These commands handle real-time data acquisition from the motor controller.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import struct
import numpy as np
from .base import MonitorCommand

@dataclass
class Monitor1Data:
    """First set of monitoring data."""
    speed: float
    torque: float
    flux: float
    vbus: float
    temperature: float
    
@dataclass
class Monitor2Data:
    """Second set of monitoring data."""
    i_a: float
    i_b: float
    i_alpha: float
    i_beta: float
    i_q: float
    i_d: float

class GetMonitor1Command(MonitorCommand):
    """Command to get first set of monitoring data."""
    
    def __init__(self):
        # Command ID for Monitor1 data
        super().__init__(monitor_id=0x0019)
    
    def to_bytes(self) -> bytes:
        """Generate monitor1 command bytes."""
        # Special format for monitor1 command
        header = np.array([0xa9, 0x00, 0x00, 0x50], dtype=np.uint8)
        payload = np.array([
            0x11, 0x00,     # Command
            0x19, 0x00,     # Monitor ID
            0x91, 0x05,     # Data selector 1
            0x49, 0x00,     # Data selector 2
            0x09, 0x05      # Data selector 3
        ], dtype=np.uint8)
        return bytes(np.concatenate([header, payload]))
    
    def parse_response(self, response: bytes) -> Optional[Monitor1Data]:
        """Parse monitor1 response data.
        
        Args:
            response: Raw response bytes
            
        Returns:
            Monitor1Data: Parsed monitoring data or None if invalid
        """
        try:
            if len(response) < 24:  # Minimum expected length
                return None
                
            data = response[4:]  # Skip header
            
            # Parse float values (assuming 32-bit float format)
            values = struct.unpack('<5f', data[:20])
            
            return Monitor1Data(
                speed=values[0],
                torque=values[1],
                flux=values[2],
                vbus=values[3],
                temperature=values[4]
            )
        except Exception as e:
            return None

class GetMonitor2Command(MonitorCommand):
    """Command to get second set of monitoring data."""
    
    def __init__(self):
        # Command ID for Monitor2 data
        super().__init__(monitor_id=0x0051)
    
    def to_bytes(self) -> bytes:
        """Generate monitor2 command bytes."""
        # Special format for monitor2 command
        header = np.array([0x89, 0x00, 0x00, 0x20], dtype=np.uint8)
        payload = np.array([
            0x11, 0x00,     # Command
            0x51, 0x1c,     # Current measurements
            0x11, 0x1c,     # Additional measurements
            0x19, 0x1d      # Final measurements
        ], dtype=np.uint8)
        return bytes(np.concatenate([header, payload]))
    
    def parse_response(self, response: bytes) -> Optional[Monitor2Data]:
        """Parse monitor2 response data.
        
        Args:
            response: Raw response bytes
            
        Returns:
            Monitor2Data: Parsed monitoring data or None if invalid
        """
        try:
            if len(response) < 28:  # Minimum expected length
                return None
                
            data = response[4:]  # Skip header
            
            # Parse float values (assuming 32-bit float format)
            values = struct.unpack('<6f', data[:24])
            
            return Monitor2Data(
                i_a=values[0],
                i_b=values[1],
                i_alpha=values[2],
                i_beta=values[3],
                i_q=values[4],
                i_d=values[5]
            )
        except Exception as e:
            return None

class MonitoringSession:
    """Helper class to manage monitoring data acquisition."""
    
    def __init__(self):
        self.monitor1 = GetMonitor1Command()
        self.monitor2 = GetMonitor2Command()
        
    def format_monitor1_data(self, data: Monitor1Data) -> str:
        """Format monitor1 data for display.
        
        Args:
            data: Monitor1 data to format
            
        Returns:
            str: Formatted string representation
        """
        return (
            f"Speed: {data.speed:.2f} rpm\n"
            f"Torque: {data.torque:.2f} Nm\n"
            f"Flux: {data.flux:.2f} Wb\n"
            f"Bus Voltage: {data.vbus:.1f} V\n"
            f"Temperature: {data.temperature:.1f} °C"
        )
    
    def format_monitor2_data(self, data: Monitor2Data) -> str:
        """Format monitor2 data for display.
        
        Args:
            data: Monitor2 data to format
            
        Returns:
            str: Formatted string representation
        """
        return (
            f"Phase Currents:\n"
            f"  Ia: {data.i_a:.2f} A\n"
            f"  Ib: {data.i_b:.2f} A\n"
            f"Alpha-Beta Currents:\n"
            f"  Iα: {data.i_alpha:.2f} A\n"
            f"  Iβ: {data.i_beta:.2f} A\n"
            f"D-Q Currents:\n"
            f"  Iq: {data.i_q:.2f} A\n"
            f"  Id: {data.i_d:.2f} A"
        )
