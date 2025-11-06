#!/usr/bin/env python3
"""
Speed Control Example

Simple example demonstrating motor speed control using the registry service API.
This replaces the hardcoded hex commands from minimal_motor_control.py with
proper parameter access through the MotorRegistry.

Hardware Requirements:
- STM32-based motor controller with MCSDK 6.x firmware
- BLDC motor with encoder or hall sensors
- 24V DC power supply
- USB serial connection

Usage:
    python examples/speed_control_example.py

Safety Note:
- Ensure motor is properly mounted and cannot cause injury
- Start with low speeds (< 500 RPM) for initial testing
- Keep emergency stop within reach
"""

import sys
import time
import signal
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from st_mcp.motor_controller import MotorController


def signal_handler(signum, frame):
    """Handle Ctrl+C for clean exit"""
    print("\n\nInterrupted by user. Exiting...")
    sys.exit(0)


def main():
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Configuration
    PORT = "/dev/ttyACM0"  # Change to your serial port (COM3 on Windows)
    BAUDRATE = 1843200     # ASPEP baudrate
    TARGET_RPM = 500       # Start with low speed for safety
    RUN_TIME_SEC = 10      # How long to run motor

    print("=" * 60)
    print("STM32 Motor Control - Speed Control Example")
    print("=" * 60)
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Target Speed: {TARGET_RPM} RPM")
    print("=" * 60)

    try:
        # Connect to motor controller
        with MotorController(port=PORT, baudrate=BAUDRATE) as motor:
            print("\n✓ Connected to motor controller")

            # Check for faults
            print("\nChecking for faults...")
            if motor.has_faults():
                fault_flags = motor.registry.read_fault_flags()
                print(f"⚠ Faults detected: 0x{fault_flags:08X}")
                print("  Acknowledging faults...")
                motor.fault_ack()
                time.sleep(0.2)

                # Check again
                if motor.has_faults():
                    print("✗ Faults persist after acknowledgment. Check motor setup.")
                    return 1
                else:
                    print("✓ Faults cleared")
            else:
                print("✓ No faults detected")

            # Read current bus voltage
            print("\nReading system status...")
            voltage = motor.registry.read_bus_voltage()
            temp = motor.registry.read_temperature()
            print(f"  Bus Voltage: {voltage:.1f} V")
            print(f"  Temperature: {temp:.1f} °C")

            # Read current PID gains
            print("\nReading speed PID gains...")
            kp, ki, kd = motor.registry.read_speed_pid()
            print(f"  Kp = {kp}")
            print(f"  Ki = {ki}")
            print(f"  Kd = {kd}")

            # Configure speed mode and start
            print(f"\n🚀 Starting motor in speed mode: {TARGET_RPM} RPM")
            motor.set_speed_mode(target_rpm=TARGET_RPM, ramp_duration_ms=1000)

            print(f"\n⏱  Running for {RUN_TIME_SEC} seconds...")
            print("=" * 60)
            print("Time  | Speed | Torque | Voltage | Temp   | Power")
            print("-" * 60)

            start_time = time.time()
            while time.time() - start_time < RUN_TIME_SEC:
                # Read telemetry
                telemetry = motor.get_telemetry()

                # Display
                elapsed = time.time() - start_time
                print(f"{elapsed:4.1f}s | "
                      f"{telemetry.speed_rpm:5d} | "
                      f"{telemetry.torque_amps:6.2f} | "
                      f"{telemetry.bus_voltage_v:7.1f} | "
                      f"{telemetry.temperature_c:6.1f} | "
                      f"{telemetry.motor_power_w:6.1f}")

                # Check for faults
                if telemetry.has_faults:
                    print(f"\n⚠ Faults detected during operation: 0x{telemetry.fault_flags:08X}")
                    break

                time.sleep(0.5)

            # Stop motor with ramp-down
            print("\n🛑 Stopping motor (with ramp)...")
            motor.stop(ramp=True)

            # Wait for motor to stop
            time.sleep(1)

            # Verify stopped
            speed = motor.get_speed()
            print(f"✓ Motor stopped. Final speed: {speed} RPM")

            print("\n" + "=" * 60)
            print("✓ Speed control test completed successfully!")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 1

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
