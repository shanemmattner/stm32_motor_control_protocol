#!/usr/bin/env python3
"""
Minimal motor control script for STM32 MCPS.
This script implements the minimal sequence needed to start a motor in velocity mode.
"""

import serial
import time
import signal
import sys
from binascii import unhexlify

class MinimalMotorControl:
    def __init__(self, port="/dev/ttyACM0", baudrate=1843200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = True
        
        # Setup signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C for clean exit"""
        print("\nExiting...")
        self.running = False
        if self.serial:
            self.disconnect()
            self.serial.close()
        sys.exit(0)

    def connect(self):
        """Open serial connection and establish communication"""
        try:
            # Open serial port
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            # Clear any pending data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False

    def send_command(self, command, expect_response=True, response_length=4):
        """Send a command and optionally wait for response"""
        try:
            # Convert hex string to bytes, removing spaces
            cmd_bytes = unhexlify(command.replace(" ", ""))
            self.serial.write(cmd_bytes)
            print(f"Sent: {command}")
            
            if expect_response:
                # Wait a bit before reading response
                time.sleep(0.1)
                response = self.serial.read(response_length)
                if response:
                    response_hex = " ".join([f"{b:02X}" for b in response])
                    print(f"Received: {response_hex}")
                    return response
                else:
                    print("No response received")
                    return None
            return None
        except Exception as e:
            print(f"Error sending command: {e}")
            return None

    def establish_connection(self):
        """Perform the connection sequence"""
        print("\nEstablishing connection...")
        
        # Initial beacon exchange
        self.send_command("55 FF FF 77")
        response = self.serial.read(4)  # Wait for beacon response
        if response:
            response_hex = " ".join([f"{b:02X}" for b in response])
            print(f"Beacon response: {response_hex}")
            
            # Echo the response back
            self.send_command(response_hex)
            
            # Wait for confirmation
            confirm = self.serial.read(4)
            if confirm:
                confirm_hex = " ".join([f"{b:02X}" for b in confirm])
                print(f"Confirmation: {confirm_hex}")
                
                # Send connection request
                self.send_command("06 00 00 60")
                return True
        
        print("Connection sequence failed")
        return False

    def configure_velocity_mode(self):
        """Configure the motor for velocity mode"""
        print("\nConfiguring velocity mode...")
        # Send configuration with exact timing from successful test
        time.sleep(1.6)  # Match timing from successful test
        self.send_command("D9 00 00 40 08 00 29 05 07 00 00 08 00 00 FF 00", expect_response=False)
        time.sleep(1.6)  # Wait between commands as in successful test

    def set_velocity(self):
        """Send velocity control command"""
        print("\nSetting velocity...")
        # Send velocity command with timing from successful test
        self.send_command("E9 02 00 A0 11 00 19 00 59 00 59 1B 99 00 91 02", expect_response=False)
        # Send multiple times like in successful test
        for _ in range(5):
            time.sleep(1.6)  # Match timing from successful test
            self.send_command("E9 02 00 A0 11 00 19 00 59 00 59 1B 99 00 91 02", expect_response=False)

    def disconnect(self):
        """Send disconnect command"""
        print("\nDisconnecting...")
        self.send_command("05 00 00 50", expect_response=False)

    def run(self):
        """Main execution sequence"""
        if not self.connect():
            return
        
        try:
            if self.establish_connection():
                print("Connection established")
                self.configure_velocity_mode()
                self.set_velocity()
                
                print("\nMotor should now be running")
                print("Press Ctrl+C to stop and exit")
                
                # Keep the script running until Ctrl+C
                while self.running:
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"Error during execution: {e}")
        finally:
            if self.serial:
                self.disconnect()
                self.serial.close()

def main():
    controller = MinimalMotorControl()
    controller.run()

if __name__ == "__main__":
    main()
