#!/usr/bin/env python3
"""
Omnimeter Serial Terminal
Interactive serial terminal for calibration and monitoring
"""

import serial
import sys
import threading
import time

PORT = '/dev/cu.usbmodem01'
BAUD = 115200

def print_help():
    print("""
╔════════════════════════════════════════════════════════════════════╗
║               OMNIMETER SERIAL TERMINAL                            ║
╠════════════════════════════════════════════════════════════════════╣
║ Calibration Commands (type and press Enter):                       ║
║   CAL:HELP          - Show calibration help on device              ║
║   CAL:SHOW          - Display current calibration data             ║
║   CAL:ZERO          - Zero calibration (short inputs to GND first) ║
║   CAL:REF:<voltage> - Reference calibration (e.g. CAL:REF:5.000)   ║
║   CAL:RANGE:<n>:<v> - Calibrate specific range (0-4)               ║
║   CAL:RESET         - Reset to factory defaults                    ║
║   CAL:SAVE          - Save calibration to flash                    ║
╠════════════════════════════════════════════════════════════════════╣
║ Terminal Commands:                                                  ║
║   help              - Show this help                               ║
║   clear             - Clear screen                                 ║
║   quit / exit       - Exit terminal                                ║
╚════════════════════════════════════════════════════════════════════╝
""")

def reader_thread(ser, running):
    """Background thread to read serial data"""
    while running[0]:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print(data.decode('utf-8', errors='ignore'), end='', flush=True)
            time.sleep(0.01)
        except Exception as e:
            if running[0]:
                print(f"\n[Error reading: {e}]")
            break

def main():
    print(f"Connecting to {PORT}...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Error: Could not open {PORT}: {e}")
        print("Make sure no other program is using the serial port.")
        sys.exit(1)
    
    print(f"Connected! Press Ctrl+C or type 'quit' to exit.\n")
    print_help()
    
    running = [True]
    reader = threading.Thread(target=reader_thread, args=(ser, running))
    reader.daemon = True
    reader.start()
    
    try:
        while True:
            try:
                line = input()
                
                # Local commands
                if line.lower() in ('quit', 'exit'):
                    break
                elif line.lower() == 'help':
                    print_help()
                    continue
                elif line.lower() == 'clear':
                    print('\033[2J\033[H', end='')
                    continue
                
                # Send to device
                ser.write((line + '\r\n').encode('utf-8'))
                time.sleep(0.1)
                
            except EOFError:
                break
                
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    
    running[0] = False
    time.sleep(0.1)
    ser.close()
    print("Disconnected.")

if __name__ == '__main__':
    main()
