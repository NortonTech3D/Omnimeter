#!/bin/bash
# Omnimeter Serial Monitor Script
# Usage: ./monitor.sh [port]

PORT="${1:-/dev/cu.usbmodem01}"

echo "═══════════════════════════════════════════════════"
echo "  OMNIMETER Serial Monitor"
echo "  Port: $PORT"
echo "  Press Ctrl+C to exit"
echo "═══════════════════════════════════════════════════"
echo ""

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "ERROR: Port $PORT not found!"
    echo ""
    echo "Available USB serial ports:"
    ls /dev/cu.usb* 2>/dev/null || echo "  (none found)"
    echo ""
    echo "Make sure the ESP32-S2 is connected."
    exit 1
fi

# Kill any existing monitors
pkill -f "pio device monitor" 2>/dev/null
pkill -f "cat $PORT" 2>/dev/null

# Configure port (optional - may help with reliability)
stty -f "$PORT" 115200 cs8 -cstopb -parenb 2>/dev/null

# Start monitoring
exec cat "$PORT"
