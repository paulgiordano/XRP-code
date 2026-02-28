from XRPLib.defaults import *
from machine import UART, Pin
import time

# Quick serial terminal for HLK-LD2450 debugging
# Try different pin pairs below (uncomment one at a time)
# Baudrate 256000 is default for HLK-LD2450

# Option 1: UART1 on GPIO18 (TX) / GPIO19 (RX)  free on XRP
uart = UART(0, baudrate=256000, tx=Pin(18), rx=Pin(19))

# Option 2: UART1 on GPIO12 (TX) / GPIO13 (RX)  also free
# uart = UART(1, baudrate=256000, tx=Pin(12), rx=Pin(13))

# Option 3: UART1 on GPIO16 (TX) / GPIO17 (RX)  if not conflicting
# uart = UART(1, baudrate=256000, tx=Pin(16), rx=Pin(17))

# Option 4: UART1 on GPIO22 (TX) / GPIO23 (RX)  another free pair
# uart = UART(1, baudrate=256000, tx=Pin(22), rx=Pin(23))

print("Serial terminal starting... (CTRL+C to stop)")
print("Expecting data from HLK-LD2450  move in front of it to trigger reports.")

while True:
    if uart.any():
        data = uart.read()
        # Print as hex for easy debugging (HLK sends binary frames)
        print("Received:", ' '.join(f'{b:02X}' for b in data))
    time.sleep(0.05)  # 20 Hz check rate