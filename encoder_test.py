from XRPLib.defaults import *
import machine
import sh1107
import time
import math

# 1. Hardware Configuration
sda_pin = machine.Pin(4)
scl_pin = machine.Pin(5)
i2c = machine.I2C(0, sda=sda_pin, scl=scl_pin, freq=400000)
WIDTH = 128
HEIGHT = 128
display = sh1107.SH1107_I2C(WIDTH, HEIGHT, i2c, addr=0x3D)

def test_draw():
    # Clear display
    display.fill(0)
    
    # Draw a border frame
    display.rect(0, 0, WIDTH, HEIGHT, 1)
    display.show()
    time.sleep(0.5)

    # Draw Text
    display.text("SH1107 TEST", 20, 10, 1)
    display.text("MicroPython", 20, 25, 1)
    display.hline(20, 35, 88, 1)
    display.show()
    time.sleep(1)

    # Simple Animation: Moving Circle
    for i in range(0, WIDTH, 8):
        display.fill(0)
        display.rect(0, 0, WIDTH, HEIGHT, 1) # Keep border
        # Draw a moving "pulse"
        y = int(32 + 15 * math.sin(i / 10))
        display.fill_rect(i, y, 10, 10, 1)
        display.text("Scanning...", 30, 50, 1)
        display.show()
        time.sleep(0.05)

    # Final Inversion Test
    display.fill(1)
    display.text("INVERTED", 30, 28, 0)
    display.show()
    time.sleep(2)
    
    display.fill(0)
    display.text("TEST COMPLETE", 10, 28, 1)
    display.show()

# Run the test
try:
    print("Starting SH1107 Test...")
    test_draw()
except Exception as e:
    print("Error:", e)
