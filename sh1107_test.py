import machine
import sh1107
import seesaw
import time

# 1. Hardware Setup
i2c = machine.I2C(1, sda=machine.Pin(18), scl=machine.Pin(19), freq=400000)
display = sh1107.SH1107_I2C(128, 128, i2c, addr=0x3D)
encoder = seesaw.Seesaw(i2c, addr=0x36)

# 2. SR04 Setup
trigger = machine.Pin(20, machine.Pin.OUT)
echo = machine.Pin(21, machine.Pin.IN)

def get_distance():
    trigger.low()
    time.sleep_us(2)
    trigger.high()
    time.sleep_us(10)
    trigger.low()
    duration = machine.time_pulse_us(echo, 1, 30000)
    return (duration * 0.0343) / 2 if duration > 0 else 0

# Variables
last_pos = encoder.get_position()
threshold = 20
locked = False  # Toggle for the pushbutton
last_button_state = False

print("XRP Ready.")

while True:
    dist = get_distance()
    curr_pos = encoder.get_position()
    button_now = encoder.get_button()
    
    # 3. Handle Pushbutton (Toggle Lock)
    # Detects the transition from "not pressed" to "pressed" (Rising edge)
    if button_now and not last_button_state:
        locked = not locked
        time.sleep(0.2) # Simple debounce
    last_button_state = button_now

    # 4. Handle Encoder (Only if not locked)
    if not locked:
        if curr_pos != last_pos:
            diff = curr_pos - last_pos
            threshold = max(2, min(100, threshold + diff))
            last_pos = curr_pos
    else:
        # Keep internal track of position so it doesn't "jump" when unlocked
        last_pos = curr_pos

    # 5. Dashboard Drawing (Cleaner Layout)
    display.fill(0)
    
    # Top Bar
    display.fill_rect(0, 0, 128, 16, 1)
    display.text("XRP RANGE HUB", 15, 4, 0)
    
    # Distance Section
    display.text("CURRENT DIST", 15, 30, 1)
    display.text("{:>6.1f} cm".format(dist), 30, 42, 1)
    
    # Limit Section
    display.hline(10, 65, 108, 1)
    display.text("LIMIT:", 10, 75, 1)
    
    # Show "LOCKED" or the value
    if locked:
        display.text("[LOCKED]", 65, 75, 1)
    else:
        display.text("{} cm".format(threshold), 65, 75, 1)
    
    # Warning Logic
    if 0 < dist < threshold:
        # Invert the bottom block
        display.fill_rect(0, 95, 128, 33, 1)
        display.text("!! OBJECT !!", 15, 105, 0)
    else:
        display.rect(5, 95, 118, 28, 1)
        display.text("PATH CLEAR", 22, 105, 1)
    
    display.show()
    time.sleep(0.05)