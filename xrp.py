from XRPLib.defaults import *
import machine
import sh1107
import seesaw
import time
import math
import random
import utime
from neopixel import NeoPixel
from XRPRadar import XRPRadar
from machine import Pin, time_pulse_us
from RadarFollower import RadarFollower
from ObstacleAvoider import ObstacleAvoider

VERSION = "1.1.0"

i2c = machine.I2C(0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400000)
#i2c = I2C(1, scl=Pin(39), sda=Pin(38), freq=400000)
display = sh1107.SH1107_I2C(128, 128, i2c, addr=0x3D)
seesaw_device = seesaw.Seesaw(i2c, addr=0x36)
vin = machine.ADC(machine.Pin(46))
led = NeoPixel(Pin(37), 1)  # WS2812 RGB LED

current_threshold = 20
last_button_state = False

#radar = XRPRadar(uart_id=0, tx_pin=machine.Pin(12), rx_pin=machine.Pin(13))
hlk_radar = XRPRadar(uart_id=0, tx_pin=18, rx_pin=19, baudrate=256000)
follower = RadarFollower(drivetrain, hlk_radar, imu=imu)  # optional imu
avoider = ObstacleAvoider(drivetrain, hlk_radar)
radar_buffer = ""
log_messages = []

def get_real_volts():
    total = 0
    for _ in range(8):
        total += vin.read_u16()
        time.sleep(0.001)          # use time.sleep (you already imported time)
    
    measured_v = (total / 8) * 3.3 / 65535
    battery_v = measured_v * 4.0303
    return battery_v

def set_led_green():
    led[0] = (0, 255, 0)
    led.write()

def set_led_red():
    led[0] = (255, 0, 0)
    led.write()

def error_routine(error_msg, state_info=""):
    set_led_red()
    add_log(f"ERROR: {error_msg}")
    print(f"ERROR: {error_msg}")
    if state_info:
        add_log(state_info)
        print(state_info)
    # Show full log on display
    display.fill(0)
    y = 0
    for msg in log_messages[-15:]:  # show last 15 entries
        display.text(msg[:20], 5, y, 1)
        y += 9
    display.show()
    # Stop program
    raise SystemExit(f"Error: {error_msg}")
        

def get_radar_distance():
    report = hlk_radar.parse_radar_report()
    if report:
        min_dist = float('inf')
        for i in range(0, len(report), 4):
            x, y, speed, res = report[i:i+4]
            if res > 0 and (x != 0 or y != 0):  # valid target
                dist = math.sqrt(x**2 + y**2)   # Euclidean distance in cm
                if dist < min_dist:
                    min_dist = dist
        if min_dist != float('inf'):
            return min_dist
    return 65535  # no targets (match ultrasonic timeout)
    
def draw_distance_bar(dist, bar_y=75):
    """Visualizes distance on the OLED. bar_y = 75 when IMU is on, 90 when off."""
    # Frame for the bar
    display.rect(14, bar_y, 102, 10, 1)
    
    # Fill the bar (closer = fuller bar)
    d_val = int(max(0, min(100, dist / 6)))
    filled_width = 100 - d_val
    if filled_width > 0:
        display.fill_rect(15, bar_y + 1, filled_width, 8, 1)
    
    # Safety threshold line
    thresh_px = 100 - int(max(20, min(600, current_threshold)))
    display.vline(15 + thresh_px, bar_y - 2, 14, 1)

def safety_drive(left_eff, right_eff, target_distance):
    add_log("Drive command received...")
    
    # 1. Reset the position for the EncodedMotor objects
    # In this library version, the method is reset_relative_position
    try:
        drivetrain.left_motor.reset_relative_position()
        drivetrain.right_motor.reset_relative_position()
    except AttributeError:
        # Fallback for some specific XRP versions
        drivetrain.left_motor.reset_encoder_position()
        drivetrain.right_motor.reset_encoder_position()
    
    start_ms = time.ticks_ms()
    
    # 2. Start moving
    drivetrain.set_effort(left_eff, right_eff)
    
    while True:
        dist_moved = abs(drivetrain.left_motor.get_position())
        
        # 1. Target Reach Check
        if dist_moved >= abs(target_distance):
            add_log(f"Target reached: {dist_moved:.1f}")
            break
            
        # 2. Get distance
        d = get_radar_distance()

        # 3. THE SOFTWARE SHIELD: 
        # We know your 'noise' is around 5cm. 
        # By setting the floor to 10cm, we ignore all electrical ghosts.
        if 10.0 < d < current_threshold:
            utime.sleep_ms(10) 
            # Current distance readout
            d_check = get_radar_distance()

            # If it's still between 10 and threshold, it's a real wall.
            if 10.0 < d_check < current_threshold:
                add_log(f"REAL Obstacle at {d_check:.1f}cm. Stopping.")
                break
        
        # 4. Safety Timeout
        if time.ticks_diff(time.ticks_ms(), start_ms) > 5000:
            break
            
        time.sleep(0.01)
        
    # Stop all motors
    drivetrain.stop()

def get_radar_report():
    report = hlk_radar.parse_radar_report()   # use the new radar
    if report:
        for i in range(0, len(report), 4):
            x, y, speed, res = report[i:i+4]
            display.text(f"T{i//4+1}: X{x:3} Y{y:3} S{speed:2} R{res}", 5, 103 + (i//4)*9, 1)
        display.show()
        return True
    return False

def run_program(index):
    try: seesaw_device.set_led(128, 0, 0) # Red for Driving
    except: pass
   
    display.fill(0)
    display.text("DRIVING...", 35, 50, 1)
    display.show()
   
    # Program Menu Logic
    if index == 0: safety_drive(0.6, 0.6, 15.0)
    elif index == 1: safety_drive(0.4, 0.7, 12.0)
    elif index == 2: safety_drive(-0.5, 0.5, 5.0)
    elif index == 3: safety_drive(0, 0.6, 8.0)
    elif index == 4: safety_drive(0.5, 0.8, 40.0)
    elif index == 5: # Square
        for _ in range(4):
            safety_drive(0.6, 0.6, 10.0)
            safety_drive(-0.5, 0.5, 4.8)
    elif index == 6: # Circle-ish
        for _ in range(8):
            safety_drive(0.6, 0.6, 6.0)
            safety_drive(-0.5, 0.5, 2.4)
    elif index == 7: # Out and Back
        safety_drive(0.7, 0.7, 8.0)
        time.sleep(0.5)
        safety_drive(-0.7, -0.7, -8.0)
    elif index == 10:  # FOLLOW
        follower.run()
    elif index == 11:  # GO 5M
        avoider.run()
    elif index == 12:  # IMU - perhaps add something
        pass
    try: seesaw_device.set_led(0, 128, 0) # Green for Idle
    except: pass

def set_distance_mode():
    """Adjusts the safety threshold using the encoder."""
    global current_threshold
    try: seesaw_device.set_led(0, 0, 128) # Blue for Config
    except: pass
    
    base_val = current_threshold
    start_pos = seesaw_device.get_position()
    
    while True:
        change = (seesaw_device.get_position() - start_pos) // 2
        current_threshold = max(2, min(100, base_val + change))
        
        display.fill(0)
        display.text("SET SAFETY", 30, 20, 1)
        display.text(f"{current_threshold} cm", 45, 40, 1)
        
        # Real-time preview of distance while setting
        draw_distance_bar(get_radar_distance())
        display.show()
        
        if seesaw_device.get_button():
            while seesaw_device.get_button(): time.sleep(0.01)
            break
        time.sleep(0.05)
        
    try: seesaw_device.set_led(0, 128, 0)
    except: pass

def main():
    global last_button_state
    imu_mode = False
    radar_mode = False
    radar_multi = True
    log_mode = False  # new log display toggle
    log_messages = []
    add_log(f"XRP System v{VERSION} starting")
    try:
        imu.calibrate()
        add_log("IMU calibrated successfully")
    except Exception as e:
        error_routine("IMU calibration failed", f"Exception: {e}")
    try:
        hlk_radar.multi_target_tracking()
        add_log("Radar initialized to multi-target")
    except Exception as e:
        error_routine("Radar initialization failed", f"Exception: {e}, radar_multi={radar_multi}")
    time.sleep(0.5)
    try: seesaw_device.set_led(0, 128, 0)
    except: pass
    set_led_green()  # System running indicator

    menus = ["STRAIGHT", "ARC", "POINT", "SWING", "CIRCLE", "SQUARE", "POLYGON", "TEST", "SET LIMIT", "HLK-LD2450", "RADAR MODE", "FOLLOW", "GO 5M", "IMU", "LOG"]

    while True:
        pos = seesaw_device.get_position()
        count = (pos) % 15
        
        if log_mode:
            # Show full log on screen
            display.fill(0)
            y = 0
            for msg in log_messages[-15:]:  # show last 15 entries
                display.text(msg[:20], 5, y, 1)
                y += 9
        else:
            # Normal dashboard display
            display.fill(0)
            display.text("XRP DASHBOARD", 15, 4, 1)
            display.hline(0, 14, 128, 1)
            display.text(f"MODE: {menus[count]}", 5, 28, 1)
            display.text(f"BATT: {get_real_volts():.2f}V", 5, 44, 1)
            display.text(f"RADAR: {'Multi' if radar_multi else 'Single'}", 5, 54, 1)  # moved down
            
            # Current distance readout
            dist = get_radar_distance()
            print("Calculated dist:", dist)
            if dist >= 65500:
                display.text("DIST: NO SENSOR", 5, 64, 1)  # moved down
            else:
                display.text(f"DIST: {int(dist)}cm", 5, 64, 1)  # moved down
            
            # === Distance bar + IMU / Radar + LOG ===
            if imu_mode or radar_mode:
                bar_y = 75          # higher when active
                data_y = 85         # data start
                log_y = 115         # log below data (room for 2 lines)
            else:
                bar_y = 90          # normal
                data_y = 0          # no data
                log_y = 105         # log right below bar
            
            draw_distance_bar(dist, bar_y)
            
            if imu_mode:
                accel = [x / 1000.0 for x in imu.get_acc_rates()]
                gyro  = [x / 1000.0 for x in imu.get_gyro_rates()]
                heading = imu.get_heading()
                display.text(f"A: {accel[0]:.1f} {accel[1]:.1f} {accel[2]:.1f} g", 5, data_y, 1)
                display.text(f"G: {gyro[0]:.1f} {gyro[1]:.1f} {gyro[2]:.1f} d/s", 5, data_y + 9, 1)
                display.text(f"H: {heading:.1f} deg", 5, data_y + 18, 1)
            
            if radar_mode:
                report = hlk_radar.parse_radar_report()
                if report:
                    num_targets = 3 if radar_multi else 1
                    for i in range(0, num_targets * 4, 4):
                        x, y, speed, res = report[i:i+4]
                        display.text(f"T{(i//4)+1}: X{x:3.0f} Y{y:3.0f} S{speed:2.0f} R{res:2.0f}", 5, data_y + (i//4)*9, 1)
                else:
                    display.text("NO TARGETS", 5, data_y, 1)
            
            # === Rolling Log (always shown, below everything) ===
            for idx, msg in enumerate(log_messages):
                display.text(msg[:20], 5, log_y + idx * 9, 1)  # truncate to 20 chars to fit screen
        
        display.show()
        
        btn = seesaw_device.get_button()
        if btn and not last_button_state:
            time.sleep(0.1) # Debounce
            if count == 13:   # IMU - toggle display modes
                if not imu_mode and not radar_mode:
                    imu_mode = True
                elif imu_mode:
                    imu_mode = False
                    radar_mode = True
                elif radar_mode:
                    radar_mode = False
            elif count == 9:   # HLK-LD2450
                radar_mode = True
                imu_mode = False
            elif count == 10:  # RADAR MODE (new toggle)
                radar_multi = not radar_multi
                if radar_multi:
                    hlk_radar.multi_target_tracking()
                    display.fill(0)
                    display.text("Multi-Target ON", 20, 50, 1)
                else:
                    hlk_radar.single_target_tracking()
                    display.fill(0)
                    display.text("Single-Target ON", 20, 50, 1)
                display.show()
                time.sleep(1.0)   # quick feedback
            elif count == 14:  # LOG - toggle log display mode
                log_mode = not log_mode
                if log_mode:
                    imu_mode = False  # turn off other modes when viewing log
                    radar_mode = False
            else:
                imu_mode = False
                radar_mode = False
                log_mode = False
                if count == 8:
                    set_distance_mode()
                else:
                    run_program(count)
                    
        last_button_state = btn
        time.sleep(0.1)
        
        radar_buffer = hlk_radar.poll_for_response()

# Entry point
if __name__ == "__main__":
    main()