from XRPLib.defaults import *
import machine
import sh1107
import seesaw
import time
import math
import random
import utime
import qwiic_i2c
import qwiic_joystick
import collections
import gc
from neopixel import NeoPixel
from XRPRadar import XRPRadar
from machine import Pin, time_pulse_us
from RadarFollower import RadarFollower
from ObstacleAvoider import ObstacleAvoider

VERSION = "1.3.0"

i2c = machine.I2C(0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400000)
qwiic_driver = qwiic_i2c.get_i2c_driver(sda=4, scl=5, freq=400000)
joy = qwiic_joystick.QwiicJoystick(address=0x20, i2c_driver=qwiic_driver)
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
log_messages = collections.deque((),50)
log_scroll_index = 0

def get_real_volts():
    total = 0
    for _ in range(8):
        total += vin.read_u16()
        time.sleep(0.001)          # use time.sleep (you already imported time)

    measured_v = (total / 8) * 3.3 / 65535
    battery_v = measured_v * 4.0303
    return battery_v

def add_log(msg):
    # 1. Simple duplicate filter to prevent spamming memory
    if len(log_messages) > 0 and log_messages[-1] == msg[:15]:
        return

    width = 15
    for i in range(0, len(msg), width):
        # deque automatically discards the oldest item when it hits 50
        log_messages.append(msg[i:i+width])

def set_led_green():
    led[0] = (0, 64, 0)
    led.write()

def set_led_red():
    led[0] = (64, 0, 0)
    led.write()

def error_routine(error_msg, e=None):
    """
    Handles system errors by stopping motors, logging the error, 
    and displaying a stack trace if an exception object is provided.
    """
    import sys
    import io
    
    # 1. Safety First
    try: drivetrain.stop()
    except: pass
    set_led_red()
    
    # 2. Log the primary error message
    add_log(f"ERR: {error_msg}")
    
    # 3. Capture and Log Stack Trace if 'e' is provided
    if e is not None:
        buf = io.StringIO()
        sys.print_exception(e, buf)
        trace_text = buf.getvalue()
        # Wrap and add each line of the trace to the log
        for line in trace_text.split('\n'):
            if line.strip():
                add_log(line.strip())
            
    # 4. Print to console for REPL debugging
    print(f"ERROR: {error_msg}")
    if e: sys.print_exception(e)
    
    # 5. Lock the display on the log so the user can scroll/see it
    # Note: This will exit the program
    display.fill(0)
    # Show last 14 lines of the updated log
    for i, msg in enumerate(log_messages[-14:]): 
        display.text(msg, 0, i * 9, 1)
    display.show()
    raise SystemExit(f"Exiting: {error_msg}")

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
    display.rect(14, bar_y, 102, 10, 1)
    
    # Adjusted scaling: 100cm max for better resolution at close range
    # Change '1.0' to '2.0' if you want a 200cm max range
    d_val = int(max(0, min(100, dist / 1.0))) 
    
    filled_width = 100 - d_val
    if filled_width > 0:
        display.fill_rect(15, bar_y + 1, filled_width, 8, 1)

    # Threshold line
    thresh_px = 100 - int(max(0, min(100, current_threshold)))
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
        try:
            drivetrain.left_motor.reset_encoder_position()
            drivetrain.right_motor.reset_encoder_position()
        except Exception as e:
            error_routine("Motor encoder reset failed", f"Exception: {e}")
    except Exception as e:
        error_routine("Motor encoder reset failed", f"Exception: {e}")

    start_ms = time.ticks_ms()

    # 2. Start moving
    try:
        drivetrain.set_effort(left_eff, right_eff)
    except Exception as e:
        error_routine("Failed to set motor effort", f"Exception: {e}")

    while True:
        try:
            dist_moved = abs(drivetrain.left_motor.get_position())
        except Exception as e:
            error_routine("Failed to get motor position", f"Exception: {e}")

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
    try:
        drivetrain.stop()
    except Exception as e:
        error_routine("Failed to stop motors", f"Exception: {e}")

def get_radar_report():
    report = hlk_radar.parse_radar_report()   # use the new radar
    if report:
        for i in range(0, len(report), 4):
            x, y, speed, res = report[i:i+4]
            display.text(f"T{i//4+1}: X{x:3} Y{y:3} S{speed:2} R{res}", 5, 103 + (i//4)*9, 1)
        display.show()
        return True
    return False

def run_joystick_control():
    """Directly control the XRP drivetrain using the Qwiic Joystick."""
    if not qwiic_i2c.is_device_connected(0x20):
        add_log("Error: No Joystick")
        return # Exit the routine if hardware is missing
    add_log("Joystick Mode: Active")
    display.fill(0)
    display.text("JOYSTICK MODE", 15, 10, 1)
    display.text("Press Button", 20, 30, 1)
    display.text("to Exit", 35, 45, 1)
    display.show()

    try:
        while True:
            # Get joystick positions (0-1023)
            # 512 is center. We normalize to -1.0 to 1.0
            x_val = (joy.get_horizontal() - 512) / 512.0
            y_val = (joy.get_vertical() - 512) / 512.0
        
            # Simple Arcade Drive Logic
            # Y is forward/back, X is steering
            left_effort = y_val - x_val
            right_effort = y_val + x_val
        
            # Constrain efforts to [-1, 1]
            left_effort = max(-1.0, min(1.0, left_effort))
            right_effort = max(-1.0, min(1.0, right_effort))
        
            # Apply a deadzone to prevent 'creeping' at center
            if abs(left_effort) < 0.1: left_effort = 0
            if abs(right_effort) < 0.1: right_effort = 0
        
            drivetrain.set_effort(left_effort, right_effort)
            
            if joy.get_button() == 0:
                add_log("Joystick button pressed")
                break
    except Exception as e:
        raise e
    finally:
        drivetrain.stop()
        add_log("Joystick Mode: Inactive")

        # Exit if the joystick button is pressed
        if joy.get_button() == 0:
            drivetrain.stop()
            add_log("Joystick Mode: Exit")

        time.sleep(0.05)

def run_program(index):
    try:
        try: seesaw_device.set_led(64, 0, 0) # Red for Driving
        except Exception as e:
            add_log(f"LED error: {e}")

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
            try:
                follower.run()
            except Exception as e:
                error_routine("Follower run failed", f"Exception: {e}")
        elif index == 11:  # AVOID
            try:
                avoider.run()
            except Exception as e:
                error_routine("Avoider run failed", f"Exception: {e}")
        elif index == 12:  # JOYSTICK (The new index for JOYSTICK)
            run_joystick_control()
        elif index == 13:  # IMU - perhaps add something
            pass
        elif index == 14:  # LOG - perhaps add something
            pass
        
        try: seesaw_device.set_led(0, 64, 0) # Green for Idle
        except Exception as e:
            add_log(f"LED error: {e}")
    except Exception as e:
        error_routine(f"Prog {index} Fail", f"Exception: {e}")

def set_distance_mode():
    """Adjusts the safety threshold using the encoder."""
    global current_threshold
    try: seesaw_device.set_led(0, 0, 64) # Blue for Config
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

    try: seesaw_device.set_led(0, 64, 0)
    except: pass

def main():
    global last_button_state, log_scroll_index
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
    try:
        if qwiic_i2c.is_device_connected(0x20):
            add_log("Joystick: Connected")
        else:
            add_log("Joystick: NOT FOUND")
    except Exception as e:
        error_routine(f"Joystick Check Fail: {e}")
    time.sleep(0.5)
    try: seesaw_device.set_led(0, 64, 0)
    except: pass
    set_led_green()  # System running indicator

    menus = ["STRAIGHT", "ARC", "POINT", "SWING", "CIRCLE", "SQUARE", "POLYGON", "TEST", "SET LIMIT", "HLK-LD2450", "FOLLOW", "AVOID", "JOYSTICK", "IMU", "LOG"]

    while True:
        try:
            current_dist = get_radar_distance()
            pos = seesaw_device.get_position()
            count = (pos) % len(menus)
        except Exception as e:
            error_routine("Failed to read encoder position", f"Exception: {e}")

        try:
            if log_mode:
                # Use encoder to scroll through log_messages
                # Reverse direction usually feels more natural for scrolling
                max_scroll = max(0, len(log_messages) - 14)
                scroll_offset = max(0, min(max_scroll, pos))
#                log_scroll_index = pos % max(1, len(log_messages))
                display.fill(0)
                # Show 14 lines starting from the scroll index
#                start = log_scroll_index
                for i in range(14):
#                    if (start + i) < len(log_messages):
#                        display.text(log_messages[start + i], 0, i * 9, 1)
                    idx = scroll_offset + i
                    if idx < len(log_messages):
                        display.text(log_messages[idx], 0, i * 9, 1)
            else:
                # Normal dashboard display
                display.fill(0)
                display.text("XRP DASHBOARD", 15, 4, 1)
                display.hline(0, 14, 128, 1)
                display.text(f"MODE: {menus[count]}", 5, 28, 1)
                display.text(f"BATT: {get_real_volts():.2f}V", 5, 44, 1)
                display.text(f"RADAR: {'Multi' if radar_multi else 'Single'}", 5, 54, 1)  # moved down

                # Current distance readout
                if current_dist >= 65500:
                    display.text("DIST: NO SENSOR", 5, 64, 1)  # moved down
                else:
                    display.text(f"DIST: {int(current_dist)}cm", 5, 64, 1)  # moved down

                # === Distance bar + IMU / Radar + LOG ===
                if imu_mode or radar_mode:
                    bar_y = 75          # higher when active
                    data_y = 85         # data start
                    log_y = 115         # log below data (room for 2 lines)
                else:
                    bar_y = 90          # normal
                    data_y = 0          # no data
                    log_y = 105         # log right below bar

                draw_distance_bar(current_dist, bar_y)

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
        except Exception as e:
            error_routine("Display update failed", f"Exception: {e}")

        try:
            btn = seesaw_device.get_button()
        except Exception as e:
            error_routine("Failed to read button", f"Exception: {e}")

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
            elif count == 14:  # LOG - toggle log display mode
                log_mode = not log_mode
                if log_mode:
                    imu_mode = False  # turn off other modes when viewing log
                    radar_mode = False
            else:
                imu_mode = False
                radar_mode = False
                if count == 8:
                    set_distance_mode()
                else:
                    run_program(count)

        last_button_state = btn
        time.sleep(0.02)

        radar_buffer = hlk_radar.poll_for_response()
        gc.collect()

# Entry point
if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, SystemExit):
        print("\nStopping execution...")
        drivetrain.stop()
        add_log("System halted")
        sys.exit(0)
    except Exception as e:
        # This is where we capture the REAL stack trace
        drivetrain.stop()
        import io
        buf = io.StringIO()
        sys.print_exception(e, buf)
        trace_text = buf.getvalue()
        
        # Log each line of the trace (wrapped to 15 chars)
        for line in trace_text.split('\n'):
            add_log(line)
            
        error_routine("Crash detected")