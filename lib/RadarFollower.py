import time
import math

class RadarFollower:
    """Class to make the robot follow the closest radar target at ~100 cm distance."""
    def __init__(self, drivetrain, hlk_radar, imu=None, threshold=100, buffer=5, max_time=30):
        self.drivetrain = drivetrain
        self.hlk_radar = hlk_radar
        self.imu = imu  # Optional for better heading control
        self.threshold = threshold  # cm
        self.buffer = buffer  # hysteresis to avoid jitter
        self.max_time = max_time  # seconds to run before timeout
        self.base_speed = 0.5  # effort (0-1)
        self.turn_gain = 0.002  # steer based on lateral X (higher = sharper turns)
        self.search_spin = 0.3  # slow spin when no target

    def get_closest_target(self):
        report = self.hlk_radar.parse_radar_report()
        if report:
            min_dist = float('inf')
            closest = None
            for i in range(0, len(report), 4):
                x, y, speed, res = report[i:i+4]
                if res > 0 and (x != 0 or y != 0):
                    dist = math.sqrt(x**2 + y**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest = (x, y, speed, res)
            return closest, min_dist
        return None, 65535

    def run(self):
        add_log("Starting radar follow...")
        start_time = time.ticks_ms()
        while True:
            target, dist = self.get_closest_target()
            
            if time.ticks_diff(time.ticks_ms(), start_time) > self.max_time * 1000:
                add_log("Timeout reached. Stopping.")
                break

            if dist == 65535:  # no target
                # Spin slowly to search
                self.drivetrain.set_effort(self.search_spin, -self.search_spin)
                add_log("No target - searching...")
            else:
                x, y, speed, res = target
                if dist > self.threshold + self.buffer:  # too far - follow
                    # Steer based on X (lateral): positive X = turn right (reduce left effort)
                    steer = self.turn_gain * x
                    left_eff = self.base_speed - steer
                    right_eff = self.base_speed + steer
                    # Clamp efforts to 0-1
                    left_eff = max(0.1, min(1.0, left_eff))
                    right_eff = max(0.1, min(1.0, right_eff))
                    self.drivetrain.set_effort(left_eff, right_eff)
                    add_log(f"Following: dist={dist:.1f} cm, x={x:.1f}")
                elif dist < self.threshold - self.buffer:  # too close - back away
                    # Back straight or slight steer away from X
                    steer = self.turn_gain * x
                    left_eff = -self.base_speed + steer
                    right_eff = -self.base_speed - steer
                    left_eff = max(-1.0, min(-0.1, left_eff))
                    right_eff = max(-1.0, min(-0.1, right_eff))
                    self.drivetrain.set_effort(left_eff, right_eff)
                    add_log(f"Backing away: dist={dist:.1f} cm, x={x:.1f}")
                else:  # perfect distance - stop
                    self.drivetrain.stop()
                    add_log(f"Maintaining distance: {dist:.1f} cm")

            # Optional: Use IMU to maintain heading if drifting (if imu provided)
            if self.imu:
                heading = self.imu.get_heading()
                # Add heading correction if needed (e.g., PID to target heading)

            time.sleep(0.05)  # 20 Hz loop

        self.drivetrain.stop()
        add_log("Radar follow complete.")