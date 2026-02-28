from XRPLib.defaults import *
import time
import math
class ObstacleAvoider:
    """Class to make the robot go forward 500 cm (5m), avoiding obstacles."""
    def __init__(self, drivetrain, hlk_radar, target_distance=500, avoid_threshold=50, max_time=60):
        self.drivetrain = drivetrain
        self.hlk_radar = hlk_radar
        self.target_distance = target_distance  # cm
        self.avoid_threshold = avoid_threshold  # cm (stop/turn if closer)
        self.max_time = max_time  # seconds timeout
        self.base_speed = 0.6
        self.turn_speed = 0.5
        self.turn_time = 1.0  # seconds to turn when avoiding

    def get_obstacle_distance(self):
        # Reuse your get_radar_distance() or rangefinder if available
        report = self.hlk_radar.parse_radar_report()
        if report:
            min_dist = float('inf')
            for i in range(0, len(report), 4):
                x, y, speed, res = report[i:i+4]
                if res > 0 and (x != 0 or y != 0):
                    dist = math.sqrt(x**2 + y**2)
                    if dist < min_dist:
                        min_dist = dist
            if min_dist != float('inf'):
                return min_dist
        return 65535

    def run(self):
        add_log("Starting 5m obstacle avoid...")
        start_time = time.ticks_ms()
        traveled = 0.0  # track with encoders

        # Reset encoders
        try:
            self.drivetrain.left_motor.reset_relative_position()
            self.drivetrain.right_motor.reset_relative_position()
        except AttributeError:
            self.drivetrain.left_motor.reset_encoder_position()
            self.drivetrain.right_motor.reset_encoder_position()

        while traveled < self.target_distance:
            if time.ticks_diff(time.ticks_ms(), start_time) > self.max_time * 1000:
                add_log("Timeout reached. Stopping.")
                break

            dist = self.get_obstacle_distance()
            if dist < self.avoid_threshold and dist != 65535:
                add_log(f"Obstacle at {dist:.1f} cm - avoiding...")
                self.drivetrain.stop()
                # Random turn left or right
                direction = random.choice([-1, 1])  # -1 left, 1 right
                self.drivetrain.set_effort(direction * self.turn_speed, -direction * self.turn_speed)
                time.sleep(self.turn_time)
                self.drivetrain.stop()
            else:
                # Go forward
                self.drivetrain.set_effort(self.base_speed, self.base_speed)
                time.sleep(0.05)

            # Update traveled (average encoders)
            left_pos = abs(self.drivetrain.left_motor.get_position())
            right_pos = abs(self.drivetrain.right_motor.get_position())
            traveled = (left_pos + right_pos) / 2.0  # assuming cm units from XRPLib

            add_log(f"Traveled: {traveled:.1f} cm")

        self.drivetrain.stop()
        add_log("5m destination reached.")