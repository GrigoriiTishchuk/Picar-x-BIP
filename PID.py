from picarx import Picarx
import time
from time import sleep

from object_avoidance.object_avoidance import ObstacleAvoidance
from object_avoidance.object_avoidance_config import (
    NORMAL_SPEED,
    SLOW_SPEED,
    AVOID_SPEED,
    AVOID_LEFT_ANGLE,
    AVOID_RIGHT_ANGLE,
    AVOID_TURN_TIME,
    RETURN_TURN_TIME,
)


SETTINGS = {
    "SPEED_FORWARD": NORMAL_SPEED,
    "SPEED_REVERSE": -20,
    "STEERING_LIMIT": 45,
    "KP": 0.25,
    "KI": 0.0,
    "KD": 0.1,
    "DEBUG_MODE": True,
}


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.prev_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time

        if dt <= 0:
            dt = 0.001

        p_out = self.kp * error
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time

        return p_out + d_out


class AutonomousCar:
    def __init__(self, settings):
        self.cfg = settings
        self.hardware = Picarx()
        self.pid = PIDController(self.cfg["KP"], self.cfg["KI"], self.cfg["KD"])
        self.avoidance = ObstacleAvoidance()

    def clamp_steering(self, angle):
        limit = self.cfg["STEERING_LIMIT"]
        return max(-limit, min(limit, angle))

    def drive_forward(self, speed=None):
        if speed is None:
            speed = self.cfg["SPEED_FORWARD"]
        self.hardware.forward(speed)

    def reverse(self):
        self.hardware.set_dir_servo_angle(0)
        self.hardware.backward(abs(self.cfg["SPEED_REVERSE"]))

    def update_steering(self, error):
        raw_angle = self.pid.compute(error)
        safe_angle = self.clamp_steering(raw_angle)

        self.hardware.set_dir_servo_angle(safe_angle)

        if self.cfg["DEBUG_MODE"]:
            print(
                f"[DEBUG] Error: {error} | "
                f"Raw Angle: {raw_angle:.2f} | "
                f"Safe Angle: {safe_angle:.2f}"
            )

    def handle_obstacle(self):
        distance = self.hardware.ultrasonic.read()
        action = self.avoidance.get_action(distance)

        if self.cfg["DEBUG_MODE"]:
            print(f"Distance: {distance} cm | Action: {action}")

        if action == "avoid_left":
            self.hardware.set_dir_servo_angle(AVOID_LEFT_ANGLE)
            self.hardware.forward(AVOID_SPEED)
            sleep(AVOID_TURN_TIME)

            self.hardware.set_dir_servo_angle(AVOID_RIGHT_ANGLE)
            self.hardware.forward(AVOID_SPEED)
            sleep(AVOID_TURN_TIME)

            return True

        elif action == "return_to_lane":
            self.hardware.set_dir_servo_angle(AVOID_RIGHT_ANGLE)
            self.hardware.forward(AVOID_SPEED)
            sleep(RETURN_TURN_TIME)

            self.hardware.set_dir_servo_angle(0)
            return True

        elif action == "slow":
            self.hardware.forward(SLOW_SPEED)
            return False

        elif action == "stop":
            self.stop()
            return True

        elif action == "clear":
            return False

        return False

    def stop(self):
        self.hardware.stop()


def get_camera_data():
    """
    Placeholder for receiving data from the Vision Team.
    Insert socket/network reading code here.
    """
    pass
def get_ultrasonic_data():
    """
    Placeholder for receiving data from the Ultrasonic Team.
    Insert socket/network reading code here.
    """
    # Simulated data for testing purposes
    # return 15 
    pass

def get_grey_scale_data():
    """
    Placeholder for receiving data from the Grey Scale Team.
    Insert socket/network reading code here.
    """
    # Simulated data for testing purposes
    # return 0.75 
    pass

def main():
    car = AutonomousCar(SETTINGS)
    print("[INFO] System ready. Waiting for vision data...")

    try:
        while True:
            obstacle_handled = car.handle_obstacle()

            if obstacle_handled:
                time.sleep(0.1)
                continue

            data = get_camera_data()

            if data == "LOST" or data is None:
                if SETTINGS["DEBUG_MODE"]:
                    print("[WARNING] Line lost! Reversing...")
                car.reverse()
                time.sleep(0.1)
                continue

            if isinstance(data, (int, float)):
                car.drive_forward()
                car.update_steering(data)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop triggered.")

    finally:
        car.stop()
        print("[INFO] Hardware shutdown safe.")


if __name__ == "__main__":
    main()