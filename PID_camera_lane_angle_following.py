from picarx import Picarx
import time
from time import sleep
import math

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from object_avoidance.object_avoidance import ObstacleAvoidance
from object_avoidance.object_avoidance_config import (
    NORMAL_SPEED,
    SLOW_SPEED,
    AVOID_SPEED,
    AVOID_LEFT_ANGLE,
    AVOID_RIGHT_ANGLE,
    AVOID_TURN_TIME,
    RETURN_TURN_TIME,

    NORMAL_SPEED,
    CORNER_SPEED,
    SEARCH_SPEED,
    REVERSE_SPEED,
    STEERING_LIMIT,
    KP,
    KI,
    KD,
    OFFSET_WEIGHT,
    ANGLE_WEIGHT,
    ROI_TOP_RATIO,
    SCANLINES,
    WHITE_THRESHOLD,
    MIN_WHITE_PIXELS,
    MIN_LANE_WIDTH,
    MAX_LANE_WIDTH,
    EXPECTED_LANE_WIDTH,
    STEERING_SMOOTHING,
    ERROR_SMOOTHING,
    CORNER_ERROR_THRESHOLD,
    CORNER_ANGLE_THRESHOLD_DEG,
    DEBUG_MODE,
)


SETTINGS = {
    # Speed
    "SPEED_FORWARD": NORMAL_SPEED,
    "SPEED_CORNER": CORNER_SPEED,
    "SPEED_SEARCH": SEARCH_SPEED,
    "SPEED_REVERSE": REVERSE_SPEED,

    # Steering
    "STEERING_LIMIT": STEERING_LIMIT,

    # PID
    "KP": KP,
    "KI": KI,
    "KD": KD,

    # Lane control
    "OFFSET_WEIGHT": OFFSET_WEIGHT,
    "ANGLE_WEIGHT": ANGLE_WEIGHT,

    # Camera
    "ROI_TOP_RATIO": ROI_TOP_RATIO,
    "SCANLINES": SCANLINES,
    "WHITE_THRESHOLD": WHITE_THRESHOLD,
    "MIN_WHITE_PIXELS": MIN_WHITE_PIXELS,
    "MIN_LANE_WIDTH": MIN_LANE_WIDTH,
    "MAX_LANE_WIDTH": MAX_LANE_WIDTH,
    "EXPECTED_LANE_WIDTH": EXPECTED_LANE_WIDTH,

    # Filtering
    "STEERING_SMOOTHING": STEERING_SMOOTHING,
    "ERROR_SMOOTHING": ERROR_SMOOTHING,

    # Corners
    "CORNER_ERROR_THRESHOLD": CORNER_ERROR_THRESHOLD,
    "CORNER_ANGLE_THRESHOLD_DEG": CORNER_ANGLE_THRESHOLD_DEG,

    # Debug
    "DEBUG_MODE": DEBUG_MODE,
}


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.001

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.prev_error = error
        self.prev_time = current_time
        return output


class LaneDetector:
    """
    Detects a black lane with white left/right borders.

    It does NOT follow a middle line.
    It finds the two white edges, calculates the lane center at several scanlines,
    then estimates:
      1. offset error: how far the lane center is from the camera center
      2. angle error: which direction the lane is turning
    """

    def __init__(self, settings):
        self.cfg = settings
        self.last_error = 0.0
        self.last_angle_deg = 0.0
        self.last_direction = 0
        self.last_lane_width = self.cfg["EXPECTED_LANE_WIDTH"]

    def threshold_white(self, frame):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV and NumPy are required for camera lane detection.")

        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # Smooth small noise before thresholding
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # White lane borders become 255, black lane becomes 0
        _, binary = cv2.threshold(
            gray,
            self.cfg["WHITE_THRESHOLD"],
            255,
            cv2.THRESH_BINARY,
        )
        return binary

    def find_border_positions_on_row(self, row):
        """Returns left_border_x and right_border_x from a single binary row."""
        white_indices = np.where(row > 0)[0]

        if len(white_indices) < self.cfg["MIN_WHITE_PIXELS"]:
            return None, None

        width = len(row)
        center_x = width // 2

        left_candidates = white_indices[white_indices < center_x]
        right_candidates = white_indices[white_indices > center_x]

        left_x = None
        right_x = None

        if len(left_candidates) >= self.cfg["MIN_WHITE_PIXELS"]:
            # Rightmost white point on the left half is the inner left border
            left_x = int(left_candidates.max())

        if len(right_candidates) >= self.cfg["MIN_WHITE_PIXELS"]:
            # Leftmost white point on the right half is the inner right border
            right_x = int(right_candidates.min())

        return left_x, right_x

    def estimate_missing_border(self, left_x, right_x):
        """If one border is missing, estimate it using last known lane width."""
        lane_width = self.last_lane_width or self.cfg["EXPECTED_LANE_WIDTH"]

        if left_x is not None and right_x is None:
            right_x = left_x + lane_width
        elif right_x is not None and left_x is None:
            left_x = right_x - lane_width

        return left_x, right_x

    def detect(self, frame):
        binary = self.threshold_white(frame)
        height, width = binary.shape[:2]
        image_center = width / 2.0

        roi_top = int(height * self.cfg["ROI_TOP_RATIO"])
        roi = binary[roi_top:height, :]
        roi_height = roi.shape[0]

        center_points = []

        for ratio in self.cfg["SCANLINES"]:
            y_roi = int(roi_height * ratio)
            y_roi = max(0, min(roi_height - 1, y_roi))
            row = roi[y_roi, :]

            left_x, right_x = self.find_border_positions_on_row(row)
            left_x, right_x = self.estimate_missing_border(left_x, right_x)

            if left_x is None or right_x is None:
                continue

            lane_width = right_x - left_x
            if lane_width < self.cfg["MIN_LANE_WIDTH"] or lane_width > self.cfg["MAX_LANE_WIDTH"]:
                continue

            self.last_lane_width = lane_width
            lane_center_x = (left_x + right_x) / 2.0
            y_full = roi_top + y_roi
            center_points.append((lane_center_x, y_full))

        if len(center_points) < 2:
            return {
                "detected": False,
                "error": self.last_error,
                "offset_error": self.last_error,
                "angle_deg": self.last_angle_deg,
                "center_points": center_points,
            }

        # Bottom point gives immediate lateral offset
        bottom_center_x, _ = center_points[-1]
        offset_error = image_center - bottom_center_x

        # Use first and last center points to estimate lane direction angle
        top_center_x, top_y = center_points[0]
        bottom_center_x, bottom_y = center_points[-1]

        dx = bottom_center_x - top_center_x
        dy = bottom_y - top_y
        if abs(dy) < 1:
            dy = 1

        # Angle relative to straight-ahead vertical direction
        angle_rad = math.atan2(dx, dy)
        angle_deg = math.degrees(angle_rad)

        # Combine offset and angle into one steering error
        # If sign is backwards on your car, multiply combined_error by -1.
        combined_error = (
            self.cfg["OFFSET_WEIGHT"] * offset_error
            - self.cfg["ANGLE_WEIGHT"] * angle_deg
        )

        # Smooth error to avoid twitching
        alpha = self.cfg["ERROR_SMOOTHING"]
        combined_error = (alpha * combined_error) + ((1.0 - alpha) * self.last_error)

        self.last_error = combined_error
        self.last_angle_deg = angle_deg

        if combined_error > 2:
            self.last_direction = 1
        elif combined_error < -2:
            self.last_direction = -1

        return {
            "detected": True,
            "error": combined_error,
            "offset_error": offset_error,
            "angle_deg": angle_deg,
            "center_points": center_points,
        }


class AutonomousCar:
    def __init__(self, settings):
        self.cfg = settings
        self.hardware = Picarx()
        self.pid = PIDController(self.cfg["KP"], self.cfg["KI"], self.cfg["KD"])
        self.lane_detector = LaneDetector(self.cfg)
        self.avoidance = ObstacleAvoidance()
        self.last_steering_angle = 0.0

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
        raw_angle = self.clamp_steering(raw_angle)

        # Smooth servo movement so the steering does not jerk
        alpha = self.cfg["STEERING_SMOOTHING"]
        smooth_angle = (alpha * raw_angle) + ((1.0 - alpha) * self.last_steering_angle)
        safe_angle = self.clamp_steering(smooth_angle)

        self.hardware.set_dir_servo_angle(safe_angle)
        self.last_steering_angle = safe_angle

        return safe_angle

    def choose_speed(self, lane_result):
        if not lane_result["detected"]:
            return self.cfg["SPEED_SEARCH"]

        error = abs(lane_result["error"])
        angle = abs(lane_result["angle_deg"])

        if (
            error > self.cfg["CORNER_ERROR_THRESHOLD"]
            or angle > self.cfg["CORNER_ANGLE_THRESHOLD_DEG"]
        ):
            return self.cfg["SPEED_CORNER"]

        return self.cfg["SPEED_FORWARD"]

    def search_for_lane(self):
        direction = self.lane_detector.last_direction
        if direction == 0:
            direction = 1

        search_angle = self.clamp_steering(direction * 18)
        self.hardware.set_dir_servo_angle(search_angle)
        self.hardware.forward(self.cfg["SPEED_SEARCH"])

        if self.cfg["DEBUG_MODE"]:
            print(f"[WARNING] Lane lost. Searching with angle {search_angle:.1f}")

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


class CameraReader:
    """
    Basic OpenCV camera reader.

    If your team sends frames over network instead of using a local USB/CSI camera,
    replace get_frame() with your socket/frame receiver.
    """

    def __init__(self, camera_index=0):
        if cv2 is None:
            raise RuntimeError("OpenCV is required for camera input.")
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera.")

    def get_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def release(self):
        self.cap.release()


def get_camera_frame(camera):
    return camera.get_frame()


def main():
    car = AutonomousCar(SETTINGS)
    camera = CameraReader(camera_index=0)

    print("[INFO] System ready. Camera lane angle following active.")

    try:
        while True:
            obstacle_handled = car.handle_obstacle()
            if obstacle_handled:
                time.sleep(0.1)
                continue

            frame = get_camera_frame(camera)
            if frame is None:
                print("[WARNING] No camera frame received.")
                car.stop()
                time.sleep(0.1)
                continue

            lane_result = car.lane_detector.detect(frame)

            if not lane_result["detected"]:
                car.search_for_lane()
                time.sleep(0.05)
                continue

            speed = car.choose_speed(lane_result)
            steering_angle = car.update_steering(lane_result["error"])
            car.drive_forward(speed)

            if SETTINGS["DEBUG_MODE"]:
                print(
                    f"[LANE] error={lane_result['error']:.2f} | "
                    f"offset={lane_result['offset_error']:.2f} | "
                    f"angle={lane_result['angle_deg']:.2f} deg | "
                    f"steer={steering_angle:.2f} | speed={speed}"
                )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop triggered.")

    finally:
        car.stop()
        camera.release()
        print("[INFO] Hardware shutdown safe.")


if __name__ == "__main__":
    main()
