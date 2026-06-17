from picarx import Picarx
import time
import math

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from object_avoidance.object_avoidance import ObstacleAvoidance
from object_avoidance.object_avoidance_config import (
    CORNER_SPEED,
    DERIVATIVE_SMOOTHING,
    ERROR_SMOOTHING,
    EXPECTED_LANE_WIDTH,
    KD,
    KI,
    KP,
    LANE_MIN_CONFIDENCE,
    LOW_CONFIDENCE_SPEED,
    MASK_MORPH_KERNEL,
    MAX_LANE_WIDTH,
    MAX_STEERING_STEP,
    MIN_LANE_WIDTH,
    MIN_WHITE_PIXELS,
    NORMAL_SPEED,
    OFFSET_WEIGHT,
    PID_INTEGRAL_LIMIT,
    REVERSE_SPEED,
    ROI_TOP_RATIO,
    SCANLINES,
    SEARCH_SPEED,
    SIGN_ACTION_DELAY,
    SIGN_CONFIRM_FRAMES,
    SIGN_COOLDOWN,
    SIGN_DETECTION_ENABLED,
    SIGN_LEFT_ANGLE,
    SIGN_RIGHT_ANGLE,
    SIGN_STOP_HOLD_TIME,
    SIGN_TURN_SPEED,
    SIGN_TURN_TIME,
    SLOW_SPEED,
    STEERING_LIMIT,
    STEERING_SMOOTHING,
    WHITE_HLS_LIGHTNESS_MIN,
    WHITE_HLS_SATURATION_MAX,
    WHITE_HSV_SATURATION_MAX,
    WHITE_HSV_VALUE_MIN,
    WHITE_THRESHOLD,
    ANGLE_WEIGHT,
    CORNER_ERROR_THRESHOLD,
    CORNER_ANGLE_THRESHOLD_DEG,
    DEBUG_MODE,
)
from street_sign_controller import StreetSignController, StreetSignDetectorAdapter


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
    "PID_INTEGRAL_LIMIT": PID_INTEGRAL_LIMIT,
    "DERIVATIVE_SMOOTHING": DERIVATIVE_SMOOTHING,

    # Lane control
    "OFFSET_WEIGHT": OFFSET_WEIGHT,
    "ANGLE_WEIGHT": ANGLE_WEIGHT,

    # Camera
    "ROI_TOP_RATIO": ROI_TOP_RATIO,
    "SCANLINES": SCANLINES,
    "WHITE_THRESHOLD": WHITE_THRESHOLD,
    "WHITE_HLS_LIGHTNESS_MIN": WHITE_HLS_LIGHTNESS_MIN,
    "WHITE_HLS_SATURATION_MAX": WHITE_HLS_SATURATION_MAX,
    "WHITE_HSV_VALUE_MIN": WHITE_HSV_VALUE_MIN,
    "WHITE_HSV_SATURATION_MAX": WHITE_HSV_SATURATION_MAX,
    "MASK_MORPH_KERNEL": MASK_MORPH_KERNEL,
    "MIN_WHITE_PIXELS": MIN_WHITE_PIXELS,
    "MIN_LANE_WIDTH": MIN_LANE_WIDTH,
    "MAX_LANE_WIDTH": MAX_LANE_WIDTH,
    "EXPECTED_LANE_WIDTH": EXPECTED_LANE_WIDTH,
    "LANE_MIN_CONFIDENCE": LANE_MIN_CONFIDENCE,

    # Filtering
    "STEERING_SMOOTHING": STEERING_SMOOTHING,
    "ERROR_SMOOTHING": ERROR_SMOOTHING,
    "MAX_STEERING_STEP": MAX_STEERING_STEP,

    # Corners
    "CORNER_ERROR_THRESHOLD": CORNER_ERROR_THRESHOLD,
    "CORNER_ANGLE_THRESHOLD_DEG": CORNER_ANGLE_THRESHOLD_DEG,
    "LOW_CONFIDENCE_SPEED": LOW_CONFIDENCE_SPEED,

    # Debug
    "DEBUG_MODE": DEBUG_MODE,

    # Street signs
    "SIGN_DETECTION_ENABLED": SIGN_DETECTION_ENABLED,
    "SIGN_ACTION_DELAY": SIGN_ACTION_DELAY,
    "SIGN_CONFIRM_FRAMES": SIGN_CONFIRM_FRAMES,
    "SIGN_COOLDOWN": SIGN_COOLDOWN,
    "SIGN_TURN_SPEED": SIGN_TURN_SPEED,
    "SIGN_LEFT_ANGLE": SIGN_LEFT_ANGLE,
    "SIGN_RIGHT_ANGLE": SIGN_RIGHT_ANGLE,
    "SIGN_TURN_TIME": SIGN_TURN_TIME,
    "SIGN_STOP_HOLD_TIME": SIGN_STOP_HOLD_TIME,
}


class PIDController:
    def __init__(self, kp, ki, kd, integral_limit, derivative_smoothing):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.derivative_smoothing = derivative_smoothing
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.001

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        alpha = self.derivative_smoothing
        derivative = (alpha * derivative) + ((1.0 - alpha) * self.prev_derivative)

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.prev_error = error
        self.prev_derivative = derivative
        self.prev_time = current_time
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.prev_time = time.time()


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
        self.min_points_for_reliable_fit = max(3, len(self.cfg["SCANLINES"]) - 1)

    def threshold_white(self, frame):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV and NumPy are required for camera lane detection.")

        if len(frame.shape) == 3:
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hls = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            hls_mask = cv2.inRange(
                hls,
                np.array([0, self.cfg["WHITE_HLS_LIGHTNESS_MIN"], 0], dtype=np.uint8),
                np.array([255, 255, self.cfg["WHITE_HLS_SATURATION_MAX"]], dtype=np.uint8),
            )
            hsv_mask = cv2.inRange(
                hsv,
                np.array([0, 0, self.cfg["WHITE_HSV_VALUE_MIN"]], dtype=np.uint8),
                np.array([255, self.cfg["WHITE_HSV_SATURATION_MAX"], 255], dtype=np.uint8),
            )
            binary = cv2.bitwise_or(hls_mask, hsv_mask)
        else:
            gray = frame
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(
                gray,
                self.cfg["WHITE_THRESHOLD"],
                255,
                cv2.THRESH_BINARY,
            )

        kernel_size = self.cfg["MASK_MORPH_KERNEL"]
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
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

    def compute_confidence(self, center_points, fitted_x):
        point_confidence = len(center_points) / max(1, len(self.cfg["SCANLINES"]))

        if len(center_points) < self.min_points_for_reliable_fit:
            return min(point_confidence, 0.35)

        xs = np.array([point[0] for point in center_points], dtype=np.float32)
        residual = float(np.mean(np.abs(xs - fitted_x)))
        residual_confidence = max(0.0, 1.0 - (residual / 25.0))
        return max(0.0, min(1.0, (0.65 * point_confidence) + (0.35 * residual_confidence)))

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
                "confidence": 0.0,
            }

        ys = np.array([point[1] for point in center_points], dtype=np.float32)
        xs = np.array([point[0] for point in center_points], dtype=np.float32)
        slope, intercept = np.polyfit(ys, xs, 1)

        bottom_y = float(height - 1)
        predicted_bottom_center_x = (slope * bottom_y) + intercept
        offset_error = image_center - predicted_bottom_center_x

        # Angle relative to straight-ahead vertical direction
        angle_rad = math.atan(slope)
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

        fitted_x = (slope * ys) + intercept
        confidence = self.compute_confidence(center_points, fitted_x)

        if len(center_points) < self.min_points_for_reliable_fit and confidence < self.cfg["LANE_MIN_CONFIDENCE"]:
            return {
                "detected": False,
                "error": self.last_error,
                "offset_error": offset_error,
                "angle_deg": angle_deg,
                "center_points": center_points,
                "confidence": confidence,
            }

        return {
            "detected": True,
            "error": combined_error,
            "offset_error": offset_error,
            "angle_deg": angle_deg,
            "center_points": center_points,
            "confidence": confidence,
        }


class AutonomousCar:
    def __init__(self, settings):
        self.cfg = settings
        self.hardware = Picarx()
        self.pid = PIDController(
            self.cfg["KP"],
            self.cfg["KI"],
            self.cfg["KD"],
            self.cfg["PID_INTEGRAL_LIMIT"],
            self.cfg["DERIVATIVE_SMOOTHING"],
        )
        self.lane_detector = LaneDetector(self.cfg)
        self.avoidance = ObstacleAvoidance()
        self.sign_controller = StreetSignController(self.cfg)
        self.sign_detector = StreetSignDetectorAdapter(self.cfg["SIGN_DETECTION_ENABLED"])
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
        target_angle = self.clamp_steering(self.pid.compute(error))

        # Smooth servo movement so the steering does not jerk
        alpha = self.cfg["STEERING_SMOOTHING"]
        smooth_angle = (alpha * target_angle) + ((1.0 - alpha) * self.last_steering_angle)

        max_step = self.cfg["MAX_STEERING_STEP"]
        lower_bound = self.last_steering_angle - max_step
        upper_bound = self.last_steering_angle + max_step
        limited_angle = max(lower_bound, min(upper_bound, smooth_angle))
        safe_angle = self.clamp_steering(limited_angle)

        self.hardware.set_dir_servo_angle(safe_angle)
        self.last_steering_angle = safe_angle

        return safe_angle

    def choose_speed(self, lane_result, obstacle_speed_limit=None):
        if not lane_result["detected"]:
            speed = self.cfg["SPEED_SEARCH"]
        else:
            error = abs(lane_result["error"])
            angle = abs(lane_result["angle_deg"])
            confidence = lane_result.get("confidence", 1.0)

            if confidence < self.cfg["LANE_MIN_CONFIDENCE"]:
                speed = self.cfg["LOW_CONFIDENCE_SPEED"]
            elif (
                error > self.cfg["CORNER_ERROR_THRESHOLD"]
                or angle > self.cfg["CORNER_ANGLE_THRESHOLD_DEG"]
            ):
                speed = self.cfg["SPEED_CORNER"]
            else:
                speed = self.cfg["SPEED_FORWARD"]

        if obstacle_speed_limit is not None:
            speed = min(speed, obstacle_speed_limit)

        return speed

    def search_for_lane(self):
        direction = self.lane_detector.last_direction
        if direction == 0:
            direction = 1

        search_angle = self.clamp_steering(direction * 18)
        self.hardware.set_dir_servo_angle(search_angle)
        self.hardware.forward(self.cfg["SPEED_SEARCH"])

        if self.cfg["DEBUG_MODE"]:
            print(f"[WARNING] Lane lost. Searching with angle {search_angle:.1f}")

    def handle_obstacle(self, now):
        distance = self.hardware.ultrasonic.read()
        control = self.avoidance.get_control(distance, now)

        if self.cfg["DEBUG_MODE"]:
            print(
                f"[OBSTACLE] raw={distance} cm | "
                f"filtered={control['distance']} | "
                f"action={control['action']}"
            )

        if control["override"]:
            self.pid.reset()
            self.last_steering_angle = 0.0
            steering_angle = control["steering_angle"]
            if steering_angle is not None:
                self.hardware.set_dir_servo_angle(steering_angle)
            if control["speed"] and control["speed"] > 0:
                self.hardware.forward(control["speed"])
            else:
                self.stop()
        elif control["action"] == "stop":
            self.stop()

        return control

    def detect_street_sign(self, frame):
        return self.sign_detector.detect(frame)

    def handle_street_sign(self, now, detected_sign=None):
        control = self.sign_controller.update(now, detected_sign)

        if self.cfg["DEBUG_MODE"] and detected_sign is not None:
            print(f"[SIGN] detected={detected_sign} | action={control['action']}")

        if control["override"]:
            self.pid.reset()
            self.last_steering_angle = 0.0
            steering_angle = control["steering_angle"]
            if steering_angle is not None:
                self.hardware.set_dir_servo_angle(steering_angle)
            if control["speed"] and control["speed"] > 0:
                self.hardware.forward(control["speed"])
            else:
                self.stop()

        return control

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
    if car.sign_detector.available:
        print("[INFO] Street sign detector loaded.")
    else:
        print("[INFO] Street sign detector not found. Sign handling is idle.")

    try:
        while True:
            now = time.time()
            obstacle_control = car.handle_obstacle(now)
            if obstacle_control["override"]:
                time.sleep(0.05)
                continue

            sign_control = car.handle_street_sign(now)
            if sign_control["override"]:
                time.sleep(0.05)
                continue

            frame = get_camera_frame(camera)
            if frame is None:
                print("[WARNING] No camera frame received.")
                car.stop()
                time.sleep(0.1)
                continue

            detected_sign = car.detect_street_sign(frame)
            sign_control = car.handle_street_sign(now, detected_sign)
            if sign_control["override"]:
                time.sleep(0.05)
                continue

            lane_result = car.lane_detector.detect(frame)

            if not lane_result["detected"]:
                car.pid.reset()
                car.search_for_lane()
                time.sleep(0.05)
                continue

            speed = car.choose_speed(lane_result, obstacle_control["speed"])
            steering_angle = car.update_steering(lane_result["error"])
            car.drive_forward(speed)

            if SETTINGS["DEBUG_MODE"]:
                print(
                    f"[LANE] error={lane_result['error']:.2f} | "
                    f"offset={lane_result['offset_error']:.2f} | "
                    f"angle={lane_result['angle_deg']:.2f} deg | "
                    f"confidence={lane_result['confidence']:.2f} | "
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
