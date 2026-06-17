import time
from collections import deque
from statistics import median

from object_avoidance.object_avoidance_config import (
    AVOID_LEFT_ANGLE,
    AVOID_RIGHT_ANGLE,
    AVOID_SPEED,
    AVOID_TURN_TIME,
    BRAKE_DISTANCE,
    CLEAR_CONFIRM_FRAMES,
    EMERGENCY_STOP_DISTANCE,
    OBSTACLE_CONFIRM_FRAMES,
    RETURN_TURN_TIME,
    SLOW_DISTANCE,
    SLOW_SPEED,
    STOP_DISTANCE,
    ULTRASONIC_WINDOW,
)


class ObstacleAvoidance:
    def __init__(self):
        self.is_avoiding = False
        self.distance_history = deque(maxlen=ULTRASONIC_WINDOW)
        self.danger_frames = 0
        self.clear_frames = 0
        self.state = "clear"
        self.state_started_at = 0.0

    def sanitize_distance(self, distance):
        if distance is None or distance <= 0:
            return None
        return float(distance)

    def filtered_distance(self, distance):
        value = self.sanitize_distance(distance)
        if value is not None:
            self.distance_history.append(value)

        if not self.distance_history:
            return None

        return float(median(self.distance_history))

    def transition_to(self, state, now):
        self.state = state
        self.state_started_at = now
        self.is_avoiding = state not in {"clear", "slow", "brake"}

    def get_control(self, distance, now):
        raw_distance = self.sanitize_distance(distance)
        filtered_distance = self.filtered_distance(distance)
        reaction_distance = filtered_distance

        if self.state == "avoid_left":
            if now - self.state_started_at >= AVOID_TURN_TIME:
                self.transition_to("avoid_right", now)
            else:
                return self._build_response(filtered_distance, "avoid_left", True, AVOID_SPEED, AVOID_LEFT_ANGLE)

        if self.state == "avoid_right":
            if now - self.state_started_at >= AVOID_TURN_TIME:
                self.transition_to("return_to_lane", now)
            else:
                return self._build_response(filtered_distance, "avoid_right", True, AVOID_SPEED, AVOID_RIGHT_ANGLE)

        if self.state == "return_to_lane":
            if now - self.state_started_at >= RETURN_TURN_TIME:
                self.transition_to("clear", now)
            else:
                return self._build_response(filtered_distance, "return_to_lane", True, AVOID_SPEED, 0)

        if reaction_distance is None:
            self.danger_frames = 0
            self.clear_frames = 0
            return self._build_response(None, "sensor_uncertain", False, SLOW_SPEED, None)

        if reaction_distance <= STOP_DISTANCE:
            self.danger_frames += 1
            self.clear_frames = 0
        elif reaction_distance >= SLOW_DISTANCE:
            self.clear_frames += 1
            self.danger_frames = 0
        else:
            self.clear_frames = 0
            self.danger_frames = 0

        if raw_distance is not None and raw_distance <= EMERGENCY_STOP_DISTANCE:
            self.transition_to("stop", now)
            return self._build_response(raw_distance, "stop", True, 0, 0)

        if reaction_distance <= STOP_DISTANCE and self.danger_frames >= OBSTACLE_CONFIRM_FRAMES:
            self.transition_to("avoid_left", now)
            return self._build_response(filtered_distance, "avoid_left", True, AVOID_SPEED, AVOID_LEFT_ANGLE)

        if reaction_distance <= BRAKE_DISTANCE:
            self.transition_to("brake", now)
            return self._build_response(filtered_distance, "brake", False, SLOW_SPEED, None)

        if reaction_distance <= SLOW_DISTANCE:
            self.transition_to("slow", now)
            return self._build_response(filtered_distance, "slow", False, SLOW_SPEED, None)

        if self.clear_frames >= CLEAR_CONFIRM_FRAMES:
            self.transition_to("clear", now)

        return self._build_response(filtered_distance, "clear", False, None, None)

    def _build_response(self, filtered_distance, action, override, speed, steering_angle):
        return {
            "distance": filtered_distance,
            "action": action,
            "override": override,
            "speed": speed,
            "steering_angle": steering_angle,
        }

    def get_action(self, distance, now=None):
        if now is None:
            now = time.time()
        return self.get_control(distance, now=now)["action"]

    def should_override_lane_keeping(self):
        return self.is_avoiding
