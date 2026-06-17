from object_avoidance.object_avoidance_config import STOP_DISTANCE, SLOW_DISTANCE


class ObstacleAvoidance:
    def __init__(self):
        self.is_avoiding = False

    def get_action(self, distance):
        if distance is None or distance <= 0:
            return "stop"

        if distance < STOP_DISTANCE:
            self.is_avoiding = True
            return "avoid_left"

        if self.is_avoiding and distance > SLOW_DISTANCE:
            self.is_avoiding = False
            return "return_to_lane"

        if distance < SLOW_DISTANCE:
            return "slow"

        return "clear"

    def should_override_lane_keeping(self):
        return self.is_avoiding