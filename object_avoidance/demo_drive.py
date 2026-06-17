from time import sleep
from picarx import Picarx

from object_avoidance.object_avoidance import ObstacleAvoidance
from object_avoidance.object_avoidance_config import NORMAL_SPEED, SLOW_SPEED, AVOID_SPEED, AVOID_LEFT_ANGLE, AVOID_RIGHT_ANGLE, AVOID_TURN_TIME,RETURN_TURN_TIME


px = Picarx()
avoidance = ObstacleAvoidance()


def lane_keep():
    """""PLACEHOLDER"""
    px.set_dir_servo_angle(0)
    px.forward(NORMAL_SPEED)


try:
    while True:
        distance = px.ultrasonic.read()
        action = avoidance.get_action(distance)

        print(f"Distance: {distance} cm"| "{action}")

        if action == "avoid_left":
            px.set_dir_servo_angle(AVOID_LEFT_ANGLE)
            px.forward(AVOID_SPEED)
            sleep(AVOID_TURN_TIME)

            px.set_dir_servo_angle(AVOID_RIGHT_ANGLE)
            px.forward(AVOID_SPEED)
            sleep(AVOID_TURN_TIME)

        elif action == "return_to_lane":
            px.set_dir_servo_angle(AVOID_RIGHT_ANGLE)
            px.forward(AVOID_SPEED)
            sleep(RETURN_TURN_TIME)

            px.set_dir_servo_angle(0)

        elif action == "slow":
            lane_keep()
            px.forward(SLOW_SPEED)

        elif action == "clear":
            lane_keep()

        elif action == "stop":
            px.stop()

        sleep(0.1)

finally:
    px.stop()