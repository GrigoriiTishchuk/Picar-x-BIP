from time import sleep
import time
from picarx import Picarx

from object_avoidance.object_avoidance import ObstacleAvoidance
from object_avoidance.object_avoidance_config import NORMAL_SPEED


px = Picarx()
avoidance = ObstacleAvoidance()


def lane_keep():
    """""PLACEHOLDER"""
    px.set_dir_servo_angle(0)
    px.forward(NORMAL_SPEED)


try:
    while True:
        distance = px.ultrasonic.read()
        control = avoidance.get_control(distance, time.time())

        print(
            f"Distance: {distance} cm | "
            f"Filtered: {control['distance']} | "
            f"Action: {control['action']}"
        )

        if control["override"]:
            if control["steering_angle"] is not None:
                px.set_dir_servo_angle(control["steering_angle"])
            if control["speed"] and control["speed"] > 0:
                px.forward(control["speed"])
            else:
                px.stop()
        elif control["action"] in {"slow", "brake", "clear", "sensor_uncertain"}:
            lane_keep()
            if control["speed"] and control["speed"] < NORMAL_SPEED:
                px.forward(control["speed"])

        sleep(0.1)

finally:
    px.stop()
