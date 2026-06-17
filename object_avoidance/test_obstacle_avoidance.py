import time

from object_avoidance.object_avoidance import ObstacleAvoidance

avoidance = ObstacleAvoidance()
start = time.time()

test_distances = [
    50,   # clear
    25,   # slow
    10,   # avoid_left
    12,   # still avoiding
    30,   # return_to_lane
    50,   # clear
]

for index, distance in enumerate(test_distances):
    control = avoidance.get_control(distance, start + (index * 0.5))

    print(
        f"Distance: {distance:>3} cm | "
        f"Filtered: {control['distance']!s:>5} | "
        f"Action: {control['action']:<15} | "
        f"Avoiding: {avoidance.is_avoiding}"
    )
