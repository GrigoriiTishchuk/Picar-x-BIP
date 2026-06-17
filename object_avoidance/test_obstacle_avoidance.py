from object_avoidance.object_avoidance import ObstacleAvoidance

avoidance = ObstacleAvoidance()

test_distances = [
    50,   # clear
    25,   # slow
    10,   # avoid_left
    12,   # still avoiding
    30,   # return_to_lane
    50,   # clear
]

for distance in test_distances:
    action = avoidance.get_action(distance)

    print(
        f"Distance: {distance:>3} cm | "
        f"Action: {action:<15} | "
        f"Avoiding: {avoidance.is_avoiding}"
    )