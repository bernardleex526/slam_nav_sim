"""Fixed mapping-route control from the algorithm's map-frame pose, never simulator truth."""

import math


def waypoint_command(map_pose, goal, now_s):
    if map_pose is None or now_s - map_pose[0] > 0.5:
        return 0.0, 0.0, False
    _, x, y, yaw = map_pose
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)
    if distance < 0.08:
        return 0.0, 0.0, True
    error = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
    linear = min(0.35, distance) * max(0, math.cos(error)) if abs(error) < 0.4 else 0.0
    angular = max(-0.6, min(0.6, 1.5 * error))
    return linear, angular, False
