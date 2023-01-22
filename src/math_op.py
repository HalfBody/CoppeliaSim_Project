import math


def get_target_angle(robot_position, target_position, robot_oriantation):
    robot_vec = [
        math.sin(robot_oriantation),
        -math.cos(robot_oriantation)
        ]

    target_vec = [
        target_position[0] - robot_position[0],
        target_position[1] - robot_position[1]
        ]

    abs_robot = math.sqrt(sum([i ** 2 for i in robot_vec]))
    abs_target = math.sqrt(sum([i ** 2 for i in target_vec]))
    scalar = robot_vec[0] * target_vec[0] + robot_vec[1] * target_vec[1]

    angle = math.acos(scalar / (abs_robot * abs_target)) * 180 / math.pi

    direction = robot_vec[0] * target_vec[1] - robot_vec[1] * target_vec[0]
    angle = -angle if direction >= 0.0 else angle

    return angle


def get_target_dist(robot_position, target_position):
    target_vec = [
        target_position[0] - robot_position[0],
        target_position[1] - robot_position[1]
        ]
    abs_target = math.sqrt(sum([i ** 2 for i in target_vec]))

    return abs_target


def get_move_dist(points):
    dist = 0
    for i in range(len(points) - 1):
        dist += get_target_dist(points[i], points[i+1])
    return dist

    