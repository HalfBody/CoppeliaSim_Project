import math

def get_target_angle(robot_position, target_position, robot_oriantation, mode):
    x = math.sin(robot_oriantation)
    y = -math.cos(robot_oriantation)

    robot_vec = [x, y]
    target_vec = [target_position[0] - robot_position[0], target_position[1] - robot_position[1]]

    abs_robot = math.sqrt(math.pow(robot_vec[0], 2) + math.pow(robot_vec[1], 2))
    abs_target = math.sqrt(math.pow(target_vec[0], 2) + math.pow(target_vec[1], 2))
    scalar = robot_vec[0]*target_vec[0] + robot_vec[1]*target_vec[1]

    angle = math.acos((scalar)/(abs_robot * abs_target))
    angle = angle*180/3.14

    direction = robot_vec[0] * target_vec[1] - robot_vec[1] * target_vec[0]
    if mode == True: print('math_op.get_target_angle: ', angle)
    if direction >= 0:
        return -angle*3.14/180
    else:
        return angle*3.14/180


def get_target_dist(robot_position, target_position, mode):
    target_vec = [target_position[0] - robot_position[0], target_position[1] - robot_position[1]]
    abs_target = math.sqrt(math.pow(target_vec[0], 2) + math.pow(target_vec[1], 2))
    if mode == True: print('math_op.get_target_dist: ', abs_target)
    return abs_target


# def get_min_lidar(n):
#     temp_arr = server.get_lidar_data(n)
#     lidar_data = []
#     for i in range(len(temp_arr)):
#         if i >= 170 and i <= 514:
#             lidar_data.append(temp_arr[i])

#     lidar_data_temp = []

#     for i in lidar_data:
#         if i != 0.0:
#             lidar_data_temp.append(i)

#     if len(lidar_data_temp) == 0:
#         lidar_data_temp.append(2)
#     print('get_min_lidar: ', lidar_data_temp)
#     return min(lidar_data_temp)

def get_move_dist(points, mode):
    dist = 0
    for i in range(len(points)-1):
        dist += get_target_dist(points[i], points[i+1], mode)
    if mode == True: print('math_op.get_move_dist: ', dist)
    return dist
