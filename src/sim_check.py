import time
import math

import matplotlib.pyplot as plt
import numpy as np

from src import fuzzy_logic_python
from src import math_op_sim


class Obstacle():
    def __init__(self, x, y):
        self.a = [x - 1, y - 1]
        self.b = [x + 1, y - 1]
        self.c = [x + 1, y + 1]
        self.d = [x - 1, y + 1]

    def get_sides(self):
        sides = np.array([[self.a, self.b], [self.b, self.c], [self.c, self.d], [self.a, self.d]])
        return sides


class Robot():
    def __init__(self, x, y):
        self.dt = 0.05
        self.position = [x, y]
        self.direction = 0

    def lidar(self):
        lidar = []
        r = 1.5
        rot = (self.direction + 90) * math.pi / 180

        for i in range(180):
            angle = i * math.pi / 180
            lidar.append([[self.position[0], self.position[1]], [self.position[0] + r * -math.sin(angle + rot), self.position[1] + r * -math.cos(angle + rot)]])

        return lidar

    def turn(self, turn_angle):
        self.direction += turn_angle

        if self.direction >= 180:
            self.direction = -360 + self.direction
        if self.direction < -180:
            self.direction = 360 + self.direction

    def movement(self, speed):
        rot = self.direction * math.pi / 180
        vx = speed * math.sin(rot)
        vy = speed * math.cos(rot)

        self.position[0] += vx * self.dt
        self.position[1] += vy * self.dt


class Sim():
    def __init__(self, LR_arr, LRF_arr):
        self.control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)

        obstacle_1 = Obstacle(-0.9, 3)
        obstacle_2 = Obstacle(0.9, 8)

        self.obstacle = [obstacle_1, obstacle_2]


    def get_lidar_data(self, lidar, obstacle, Robot):
        temp_arr = []
        for i in range(len(lidar)):
            temp_arr.append([])

        for i in obstacle:
            index = 0
            sides = i.get_sides()
            for j in lidar:
                for k in range(len(sides)):
                    side = sides[k]

                    A1 = j[0][1] - j[1][1]
                    B1 = j[1][0] - j[0][0]
                    C1 = j[0][0] * j[1][1] - j[1][0] * j[0][1]

                    A2 = side[0][1] - side[1][1]
                    B2 = side[1][0] - side[0][0]
                    C2 = side[0][0] * side[1][1] - side[1][0] * side[0][1]

                    if B1*A2 - B2*A1 and A1:
                        y = (C2*A1 - C1*A2) / (B1*A2 - B2*A1)
                        x = (-C1 - B1*y) / A1

                        if (min(j[0][0], j[1][0]) - 0.001 <= x <= max(j[0][0], j[1][0]) + 0.001 and min(j[0][1], j[1][1]) - 0.001 <= y <= max(j[0][1], j[1][1]) + 0.001)\
                            and (min(side[0][0], side[1][0]) - 0.001 <= x <= max(side[0][0], side[1][0]) + 0.001 and min(side[0][1], side[1][1]) - 0.001 <= y <= max(side[0][1], side[1][1]) + 0.001):
                            temp_arr[index].append(math_op_sim.get_target_dist(Robot.position, [x, y]))
                        else:
                            temp_arr[index].append(1.5)
                            
                    elif B1*A2 - B2*A1 and A2:
                        y = (C2*A1 - C1*A2) / (B1*A2 - B2*A1)
                        x = (-C2 - B2*y) / A2

                        if (min(j[0][0], j[1][0]) - 0.001 <= x <= max(j[0][0], j[1][0]) + 0.001 and min(j[0][1], j[1][1]) - 0.001 <= y <= max(j[0][1], j[1][1]) + 0.001)\
                            and (min(side[0][0], side[1][0]) - 0.001 <= x <= max(side[0][0], side[1][0]) + 0.001 and min(side[0][1], side[1][1]) - 0.001 <= y <= max(side[0][1], side[1][1]) + 0.001):
                            temp_arr[index].append(math_op_sim.get_target_dist(Robot.position, [x, y]))
                        else:
                            temp_arr[index].append(1.5)
                    else:
                        temp_arr[index].append(1.5)
                index += 1

        lidar_temp = []
        for i in range(len(lidar)):
            lidar_temp.append(min(temp_arr[i]))
        
        r = []
        rf = []
        lf = []
        l = []
        f = []

        for i in range(len(lidar)):
            if i < 36:
                l.append(lidar_temp[i])
            elif 36 <= i < 72:
                lf.append(lidar_temp[i])
            elif 72 <= i < 108:
                f.append(lidar_temp[i])
            elif 108 <= i < 144:
                rf.append(lidar_temp[i])
            else:
                r.append(lidar_temp[i])

        ret = [min(l), min(lf), min(f), min(rf), min(r)]
        return ret


    def start_sim(self):
        for i in self.obstacle:
            for j in i.get_sides():
                plt.plot([j[0][0], j[1][0]], [j[0][1], j[1][1]], 'r')

        robot = Robot(0, 0)
        target = [0, 11]

        points = [[0, 0]]
        start_time = time.time()
        min_lidar = []

        while True:
            turn_angle = math_op_sim.get_target_angle(robot.position, target, robot.direction)
            lidar_data = self.get_lidar_data(robot.lidar(), self.obstacle, robot)

            for i in lidar_data:
                min_lidar.append(i)
                
            fuz_control = self.control.fuz_log(lidar_data, turn_angle, 0)
            angle = fuz_control.get('angle')
            speed = fuz_control.get('speed')

            robot.turn(angle)
            robot.movement(speed)
            points.append([robot.position[0], robot.position[1]])

            plt.plot(robot.position[0], robot.position[1], ".k")
            plt.axis([-5.5, 5.5, 0, 11])
            plt.pause(0.001)

            if math_op_sim.get_target_dist(robot.position, target) < 0.1:
                break
        
        dist = math_op_sim.get_move_dist(points)
        work_time = len(points) * robot.dt
        min_lidar_data = min(min_lidar)

        return [work_time, dist, min_lidar_data]


if __name__ == '__main__':
    #LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
    #LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

    LR_arr = [1.1378750550921701, 1.2590445005349755, 1.25500513225747, 1.3158277513464467, 1.5062737632906598, 1.4645018439598694, 1.5730323677720999, 1.629882798085442]
    LRF_arr = [1.3900597868435844, 1.49282832926887, 1.5137960962878325, 1.6479956682329573, 1.8458763871153316, 1.861200935732206, 1.8562948638711533, 1.9646681673587523]

    sim = Sim(LR_arr, LRF_arr)
    res = sim.start_sim()
    print(f'Result:\nTime: {res[0]}\nDistance: {res[1]}\nMin lidar data: {res[2]}\n')

    a = input()
    