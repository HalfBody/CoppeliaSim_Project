import math
import matplotlib.pyplot as plt


class mapping():
    def __init__(self):
        self.points = []

    def add_points(self, sim):
        robot_pos = sim.get_robot_position()
        robot_rot = sim.get_robot_rotation() * math.pi / 180
        lidar = sim.get_lidar_data()

        lidar_len = len(lidar)
        for i in range(len(lidar)):
            if  0.01 <= lidar[i] or lidar[i] <= 2.0:
                if i < lidar_len / 2:
                    angle = robot_rot - (70 * math.pi / 180 - i * ((140 * math.pi / 180) / lidar_len)) 
                else:
                    angle = robot_rot + (i * ((140 * math.pi / 180) / lidar_len) - (70 * math.pi / 180)) 

                angle = 2 * math.pi + robot_rot
                angle = -angle if angle >= math.pi else angle

                x = lidar[i] * math.sin(angle) + robot_pos[0]
                y = lidar[i] * -math.cos(angle) + robot_pos[1]

                self.points.append([x, y])


    def map_draw(self):
        print('Количество точек:', len(self.points))
        for i in self.points:
            plt.plot(i[0], i[1], ".k")
        plt.show()

        