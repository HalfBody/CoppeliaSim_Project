import math
import matplotlib.pyplot as plt


class mapping():
    def __init__(self):
        self.points = []

    def add_points(self, sim):
        robot_pos = sim.get_robot_position()
        robot_rot = sim.get_robot_rotation() * math.pi / 180
        lidar_data = sim.get_lidar_data()

        for i in range(len(lidar_data)):
            if lidar_data[i] == 0 or lidar_data[i] > 3:
                continue
            else:
                if i <= 342:
                    angle = robot_rot - (135 * math.pi / 180 - i * ((270 * math.pi / 180) / 684)) 
                else:
                    angle = robot_rot + (i * ((270 * math.pi / 180) / 684) - (135 * math.pi / 180)) 

                if angle >= math.pi:
                    angle = -2 * math.pi + robot_rot
                if angle < -math.pi:
                    angle = 2 * math.pi + robot_rot

                x = lidar_data[i] * math.sin(angle) + robot_pos[0]
                y = lidar_data[i] * -math.cos(angle) + robot_pos[1]

                self.points.append([x, y])


    def map_draw(self):
        print(len(self.points))
        for i in self.points:
            plt.plot(i[0], i[1], ".k")
        plt.show()


