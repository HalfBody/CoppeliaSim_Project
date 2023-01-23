import os
import math
import time

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

from breezyslam.algorithms import RMHC_SLAM
import breezyslam.sensors as sensors

from src import server
from src import math_op
from src import fuzzy_logic_python


MAP_SIZE_PIXELS = 200


# запуск цикла управления 
def control_loop(turn_points, control, target, lidar, r_rotation, r_position, sim):
    # получение угла целевого направления  
    target_angle = math_op.get_target_angle(r_position, target, r_rotation)
    
    # признак детекции препятствия
    detect = True if min(lidar) < 1.5 else False

    # выбор сратегии 
    # либо уворот от препятствия (препятствие есть),
    # либо движение к цели (препятствия нет)
    speed = 0.0
    angle = 0.0
    if detect == False:
        speed = 2
        if abs(target_angle) <= 0.5:
            angle = target_angle 
        else:
            angle = target_angle / abs(target_angle)
    else:
        # функция запуска СУ
        fuz_control = control.fuz_log(lidar, target_angle)
        # получение уставки на угол от СУ
        angle = fuz_control.get('angle')
        # получение уставки на скорость от СУ
        speed = math.ceil(fuz_control.get('speed'))
    
    # при угле от СУ в некоторых погрешнотях робот движется прямо, 
    # если уставка на угол велика - поворот без движения
    if abs(angle) < 0.0001:
        sim.move(speed, 0)
    else:
        angle = math.ceil(angle) if angle > 0.0 else math.floor(angle)
        sim.move(speed, angle * 2)
    
    turn_points.append(r_position)


def main():
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

    # подготовка сцены в симуляторе
    sim = server.sim()

    sim.load_scene(f'{ROOT_DIR}/Scenes/Initial.ttt')
    sim.load_model(f'{ROOT_DIR}/Models/KUKA YouBot_2.ttm')
    sim.replace_robot(0, 0, 0.1)

    # количество роботов
    n = 0
    i = [0, 0, 0]
    target_list = [[[0, -11]]]

    LR_arr = [0.6, 0.7, 0.6, 0.7, 1.0, 1.1, 1.0, 1.2]
    LRF_arr = [0.5, 0.6, 0.55, 0.6, 1.0, 1.1, 1.0, 1.2]

    # инициализация модели SLAM
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    lidar_model = sensors.Laser(684, 1, 140, 2000)
    slam = RMHC_SLAM(lidar_model, MAP_SIZE_PIXELS, 35)

    turn_points = []
    control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)
    
    start_time = time.time()
    curr_time = start_time
    sim.start_sim()

    figure, ax = plt.subplots(figsize=(4, 4))
    plt.title("SLAM map", fontsize=20)
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    figure.show()

    r_pos_prev = sim.get_robot_position()
    r_rot_prev = sim.get_robot_rotation()
    while(True):
        delta_time = time.time() - curr_time
        curr_time = time.time()
        target = target_list[n][i[n]]

        # обновление карты SLAM
        scan = sim.get_lidar_data()
        lidar = [
            laser if laser != 0.0 and laser <= 1.5 else 1.5
            for laser in scan
        ]

        scan = [laser * 1000 for laser in reversed(scan)]
        r_pos_curr = sim.get_robot_position()
        r_rot_curr = sim.get_robot_rotation()
        
        delta_pos = [
            math.sqrt((r_pos_curr[0] - r_pos_prev[0]) ** 2 + (r_pos_curr[1] - r_pos_prev[1]) ** 2),
            (r_rot_curr - r_rot_prev) * 180.0 / math.pi,
            delta_time
        ]

        r_pos_prev = r_pos_curr
        r_rot_prev = r_rot_curr
        
        slam.update(scan, delta_pos)

        control_loop(turn_points, control, target, lidar, r_rot_curr, r_pos_curr, sim)

        dist = math_op.get_target_dist(r_pos_curr, target)
        if dist < 0.2:
            i[n] += 1
            if i[n] == len(target_list[0]):
                sim.move(0, 0)
                turn_points.append(r_pos_curr)
                break
        
        print(f'DELTA: {delta_pos}')
        print(f'POSITION: [{r_pos_curr[0]}, [{r_pos_curr[1]}, r_rot_curr]')
        print('*'*80, end='\n\n')
        
        # динамическое отображение карты SLAM
        slam.getmap(mapbytes)
        ax.imshow(np.array(mapbytes).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
        figure.canvas.draw()
        figure.canvas.flush_events()

    dist = math_op.get_move_dist(turn_points)
    end_time = time.time() - start_time

    sim.stop_sim()
    print(f'Time: {end_time}')

    slam.getmap(mapbytes)
    save_image(mapbytes)


def save_image(bytearr: bytearray):
    image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), bytearr, 'raw', 'L', 0, 1)
    image.save('map.png')

    print("Saved to map.png")


if __name__ == "__main__":
    main()
