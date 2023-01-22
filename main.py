import os
import math
import time

from src import server

from src import map
from src import math_op
from src import fuzzy_logic_python


# запуск цикла управления 
def control_loop(turn_points, control, target, n, map, sim):
    # получение угла поворота робота
    r_rotation = sim.get_robot_rotation()
    print('main_control_loop r_rotation: ', r_rotation)

    # получение позиции робота
    r_position = sim.get_robot_position()
    print('main_control_loop r_position: ', r_position)

    # получение угла целевого направления  
    target_angle = math_op.get_target_angle(r_position, target, r_rotation) 
    print('main_control_loop target_angle: ', target_angle)

    # получение значений лидара
    lidar = sim.get_lidar_inv()
    
    # признак детекции препятствия
    detect = True if min(lidar) < 1.5 else False

    # выбор сратегии 
    # либо уворот от препятствия (препятствие есть),
    # либо движение к цели (препятствия нет)
    speed = 0.0
    angle = 0.0
    if detect == False:
        print('Езда к цели')
        speed = 2
        if abs(target_angle) <= 0.5:
            angle = target_angle 
        else:
            angle = target_angle / abs(target_angle)
    else:
        print('Уворот от препятствия')
        # функция запуска СУ
        fuz_control = control.fuz_log(lidar, target_angle)
        # получение уставки на угол от СУ
        angle = fuz_control.get('angle')
        # получение уставки на скорость от СУ
        speed = math.ceil(fuz_control.get('speed'))

    print ('Мгновенные значения управления и ориентации: ', angle, ' - угол СУ, ', speed, ' - скорость усл.ед')
    # при угле от СУ в некоторых погрешнотях робот движется прямо, 
    # если уставка на угол велика - поворот без движения
    if abs(angle) < 0.0001:
        sim.move(speed, 0)
    else:
        angle = math.ceil(angle) if angle > 0.0 else math.floor(angle)
        sim.move(speed, angle * 2)
    
    print('\n')
    turn_points.append(sim.get_robot_position())
    map.add_points(sim)


def main():
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

    # подготовка сцены в симуляторе
    sim = server.sim()

    sim.load_scene(f'{ROOT_DIR}/Scenes/Initial.ttt')
    sim.load_model(f'{ROOT_DIR}/Models/KUKA YouBot_2.ttm')
    sim.replace_robot(0, 0, 0.1)

    # number of robots
    n = 0
    i = [0, 0, 0]
    target_list = [[[0, -11]]]

    #LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
    #LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

    LR_arr = [
        1.2078750550921701, 1.2590445005349755, 1.25500513225747, 1.3158277513464467,
        1.5062737632906598, 1.4645018439598694, 1.5730323677720999, 1.629882798085442
        ]
    LRF_arr = [
        1.3900597868435844, 1.49282832926887, 1.5137960962878325, 1.6479956682329573,
        1.8458763871153316, 1.861200935732206, 2.0262948675179078, 2.0446681673587523
        ]

    loops = 1
    for _ in range(loops):
        start_time = time.time()
        sim.start_sim()

        turn_points = []
        
        control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)
        create_map = map.mapping()
        
        while(True):
            target = target_list[n][i[n]]

            control_loop(turn_points, control, target, n, create_map, sim)

            dist = math_op.get_target_dist(sim.get_robot_position(), target)
            if dist < 0.1:
                i[n] += 1
                if i[n] == len(target_list[0]):
                    sim.move(0, 0)
                    turn_points.append(sim.get_robot_position())
                    break


        dist = math_op.get_move_dist(turn_points)
        end_time = time.time() - start_time

        create_map.map_draw()

        sim.stop_sim()


if __name__ == "__main__":
    main()
