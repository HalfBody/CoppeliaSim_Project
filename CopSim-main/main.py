import server_req
import server

import math

import math_op

import time

import fuzzy_logic_python

import map

import os


# запуск цикла управления 
def main_control_loop(turn_points, control, target, n, map, sim):
    # получение угла поворота робота
    rotation = sim.get_robot_rotation()
    # описание модуля матс_оп
    # получение в цикле угла целевого направления    
    target_angle = math_op.get_target_angle(sim.get_robot_position(), target, rotation)
    # функция запуска СУ
    fuz_control = control.fuz_log([], target_angle, sim)
    # получение уставки на угол от СУ
    angle = fuz_control.get('angle')
    # получение уставки на скорость от СУ
    speed = math.ceil(fuz_control.get('speed'))


    print ('Мгновенные значения управления и ориентации: ', angle, ' - угол СУ, ', speed, ' - скорость усл.ед, ', rotation, target_angle)
    #server_req.print_to_console(f'Control loop:\nAngle: {angle}\nSpeed: {speed}\n')
    # при угле от СУ в некоторых погрешнотях робот движется прямо, если уставка на угол велика - поворот без движения
    if 0.00001 > angle > -0.00001:
        #end_time = time.time() + 1

        #while end_time - time.time() > 0.01:
        sim.move(speed, 0)
        #time.sleep(3)
        sim.step_trigger()
    else:
        #sim.turn(angle, 2)
        #end_time = time.time() + 1

        #while end_time - time.time() > 0.01:
        if angle > 0:
            angle = math.ceil(angle)
        if angle < 0:
            angle = math.floor(angle)

        sim.move(speed, angle*2)
        #time.sleep(3)
        sim.step_trigger()
    print('\n')
    # обновление списка точек поворота и карты
    turn_points.append(sim.get_robot_position())
    map.add_points(sim)



if __name__ == "__main__":
    sim = server.sim()

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    # подготовка сцены в симуляторе

    sim.step_enable()
    sim.load_scene(f'{ROOT_DIR}\Scenes\Initial.ttt')

    sim.load_model(f'{ROOT_DIR}\Models\KUKA YouBot_2.ttm')
    
    sim.replace_robot(0, 0, 0.1)

    for j in range(1):
        time.sleep(3)
        sim.start_sim()
        turn_points = []

        #LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
        #LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

        LR_arr = [1.2078750550921701, 1.2590445005349755, 1.25500513225747, 1.3158277513464467, 1.5062737632906598, 1.4645018439598694, 1.5730323677720999, 1.629882798085442]
        LRF_arr = [1.3900597868435844, 1.49282832926887, 1.5137960962878325, 1.6479956682329573, 1.8458763871153316, 1.861200935732206, 2.0262948675179078, 2.0446681673587523]
        control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)
        create_map = map.mapping()

        target_list = [[[0, -11]]]
        
        i = [0, 0, 0]

        # number of robots
        n = 0

        start_time = time.time()
        sim.step_trigger()

        while(True):
            target = target_list[n][i[n]]
            main_control_loop(turn_points, control, target, n, create_map, sim)
            sim.step_trigger()

            dist = math_op.get_target_dist(sim.get_robot_position(), target)
            if dist < 0.1:
                i[n] += 1
                if i[n] == len(target_list[0]):
                    sim.move(0, 0)
                    turn_points.append(sim.get_robot_position())
                    break


        dist = math_op.get_move_dist(turn_points)
        end_time = time.time() - start_time

        #create_map.map_draw()
        #server_req.print_to_console(f'Robot num: {n}\nGOAL ACHIEVED\nWork time: {end_time}\nTravel distance: {dist}')

        sim.stop_sim()
