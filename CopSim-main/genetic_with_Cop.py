import server
import math
import math_op
import time
import fuzzy_logic_python
import map
import os
import random
import numpy as np


# запуск цикла управления 
def main_control_loop(turn_points, control, target, n, map, sim, mode):

    # получение угла поворота робота
    r_rotation = sim.get_robot_rotation()
    if mode == True: print('main_control_loop r_rotation: ', r_rotation) #

    # получение позиции робота
    r_position = sim.get_robot_position()
    if mode == True: print('main_control_loop r_position: ', r_position) # возвращает значение в радианах

    # получение угла целевого направления  
    target_angle = math_op.get_target_angle(r_position, target, r_rotation, mode) 
    if mode == True: print('main_control_loop target_angle: ', target_angle)

    # получение значений лидара
    lidar = sim.get_lidar_data(mode)
    lidar_cut = []
    for i in lidar:
        if i == 0.0:
            i = 2.0
        if i >= 2:
            i = 2.0
        lidar_cut.append(i)
    
    # признак детекции препятствия
    detect = False
    detect_min = min(lidar_cut)
    if mode == True: print(detect_min)
    if(detect_min <= 1.5): detect = True

    # выбор сратегии 
    # либо уворот от препятствия (препятствие есть),
    # либо движение к цели (препятствия нет)
    speed = 0.0
    angle = 0.0

    turn_points.append(r_position)
    map.add_points(r_position, r_rotation, lidar)

    if(detect == False):
        if mode == True: print('Езда к цели')
        speed = 2
        if(abs(target_angle) <= 0.5):
            angle = target_angle 
        else:
            angle = 0.5
    else:
        if mode == True: print('Уворот от препятствия')
        # функция запуска СУ
        fuz_control = control.fuz_log(lidar, target_angle, sim, mode)
        # получение уставки на угол от СУ
        angle = fuz_control.get('angle')
        # получение уставки на скорость от СУ
        speed = math.ceil(fuz_control.get('speed'))
    
    if mode == True: print ('Мгновенные значения управления и ориентации: ', angle, ' - угол СУ, ', speed, ' - скорость усл.ед')
    # при угле от СУ в некоторых погрешнотях робот движется прямо, 
    # если уставка на угол велика - поворот без движения

    # angle = 0
    # speed = 0

    if 0.01 > angle > -0.01:
        sim.move(speed, 0, mode)
        #sim.step_trigger()
    else:
        if angle > 0:
            angle = math.ceil(angle)
        if angle < 0:
            angle = math.floor(angle)

        sim.move(speed, angle, mode)
        #sim.step_trigger()
    
    if mode == True: print('\n')
    return(detect_min)
    # turn_points.append(sim.get_robot_position())
    # map.add_points(sim)

def experiment(LR_ARR, LRF_ARR, SIM):
    
    sim = SIM
    #sim.step_enable()
    sim.replace_robot(0, 0, 0.1)
          
    time.sleep(3)
        
    sim.start_sim()

    #LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
    #LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

    # LR_arr = [1.2078750550921701, 1.2590445005349755, 1.25500513225747, 1.3158277513464467, 1.5062737632906598, 1.4645018439598694, 1.5730323677720999, 1.629882798085442]
    # LRF_arr = [1.3900597868435844, 1.49282832926887, 1.5137960962878325, 1.6479956682329573, 1.8458763871153316, 1.861200935732206, 2.0262948675179078, 2.0446681673587523]
    LR_arr = LR_ARR
    LRF_arr = LRF_ARR

    control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)
        
    create_map = map.mapping()
    turn_points = []
    target_list = [[[2, -6]]]
    minaimal_data_lidar = 100
        
    i = [0, 0, 0]

    # number of robots
    n = 0

    start_time = time.time()
    #sim.step_trigger()

    count = 0

    target = target_list[n][i[n]]
        
    while(True):
            
        #target = target_list[n][i[n]]
            
        lid_min = main_control_loop(turn_points, control, target, n, create_map, sim, False)
        if lid_min < minaimal_data_lidar: minaimal_data_lidar = lid_min    
        #sim.step_trigger()

        dist = math_op.get_target_dist(sim.get_robot_position(), target, False)
            
        if dist < 0.1:
                
            i[n] += 1
            if i[n] == len(target_list[0]):
                sim.move(0, 0, False)
                turn_points.append(sim.get_robot_position())
                break

        count+=1

        
    dist = math_op.get_move_dist(turn_points, False)
    end_time = time.time() - start_time

    #create_map.map_draw()
    #server_req.print_to_console(f'Robot num: {n}\nGOAL ACHIEVED\nWork time: {end_time}\nTravel distance: {dist}')

    sim.stop_sim()
    return(dist, end_time, minaimal_data_lidar)

def usefull_func(res):
    out = (res[2] * 1000) / (math.pow(res[0], 2) + math.pow(res[1], 2))
    return out


if __name__ == "__main__":
    
    sim = server.sim()

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    
    # подготовка сцены в симуляторе
    #sim.step_enable()
    sim.load_scene(f'{ROOT_DIR}\Scenes\Initial.ttt')
    sim.load_model(f'{ROOT_DIR}\Models\KUKA YouBot_2.ttm')
    #sim.replace_robot(0, 0, 0.1)

    #sim.start_sim()

    LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
    LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

    LR = [LR_arr]
    LRF = [LRF_arr]

    for i in range(2):
        LR_temp = []
        LRF_temp = []

        for i in range(len(LR_arr)):
            if i == 0:
                rnd_1 = random.uniform(-0.05, 0.05)
                rnd_2 = random.uniform(-0.05, 0.05)
                LR_temp.append(LR_arr[0] + rnd_1)
                LRF_temp.append(LRF_arr[0] + rnd_2)
            elif i == 2:
                rnd_1 = random.uniform(-0.05, 0.05)
                rnd_2 = random.uniform(-0.05, 0.05)
                LR_temp.append(LR_temp[i-1] + rnd_1)
                LRF_temp.append(LRF_temp[i-1] + rnd_2)
            elif i == 5:
                rnd_1 = random.uniform(-0.05, 0.05)
                rnd_2 = random.uniform(-0.05, 0.05)
                LR_temp.append(LR_temp[i-1] + rnd_1)
                LRF_temp.append(LRF_temp[i-1] + rnd_2)
            else:
                rnd_1 = random.uniform(0, 0.1)
                rnd_2 = random.uniform(0, 0.1)
                LR_temp.append(LR_temp[i-1] + rnd_1)
                LRF_temp.append(LRF_temp[i-1] + rnd_2)

        LR.append(LR_temp)
        LRF.append(LRF_temp)

    ############### GENETIC ALGORITHM START ###############

    for i in range(5):
        gen_result = []
        print(f'\n######### {i+1}/5 GEN #########\n')

        for j in range(4):
            print(f'{j+1}/2 robot')

            res = experiment(LR[j], LRF[j], sim)
            gen_result.append(res)

            print(f'Result:\nTime: {res[1]}\nDistance: {res[0]}\nMin lidar data: {res[2]}\n')
            time.sleep(3)
            #print(f'Result:\nTime: {res[1]}\nDistance: {res[0]}\n')

        result = np.zeros(len(gen_result))
        for i in range(len(gen_result)):
            result[i] = usefull_func(gen_result[i])

        temp_arr = []
        for i in result:
            temp_arr.append(i)
        temp_arr.sort()

        first = np.where(result == temp_arr[len(temp_arr) - 1])[0][0]
        second = np.where(result == temp_arr[len(temp_arr) - 2])[0][0]

        LR_first = LR[first]
        LRF_first = LRF[first]
        LR_second = LR[second]
        LRF_second = LRF[second]

        LR = [LR_first, LR_second]
        LRF = [LRF_first, LRF_second]

        ### CROSSENGOVER ###

        LR_temp_1 = []
        LR_temp_2 = []
        LRF_temp_1 = []
        LRF_temp_2 = []

        for i in range(len(LRF_first)):
            if i == 0:
                LR_temp_1.append(LR_second[i])
                LR_temp_2.append(LR_first[i])
                LRF_temp_1.append(LRF_second[i])
                LRF_temp_2.append(LRF_first[i])
            else:
                LR_temp_1.append(LR_first[i])
                LR_temp_2.append(LR_second[i])
                LRF_temp_1.append(LRF_first[i])
                LRF_temp_2.append(LRF_second[i])

        LR.append(LR_temp_1)
        LR.append(LR_temp_2)
        LRF.append(LRF_temp_1)
        LRF.append(LRF_temp_2)

        ### MUTATION ###

        for i in range(6):
            LR_temp = []
            LRF_temp = []

            for i in range(len(LR_arr)):
                if i == 0:
                    rnd_1 = random.uniform(-0.1, 0.1)
                    rnd_2 = random.uniform(-0.1, 0.1)
                    LR_temp.append(LR_first[0] + rnd_1)
                    LRF_temp.append(LRF_first[0] + rnd_2)
                elif i == 2:
                    rnd_1 = random.uniform(-0.1, 0.1)
                    rnd_2 = random.uniform(-0.1, 0.1)
                    LR_temp.append(LR_temp[i-1] + rnd_1)
                    LRF_temp.append(LRF_temp[i-1] + rnd_2)
                elif i == 5:
                    rnd_1 = random.uniform(-0.1, 0.1)
                    rnd_2 = random.uniform(-0.1, 0.1)
                    LR_temp.append(LR_temp[i-1] + rnd_1)
                    LRF_temp.append(LRF_temp[i-1] + rnd_2)
                else:
                    rnd_1 = random.uniform(0, 0.2)
                    rnd_2 = random.uniform(0, 0.2)
                    LR_temp.append(LR_temp[i-1] + rnd_1)
                    LRF_temp.append(LRF_temp[i-1] + rnd_2)

            LR.append(LR_temp)
            LRF.append(LRF_temp)

    print(f'LR best: {LR_first}')
    print(f'LRF best: {LRF_first}')

