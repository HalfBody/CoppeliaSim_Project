import requests
import math


def prepare_sim():
    requests.post('http://127.0.0.1:5000/start-v-rep-server?ip=127.0.0.1&port=19997')
    requests.post('http://127.0.0.1:5000/load-scene?scene=Initial.ttt')

    requests.post('http://127.0.0.1:5000/add-robot?model=KUKA YouBot.ttm')
    requests.post('http://127.0.0.1:5000/replace?robot-id=0&x=0&y=0&z=0.1')


def start_sim():
    requests.post('http://127.0.0.1:5000/start-simulation')


def stop_sim():
    requests.post('http://127.0.0.1:5000/stop-simulation')


def get_lidar_data(n):
    lidar_data = requests.get(f'http://127.0.0.1:5000/get-lidar/{n}').text

    temp_arr = []
    i = 0
    while i < len(lidar_data):
        if lidar_data[i] == '[':
            i += 1
            continue

        if lidar_data[i]  == ']':
            break

        temp = ''
        while True:
            temp += lidar_data[i]
            i += 1

            if lidar_data[i] == ']':
                break

            if lidar_data[i] == ',':
                i += 2
                break

        temp_arr.append(float(temp))

    return temp_arr


def movement(speed, n):
    if speed >= 0:
        requests.post(f'http://127.0.0.1:5000/backward?robot-id={n}&speed={speed}')
    else:
        requests.post(f'http://127.0.0.1:5000/forward?robot-id={n}&speed={-speed}')


def move_dist(dist, speed, n):
    requests.post(f'http://127.0.0.1:5000/move-distance?robot-id={n}&distance={str(dist)}&speed={speed}')


def turn(angle, speed, n):
    if angle >= 0:
        requests.post(f'http://127.0.0.1:5000/turn-angle?robot-id={n}&angle={angle}&speed={speed}')
    else:
        requests.post(f'http://127.0.0.1:5000/turn-angle?robot-id={n}&angle={-angle}&speed={-speed}')


def stop(n):
    requests.post(f'http://127.0.0.1:5000/stop?robot-id={n}')


def get_position(n):
    position = requests.get(f'http://127.0.0.1:5000/get-robot-position/{n}').text

    temp_arr = []
    i = 0
    while i < len(position):
        if position[i] == '[':
            i += 1
            continue

        if position[i]  == ']':
            break

        temp = ''
        while True:
            temp += position[i]
            i += 1

            if position[i] == ']':
                break

            if position[i] == ',':
                i += 2
                break

        temp_arr.append(float(temp))

    return temp_arr


def get_rotation(n):
    rotation = requests.get(f'http://127.0.0.1:5000/get-robot-rotation/{n}').text
    temp_arr = []
    i = 0
    while i < len(rotation):
        if rotation[i] == '[':
            i += 1
            continue

        if rotation[i]  == ']':
            break

        temp = ''
        while True:
            temp += rotation[i]
            i += 1

            if rotation[i] == ']':
                break

            if rotation[i] == ',':
                i += 2
                break

        temp_arr.append(float(temp))

    angle = temp_arr[2]*180/3.14

    if angle >= 0:
        angle = 180 - angle
    if angle < 0:
        angle = 180 - angle

    return temp_arr[2]

def print_to_console(text):
    requests.post(f'http://127.0.0.1:5000/print-message?text={text}')


def get_robot_speed(n):
    speed = requests.get(f'http://127.0.0.1:5000/get-robot-speed/{n}').text

    temp_arr = []
    i = 0
    while i < len(speed):
        if speed[i] == '[':
            i += 1
            continue

        if speed[i]  == ']':
            break

        temp = ''
        while True:
            temp += speed[i]
            i += 1

            if speed[i] == ']':
                break

            if speed[i] == ',':
                i += 2
                break

        temp_arr.append(float(temp))

    actual_speed = math.sqrt(math.pow(temp_arr[0], 2) + math.pow(temp_arr[1], 2))

    return actual_speed*20
    