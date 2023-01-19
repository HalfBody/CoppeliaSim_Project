import server_req


def fuzzy_log(eng, lidar, target_angle):

    a = eng.fuzzy_log(lidar[0], lidar[1], lidar[2], lidar[3], lidar[4], target_angle)
    #server_req.print_to_console(f'Fuzzy logic\nangle: {a[0][0]}\nvel: {a[0][1]}\n')
    return [a[0][0], a[0][1]]
