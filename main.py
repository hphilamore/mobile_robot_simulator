

from robot_simulator import *  # This line must be included at the START of your program
from math import atan2, pi


x = 100
y = 50
theta = pi / 2

x_sequence = [125, 150, 130, 50]
y_sequence = [100, 50, 175, 75]

add_obstacles(x_sequence, y_sequence)

for x_new, y_new in zip(x_sequence, y_sequence):

    delta_x = (x_new - x)
    delta_y = (y_new - y)

    distance = (delta_x ** 2 + delta_y ** 2) ** (1 / 2)

    beta = atan2(delta_y, delta_x)
    alpha = beta - theta
    angle_degrees = int(round(alpha * 180 / pi, 0))

    if angle_degrees >= 0:
        for i in range(abs(angle_degrees)):
            update_simulation(0, 1)
    else:
        for i in range(abs(angle_degrees)):
            update_simulation(0, -1)

    for i in range(int(distance)):
        update_simulation(1, 0)

    x = x_new
    y = y_new
    theta = beta
    print('theta', theta)
    print()

save_simulation_data()  # This line must be included at the END of your program