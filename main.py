# TODO: If collision, block movement correctly (i.e. if obstacle on left, can't turn left, if obstacle in front can't go forward)

# TODO: Exercises
# TODO: FOOD: If food found (stall = True) break out of loop, use random to choose direction. Treat wall as obstacle
# TODO: OBSTACLE: If obstacle encountered, including wall, devise strategy to move around/away from it
# TODO: Obstacle: If obstacle encountered, rotate until it's to left of robot, move forward, if it goes out of range,
#  do something else

from robot_simulator import *  # This line must be included at the START of your program
from math import atan2, pi

plt.ion()

x = 100
y = 50
theta = pi/2

x_sequence = [125, 150]#, 130, 50]
y_sequence = [100, 50]#, 175, 75]

for x_new,y_new in zip(x_sequence, y_sequence):

    delta_x = (x_new - x)
    delta_y = (y_new - y)
    distance = (delta_x ** 2 + delta_y ** 2) ** (1/2)

    beta = atan2(delta_y, delta_x)
    # print('beta', beta)
    # print('beta_degrees', round(beta*180/pi,0))
    alpha = beta - theta
    # print('alpha', alpha)
    # print('alpha_degrees', round(alpha * 180 / pi,0))
    angle_degrees = int(round(alpha * 180 / pi,0))

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
    # print('theta', theta)
    # print()

save_simulation_data()  # This line must be included at the END of your program