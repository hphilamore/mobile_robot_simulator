from robot_simulator import *  # This line must be included at the START of your program
from math import atan2, pi

plt.ion()

x_now = 100
y_now = 50
theta_now = pi/2

x_sequence = [125, 150, 130, 50]
y_sequence = [100, 50, 175, 75]

for x,y in zip(x_sequence, y_sequence):

    delta_x = (x - x_now)
    delta_y = (y - y_now)
    print(delta_x, delta_y)

    distance = (delta_x ** 2 + delta_y ** 2) ** (1/2)
    print('distance = ',distance, round(distance,0))
    beta = atan2(delta_y, delta_x)
    print('beta', beta)
    print('beta_degrees', round(beta*180/pi,0))
    alpha = beta - theta_now
    print('alpha', alpha)
    print('alpha_degrees', round(alpha * 180 / pi,0))
    angle_degrees = int(round(alpha * 180 / pi,0))

    if angle_degrees >= 0:
        for i in range(abs(angle_degrees)):
            update_simulation(0, 1)
    else:
        for i in range(abs(angle_degrees)):
            update_simulation(0, -1)

    for i in range(int(distance)):
        update_simulation(1, 0)

    x_now = x
    y_now = y
    theta_now = beta
    print('theta', theta_now)
    print()

save_simulation_data()  # This line must be included at the END of your program