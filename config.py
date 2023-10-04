import matplotlib.pyplot as plt
import numpy as np

arena_width = 200
robot_radius = 5

def update_simulator(i, x, y, theta):
    # Setup plot
    fig = plt.figure(dpi=120)
    ax = fig.add_subplot(111, aspect='equal',
                         autoscale_on=False,
                         xlim=(0, arena_width),
                         ylim=(0, arena_width))

    # Initialise plotted robot
    # gui_robot, = ax.plot([], [], 'bo', ms=robot_radius * 2)
    gui_robot.set_data([], [])
    gui_dir, = ax.plot([], [], 'r-', c="black")
    # gui_sensor = ax.plot(*[[],[]]*num_sensors,'r-', c="red")
    gui_obstacles, = ax.plot([], [], 'bo', ms=24, c="orange")

    # Draw robot at position x, y
    gui_robot.set_data(x, y)

    # Draw little indicator to show which direction robot is facing
    tx = x + (robot_radius * 1.4 * np.cos(theta))
    ty = y + (robot_radius * 1.4 * np.sin(theta))
    gui_dir.set_data((x, tx), (y, ty))

    # Output
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.savefig(f'str{i}.png')