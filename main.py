
import numpy as np
import matplotlib.pyplot as plt
from obstacles_c import *
import config

# Constants which define the simulator
# We do not need to change these.
numframes = 50
arena_width = 200
obstacles = False  # switches obstacles on or off
num_obstacles = 1
num_sensors = 8


#------------------------------------------------------------
# SETUP PLOTTING
# enable GUI
plt.ion()


# An instance of our simulated Robot
my_robot = Robot_c(arena_width/10,
                   arena_width/10,
                   np.pi/2)

# TODO: Move to config
# if obstacles:
#   # A list of obstacles within the space
#   obstacles = []
#   obstacles_xy = []
#   for i in range( num_obstacles ):
#     obstacles.append( Obstacle_c( arena_width, 12, i, num_obstacles) )
#     obstacles_xy.append( [obstacles[i].x, obstacles[i].y] )
#
#   obstacles_xy = np.asarray( obstacles_xy, dtype=float)
#   # Plot obstacles
#   gui_obstacles.set_data( obstacles_xy[:,0], obstacles_xy[:,1]  )

# An instance of our controller!
# my_controller = Controller_c()

for i in range(numframes):
    print(i, end='\t')
    v, w = 1, 0 #my_controller.update( my_robot )

    # # Update robot position, check for collision,
    # # then update sensors.9+
    my_robot.updatePosition(i, v, w)
    print(round(my_robot.x,3), round(my_robot.y,3))

    # if obstacles:
    #   for obstacle in obstacles:
    #     my_robot.collisionCheck( obstacle )
    #     my_robot.updateSensors( obstacle )

    # config.update_simulator(i, my_robot.x, my_robot.y, my_robot.theta)
