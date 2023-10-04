from robot_simulator import *

# We do not need to change these.
numframes = 50

#------------------------------------------------------------
# enable GUI
plt.ion()

for i in range(numframes):
    # print(i, end='\t')
    v, w = 1, 0 #my_controller.update( my_robot )

    # # Update robot position, check for collision,
    # # then update sensors.9+
    update_simulation(i, v, w)

    # print(round(robot.x,3), round(robot.y,3))

output_simulation_video()

# TODO: Move to my_robot.update simulator
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





