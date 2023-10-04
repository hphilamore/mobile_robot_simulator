
import numpy as np
import matplotlib.pyplot as plt
from obstacles_c import *
from obstacles_c import Obstacle_c
from obstacles_c import Controller_c
import matplotlib.animation as animation

# %matplotlib inline

# To produce our animated simulation output
# from matplotlib import rc
# rc('animation', html='jshtml')

# Constants which define the simulator
# We do not need to change these.
numframes = 50
arena_width = 200
obstacles = False  # switches obstacles on or off
num_obstacles = 1
num_sensors = 8


#------------------------------------------------------------
# set up figure and animation
fig = plt.figure(dpi=120)
#fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(0, arena_width), ylim=(0, arena_width))


# fig, ax = plt.subplots()
# ax.set_xlim(0,1)
# ax.set_ylim(0,1)


# An instance of our simulated Robot!
# Placed in the centre of the arena.
# my_robot = Robot_c(arena_width/2,
#                    arena_width/2,
#                    np.random.random()*np.pi*2)
# Placed in the corner of the arena.
my_robot = Robot_c(arena_width/10,
                   arena_width/10,
                   np.pi/2)



gui_robot, = ax.plot([], [], 'bo', ms=my_robot.radius*2)
gui_robot.set_data([], [])

gui_dir, = ax.plot([], [], 'r-', c="black")
gui_sensor = ax.plot(*[[],[]]*num_sensors,'r-', c="red")
gui_obstacles, = ax.plot([],[],'bo', ms=24, c="orange")
# x = 100#np.linspace(0, 10*np.pi, 100)
# y = 100#np.sin(x)
# line1, = ax.plot(x, y, 'bo')

if obstacles:
  # A list of obstacles within the space
  obstacles = []
  obstacles_xy = []
  for i in range( num_obstacles ):
    obstacles.append( Obstacle_c( arena_width, 12, i, num_obstacles) )
    obstacles_xy.append( [obstacles[i].x, obstacles[i].y] )

  obstacles_xy = np.asarray( obstacles_xy, dtype=float)
  gui_obstacles.set_data( obstacles_xy[:,0], obstacles_xy[:,1]  )

# An instance of our controller!
my_controller = Controller_c()

for i in range(numframes):
    print(i, end='\t')
    vl, vr = my_controller.update( my_robot )

    # # Update robot position, check for collision,
    # # then update sensors.9+
    my_robot.updatePosition(vl, vr)
    print(round(my_robot.x,3), round(my_robot.y,3))

    if obstacles:
      for obstacle in obstacles:
        my_robot.collisionCheck( obstacle )
        my_robot.updateSensors( obstacle )

    # Draw the robot, change colour for collision
    gui_robot.set_data(my_robot.x, my_robot.y)

    # Draw a little indicator so we can see which
    # way the robot is facing
    tx = my_robot.x + (my_robot.radius*1.4*np.cos(my_robot.theta))
    ty = my_robot.y + (my_robot.radius*1.4*np.sin(my_robot.theta))
    gui_dir.set_data( (my_robot.x,tx), (my_robot.y, ty) )

    # Output
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.savefig(f'str{i}.png')

    # plt.savefig(f'{str(i)}.jpeg')


    # if my_robot.stall == 1:
    #   gui_robot.set_color("red")
    # else:
    #   gui_robot.set_color("blue")

    # # Draw a little indicator so we can see which
    # # way the robot is facing
    # tx = my_robot.x + (my_robot.radius*1.4*np.cos(my_robot.theta))
    # ty = my_robot.y + (my_robot.radius*1.4*np.sin(my_robot.theta))
    # gui_dir.set_data( (my_robot.x,tx), (my_robot.y, ty) )


    # # Draw the sensor beams
    # for i in range(8):
    #   prox_sensor = my_robot.prox_sensors[i]
    #   ox = prox_sensor.x
    #   oy = prox_sensor.y
    #   if prox_sensor.reading > 0:
    #     tx = prox_sensor.x + prox_sensor.reading * np.cos( prox_sensor.theta)
    #     ty = prox_sensor.y + prox_sensor.reading * np.sin( prox_sensor.theta)
    #   else:
    #     tx = prox_sensor.x + np.cos( prox_sensor.theta)
    #     ty = prox_sensor.y + np.sin( prox_sensor.theta)

    #   gui_sensor[i].set_data( (ox,tx), (oy, ty) )

    # # Update the current score in the title!
    # my_robot.updateScore()
    # ax.set_title('Score: {0:f}'.format( my_robot.score ))
    # fig.canvas.draw()
    # plt.pause(0.1)
    # plt.show()
    # plt.savefig(f'{str(i)}.jpeg')

#     return gui_robot,

# plt.close()
# ani = animation.FuncAnimation(fig, animate, frames=numframes, interval=10, blit=True)
# ani

#plt.show()