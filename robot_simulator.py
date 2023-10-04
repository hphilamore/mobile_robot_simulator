from math import *
import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
from os import listdir
from os.path import isfile, join

class Obstacle_c:

  # Assigns itself a random position within
  # the arena, keeping a set distance from
  # the centre
  def __init__(self, arena_size=200, radius=10, rot=0.0, max_obstacles=1):

    self.radius = radius

    # For defined placement
    rot_ang = rot * ((np.pi*2)/max_obstacles)
    rand_dist = np.random.uniform(.35, .65) * (arena_size/2)
    self.x = (arena_size/2) + rand_dist*np.cos(rot_ang)
    self.y = (arena_size/2) + rand_dist*np.sin(rot_ang)


# The model of the proximity sensor.
class ProxSensor_c:

  # current global xy position of sensor
  x = 0
  y = 0
  theta = 0

  # To store the last sensor reading
  reading = 0

  # To set sensor local position around the robot body.
  offset_dist = 0
  offset_angl = 0

  # maximum scan range:
  max_range = 20

  # constructor. by default, sensor sits 10 distance forward
  # and faces 0 radians with respect to robot origin (0,0).
  def __init__(self, offset_dist=5, offset_angl=0):
    self.offset_dist = offset_dist
    self.offset_angl = offset_angl


  def updateGlobalPosition(self, robot_x, robot_y, robot_theta ):

    # Current direction of the sensor is the rotation
    # of the robot body plus the fixed rotation of the
    # sensor around that same body.
    self.theta = self.offset_angl + robot_theta

    # With the rotation, we now work out where the
    # sensor sits in cartesian space (x,y) by projecting
    # out by offset_distance.
    # Note, we do this as if the sensor was at origin (0,0)
    sensor_x = (self.offset_dist*np.cos(self.theta))
    sensor_y = (self.offset_dist*np.sin(self.theta))

    # commit new position to memory, translating to the
    # robots current x,y position.
    self.x = sensor_x + robot_x
    self.y = sensor_y + robot_y

    # If we've reset position, the last sensor reading
    # is now wrong.
    self.reading = -1

  def scanFor( self, obstruction ):

    # See if the obstruction is within the detection
    # range of the sensor.
    distance = np.sqrt( ((obstruction.x - self.x)**2) + ((obstruction.y - self.y)**2) )
    distance = distance - obstruction.radius

    # if out of range, return error
    # note: real sensors aren't this easy.
    if distance > self.max_range:
      return

    # compute this sensors angle toward the obstruction
    # (e.g. where is the object relative to the sensor?)
    a2o = atan2( ( obstruction.y - self.y), (obstruction.x-self.x ))

    # computer the smallest angle between the line of
    # sight of the sensor, and the current angle to the
    # obstruction.
    # [insert url here]
    angle_between = atan2( sin(self.theta-a2o),  cos(self.theta-a2o) )
    angle_between = abs( angle_between )

    # If the detection is outside of the field of view
    # of the sensor, then we return and do nothing else.
    # This will either leave the reading as -1 (invalid)
    # for the next call.  Or leave the reading as the
    # last valid reading (>-1) it had.
    if angle_between > np.pi/8:
      return

    # If the current reading is -1 then that means
    # this is the first valid reading, and we update
    # the sensor.
    if self.reading < 0:
      self.reading = distance

    # If the sensor already has a valid reading (>-1)
    # then we only store the new reading if it is closer.
    # (closer obstructions block the field of view)
    if self.reading > 0:
      if distance < self.reading:
        self.reading = distance



# The model of the robot.
class Robot_c:

  # We could do something like, manually add 2 sensors
  #prox_sensors.append( ProxSensor_c(2, np.pi/8) )
  #prox_sensors.append( ProxSensor_c(2, -np.pi/8) )

  def __init__(self, x=50,y=50,theta=np.pi):
    self.x = x
    self.y = y
    self.theta = theta
    self.stall = -1 # to check for collisions
    self.score = 0
    self.radius = 5 # 5cm radius
    self.wheel_sep = self.radius*2 # wheel on either side
    self.v = 0
    self.w = 0
    self.arena_width = 200

    # This is the body plan of sensors from
    # an e-puck robot! (in radians)
    self.sensor_dirs = [5.986479,
                        5.410521,
                        4.712389,
                        3.665191,
                        2.617994,
                        1.570796,
                        0.8726646,
                        0.296706,
                        ]

    self.prox_sensors = [] #= ProxSensor_c()
    for i in range(0,8):
      self.prox_sensors.append( ProxSensor_c(self.radius, self.sensor_dirs[i]) )

  def update_simulator(self, i):

    # Setup plot
    fig = plt.figure(dpi=120)
    ax = fig.add_subplot(111, aspect='equal',
                         autoscale_on=False,
                         xlim=(0, self.arena_width),
                         ylim=(0, self.arena_width))

    # Initialise plotted robot
    gui_robot, = ax.plot([], [], 'bo', ms=self.radius * 2)
    gui_robot.set_data([], [])
    gui_dir, = ax.plot([], [], 'r-', c="black")
    # gui_sensor = ax.plot(*[[],[]]*num_sensors,'r-', c="red")
    gui_obstacles, = ax.plot([], [], 'bo', ms=24, c="orange")

    # Draw robot at position x, y
    gui_robot.set_data(self.x, self.y)

    # Draw little indicator to show which direction robot is facing
    tx = self.x + (self.radius * 1.4 * np.cos(self.theta))
    ty = self.y + (self.radius * 1.4 * np.sin(self.theta))
    gui_dir.set_data((self.x, tx), (self.y, ty))


    print(f'time = {i}      x = {round(self.x, 3)}      y = {round(self.y, 3)}')
    plt.title(f'time = {i}         x = {round(self.x, 3)}         y = {round(self.y, 3)}')

    # Output
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.savefig("img/{:02d}_{}".format(i, '.png')) # Use 02d modifierfor two-digit numbers (zero-padded)
    # plt.savefig(f'img/{i}.png')


  def updatePosition( self, i, v, w ):

    # v can only be 0 or 1
    if v not in [1, -1]:
      v = 0

    # v can only be -1, 1 or 0
    if w not in [1, -1]:
      w = 0
    else:
      # convert w to 1 degree in rads
      w = pi/180

    # save requested wheel speed for later.
    self.v = v
    self.w = w

    # clear stall flag, attempt move
    self.stall = -1

    # robot matrix, contributions to motion x,y,theta
    r_matrix = [v, 0, w]

    # kinematic matrix
    k_matrix = [
                [ np.cos(self.theta),-np.sin(self.theta),0],
                [ np.sin(self.theta), np.cos(self.theta),0],
                [0,0,1]
               ]

    result_matrix = np.matmul(k_matrix, r_matrix)

    self.x += result_matrix[0]
    self.y += result_matrix[1]
    self.theta -= result_matrix[2]

    # Once we have updated the robots new global position
    # we should also update the position of its sensor(s)
    for prox_sensor in self.prox_sensors:
      prox_sensor.updateGlobalPosition( self.x, self.y, self.theta )

    self.update_simulator(i)


  # The sensor checks if it is in range to an obstruction,
  # and if yes, calculates the simulated proximity reading.
  # if no, determines and error result.
  def updateSensors(self, obstruction ):

    # for each sensor
    # for each obstruction
    for prox_sensor in self.prox_sensors:
      prox_sensor.scanFor( obstruction )

  def collisionCheck(self, obstruction ):
    distance = np.sqrt( ((obstruction.x - self.x)**2) + ((obstruction.y - self.y)**2) )
    distance -= self.radius
    distance -= obstruction.radius
    if distance < 0:
      self.stall = 1
      angle = atan2( obstruction.y - self.y, obstruction.x - self.x)
      self.x += distance * np.cos(angle)
      self.y += distance * np.sin(angle)

  def updateScore(self):

    # 1 minus the difference between wheel speed
    # to encourage straight line travel.
    # square root rewards small differences
    diff = np.abs(((self.vl+1) - (self.vr + 1))) * 0.5

    if diff > 0.0:
      diff =  1 - np.sqrt( diff )
    else:
      diff = 1 # - 0


    # Reward motor activation / penalise no movement
    vel = (np.abs(self.vl) + np.abs(self.vr))/2

    new_score = vel * diff

    if self.stall == 1:
      new_score -= 3

    self.score += new_score

  def make_video(self):

    path_out = 'simulation_video.mp4'
    path_in = 'img/'
    fps = 10
    files = [f for f in listdir(path_in) if isfile(join(path_in, f)) and f.endswith(".png")]
    files.sort()
    frame_array = []

    for i in range(len(files)):
      filename = path_in + files[i]
      # Read each file
      img = cv2.imread(filename)
      height, width, layers = img.shape
      size = (width, height)

      # Insert the frames into an image array
      frame_array.append(img)

    # Generate video file
    out = cv2.VideoWriter(path_out, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)

    for i in range(len(frame_array)):
      # Write each frame to the video
      out.write(frame_array[i])

    # Publish video
    out.release()

# An instance of the simulated Robot at initial position
x_init = 20
y_init = 20
theta_init = np.pi/2
robot = Robot_c(x_init, y_init, theta_init)
# my_robot = Robot_c(arena_width/10,
#                    arena_width/10,
#                    np.pi/2)

# Rename the Robot method for updating position
update_simulation = robot.updatePosition

output_simulation_video = robot.make_video