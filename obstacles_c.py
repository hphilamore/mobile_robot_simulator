from math import *
import numpy as np

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


  # def updatePosition( self, vl, vr ):
  #
  #   if vl > 1.0:
  #     vl = 1.0
  #   if vl < -1.0:
  #     vl = -1.0
  #   if vr > 1.0:
  #     vr = 1.0
  #   if vr < -1.0:
  #     vr = -1.0
  #
  #   # save requested wheel speed for later.
  #   self.vl = vl
  #   self.vr = vr
  #
  #   # clear stall flag, attempt move
  #   self.stall = -1
  #
  #   # robot matrix, contributions to motion x,y,theta
  #   r_matrix = [(vl/2)+(vr/2),
  #               0,
  #               (vr-vl)/self.wheel_sep]
  #
  #   # kinematic matrix
  #   k_matrix = [
  #               [ np.cos(self.theta),-np.sin(self.theta),0],
  #               [ np.sin(self.theta), np.cos(self.theta),0],
  #               [0,0,1]
  #              ]
  #
  #   result_matrix = np.matmul(k_matrix, r_matrix)
  #
  #   self.x += result_matrix[0]
  #   self.y += result_matrix[1]
  #   self.theta -= result_matrix[2]
  #
  #   # Once we have updated the robots new global position
  #   # we should also update the position of its sensor(s)
  #   for prox_sensor in self.prox_sensors:
  #     prox_sensor.updateGlobalPosition( self.x, self.y, self.theta )

  def updatePosition( self, v, w ):

    # if vl > 1.0:
    #   vl = 1.0
    # if vl < -1.0:
    #   vl = -1.0
    # if vr > 1.0:
    #   vr = 1.0
    # if vr < -1.0:
    #   vr = -1.0

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


#
# Create controller code within def update(self, robot):
#
# The logical controller has been copied here for
# you to make a start.
#
# Make sure you return vl and vr at the end of
# update to move your simulated robot.
#
class Controller_c:

  # This function is called again and again
  # by the simulator to decide what the robot
  # should do.
  # Remember to follow a general rule of:
  # 1) sense (read sensors)
  # 2) plan (decide something)
  # 3) act (set motor speeds)
  def update( self, robot ):

    # We will use vl and vr (velocity-left
    # and velocity-right) to store motor
    # speeds to later send to the motors
    # Motor velocity should be in the range
    # [ -1.0 : +1.0 ]
    v = 1 #0.2
    w = 0 #0.2

    # # Read sensor 0 and store the result
    # # Note, use robot.prox_sensors[ n ].reading
    # # where n is in the range [ 0 : 7 ] for
    # # the 8 sensors around the body.
    # sensor0 = robot.prox_sensors[0].reading
    # #sensor1 = robot.prox_sensors[1].reading
    # #sensor2 = robot.prox_sensors[2].reading
    # # etc...

    # # Make a decision for motor speeds based
    # # on sensor0.  Note, sensors always return
    # # a value between [0:20], or -1 if they have
    # # not detected something.
    # if sensor0 >= 0:
    #   if sensor0 < 10:

    #     # We set the left velocity negative,
    #     # which would mean backwards
    #     vl = -0.3

    #     # We set the right velocity positive,
    #     # which would mean forwards.
    #     vr = 0.3

    #   elif sensor0 < 15:
    #     # We set the left velocity negative,
    #     # which would mean backwards
    #     vl = -0.1

    #     # We set the right velocity positive,
    #     # which would mean forwards.
    #     vr = 0.1



    # This controller should always return
    # vl, vr
    return v, w

