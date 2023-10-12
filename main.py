from robot_simulator import *  # This line must be included at the START of your program
from math import atan, pi

# Adds markers to the simulation
add_markers([50, 116, 63], 
            [75, 60, 154])

# Updates the simulation (moves the robot)
update_simulation(0, 0)

print(robot.x, robot.y)

save_simulation_data()        # Outputs simulation as video and text file