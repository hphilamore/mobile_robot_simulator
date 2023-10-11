
from robot_simulator import *  # Starts simulator

for i in range(75):
    # Updates the simulation (moves the robot)
    # 1st input: Linear (forward) speed of robot
    # 2nd input: Angular (turning) speed of robot
    update_simulation(-1, 0)

save_simulation_data()  # Outputs simulation as video and text file