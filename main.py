from robot_simulator import *  # Starts simulator

for i in range(50):
    update_simulation(1, 0)  

for i in range(90):
    update_simulation(0, 1)

save_simulation_data()        # Outputs simulation as video and text file