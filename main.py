from robot_simulator import *

# The square has 4 sides
sides = 3

side_length = 50

# Calculate external angle
external_angle = int(360 / sides)

# Turn
for k in range(external_angle-90):
    update_simulation(0, 1)  # v = 0 and w = 1

# Use of nested loops
for i in range(sides):

    # Move straight
    for j in range(side_length):
        update_simulation(1, 0)  # v = 1 and w = 0

    # Turn
    for k in range(external_angle):
        update_simulation(0, 1)  # v = 0 and w = 1

save_simulation_data() 