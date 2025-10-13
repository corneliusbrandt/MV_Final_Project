uint8 STOP=0
uint8 FORWARD=1
uint8 LEFT=2
uint8 RIGHT=3

# Direction in which the robot should drive
# Must be one of: STOP, FORWARD, LEFT or RIGHT
uint8 direction

# Percentage of how much the gripper should open
# 0=fully closed, 100=fully opened
uint8 gripper_percentage

# Position of where to move the arm in 3D space
geometry_msgs/Point target_position
