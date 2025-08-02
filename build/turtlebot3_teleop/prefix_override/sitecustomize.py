import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ishaan/turtlebot_sim/install/turtlebot3_teleop'
