import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/trg/Turtlebot3_ppo_demo/install/turtlebot3_ppo'
