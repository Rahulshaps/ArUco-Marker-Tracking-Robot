import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/group4/Documents/EECE5554-FP/navphy_ws/install/target_tracking'
