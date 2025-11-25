import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jsingh/turtlebot3_ws/src/driver_package/install/driver_package'
