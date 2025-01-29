import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tayya/four_wheel_ws/install/zinger_swerve_controller'
