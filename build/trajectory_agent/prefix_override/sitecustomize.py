import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patricio/cpr_ws/sim_ws/install/trajectory_agent'
