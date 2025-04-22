import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/incsl-wego22/ws_capstone/src/front_tracking_node/install/front_tracking_node'
