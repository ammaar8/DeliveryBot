#!/usr/bin/env python3

import rospy
import subprocess
import signal
import roslaunch
from std_srvs.srv import Empty
from deliverybot_navigation.srv import MapFilePath, MapFilePathResponse

map_server = None

# def set_map(req):
#     global map_server
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     if map_server is not None:
#         map_server.shutdown()
#     cli_args = [
#         "deliverybot_navigation",
#         "navigation_map_server.launch",
#         "map_path:={}".format(req.map_file_path)
#         ]
#     map_server_launch = roslaunch.rlutil.resolve_launch_arguments(cli_args)
#     map_server_args = cli_args[2:]
#     map_server = roslaunch.parent.ROSLaunchParent(uuid, [(map_server_launch[0], map_server_args)])
#     map_server.start()
#     map_server.spin()
#     return True

def set_map(req):
    global map_server
    if map_server is not None:
        map_server.send_signal(signal.SIGINT)
    map_server = subprocess.Popen(["rosrun", "map_server", "map_server", req.map_file_path])
    return True
  
    

if __name__ == "__main__":
    rospy.init_node("navigation_map_server")
    rospy.Service('set_map', MapFilePath, set_map)
    rospy.spin()