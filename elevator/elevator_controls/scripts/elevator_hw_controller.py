#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys
from std_srvs.srv import Empty, EmptyResponse
from elevator_controls.srv import ElevatorFloorGoal, ElevatorFloorGoalResponse
global ns 
ns = rospy.get_namespace()

door = {
    "CLOSED": 0,
    "OPEN": 1,
}

floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

floor_door_left_pubs = []
floor_door_right_pubs = []

for i in range(len(floors.keys())):
    floor_door_left_pubs.append(
        rospy.Publisher(
        rospy.names.ns_join(ns, str(floors[i]) + '_door_left_controller/command'),
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        rospy.names.ns_join(ns, str(floors[i]) + '_door_right_controller/command'),
        Float64,
        queue_size=10)
    )

# Publishers
car_pos_pub = rospy.Publisher(
    rospy.names.ns_join(ns, 'floor_position_controller/command'),
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    rospy.names.ns_join(ns, 'car_door_left_controller/command'),
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    rospy.names.ns_join(ns, 'car_door_right_controller/command'),
    Float64,
    queue_size=10
)


DOOR_OPEN = 0.5
FLOOR = 0
DOOR = door["CLOSED"]

def go_to_floor(req):
    global FLOOR, DOOR
    rospy.loginfo("Going to " + str(req.floor) + " floor")
    FLOOR = int(req.floor)
    if DOOR != door["CLOSED"]:
        close_doors(req)
    car_pos_pub.publish(4.0 * req.floor)
    rospy.Rate(1).sleep()
    return ElevatorFloorGoalResponse()

def open_doors(req):
    global DOOR
    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))    
    floor_door_right_pubs[FLOOR].publish(DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-DOOR_OPEN)
    pub_car_right.publish(DOOR_OPEN)
    pub_car_left.publish(-DOOR_OPEN)
    DOOR = door["OPEN"]
    rospy.Rate(1).sleep()
    return EmptyResponse()

def close_doors(req):
    global DOOR
    rospy.loginfo("CLOSING DOORS")
    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)
    pub_car_right.publish(0)
    pub_car_left.publish(0)
    DOOR = door["CLOSED"]
    rospy.Rate(1).sleep()
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('elevator_controller')
    rospy.Service('open_elevator_doors', Empty, open_doors)
    rospy.Service('close_elevator_doors', Empty, close_doors)
    rospy.Service('elevator_goto_floor', ElevatorFloorGoal, go_to_floor)
    rospy.spin()