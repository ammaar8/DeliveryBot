#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time
import sys

global ns

door = {
    "CLOSED": 0,
    "OPEN": 1,
}

floors = {
    0 : "lobby",
    1 : "first",
    2 : "second",
}

try:
    ns = sys.argv[1]
except:
    ns = ""    
rospy.loginfo("Namespace := " + ns)

floor_door_left_pubs = []
floor_door_right_pubs = []

for i in range(len(floors.keys())):
    floor_door_left_pubs.append(
        rospy.Publisher(
        ns + '/' + str(floors[i]) + '_door_left_controller/command',
        Float64,
        queue_size=10)
    )

    floor_door_right_pubs.append(rospy.Publisher(
        ns + '/' + str(floors[i]) + '_door_right_controller/command',
        Float64,
        queue_size=10)
    )

# Publishers
floor_pub = rospy.Publisher(
    ns + '/floor_position_controller/command',
    Float64,
    queue_size=10
)

pub_car_left = rospy.Publisher(
    ns + '/car_door_left_controller/command',
    Float64,
    queue_size=10
)
    
pub_car_right = rospy.Publisher(
    ns + '/car_door_right_controller/command',
    Float64,
    queue_size=10
)



DOOR_OPEN = 0.5

FLOOR = 0
DOOR = door["CLOSED"]


def go_to_floor(floor):
    global FLOOR, DOOR

    FLOOR = int(floor)
    if DOOR != door["CLOSED"]:
        close_doors()

    floor_pub.publish(4.0 * floor)

def open_doors():
    global DOOR

    rospy.loginfo("OPENING DOORS" + " FLOOR " + str(FLOOR))

    
    floor_door_right_pubs[FLOOR].publish(DOOR_OPEN)
    floor_door_left_pubs[FLOOR].publish(-DOOR_OPEN)

    pub_car_right.publish(DOOR_OPEN)
    pub_car_left.publish(-DOOR_OPEN)

    DOOR = door["OPEN"]


def close_doors():
    global DOOR
    rospy.loginfo("CLOSING DOORS")

    
    floor_door_right_pubs[FLOOR].publish(0)
    floor_door_left_pubs[FLOOR].publish(0)

    pub_car_right.publish(0)
    pub_car_left.publish(0)
    
    DOOR = door["CLOSED"]


if __name__=="__main__":
    rospy.init_node('elevator_teleop')
    rate = rospy.Rate(10)
    
    while True:
        option = int(input("1. Go to floor\n2. Open Doors\n3. Close Doors\nOption: "))
        if(option == 1):
            floor_no = int(input("Floor Number: "))
            go_to_floor(floor_no)
        elif(option == 2):
            open_doors()
        elif(option == 3):
            close_doors()
        else:
            pass
        
        rate.sleep()

        

